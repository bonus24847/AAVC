"""Orchestrator state — the single source of truth for the main loop.

Combines: mission phase, command pointer, time budget, latest telemetry,
mode (online/offline), and observed anomalies. Built as a plain dataclass
(not Pydantic) because it's mutated continuously and Pydantic validation
on every tick is overhead we don't need here.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.schemas import (
    MissionCommand,
    MissionPhase,
    MissionPlan,
)

from .flight_clock import FlightClock


class OrchestratorMode(str, Enum):
    OFFLINE = "offline"  # Execute the pre-baked plan with the offline tactical rules


class TerminalState(str, Enum):
    RUNNING = "running"
    COMPLETED = "completed"     # Mission finished normally
    LANDED_RTH = "landed_rth"   # RTH triggered, landed safely
    ABORTED = "aborted"          # Safety abort
    FAILED = "failed"            # Connection lost, crash, etc.


@dataclass
class OrchestratorState:
    """Mutable state shared across the orchestrator's async tasks."""

    mode: OrchestratorMode
    plan: MissionPlan
    telemetry: CurrentTelemetry
    phase: MissionPhase = MissionPhase.TAKEOFF
    command_pointer: int = 0                  # Index into plan.commands
    # Origin of the operation-window clock, on the FLIGHT clock below (NOT on
    # time.monotonic — the name predates flight_clock and is kept only to
    # avoid churning every caller). See start_window().
    mission_start_monotonic: float = 0.0
    operation_window_s: float = 1200.0        # 20-min AAVC operation window
    anomalies: list[str] = field(default_factory=list)
    # ONLY record_anomaly entries. `anomalies` above doubles as the FULL audit
    # trail (TELEM samples, SORTIE/TRANSIT events, operator commands) — the GCS
    # anomaly feed must not stream that (2026-07-17: the red ANOMALY banner
    # showed routine 1 Hz TELEM lines and drowned real anomalies).
    anomaly_log: list[str] = field(default_factory=list)
    terminal: TerminalState = TerminalState.RUNNING
    # ── V1.3 multi-sortie delivery ──
    sortie_index: int = 0                     # 1-based current sortie; 0 = pre-mission
    max_sorties: int = 4                      # ceiling on FLIGHTS (arm→disarm cycles),
                                               # NOT pads/deliveries — see below
    # ── FLIGHT ⊃ DELIVERY (2026-07-24 briefing) ──
    # A FLIGHT is one arm→disarm cycle; sortie_index above IS the flight
    # counter and max_sorties above IS the number of flights. eggs_aboard eggs
    # are carried per flight; the flight serves flight_ids in order.
    eggs_aboard: int = 1
    max_deliveries: int = 4                   # pads to serve in the window (≤ placed)
    delivery_index: int = 0                   # 1-based delivery across the mission
    flight_ids: list[int] = field(default_factory=list, compare=False, repr=False)
    assigned_marker_id: int | None = None     # THIS sortie's committee-assigned pad id
    # Ordered committee assignments for sorties 1..N — the 4-of-6 mission queue.
    # ONE mechanism for both operator flows: headless --assigned-ids / config
    # seeds it at startup; the GCS sets it via POST /api/cmd/mission_ids. The
    # per-sortie gate reads queue[sortie-1]; a manual id in the GO request
    # overrides the queue for that sortie only.
    assigned_id_queue: list[int] = field(default_factory=list, compare=False, repr=False)
    # Recomputed by the per-sortie gate loop: does the remaining window cover
    # another full sortie + reserve? Read by /api/cmd/preflight/go, which 409s
    # without `force` when False (the operator may still force a late sortie —
    # the per-minute overtime penalty is their call, not the software's).
    sortie_time_ok: bool = field(default=True, compare=False, repr=False)
    # ── energy budget (orchestrator/energy_policy.py) ──
    energy_capacity_mah: float = field(default=0.0, compare=False, repr=False)
    # Consumption reading at the start of the CURRENT pack. A battery swap
    # rebases it so per-sortie costs stay comparable across packs.
    energy_baseline_mah: float = field(default=0.0, compare=False, repr=False)
    sortie_energy_mah: list[float] = field(default_factory=list, compare=False, repr=False)
    # Gate hint, mirroring sortie_time_ok: the GO endpoint refuses unless forced.
    sortie_energy_ok: bool = field(default=True, compare=False, repr=False)
    energy_tier: str = field(default="none", compare=False, repr=False)
    energy_detail: str = field(default="", compare=False, repr=False)
    energy_sorties_left: float = field(default=float("nan"), compare=False, repr=False)
    # The pack as the LAST sortie left it. A swap can only happen between
    # sorties (the resupply hold), so detection compares these with the next
    # sortie's entry readings — a window the aircraft is not flying through.
    energy_exit_mah: float = field(default=float("nan"), compare=False, repr=False)
    energy_exit_pct: float = field(default=float("nan"), compare=False, repr=False)
    # Did the FC read back the flight-envelope pins (ceiling-legal RTL altitude,
    # validated pad-descent speed, no auto-disarm on the pad)? Applying params is
    # best-effort by design, so this is the check that the best effort worked.
    # Third member of the same family as sortie_time_ok / sortie_energy_ok: an
    # advisory row on the card, a refusal at the GO endpoint, forceable.
    param_pins_ok: bool = field(default=True, compare=False, repr=False)
    param_pins_detail: str = field(default="", compare=False, repr=False)
    # One-shot latch for start_window(): the 20-min clock starts at the FIRST
    # operator GO (when the committee's countdown is running), not at process
    # start — the orchestrator may boot minutes before the slot begins.
    window_started: bool = field(default=False, compare=False, repr=False)
    # True while a MAVLink heartbeat is being received. The dashboard
    # launcher flips this off when MAVSDK connect() fails / times out
    # so the UI can render an explicit "NO LINK" banner instead of
    # showing stale gauges. The orchestrator's main loop also flips
    # it on after DroneCommander.connect() resolves.
    link_connected: bool = False
    # Stop indices already dropped, tracked per delivery target. Tactical Rule 6
    # DROP_NOW and the legacy single-payload SAR drop use stop 0; a multi-tree
    # delivery mission uses stop 1..N so each tree drops EXACTLY once. The
    # `payload_dropped` property below preserves the old single-flag API.
    dropped_stops: set[int] = field(default_factory=set, compare=False, repr=False)
    # Marker ids actually DELIVERED (touchdown + release confirmed by
    # mission.py::_serve) across the WHOLE mission, in delivery order. The
    # per-flight gate's recovery-flight chunking
    # (orchestrator/main.py::_chunk_for) subtracts this from
    # assigned_id_queue — via mission_brain.flights.remaining_owed — to work
    # out what a flight PAST the queue's positional chunks still owes.
    # Tracked explicitly rather than inferred from dropped_stops/stop_index:
    # a manual per-flight GO override can serve an id that was never queued
    # at all, and stop_index is a flat ordinal across the mission, not a
    # queue position — neither maps back to "which queue id got delivered"
    # unambiguously (I5, review 2026-07-24).
    delivered_marker_ids: list[int] = field(
        default_factory=list, compare=False, repr=False)
    # Guards the two concurrent drop paths — the tactical DROP_NOW and the
    # planned DROP_PAYLOAD (both in the mission loop) — so a single payload is
    # released at most once: both claim payload_dropped under
    # this lock before awaiting the drop. asyncio.Lock binds to the running
    # loop lazily on 3.12, so constructing state off-loop (tests) is fine.
    drop_lock: asyncio.Lock = field(default_factory=asyncio.Lock, compare=False, repr=False)
    # Pre-flight GO gate (orchestrator.preflight). Before takeoff the mission holds
    # in MissionPhase.PREFLIGHT until the operator authorizes launch (the dashboard
    # POSTs /api/cmd/preflight/go → sets the Event); headless auto-proceeds on green.
    # `preflight_can_go` is recomputed each tick by the gate loop (= all critical
    # checks pass) and read by the /go endpoint so it only fires on a green board;
    # `preflight_report` is the latest report dict (served by GET /api/cmd/preflight).
    awaiting_preflight_go: bool = field(default=False, compare=False, repr=False)
    preflight_can_go: bool = field(default=False, compare=False, repr=False)
    preflight_report: dict[str, Any] | None = field(default=None, compare=False, repr=False)
    preflight_resume_event: asyncio.Event = field(
        default_factory=asyncio.Event, compare=False, repr=False)
    # Anomaly kinds already recorded — record_anomaly dedupes by KIND (not by
    # the timestamped message) so a condition persisting across ticks is logged
    # once, not once per tick.
    _seen_anomaly_kinds: set[str] = field(default_factory=set, compare=False, repr=False)
    # C3: optional sink teed from record_anomaly / record_audit to persist the
    # audit trail to disk (runs/<mission_id>/audit.jsonl) so a crash doesn't lose
    # it. Injected by orchestrator.main; None = in-memory only (tests).
    audit_sink: Callable[[str], None] | None = field(default=None, compare=False, repr=False)
    # The aircraft's own clock (orchestrator/flight_clock.py), fed from
    # telemetry.vehicle_time_s. Every mission deadline is measured on it.
    flight_clock: FlightClock = field(default_factory=FlightClock,
                                      compare=False, repr=False)

    def now(self) -> float:
        """Mission time in the AIRCRAFT's time base (orchestrator.flight_clock).

        THE clock for anything that expresses "how much flying is left": the
        operation window, the TimePolicy reserves, every leg and rung timeout.
        On hardware it is wall time; in SITL it is simulated time, which is
        the only time the aircraft actually moves in.

        Host-side liveness — camera frame age, telemetry staleness, the energy
        sampler's cadence — deliberately keeps using time.monotonic(): those
        ask "is this process still being fed?", a question about the host.
        """
        self.flight_clock.feed(self.telemetry.vehicle_time_s)
        return self.flight_clock.now()

    def start_window(self) -> None:
        """Start the operation-window clock (idempotent). Called exactly once,
        when the sortie-1 gate releases — every elapsed/remaining read before
        that sees a full window."""
        if not self.window_started:
            self.window_started = True
            self.mission_start_monotonic = self.now()

    def time_elapsed_s(self) -> float:
        return self.now() - self.mission_start_monotonic

    def time_remaining_s(self) -> float:
        return max(0.0, self.operation_window_s - self.time_elapsed_s())

    def current_command(self) -> MissionCommand | None:
        if 0 <= self.command_pointer < len(self.plan.commands):
            return self.plan.commands[self.command_pointer]
        return None

    def advance(self) -> None:
        self.command_pointer += 1

    @property
    def payload_dropped(self) -> bool:
        """Back-compat: True once ANY payload has dropped. The tactical DROP_NOW
        and legacy single-drop paths read/write this; it maps onto stop 0 of
        `dropped_stops` so multi-tree delivery (stops 1..N) tracks per target."""
        return bool(self.dropped_stops)

    @payload_dropped.setter
    def payload_dropped(self, value: bool) -> None:
        if value:
            self.dropped_stops.add(0)
        else:
            self.dropped_stops.discard(0)

    def set_terminal(
        self, terminal: TerminalState, phase: MissionPhase | None = None
    ) -> None:
        """Transition the terminal state (and optionally the mission phase) as
        one operation. asyncio is single-threaded so the pair-write is atomic
        (no await between); this is the single documented place where terminal
        and phase change together. A late mission-progress callback must not
        resurrect a nav phase after this — see executor.progress_cb."""
        if phase is not None:
            self.phase = phase
        self.terminal = terminal

    def remaining_commands(self) -> list[MissionCommand]:
        return self.plan.commands[self.command_pointer:]

    def _persist_audit(self, entry: str) -> None:
        """Append an audit/anomaly line to the in-memory list and tee it to the
        optional audit sink (C3: crash-safe runs/<id>/audit.jsonl). The sink is
        best-effort — a persistence error must never raise into the flight path."""
        self.anomalies.append(entry)
        if self.audit_sink is not None:
            try:
                self.audit_sink(entry)
            except Exception:  # noqa: BLE001 — audit persistence is non-fatal
                pass

    def record_anomaly(self, kind: str) -> None:
        # Dedupe by KIND, not by the timestamped message: a condition that
        # persists across watchdog/tactical ticks (NaN battery, tactical-loop
        # errors) must be recorded once, not once per tick. Stores the
        # timestamp of first occurrence.
        if kind in self._seen_anomaly_kinds:
            return
        self._seen_anomaly_kinds.add(kind)
        entry = f"t={self.time_elapsed_s():.1f}s {kind}"
        self.anomaly_log.append(entry)
        self._persist_audit(entry)

    def record_audit(self, entry: str) -> None:
        """Record a non-deduped audit EVENT (operator commands, wizard actions)
        already stamped by the caller — appended verbatim + teed to the sink.
        Unlike record_anomaly, repeated identical events are all kept."""
        self._persist_audit(entry)
