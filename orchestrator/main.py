"""AAVC competition orchestrator — entry point (rules V1.3).

Wires the lightweight flight stack and flies the multi-sortie egg-delivery
mission (pad positions unknown at takeoff — the committee assigns a marker id
per sortie; pads are discovered in flight and remembered across sorties):

    DroneCommander (MAVSDK) ── connect ── PX4 (SITL or real 6X)
        │
        ├── TelemetrySubscriber      → state.telemetry (synchronized snapshot)
        ├── SafetyWatchdog           → battery / GPS / geofence / no-fly /
        │                              ceiling / datalink / time
        ├── VisionWorker → TargetTracker → decode + confirm landing pads
        ├── dashboard (optional)     → live map + cameras + per-sortie GO gate
        └── run_delivery_mission     → per sortie: transit P1→P2→P3 @20 m →
                                        (sweep if pad unknown) → land ON the
                                        assigned pad → release the egg →
                                        egress transit → land at L&R → resupply

Offline + deterministic by design — no LLM, no network. Safe to run headless
(``--no-dashboard`` + ``--assigned-ids``) or with the GCS.

    python -m orchestrator.main --config sitl/aavc_config.yaml
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any

import yaml
from loguru import logger

from mavlink_adapter.commands import DEFAULT_PX4_TUNING, ConnectionConfig, DroneCommander
from mavlink_adapter.raw_subscriber import RawMavlinkSubscriber
from mavlink_adapter.telemetry import TelemetrySubscriber
from mission_brain.flights import budgeted_flights_for, chunk_flights, remaining_owed
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import load_profile
from mission_brain.schemas import CommandKind, Coordinate, MissionPhase
from mission_brain.search_pattern import build_search_pattern
from vision.detectors.aruco import VALID_MARKER_IDS
from vision.projection import configure_cameras

from . import audit, preflight
from .energy_policy import EnergyPolicy, energy_consumed_mah
from .frame_recorder import FrameRecorder
from .gcs_status import GcsMissionStatus
from .mission import FlightGate, run_delivery_mission
from .safety import SafetyWatchdog
from .state import OrchestratorMode, OrchestratorState, TerminalState
from .tactical_align import AlignParams
from .target_tracker import TargetState, TargetTracker
from .time_policy import TimePolicy
from .vision_worker import (
    DEFAULT_FRAME_MAX_AGE_S,
    DEFAULT_INTERVAL_S,
    VisionWorker,
)


def _load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        logger.warning(f"[main] config {path} not found — using defaults")
        return {}
    return yaml.safe_load(path.read_text()) or {}


def _parse_assigned_ids(raw: str | list[Any]) -> list[int]:
    """Parse + validate the committee-assigned marker ids (L2).

    Accepts a comma string (``--assigned-ids "3,1,4,6"``) or a list (config
    ``mission.assigned_marker_ids``). Every id must be a competition marker id
    (1..6); an out-of-range or non-integer id raises with a clear message rather
    than being silently accepted and then never decoded (burning window time)."""
    valid = sorted(VALID_MARKER_IDS)
    tokens = ([t.strip() for t in raw.split(",")] if isinstance(raw, str)
              else [str(x).strip() for x in raw])
    ids: list[int] = []
    for tok in tokens:
        if not tok:
            continue
        try:
            value = int(tok)
        except (TypeError, ValueError):
            raise ValueError(
                f"assigned marker id {tok!r} is not an integer; valid ids are {valid}"
            ) from None
        if value not in VALID_MARKER_IDS:
            raise ValueError(f"assigned marker id {value} outside the valid set {valid}")
        ids.append(value)
    return ids


def _build_time_policy(sc: dict[str, Any], profile: Any) -> TimePolicy:
    """Build the sortie time-budget policy from the `search:` config block (L1).

    Only keys actually present override the TimePolicy dataclass defaults — so
    the single source of fallback truth is the dataclass, not stale inline
    literals (the old inline 300/200 fallbacks disagreed with both the dataclass
    350/210 and the shipped config)."""
    keys = ("serve_cost_s", "sortie_cost_s", "known_sortie_cost_s", "rth_reserve_s")
    overrides = {k: float(sc[k]) for k in keys if k in sc}
    return TimePolicy(watchdog_floor_s=profile.min_time_remaining_s, **overrides)


def _build_energy_policy(cfg: dict[str, Any], reserve_frac: float,
                         eggs_aboard: int = 1) -> EnergyPolicy:
    """Build the FLIGHT energy-budget policy (the twin of the time policy).

    The reserve fraction comes from `failsafes.bat_low_thr`, not a key of its
    own, so a second number here could only ever drift out of agreement with it.
    ⚠ CORRECTED 2026-08-16: this used to justify the choice as "below that
    threshold the FC flies its own low-battery RTL". It does not — with
    COM_LOW_BAT_ACT=3 the FC only WARNS at BAT_LOW_THR and does not return
    until BAT_CRIT_THR (0.15). The reserve is still the right number to plan
    against (it is the first line anything reacts to at all, and the companion
    RTHs at 30% well above it), but it is a POLICY floor, not the edge of a
    failsafe — do not re-derive it from "where the FC acts".

    ``eggs_aboard`` comes from the resolved mission config for the same reason:
    the seed cost of a flight scales with the deliveries it carries, and a seed
    frozen at one delivery would let the pre-flight gate approve a four-egg
    flight the pack cannot finish.
    """
    bat = cfg.get("battery") or {}
    keys = ("capacity_mah", "seed_sortie_mah", "seed_delivery_mah", "margin_mah")
    overrides = {k: float(bat[k]) for k in keys if k in bat}
    return EnergyPolicy(reserve_frac=reserve_frac,
                        eggs_aboard=max(1, int(eggs_aboard)), **overrides)


def _build_connection(cc: dict[str, Any], connect_override: str | None) -> ConnectionConfig:
    """Build ConnectionConfig from the optional ``connection:`` config block.

    ``--connect`` (connect_override) wins for the MAVLink endpoint. Everything else —
    the drop servo channel / PWM band / fallback endpoint — can be tuned at the G5
    bench from config without editing source. Omitted keys keep the dataclass defaults.
    """
    kw: dict[str, Any] = {}
    if connect_override:
        kw["system_address"] = connect_override
    elif cc.get("system_address"):
        kw["system_address"] = str(cc["system_address"])
    for key in ("drop_servo_channel", "drop_servo_pwm_release",
                "drop_servo_pwm_hold", "drop_payload_count"):
        if key in cc:
            kw[key] = int(cc[key])
    # payload_id -> actuator-set index (== AUX pin) when the rack is not wired
    # in delivery order; ConnectionConfig validates range/uniqueness/coverage.
    if "drop_servo_channels" in cc:
        kw["drop_servo_channels"] = tuple(int(c) for c in cc["drop_servo_channels"])
    if "drop_fallback_endpoint" in cc:
        kw["drop_fallback_endpoint"] = str(cc["drop_fallback_endpoint"])
    for key in ("connect_timeout_s", "arming_timeout_s"):
        if key in cc:
            kw[key] = float(cc[key])
    return ConnectionConfig(**kw)


def _is_sitl_endpoint(system_address: str) -> bool:
    """HINT ONLY: does the endpoint *look* like a simulator?

    ⚠ This cannot decide the question and must never gate anything that would
    be unsafe on hardware. The premise it was written on ("SITL speaks UDP, the
    CM4 talks serial") is false in this repo's own default configuration:
    cm4/launch_flight.sh runs the REAL aircraft through a mavlink-router at
    `udpin://0.0.0.0:14540`, so the real bird looks exactly like SITL here.
    Ask the autopilot instead — `_detect_simulator`.

    Kept as the cheap pre-check for `sim_battery`, where a wrong answer costs
    only a param timeout, never safety.
    """
    return system_address.strip().lower().startswith(("udp", "tcp"))


async def _detect_simulator(commander: Any) -> bool | None:
    """Ask the AUTOPILOT whether it is a simulator. ``None`` = cannot tell.

    `SIM_GZ_EN` is compiled into px4_sitl only — a Pixhawk's firmware has no
    such parameter — so its presence is the honest test, and no endpoint,
    hostname or CLI flag can lie about it.

    The trap this deliberately avoids: "parameter absent" and "link is dead"
    look identical from a single failed read, and defaulting either way is
    unsafe (call hardware a sim and safety pins get disabled; call a sim
    hardware and nothing is testable). So a missing SIM_GZ_EN is only believed
    once `SYS_AUTOSTART` — which every build has — answers on the same link.
    If neither reads, the caller gets ``None`` and must refuse rather than
    guess.
    """
    try:
        await commander.get_param_int("SIM_GZ_EN")
        return True
    except Exception:                                    # noqa: BLE001
        pass
    try:
        await commander.get_param_int("SYS_AUTOSTART")    # proves the link works
        return False
    except Exception:                                    # noqa: BLE001
        return None


async def _wait_for_gps(state: OrchestratorState, timeout_s: float = 30.0) -> bool:
    """Block until a usable GPS fix arrives (lat/lon non-NaN, ≥2D fix)."""
    t0 = asyncio.get_running_loop().time()
    while asyncio.get_running_loop().time() - t0 < timeout_s:
        t = state.telemetry
        if not math.isnan(t.lat) and not math.isnan(t.lon) and t.gps_fix_type >= 2:
            return True
        await asyncio.sleep(0.5)
    return False


async def emergency_recover(state: OrchestratorState, commander: Any) -> None:
    """What to do when the mission loop dies with the aircraft possibly airborne.

    Normally: an orderly RTH, LAND as a last resort, commanded while the safety
    watchdog is still up — killing the process and leaving the aircraft flying
    is the one outcome worse than a failed mission.

    **Except after a pilot takeover, where the answer is to do nothing.** Seen
    live on the bench 2026-08-18 (props off, first real OFFBOARD test):
    ``safety.py`` logged "PILOT TAKEOVER (POSCTL) — orchestrator standing down,
    no further commands" at 16:40:16, and at 16:40:53 this path sent an RTL
    anyway — the loop had been sitting inside the 60 s takeoff-altitude wait and
    only surfaced afterwards. With no props it changed nothing. In the air it is
    the companion pulling the aircraft into AUTO.RTL out from under a pilot who
    took manual control precisely to stop it doing something, leaving them to
    fight the mode switch back. A stand-down that still issues commands is not a
    stand-down.

    So on takeover: no command, and PILOT_TAKEOVER stays the terminal state
    rather than being overwritten with FAILED — the aircraft is the pilot's, and
    the RC plus the FC's own failsafes are the net.
    """
    if state.terminal is TerminalState.PILOT_TAKEOVER:
        logger.warning("[main] mission loop ended after PILOT TAKEOVER — NOT "
                       "commanding RTH; the pilot is flying the aircraft")
        state.record_audit(
            f"t={state.time_elapsed_s():.1f}s PILOT TAKEOVER — emergency RTH "
            "suppressed, aircraft left to the pilot")
        return

    logger.error("[main] mission loop crashed — commanding emergency RTH")
    state.record_anomaly("mission_loop_exception")
    state.set_terminal(TerminalState.FAILED, MissionPhase.RTH)
    try:
        await commander.rth()
    except Exception:
        logger.exception("[main] emergency RTH failed — last-resort LAND")
        try:
            await commander.land()
        except Exception:
            logger.exception(
                "[main] LAND failed too — FC failsafes are the only net left")


async def _guard_mission(run: Callable[[], Awaitable[None]],
                         state: OrchestratorState, commander: Any) -> None:
    """Run the mission loop and, on ANY exit that could leave the aircraft
    airborne — a crash OR a cancel/Ctrl-C — bring it home before the caller's
    ``finally`` stops the watchdog.

    ``except Exception`` alone missed ``asyncio.CancelledError``, which is a
    ``BaseException`` on 3.12: a Ctrl-C or a task cancel skipped
    ``emergency_recover`` entirely and stopped the watchdog with the aircraft
    still flying, leaving only the FC's own failsafes. The cancel is re-raised
    so the process still exits; a recovery that itself fails on the way down is
    swallowed rather than masking the cancellation."""
    try:
        await run()
    except asyncio.CancelledError:
        logger.warning("[main] mission loop cancelled — emergency recover, then re-raise")
        try:
            await emergency_recover(state, commander)
        except BaseException:  # noqa: BLE001 — cleanup on the way down; never mask the cancel
            pass
        raise
    except Exception:
        logger.exception("[main] mission loop raised")
        await emergency_recover(state, commander)


def _evaluate_energy(state: OrchestratorState, energy_policy: EnergyPolicy) -> None:
    """Refresh the energy-budget hint the pre-flight card and the GO gate read.

    Kept beside the time-budget evaluation so the two stay symmetric: neither is
    a flight action, both only refuse to START work.
    """
    consumed, tier = energy_consumed_mah(state.telemetry, energy_policy.capacity_mah)
    rel = (consumed - state.energy_baseline_mah
           if not math.isnan(consumed) else consumed)
    state.energy_tier = tier
    state.sortie_energy_ok, state.energy_detail = energy_policy.can_start_sortie(
        rel, state.sortie_energy_mah)
    state.energy_sorties_left = energy_policy.sorties_remaining(
        rel, state.sortie_energy_mah)


def _sortie_gate_factory(
    state: OrchestratorState,
    *,
    dash: Any,
    home: Coordinate,
    geofence: list[tuple[float, ...]],
    cfg: dict[str, Any],
    profile: Any,
    policy: Any,
    tracker: TargetTracker,
    energy_policy: Any = None,
    skip_preflight: bool = False,
    headless_timeout_s: float = 120.0,
    commander: Any = None,
    rc_go: bool = False,
) -> FlightGate:
    """Build the per-FLIGHT gate (rules V1.3): before EVERY flight the mission
    holds in ``MissionPhase.PREFLIGHT`` until the operator GOs.

    The gate owns the CHUNKING: it hands the mission loop flight i's list of at
    most ``state.eggs_aboard`` committee-assigned ids —
    ``chunk_flights(state.assigned_id_queue, state.eggs_aboard)[i-1]`` — or None
    to end the mission. The loop just serves the list it is handed.

    Flight indices PAST those positional chunks are RECOVERY flights (I5,
    review 2026-07-24): ``_chunk_for`` then carries whatever the queue still
    OWES — ``state.assigned_id_queue`` minus ``state.delivered_marker_ids``
    (``mission_brain.flights.remaining_owed``) — so a flight that comes home
    with undelivered eggs (a pad never found, a per-delivery budget abort, a
    release-channel shortage) is automatically followed by one that carries
    exactly those, no operator re-queue needed. ``state.max_sorties`` is
    seeded (main's ``run()``) via ``budgeted_flights_for`` precisely so at
    least one such flight is always budgeted; a fully-served queue makes the
    owed set empty and ``_chunk_for`` returns None, ending the mission
    immediately rather than flying a pointless extra flight.

    With a dashboard: re-evaluate + push the readiness board at ~1 Hz and wait
    for ``/api/cmd/preflight/go`` — the endpoint resolves an assigned marker id
    (a manual pick, or the queue's slot for this hold) into
    ``state.assigned_marker_id`` (and enforces the time/energy policy unless
    forced) before setting the event. That single id is the documented
    per-flight MANUAL OVERRIDE and applies at ``eggs_aboard == 1`` (where a
    flight IS one delivery) or whenever this flight has no chunk to fly
    (``not chunk``) — the queue is empty, or already exhausted by an earlier
    flight; a multi-egg flight WITH a chunk always flies that chunk.
    Headless: take the same chunk from the SAME ``state.assigned_id_queue``
    (seeded from --assigned-ids) and auto-proceed when the criticals pass. The
    queue is re-read from state on every call so a mid-mission
    /api/cmd/mission_ids update applies at the next hold.
    """
    pf = cfg.get("preflight", {}) or {}
    kwargs: dict[str, Any] = dict(
        geofence=[list(v) for v in geofence],
        home_lat=home.lat,
        home_lon=home.lon,
        # The critical floor is the FC's own low-battery threshold — below it a
        # launch flies straight into the failsafe. Default to that rather than to
        # a second, independently-drifting number; the charge ABOVE it is the
        # energy budget's call (advisory row, forceable refusal).
        min_battery_pct=float(pf.get(
            "battery_pct",
            100.0 * (energy_policy.reserve_frac if energy_policy is not None
                     else 0.25))),
        min_gps_sats=int(pf.get("gps_sats", 6)),
        min_time_remaining_s=profile.min_time_remaining_s,
        camera_max_age_s=float(pf.get("camera_max_age_s", 5.0)),
    )
    broadcaster = getattr(dash, "broadcaster", None)
    can_push = broadcaster is not None and hasattr(broadcaster, "push_preflight")

    def _evaluate() -> bool:
        report = preflight.run_preflight(state, **kwargs)
        state.preflight_can_go = report.all_critical_pass
        state.preflight_report = report.to_dict()
        # The window clock only bites once started (first GO); before that a
        # sortie can always start. Once the registry holds confirmed pads the
        # likely next sortie is a direct (known-pad) one — hint with the
        # known-sortie rule so a fittable sortie isn't discouraged.
        if energy_policy is not None:
            _evaluate_energy(state, energy_policy)
        if not state.window_started:
            state.sortie_time_ok = True
        elif tracker.distinct_confirmed_ids():
            state.sortie_time_ok = policy.can_start_known_sortie(
                state.time_remaining_s())
        else:
            state.sortie_time_ok = policy.can_start_sortie(
                state.time_remaining_s())
        if broadcaster is not None and hasattr(broadcaster, "push_preflight"):
            try:
                broadcaster.push_preflight(state.preflight_report)
            except Exception:
                logger.exception("[preflight] push failed")
        return report.all_critical_pass

    def _chunk_for(flight: int) -> list[int] | None:
        """THIS flight's ≤eggs_aboard slice of the committee queue. Re-read
        from state on every call — the GCS may set/extend the queue
        mid-mission, and GO resolves at click time.

        Flight indices WITHIN the queue's positional chunks fly them exactly
        as chunked. A flight index PAST them is a RECOVERY flight (I5): it
        carries whatever the queue still OWES — the positional queue minus
        what ``state.delivered_marker_ids`` says has actually been delivered
        — chunked the same size. None once nothing remains owed (including
        when nothing was ever queued), so the mission ends on this flight
        rather than holding for a pointless GO.
        """
        flights = chunk_flights(state.assigned_id_queue, state.eggs_aboard)
        if 1 <= flight <= len(flights):
            return flights[flight - 1]
        owed = remaining_owed(state.assigned_id_queue, state.delivered_marker_ids)
        if not owed:
            return None
        owed_chunks = chunk_flights(owed, state.eggs_aboard)
        idx = flight - len(flights) - 1
        return owed_chunks[idx] if 0 <= idx < len(owed_chunks) else None

    async def _gate(sortie: int) -> list[int] | None:
        if state.terminal != TerminalState.RUNNING:
            return None
        chunk = _chunk_for(sortie)
        # I1 (review 2026-07-24): _chunk_for already returns None once every
        # queued id is actually delivered — but that alone did not stop the
        # INTERACTIVE hold below from running anyway (a green PREFLIGHT board
        # after a completed mission), and /api/cmd/preflight/go resolves its
        # id POSITIONALLY (assigned_id_queue[sortie_index]) with no notion of
        # state.delivered_marker_ids — so the documented one-click GO handed
        # back an ALREADY-DELIVERED id, which the manual-override branch below
        # then turned into a bogus extra flight with no egg aboard. End the
        # mission here instead, for every branch (interactive/headless/skip),
        # the moment nothing remains owed. Gated on a NON-EMPTY queue: with no
        # queue at all, `chunk` is also None on flight 1 (nothing to chunk
        # yet), and that is the documented "no queue → the GO's manual pick IS
        # the flight" path — the only legitimate `not chunk` case, preserved
        # by this check never firing when the queue is empty.
        if (chunk is None and state.assigned_id_queue
                and not remaining_owed(state.assigned_id_queue,
                                       state.delivered_marker_ids)):
            return None
        if skip_preflight:
            logger.warning(f"[preflight] SKIPPED (--skip-preflight) — flight "
                           f"{sortie} assigned={chunk}")
            return chunk

        if can_push:
            # Interactive: hold for the operator's GO (which carries the id).
            state.assigned_marker_id = None
            state.awaiting_preflight_go = True
            state.phase = MissionPhase.PREFLIGHT
            state.preflight_resume_event.clear()
            logger.info(f"[preflight] flight {sortie}: holding for operator GO "
                        "(enter the committee-assigned pad id)…")
            while state.terminal == TerminalState.RUNNING:
                _evaluate()
                try:
                    await asyncio.wait_for(
                        state.preflight_resume_event.wait(), timeout=1.0)
                    break
                except asyncio.TimeoutError:
                    continue
            state.awaiting_preflight_go = False
            if (state.terminal != TerminalState.RUNNING
                    or not state.preflight_resume_event.is_set()):
                return None
            # Resolve AFTER the GO: the operator may have (re)set the queue
            # during the hold, and the endpoint resolves the id at click time.
            chunk = _chunk_for(sortie)
            manual = state.assigned_marker_id
            if manual is not None and (state.eggs_aboard == 1 or not chunk):
                # One egg per flight → a flight IS one delivery, so the GO
                # body's id (a manual pick, else the queue's slot) is the whole
                # flight: the documented per-flight manual override. With more
                # eggs aboard the queue's chunk wins — EXCEPT when there is no
                # queue at all, where the GO's id is the only assignment there
                # is and returning None would end the mission on a green GO.
                chunk = [manual]
            state.flight_ids = list(chunk or [])
            logger.info(f"[preflight] flight {sortie}: GO — assigned pads "
                        f"{state.flight_ids}")
            return chunk

        # Headless: the queued assignments ARE the committee (SITL / bench runs).
        if chunk is None:
            return None
        # A registry-known pad flies a much shorter delivery AND the watchdog's
        # time floor exempts its egress/landing — gate on the matching rule, and
        # only when EVERY pad this flight serves is already registered.
        known = all(tracker.confirmed_by_marker(a) is not None for a in chunk)
        ok = (policy.can_start_known_sortie(state.time_remaining_s()) if known
              else policy.can_start_sortie(state.time_remaining_s()))
        if state.window_started and not ok:
            logger.warning(f"[preflight] sortie {sortie}: window too short "
                           f"({state.time_remaining_s():.0f}s left for a "
                           f"{'known' if known else 'full-sweep'} sortie) — ending")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(time reserve)")
            return None
        # The energy twin of the same refusal. The dashboard GO endpoint has
        # honoured this since the budget shipped; headless did not, so a bench or
        # SITL run would launch on a pack the policy had already refused and let
        # the FC's low-battery failsafe end the sortie with the egg aboard.
        # Headless has no FORCE button — `--skip-preflight` is its escape hatch.
        if energy_policy is not None and not state.sortie_energy_ok:
            logger.warning(f"[preflight] sortie {sortie}: {state.energy_detail} "
                           "— ending (swap the pack, or --skip-preflight)")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(energy reserve)")
            return None
        if not state.param_pins_ok:
            logger.error(f"[preflight] sortie {sortie}: {state.param_pins_detail} "
                         "— refusing to launch outside the validated envelope")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(envelope params)")
            return None
        state.phase = MissionPhase.PREFLIGHT
        deadline = asyncio.get_running_loop().time() + headless_timeout_s
        logger.info(f"[preflight] sortie {sortie}: headless — auto-proceed when "
                    "all critical checks pass…")
        while (asyncio.get_running_loop().time() < deadline
               and state.terminal == TerminalState.RUNNING):
            if _evaluate():
                logger.info(f"[preflight] flight {sortie}: criticals pass — "
                            f"assigned pads {chunk}")
                return chunk
            await asyncio.sleep(1.0)
        if state.terminal != TerminalState.RUNNING:
            return None
        fails = ", ".join(
            i["id"] for i in (state.preflight_report or {}).get("items", [])
            if i.get("critical") and i.get("status") != "pass"
        )
        logger.error(f"[preflight] sortie {sortie}: headless timeout — unmet "
                     f"critical checks: {fails or 'unknown'}")
        state.record_anomaly("preflight_timeout")
        return None

    if not rc_go:
        return _gate

    async def _wait_rc_go(sortie: int) -> bool:
        """RC-GO hold (operator conops 2026-08-12): the GO above only STAGED
        the flight — nothing moves until the SAFETY PILOT arms via RC and
        flips the mode switch to OFFBOARD. A zero-velocity offboard setpoint
        is streamed at ~5 Hz for the whole hold: PX4 rejects the OFFBOARD
        switch without a live setpoint stream, and the stream alone never
        changes mode or spins a motor. On armed+OFFBOARD the mission takes
        over within ~200 ms (``arm_and_takeoff`` sees the vehicle already
        armed, skips the arm, and commands AUTO.TAKEOFF). No timeout: the
        20-min window clock starts at the flip (mission.py start_window runs
        AFTER this gate returns), so holding costs nothing."""
        state.phase = MissionPhase.PREFLIGHT
        logger.info(f"[preflight] flight {sortie}: RC-GO — staged; ARM with "
                    "the RC, then flip to OFFBOARD to launch…")
        state.record_audit(f"t={state.time_elapsed_s():.1f}s RC-GO flight "
                           f"{sortie} staged — waiting for RC arm + OFFBOARD")
        prime_warned = False
        while state.terminal == TerminalState.RUNNING:
            t = state.telemetry
            if t.is_armed and t.flight_mode == "OFFBOARD":
                logger.info(f"[preflight] flight {sortie}: RC-GO — armed + "
                            "OFFBOARD observed, launching")
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s RC-GO flight {sortie} "
                    "armed+OFFBOARD — launching")
                return True
            if commander is not None:
                try:
                    await commander.prime_offboard_hold()
                    prime_warned = False
                except Exception as e:
                    # Keep holding — the OFFBOARD switch simply won't engage
                    # until the stream is back. Warn once per outage.
                    if not prime_warned:
                        logger.warning(f"[preflight] RC-GO setpoint stream "
                                       f"failed ({e}) — retrying")
                        prime_warned = True
            await asyncio.sleep(0.2)
        return False

    async def _rc_go_gate(sortie: int) -> list[int] | None:
        chunk = await _gate(sortie)
        if chunk is None:
            return None
        if not await _wait_rc_go(sortie):
            return None
        return chunk

    return _rc_go_gate


async def run(args: argparse.Namespace) -> int:
    cfg = _load_config(Path(args.config))
    profile = load_profile(args.profile)
    logger.info(f"[main] profile={profile.name} ceiling={profile.altitude_ceiling_m} m "
                f"transit={profile.transit_alt_m} m floor={profile.search_floor_m} m "
                f"deliveries≤{profile.max_sorties} eggs={profile.eggs_aboard}")

    # Real-lens camera calibration (FOV + mount angles) from config, applied in
    # place to the shared NADIR model. Defaults match SITL; set the REAL
    # values in the config `cameras:` block at G5/G6 before free flight.
    cams = cfg.get("cameras") or {}
    if cams:
        configure_cameras(nadir=cams.get("nadir"))
        logger.info("[main] applied camera calibration from config (cameras:)")

    geofence = [tuple(v) for v in cfg.get("controlled_airspace", [])]
    search_area = [tuple(v) for v in cfg.get("search_area", [])]
    transit_route = [Coordinate(lat=float(v[0]), lon=float(v[1]))
                     for v in cfg.get("transit_route", [])]

    # ── connect ──
    # The fraction below which the FC flies its own low-battery RTL. Resolved
    # ONCE: it is simultaneously the failsafe pinned on the FC, the reserve the
    # energy budget refuses to plan into, and the pre-flight card's hard floor.
    # Read separately by each, they could drift apart and the planner would be
    # planning through a failsafe.
    bat_low_thr = float((cfg.get("failsafes") or {}).get("bat_low_thr", 0.25))
    conn = _build_connection(cfg.get("connection") or {}, args.connect)
    commander = DroneCommander(conn)

    # Bare placeholder telemetry until the subscriber starts (state needs one).
    telem = TelemetrySubscriber(commander.system)
    # Optional raw pymavlink listener: augments telemetry with per-servo PWM,
    # per-ESC current/RPM, and consumed-mAh (fields MAVSDK doesn't expose) for the
    # dashboard. Reads a dedicated UDP endpoint (SITL: PX4's 14550 GCS broadcast;
    # real CM4: a mavlink-router [UdpEndpoint raw]). 0/absent disables it, and any
    # failure degrades gracefully — the safety watchdog never reads these fields.
    raw_telem = RawMavlinkSubscriber(
        telem.state,
        udp_port=int((cfg.get("connection") or {}).get("raw_telemetry_port", 0)),
    )
    # Build a minimal initial plan so state is valid; rebuilt once GPS is known.
    placeholder_home = Coordinate(
        lat=float(cfg.get("site", {}).get("center_lat", 0.0)),
        lon=float(cfg.get("site", {}).get("center_lon", 0.0)),
    )
    # Blind search inside the SEARCH AREA (the pads live there); the controlled
    # airspace is the geofence. Both polygons are mission-critical (V1.3).
    if len(geofence) < 3:
        logger.error("[main] no controlled_airspace polygon — the geofence is mandatory")
        return 2
    if len(search_area) < 3:
        logger.error("[main] no search_area polygon — the sweep needs the rules' "
                     "search area (config search_area:)")
        return 2
    if not transit_route:
        logger.error("[main] no transit_route — the rules mandate the P1→P2→P3 "
                     "corridor (config transit_route:)")
        return 2
    if len(transit_route) != 3:
        logger.warning(f"[main] transit_route has {len(transit_route)} points "
                       "(rules publish 3) — flying it as configured")
    sc = cfg.get("search", {}) or {}
    mc = cfg.get("mission", {}) or {}
    spec = _build_spec(search_area, placeholder_home, sc, profile.altitude_ceiling_m)
    tracker = _build_tracker(sc)
    plan = render_live_plan(placeholder_home, spec, discovered=[], profile=profile,
                            transit_route=transit_route)

    # Headless/SITL committee stand-in: --assigned-ids beats the config list.
    # Both are validated against the competition id set (1..6) — fail fast.
    if args.assigned_ids:
        assigned_ids = _parse_assigned_ids(str(args.assigned_ids))
    else:
        assigned_ids = _parse_assigned_ids(mc.get("assigned_marker_ids", []))

    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem.state,
        operation_window_s=profile.operation_window_s,
        max_sorties=int(mc.get("max_sorties", profile.max_sorties)),
    )
    # ONE queue for both operator flows: --assigned-ids/config seeds it here;
    # the GCS may (re)set it via POST /api/cmd/mission_ids. The per-flight gate
    # and the dashboard GO both consume state.assigned_id_queue.
    state.assigned_id_queue = list(assigned_ids)
    # FLIGHTS vs DELIVERIES. The queue lists DELIVERIES (one pad each); one
    # FLIGHT (arm→disarm) carries up to eggs_aboard of them, so max_sorties —
    # the loop's flight budget — is the number of flights those deliveries need.
    # Config `mission.max_sorties` has always meant "≤4 pads", i.e. deliveries,
    # so it seeds max_deliveries.
    state.eggs_aboard = int(mc.get("eggs_aboard", profile.eggs_aboard))
    state.max_deliveries = int(mc.get("max_deliveries", state.max_sorties))
    # Budget the maximum LEGAL number of flights, not the count the CURRENT
    # queue implies: /api/cmd/mission_ids may extend the queue mid-mission (the
    # queue is re-read at every hold), and a ceiling frozen at the initial
    # length would accept that write and then never fly it. budgeted_flights_for
    # (I5, review 2026-07-24) adds at least one RECOVERY flight beyond that
    # best-case count — at the shipping eggs_aboard=4 default max_deliveries
    # alone budgets exactly ONE flight, so a flight that came home with
    # undelivered eggs previously had no way to get them flown; see
    # _chunk_for's docstring for the gate-side half of this fix. The outer
    # max(1, ...) keeps this line's own floor >= 1 even though mission.py's
    # loop clamps the same way — budgeted_flights_for(0, ...) is 0 (nothing to
    # ever recover), and this is what gets logged just below.
    state.max_sorties = max(1, budgeted_flights_for(state.max_deliveries,
                                                     state.eggs_aboard))
    logger.info(f"[main] {state.max_deliveries} deliveries max, "
                f"{state.eggs_aboard} egg(s) aboard → ≤{state.max_sorties} "
                f"flight(s); queue={state.assigned_id_queue or 'unset'}")
    # Crash-safe audit trail.
    run_dir = audit.setup_run_logging(plan.mission_id)
    audit_log = audit.AuditLog(run_dir / "audit.jsonl")
    # Live pad feed for the AAVC GCS console (user contract 2026-08-12: a pad
    # appears on the console map only once the drone has SCANNED/confirmed
    # it). Constructed here — before any flying — because its first write
    # clears whatever stale mission_status.json a previous run left behind.
    # The audit sink is teed so DELIVERY … RELEASE lines colour pads
    # delivered on the console; the scan-side hook is registered on the
    # VisionWorker below.
    gcs_feed = GcsMissionStatus(
        path=Path(cfg.get("gcs_status_path", "captures/mission_status.json")),
        origin_lat=float(cfg["site"]["center_lat"]),
        origin_lon=float(cfg["site"]["center_lon"]),
        assigned=assigned_ids,
        serve_cost_s=float(sc.get("serve_cost_s", 80.0)),
    )

    def _audit_tee(entry: str) -> None:
        audit_log.record(entry)
        gcs_feed.on_audit(entry)

    state.audit_sink = _audit_tee

    # ── optional GCS dashboard (decoupled seam) ──
    dash = None
    if not args.no_dashboard:
        try:
            from dashboard.integration import start_dashboard
            dash = await start_dashboard(
                state, commander, host=args.host, port=args.port
            )
            logger.info(f"[main] dashboard up at http://{args.host}:{args.port}")
        except Exception as e:
            logger.warning(f"[main] dashboard unavailable ({e}); running headless")

    on_drop_prediction = getattr(dash, "record_drop", None)

    try:
        await commander.connect()
        state.link_connected = True
        await telem.start()
        await raw_telem.start()
        logger.info("[main] waiting for GPS fix…")
        if not await _wait_for_gps(state):
            logger.error("[main] no GPS fix — aborting")
            state.set_terminal(TerminalState.FAILED, MissionPhase.ABORT)
            return 3

        # Real launch point = current fix (works with coarse, no-RTK GPS). Rebuild
        # the search pattern about the true home now that GPS is known. The sweep
        # covers the SEARCH AREA polygon (the geofence is the airspace boundary).
        home = Coordinate(lat=state.telemetry.lat, lon=state.telemetry.lon)
        spec = _build_spec(search_area, home, sc, profile.altitude_ceiling_m)
        state.plan = render_live_plan(home, spec, discovered=[], profile=profile,
                                      transit_route=transit_route)
        logger.info(f"[main] home=({home.lat:.7f},{home.lon:.7f}); search area "
                    f"{spec.leg_count} legs / {len(spec.waypoints)} waypoints @ "
                    f"{spec.sweep_alt_m:.0f} m, transit {len(transit_route)} pts @ "
                    f"{profile.transit_alt_m:.0f} m")

        # ── FC-level failsafes (independent of the companion) ──
        # Always ON now. They used to be switched OFF for the System-ID sweep,
        # which was the only flight this stack ever made that wanted them off;
        # that module was removed 2026-08-15 (PX4's own autotune replaces it),
        # and with it the only reason this code could ever disarm a failsafe.
        # The fence is FAIL-CLOSED (2026-08-17). It used to record an anomaly and
        # fly on, which was survivable only while GF_MAX_HOR_DIST held a radius
        # fence underneath it — that one is a stored param, so it is in force
        # whether or not an upload succeeds. The operator retired the radius that
        # day: a circle cannot describe a rotated-rectangle field (it either
        # clips legal airspace or leaks outside it), and a wrong one kills the
        # mission outright — a live GF_MAX_HOR_DIST=15 would have RTL'd us ~7 m
        # past P1. So this polygon is now the ONLY FC-level fence, and PX4 treats
        # a missing one as "accept all points" rather than as an error
        # (Geofence::isInsidePolygonOrCircle). An unverified upload therefore
        # means no airspace limit on the aircraft at all, with only the companion
        # left holding the rule — the exact single-layer state the GF_ACTION fix
        # existed to end. Refuse the flight; the operator re-runs on a good link.
        if geofence and len(geofence) >= 3:
            try:
                await commander.upload_geofence(geofence)
                await commander.set_geofence_action_rtl()
            except Exception as e:
                logger.error(f"[main] FC geofence NOT verified ({e}) — refusing to "
                             "fly: PX4 reads a missing fence as 'accept all points', "
                             "so this would launch with no airspace limit on board")
                state.record_anomaly(f"geofence setup failed: {e}")
                state.set_terminal(TerminalState.FAILED, MissionPhase.ABORT)
                return 4
        try:
            await commander.set_datalink_loss_rtl(profile.datalink_loss_threshold_s)
        except Exception as e:
            state.record_anomaly(f"datalink-loss RTL setup failed: {e}")
        # RC-loss RTL (S4) — never pinned before; the real bird rode on the
        # FC/QGC default. COM_RCL_EXCEPT=4 (inside set_rc_loss_rtl) exempts
        # Offboard/Mission so a no-RC SITL/autonomous run isn't RTL'd.
        try:
            await commander.set_rc_loss_rtl()
        except Exception as e:
            state.record_anomaly(f"RC-loss RTL setup failed: {e}")
        # FC-level battery failsafe (S4) — the authoritative backstop below
        # the companion watchdog's 30%/20% RTH/LAND.
        fs = cfg.get("failsafes", {}) or {}
        try:
            await commander.set_battery_failsafe(
                low=bat_low_thr,
                crit=float(fs.get("bat_crit_thr", 0.15)),
                emergen=float(fs.get("bat_emergen_thr", 0.07)),
                action=int(fs.get("low_bat_act", 3)),
            )
        except Exception as e:
            state.record_anomaly(f"battery failsafe setup failed: {e}")
        # (One-motor-out detection used to be pinned here — CA_FAILURE_MODE +
        # COM_ACT_FAIL_ACT. REMOVED 2026-08-17: the ESCs on this airframe are
        # PWM-only with no telemetry lead, so nothing can ever report a motor
        # failure and both pins were arming a detector that cannot fire. See
        # CLAUDE.md §2; getting it back is an ESC purchase, not a param.)
        # Stabilized-nadir camera gimbal (PX4 mount driver) — best-effort:
        # SITL's PX4 lacks the module (params warn + skip); the real 6X is
        # configured here so the servo holds the camera straight down.
        gc = cfg.get("gimbal", {}) or {}
        if gc.get("enabled", False) and gc.get("params"):
            try:
                await commander.set_gimbal_mount(gc["params"])
            except Exception as e:
                state.record_anomaly(f"gimbal mount setup failed: {e}")
        # ── flight tuning (applied once before takeoff) ──
        # Outer-loop limits (anti-flip tilt, yaw-rate, speed unlock, tighter
        # waypoint acceptance). Config `px4_tuning:`, else the reviewed defaults.
        energy_policy = _build_energy_policy(cfg, bat_low_thr, state.eggs_aboard)
        state.energy_capacity_mah = energy_policy.capacity_mah
        px4_tuning = cfg.get("px4_tuning") or DEFAULT_PX4_TUNING
        try:
            n_pins = await commander.apply_param_overrides(px4_tuning)
            if n_pins < len(px4_tuning):
                state.record_anomaly(
                    f"px4 tuning: only {n_pins}/{len(px4_tuning)} params applied")
            # Applying is best-effort; the ENVELOPE is not. Read the pins back —
            # "applied 0/24" (stale mavsdk_server on the ports) would otherwise
            # fly the mission on PX4 defaults: RTL at 60 m and 1.5 m/s onto the pad.
            bad_pins = await commander.verify_envelope_pins(px4_tuning)
        except Exception as e:
            state.record_anomaly(f"px4 tuning apply failed: {e}")
            bad_pins = ["the whole tuning block failed to apply"]
        state.param_pins_ok = not bad_pins
        state.param_pins_detail = (
            "envelope pins held by the FC" if not bad_pins
            else "FC is NOT holding: " + "; ".join(bad_pins))
        if bad_pins:
            state.record_anomaly(
                "flight-envelope params are at their PX4 defaults — "
                + state.param_pins_detail
                + ". A failsafe RTL would climb through the 20 m ceiling and the "
                "pad descent is ~4x the validated speed. Fix the link (a stale "
                "mavsdk_server holding :14540/:50051 fails every param RPC) and "
                "restart, or launch with FORCE at your own risk.")
        # SITL only — PX4's battery simulator, so the energy budget is
        # exercisable before hardware. These params do NOT exist on the real 6X
        # (the module is not in the fmu-v6x build), and cm4/launch_flight.sh
        # ships the same config file, so gate on the SITL endpoint rather than on
        # the block's presence: on hardware each write would only burn a param
        # timeout on the pad, inside the scored window.
        sim_bat = cfg.get("sim_battery") or {}
        if sim_bat and _is_sitl_endpoint(conn.system_address):
            try:
                await commander.apply_param_overrides(
                    {k: float(v) for k, v in sim_bat.items()})
            except Exception as e:  # noqa: BLE001
                logger.debug(f"[main] sim battery params not applied: {e}")
        elif sim_bat:
            logger.info("[main] sim_battery block skipped — not a SITL endpoint "
                        f"({conn.system_address})")
        # A pack swapped without updating the config and an uncalibrated power
        # module look identical from here, and both make every mAh number
        # fiction. Say so once, loudly, rather than silently trusting it.
        try:
            fc_capacity = float(await commander.get_param_float("BAT1_CAPACITY"))
        except Exception:  # noqa: BLE001 — never block a launch on a param read
            fc_capacity = -1.0
        if fc_capacity <= 0:
            # EXPECTED on this airframe since 2026-08-17, not a defect to fix.
            # BAT1_CAPACITY <= 0 is what puts PX4 on the voltage-only branch of
            # estimateStateOfCharge, and that is the only honest branch here:
            # the PM02D (2026-08-20, replacing the failed PM03D's stand-in
            # converter) powers ONLY the FC, the motors run off a board the FC
            # cannot sense, so coulomb counting would integrate the ~0.7 A
            # avionics draw alone and report a percentage that is too high,
            # quietly. Recorded so the audit trail says which gauge flew, NOT
            # as a request to set a capacity.
            state.record_anomaly(
                "BAT1_CAPACITY<=0 — FC state-of-charge is voltage-only, by design "
                "on the PM02D-avionics-only wiring (the FC cannot see motor "
                "current). Percentages depend entirely on "
                "BAT1_V_DIV/V_EMPTY/V_CHARGED")
        elif abs(fc_capacity - energy_policy.capacity_mah) > 100.0:
            state.record_anomaly(
                f"battery capacity mismatch: FC {fc_capacity:.0f} mAh vs config "
                f"{energy_policy.capacity_mah:.0f} mAh")

        # ── safety watchdog ──
        watchdog = SafetyWatchdog(
            state, commander, controlled_airspace=cfg.get("controlled_airspace", []),
            rth_battery_pct=profile.rth_battery_pct,
            land_battery_pct=profile.land_battery_pct,
            min_time_remaining_for_continue_s=profile.min_time_remaining_s,
            datalink_loss_threshold_s=profile.datalink_loss_threshold_s,
            gps_loss_threshold_s=profile.gps_loss_threshold_s,
            telemetry_stale_threshold_s=profile.telemetry_stale_threshold_s,
            geofence_margin_m=profile.geofence_margin_m,
            no_fly_zones=cfg.get("no_fly_zones", []),
            altitude_ceiling_m=profile.altitude_ceiling_m,
            search_floor_m=profile.search_floor_m,
            home_lat=home.lat, home_lon=home.lon,
            on_rth=lambda: state.record_audit("watchdog: RTH triggered"),
            on_abort=lambda: state.record_audit("watchdog: ABORT triggered"),
        )
        await watchdog.start()

        # ── vision worker → target tracker (+ dashboard feed) ──
        vc = cfg.get("vision", {}) or {}
        frame_max_age_s = float(vc.get("frame_max_age_s", DEFAULT_FRAME_MAX_AGE_S))
        vision = VisionWorker(state, target_description=profile.default_target,
                              frame_max_age_s=frame_max_age_s,
                              interval_s=float(vc.get("poll_interval_s",
                                                      DEFAULT_INTERVAL_S)),
                              decode_workers=int(vc.get("decode_workers", 1)))
        vision.on_fix(tracker.ingest)     # discovery: confirm targets from fixes
        # AAVC GCS console feed: registered AFTER tracker.ingest (same ordering
        # rule as the dashboard pusher below) and independent of it, so
        # --no-dashboard runs still light pads up on the console map.
        vision.on_fix(gcs_feed.tracker_pusher(tracker))
        if dash is not None and getattr(dash, "broadcaster", None) is not None:
            b = dash.broadcaster
            if hasattr(b, "record_vision"):
                vision.on_observation(b.record_vision)
            if hasattr(b, "record_detected_objects"):
                # Live map pins = tracker CONFIRMATIONS, pushed the moment each
                # target is confirmed mid-sweep (one event per target) — NOT the
                # raw per-frame detections, which flickered all over the map.
                # Registered AFTER tracker.ingest so the fix is clustered first.
                vision.on_fix(_confirm_pusher(tracker, b))
        await vision.start()

        # ── mission frame recorder (R2: the RECORD half of "record and transmit") ──
        rc = cfg.get("recording", {}) or {}
        frames = FrameRecorder(
            state, run_dir / "frames",
            nadir=vision.nadir_frame,
            hz=float(rc.get("hz", 1.0)),
            jpeg_quality=int(rc.get("jpeg_quality", 80)),
            enabled=bool(rc.get("enabled", True)),
        )
        await frames.start()

        # ── fly (the per-sortie gate holds in PREFLIGHT before every launch) ──
        align = _align_for(profile, frame_max_age_s, _align_tuning(cfg))
        policy = _build_time_policy(sc, profile)
        sortie_gate = _sortie_gate_factory(
            state, dash=dash, home=home, geofence=geofence, cfg=cfg,
            profile=profile, policy=policy, energy_policy=energy_policy,
            tracker=tracker,
            skip_preflight=args.skip_preflight,
            commander=commander, rc_go=args.rc_go,
        )
        on_plan_update = _plan_pusher(dash, gcs_feed)

        # 1 Hz mirror of the REAL mission phase + flight clock onto the AAVC
        # GCS console's stepper (user report 2026-08-12: the mission bar never
        # moved with the actual flight). Poll rather than hook: state.phase is
        # assigned from a dozen sites across mission.py/tactical_align.py, and
        # a 1 s cadence also keeps the console's 45 s staleness gate fed.
        async def _gcs_progress_poll() -> None:
            while True:
                try:
                    gcs_feed.set_progress(
                        state.phase.value, state.time_elapsed_s(),
                        delivered=len(state.delivered_marker_ids),
                        assigned=len(state.assigned_id_queue
                                     or state.flight_ids or []),
                    )
                except Exception:  # display aid — never disturb the mission
                    pass
                await asyncio.sleep(1.0)

        gcs_poll = asyncio.create_task(_gcs_progress_poll())
        try:
            await _guard_mission(
                lambda: run_delivery_mission(
                    commander, state, tracker, spec,
                    home=home, transit_route=transit_route, sortie_gate=sortie_gate,
                    profile=profile, align=align, policy=policy,
                    max_pads=int(sc.get("max_pads", 6)),
                    decode_dwell_s=float(sc.get("decode_dwell_s", 4.0)),
                    on_drop_prediction=on_drop_prediction,
                    on_plan_update=on_plan_update,
                    refresh_energy=lambda: _evaluate_energy(state, energy_policy),
                ),
                state, commander,
            )
        finally:
            gcs_poll.cancel()
            # Tear down supervision + telemetry on EVERY path (success or the
            # emergency RTH above) — previously these only ran on the happy path.
            for label, stop in (
                ("frames", frames.stop),
                ("vision", vision.stop),
                ("watchdog", watchdog.stop),
                ("raw_telem", raw_telem.stop),
                ("telem", telem.stop),
            ):
                try:
                    await stop()
                except Exception:
                    logger.exception(f"[main] {label}.stop() failed")

        # ── SITL-only audit: discovered vs ground truth (NEVER used for planning) ──
        if args.truth_json:
            truth = audit.read_truth_targets(args.truth_json)
            if truth:
                comp = audit.compare_with_truth(tracker.snapshot(), truth)
                for line in comp.lines:
                    logger.info(f"[audit] {line}")
                    state.record_audit(f"AUDIT {line}")

        logger.info(f"[main] mission terminal={state.terminal.value}")
        gcs_feed.set_done(state.time_elapsed_s())
        return 0 if state.terminal in (
            TerminalState.COMPLETED, TerminalState.LANDED_RTH) else 1
    finally:
        if dash is not None:
            try:
                # Bound the teardown so a wedged dashboard shutdown can never hang
                # the process after a completed mission (post-mission hang, 2026-06).
                await asyncio.wait_for(dash.stop(), timeout=10.0)
            except Exception:
                pass
        # Kill the embedded mavsdk_server: its non-daemon stdout-reader thread
        # otherwise blocks interpreter shutdown, hanging the process after a
        # completed mission (post-mission hang root cause, see commander.close()).
        try:
            commander.close()
        except Exception:
            pass


def _rungs_for(profile: Any) -> tuple[float, ...]:
    """Descend rungs clamped under the ceiling, ending near the ground (the
    final 2 m rung tightens the land-ON drift for the 1 m pad).

    Low-ceiling profiles (KMUTNB sky-field, ceiling 5 m) get their own short
    ladder: the legacy filter would emit ``(5, 5, 3, 2, 1.5)`` — a duplicated
    top rung AND mis-indexed per-rung tolerances/speeds (AlignParams defaults
    are positional, sized for the 6-rung KMITL ladder)."""
    ceil = profile.altitude_ceiling_m
    if ceil <= 6.0:
        return (min(4.0, ceil - 1.0), 3.0, 2.0, 1.5)
    top = min(12.0, ceil)
    rungs = tuple(r for r in (top, 8.0, 5.0, 3.0, 2.0, 1.5) if r <= ceil)
    return rungs or (min(5.0, ceil),)


def _align_tuning(cfg: dict[str, Any]) -> dict[str, Any]:
    """The landing loop's RATE trio from the ``align:`` config block, as
    kwargs. Empty when the block is absent, so the dataclass defaults (the
    flown values) stand. Exposed for the field: ``tools/landing_trial.py``
    A/Bs land-ON precision, and the operator should be able to try a slower
    or faster loop without editing the flight core.

    Anything set here MUST keep the wall-clock constants intact — lock_cycles
    and max_lost_cycles are counted in cycles, so they scale WITH cycle_hz
    (see AlignParams)."""
    ac = cfg.get("align", {}) or {}
    out: dict[str, Any] = {}
    for key, cast in (("cycle_hz", float), ("lock_cycles", int),
                      ("max_lost_cycles", int), ("median_window", int)):
        if key in ac:
            out[key] = cast(ac[key])
    return out


def _align_for(profile: Any, frame_max_age_s: float,
               tuning: dict[str, Any] | None = None) -> AlignParams:
    """AlignParams matched to the profile's altitude band AND field geometry.

    A very low ceiling (<= 6 m) also needs the short 4-rung ladder with
    positionally-matched tolerances/descent speeds. The terminal accept radius,
    though, follows the FIELD not the ceiling — ``profile.terminal_accept_radius_m``
    — because pad SPACING, not altitude, is what a too-wide radius grabs the
    wrong pad from. (The old code hard-coded ``accept_radius_m=5`` inside the
    ``<= 6`` branch; when the KMUTNB ceiling rose 5 -> 10 m that branch stopped
    firing and the tight sky-field silently got the 15 m KMITL default — wider
    than its 14.5 m pad spacing.)"""
    rungs = _rungs_for(profile)
    accept = profile.terminal_accept_radius_m
    tune = dict(tuning or {})
    if profile.altitude_ceiling_m <= 6.0:
        return AlignParams(
            rungs=rungs,
            rung_tol_m=(1.0, 0.6, 0.35, 0.2),
            rung_descent_mps=(1.5, 0.8, 0.5, 0.4),
            accept_radius_m=accept,
            frame_max_age_s=frame_max_age_s,
            **tune,
        )
    return AlignParams(rungs=rungs, accept_radius_m=accept,
                       frame_max_age_s=frame_max_age_s, **tune)


def _build_spec(area: list[tuple[float, ...]], home: Coordinate,
                sc: dict[str, Any], ceiling_m: float) -> Any:
    """Boustrophedon sweep of the SEARCH AREA polygon + the `search:` config."""
    axis = sc.get("sweep_axis_deg")
    return build_search_pattern(
        area, home,
        sweep_alt_m=float(sc.get("sweep_alt_m", 12.0)),
        overlap_frac=float(sc.get("overlap_frac", 0.4)),
        margin_m=float(sc.get("margin_m", 5.0)),
        speed_mps=float(sc.get("speed_mps", 10.0)),
        ceiling_m=ceiling_m,
        axis_deg=float(axis) if axis is not None else None,
    )


def _build_tracker(sc: dict[str, Any]) -> TargetTracker:
    """Pad registry from the `search:` config (confirm votes, gates, clusters)."""
    band = sc.get("radius_band", [0.4, 2.5])
    return TargetTracker(
        cluster_radius_m=float(sc.get("cluster_radius_m", 8.0)),
        confirm_votes=int(sc.get("confirm_votes", 3)),
        min_confidence=float(sc.get("min_confidence", 0.5)),
        min_span_s=float(sc.get("min_span_s", 0.6)),
        target_radius_m=float(sc.get("target_radius_m", 0.2)),
        radius_band=(float(band[0]), float(band[1])),
        max_fix_ground_dist_m=float(sc.get("max_fix_ground_dist_m", 50.0)),
        serve_dedupe_m=float(sc.get("serve_dedupe_m", 12.0)),
    )


def _confirm_pusher(tracker: TargetTracker, broadcaster: Any) -> Any:
    """Closure that pushes each newly CONFIRMED target to the dashboard map the
    moment the tracker confirms it during the sweep (display-only — WHEN to
    serve stays the mission loop's decision). Runs on the VisionWorker thread;
    ``record_detected_objects`` is the broadcaster's thread-safe entry for that.
    """
    from dashboard.payloads import DetectedObjectEvent  # dashboard already up

    pushed: set[int] = set()
    show = (TargetState.CONFIRMED, TargetState.SERVING, TargetState.SERVED)

    def _push(_fix: Any) -> None:
        events = []
        for t in tracker.snapshot():
            if t.target_id in pushed or t.state not in show:
                continue
            pushed.add(t.target_id)
            events.append(DetectedObjectEvent(
                t_monotonic=time.monotonic(),
                label=(f"aruco pad {t.marker_id}" if t.marker_id is not None
                       else f"pad? #{t.target_id}"),
                clothing_color="unknown",
                member_count=t.votes_nadir,
                pose="confirmed",
                confidence=t.best_confidence,
                lat=t.lat, lon=t.lon,
                is_designated_match=t.marker_id is not None,
            ))
        if events:
            try:
                broadcaster.record_detected_objects(events)
            except Exception:
                logger.exception("[main] confirmed-target push failed (non-fatal)")

    return _push


def _plan_pusher(dash: Any, gcs_feed: Any) -> Any:
    """A closure that pushes every rebuilt live plan to the dashboard
    broadcaster (when present) AND the GCS console feed (always):
    mission_status.json gains ``plan``/``plan_ptr`` so the operator's map can
    show where the aircraft is going NEXT — the first write lands at gate
    release, while the launch-point WiFi still reaches the console (Fix 3,
    G7 debrief 2026-08-21: the takeover came early precisely because the
    screen could not answer that question). DROP_PAYLOAD commands ride their
    GOTO's position and are skipped; ``kind`` is the command's mission-phase
    tag and ``seq`` a 1-based display index."""
    b = getattr(dash, "broadcaster", None) if dash is not None else None
    if b is not None and not hasattr(b, "push_plan"):
        b = None

    def _push(plan: Any, pointer: int) -> None:
        if b is not None:
            try:
                b.push_plan(json.loads(plan.model_dump_json()), pointer)
            except Exception:
                logger.exception("[main] plan push failed")
        try:
            pts: list[list[Any]] = []
            for c in plan.commands:
                coord = getattr(c, "coord", None)
                if coord is None or c.kind == CommandKind.DROP_PAYLOAD:
                    continue
                pts.append([round(float(coord.lat), 7),
                            round(float(coord.lon), 7),
                            str(getattr(c.phase, "value", c.phase)),
                            len(pts) + 1])
            gcs_feed.set_plan(pts, int(pointer))
        except Exception:
            logger.exception("[main] gcs plan push failed")

    return _push


def main() -> None:
    p = argparse.ArgumentParser(
        description="AAVC 2026 V1.3 multi-sortie egg-delivery orchestrator")
    p.add_argument("--config", default="sitl/aavc_config.yaml",
                   help="field config (geofence, search area, transit route, marker)")
    p.add_argument("--truth-json", default=None,
                   help="OPTIONAL ground-truth pads JSON (SITL spawn_targets) — used "
                        "ONLY for a post-flight discovered-vs-truth audit, never for planning")
    p.add_argument("--connect", default=None,
                   help="MAVLink endpoint (default udpin://0.0.0.0:14540)")
    p.add_argument("--assigned-ids", default=None,
                   help="seed the ordered mission-id queue: comma-separated marker "
                        "ids, one per sortie (e.g. '3,1,4,6'); overrides config "
                        "mission.assigned_marker_ids. Headless runs consume the "
                        "queue directly; with a dashboard the operator can set it "
                        "(POST /api/cmd/mission_ids) or override any sortie with a "
                        "manual pick at GO")
    # KMUTNB repo default: the sky-field practice profile (5 m ceiling). The
    # KMITL-rules envelope is one flag away: --profile competition.
    p.add_argument("--profile",
                   default=os.environ.get("AAVC_PROFILE", "kmutnb_skyfield"))
    p.add_argument("--no-dashboard", action="store_true")
    p.add_argument("--rc-go", action="store_true",
                   help="the PILOT launches: after the GO stages the flight, "
                        "hold + stream offboard setpoints and release only "
                        "when the RC has ARMED and flipped to OFFBOARD "
                        "(flip to POSCTL mid-flight = orchestrator stands "
                        "down)")
    p.add_argument("--skip-preflight", action="store_true",
                   help="bypass the pre-flight readiness gate (bench/debug only)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    try:
        code = asyncio.run(run(args))
    except KeyboardInterrupt:
        # Operator Ctrl-C: asyncio.run has already cancelled the mission task and
        # run the teardown finally (telemetry/vision stop → dash.stop →
        # commander.close kills mavsdk_server) before re-raising. Swallow the
        # traceback and exit cleanly. (Foreground only — a backgrounded run gets
        # SIGINT=SIG_IGN from shell job control, so stop it with SIGTERM/SIGKILL.)
        logger.warning("[main] interrupted by operator (Ctrl-C) — exited")
        code = 130
    raise SystemExit(code)


if __name__ == "__main__":
    main()
