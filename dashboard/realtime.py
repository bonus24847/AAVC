"""RealtimeBroadcaster — fan-out of orchestrator state + events to WebSocket clients.

Maintains:

- A set of connected WebSocket clients
- A 5 Hz `_broadcast_loop` task that pushes one TelemetryFrame to every client
- Event-driven push for vision observations, detected objects, anomalies,
  MAVLink commands, drop predictions, and command session/result events
- Bounded ring buffers (deque maxlen=50) so late-joining clients see recent
  history on connect

If a client send fails, the client is dropped from the set; the broadcaster
never raises into orchestrator code paths.
"""

from __future__ import annotations

import asyncio
import collections
import math
import time
from collections.abc import Iterable
from typing import Any, Literal

from loguru import logger

from mission_brain.schemas import VisionAnalysis, active_airframe
from orchestrator.drop_trajectory import DropPrediction
from orchestrator.state import OrchestratorState

from .payloads import (
    MAVLINK_FOR_COMMAND,
    AnomalyEvent,
    CommandEvent,
    DetectedObjectEvent,
    DropPredictionEvent,
    TelemetryFrame,
    VisionEvent,
    WsEnvelope,
)

DEFAULT_BROADCAST_HZ = 5.0
DEFAULT_RING_MAXLEN = 50


class RealtimeBroadcaster:
    def __init__(
        self,
        state: OrchestratorState,
        broadcast_hz: float = DEFAULT_BROADCAST_HZ,
        ring_maxlen: int = DEFAULT_RING_MAXLEN,
    ) -> None:
        self.state = state
        self.broadcast_hz = broadcast_hz
        self._period_s = 1.0 / broadcast_hz
        self._clients: set[Any] = set()
        self._task: asyncio.Task[None] | None = None
        # Captured at start(); lets OFF-LOOP callbacks (the VisionWorker runs in
        # its own thread) schedule broadcasts thread-safely instead of calling
        # asyncio.create_task() with no running loop.
        self._loop: asyncio.AbstractEventLoop | None = None
        self._start = time.monotonic()
        # Strong refs to in-flight one-shot broadcast tasks. asyncio.create_task
        # keeps only a weak ref, so a one-shot event (command_result, anomaly)
        # could be GC'd before delivery. See _push_event.
        self._inflight: set[asyncio.Task[None]] = set()
        # Ring buffers for late joiners
        self._recent_vision: collections.deque[VisionEvent] = collections.deque(
            maxlen=ring_maxlen
        )
        self._recent_commands: collections.deque[CommandEvent] = collections.deque(
            maxlen=ring_maxlen
        )
        self._recent_anomalies: collections.deque[AnomalyEvent] = collections.deque(
            maxlen=ring_maxlen
        )
        self._recent_drops: collections.deque[DropPredictionEvent] = collections.deque(
            maxlen=ring_maxlen
        )
        self._recent_objects: collections.deque[DetectedObjectEvent] = collections.deque(
            maxlen=ring_maxlen
        )
        # Track which anomalies we've already pushed (state.anomalies grows)
        self._anomaly_cursor: int = 0

    # ----------------- lifecycle -----------------

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._task = asyncio.create_task(self._broadcast_loop())
        logger.info(
            f"[broadcaster] started, broadcast_hz={self.broadcast_hz}, "
            f"period={self._period_s * 1000:.0f}ms"
        )

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        for ws in list(self._clients):
            try:
                # Bound the close so one wedged (half-open) socket can't stall
                # dashboard teardown — which runs in main()'s finally and would
                # otherwise block process exit after the mission.
                await asyncio.wait_for(ws.close(), timeout=2.0)
            except Exception:
                pass
        self._clients.clear()

    # ----------------- client registry -----------------

    def add_client(self, ws: Any) -> None:
        self._clients.add(ws)

    def remove_client(self, ws: Any) -> None:
        self._clients.discard(ws)

    async def send_hello(self, ws: Any) -> None:
        """Send the recent history to a freshly-connected client."""
        env = WsEnvelope(kind="hello", payload={
            "telemetry": self._telemetry_frame().model_dump(),
            "recent_vision": [v.model_dump() for v in self._recent_vision],
            "recent_commands": [c.model_dump() for c in self._recent_commands],
            "recent_anomalies": [a.model_dump() for a in self._recent_anomalies],
            "recent_drops": [d.model_dump() for d in self._recent_drops],
            "recent_objects": [o.model_dump() for o in self._recent_objects],
        })
        await self._send_one(ws, env)

    # ----------------- public helpers for adjacent modules -----------------
    # dashboard/commands.py needs to (a) compute a broadcaster-epoch
    # timestamp and (b) push events onto the WS. We expose narrow public
    # accessors instead of letting commands.py reach into `_start` /
    # `_push_event`.

    def now_relative(self) -> float:
        """Monotonic seconds since broadcaster start. Matches the
        `t_monotonic` field carried on every other event payload."""
        return time.monotonic() - self._start

    def push_session_event(self, payload: dict[str, Any]) -> None:
        self._push_event("command_session", payload)

    def push_command_result(self, payload: dict[str, Any]) -> None:
        self._push_event("command_result", payload)

    def push_preflight(self, payload: dict[str, Any]) -> None:
        """Push the latest pre-flight readiness board (orchestrator.preflight)."""
        self._push_event("preflight", payload)

    def push_plan(self, plan: dict[str, Any], pointer: int) -> None:
        """Push a rebuilt mission plan so the map repaints without a page reload.

        Blind search grows the plan in flight (a serve pair per discovered
        target); the mission loop calls this on every rebuild. ``plan`` is the
        already-serialised MissionPlan dict; ``pointer`` is the live command index."""
        self._push_event("plan_update", {"plan": plan, "command_pointer": pointer})

    # System-ID + Autotune (pre-flight tuning module). All dict payloads built by
    # dashboard/tuner.py from tuning.* dataclasses — pushed through the generic fan-out.
    def push_sysid_status(self, payload: dict[str, Any]) -> None:
        self._push_event("sysid_status", payload)

    def push_sysid_result(self, payload: dict[str, Any]) -> None:
        self._push_event("sysid_result", payload)

    def push_tuner_design(self, payload: dict[str, Any]) -> None:
        self._push_event("tuner_design", payload)

    def push_tuner_apply(self, payload: dict[str, Any]) -> None:
        self._push_event("tuner_apply", payload)

    def push_autotune_status(self, payload: dict[str, Any]) -> None:
        self._push_event("autotune_status", payload)

    def record_detected_objects(self, events: list[DetectedObjectEvent]) -> None:
        """Push one or more DetectedObjectEvent into the ring + broadcast.

        Rebases each event's `t_monotonic` to the broadcaster epoch so
        the UI's time axis stays consistent. The producer (VisionWorker)
        stamps raw time.monotonic() because it has no reference to this
        broadcaster; the rebase is explicit here so a future caller can't
        forget.
        """
        now = time.monotonic() - self._start
        for ev in events:
            rebased = ev.model_copy(update={"t_monotonic": now})
            self._recent_objects.append(rebased)
            self._push_event("detected_object", rebased.model_dump())

    # ----------------- event hooks (called by orchestrator) -----------------

    def record_vision(self, analysis: VisionAnalysis) -> None:
        t = self.state.telemetry
        target_lat = target_lon = None
        if analysis.matches_designated_description:
            if not math.isnan(t.lat) and not math.isnan(t.lon):
                target_lat, target_lon = t.lat, t.lon
        ev = VisionEvent(
            t_monotonic=time.monotonic() - self._start,
            phase=self.state.phase.value,
            matches_designated_description=analysis.matches_designated_description,
            confidence=analysis.confidence,
            rationale=analysis.rationale,
            target_lat=target_lat,
            target_lon=target_lon,
        )
        self._recent_vision.append(ev)
        self._push_event("vision", ev.model_dump())

    def record_command(self, method: str, kwargs: dict[str, Any]) -> None:
        ev = CommandEvent(
            t_monotonic=time.monotonic() - self._start,
            method=method,
            args=self._sanitize(kwargs),
            mavlink=MAVLINK_FOR_COMMAND.get(method, ""),
        )
        self._recent_commands.append(ev)
        self._push_event("command", ev.model_dump())

    def record_drop(self, prediction: DropPrediction) -> None:
        # DropPrediction.points is the field name (orchestrator/drop_trajectory.py)
        traj = [(p.t_s, p.lat, p.lon, p.alt_agl_m) for p in prediction.points]
        ev = DropPredictionEvent(
            t_monotonic=time.monotonic() - self._start,
            impact_lat=prediction.impact_lat,
            impact_lon=prediction.impact_lon,
            impact_t_s=prediction.impact_t_s,
            horizontal_drift_m=prediction.horizontal_drift_m,
            trajectory=traj,
        )
        self._recent_drops.append(ev)
        self._push_event("drop_prediction", ev.model_dump())

    def _drain_new_anomalies(self) -> Iterable[AnomalyEvent]:
        """Yield AnomalyEvent for any anomalies added to state since last drain.

        Reads `state.anomaly_log` (record_anomaly entries only) — NOT
        `state.anomalies`, which doubles as the full audit trail and would
        flood the operator's anomaly feed with routine 1 Hz TELEM samples
        (2026-07-17). Falls back to `.anomalies` for a state without the field.
        """
        anomalies = getattr(self.state, "anomaly_log", None)
        if anomalies is None:
            anomalies = self.state.anomalies
        while self._anomaly_cursor < len(anomalies):
            msg = anomalies[self._anomaly_cursor]
            self._anomaly_cursor += 1
            ev = AnomalyEvent(
                t_monotonic=time.monotonic() - self._start,
                message=msg,
            )
            self._recent_anomalies.append(ev)
            yield ev

    # ----------------- internal -----------------

    def _telemetry_frame(self) -> TelemetryFrame:
        t = self.state.telemetry

        def _nf(v: float) -> float | None:
            return None if math.isnan(v) else v

        _af = getattr(getattr(self.state, "plan", None), "airframe", None)
        # Enum → its .value; a plain-string airframe passes through unchanged;
        # None (no plan yet) → whatever this process is configured to fly.
        airframe = str(getattr(_af, "value", _af) or active_airframe().value)
        return TelemetryFrame(
            t_monotonic=time.monotonic() - self._start,
            lat=_nf(t.lat),
            lon=_nf(t.lon),
            alt_msl_m=_nf(t.alt_m),
            alt_agl_m=_nf(t.relative_alt_m),
            ground_speed_mps=_nf(t.ground_speed_mps),
            heading_deg=_nf(t.heading_deg),
            roll_deg=_nf(t.roll_deg),
            pitch_deg=_nf(t.pitch_deg),
            roll_rate_dps=_nf(t.roll_rate_dps),
            pitch_rate_dps=_nf(t.pitch_rate_dps),
            yaw_rate_dps=_nf(t.yaw_rate_dps),
            battery_percent=_nf(t.battery_percent),
            battery_voltage_v=_nf(t.battery_voltage_v),
            battery_consumed_mah=_nf(t.battery_consumed_mah),
            battery_current_a=_nf(t.battery_current_a),
            battery_capacity_mah=(self.state.energy_capacity_mah or None),
            energy_tier=self.state.energy_tier,
            energy_sorties_left=_nf(self.state.energy_sorties_left),
            sortie_energy_ok=self.state.sortie_energy_ok,
            gps_fix_type=t.gps_fix_type,
            gps_satellites=t.gps_satellites,
            datalink_rssi=t.datalink_rssi,
            is_armed=t.is_armed,
            flight_mode=t.flight_mode,
            phase=self.state.phase.value,
            command_pointer=self.state.command_pointer,
            elapsed_s=self.state.time_elapsed_s(),
            remaining_s=self.state.time_remaining_s(),
            terminal=self.state.terminal.value,
            airframe=airframe,
            link_connected=self.state.link_connected,
            sortie_index=getattr(self.state, "sortie_index", 0),
            max_sorties=getattr(self.state, "max_sorties", 0),
            delivery_index=getattr(self.state, "delivery_index", 0),
            max_deliveries=getattr(self.state, "max_deliveries", 0),
            eggs_aboard=getattr(self.state, "eggs_aboard", 1),
            flight_ids=list(getattr(self.state, "flight_ids", [])),
            assigned_marker_id=getattr(self.state, "assigned_marker_id", None),
            assigned_id_queue=list(getattr(self.state, "assigned_id_queue", [])),
            sortie_time_ok=getattr(self.state, "sortie_time_ok", True),
            servo_pwm_us=list(t.servo_pwm_us),
            esc_current_a=list(t.esc_current_a),
            esc_rpm=list(t.esc_rpm),
        )

    def _push_event(
        self,
        kind: Literal[
            "telemetry", "vision", "command", "anomaly",
            "drop_prediction", "hello",
            "detected_object", "command_session", "command_result",
            "preflight", "plan_update",
            "sysid_status", "sysid_result", "tuner_design", "tuner_apply", "autotune_status",
            "ping",
        ],
        payload: dict[str, Any],
    ) -> None:
        env = WsEnvelope(kind=kind, payload=payload)
        loop = self._loop
        if loop is None:
            return  # not started yet — nothing is listening

        # Hold a strong ref until the broadcast finishes; a bare create_task is
        # weakly referenced and one-shot events could vanish under GC pressure
        # ("Task was destroyed but it is pending"). Mirrors the dispatched-task
        # guard in dashboard/commands.py.
        def _spawn() -> None:
            task = asyncio.create_task(self._broadcast(env))
            self._inflight.add(task)
            task.add_done_callback(self._inflight.discard)

        # record_vision / record_detected_objects fire from the VisionWorker's
        # OFF-LOOP thread, where asyncio.create_task() raises "no running event
        # loop". Schedule onto the captured loop thread-safely; call directly
        # only when we're already running on that loop.
        try:
            on_loop = asyncio.get_running_loop() is loop
        except RuntimeError:
            on_loop = False
        if on_loop:
            _spawn()
        else:
            # Off the loop (VisionWorker thread). If the loop is mid-shutdown
            # this raises RuntimeError on the worker thread where nothing would
            # catch it — swallow it; a dropped late event must not crash the
            # producer.
            try:
                loop.call_soon_threadsafe(_spawn)
            except RuntimeError:
                logger.warning("[broadcaster] loop closed; dropping off-loop event")

    async def _broadcast(self, env: WsEnvelope) -> None:
        if not self._clients:
            return
        dead: list[Any] = []
        msg = env.model_dump_json()
        for ws in list(self._clients):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    async def _send_one(self, ws: Any, env: WsEnvelope) -> None:
        try:
            await ws.send_text(env.model_dump_json())
        except Exception:
            self._clients.discard(ws)

    async def _broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(self._period_s)
            # Drain any new anomalies first so they ride alongside the frame
            for ev in self._drain_new_anomalies():
                await self._broadcast(WsEnvelope(kind="anomaly", payload=ev.model_dump()))
            frame = self._telemetry_frame()
            await self._broadcast(WsEnvelope(kind="telemetry", payload=frame.model_dump()))

    @staticmethod
    def _sanitize(d: dict[str, Any]) -> dict[str, Any]:
        """Make sure dict values are JSON-serialisable."""
        out: dict[str, Any] = {}
        for k, v in d.items():
            if isinstance(v, float) and math.isnan(v):
                out[k] = None
            elif isinstance(v, (int, float, str, bool, list, dict)) or v is None:
                out[k] = v
            else:
                out[k] = str(v)
        return out
