"""Dashboard command channel — operator-issued mission commands.

The web GCS sends POST /api/cmd/* to take manual control:

    POST /api/cmd/arm       Open the command session (acks back).
    POST /api/cmd/disarm    Close the session.
    POST /api/cmd/takeoff   Auto-takeoff (vehicle must already be armed).
    POST /api/cmd/hold      Hold position (AUTO.LOITER).
    POST /api/cmd/resume    Resume the uploaded mission (AUTO.MISSION).
    POST /api/cmd/rtl       Return-to-launch.
    POST /api/cmd/land      Land in place.
    POST /api/cmd/pause     Pause mission (PX4 holds at current item).
    POST /api/cmd/abort     Abort — land in place + mark ABORTED.
    POST /api/cmd/drop      Release a payload (commander.drop_payload()).

Safety design — two independent guards:
  1. CommandSession.armed must be true. The dashboard's "Arm to command"
     toggle flips this. Disarming on browser disconnect is the caller's
     responsibility (frontend sends /disarm on unmount / page-leave).
  2. CSRF mitigation: every /api/cmd/* endpoint requires the custom header
     `X-AAVC-CMD: 1`. Cross-origin form POSTs cannot add a custom header
     without triggering a CORS preflight, which we do not whitelist.

We do NOT add token auth. The dashboard binds to a private network
(orchestrator's host, port 8765) by design.

Every request — accepted OR rejected — appends to `state.anomalies` so
the existing audit log captures the full chain. Successful dispatch
returns a CommandResultEvent on the WS so any other open browser sees
what was sent.
"""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass
from typing import Any

from fastapi import APIRouter, Header, HTTPException, status
from loguru import logger
from pydantic import BaseModel, ConfigDict, Field, field_validator

from orchestrator.constants import TOUCHDOWN_ALT_GUARD_M
from orchestrator.state import OrchestratorState

from .payloads import CommandResultEvent, CommandSessionEvent
from .realtime import RealtimeBroadcaster

# Frontend must send this header on every /api/cmd/* call. It blocks
# cross-origin form-POST CSRF: a malicious page can submit a form to our
# endpoints but CANNOT add a custom header without triggering a CORS
# preflight, which we don't allow from foreign origins.
_REQUIRED_CMD_HEADER = "X-AAVC-CMD"
_REQUIRED_CMD_VALUE = "1"

# Command verbs whose blast-radius auto-disarms the session after they
# settle, so an accidental re-fire can't happen once the aircraft is down.
# `drop` is here so a second accidental egg-release click is rejected until the
# operator deliberately re-arms; `kill`/`vehicle_disarm` because they put the
# aircraft down.
DESTRUCTIVE_COMMANDS: frozenset[str] = frozenset(
    {"rtl", "land", "abort", "kill", "vehicle_disarm", "drop"}
)

# Manual DROP altitude interlock (S3). Mirrors the autonomous touchdown gate in
# orchestrator/tactical_align.py: release is refused unless telemetry reads at or
# below this height. A missing (NaN) altitude fails closed. `force` overrides.
_DROP_ALT_GUARD_M = TOUCHDOWN_ALT_GUARD_M


@dataclass
class CommandSession:
    """Module-level singleton — gates whether dashboard commands fire."""

    armed: bool = False
    armed_at: float | None = None
    operator_note: str = ""


class ArmRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    operator_note: str = Field(default="", max_length=200)


class CommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    operator_note: str = Field(default="", max_length=200)


class DropRequest(CommandRequest):
    # Override the touchdown-gated release refusal (S3). A forced airborne drop
    # is audited; the operator accepts the broken-egg risk.
    force: bool = False


class MissionIdsRequest(BaseModel):
    """The 4-of-6 ordered mission queue: which pads (in sortie order) the drone
    will serve. `ids=[]` clears the queue (pure per-sortie manual mode)."""

    model_config = ConfigDict(extra="forbid")
    operator_note: str = Field(default="", max_length=200)
    ids: list[int] = Field(default_factory=list, max_length=4)

    @field_validator("ids")
    @classmethod
    def _ids_valid(cls, v: list[int]) -> list[int]:
        for i in v:
            if not 1 <= i <= 6:
                raise ValueError(f"marker id {i} outside the valid set 1..6")
        if len(set(v)) != len(v):
            raise ValueError("mission queue ids must be distinct")
        return v


class PreflightGoRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    operator_note: str = Field(default="", max_length=200)
    # The operator must explicitly acknowledge the egg-loaded advisory
    # (it can't be verified pre-arm) before the launch gate releases.
    payload_confirmed: bool = False
    # THIS sortie's committee-assigned landing-pad marker id (handed over at
    # resupply/weigh-in). Required — the mission cannot serve without it.
    assigned_marker_id: int | None = Field(default=None, ge=1, le=6)
    # Override the time-policy refusal (the window can't cover another full
    # sortie): a late launch eats the per-minute overtime penalty, which is the
    # operator's call to make, not the software's.
    force: bool = False


def make_command_router(
    state: OrchestratorState,
    commander: Any,                       # DroneCommander or LoggedCommander
    broadcaster: RealtimeBroadcaster,
    session: CommandSession | None = None,
) -> APIRouter:
    """Build the FastAPI router exposing the command endpoints.

    `commander` is duck-typed so it accepts either the raw DroneCommander
    or the LoggedCommander wrapper used when the dashboard is active.
    """
    r = APIRouter(prefix="/api/cmd", tags=["commands"])
    # A shared CommandSession may be injected so adjacent routers enforce the
    # SAME arm gate; standalone callers get a fresh one.
    if session is None:
        session = CommandSession()

    # ---------- helpers ----------

    def _audit(text: str) -> None:
        state.record_audit(f"[cmd-channel] {text}")
        logger.info(f"[dashboard-cmd] {text}")

    def _emit_session(armed: bool, note: str) -> None:
        ev = CommandSessionEvent(
            t_monotonic=broadcaster.now_relative(),
            armed=armed,
            operator_note=note,
        )
        broadcaster.push_session_event(ev.model_dump())

    def _emit_result(name: str, ok: bool, detail: str, note: str) -> None:
        ev = CommandResultEvent(
            t_monotonic=broadcaster.now_relative(),
            command=name,
            ok=ok,
            detail=detail,
            operator_note=note,
        )
        broadcaster.push_command_result(ev.model_dump())

    def _require_armed() -> None:
        if not session.armed:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "Command channel is not armed. POST /api/cmd/arm with an "
                    "operator_note first."
                ),
            )

    def _check_header(x_aavc_cmd: str | None) -> None:
        """Reject if the required CSRF-mitigation header is missing."""
        if x_aavc_cmd != _REQUIRED_CMD_VALUE:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"missing or wrong {_REQUIRED_CMD_HEADER} header",
            )

    # Strong refs to in-flight dispatched tasks so the GC doesn't sweep
    # them mid-await. asyncio.create_task only keeps a weak reference;
    # losing the only reference can produce a "Task was destroyed but it
    # is pending!" warning and silent loss of the result event.
    dispatched_tasks: set[asyncio.Task[None]] = set()

    async def _dispatch(verb: str, make_coro: Any, note: str) -> dict[str, Any]:
        """Schedule a commander coroutine and ack immediately.

        `make_coro` is a zero-arg callable returning a coroutine. We build
        the coroutine INSIDE the try/except so a synchronous error in the
        call becomes a structured CommandResultEvent instead of an
        unhandled 500. Most MAVSDK verbs are long-running (RTL takes
        minutes); we don't await them inline — the broadcaster fans the
        result back when the underlying coroutine settles.
        """
        _audit(f"dispatch verb={verb} note={note!r}")
        loop = asyncio.get_running_loop()
        try:
            coro = make_coro()
        except Exception as e:
            _audit(f"verb={verb} construction failed: {e}")
            _emit_result(verb, ok=False, detail=f"construction: {e}", note=note)
            raise HTTPException(
                status_code=status.HTTP_502_BAD_GATEWAY,
                detail=f"commander.{verb} construction failed: {e}",
            ) from e

        async def _runner() -> None:
            try:
                await coro
                _emit_result(verb, ok=True, detail="dispatched", note=note)
            except Exception as e:
                _audit(f"verb={verb} failed: {e}")
                _emit_result(verb, ok=False, detail=str(e), note=note)
            finally:
                # After a destructive verb settles, auto-disarm the session
                # so an accidental re-fire can't happen once down. Runs in
                # `finally` so it triggers even on dispatch failure.
                if verb in DESTRUCTIVE_COMMANDS and session.armed:
                    session.armed = False
                    session.armed_at = None
                    _audit(f"SESSION AUTO-DISARMED after {verb}")
                    _emit_session(False, f"auto-disarm after {verb}")

        task = loop.create_task(_runner())
        dispatched_tasks.add(task)
        task.add_done_callback(dispatched_tasks.discard)
        # Mirror dashboard-issued verbs into the MAVLink command trace so the
        # CommandLog widget records them — commander.system.* calls bypass the
        # LoggedCommander wrapper entirely.
        try:
            broadcaster.record_command(f"dashboard_{verb}", {"operator_note": note})
        except Exception:
            logger.exception("[dashboard-cmd] broadcaster.record_command raised")
        return {"ok": True, "command": verb, "detail": "dispatched"}

    # ---------- session ----------

    @r.post("/arm")
    async def arm(
        req: ArmRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        session.armed = True
        session.armed_at = time.time()
        session.operator_note = req.operator_note
        _audit(f"SESSION ARMED note={req.operator_note!r}")
        _emit_session(True, req.operator_note)
        return {"ok": True, "armed": True}

    @r.post("/disarm")
    async def disarm(
        req: ArmRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        was = session.armed
        session.armed = False
        session.armed_at = None
        session.operator_note = req.operator_note
        if was:
            _audit(f"SESSION DISARMED note={req.operator_note!r}")
        _emit_session(False, req.operator_note)
        return {"ok": True, "armed": False}

    @r.get("/session")
    async def get_session() -> dict[str, Any]:
        return {
            "armed": session.armed,
            "armed_at": session.armed_at,
            "operator_note": session.operator_note,
        }

    # ---------- mission verbs ----------

    @r.post("/takeoff")
    async def cmd_takeoff(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Auto-takeoff — climbs to the PX4 takeoff altitude. The vehicle
        must already be armed; MAVSDK's action.takeoff() switches to
        Takeoff mode but does NOT arm."""
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch(
            "takeoff", lambda: commander.system.action.takeoff(), req.operator_note,
        )

    @r.post("/hold")
    async def cmd_hold(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Hold (Loiter) — hold position at current altitude."""
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch("hold", lambda: commander.system.action.hold(), req.operator_note)

    @r.post("/resume")
    async def cmd_resume(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch(
            "resume", lambda: commander.system.mission.start_mission(), req.operator_note,
        )

    @r.post("/rtl")
    async def cmd_rtl(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch("rtl", lambda: commander.rth(), req.operator_note)

    @r.post("/land")
    async def cmd_land(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch("land", lambda: commander.land(), req.operator_note)

    @r.post("/pause")
    async def cmd_pause(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        # PX4 Mission "pause" == set hold during mission.
        return await _dispatch(
            "pause", lambda: commander.system.mission.pause_mission(), req.operator_note,
        )

    @r.post("/abort")
    async def cmd_abort(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        # Hard abort — dispatch land FIRST so it's queued on the MAVSDK event
        # loop, THEN flip state.terminal. The other order made the segment loop
        # tear down the commander before land() could run.
        _audit("ABORT requested via dashboard")
        result = await _dispatch("abort", lambda: commander.land(), req.operator_note)
        from orchestrator.state import TerminalState
        state.terminal = TerminalState.ABORTED
        return result

    @r.post("/kill")
    async def cmd_kill(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Emergency motor cut (action.kill) — the vehicle drops. The GCS
        KillSwitch button posts here; without this route the click 404s."""
        _check_header(x_aavc_cmd)
        _require_armed()
        _audit("KILL requested via dashboard — cutting motors")
        return await _dispatch("kill", lambda: commander.abort(), req.operator_note)

    @r.post("/vehicle_arm")
    async def cmd_vehicle_arm(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Arm the PX4 motors (distinct from the command-channel session arm)."""
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch(
            "vehicle_arm", lambda: commander.system.action.arm(), req.operator_note,
        )

    @r.post("/vehicle_disarm")
    async def cmd_vehicle_disarm(
        req: CommandRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Disarm the PX4 motors."""
        _check_header(x_aavc_cmd)
        _require_armed()
        return await _dispatch(
            "vehicle_disarm", lambda: commander.system.action.disarm(), req.operator_note,
        )

    @r.post("/drop")
    async def cmd_drop(
        req: DropRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Release the egg via commander.drop_payload().

        Touchdown-gated (S3): the locked doctrine is that the egg is NEVER
        released airborne (a broken egg scores zero). We refuse the manual drop
        unless telemetry reads at/below the touchdown guard height — mirroring
        the autonomous path's airborne-SKIP — with an explicit `force` escape."""
        _check_header(x_aavc_cmd)
        _require_armed()
        alt = state.telemetry.relative_alt_m
        near_ground = (not math.isnan(alt)) and alt <= _DROP_ALT_GUARD_M
        if not near_ground and not req.force:
            state.record_anomaly("dashboard_drop_refused_airborne")
            _audit(
                f"DROP refused: airborne (rel_alt={alt:.1f}m > {_DROP_ALT_GUARD_M}m); "
                "touchdown-gated release. Re-issue with force to override."
            )
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    f"drop refused: vehicle airborne (rel_alt={alt:.1f} m). The egg is "
                    "released only after touchdown; set force=true to override."
                ),
            )
        if not near_ground and req.force:
            state.record_anomaly("dashboard_drop_forced_airborne")
            _audit(f"DROP forced airborne by operator (rel_alt={alt:.1f}m)")
        return await _dispatch("drop", lambda: commander.drop_payload(), req.operator_note)

    # ---------- pre-flight readiness gate ----------

    @r.get("/preflight")
    async def get_preflight() -> dict[str, Any]:
        """Latest readiness board + whether the mission is holding for GO."""
        # queued_id = what GO would resolve for THIS hold (sortie_index is
        # still the PREVIOUS sortie's number during the hold — see /go).
        queue = state.assigned_id_queue
        queued_id = (
            queue[state.sortie_index] if len(queue) > state.sortie_index else None
        )
        return {
            "awaiting_go": state.awaiting_preflight_go,
            "can_go": state.preflight_can_go,
            "report": state.preflight_report,
            "sortie_index": state.sortie_index,          # 1-based FLIGHT
            "max_sorties": state.max_sorties,            # flights in the window
            "delivery_index": state.delivery_index,      # 1-based DELIVERY
            "max_deliveries": state.max_deliveries,      # pads to serve
            "eggs_aboard": state.eggs_aboard,            # deliveries per flight
            "sortie_time_ok": state.sortie_time_ok,
            "sortie_energy_ok": getattr(state, "sortie_energy_ok", True),
            "energy_detail": getattr(state, "energy_detail", ""),
            "assigned_id_queue": list(queue),
            "queued_id": queued_id,
        }

    @r.post("/mission_ids")
    async def cmd_mission_ids(
        req: MissionIdsRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Set the 4-of-6 ordered mission queue (which pads the drone serves,
        in sortie order). No commander dispatch — pure state, like /arm.
        Allowed mid-mission: the change applies at the next PREFLIGHT hold
        (and to the CURRENT hold, since GO resolves at click time). The GO
        click per sortie is deliberately retained as the egg-loaded /
        crew-clear acknowledgment."""
        _check_header(x_aavc_cmd)
        _require_armed()
        # Against DELIVERIES, not flights: with 4 eggs aboard the whole queue is
        # ONE flight (max_sorties == 1), so validating against max_sorties would
        # 409 a perfectly valid 4-id queue.
        if len(req.ids) > state.max_deliveries:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"queue of {len(req.ids)} exceeds max_deliveries="
                       f"{state.max_deliveries}",
            )
        state.assigned_id_queue = list(req.ids)
        _audit(f"MISSION QUEUE set={req.ids or 'cleared'} note={req.operator_note!r}")
        _emit_result("mission_ids", ok=True,
                     detail=f"queue {req.ids or 'cleared'}", note=req.operator_note)
        return {"ok": True, "queue": list(req.ids)}

    @r.post("/preflight/go")
    async def cmd_preflight_go(
        req: PreflightGoRequest,
        x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        """Operator authorizes THIS sortie's launch. Fires only on a green board
        (all critical checks pass), with the egg-loaded advisory acknowledged,
        an assignment available (a manual id, or the mission queue set via
        /api/cmd/mission_ids), and the window able to cover another sortie
        (or `force`). With a queue, GO is ONE click — the id-entry step is
        gone but the human resupply ack deliberately stays."""
        _check_header(x_aavc_cmd)
        _require_armed()
        if not state.awaiting_preflight_go:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="not in pre-flight hold (mission is not awaiting GO)",
            )
        if not state.preflight_can_go:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="critical pre-flight checks are not all satisfied",
            )
        if not req.payload_confirmed:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="confirm the egg cargo is loaded before GO",
            )
        # Resolve the effective id: a manual pick overrides; else the mission
        # queue for THIS sortie. During the hold for sortie n, sortie_index is
        # still n-1 — the same slot the gate's queue[sortie-1] reads.
        effective_id = req.assigned_marker_id
        source = "manual"
        if effective_id is None and len(state.assigned_id_queue) > state.sortie_index:
            effective_id = state.assigned_id_queue[state.sortie_index]
            source = "queue"
        if effective_id is None:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="no assignment for this sortie — pick the id (1-6) or "
                       "set the mission queue (/api/cmd/mission_ids)",
            )
        if not getattr(state, "param_pins_ok", True) and not req.force:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="the FC is not holding the flight-envelope params ("
                       + (getattr(state, "param_pins_detail", "") or "unknown")
                       + ") — a failsafe RTL would climb through the 20 m "
                       "ceiling and the pad descent is ~4x validated. Restart "
                       "with a clean MAVLink link, or tick FORCE.",
            )
        if not getattr(state, "sortie_energy_ok", True) and not req.force:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="the pack can't cover another sortie — swap the battery, "
                       "or tick FORCE to launch on what is left",
            )
        if not state.sortie_time_ok and not req.force:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="remaining window can't cover another sortie + reserve — "
                       "tick FORCE to launch anyway (overtime penalty applies)",
            )
        # Order matters: the mission gate reads the id the moment the event
        # fires (single-threaded loop → this pair-write is race-free).
        state.assigned_marker_id = effective_id
        state.preflight_resume_event.set()
        _audit(f"PREFLIGHT GO — sortie {state.sortie_index + 1} pad="
               f"{effective_id} source={source} force={req.force} "
               f"note={req.operator_note!r}")
        _emit_result("preflight_go", ok=True,
                     detail=f"launch authorized (pad {effective_id}, {source})",
                     note=req.operator_note)
        return {"ok": True, "go": True, "assigned_marker_id": effective_id}

    return r
