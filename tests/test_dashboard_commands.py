"""Dashboard command router safety: kill/vehicle_arm/vehicle_disarm routes (S1)
and the touchdown-gated manual DROP guard (S3).

httpx/TestClient isn't a dependency, so we exercise the router the lean way:
build it, index its endpoints by path, and await them directly with an explicit
X-AAVC-CMD header. The router (guard logic + dispatch) is the unit under test;
the commander is a recording stub.
"""

from __future__ import annotations

import asyncio
import math
import types

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from dashboard.commands import (
    CommandRequest,
    CommandSession,
    DropRequest,
    MissionIdsRequest,
    PreflightGoRequest,
    make_command_router,
)
from dashboard.realtime import RealtimeBroadcaster
from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState

_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


def _state(alt: float = math.nan) -> OrchestratorState:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    telem = CurrentTelemetry()
    telem.relative_alt_m = alt
    return OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem)


class _Action:
    def __init__(self, calls: list[str]) -> None:
        self.calls = calls

    async def arm(self) -> None:
        self.calls.append("action.arm")

    async def disarm(self) -> None:
        self.calls.append("action.disarm")


class RecordingCommander:
    """Records what the router dispatches — no real MAVLink."""

    def __init__(self, *, pilot_in_control: bool = False) -> None:
        self.calls: list[str] = []
        self.drops: list[int] = []
        self.pilot_in_control = pilot_in_control
        self._system = types.SimpleNamespace(action=_Action(self.calls))

    @property
    def system(self) -> types.SimpleNamespace:
        return self._system

    async def abort(self) -> None:
        self.calls.append("abort")

    async def drop_payload(self, payload_id: int = 0) -> None:
        self.drops.append(payload_id)
        self.calls.append("drop_payload")


def _endpoints(
    state: OrchestratorState, commander: RecordingCommander, session: CommandSession
) -> dict[str, object]:
    bc = RealtimeBroadcaster(state)
    router = make_command_router(state, commander, bc, session)
    return {route.path: route.endpoint for route in router.routes}  # type: ignore[attr-defined]


def _armed() -> CommandSession:
    s = CommandSession()
    s.armed = True
    return s


async def _call(endpoint, req, header: str | None = "1"):  # type: ignore[no-untyped-def]
    """Invoke a route handler, then let the dispatched background task settle."""
    res = await endpoint(req, x_aavc_cmd=header)
    await asyncio.sleep(0.05)
    return res


# ── S1: kill + vehicle arm/disarm routes exist and dispatch ──


def test_kill_route_dispatches_abort_and_autodisarms() -> None:
    state, cmd, session = _state(), RecordingCommander(), _armed()
    eps = _endpoints(state, cmd, session)
    assert "/api/cmd/kill" in eps, "kill route missing — GCS KILL button would 404"
    asyncio.run(_call(eps["/api/cmd/kill"], CommandRequest()))
    assert cmd.calls == ["abort"]
    assert session.armed is False  # kill is destructive → session auto-disarms


def test_vehicle_arm_and_disarm_routes_dispatch() -> None:
    state, cmd, session = _state(), RecordingCommander(), _armed()
    eps = _endpoints(state, cmd, session)
    assert "/api/cmd/vehicle_arm" in eps
    assert "/api/cmd/vehicle_disarm" in eps
    asyncio.run(_call(eps["/api/cmd/vehicle_arm"], CommandRequest()))
    assert cmd.calls == ["action.arm"]
    assert session.armed is True  # arming a vehicle is not destructive

    cmd2, session2 = RecordingCommander(), _armed()
    eps2 = _endpoints(_state(), cmd2, session2)
    asyncio.run(_call(eps2["/api/cmd/vehicle_disarm"], CommandRequest()))
    assert cmd2.calls == ["action.disarm"]
    assert session2.armed is False  # vehicle_disarm is destructive


def test_kill_requires_header() -> None:
    eps = _endpoints(_state(), RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(_call(eps["/api/cmd/kill"], CommandRequest(), header=None))
    assert exc.value.status_code == 403


def test_kill_requires_armed_session() -> None:
    eps = _endpoints(_state(), RecordingCommander(), CommandSession())  # not armed
    with pytest.raises(HTTPException) as exc:
        asyncio.run(_call(eps["/api/cmd/kill"], CommandRequest()))
    assert exc.value.status_code == 409


# ── S3: touchdown-gated manual DROP guard ──


def test_drop_refused_when_airborne() -> None:
    state = _state(alt=10.0)
    cmd = RecordingCommander()
    eps = _endpoints(state, cmd, _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(_call(eps["/api/cmd/drop"], DropRequest()))
    assert exc.value.status_code == 409
    assert cmd.drops == []  # egg NOT released airborne
    assert any("airborne" in a for a in state.anomalies)


def test_drop_dispatched_when_near_ground() -> None:
    state = _state(alt=1.0)  # <= 2.5 m guard → touchdown-plausible
    cmd = RecordingCommander()
    eps = _endpoints(state, cmd, _armed())
    asyncio.run(_call(eps["/api/cmd/drop"], DropRequest()))
    assert cmd.drops == [0]


def test_drop_forced_airborne_releases_and_audits() -> None:
    state = _state(alt=10.0)
    cmd = RecordingCommander()
    eps = _endpoints(state, cmd, _armed())
    asyncio.run(_call(eps["/api/cmd/drop"], DropRequest(force=True)))
    assert cmd.drops == [0]
    assert any("forced" in a for a in state.anomalies)


def test_drop_refused_on_nan_altitude_without_force() -> None:
    state = _state(alt=math.nan)  # no altitude fix → fail closed
    cmd = RecordingCommander()
    eps = _endpoints(state, cmd, _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(_call(eps["/api/cmd/drop"], DropRequest()))
    assert exc.value.status_code == 409
    assert cmd.drops == []


# ── W3: the 4-of-6 ordered mission-id queue ──


def _holding_state(queue: list[int] | None = None, sortie_index: int = 0):
    """State in a PREFLIGHT hold with a green board (ready for GO)."""
    state = _state()
    state.awaiting_preflight_go = True
    state.preflight_can_go = True
    state.sortie_time_ok = True
    state.sortie_index = sortie_index
    if queue is not None:
        state.assigned_id_queue = list(queue)
    return state


def test_mission_ids_sets_queue_and_audits() -> None:
    state = _state()
    eps = _endpoints(state, RecordingCommander(), _armed())
    res = asyncio.run(eps["/api/cmd/mission_ids"](
        MissionIdsRequest(ids=[3, 1, 4, 6]), x_aavc_cmd="1"))
    assert res["ok"] is True and res["queue"] == [3, 1, 4, 6]
    assert state.assigned_id_queue == [3, 1, 4, 6]
    assert any("MISSION QUEUE" in a for a in state.anomalies)


def test_mission_ids_empty_clears_the_queue() -> None:
    state = _state()
    state.assigned_id_queue = [3, 1]
    eps = _endpoints(state, RecordingCommander(), _armed())
    asyncio.run(eps["/api/cmd/mission_ids"](MissionIdsRequest(ids=[]), x_aavc_cmd="1"))
    assert state.assigned_id_queue == []


def test_mission_ids_rejects_duplicates_and_out_of_range() -> None:
    with pytest.raises(ValidationError):
        MissionIdsRequest(ids=[3, 3])
    with pytest.raises(ValidationError):
        MissionIdsRequest(ids=[-1])            # 0 is a real pad since 2026-08-27
    with pytest.raises(ValidationError):
        MissionIdsRequest(ids=[7])
    with pytest.raises(ValidationError):
        MissionIdsRequest(ids=[1, 2, 3, 4, 5])    # > 4 entries


def test_mission_ids_rejects_more_than_max_deliveries() -> None:
    state = _state()
    state.max_deliveries = 2
    eps = _endpoints(state, RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps["/api/cmd/mission_ids"](
            MissionIdsRequest(ids=[3, 1, 4]), x_aavc_cmd="1"))
    assert exc.value.status_code == 409
    assert "max_deliveries" in exc.value.detail


def test_mission_ids_accepts_a_full_queue_flown_in_ONE_flight() -> None:
    """With 4 eggs aboard the whole queue is one FLIGHT (max_sorties == 1) —
    validating the queue length against flights would 409 a valid 4-id queue."""
    state = _state()
    state.eggs_aboard = 4
    state.max_deliveries = 4
    state.max_sorties = 1                       # one arm→disarm cycle
    eps = _endpoints(state, RecordingCommander(), _armed())
    res = asyncio.run(eps["/api/cmd/mission_ids"](
        MissionIdsRequest(ids=[3, 1, 4, 6]), x_aavc_cmd="1"))
    assert res["ok"] is True
    assert state.assigned_id_queue == [3, 1, 4, 6]


def test_mission_ids_requires_header_and_armed_session() -> None:
    state = _state()
    eps = _endpoints(state, RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps["/api/cmd/mission_ids"](
            MissionIdsRequest(ids=[3]), x_aavc_cmd=None))
    assert exc.value.status_code == 403
    eps2 = _endpoints(_state(), RecordingCommander(), CommandSession())  # not armed
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps2["/api/cmd/mission_ids"](
            MissionIdsRequest(ids=[3]), x_aavc_cmd="1"))
    assert exc.value.status_code == 409


def test_preflight_go_pulls_id_from_queue() -> None:
    state = _holding_state(queue=[3, 1, 4, 6], sortie_index=0)
    eps = _endpoints(state, RecordingCommander(), _armed())
    res = asyncio.run(eps["/api/cmd/preflight/go"](
        PreflightGoRequest(payload_confirmed=True), x_aavc_cmd="1"))
    assert res["assigned_marker_id"] == 3          # queue[sortie_index]
    assert state.assigned_marker_id == 3
    assert state.preflight_resume_event.is_set()


def test_preflight_go_manual_pick_overrides_queue() -> None:
    state = _holding_state(queue=[3, 1, 4, 6], sortie_index=0)
    eps = _endpoints(state, RecordingCommander(), _armed())
    res = asyncio.run(eps["/api/cmd/preflight/go"](
        PreflightGoRequest(payload_confirmed=True, assigned_marker_id=5),
        x_aavc_cmd="1"))
    assert res["assigned_marker_id"] == 5
    assert state.assigned_marker_id == 5


def test_preflight_go_409_when_queue_exhausted_and_no_manual_pick() -> None:
    state = _holding_state(queue=[3], sortie_index=1)   # sortie 2, queue len 1
    eps = _endpoints(state, RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps["/api/cmd/preflight/go"](
            PreflightGoRequest(payload_confirmed=True), x_aavc_cmd="1"))
    assert exc.value.status_code == 409
    assert not state.preflight_resume_event.is_set()


def test_preflight_go_still_requires_payload_confirmed_with_queue() -> None:
    """The queue removes the id-entry step, NOT the human resupply ack."""
    state = _holding_state(queue=[3, 1, 4, 6], sortie_index=0)
    eps = _endpoints(state, RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps["/api/cmd/preflight/go"](
            PreflightGoRequest(payload_confirmed=False), x_aavc_cmd="1"))
    assert exc.value.status_code == 409
    assert not state.preflight_resume_event.is_set()


def test_preflight_go_force_overrides_short_window() -> None:
    """The overtime penalty is the OPERATOR'S call (locked decision): with the
    board otherwise green but the window too short, GO without force 409s and
    GO with force fires. (Found live 2026-07-15: the time row doubling as a
    critical made FORCE a dead path — the board could never be green when
    force was needed.)"""
    state = _holding_state(queue=[3, 1, 4, 6], sortie_index=0)
    state.sortie_time_ok = False           # window can't cover another sortie
    eps = _endpoints(state, RecordingCommander(), _armed())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(eps["/api/cmd/preflight/go"](
            PreflightGoRequest(payload_confirmed=True), x_aavc_cmd="1"))
    assert exc.value.status_code == 409
    assert "FORCE" in str(exc.value.detail)
    assert not state.preflight_resume_event.is_set()

    res = asyncio.run(eps["/api/cmd/preflight/go"](
        PreflightGoRequest(payload_confirmed=True, force=True), x_aavc_cmd="1"))
    assert res["ok"] is True and res["assigned_marker_id"] == 3
    assert state.preflight_resume_event.is_set()


# ── after a pilot takeover the console must stop flying the aircraft ────────
#
# The 2026-08-21 zombie-re-arm fix put ``_guard_pilot`` on every
# DroneCommander method that changes FC state, and stopped there: these
# handlers call ``commander.system.action.*`` DIRECTLY, so a console click
# still reached a pilot-owned aircraft. Left out of that fix's scope on
# purpose, carried as an open item, closed here.
#
# The split matters as much as the guard. ``kill`` and ``vehicle_disarm`` SAFE
# the aircraft — refusing those after a takeover would be the wrong failure
# direction, and the safety pilot is exactly who might need them.

_FLYING_VERBS = ["takeoff", "hold", "resume", "rtl", "land", "vehicle_arm"]


@pytest.mark.parametrize("verb", _FLYING_VERBS)
def test_a_pilot_owned_aircraft_refuses_every_flying_command(verb: str) -> None:
    cmd = RecordingCommander(pilot_in_control=True)
    eps = _endpoints(_state(), cmd, _armed())
    route = f"/api/cmd/{verb}"
    if route not in eps:
        pytest.skip(f"{route} not mounted")
    with pytest.raises(HTTPException) as exc:
        asyncio.run(_call(eps[route], CommandRequest()))
    assert exc.value.status_code == 409
    assert cmd.calls == [], f"{verb} reached the FC after a takeover"


@pytest.mark.parametrize("verb, expect", [("kill", "abort"),
                                          ("vehicle_disarm", "action.disarm")])
def test_safing_commands_still_work_after_a_takeover(verb: str, expect: str) -> None:
    """The pilot may have taken over BECAUSE something is wrong. Do not take
    the kill switch away from them."""
    cmd = RecordingCommander(pilot_in_control=True)
    eps = _endpoints(_state(), cmd, _armed())
    asyncio.run(_call(eps[f"/api/cmd/{verb}"], CommandRequest()))
    assert cmd.calls == [expect]


def test_nothing_changes_while_the_pilot_has_not_taken_over() -> None:
    """The guard reads a latch that is False for the whole normal mission —
    pin that it costs the ordinary path nothing."""
    cmd = RecordingCommander()
    eps = _endpoints(_state(), cmd, _armed())
    asyncio.run(_call(eps["/api/cmd/vehicle_arm"], CommandRequest()))
    assert cmd.calls == ["action.arm"]
