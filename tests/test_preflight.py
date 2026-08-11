"""Unit tests for the pre-flight readiness gate (orchestrator.preflight).

`run_preflight` is a pure snapshot evaluator, so these build a real
OrchestratorState with a healthy telemetry + plan and assert the board flips an
individual CRITICAL item to fail (and `all_critical_pass` with it) when a single
signal degrades. Advisory items never gate.
"""

import time

import pytest

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import (
    CommandKind,
    Coordinate,
    MissionCommand,
    MissionPhase,
    MissionPlan,
)
from mission_brain.search_pattern import build_search_pattern
from orchestrator.preflight import run_preflight
from orchestrator.state import OrchestratorMode, OrchestratorState

HOME_LAT, HOME_LON = 14.6525, 101.1875
# A geofence square comfortably containing HOME.
GEOFENCE = [
    [14.6515, 101.1865],
    [14.6515, 101.1885],
    [14.6535, 101.1885],
    [14.6535, 101.1865],
]


def _healthy_telemetry() -> CurrentTelemetry:
    t = CurrentTelemetry()
    t.is_connected = True
    t.is_armable = True
    t.is_global_position_ok = True
    t.is_local_position_ok = True
    t.is_home_position_ok = True
    t.is_gyrometer_calibrated = True
    t.is_accelerometer_calibrated = True
    t.is_magnetometer_calibrated = True
    t.gps_fix_type = 3
    t.gps_satellites = 12
    t.battery_percent = 95.0
    t.is_armed = False
    t.relative_alt_m = 0.0
    t.datalink_rssi = -1   # SITL: advisory warn, never blocks
    t.lat, t.lon = HOME_LAT, HOME_LON
    return t


def _plan_with_targets() -> MissionPlan:
    """A healthy unknown-pad sortie plan: TAKEOFF + boustrophedon sweep GOTOs
    + LAND. Blind search has no pre-loaded drops; the gate checks the sweep."""
    home = Coordinate(lat=HOME_LAT, lon=HOME_LON)
    spec = build_search_pattern(GEOFENCE, home, sweep_alt_m=16.0)
    return render_live_plan(home, spec, discovered=[], profile=COMPETITION)


def _plan_known_pad() -> MissionPlan:
    """A registry-known sortie plan: no search legs, straight to a LOCALIZE
    goto + drop. V1.3 sorties 2..4 usually look like this — launch-worthy."""
    coord = Coordinate(lat=HOME_LAT, lon=HOME_LON, alt_m=16.0)
    cmds = [
        MissionCommand(seq=0, kind=CommandKind.TAKEOFF, phase=MissionPhase.TAKEOFF,
                       coord=coord, altitude_m=16.0),
        MissionCommand(seq=1, kind=CommandKind.GOTO, phase=MissionPhase.LOCALIZE,
                       coord=coord, altitude_m=16.0, stop_index=0),
        MissionCommand(seq=2, kind=CommandKind.DROP_PAYLOAD, phase=MissionPhase.DROP,
                       coord=coord, payload_id=0, stop_index=0),
        MissionCommand(seq=3, kind=CommandKind.LAND, phase=MissionPhase.LAND, coord=coord),
    ]
    return MissionPlan(mission_id="knownpad", expected_duration_s=100.0,
                       commands=cmds, target_group_strategy="x", fallback_strategy="y")


def _plan_without_route() -> MissionPlan:
    # A valid (>= 4 command) plan with NEITHER a search pattern NOR a known-pad
    # LOCALIZE goto — no way to reach the assigned pad; the gate must fail it.
    coord = Coordinate(lat=HOME_LAT, lon=HOME_LON, alt_m=16.0)
    cmds = [
        MissionCommand(seq=0, kind=CommandKind.TAKEOFF, phase=MissionPhase.TAKEOFF,
                       coord=coord, altitude_m=16.0),
        MissionCommand(seq=1, kind=CommandKind.HOVER, phase=MissionPhase.SEARCH,
                       coord=coord, altitude_m=16.0, duration_s=5.0),
        MissionCommand(seq=2, kind=CommandKind.HOVER, phase=MissionPhase.SEARCH,
                       coord=coord, altitude_m=16.0, duration_s=5.0),
        MissionCommand(seq=3, kind=CommandKind.LAND, phase=MissionPhase.LAND, coord=coord),
    ]
    return MissionPlan(mission_id="noroute", expected_duration_s=100.0,
                       commands=cmds, target_group_strategy="x", fallback_strategy="y")


def _state(plan: MissionPlan, telemetry: CurrentTelemetry) -> OrchestratorState:
    st = OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telemetry)
    st.link_connected = True
    return st


def _run(st: OrchestratorState, camera_frame, now_wall):
    return run_preflight(
        st, geofence=GEOFENCE, home_lat=HOME_LAT, home_lon=HOME_LON,
        camera_frame=camera_frame, camera_max_age_s=5.0, now_wall=now_wall,
    )


@pytest.fixture()
def fresh_cam(tmp_path):
    """A camera frame whose mtime we control via the returned (path, now)."""
    p = tmp_path / "aavc_nadir.png"
    p.write_bytes(b"\x89PNG")
    return p, p.stat().st_mtime + 1.0   # now = 1 s after write → fresh


def _item(report, item_id):
    return next(i for i in report.items if i.id == item_id)


def test_all_healthy_board_is_green(fresh_cam):
    cam, now = fresh_cam
    report = _run(_state(_plan_with_targets(), _healthy_telemetry()), cam, now)
    assert report.all_critical_pass
    assert _item(report, "camera").status == "pass"
    # the payload advisory is pending but does NOT block.
    assert _item(report, "payload").status == "pending"
    assert _item(report, "payload").critical is False


def test_low_gps_fails(fresh_cam):
    cam, now = fresh_cam
    t = _healthy_telemetry()
    t.gps_fix_type, t.gps_satellites = 1, 3
    report = _run(_state(_plan_with_targets(), t), cam, now)
    assert not report.all_critical_pass
    assert _item(report, "gps").status == "fail"


def test_low_battery_fails(fresh_cam):
    cam, now = fresh_cam
    t = _healthy_telemetry()
    t.battery_percent = 22.0
    report = _run(_state(_plan_with_targets(), t), cam, now)
    assert not report.all_critical_pass
    assert _item(report, "battery").status == "fail"


def test_home_outside_geofence_fails(fresh_cam):
    cam, now = fresh_cam
    report = run_preflight(
        _state(_plan_with_targets(), _healthy_telemetry()),
        geofence=GEOFENCE, home_lat=14.6600, home_lon=101.1875,  # north of the box
        camera_frame=cam, camera_max_age_s=5.0, now_wall=now,
    )
    assert _item(report, "geofence").status == "fail"
    assert not report.all_critical_pass


def test_stale_camera_fails(fresh_cam):
    cam, _ = fresh_cam
    stale_now = cam.stat().st_mtime + 99.0
    report = _run(_state(_plan_with_targets(), _healthy_telemetry()), cam, stale_now)
    assert _item(report, "camera").status == "fail"
    assert not report.all_critical_pass


def test_missing_camera_fails(tmp_path):
    missing = tmp_path / "nope.png"
    report = _run(_state(_plan_with_targets(), _healthy_telemetry()), missing, time.time())
    assert _item(report, "camera").status == "fail"


def test_plan_with_no_route_to_a_pad_fails(fresh_cam):
    cam, now = fresh_cam
    report = _run(_state(_plan_without_route(), _healthy_telemetry()), cam, now)
    assert _item(report, "search").status == "fail"
    assert not report.all_critical_pass


def test_known_pad_sortie_plan_passes(fresh_cam):
    """A registry-known sortie has NO search legs — still launch-worthy."""
    cam, now = fresh_cam
    report = _run(_state(_plan_known_pad(), _healthy_telemetry()), cam, now)
    assert _item(report, "search").status == "pass"
    assert report.all_critical_pass


def test_armed_or_airborne_fails(fresh_cam):
    cam, now = fresh_cam
    t = _healthy_telemetry()
    t.is_armed = True
    report = _run(_state(_plan_with_targets(), t), cam, now)
    assert _item(report, "on_ground").status == "fail"


def test_not_armable_fails(fresh_cam):
    cam, now = fresh_cam
    t = _healthy_telemetry()
    t.is_armable = False
    report = _run(_state(_plan_with_targets(), t), cam, now)
    assert _item(report, "armable").status == "fail"
    assert not report.all_critical_pass


def test_short_window_is_advisory_not_critical(fresh_cam):
    """The window-too-short refusal belongs to the TimePolicy gate + the GO
    endpoint's `sortie_time_ok || force` — NOT the critical board. When the
    time row was critical, the operator's FORCE checkbox was a dead path
    (found live 2026-07-15: sortie-4 hold at 2:57 remaining → NOT READY and
    /go 409'd on criticals despite force=true). A short window must read WARN/
    advisory; everything else healthy → the board stays green so FORCE works."""
    cam, now = fresh_cam
    st = _state(_plan_with_targets(), _healthy_telemetry())
    st.operation_window_s = 100.0          # remaining ≈100 s < the 180 s floor
    report = _run(st, cam, now)
    time_item = next(i for i in report.items if i.id == "time")
    assert time_item.critical is False
    assert time_item.status == "warn"
    assert report.all_critical_pass is True


def _report_for_healthy_state(tmp_path, **state_attrs):
    """A report from a healthy state, with state attributes overridden.

    Shared with tests/test_energy_policy.py so the energy row is asserted
    against a REAL report rather than by reading the source."""
    cam = tmp_path / "aavc_nadir.png"
    cam.write_bytes(b"\x89PNG")
    st = _state(_plan_with_targets(), _healthy_telemetry())
    for k, v in state_attrs.items():
        setattr(st, k, v)
    return _run(st, cam, cam.stat().st_mtime + 1.0)
