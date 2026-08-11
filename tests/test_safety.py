"""Unit tests for the safety watchdog (orchestrator.safety) — the mission's
only hard-stop, previously untested.

Covers the pure geometry helpers and each ``_check_once`` trigger: battery
LAND/RTH, GPS sustained-loss (+ recovery reset), geofence breach, the
time-budget floor, the NaN guards, the not-armed gate, and the RTH→LAND
escalation. No SITL — a fake commander records rth()/land() calls; async
triggers are driven via ``asyncio.run`` (the project has no pytest-asyncio).
"""

from __future__ import annotations

import asyncio
import math

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.schemas import (
    CommandKind,
    Coordinate,
    MissionCommand,
    MissionPhase,
    MissionPlan,
)
from orchestrator.safety import (
    SafetyWatchdog,
    _distance_to_polygon_edge,
    _haversine_m,
    _point_in_polygon,
)
from orchestrator.state import OrchestratorMode, OrchestratorState, TerminalState

HOME_LAT, HOME_LON = 14.6525, 101.1875
# A geofence square comfortably containing HOME (~±110 m each way).
AIRSPACE = [
    [14.6515, 101.1865],
    [14.6515, 101.1885],
    [14.6535, 101.1885],
    [14.6535, 101.1865],
]


# ── pure geometry helpers ───────────────────────────────────────────────────


def test_point_in_polygon_inside_and_outside() -> None:
    assert _point_in_polygon(HOME_LAT, HOME_LON, AIRSPACE) is True
    assert _point_in_polygon(14.6600, HOME_LON, AIRSPACE) is False   # north
    assert _point_in_polygon(HOME_LAT, 101.2000, AIRSPACE) is False  # east


def test_point_in_polygon_on_vertex_does_not_crash() -> None:
    # Exercising the boundary must not raise (ray-cast tie-break is fine either way).
    _point_in_polygon(14.6515, 101.1865, AIRSPACE)


def test_haversine_zero_and_symmetry() -> None:
    assert _haversine_m(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON) == 0.0
    d1 = _haversine_m(HOME_LAT, HOME_LON, HOME_LAT + 0.001, HOME_LON)
    d2 = _haversine_m(HOME_LAT + 0.001, HOME_LON, HOME_LAT, HOME_LON)
    assert math.isclose(d1, d2, rel_tol=1e-9)
    assert 100.0 < d1 < 120.0          # 0.001° latitude ≈ 111 m


def test_distance_to_polygon_edge_degenerate_is_inf() -> None:
    assert _distance_to_polygon_edge(HOME_LAT, HOME_LON, [[14.0, 101.0]]) == float("inf")


def test_distance_to_polygon_edge_centre_is_finite_positive() -> None:
    d = _distance_to_polygon_edge(HOME_LAT, HOME_LON, AIRSPACE)
    assert d > 0.0 and math.isfinite(d)


# ── watchdog trigger fixtures ───────────────────────────────────────────────


class _FakeCommander:
    """Records terminal-action calls; both are async no-ops."""

    def __init__(self) -> None:
        self.rth_calls = 0
        self.land_calls = 0

    async def rth(self) -> None:
        self.rth_calls += 1

    async def land(self, *, disarm: bool = True) -> None:
        self.land_calls += 1


def _dummy_plan() -> MissionPlan:
    # The watchdog never indexes the plan; this is the minimal VALID plan
    # (MissionPlan requires >= 4 commands), mirroring test_preflight.
    coord = Coordinate(lat=HOME_LAT, lon=HOME_LON, alt_m=16.0)
    cmds = [
        MissionCommand(seq=0, kind=CommandKind.TAKEOFF, phase=MissionPhase.TAKEOFF,
                       coord=coord, altitude_m=16.0),
        MissionCommand(seq=1, kind=CommandKind.GOTO, phase=MissionPhase.LOCALIZE,
                       coord=coord, altitude_m=16.0, stop_index=0),
        MissionCommand(seq=2, kind=CommandKind.DROP_PAYLOAD, phase=MissionPhase.DROP,
                       coord=coord, payload_id=0, stop_index=0),
        MissionCommand(seq=3, kind=CommandKind.RTH, phase=MissionPhase.RTH, coord=coord),
    ]
    return MissionPlan(
        mission_id="safety-test", expected_duration_s=100.0, commands=cmds,
        target_group_strategy="x", fallback_strategy="y",
    )


def _flying_telemetry() -> CurrentTelemetry:
    """Healthy, armed, in-flight, inside the geofence — the all-clear baseline."""
    t = CurrentTelemetry()
    t.is_connected = True
    t.is_armed = True
    t.battery_percent = 80.0
    t.gps_fix_type = 3
    t.datalink_rssi = -1            # SITL sentinel: link check not evaluated
    t.lat, t.lon = HOME_LAT, HOME_LON
    t.relative_alt_m = 12.0
    return t


def _make_wd(t: CurrentTelemetry) -> tuple[SafetyWatchdog, OrchestratorState, _FakeCommander]:
    state = OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=_dummy_plan(), telemetry=t)
    cmd = _FakeCommander()
    wd = SafetyWatchdog(state, cmd, AIRSPACE)  # type: ignore[arg-type]
    return wd, state, cmd


async def _check_and_settle(wd: SafetyWatchdog) -> None:
    """Run one watchdog tick and await any terminal action it dispatched (the
    rth/land run as background tasks via _spawn_action)."""
    await wd._check_once()
    tasks = list(wd._action_tasks)
    if tasks:
        await asyncio.gather(*tasks, return_exceptions=True)


# ── battery ─────────────────────────────────────────────────────────────────


def test_battery_critical_triggers_abort_land() -> None:
    t = _flying_telemetry()
    t.battery_percent = 15.0           # < land floor (20)
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.ABORTED
    assert cmd.land_calls == 1 and cmd.rth_calls == 0
    assert any("battery_critical" in a for a in state.anomalies)


def test_battery_low_triggers_rth() -> None:
    t = _flying_telemetry()
    t.battery_percent = 25.0           # < rth threshold (30), >= land floor
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.LANDED_RTH
    assert cmd.rth_calls == 1 and cmd.land_calls == 0
    assert any("battery_low" in a for a in state.anomalies)


def test_battery_nan_records_anomaly_but_does_not_trigger() -> None:
    t = _flying_telemetry()
    t.battery_percent = math.nan       # sensor dropout / pre-first-frame
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0 and cmd.land_calls == 0
    assert any("battery_telemetry_nan" in a for a in state.anomalies)


def test_battery_nan_sustained_escalates_but_never_rths() -> None:
    """A NaN battery stream that persists past the debounce escalates to a
    distinct anomaly (so the operator sees battery protection is blind) but
    deliberately does NOT auto-RTH — companion NaN is indistinguishable from a
    telemetry-plumbing fault; the FC battery failsafe is the authoritative layer."""
    t = _flying_telemetry()
    t.battery_percent = math.nan
    wd, state, cmd = _make_wd(t)

    async def run() -> None:
        await wd._check_once()             # first tick: record + start the timer
        assert wd._battery_nan_since is not None
        assert not any("nan_sustained" in a for a in state.anomalies)
        wd._battery_nan_since = (
            asyncio.get_running_loop().time() - (wd.battery_nan_threshold_s + 1.0)
        )
        await _check_and_settle(wd)

    asyncio.run(run())
    assert state.terminal == TerminalState.RUNNING          # NEVER auto-RTH on NaN
    assert cmd.rth_calls == 0 and cmd.land_calls == 0
    assert any("battery_telemetry_nan_sustained" in a for a in state.anomalies)


def test_battery_recovery_resets_the_nan_timer() -> None:
    t = _flying_telemetry()
    t.battery_percent = math.nan
    wd, state, cmd = _make_wd(t)

    async def run() -> None:
        await wd._check_once()
        assert wd._battery_nan_since is not None
        t.battery_percent = 80.0                # a real reading arrives
        await wd._check_once()
        assert wd._battery_nan_since is None     # timer reset

    asyncio.run(run())
    assert state.terminal == TerminalState.RUNNING


# ── GPS ─────────────────────────────────────────────────────────────────────


def test_gps_sustained_loss_triggers_rth() -> None:
    t = _flying_telemetry()
    t.gps_fix_type = 0                 # no fix
    wd, state, cmd = _make_wd(t)

    async def run() -> None:
        await wd._check_once()         # first tick: record the loss, debounced
        assert state.terminal == TerminalState.RUNNING
        # Backdate the loss past the debounce threshold → next tick must trigger.
        wd._gps_lost_since = (
            asyncio.get_running_loop().time() - (wd.gps_loss_threshold_s + 1.0)
        )
        await _check_and_settle(wd)

    asyncio.run(run())
    assert state.terminal == TerminalState.LANDED_RTH
    assert cmd.rth_calls == 1


def test_gps_recovery_resets_the_timer() -> None:
    t = _flying_telemetry()
    t.gps_fix_type = 0
    wd, state, cmd = _make_wd(t)

    async def run() -> None:
        await wd._check_once()
        assert wd._gps_lost_since is not None   # loss recorded
        t.gps_fix_type = 3                       # recovered before the threshold
        await wd._check_once()
        assert wd._gps_lost_since is None        # timer reset, no accumulation

    asyncio.run(run())
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0


# ── geofence / time / gates ─────────────────────────────────────────────────


def test_geofence_breach_triggers_rth() -> None:
    t = _flying_telemetry()
    t.lat = 14.6600                    # north of the box → outside
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.LANDED_RTH
    assert cmd.rth_calls == 1
    assert any("geofence_breach" in a for a in state.anomalies)


def test_nan_position_does_not_crash_geofence() -> None:
    t = _flying_telemetry()
    t.lat = math.nan
    t.lon = math.nan
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))   # the geofence check must skip, not raise
    assert state.terminal == TerminalState.RUNNING


def test_time_budget_exhausted_triggers_rth() -> None:
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    state.operation_window_s = 10.0    # remaining ≈ 10 s < the 180 s floor
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.LANDED_RTH
    assert any("time_budget_exhausted" in a for a in state.anomalies)


def test_time_budget_exhausted_does_not_rth_during_drop() -> None:
    """M1 (review 2026-07-24): tactical_align enters MissionPhase.DROP AFTER
    the landing (release, plus the post-delivery climb-out/pre-egress goto
    that follow it under the same leftover phase) — the vehicle is on the
    ground with an egg half-released, so forcing an RTH there is never the
    right answer. DROP must be exempt exactly like TRANSIT_EGRESS/LAND/RTH."""
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.DROP
    state.operation_window_s = 10.0    # remaining ≈ 10 s < the 180 s floor
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0
    assert not any("time_budget_exhausted" in a for a in state.anomalies)


def test_not_armed_skips_all_triggers() -> None:
    t = _flying_telemetry()
    t.is_armed = False
    t.battery_percent = 5.0            # would abort if armed
    wd, state, cmd = _make_wd(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0 and cmd.land_calls == 0


def test_tuning_mode_skips_geofence_and_time() -> None:
    # enforce_mission_limits=False (the System-ID/Autotune tool) must not RTH on
    # a geofence breach or the time floor — it flies the drone itself.
    t = _flying_telemetry()
    t.lat = 14.6600                    # outside the box
    state = OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=_dummy_plan(), telemetry=t)
    cmd = _FakeCommander()
    wd = SafetyWatchdog(state, cmd, AIRSPACE, enforce_mission_limits=False)  # type: ignore[arg-type]
    state.operation_window_s = 10.0
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0


# ── escalation ──────────────────────────────────────────────────────────────


def test_rth_escalates_to_land_on_worsening_battery() -> None:
    t = _flying_telemetry()
    t.battery_percent = 25.0           # low → RTH
    wd, state, cmd = _make_wd(t)

    async def run() -> None:
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.LANDED_RTH and cmd.rth_calls == 1
        t.battery_percent = 15.0       # now critical mid-RTH → escalate to LAND
        await _check_and_settle(wd)

    asyncio.run(run())
    assert state.terminal == TerminalState.ABORTED
    assert cmd.land_calls == 1


# ── V1.3 airspace rules: no-fly zone, altitude ceiling, search floor ────────

NFZ = [[14.6520, 101.1870], [14.6520, 101.1875],
       [14.6528, 101.1875], [14.6528, 101.1870]]   # inside the airspace, SW


def _make_wd_v11(
    t: CurrentTelemetry, **kw,
) -> tuple[SafetyWatchdog, OrchestratorState, _FakeCommander]:
    state = OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=_dummy_plan(), telemetry=t)
    cmd = _FakeCommander()
    wd = SafetyWatchdog(state, cmd, AIRSPACE, no_fly_zones=[NFZ],  # type: ignore[arg-type]
                        altitude_ceiling_m=20.0, search_floor_m=10.0, **kw)
    return wd, state, cmd


def test_no_fly_zone_entry_triggers_rth() -> None:
    t = _flying_telemetry()
    t.lat, t.lon = 14.6524, 101.18725          # inside NFZ, inside airspace
    wd, state, cmd = _make_wd_v11(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.LANDED_RTH
    assert cmd.rth_calls == 1
    assert any("no_fly_zone_breach" in a for a in state.anomalies)


def test_ceiling_warn_is_anomaly_only() -> None:
    t = _flying_telemetry()
    t.relative_alt_m = 20.8                    # > ceiling+warn, < ceiling+breach
    wd, state, cmd = _make_wd_v11(t)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0
    assert any("altitude_ceiling_warn" in a for a in state.anomalies)


def test_ceiling_breach_sustained_triggers_rth() -> None:
    t = _flying_telemetry()
    t.relative_alt_m = 23.0                    # > ceiling+breach (22)
    wd, state, cmd = _make_wd_v11(t, ceiling_breach_threshold_s=0.0)

    async def _two_ticks() -> None:
        await wd._check_once()                 # starts the debounce
        await asyncio.sleep(0.01)
        await _check_and_settle(wd)            # sustained → RTH

    asyncio.run(_two_ticks())
    assert state.terminal == TerminalState.LANDED_RTH
    assert cmd.rth_calls == 1
    assert any("altitude_ceiling_breach_sustained" in a for a in state.anomalies)


def test_ceiling_breach_recovers_when_alt_drops() -> None:
    t = _flying_telemetry()
    t.relative_alt_m = 23.0
    wd, state, cmd = _make_wd_v11(t, ceiling_breach_threshold_s=60.0)

    async def _breach_then_recover() -> None:
        await wd._check_once()                 # debounce armed, no action yet
        t.relative_alt_m = 19.0                # back under the ceiling
        await wd._check_once()
        assert wd._ceiling_breach_since is None

    asyncio.run(_breach_then_recover())
    assert state.terminal == TerminalState.RUNNING and cmd.rth_calls == 0


def test_below_floor_in_search_is_advisory_only() -> None:
    t = _flying_telemetry()
    t.relative_alt_m = 7.0
    wd, state, cmd = _make_wd_v11(t)
    state.phase = MissionPhase.SEARCH
    asyncio.run(_check_and_settle(wd))
    assert any("below_search_floor" in a for a in state.anomalies)
    assert state.terminal == TerminalState.RUNNING and cmd.rth_calls == 0


def test_below_floor_during_delivery_descent_is_legal() -> None:
    t = _flying_telemetry()
    t.relative_alt_m = 4.0
    wd, state, cmd = _make_wd_v11(t)
    state.phase = MissionPhase.LOCALIZE        # the rules' sub-floor carve-out
    asyncio.run(_check_and_settle(wd))
    assert not any("below_search_floor" in a for a in state.anomalies)
    assert state.terminal == TerminalState.RUNNING
