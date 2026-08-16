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
import time

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


def _make_wd(t: CurrentTelemetry,
             **kw) -> tuple[SafetyWatchdog, OrchestratorState, _FakeCommander]:
    """Watchdog under test. `battery_sustain_s` defaults to 0 here so a single
    tick still decides: these cases ask "does this threshold act at all", and
    the debounce has its own tests below."""
    state = OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=_dummy_plan(), telemetry=t)
    cmd = _FakeCommander()
    kw.setdefault("battery_sustain_s", 0.0)
    wd = SafetyWatchdog(state, cmd, AIRSPACE, **kw)  # type: ignore[arg-type]
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


# ── pilot takeover (RC-GO conops 2026-08-12) ────────────────────────────────
# A sustained MANUAL mode while armed past PREFLIGHT = the safety pilot took
# the aircraft. The watchdog must set PILOT_TAKEOVER *directly* — terminal
# with NO companion command (rth/land would fight the pilot).


def test_pilot_takeover_posctl_stands_down_without_commands() -> None:
    t = _flying_telemetry()
    t.flight_mode = "POSCTL"
    wd, state, cmd = _make_wd(t)
    wd.pilot_takeover_threshold_s = 0.0     # collapse the debounce for the test
    state.phase = MissionPhase.TRANSIT_INGRESS

    async def _two_ticks() -> None:
        await _check_and_settle(wd)         # tick 1 starts the debounce window
        await _check_and_settle(wd)         # tick 2 crosses it
    asyncio.run(_two_ticks())

    assert state.terminal == TerminalState.PILOT_TAKEOVER
    assert cmd.rth_calls == 0 and cmd.land_calls == 0
    assert any("pilot_takeover_posctl" in a for a in state.anomalies)


def test_pilot_takeover_exempt_during_preflight_rc_go_hold() -> None:
    # RC-GO: the pilot ARMS in POSCTL on the ground while the gate still holds
    # in PREFLIGHT — that exact posture must never read as a takeover.
    t = _flying_telemetry()
    t.flight_mode = "POSCTL"
    wd, state, cmd = _make_wd(t)
    wd.pilot_takeover_threshold_s = 0.0
    state.phase = MissionPhase.PREFLIGHT

    async def _two_ticks() -> None:
        await _check_and_settle(wd)
        await _check_and_settle(wd)
    asyncio.run(_two_ticks())

    assert state.terminal == TerminalState.RUNNING


def test_pilot_takeover_is_debounced_and_resets_on_auto_mode() -> None:
    # One POSCTL sample followed by an AUTO mode = a transient blip, not a
    # takeover; the debounce clock must reset (default 1 s threshold).
    t = _flying_telemetry()
    t.flight_mode = "POSCTL"
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.SEARCH

    async def _blip() -> None:
        await _check_and_settle(wd)         # starts the window
        t.flight_mode = "MISSION"
        await _check_and_settle(wd)         # resets it
        t.flight_mode = "POSCTL"
        await _check_and_settle(wd)         # fresh window — still below 1 s
    asyncio.run(_blip())

    assert state.terminal == TerminalState.RUNNING
    assert wd._manual_mode_since is not None


def test_offboard_and_auto_modes_are_not_a_takeover() -> None:
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    wd.pilot_takeover_threshold_s = 0.0
    state.phase = MissionPhase.LOCALIZE

    async def _ticks() -> None:
        for mode in ("OFFBOARD", "MISSION", "HOLD", "RETURN_TO_LAUNCH", "LAND"):
            t.flight_mode = mode
            await _check_and_settle(wd)
            await _check_and_settle(wd)
    asyncio.run(_ticks())

    assert state.terminal == TerminalState.RUNNING


# ── battery debounce (no current sensing after the PM03D failure) ────────────


def test_battery_sag_under_load_does_not_trigger() -> None:
    """A transient dip must NOT land the aircraft mid-field.

    With the motors on a board the FC cannot sense, PX4's gauge is purely
    voltage-derived and its load compensation is gated on a current it no
    longer has (lib/battery/battery.cpp), so the reading sags whenever the
    motors pull and springs back. One sample cannot tell that from a flat pack.
    """
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t, battery_sustain_s=5.0)

    async def run() -> None:
        t.battery_percent = 12.0          # sag, well under the LAND floor
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.RUNNING
        t.battery_percent = 55.0          # springs back a tick later
        await _check_and_settle(wd)
        t.battery_percent = 11.0          # sags again — timer must have reset
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.RUNNING
        assert cmd.land_calls == 0 and cmd.rth_calls == 0

    asyncio.run(run())


def test_battery_flat_still_lands_once_sustained() -> None:
    """A pack that is genuinely empty never comes back up, so it still acts."""
    t = _flying_telemetry()
    t.battery_percent = 12.0
    wd, state, cmd = _make_wd(t, battery_sustain_s=0.05)

    async def run() -> None:
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.RUNNING   # armed, not fired
        await asyncio.sleep(0.06)
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.ABORTED
        assert cmd.land_calls == 1
        assert any("battery_critical" in a for a in state.anomalies)

    asyncio.run(run())


def test_battery_rth_debounce_is_independent_of_land() -> None:
    """A reading between the two thresholds arms RTH only, and still fires."""
    t = _flying_telemetry()
    t.battery_percent = 25.0             # < rth (30), >= land floor (20)
    wd, state, cmd = _make_wd(t, battery_sustain_s=0.05)

    async def run() -> None:
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.RUNNING
        await asyncio.sleep(0.06)
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.LANDED_RTH
        assert cmd.rth_calls == 1 and cmd.land_calls == 0

    asyncio.run(run())


# ── motor health: one rotor out (2026-08-16) ────────────────────────────────
#
# The FC layer (CA_FAILURE_MODE=1 + COM_ACT_FAIL_ACT=2) is the primary; this
# watchdog check is the backstop for FD_ACT_EN having been switched off, for
# FD's own thresholds never having been fitted to this power train, and for
# leaving an audit trail the FC's detector does not. (It is NOT, as this
# comment first claimed, a backstop for "reset without a reboot" — FD_ACT_EN
# applies live and its default is on; see safety.py::_check_motor_health.)
# Every test below is really asking the same question in two
# directions: does it act on a real motor failure, and does it stay silent on
# everything that merely LOOKS like one?


def _esc(currents: list[float]) -> list[float]:
    """Build an 8-slot ESC_STATUS-shaped list (PX4 reserves 8; only the first
    6 are rotors on this airframe)."""
    return list(currents) + [0.0] * (8 - len(currents))


def _flying_with_escs(currents: list[float]) -> CurrentTelemetry:
    t = _flying_telemetry()
    t.esc_current_a = _esc(currents)
    t.esc_monotonic = time.monotonic()
    return t


def test_healthy_six_motor_current_does_nothing() -> None:
    t = _flying_with_escs([7.0, 7.2, 6.8, 7.1, 6.9, 7.0])
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0 and cmd.rth_calls == 0


def test_one_dead_motor_sustained_lands_in_place() -> None:
    """Motor 3 pulling nothing while the other five hover at ~7 A."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 0.05, 6.9, 7.0])
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.05)

    async def run() -> None:
        await _check_and_settle(wd)
        assert state.terminal == TerminalState.RUNNING     # debounce armed only
        await asyncio.sleep(0.06)
        t.esc_monotonic = time.monotonic()                  # data still live
        await _check_and_settle(wd)

    asyncio.run(run())
    assert state.terminal == TerminalState.ABORTED
    assert cmd.land_calls == 1 and cmd.rth_calls == 0       # LAND, never RTH
    assert any("motor_failure_3_sustained" in a for a in state.anomalies)


def test_one_dead_motor_recovering_resets_the_debounce() -> None:
    """A single dropped ESC sample must not land a healthy aircraft."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 0.0, 6.9, 7.0])
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=5.0)

    async def run() -> None:
        await _check_and_settle(wd)
        assert wd._motor_fail_since is not None
        t.esc_current_a = _esc([7.0, 7.2, 6.8, 7.0, 6.9, 7.0])
        t.esc_monotonic = time.monotonic()
        await _check_and_settle(wd)
        assert wd._motor_fail_since is None

    asyncio.run(run())
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0


def test_two_quiet_motors_are_treated_as_a_telemetry_fault() -> None:
    """ESC_STATUS carries four channels per message, so a dropped block zeroes
    a PAIR. Two motor failures and one lost packet look identical from here —
    and only one of them is fixed by landing, so this must not act."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 7.1, 0.0, 0.0])
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0
    assert any("esc_current_implausible" in a for a in state.anomalies)


def test_idle_on_the_ground_never_reads_as_a_motor_failure() -> None:
    """Armed on a pad between deliveries (COM_DISARM_LAND=-1 keeps it armed):
    every ESC idles near zero, so the median is below the flight threshold and
    nothing here is evidence of anything."""
    t = _flying_with_escs([0.4, 0.4, 0.0, 0.4, 0.4, 0.4])
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0


def test_absent_esc_telemetry_is_blind_not_fatal() -> None:
    """SITL sends no ESC_STATUS at all, and the real aircraft's ESC telemetry
    lead is unverified until G5. Both must fly, loudly rather than silently."""
    t = _flying_telemetry()                       # esc_current_a stays []
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0
    assert any("motor_health_blind_no_esc_telemetry" in a for a in state.anomalies)


def test_stale_esc_telemetry_does_not_act() -> None:
    """The ESC lists are overwritten in place and never cleared, so a raw
    listener that dies leaves the last good frame looking live forever."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 0.0, 6.9, 7.0])
    t.esc_monotonic = time.monotonic() - 30.0
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0, esc_stale_s=2.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0
    assert any("motor_health_blind_esc_stale" in a for a in state.anomalies)


def test_motor_count_zero_disables_the_check() -> None:
    t = _flying_with_escs([7.0, 7.2, 6.8, 0.0, 6.9, 7.0])
    wd, state, cmd = _make_wd(t, motor_count=0, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0


def test_unused_esc_slots_do_not_count_as_dead_motors() -> None:
    """PX4 reserves 8 ESC slots; slots 6-7 read 0.0 on a hexa forever. Slicing
    to motor_count is the whole reason this passes."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 7.1, 6.9, 7.0])
    assert t.esc_current_a[6] == 0.0 and t.esc_current_a[7] == 0.0
    wd, state, cmd = _make_wd(t, motor_count=6, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.land_calls == 0
    assert not any("motor" in a for a in state.anomalies)


def test_motor_failure_is_checked_before_battery() -> None:
    """A dead rotor with a low pack must land (motor), not RTH (battery) —
    ordering inside _check_once, not two independent rules."""
    t = _flying_with_escs([7.0, 7.2, 6.8, 0.0, 6.9, 7.0])
    t.battery_percent = 25.0                       # under the RTH threshold
    wd, state, cmd = _make_wd(t, motor_fail_sustain_s=0.0)
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.ABORTED
    assert cmd.land_calls == 1 and cmd.rth_calls == 0
