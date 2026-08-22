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
        self.stood_down = 0

    async def rth(self) -> None:
        self.rth_calls += 1

    async def land(self, *, disarm: bool = True) -> None:
        self.land_calls += 1

    def stand_down(self) -> None:
        self.stood_down += 1


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


def test_gps_sustained_loss_lands_in_place() -> None:
    """LAND, not RTH (operator 2026-08-17): with no flow module, no GPS means
    no horizontal estimate — an RTL could not navigate home anyway. LAND is
    the one response that still works without a position."""
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
    assert state.terminal == TerminalState.ABORTED
    assert cmd.land_calls == 1 and cmd.rth_calls == 0


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


def test_not_armed_skips_flight_triggers_but_not_the_takeover_family() -> None:
    """The armed gate still silences battery/GPS/geofence on the ground — but
    it now sits BELOW the takeover/disarm detectors. Until 2026-08-21 it sat
    ABOVE them, which is what blinded the watchdog the moment the G7 pilot
    disarmed (this test's previous body pinned that defect as correct)."""
    t = _flying_telemetry()
    t.is_armed = False
    t.battery_percent = 5.0            # would abort if armed
    t.flight_mode = "LAND"             # parked posture, AUTO mode
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.LAND
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.RUNNING
    assert cmd.rth_calls == 0 and cmd.land_calls == 0 and cmd.stood_down == 0


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
    # The command OWNER is latched too, not just the mission loop: a command
    # queued after the in-progress one must not still reach the FC.
    assert cmd.stood_down == 1
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


# ── flight-phase disarm + the reordered takeover check (G7 2026-08-21) ──────
# The field takeover is "flip POSCTL, then DISARM within ~0.5 s" — measured
# 0.46-0.48 s of ARMED+POSCTL in both incident ULogs, under the 1.0 s
# debounce. The old ordering (armed gate ABOVE the takeover check) then went
# permanently blind and the mission loop re-armed the parked aircraft.


def test_disarm_mid_flight_phase_fires_immediately_without_debounce() -> None:
    """A disarm in a phase that is armed by design is definitive: one tick,
    no debounce, stand down — this is what stops the zombie re-arm."""
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.SEARCH
    t.is_armed = False                     # the takeover disarm
    t.flight_mode = "POSCTL"               # mode as measured in the ULog
    asyncio.run(_check_and_settle(wd))     # ONE tick
    assert state.terminal == TerminalState.PILOT_TAKEOVER
    assert cmd.stood_down == 1
    assert cmd.rth_calls == 0 and cmd.land_calls == 0
    assert any("disarm_in_flight_phase" in a for a in state.anomalies)


def test_disarm_on_the_pad_in_drop_phase_fires() -> None:
    """DROP keeps the vehicle armed by design (COM_DISARM_LAND=-1; re-arming
    over the field is forbidden) — a disarm there must end the mission, not
    leave the loop free to re-arm between deliveries."""
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.DROP
    t.is_armed = False
    t.relative_alt_m = 0.0
    asyncio.run(_check_and_settle(wd))
    assert state.terminal == TerminalState.PILOT_TAKEOVER
    assert cmd.stood_down == 1


def test_disarm_in_ground_legitimate_phases_does_not_fire() -> None:
    """Disarm in PREFLIGHT/TAKEOFF/LAND/RTH/ABORT is the design working: the
    pre-arm hold, the arm happening DURING the TAKEOFF phase, the multi-flight
    landing, and the watchdog's own terminal actions."""
    for phase in (MissionPhase.PREFLIGHT, MissionPhase.TAKEOFF,
                  MissionPhase.LAND, MissionPhase.RTH, MissionPhase.ABORT):
        t = _flying_telemetry()
        t.is_armed = False
        t.flight_mode = "LAND"
        wd, state, cmd = _make_wd(t)
        state.phase = phase
        asyncio.run(_check_and_settle(wd))
        assert state.terminal == TerminalState.RUNNING, phase
        assert cmd.stood_down == 0, phase


def test_boot_and_between_flight_posctl_postures_do_not_fire() -> None:
    """A1 (design review 2026-08-21): at boot state.phase still holds its
    TAKEOFF default while the RC transmitter sits in POSCTL, and between
    flights the pilot re-stages in POSCTL during the LAND→PREFLIGHT gap.
    Neither may read as a takeover — without this gate the reorder would have
    killed every hardware run at startup."""
    for phase in (MissionPhase.TAKEOFF, MissionPhase.LAND):
        t = _flying_telemetry()
        t.is_armed = False
        t.flight_mode = "POSCTL"
        wd, state, cmd = _make_wd(t)
        wd.pilot_takeover_threshold_s = 0.0

        async def _many(w: SafetyWatchdog) -> None:
            for _ in range(3):
                await _check_and_settle(w)
        asyncio.run(_many(wd))
        assert state.terminal == TerminalState.RUNNING, phase
        assert cmd.stood_down == 0, phase


def test_sub_debounce_disarm_still_stands_down_via_the_disarm_detector() -> None:
    """The incident shape end-to-end, default 1.0 s debounce kept: armed+POSCTL
    for ONE tick (under the debounce), then the disarm lands. The old code
    went blind at the disarm; now the disarm itself is the trigger — and when
    both conditions hold on the same tick, the disarm detector wins (pinned:
    its anomaly kind, not the mode's, is recorded)."""
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    state.phase = MissionPhase.SEARCH
    t.flight_mode = "POSCTL"

    async def _scenario() -> None:
        await _check_and_settle(wd)        # armed tick: debounce only STARTS
        assert state.terminal == TerminalState.RUNNING
        t.is_armed = False                 # disarm ~0.5 s after the flip
        await _check_and_settle(wd)
    asyncio.run(_scenario())
    assert state.terminal == TerminalState.PILOT_TAKEOVER
    assert cmd.stood_down == 1
    assert any("disarm_in_flight_phase" in a for a in state.anomalies)
    assert not any("pilot_takeover_posctl" in a for a in state.anomalies)


def test_takeover_fires_once_even_while_a_terminal_action_settles() -> None:
    """A2 (design review 2026-08-21): while a prior RTH action is still
    settling, _run keeps ticking — without the fire-once latch every 0.5 s
    tick re-wrote the audit trail (record_audit does not dedupe) and
    re-overwrote the terminal. The FIRST fire may overwrite LANDED_RTH (a
    pilot rescuing a watchdog RTL owns the aircraft); later ticks stay
    silent."""
    t = _flying_telemetry()
    wd, state, cmd = _make_wd(t)
    wd.pilot_takeover_threshold_s = 0.0
    state.set_terminal(TerminalState.LANDED_RTH, MissionPhase.RTH)
    wd._terminal_action = "rth"            # a watchdog RTH is in progress
    t.flight_mode = "POSCTL"               # pilot takes over mid-RTL

    async def _scenario() -> None:
        hold = asyncio.Event()
        task = asyncio.create_task(hold.wait())
        wd._action_tasks.add(task)         # …and its action has not settled
        await wd._check_once()             # starts the (collapsed) debounce
        await wd._check_once()             # fires: overwrites LANDED_RTH
        await wd._check_once()             # latched: silent
        await wd._check_once()
        hold.set()
        await task
    asyncio.run(_scenario())
    assert state.terminal == TerminalState.PILOT_TAKEOVER
    assert cmd.stood_down == 1
    assert len([a for a in state.anomalies if "PILOT TAKEOVER" in a]) == 1


# ── battery debounce (no motor-current sensing: PM03D failed; the PM02D that
#    replaced it powers the FC alone) ─────────────────────────────────────────


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



# ── teardown must not outrun the terminal action (2026-08-21 review) ─────────
# _trigger_rth/_trigger_abort dispatch commander.rth()/land() as BACKGROUND
# tasks, and those coroutines are what finally send the explicit disarm
# (COM_DISARM_LAND=-1 means PX4 will not do it). The mission loop exits ~2 s
# after the terminal flips, so a stop() that only cancelled the tick loop let
# main reach commander.close() mid-RTL: PX4 landed, nothing disarmed, and the
# aircraft sat on the ground ARMED with the companion dead.


def test_stop_waits_for_an_in_flight_terminal_action() -> None:
    t = _flying_telemetry()
    t.battery_percent = 25.0                 # low -> RTH
    wd, state, cmd = _make_wd(t)
    landed = []

    async def slow_rth() -> None:
        await asyncio.sleep(0.2)             # stands in for the RTL + disarm
        landed.append("disarmed")

    cmd.rth = slow_rth                       # type: ignore[assignment]

    async def run() -> None:
        await wd._check_once()               # dispatches rth() in the background
        assert not landed, "rth should still be in flight"
        await wd.stop()

    asyncio.run(run())
    assert landed == ["disarmed"], "stop() returned before the disarm"


def test_stop_gives_up_on_a_wedged_action_rather_than_hanging() -> None:
    """The wait is a backstop above rth()'s own 180 s landing wait — it must
    never become the thing that hangs a shutdown."""
    t = _flying_telemetry()
    t.battery_percent = 25.0
    wd, state, cmd = _make_wd(t)

    async def never_returns() -> None:
        await asyncio.sleep(3600)

    cmd.rth = never_returns                  # type: ignore[assignment]

    async def run() -> None:
        await wd._check_once()
        await wd.stop(action_timeout_s=0.05)  # returns instead of hanging

    asyncio.run(run())
