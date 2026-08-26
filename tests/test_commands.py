"""Unit tests for flight-command helpers that guard documented real bugs.

- `_pwm_to_norm`: the old `pwm/2000` mapping left BOTH the release (1900 µs) and
  hold (1100 µs) PWMs in the upper half of travel, so the drop servo may never
  have swung between positions → payload never released. These tests pin the
  bipolar mapping so a revert is caught.
- `_arm_with_retry`: PX4 transiently denies re-arming right after the
  land-detector auto-disarm; without the retry a single denial aborts the whole
  5-target sortie at the climb-out between targets.

Async units are driven with `asyncio.run(...)` from sync tests (no pytest-asyncio
dependency, matching the lean core).
"""
import asyncio

import pytest
from mavsdk.action import ActionError

from mavlink_adapter.commands import (
    DroneCommander,
    PilotInControlError,
    _pwm_to_norm,
)

# ── pilot-in-control latch — the command owner refuses after a takeover ──

def test_stand_down_makes_the_commander_refuse_movement() -> None:
    """RESUME 2026-08-19: the mission loop stops between awaits, but a command
    that STARTS after the in-progress one would still reach the FC. The flag
    belongs on the object that owns the sending — once the pilot has the
    aircraft, EVERY movement command raises instead of commanding."""
    c = DroneCommander.__new__(DroneCommander)     # skip the real MAVSDK System
    assert c._pilot_in_control is False            # class default, safe pre-connect
    c.stand_down()
    assert c._pilot_in_control is True
    for make in (lambda: c.goto(13.7, 100.7, 10.0),
                 lambda: c.arm_and_takeoff(10.0),
                 lambda: c.land(),
                 lambda: c.rth(),
                 lambda: c.drop_payload(0),
                 lambda: c.prime_offboard_hold()):
        with pytest.raises(PilotInControlError):
            asyncio.run(make())


# ── _pwm_to_norm — the drop servo must actually cross centre ──

def test_pwm_to_norm_centre_and_endpoints():
    assert _pwm_to_norm(1500) == 0.0
    assert _pwm_to_norm(1000) == -1.0
    assert _pwm_to_norm(2000) == 1.0


def test_pwm_to_norm_release_and_hold_straddle_centre():
    # The regression this guards: release and hold must be on OPPOSITE sides of 0.
    assert _pwm_to_norm(1900) > 0.0
    assert _pwm_to_norm(1100) < 0.0
    assert _pwm_to_norm(1900) == pytest.approx(0.8)
    assert _pwm_to_norm(1100) == pytest.approx(-0.8)


def test_pwm_to_norm_clamps_out_of_band():
    assert _pwm_to_norm(2500) == 1.0
    assert _pwm_to_norm(500) == -1.0


# ── _arm_with_retry — tolerate the transient post-land re-arm denial ──

class _Denied(ActionError):
    """A stand-in for a COMMAND_DENIED ActionError (the real ctor takes an
    awkward (result, origin) pair we don't need here)."""

    def __init__(self) -> None:  # noqa: D107 — intentionally bypass base ctor
        pass

    def __str__(self) -> str:
        return "COMMAND_DENIED: 'Command Denied'; origin: arm()"


class _FakeAction:
    def __init__(self, fail_times: int) -> None:
        self.calls = 0
        self._fail_times = fail_times

    async def arm(self) -> None:
        self.calls += 1
        if self.calls <= self._fail_times:
            raise _Denied()


def _commander_with(action: _FakeAction) -> DroneCommander:
    # Skip DroneCommander.__init__ (it builds a real MAVSDK System / gRPC).
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"action": action})()  # type: ignore[assignment]
    return c


def test_arm_retries_then_succeeds():
    fake = _FakeAction(fail_times=3)
    asyncio.run(_commander_with(fake)._arm_with_retry(attempts=10, delay_s=0.0))
    assert fake.calls == 4  # 3 denials + 1 success


def test_arm_first_try_no_retry():
    fake = _FakeAction(fail_times=0)
    asyncio.run(_commander_with(fake)._arm_with_retry(attempts=10, delay_s=0.0))
    assert fake.calls == 1


def test_arm_exhausts_and_raises_runtimeerror():
    fake = _FakeAction(fail_times=999)
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with(fake)._arm_with_retry(attempts=4, delay_s=0.0))
    assert fake.calls == 4  # gives up after exactly `attempts`, then raises (not ActionError)


# ── stand_down must cover EVERYTHING that changes FC state (G7 2026-08-21) ──


def test_stand_down_covers_mission_arming_params_and_side_channels() -> None:
    """G7 attempt-1 zombie re-arm: the original guard set left run_mission
    (which ARMS), _arm_with_retry, the raw pymavlink drop fallback, and every
    FC-state setter unguarded — a stood-down commander could still rewrite
    failsafes under the pilot. Every FC-state-changing method now refuses.
    (abort() stays deliberately unguarded — it is the emergency motor kill.)"""
    c = DroneCommander.__new__(DroneCommander)
    c.stand_down()
    for make in (lambda: c.run_mission([object()]),          # arms + flies
                 lambda: c._arm_with_retry(),
                 lambda: c._drop_via_set_actuator(1),        # raw side-channel
                 lambda: c.set_param_float("MPC_XY_CRUISE", 5.0),
                 lambda: c.set_param_int("MIS_TKO_LAND_REQ", 0),
                 lambda: c.apply_param_overrides({"MPC_XY_CRUISE": 5.0}),
                 lambda: c.set_gimbal_mount({"MNT_MODE_IN": 4.0}),
                 lambda: c.set_geofence_action_rtl(),
                 lambda: c.set_datalink_loss_rtl(),
                 lambda: c.set_rc_loss_rtl(),
                 lambda: c.set_battery_failsafe(
                     low=0.2, crit=0.1, emergen=0.05, action=2),
                 lambda: c.upload_geofence(
                     [(13.70, 100.70), (13.71, 100.70), (13.71, 100.71)])):
        with pytest.raises(PilotInControlError):
            asyncio.run(make())


def test_arm_retry_aborts_when_takeover_lands_mid_retry() -> None:
    """A takeover mid-retry (a window of up to 15 s) must abort the REMAINING
    arm attempts — _arm_with_retry is the exact call the G7 zombie hammered
    ten times against the parked aircraft."""
    class _DenyAndTakeover(_FakeAction):
        commander: DroneCommander | None = None

        async def arm(self) -> None:
            self.calls += 1
            assert self.commander is not None
            self.commander.stand_down()    # the takeover lands mid-window
            raise _Denied()

    fake = _DenyAndTakeover(fail_times=999)
    c = _commander_with(fake)
    fake.commander = c
    with pytest.raises(PilotInControlError):
        asyncio.run(c._arm_with_retry(attempts=10, delay_s=0.0))
    assert fake.calls == 1                 # attempt 2's guard refused


def test_arm_and_takeoff_reguards_before_the_takeoff_command() -> None:
    """A3 (design review 2026-08-21): between arm success and
    action.takeoff() sit a settle sleep + a home-alt refresh (1-6 s). A
    takeover in that window must stop the takeoff command itself — the entry
    guard alone cannot."""
    class _Act:
        def __init__(self) -> None:
            self.takeoff_calls = 0

        async def set_takeoff_altitude(self, alt: float) -> None:
            pass

        async def takeoff(self) -> None:
            self.takeoff_calls += 1

    act = _Act()
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"action": act})()  # type: ignore[assignment]

    async def _not_armed() -> bool:
        return False

    async def _arm_ok() -> None:
        pass

    async def _refresh_then_takeover() -> None:
        c.stand_down()                     # pilot takes over during the settle

    c._is_armed = _not_armed                        # type: ignore[method-assign]
    c._arm_with_retry = _arm_ok                     # type: ignore[method-assign]
    c._refresh_home_alt = _refresh_then_takeover    # type: ignore[method-assign]
    with pytest.raises(PilotInControlError):
        asyncio.run(c.arm_and_takeoff(12.0))
    assert act.takeoff_calls == 0


def test_land_does_not_disarm_when_the_pilot_takes_over_mid_descent() -> None:
    """land() guards at entry and then BLOCKS up to 30 s waiting for touchdown.
    A takeover inside that window used to run straight into the unconditional
    action.disarm() on the far side — sending the one command that must never
    reach a flying aircraft, at the exact moment the pilot has just flown it
    out of the descent we asked for."""
    class _Act:
        def __init__(self) -> None:
            self.land_calls = 0
            self.disarm_calls = 0

        async def land(self) -> None:
            self.land_calls += 1

        async def disarm(self) -> None:
            self.disarm_calls += 1

    act = _Act()
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"action": act})()  # type: ignore[assignment]

    async def _land_then_takeover(**kw) -> bool:
        c.stand_down()                     # pilot flips POSCTL during the descent
        return True

    disarm_waits: list[int] = []

    async def _wait_disarmed(**kw) -> bool:
        disarm_waits.append(1)
        return True

    c._wait_until_landed = _land_then_takeover      # type: ignore[method-assign]
    c._wait_until_disarmed = _wait_disarmed         # type: ignore[method-assign]
    asyncio.run(c.land())
    assert act.land_calls == 1                      # the land command DID go out
    assert act.disarm_calls == 0                    # …the disarm tail did not
    assert disarm_waits == []


def test_rth_does_not_disarm_when_the_pilot_takes_over_mid_return() -> None:
    """Same window, 180 s wide: rth() flies home, lands, then disarms."""
    class _Act:
        def __init__(self) -> None:
            self.disarm_calls = 0

        async def return_to_launch(self) -> None:
            pass

        async def disarm(self) -> None:
            self.disarm_calls += 1

    class _Param:
        async def set_param_float(self, name: str, v: float) -> None:
            pass

    act = _Act()
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"action": act, "param": _Param()})()  # type: ignore[assignment]

    async def _land_then_takeover(**kw) -> bool:
        c.stand_down()
        return True

    async def _wait_disarmed(**kw) -> bool:
        return True

    c._wait_until_landed = _land_then_takeover      # type: ignore[method-assign]
    c._wait_until_disarmed = _wait_disarmed         # type: ignore[method-assign]
    asyncio.run(c.rth())
    assert act.disarm_calls == 0


# ── FC failsafe pins: NAV_RCL_ACT / battery thresholds (S4) ──


class _FakeParam:
    """Records PX4 param writes; get_param_int reflects the last set unless a
    readback override is injected (to simulate a value that didn't stick)."""

    def __init__(self, readback: dict[str, int] | None = None,
                 float_readback: dict[str, float] | None = None) -> None:
        self.int_sets: dict[str, int] = {}
        self.float_sets: dict[str, float] = {}
        self._readback = readback or {}
        self._float_readback = float_readback or {}

    async def set_param_int(self, name: str, value: int) -> None:
        self.int_sets[name] = value

    async def get_param_int(self, name: str) -> int:
        return self._readback.get(name, self.int_sets.get(name, 0))

    async def set_param_float(self, name: str, value: float) -> None:
        self.float_sets[name] = value

    async def get_param_float(self, name: str) -> float:
        if name in self._float_readback:
            return self._float_readback[name]
        if name in self.float_sets:
            return self.float_sets[name]
        raise RuntimeError(f"param {name} not found")


def _commander_with_param(param: _FakeParam) -> DroneCommander:
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"param": param})()  # type: ignore[assignment]
    return c


def test_set_rc_loss_rtl_pins_and_reads_back():
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_rc_loss_rtl())
    assert p.int_sets["NAV_RCL_ACT"] == 2       # 2 = Return
    assert p.int_sets["COM_RCL_EXCEPT"] == 4     # allow RC loss during Offboard/Mission


def test_set_rc_loss_rtl_raises_when_readback_wrong():
    p = _FakeParam(readback={"NAV_RCL_ACT": 0})  # PX4 didn't store it
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_param(p).set_rc_loss_rtl())


def test_set_battery_failsafe_sets_thresholds_and_action():
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_battery_failsafe(
        low=0.25, crit=0.15, emergen=0.07, action=3))
    assert p.float_sets["BAT_LOW_THR"] == 0.25
    assert p.float_sets["BAT_CRIT_THR"] == 0.15
    assert p.float_sets["BAT_EMERGEN_THR"] == 0.07
    assert p.int_sets["COM_LOW_BAT_ACT"] == 3


def test_set_battery_failsafe_raises_when_action_readback_wrong():
    p = _FakeParam(readback={"COM_LOW_BAT_ACT": 0})
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_param(p).set_battery_failsafe(
            low=0.25, crit=0.15, emergen=0.07, action=3))


# ── geofence breach action (GF_ACTION, corrected 2026-08-16) ──


def test_geofence_action_is_return_not_hold():
    """GF_ACTION=2 is HOLD, not Return — this shipped wrong for its whole life.

    PX4's enum (src/modules/navigator/geofence_params.c) is
    0 None / 1 Warning / 2 Hold / 3 Return / 4 Terminate / 5 Land. Setting 2
    made the FC park the aircraft AT the breach point — outside the fence —
    which is the very outcome safety.py's geofence comment says the design
    moved away from.
    """
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_geofence_action_rtl())
    assert p.int_sets["GF_ACTION"] == 3, "3 = Return; 2 would be Hold"


def test_geofence_action_stays_in_the_companion_detectable_set():
    """A geofence action of Hold can never be spotted from the companion.

    PX4 answers our own goto_location (DO_REPOSITION) with AUTO_LOITER, which
    MAVSDK reports as HOLD — the mode the mission flies in end to end. So a
    Hold failsafe is indistinguishable from normal flight. Return (3) and Land
    (5) are the only actions that surface as a distinct mode; pin that, because
    "restore GF_ACTION=2" is exactly the change a future reader would make.
    """
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_geofence_action_rtl())
    assert p.int_sets["GF_ACTION"] in (3, 5)


def test_geofence_action_raises_when_readback_wrong():
    p = _FakeParam(readback={"GF_ACTION": 2})     # PX4 kept the old Hold value
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_param(p).set_geofence_action_rtl())


# ── onboard geofence: "uploaded" is not the same as "fenced" (2026-08-17) ──

# The real KMUTNB controlled airspace (gcs/kmutnb_field.yaml), so a reader can
# see these are field corners and not arbitrary numbers.
_AIRSPACE = [
    (13.8227703, 100.5118179),
    (13.8223327, 100.5121457),
    (13.8225676, 100.5124741),
    (13.8230053, 100.5121463),
]


class _FakeGeofence:
    """Stands in for the MAVSDK geofence plugin. ``stored`` decides what the FC
    is holding AFTER the upload — the whole point of the readback, since PX4
    answers a missing fence with "accept all points" instead of an error."""

    def __init__(self, stored: str = "as-uploaded", *, fail_first: int = 0) -> None:
        self.uploaded: object | None = None
        self.cleared = False
        self._stored = stored
        self.fail_first = fail_first     # transient RPC timeouts before success
        self.attempts = 0

    async def clear_geofence(self) -> None:
        self.cleared = True

    async def upload_geofence(self, data: object) -> None:
        self.attempts += 1
        if self.attempts <= self.fail_first:
            raise TimeoutError("TIMEOUT: 'Timeout'; origin: upload_geofence()")
        self.uploaded = data

    async def download_geofence(self) -> object:
        from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon

        if self._stored == "download-fails":
            raise RuntimeError("MISSION_REQUEST_LIST timed out")
        if self._stored == "empty":
            return GeofenceData([], [])          # the upload silently did not land
        if self._stored == "other-field":        # a fence — just not this field's
            pts = [Point(lat + 0.01, lon) for lat, lon in _AIRSPACE]
            return GeofenceData([Polygon(pts, FenceType.INCLUSION)], [])
        return self.uploaded


def _commander_with_geofence(gf: _FakeGeofence) -> DroneCommander:
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"geofence": gf})()  # type: ignore[assignment]
    return c


def test_upload_geofence_returns_vertex_count_when_the_fc_confirms_it():
    gf = _FakeGeofence()
    assert asyncio.run(_commander_with_geofence(gf).upload_geofence(_AIRSPACE)) == 4
    assert gf.cleared, "a stale fence from a previous mission must be cleared first"


def test_upload_geofence_raises_when_the_fc_holds_no_fence():
    """The failure this exists for: PX4 does NOT fail closed on a missing fence.

    Geofence::isInsidePolygonOrCircle opens with
    `if (isEmpty()) { /* Empty fence -> accept all points */ return true; }`,
    so an upload that didn't land leaves the aircraft accepting every point on
    earth — and from the caller's side that is indistinguishable from success.
    With GF_MAX_HOR_DIST retired (2026-08-17) nothing sits underneath this, so
    an unverified upload has to be an error, not a warning.
    """
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_geofence(_FakeGeofence("empty")).upload_geofence(_AIRSPACE))


def test_upload_geofence_raises_when_the_fc_holds_another_field():
    """Counting items is not verifying them: on 2026-08-15 an orchestrator
    connected to the wrong PX4 and uploaded the wrong field's fence. The vertex
    count matched; the airspace did not. Compare coordinates, not lengths."""
    with pytest.raises(RuntimeError):
        asyncio.run(
            _commander_with_geofence(_FakeGeofence("other-field")).upload_geofence(_AIRSPACE))


def test_upload_geofence_raises_when_the_readback_itself_fails():
    """No readback means no evidence, and no evidence means no fence."""
    with pytest.raises(RuntimeError):
        asyncio.run(
            _commander_with_geofence(_FakeGeofence("download-fails")).upload_geofence(_AIRSPACE))


# ── gimbal mount params (stabilized-nadir pitch servo, PX4 mount driver) ──


def test_set_gimbal_mount_pushes_typed_params():
    """INT-typed MNT_* names go via set_param_int, the rest via
    set_param_float — apply_param_overrides is float-only and PX4 rejects a
    float write to an INT32 param (the COM_DL_LOSS_T lesson)."""
    p = _FakeParam()
    n = asyncio.run(_commander_with_param(p).set_gimbal_mount({
        "MNT_MODE_IN": 4, "MNT_MODE_OUT": 1, "MNT_DO_STAB": 1,
        "MNT_RANGE_PITCH": 90.0, "MNT_OFF_PITCH": 0.0,
    }))
    assert n == 5
    assert p.int_sets == {"MNT_MODE_IN": 4, "MNT_MODE_OUT": 1, "MNT_DO_STAB": 1}
    assert p.float_sets == {"MNT_RANGE_PITCH": 90.0, "MNT_OFF_PITCH": 0.0}


def test_set_gimbal_mount_is_best_effort():
    """The gimbal is NOT flight-critical and SITL's PX4 lacks the mount module —
    a per-param failure is swallowed (counted out), never raised."""

    class _FailingParam(_FakeParam):
        async def set_param_int(self, name: str, value: int) -> None:
            raise RuntimeError("param missing in SITL")

    p = _FailingParam()
    n = asyncio.run(_commander_with_param(p).set_gimbal_mount({
        "MNT_MODE_IN": 4, "MNT_RANGE_PITCH": 90.0,
    }))
    assert n == 1                       # only the float landed
    assert p.float_sets == {"MNT_RANGE_PITCH": 90.0}


# ── flight-envelope pins: applying is best-effort, the envelope is not ──


def test_apply_param_overrides_routes_int_typed_params():
    """PX4 rejects a float write to an INT32 param. Two of these shipped broken
    before the list became shared (EKF2_* as 'applied 23/25', then
    SIM_BAT_ENABLE timing out on every SITL boot), so pin the routing."""
    p = _FakeParam()
    n = asyncio.run(_commander_with_param(p).apply_param_overrides({
        "EKF2_RNG_CTRL": 1, "SIM_BAT_ENABLE": 1, "MPC_Z_V_AUTO_DN": 0.4,
    }))
    assert n == 3
    assert p.int_sets == {"EKF2_RNG_CTRL": 1, "SIM_BAT_ENABLE": 1}
    assert p.float_sets == {"MPC_Z_V_AUTO_DN": 0.4}


def test_gimbal_pitch_channel_selector_is_written_as_an_int():
    """MNT_MAN_PITCH is not an angle — PX4 types it INT32 because it selects the
    RC AUX channel. It was in the float path, which is what made the gimbal push
    report 5/6 applied."""
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_gimbal_mount({"MNT_MAN_PITCH": 0}))
    assert p.int_sets == {"MNT_MAN_PITCH": 0}
    assert p.float_sets == {}


def test_envelope_pins_pass_when_the_fc_holds_them():
    pins = {"RTL_RETURN_ALT": 20.0, "MPC_Z_V_AUTO_DN": 0.4, "COM_DISARM_LAND": -1.0}
    p = _FakeParam()
    c = _commander_with_param(p)
    asyncio.run(c.apply_param_overrides(pins))
    assert asyncio.run(c.verify_envelope_pins(pins)) == []


def test_envelope_pins_report_a_value_that_did_not_stick():
    pins = {"RTL_RETURN_ALT": 20.0, "MPC_Z_V_AUTO_DN": 0.4, "COM_DISARM_LAND": -1.0}
    p = _FakeParam(float_readback={"RTL_RETURN_ALT": 60.0})   # PX4 kept its default
    c = _commander_with_param(p)
    asyncio.run(c.apply_param_overrides(pins))
    bad = asyncio.run(c.verify_envelope_pins(pins))
    assert len(bad) == 1
    assert "RTL_RETURN_ALT" in bad[0] and "60" in bad[0]


def test_envelope_pins_catch_the_poisoned_link_case():
    """The documented failure: a stale mavsdk_server holds the ports, every
    param RPC fails ('applied 0/24'), and the mission would fly on PX4 defaults —
    RTL at 60 m through the 20 m ceiling, 1.5 m/s onto the pad."""
    pins = {"RTL_RETURN_ALT": 20.0, "MPC_Z_V_AUTO_DN": 0.4, "COM_DISARM_LAND": -1.0}
    p = _FakeParam(float_readback={
        "RTL_RETURN_ALT": 60.0, "MPC_Z_V_AUTO_DN": 1.5, "COM_DISARM_LAND": 2.0})
    bad = asyncio.run(_commander_with_param(p).verify_envelope_pins(pins))
    assert len(bad) == 3


def test_envelope_pins_only_check_what_was_asked_for():
    """A tuning-mode run that never pins the envelope must not be told the FC is
    holding the wrong values for params it deliberately left alone."""
    p = _FakeParam()
    assert asyncio.run(
        _commander_with_param(p).verify_envelope_pins({"MPC_XY_CRUISE": 8.0})) == []


def test_unreadable_pin_is_a_finding_not_a_pass():
    p = _FakeParam()          # nothing was ever set → get_param_float raises
    bad = asyncio.run(
        _commander_with_param(p).verify_envelope_pins({"RTL_RETURN_ALT": 20.0}))
    assert len(bad) == 1 and "RTL_RETURN_ALT" in bad[0]


# ── the fence upload retries; the fence CONTRACT does not soften ───────────
#
# One shot until 2026-08-23, and on 2026-08-19 that cost two staged flights in
# a single session: clear_geofence TIMEOUT, upload error, "FC geofence NOT
# verified — refusing to fly", twice, twenty seconds apart, on a link that was
# otherwise fine. The refusal is right — PX4 reads a missing fence as "accept
# all points" — so the fix is to stop losing the upload, not to stop checking.

def test_a_transient_timeout_no_longer_grounds_the_aircraft():
    gf = _FakeGeofence(fail_first=2)          # fails twice, succeeds on the 3rd
    assert asyncio.run(_commander_with_geofence(gf).upload_geofence(_AIRSPACE)) == 4
    assert gf.attempts == 3
    assert gf.cleared, "each retry must clear before re-uploading"


def test_a_fence_that_never_lands_is_still_do_not_fly():
    """Retrying must not turn 'unverified' into 'good enough'."""
    gf = _FakeGeofence(fail_first=99)
    with pytest.raises(RuntimeError, match="not verified after 3 attempts"):
        asyncio.run(_commander_with_geofence(gf).upload_geofence(_AIRSPACE))
    assert gf.attempts == 3


def test_a_fence_that_reads_back_as_another_field_is_refused_every_attempt():
    """The readback failure mode, not the RPC one: the upload 'succeeds' each
    time and the FC holds someone else's airspace. Retrying cannot fix that,
    and it must still refuse rather than fly fenced to the wrong field."""
    gf = _FakeGeofence("other-field")
    with pytest.raises(RuntimeError, match="not verified after 3 attempts"):
        asyncio.run(_commander_with_geofence(gf).upload_geofence(_AIRSPACE))


# ── takeoff climb gate vs PX4's own acceptance radius (2026-08-24) ───────────
# PX4 leaves AUTO_TAKEOFF as soon as it is within NAV_MC_ALT_RAD of the target,
# so the aircraft levels off at target - NAV_MC_ALT_RAD and that is the highest
# altitude this gate will ever observe. The old tolerance was a percentage with
# no relationship to that radius: 0.15*6.5 = 0.975 against PX4's 0.8 left a
# 17 cm margin nobody had chosen, and two real flights landed on either side of
# it — 5.72 m passed, 5.43 m failed and aborted the whole mission at takeoff.


def _tol(target: float) -> float:
    from mavlink_adapter.commands import DroneCommander
    return DroneCommander._altitude_tolerance_m(None, target)   # type: ignore[arg-type]


def test_the_climb_gate_always_clears_px4s_own_acceptance_radius() -> None:
    """The gate must be reachable. A tolerance under NAV_MC_ALT_RAD can never
    pass, because PX4 stops climbing exactly there."""
    from mavlink_adapter.commands import DEFAULT_PX4_TUNING
    radius = DEFAULT_PX4_TUNING["NAV_MC_ALT_RAD"]
    for target in (3.0, 5.0, 6.5, 9.0, 12.0, 20.0):
        levels_off_at = target - radius
        passes_at = target - _tol(target)
        assert passes_at < levels_off_at, (
            f"target {target}: gate wants {passes_at:.2f} m but PX4 stops at "
            f"{levels_off_at:.2f} m — unreachable")


def test_the_margin_is_the_same_at_every_climb_altitude() -> None:
    """The old percentage gave 17 cm of margin at 6.5 m and 20 cm at 20 m —
    both accidents of arithmetic. Derived from the radius it is one number, so
    a profile change cannot quietly shrink it."""
    from mavlink_adapter.commands import _ALT_ACCEPT_MARGIN_M, DEFAULT_PX4_TUNING
    radius = DEFAULT_PX4_TUNING["NAV_MC_ALT_RAD"]
    margins = {round((t - radius) - (t - _tol(t)), 6) for t in (3.0, 6.5, 9.0, 20.0)}
    assert margins == {_ALT_ACCEPT_MARGIN_M}, margins


def test_the_two_real_flights_would_both_have_passed() -> None:
    """The measured levelling altitudes, 2026-08-24 ULogs 07_22_22 and
    07_44_22, commanded 6.5 m. Under the old rule 5.43 m failed."""
    gate = 6.5 - _tol(6.5)
    for measured in (5.72, 5.61, 5.43):
        assert measured >= gate, f"{measured} m would still fail (gate {gate:.2f})"


def test_the_gate_still_rejects_a_takeoff_that_never_climbed() -> None:
    """Loosening it must not turn the check into a rubber stamp — it exists to
    catch a thrust-loss / EKF-drift takeoff that barely left the ground."""
    gate = 6.5 - _tol(6.5)
    for stuck in (0.0, 1.0, 2.5, 4.0):
        assert stuck < gate, f"{stuck} m must NOT count as a 6.5 m climb"


def test_the_vertical_radius_is_pinned_in_both_field_configs() -> None:
    """It was unpinned until 2026-08-24, so a PX4 default change would have
    moved the takeoff gate underneath us with nothing to notice it."""
    from pathlib import Path

    import yaml

    from mavlink_adapter.commands import DEFAULT_PX4_TUNING
    root = Path(__file__).resolve().parents[1] / "sitl"
    for name in ("aavc_config.yaml", "kmitl_config.yaml"):
        tune = yaml.safe_load((root / name).read_text())["px4_tuning"]
        assert tune.get("NAV_MC_ALT_RAD") == DEFAULT_PX4_TUNING["NAV_MC_ALT_RAD"], name


# ── expected_mode: the mode WE asked for, so safety can tell an FC failsafe ──

def test_land_and_rth_record_the_mode_they_ask_for_and_goto_clears_it() -> None:
    """2026-08-26: PX4's own geofence RTL looked, to the watchdog, exactly like
    the mission's own RTL/LAND. The commander now records which AUTO mode it
    requested; a movement command clears it (the aircraft is ours again)."""
    import asyncio as _a
    from types import SimpleNamespace as N
    c = DroneCommander.__new__(DroneCommander)
    c._pilot_in_control = False
    c._home_alt_msl = 50.0
    c.home_alt_source = None
    c.expected_mode = None
    calls: list[str] = []

    async def _land() -> None: calls.append("land")
    async def _goto(*a) -> None: calls.append("goto")
    c.system = N(action=N(land=_land, goto_location=_goto))

    _a.run(c.land(disarm=False))
    assert c.expected_mode == "LAND"
    _a.run(c.goto(13.7, 100.7, 8.0))
    assert c.expected_mode is None
    assert calls == ["land", "goto"]
