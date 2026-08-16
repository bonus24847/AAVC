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

from mavlink_adapter.commands import DroneCommander, _pwm_to_norm

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


# ── one-motor-out (CA_FAILURE_MODE / COM_ACT_FAIL_ACT, 2026-08-16) ──


def test_set_motor_failure_failsafe_arms_both_halves():
    """PX4 ships the hexa's rotor redundancy switched off in two places, and
    BOTH have to be set: the allocator has to drop the dead motor from its
    effectiveness matrix (CA_FAILURE_MODE), and the commander has to do
    something about it (COM_ACT_FAIL_ACT, default 0 = log a warning)."""
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_motor_failure_failsafe())
    assert p.int_sets["CA_FAILURE_MODE"] == 1    # remove the failed motor
    assert p.int_sets["COM_ACT_FAIL_ACT"] == 2   # 2 = Land where it is


def test_set_motor_failure_failsafe_raises_when_allocator_readback_wrong():
    """The dangerous case is the half that fails QUIETLY: COM_ACT_FAIL_ACT
    stored, allocator mode not. The aircraft would then land on a motor
    failure — while still mixing for six healthy rotors on the way down."""
    p = _FakeParam(readback={"CA_FAILURE_MODE": 0})
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_param(p).set_motor_failure_failsafe())


def test_set_motor_failure_failsafe_raises_when_action_readback_wrong():
    p = _FakeParam(readback={"COM_ACT_FAIL_ACT": 0})
    with pytest.raises(RuntimeError):
        asyncio.run(_commander_with_param(p).set_motor_failure_failsafe())


def test_set_motor_failure_failsafe_action_is_overridable():
    """Return (3) instead of Land (2) is a one-integer config change, not a
    code edit — the field layout decides which is safer."""
    p = _FakeParam()
    asyncio.run(_commander_with_param(p).set_motor_failure_failsafe(action=3))
    assert p.int_sets["COM_ACT_FAIL_ACT"] == 3


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
