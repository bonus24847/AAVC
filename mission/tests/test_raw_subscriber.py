"""Raw pymavlink telemetry augmentation (dashboard ESC/servo/consumed widgets).

MAVSDK doesn't expose per-servo PWM, per-ESC current/RPM, or the running
consumed-mAh counter; RawMavlinkSubscriber reads those off a dedicated UDP
endpoint. _apply is a pure msg→state function, so we drive it directly. ATTITUDE
is deliberately NOT handled here (MAVSDK owns roll/pitch).
"""

from __future__ import annotations

import asyncio
from typing import Any

from mavlink_adapter.raw_subscriber import _MSG_TYPES, RawMavlinkSubscriber
from mavlink_adapter.telemetry import CurrentTelemetry


class _Msg:
    def __init__(self, mtype: str, **attrs: Any) -> None:
        self._t = mtype
        self.__dict__.update(attrs)

    def get_type(self) -> str:
        return self._t


def _sub() -> RawMavlinkSubscriber:
    return RawMavlinkSubscriber(CurrentTelemetry())


def test_attitude_is_not_subscribed() -> None:
    assert "ATTITUDE" not in _MSG_TYPES
    assert set(_MSG_TYPES) == {"SERVO_OUTPUT_RAW", "ESC_STATUS", "BATTERY_STATUS"}


def test_attitude_message_is_ignored_so_mavsdk_owns_roll_pitch() -> None:
    sub = _sub()
    sub._apply(_Msg("ATTITUDE", roll=1.0, pitch=1.0, rollspeed=1.0,
                    pitchspeed=1.0, yawspeed=1.0))
    import math
    assert math.isnan(sub.state.roll_deg)     # untouched — MAVSDK writes it
    assert math.isnan(sub.state.pitch_deg)


def test_servo_output_raw_populates_pwm() -> None:
    sub = _sub()
    attrs = {f"servo{i}_raw": 1000 + i for i in range(1, 17)}
    sub._apply(_Msg("SERVO_OUTPUT_RAW", **attrs))
    assert sub.state.servo_pwm_us[0] == 1001
    assert sub.state.servo_pwm_us[8] == 1009
    assert len(sub.state.servo_pwm_us) == 16


def test_esc_status_populates_rpm_current_voltage() -> None:
    sub = _sub()
    sub._apply(_Msg("ESC_STATUS", index=0, rpm=[100, 200, 300, 400],
                    current=[1.0, 2.0, 3.0, 4.0], voltage=[22.0, 22.1, 22.2, 22.3]))
    assert sub.state.esc_rpm[0] == 100
    assert sub.state.esc_rpm[3] == 400
    assert sub.state.esc_current_a[1] == 2.0
    assert sub.state.esc_voltage_v[2] == 22.2


def test_battery_status_populates_consumed_mah() -> None:
    sub = _sub()
    sub._apply(_Msg("BATTERY_STATUS", current_consumed=1234))
    assert sub.state.battery_consumed_mah == 1234.0
    # A negative sentinel (unknown) must NOT overwrite it.
    sub._apply(_Msg("BATTERY_STATUS", current_consumed=-1))
    assert sub.state.battery_consumed_mah == 1234.0


def test_disabled_port_starts_no_listener() -> None:
    sub = RawMavlinkSubscriber(CurrentTelemetry(), udp_port=0)
    asyncio.run(sub.start())
    assert sub._task is None
    asyncio.run(sub.stop())      # safe no-op
