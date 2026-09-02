"""The companion's AGL must ride on the home altitude LATCHED AT ARMING, not on
PX4's live ``relative_alt`` (2026-08-26 RTH, ULog ``2026-08-26/08_21_26.ulg``).

PX4 1.17 (upstream 6604c52c98, "Adjust home position altitude after GNSS
altitude correction", #25003) rewrites ``home.alt`` for 120 s after takeoff
whenever baro and the integrated GPS vertical velocity agree with each other
but disagree with GPS altitude by > 1 m. That logic assumes a GPS-referenced
EKF; ours is baro-referenced (``EKF2_HGT_REF=0``), so the EKF's altitude does
NOT follow the GPS drift and the shift INTRODUCES an error in
``gpos.alt - home.alt`` — the number MAVSDK hands us as ``relative_altitude_m``.
Every real flight since 2026-08-21 shows it (+3.29 / -4.65 / +0.91 / +1.17 /
-3.18 m). On 2026-08-26 the aircraft held 8.5 m (lidar, EKF and setpoint all
agree) while ``relative_alt`` walked to 11.7 m, and the ceiling watchdog —
correct on its input — flew it home.

The goto path was never affected because ``DroneCommander`` converts AGL to
MSL with a home cached at arming. This pins the same rule onto the telemetry
snapshot every consumer reads (ceiling watchdog, touchdown fallback, climb
waits, TELEM audit): ``relative_alt_m = absolute - home_latched_at_arm``.
"""

from __future__ import annotations

import asyncio
import math
from types import SimpleNamespace

import pytest

from mavlink_adapter.commands import DroneCommander
from mavlink_adapter.telemetry import TelemetrySubscriber

# ── a scriptable MAVSDK stand-in: each stream yields what the test lists ──

class _Telemetry:
    def __init__(self) -> None:
        self.script: dict[str, list[object]] = {}

    def _stream(self, name: str):
        async def gen():
            for item in self.script.get(name, []):
                yield item
        return gen()

    def home(self):
        return self._stream("home")

    def position(self):
        return self._stream("position")

    def armed(self):
        return self._stream("armed")

    def landed_state(self):
        return self._stream("landed_state")


def _sub() -> tuple[TelemetrySubscriber, _Telemetry]:
    tel = _Telemetry()
    system = SimpleNamespace(telemetry=tel)
    return TelemetrySubscriber(system=system), tel  # type: ignore[arg-type]


def _home(alt: float) -> SimpleNamespace:
    return SimpleNamespace(latitude_deg=13.8227953, longitude_deg=100.511633,
                           absolute_altitude_m=alt)


def _pos(abs_alt: float, rel_alt: float) -> SimpleNamespace:
    return SimpleNamespace(latitude_deg=13.8227953, longitude_deg=100.511633,
                           absolute_altitude_m=abs_alt, relative_altitude_m=rel_alt)


def _landed(name: str) -> SimpleNamespace:
    return SimpleNamespace(name=name)


def _feed(sub: TelemetrySubscriber, tel: _Telemetry, stream: str, *items: object) -> None:
    """Push `items` through ONE subscriber coroutine, in order."""
    tel.script = {stream: list(items)}
    coro = {
        "home": sub._sub_home,
        "position": sub._sub_position,
        "armed": sub._sub_armed,
        "landed_state": sub._sub_landed_state,
    }[stream]
    asyncio.run(coro())


# ── the 2026-08-26 flight, replayed from the ULog ──

def test_relative_alt_ignores_px4_rewriting_home_in_flight() -> None:
    sub, tel = _sub()
    # on the ground, disarmed: PX4's home is 53.41 m MSL
    _feed(sub, tel, "home", _home(53.41))
    _feed(sub, tel, "armed", True)                       # RC arm (t=0)
    _feed(sub, tel, "landed_state", _landed("IN_AIR"))   # t≈4 takeoff
    _feed(sub, tel, "position", _pos(61.90, 8.49))       # transit at 8.5 m
    assert sub.state.relative_alt_m == pytest.approx(8.49, abs=0.01)

    # t=27 and t=46.6: PX4 shifts home.alt 53.41 → 52.27 → 50.23 while the
    # aircraft does not move (lidar 8.5 m the whole time)
    _feed(sub, tel, "home", _home(52.27), _home(50.23))
    _feed(sub, tel, "position", _pos(61.90, 11.67))      # PX4's own relative
    assert sub.state.home_alt_msl == pytest.approx(53.41)
    assert sub.state.relative_alt_m == pytest.approx(8.49, abs=0.01), \
        "the watchdog's altitude followed PX4's rewritten home — the RTH bug"
    assert sub.state.px4_relative_alt_m == pytest.approx(11.67)  # kept raw


def test_home_re_capture_on_the_ground_is_accepted() -> None:
    """PX4 re-captures home at every disarm/arm (landing spot, current baro
    frame). Those are the captures we WANT — only airborne rewrites are
    refused."""
    sub, tel = _sub()
    _feed(sub, tel, "home", _home(53.41))
    _feed(sub, tel, "armed", True)
    _feed(sub, tel, "landed_state", _landed("IN_AIR"))
    _feed(sub, tel, "position", _pos(61.90, 8.49))
    _feed(sub, tel, "home", _home(50.23))                 # airborne rewrite: refused
    assert sub.state.home_alt_msl == pytest.approx(53.41)

    # landed, disarmed: PX4 sets home at the landing point (t=98.7 in the log)
    _feed(sub, tel, "landed_state", _landed("ON_GROUND"))
    _feed(sub, tel, "armed", False)
    _feed(sub, tel, "home", _home(53.48))
    _feed(sub, tel, "position", _pos(53.48, 0.0))
    assert sub.state.home_alt_msl == pytest.approx(53.48)
    assert sub.state.relative_alt_m == pytest.approx(0.0, abs=0.01)

    # next flight: armed, still on the ground — the arm-time re-capture
    # arrives up to 2 s after the arm edge and must still be taken
    _feed(sub, tel, "armed", True)
    _feed(sub, tel, "home", _home(53.52))
    assert sub.state.home_alt_msl == pytest.approx(53.52)
    # ...but nothing after the aircraft has left the ground
    _feed(sub, tel, "landed_state", _landed("TAKING_OFF"))
    _feed(sub, tel, "home", _home(51.0))
    assert sub.state.home_alt_msl == pytest.approx(53.52)


def test_a_pad_landing_mid_flight_keeps_the_latch_frozen() -> None:
    """Between deliveries the aircraft sits ON_GROUND on a pad, still ARMED
    (COM_DISARM_LAND=-1). PX4's 120 s correction window can still be open
    there — the latch stays frozen until a real disarm."""
    sub, tel = _sub()
    _feed(sub, tel, "home", _home(53.41))
    _feed(sub, tel, "armed", True)
    _feed(sub, tel, "landed_state", _landed("IN_AIR"))
    _feed(sub, tel, "landed_state", _landed("ON_GROUND"))   # on the pad
    _feed(sub, tel, "home", _home(50.23))
    assert sub.state.home_alt_msl == pytest.approx(53.41)


def test_freezes_on_px4_altitude_even_if_landed_state_never_reports() -> None:
    """A dead landed_state stream must not re-open the latch: PX4's own
    relative altitude passing 1 m is enough to know we are airborne (and at
    that instant it is still trustworthy — the correction needs a takeoff
    plus > 1 m of divergence first)."""
    sub, tel = _sub()
    _feed(sub, tel, "home", _home(53.41))
    _feed(sub, tel, "armed", True)
    _feed(sub, tel, "position", _pos(61.90, 8.49))       # landed_state: UNKNOWN
    _feed(sub, tel, "home", _home(50.23))
    assert sub.state.home_alt_msl == pytest.approx(53.41)
    _feed(sub, tel, "position", _pos(61.90, 11.67))
    assert sub.state.relative_alt_m == pytest.approx(8.49, abs=0.01)


def test_falls_back_to_px4_relative_before_any_home_is_known() -> None:
    """No home yet (no GPS fix, SITL warming up): the old behaviour, unchanged."""
    sub, tel = _sub()
    _feed(sub, tel, "position", _pos(53.41, 0.3))
    assert math.isnan(sub.state.home_alt_msl)
    assert sub.state.relative_alt_m == pytest.approx(0.3)


# ── the commander's AGL→MSL conversion rides the same latch ──

def test_goto_prefers_the_latched_home_over_the_connect_time_cache() -> None:
    """On the RC-arm conops (RC-GO) arm_and_takeoff() sees "already armed" and
    never refreshes its connect-time cache — a second flight after a landing
    elsewhere in the baro frame would convert AGL with a stale home. The
    telemetry latch is refreshed at every arming edge, so goto() takes it."""
    sent: list[tuple[float, float, float, float]] = []

    async def goto_location(lat: float, lon: float, alt: float, yaw: float) -> None:
        sent.append((lat, lon, alt, yaw))

    c = DroneCommander.__new__(DroneCommander)
    c._pilot_in_control = False
    c._home_alt_msl = 53.41                                # connect-time cache
    c.system = SimpleNamespace(action=SimpleNamespace(goto_location=goto_location))
    c.home_alt_source = lambda: 53.90                      # latched at this arming
    asyncio.run(c.goto(13.7, 100.7, 10.0))
    assert sent[0][2] == pytest.approx(63.90)

    # no latch yet (nan) → the connect-time cache still works
    c.home_alt_source = lambda: math.nan
    asyncio.run(c.goto(13.7, 100.7, 10.0))
    assert sent[1][2] == pytest.approx(63.41)
