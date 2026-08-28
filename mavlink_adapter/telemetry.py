"""Async telemetry subscriber — coalesces multiple MAVSDK streams into a
single CurrentTelemetry snapshot the orchestrator can poll.

MAVSDK exposes each field (position, battery, GPS, RC, EKF) as its own
async stream. The orchestrator wants one synchronized view; this module
fans-in via asyncio tasks and updates a shared dataclass.
"""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field

from loguru import logger
from mavsdk import System


@dataclass
class CurrentTelemetry:
    """Snapshot of vehicle state at a moment in time.

    Fields populated by `TelemetrySubscriber` come from MAVSDK — including
    roll/pitch (attitude_euler), the body rates (attitude_angular_velocity_body),
    and per-servo PWM (actuator_output_status). Fields populated by
    `RawMavlinkSubscriber` (dashboard support) come from a parallel pymavlink
    listener on a separate UDP port — the per-ESC current/RPM/voltage and the
    running consumed_mah counter, which MAVSDK doesn't expose. The dashboard
    widgets (attitude indicator, motor bars, servo PWM bars) read both.

    Defaults are nan/[]/0 so tests and code that touch only MAVSDK fields
    don't have to know about the pymavlink ones.
    """

    last_update_monotonic: float = field(default_factory=time.monotonic)
    is_connected: bool = False
    is_armed: bool = False
    flight_mode: str = "UNKNOWN"
    # PX4's LAND DETECTOR verdict (MAVSDK telemetry.landed_state →
    # EXTENDED_SYS_STATE): ON_GROUND / IN_AIR / TAKING_OFF / LANDING /
    # UNKNOWN. The touchdown-gated release keys on THIS, never on an
    # altitude threshold — the AGL estimate's ground level wanders ~1 m per
    # arming (2026-08-13 screencast: box released mid-air at ~1 m because
    # the old alt<=1.5 gate fired while still sinking).
    landed_state: str = "UNKNOWN"
    lat: float = math.nan
    lon: float = math.nan
    alt_m: float = math.nan
    # AGL = alt_m - home_alt_msl, with home LATCHED at arming (see
    # TelemetrySubscriber._sub_home). NOT PX4's own relative_alt: PX4 1.17
    # (upstream 6604c52c98, #25003) rewrites home.alt for 120 s after takeoff
    # from a baro-vs-GPS comparison that assumes a GPS-referenced EKF; ours is
    # baro-referenced (EKF2_HGT_REF=0), so the rewrite moves the number while
    # the aircraft stays put — 2026-08-26: 8.5 m by lidar/EKF/setpoint, 11.7 m
    # by relative_alt, and the ceiling watchdog flew it home. Every real
    # flight since 2026-08-21 carried a shift of +0.9 to -4.65 m.
    relative_alt_m: float = math.nan
    # PX4's live GLOBAL_POSITION_INT.relative_alt, kept raw for diagnostics —
    # the gap between it and relative_alt_m IS the home rewrite.
    px4_relative_alt_m: float = math.nan
    # Home MSL altitude latched at the last arming (nan until a HOME_POSITION
    # has been seen). goto() converts AGL->MSL with this same value.
    home_alt_msl: float = math.nan
    ground_speed_mps: float = math.nan
    heading_deg: float = math.nan
    battery_percent: float = math.nan
    battery_voltage_v: float = math.nan
    battery_current_a: float = math.nan     # pack draw; drives the GCS power readout
    gps_fix_type: int = 0       # 0=no fix, 3=3D, 4=DGPS, 5=RTK_FLOAT, 6=RTK_FIXED
    gps_satellites: int = 0
    # The AIRCRAFT's own clock, seconds, from raw_gps().timestamp_us — PX4
    # stamps it with hrt_absolute_time(), which is the lockstep SIMULATION
    # clock in SITL and the real one on hardware. Drives orchestrator's
    # FlightClock so every mission deadline is measured in flying time rather
    # than host time; see orchestrator/flight_clock.py for why that matters.
    # The epoch is arbitrary and may jump on a reboot — only DIFFERENCES mean
    # anything, and FlightClock is what enforces that.
    vehicle_time_s: float = math.nan
    datalink_rssi: int = -1     # not always available; depends on RFD900 + PX4 driver
    rc_signal_available: bool = False
    # --- MAVSDK telemetry.health() (pre-flight readiness gate; ~1 Hz) ---
    # All default False so a pre-first-frame snapshot reads "not ready" (fail
    # closed) rather than spuriously green. `is_armable` is PX4's composite
    # pre-arm verdict; the rest let the preflight checklist show WHY.
    is_armable: bool = False
    is_global_position_ok: bool = False
    is_local_position_ok: bool = False
    is_home_position_ok: bool = False
    is_gyrometer_calibrated: bool = False
    is_accelerometer_calibrated: bool = False
    is_magnetometer_calibrated: bool = False
    # --- pymavlink-populated (dashboard / S2) ---
    roll_deg: float = math.nan
    pitch_deg: float = math.nan
    roll_rate_dps: float = math.nan
    pitch_rate_dps: float = math.nan
    yaw_rate_dps: float = math.nan
    servo_pwm_us: list[int] = field(default_factory=list)        # up to 16 channels
    esc_current_a: list[float] = field(default_factory=list)     # per-motor current
    esc_rpm: list[int] = field(default_factory=list)             # per-motor RPM
    esc_voltage_v: list[float] = field(default_factory=list)     # per-motor voltage
    # esc_current_a/rpm/voltage are DASHBOARD DISPLAY ONLY now. The motor-health
    # watchdog that read per-motor current was removed 2026-08-17 (these ESCs are
    # PWM-only, no current telemetry lead), so nothing reads them for a flight
    # decision — and the staleness stamp that check needed went with it.
    battery_consumed_mah: float = math.nan
    # When that coulomb count last ARRIVED. It rides the optional raw-MAVLink
    # listener, so it can go quiet while MAVSDK telemetry keeps flowing —
    # last_update_monotonic above is touched by both and cannot tell them apart.
    # The energy budget demotes a stale count to the percentage estimate rather
    # than let a frozen number keep passing as a measurement.
    battery_consumed_monotonic: float = math.nan

    def age_s(self) -> float:
        return time.monotonic() - self.last_update_monotonic


class TelemetrySubscriber:
    """Subscribes to MAVSDK telemetry streams and updates a shared snapshot."""

    def __init__(self, system: System) -> None:
        self.system = system
        self.state = CurrentTelemetry()
        self._tasks: list[asyncio.Task[None]] = []
        self._stream_seen: dict[str, float] = {}
        # How many times a subscriber's stream died and was restarted. Surfaced
        # so a flight that spent its time re-establishing streams looks
        # different afterwards from one that did not.
        self.stream_failures = 0
        # Home-altitude latch (CurrentTelemetry.home_alt_msl): the latest
        # HOME_POSITION seen, and whether the aircraft has been airborne since
        # the current arming — once it has, home updates are REFUSED until a
        # disarm. A pad landing between deliveries (armed, ON_GROUND) keeps it
        # frozen: PX4's 120 s correction window can still be open there.
        self._home_seen_msl: float = math.nan
        self._home_refused_msl: float = math.nan   # last in-flight value warned about
        self._airborne_since_arm = False
        self._home_gap_warned = False

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        subs = {
            "connection": self._sub_connection,
            "position": self._sub_position,
            "home": self._sub_home,
            "velocity": self._sub_velocity,
            "attitude": self._sub_attitude,
            "attitude_rates": self._sub_attitude_rates,
            "actuator_outputs": self._sub_actuator_outputs,
            "battery": self._sub_battery,
            "armed": self._sub_armed,
            "flight_mode": self._sub_flight_mode,
            "landed_state": self._sub_landed_state,
            "gps_info": self._sub_gps_info,
            "vehicle_clock": self._sub_vehicle_clock,
            "rc_status": self._sub_rc_status,
            "health": self._sub_health,
        }
        self._tasks = [loop.create_task(self._supervise(name, factory))
                       for name, factory in subs.items()]
        logger.info(f"[telemetry] subscribed to {len(self._tasks)} streams")

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        logger.info("[telemetry] stopped")

    def _touch(self, stream: str = "") -> None:
        self.state.last_update_monotonic = time.monotonic()
        if stream:
            self._stream_seen[stream] = time.monotonic()

    def stream_age_s(self, stream: str) -> float:
        """Seconds since THIS stream last delivered, or inf if never.

        ``CurrentTelemetry.age_s()`` cannot answer this: all 14 subscribers
        touch the same timestamp, so a dead stream is invisible — the other 13
        keep it fresh while the dead one's field stays frozen at its last value
        forever. That is not academic: a frozen ``is_armed`` blinds the
        disarm detector permanently (the exact hole the takeover fix closed),
        and a frozen ``relative_alt_m`` feeds the ceiling watchdog and every
        climb/descent wait a number that stopped moving (2026-08-22 review)."""
        seen = self._stream_seen.get(stream)
        return float("inf") if seen is None else time.monotonic() - seen

    def dead_streams(self, max_age_s: float = 5.0) -> list[str]:
        """Streams that have gone quiet past ``max_age_s`` (started but stale),
        so a caller can say WHICH one died rather than "telemetry is old"."""
        now = time.monotonic()
        return sorted(name for name, seen in self._stream_seen.items()
                      if now - seen > max_age_s)

    async def _supervise(self, name: str, factory: Callable[[], Awaitable[None]]) -> None:
        """Run one subscriber and RESTART it if its stream raises.

        Eight of these had no exception handling at all: a gRPC stream error
        killed the task silently — no log, no retry, no restart — and the field
        it fed froze at its last value with the shared freshness stamp still
        being touched by the survivors. Restarting is the right response
        (MAVSDK streams do drop and re-establish); recording the death is the
        part that was missing."""
        backoff = 0.5
        while True:
            try:
                await factory()
                logger.warning(f"[telemetry] stream {name} ended — restarting")
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 — one stream must not end the flight
                logger.warning(f"[telemetry] stream {name} failed ({e}) — restarting")
            self.stream_failures += 1
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, 5.0)

    async def _sub_connection(self) -> None:
        async for st in self.system.core.connection_state():
            self.state.is_connected = st.is_connected
            self._touch("connection")

    async def _sub_position(self) -> None:
        async for pos in self.system.telemetry.position():
            self.state.lat = pos.latitude_deg
            self.state.lon = pos.longitude_deg
            self.state.alt_m = pos.absolute_altitude_m
            self.state.px4_relative_alt_m = pos.relative_altitude_m
            home = self.state.home_alt_msl
            if math.isnan(home):
                # no HOME_POSITION yet (no fix / SITL warming up): PX4's own
                # value is all there is — and on the ground it is right.
                self.state.relative_alt_m = pos.relative_altitude_m
            else:
                self.state.relative_alt_m = pos.absolute_altitude_m - home
                self._note_home_gap(pos.relative_altitude_m)
            # PX4's relative alt is trustworthy at the instant of takeoff (the
            # correction needs a takeoff plus > 1 m of divergence first), so
            # it is a second, landed_state-independent way to know we are
            # airborne and must freeze the latch.
            if self.state.is_armed and pos.relative_altitude_m > 1.0:
                self._airborne_since_arm = True
            self._touch("position")

    async def _sub_home(self) -> None:
        """Latch the home MSL altitude at arming; refuse airborne rewrites."""
        async for home in self.system.telemetry.home():
            self._home_seen_msl = float(home.absolute_altitude_m)
            self._maybe_latch_home()
            self._touch("home")

    def _maybe_latch_home(self) -> None:
        if math.isnan(self._home_seen_msl):
            return
        if self.state.is_armed and self._airborne_since_arm:
            # HOME_POSITION streams at ~1 Hz and repeats the shifted value —
            # warn once per NEW value, not once per sample (2026-08-26: one
            # line per second for a whole flight).
            if (abs(self._home_seen_msl - self.state.home_alt_msl) > 0.05
                    and abs(self._home_seen_msl - self._home_refused_msl) > 0.05):
                self._home_refused_msl = self._home_seen_msl
                logger.warning(
                    f"[telemetry] PX4 rewrote home.alt in flight "
                    f"{self.state.home_alt_msl:.2f} → {self._home_seen_msl:.2f} m "
                    "— IGNORED, AGL stays on the home latched at arming")
            return
        self._home_refused_msl = math.nan
        if abs(self._home_seen_msl - self.state.home_alt_msl) > 0.05 or \
                math.isnan(self.state.home_alt_msl):
            logger.info(f"[telemetry] home MSL latched: {self._home_seen_msl:.2f} m")
        self.state.home_alt_msl = self._home_seen_msl

    def _note_home_gap(self, px4_relative_alt_m: float) -> None:
        gap = abs(self.state.relative_alt_m - px4_relative_alt_m)
        if gap > 1.0 and not self._home_gap_warned:
            self._home_gap_warned = True
            logger.warning(
                f"[telemetry] AGL {self.state.relative_alt_m:.1f} m vs PX4 "
                f"relative_alt {px4_relative_alt_m:.1f} m — home rewrite in "
                "flight; flying on the latched home")
        elif gap < 0.5:
            self._home_gap_warned = False

    async def _sub_velocity(self) -> None:
        async for vel in self.system.telemetry.velocity_ned():
            self.state.ground_speed_mps = math.hypot(vel.north_m_s, vel.east_m_s)
            self._touch("velocity")

    async def _sub_attitude(self) -> None:
        async for att in self.system.telemetry.attitude_euler():
            self.state.heading_deg = att.yaw_deg
            self.state.roll_deg = att.roll_deg
            self.state.pitch_deg = att.pitch_deg
            self._touch("attitude")

    async def _sub_attitude_rates(self) -> None:
        """Angular velocity (body-frame) — needed by the dashboard attitude
        indicator and stability indicators."""
        # PX4 doesn't broadcast ATTITUDE_QUATERNION_COV / ANGULAR_VELOCITY by
        # default at a useful rate. Request 10 Hz before subscribing.
        try:
            await self.system.telemetry.set_rate_attitude_quaternion(10.0)
        except Exception as e:
            logger.warning(
                f"[telemetry] set_rate_attitude_quaternion failed: {e} — "
                "attitude-rate stream may run slow (dashboard indicator only)"
            )
        try:
            async for rates in self.system.telemetry.attitude_angular_velocity_body():
                self.state.roll_rate_dps = math.degrees(rates.roll_rad_s)
                self.state.pitch_rate_dps = math.degrees(rates.pitch_rad_s)
                self.state.yaw_rate_dps = math.degrees(rates.yaw_rad_s)
                self._touch("attitude_rates")
        except Exception as e:
            logger.warning(f"[telemetry] attitude rates stream unavailable: {e}")

    async def _sub_actuator_outputs(self) -> None:
        """PWM/normalized outputs sent to actuator group 0 (motors+servos).

        On PX4 multirotor the first 4 actuators are motors (throttle 0..1),
        and additional actuators (servo for drop release etc.) follow. The
        dashboard's ThrottleBars / ServoChannels widgets read from this list.

        ACTUATOR_OUTPUT_STATUS isn't streamed by PX4 SITL by default — request
        5 Hz via set_rate_actuator_output_status before subscribing.
        """
        try:
            await self.system.telemetry.set_rate_actuator_output_status(5.0)
        except Exception as e:
            logger.warning(f"[telemetry] could not request actuator rate: {e}")
        try:
            async for out in self.system.telemetry.actuator_output_status():
                # out.actuator is a list of float (normalized for motors;
                # PWM µs for servos depending on PX4 build). We store raw.
                actuators = list(out.actuator) if out.actuator else []
                # Convert any [0..1] motor values to PWM µs for a uniform UI.
                # Heuristic: values <= 1.5 are normalized (multirotor motor);
                # > 1.5 means already in PWM µs (servo channel).
                pwm: list[int] = []
                for v in actuators:
                    if v <= 1.5:
                        pwm.append(int(1000 + 1000 * max(0.0, min(1.0, float(v)))))
                    else:
                        pwm.append(int(v))
                self.state.servo_pwm_us = pwm
                self._touch("actuator_outputs")
        except Exception as e:
            logger.warning(f"[telemetry] actuator outputs stream unavailable: {e}")

    async def _sub_battery(self) -> None:
        # 2026-08-28 (KMITL trial): the pack reading reached the mission every
        # ~30 s — the FC's TELEM2 instance was throttled to MAV_1_RATE=1200 B/s
        # and PX4 scaled every stream down to fit. Ask for 1 Hz explicitly on
        # top of fixing the board value (tools/preflight_params.py BOARD).
        try:
            await self.system.telemetry.set_rate_battery(1.0)
        except Exception as e:
            logger.warning(f"[telemetry] set_rate_battery failed: {e} — battery "
                           "may update slowly; check MAV_1_RATE on the board")
        async for bat in self.system.telemetry.battery():
            # MAVSDK Battery.remaining_percent is already 0..100
            # (telemetry.proto: "range: 0% to 100%") — unchanged 2.x→3.x, and
            # pyproject pins mavsdk>=3,<4. The old `raw*100 if raw<=1.0` rescale
            # corrupted a genuine 1% reading into 100% — defeating low-battery RTH
            # at the worst moment. Trust the documented 0..100 contract; no rescale.
            self.state.battery_percent = float(bat.remaining_percent)
            self.state.battery_voltage_v = bat.voltage_v
            self.state.battery_current_a = float(
                getattr(bat, "current_battery_a", math.nan))
            self._touch("battery")

    async def _sub_armed(self) -> None:
        async for armed in self.system.telemetry.armed():
            if bool(armed) != self.state.is_armed:
                # every arming edge (either direction) re-opens the latch —
                # PX4 re-captures home at arm and at disarm and those are
                # the captures we want; only in-flight rewrites are refused.
                self._airborne_since_arm = False
            self.state.is_armed = armed
            self._maybe_latch_home()
            self._touch("armed")

    async def _sub_flight_mode(self) -> None:
        async for mode in self.system.telemetry.flight_mode():
            self.state.flight_mode = mode.name
            self._touch("flight_mode")

    async def _sub_landed_state(self) -> None:
        async for ls in self.system.telemetry.landed_state():
            self.state.landed_state = ls.name
            if self.state.is_armed and ls.name in ("TAKING_OFF", "IN_AIR", "LANDING"):
                self._airborne_since_arm = True
            self._touch("landed_state")

    async def _sub_gps_info(self) -> None:
        async for info in self.system.telemetry.gps_info():
            self.state.gps_fix_type = info.fix_type.value
            self.state.gps_satellites = info.num_satellites
            self._touch("gps_info")

    async def _sub_vehicle_clock(self) -> None:
        """The aircraft's own time base (orchestrator/flight_clock.py).

        raw_gps rather than imu/odometry deliberately: PX4 stamps all three
        with hrt_absolute_time(), but raw_gps streams at a few Hz instead of
        tens-to-hundreds, and a mission clock whose deadlines are measured in
        seconds does not need — and should not pay gRPC for — an IMU rate.
        A stalled stream is safe: FlightClock falls back to the wall.
        """
        async for gps in self.system.telemetry.raw_gps():
            self.state.vehicle_time_s = gps.timestamp_us / 1e6
            self._touch("vehicle_clock")

    async def _sub_rc_status(self) -> None:
        # MAVSDK's Telemetry plugin does not expose the dedicated telemetry
        # radio's RADIO_STATUS RSSI directly. As the closest proxy for
        # "data-link health" we use RC link signal strength — when the safety
        # pilot's RC is healthy the operator can override; when it weakens,
        # the bird is increasingly on its own. The safety watchdog uses this
        # as a degraded-link trigger; an absolute RADIO_STATUS hook would
        # require pymavlink alongside MAVSDK, which we deliberately avoid.
        #
        # `was_available_once` distinguishes "no RC has ever been seen"
        # (SITL with no joystick — datalink_rssi stays -1, safety check is a
        # no-op) from "RC was paired but signal dropped" (datalink_rssi=0,
        # safety check fires sustained-loss RTH after threshold). Without
        # this gate, SITL falsely triggers RTH within 5 s of takeoff.
        async for rc in self.system.telemetry.rc_status():
            self.state.rc_signal_available = rc.is_available
            if rc.is_available:
                # mavsdk-python: signal_strength_percent is 0..100
                self.state.datalink_rssi = int(rc.signal_strength_percent)
            elif rc.was_available_once:
                # Had RC, lost it — degraded link.
                self.state.datalink_rssi = 0
            else:
                # No RC ever connected (SITL default). Sentinel = -1 means
                # "not available, do not evaluate"; safety.py honours this.
                self.state.datalink_rssi = -1
            self._touch("rc_status")

    async def _sub_health(self) -> None:
        """EKF / calibration / armable health — the pre-flight readiness gate
        (orchestrator.preflight) reads these. MAVSDK streams Health at ~1 Hz;
        `is_armable` is PX4's composite pre-arm verdict, the rest explain it."""
        try:
            async for h in self.system.telemetry.health():
                self.state.is_armable = h.is_armable
                self.state.is_global_position_ok = h.is_global_position_ok
                self.state.is_local_position_ok = h.is_local_position_ok
                self.state.is_home_position_ok = h.is_home_position_ok
                self.state.is_gyrometer_calibrated = h.is_gyrometer_calibration_ok
                self.state.is_accelerometer_calibrated = h.is_accelerometer_calibration_ok
                self.state.is_magnetometer_calibrated = h.is_magnetometer_calibration_ok
                self._touch("health")
        except Exception as e:
            logger.warning(f"[telemetry] health stream unavailable: {e}")
