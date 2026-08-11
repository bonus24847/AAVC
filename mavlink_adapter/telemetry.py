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
    lat: float = math.nan
    lon: float = math.nan
    alt_m: float = math.nan
    relative_alt_m: float = math.nan
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

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._tasks = [
            loop.create_task(self._sub_connection()),
            loop.create_task(self._sub_position()),
            loop.create_task(self._sub_velocity()),
            loop.create_task(self._sub_attitude()),
            loop.create_task(self._sub_attitude_rates()),
            loop.create_task(self._sub_actuator_outputs()),
            loop.create_task(self._sub_battery()),
            loop.create_task(self._sub_armed()),
            loop.create_task(self._sub_flight_mode()),
            loop.create_task(self._sub_gps_info()),
            loop.create_task(self._sub_vehicle_clock()),
            loop.create_task(self._sub_rc_status()),
            loop.create_task(self._sub_health()),
        ]
        logger.info(f"[telemetry] subscribed to {len(self._tasks)} streams")

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        logger.info("[telemetry] stopped")

    def _touch(self) -> None:
        self.state.last_update_monotonic = time.monotonic()

    async def _sub_connection(self) -> None:
        async for st in self.system.core.connection_state():
            self.state.is_connected = st.is_connected
            self._touch()

    async def _sub_position(self) -> None:
        async for pos in self.system.telemetry.position():
            self.state.lat = pos.latitude_deg
            self.state.lon = pos.longitude_deg
            self.state.alt_m = pos.absolute_altitude_m
            self.state.relative_alt_m = pos.relative_altitude_m
            self._touch()

    async def _sub_velocity(self) -> None:
        async for vel in self.system.telemetry.velocity_ned():
            self.state.ground_speed_mps = math.hypot(vel.north_m_s, vel.east_m_s)
            self._touch()

    async def _sub_attitude(self) -> None:
        async for att in self.system.telemetry.attitude_euler():
            self.state.heading_deg = att.yaw_deg
            self.state.roll_deg = att.roll_deg
            self.state.pitch_deg = att.pitch_deg
            self._touch()

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
                self._touch()
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
                self._touch()
        except Exception as e:
            logger.warning(f"[telemetry] actuator outputs stream unavailable: {e}")

    async def _sub_battery(self) -> None:
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
            self._touch()

    async def _sub_armed(self) -> None:
        async for armed in self.system.telemetry.armed():
            self.state.is_armed = armed
            self._touch()

    async def _sub_flight_mode(self) -> None:
        async for mode in self.system.telemetry.flight_mode():
            self.state.flight_mode = mode.name
            self._touch()

    async def _sub_gps_info(self) -> None:
        async for info in self.system.telemetry.gps_info():
            self.state.gps_fix_type = info.fix_type.value
            self.state.gps_satellites = info.num_satellites
            self._touch()

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
            self._touch()

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
            self._touch()

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
                self._touch()
        except Exception as e:
            logger.warning(f"[telemetry] health stream unavailable: {e}")
