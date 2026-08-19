"""Pymavlink raw-message subscriber — fills in the fields MAVSDK doesn't expose.

MAVSDK Python's Telemetry plugin already gives us position, velocity, GPS,
battery, flight mode, RC, and attitude (roll/pitch/yaw + body rates) — but NOT
per-servo PWM, per-ESC current/RPM, or the running consumed-mAh counter. The
dashboard's motor bars, servo-channel, and consumed-energy widgets need those.
So we run a parallel pymavlink listener on a dedicated UDP port that subscribes
to just:

    - SERVO_OUTPUT_RAW (msg 36) — servo1_raw..servo16_raw PWM µs
    - ESC_STATUS      (msg 291) — per-channel current/RPM/voltage
    - BATTERY_STATUS  (msg 147) — current_consumed mAh

We do NOT read ATTITUDE here — MAVSDK's attitude_euler already writes roll/pitch
(see telemetry.py), and a second writer would fight it.

SITL: PX4 broadcasts its GCS link to udp 14550, so point this at 14550 (no local
QGC bound). Real CM4 / HITL: mavlink-router fans a dedicated [UdpEndpoint raw]
127.0.0.1:14551 (see cm4/launch_flight.sh). Config `connection.raw_telemetry_port`
selects the port; 0/absent disables the listener entirely.

This module does NOT replace MAVSDK; it augments CurrentTelemetry with fields
the dashboard needs. Every SAFETY-ACTING check in the watchdog reads
MAVSDK-populated fields (battery_percent, datalink_rssi, gps_fix_type, …), so a
pymavlink failure cannot ground a healthy aircraft.

That property is unconditional again as of 2026-08-17. A motor-health check
read `esc_current_a` from here for one day (2026-08-16 to 08-17); it went away
when the bench showed this airframe's ESCs are PWM-only with no telemetry lead,
so `esc_current_a` never fills on this aircraft. The field stays for the
dashboard to display whatever a future telemetry-capable ESC would send —
nothing in the flight path reads it.
"""

from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING, Any

from loguru import logger

if TYPE_CHECKING:
    from .telemetry import CurrentTelemetry

DEFAULT_RAW_PORT = 14551
DEFAULT_BIND_HOST = "0.0.0.0"
_SERVO_CHANNELS = 16  # SERVO_OUTPUT_RAW carries servo1..servo16
_ESC_CHANNELS = 8     # PX4 reserves 8 ESC slots; the hexa uses 6, so slots 6..7
                      # stay 0.0 and are NOT motors. The ESC lists are dashboard
                      # display only now — the motor-health check that had to
                      # slice them to the real rotor count was removed 2026-08-17.
# The MAVLink message types we subscribe to (ATTITUDE is deliberately absent —
# MAVSDK owns roll/pitch; see the module docstring).
_MSG_TYPES = ("SERVO_OUTPUT_RAW", "ESC_STATUS", "BATTERY_STATUS")


class RawMavlinkSubscriber:
    """Listens on a dedicated UDP port for the MAVLink messages MAVSDK skips.

    Lifecycle mirrors `TelemetrySubscriber`: `start()` spawns a background
    task that loops on `recv_match()`; `stop()` cancels it. Failure modes
    are non-fatal: if pymavlink can't bind or the port is silent, the
    dashboard widgets degrade gracefully (show NaN / empty) but the
    orchestrator continues uninterrupted.
    """

    def __init__(
        self,
        telemetry_state: CurrentTelemetry,
        udp_port: int = DEFAULT_RAW_PORT,
        bind_host: str = DEFAULT_BIND_HOST,
    ) -> None:
        self.state = telemetry_state
        self.udp_port = udp_port
        self.bind_host = bind_host
        self._task: asyncio.Task[None] | None = None
        self._conn: Any = None

    async def start(self) -> None:
        if self.udp_port <= 0:
            logger.info("[raw_subscriber] disabled (connection.raw_telemetry_port unset)")
            return
        # pymavlink imports are kept local so the rest of the codebase doesn't
        # incur the dep cost if dashboard is disabled.
        try:
            from pymavlink import mavutil
        except ImportError:
            logger.warning(
                "[raw_subscriber] pymavlink not installed; dashboard ESC/servo "
                "widgets will be empty"
            )
            return

        url = f"udpin:{self.bind_host}:{self.udp_port}"
        try:
            self._conn = mavutil.mavlink_connection(
                url,
                source_system=255,        # GCS-class id
                source_component=190,     # generic onboard companion
                input=True,
            )
        except Exception as e:
            logger.warning(
                f"[raw_subscriber] could not bind {url}: {e}. "
                "Dashboard ESC/servo/consumed widgets will be empty."
            )
            return

        logger.info(
            f"[raw_subscriber] listening on {url} for SERVO_OUTPUT_RAW/"
            f"ESC_STATUS/BATTERY_STATUS"
        )
        loop = asyncio.get_running_loop()
        self._task = loop.create_task(self._run(loop))

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass

    async def _run(self, loop: asyncio.AbstractEventLoop) -> None:
        # pymavlink's recv_match is blocking; offload to a thread so we don't
        # stall the asyncio event loop.
        while True:
            msg = await loop.run_in_executor(None, self._recv_one)
            if msg is None:
                # Heartbeat-style poll; let the loop breathe.
                await asyncio.sleep(0.05)
                continue
            self._apply(msg)

    def _recv_one(self) -> Any:
        """Blocking single-message read with a short timeout — returns None on timeout."""
        try:
            return self._conn.recv_match(type=_MSG_TYPES, blocking=True, timeout=0.5)
        except Exception:
            return None

    def _apply(self, msg: Any) -> None:
        """Dispatch a pymavlink message into CurrentTelemetry fields."""
        t = self.state
        mtype = msg.get_type()
        if mtype == "SERVO_OUTPUT_RAW":
            # PX4 emits servo1_raw..servo16_raw (0..2500 µs typical 1000..2000)
            pwm = []
            for i in range(1, _SERVO_CHANNELS + 1):
                pwm.append(int(getattr(msg, f"servo{i}_raw", 0)))
            t.servo_pwm_us = pwm
        elif mtype == "ESC_STATUS":
            # ESC_STATUS reports for `index` to `index+3` (block of 4 ESCs per msg).
            # We accumulate into a fixed 8-slot list; later messages overwrite.
            if not t.esc_rpm:
                t.esc_rpm = [0] * _ESC_CHANNELS
                t.esc_current_a = [0.0] * _ESC_CHANNELS
                t.esc_voltage_v = [0.0] * _ESC_CHANNELS
            base = int(getattr(msg, "index", 0))
            rpm = getattr(msg, "rpm", [0, 0, 0, 0])
            current = getattr(msg, "current", [0.0, 0.0, 0.0, 0.0])
            voltage = getattr(msg, "voltage", [0.0, 0.0, 0.0, 0.0])
            for offset in range(4):
                idx = base + offset
                if 0 <= idx < _ESC_CHANNELS:
                    t.esc_rpm[idx] = int(rpm[offset]) if offset < len(rpm) else 0
                    t.esc_current_a[idx] = float(current[offset]) if offset < len(current) else 0.0
                    t.esc_voltage_v[idx] = float(voltage[offset]) if offset < len(voltage) else 0.0
        elif mtype == "BATTERY_STATUS":
            consumed = getattr(msg, "current_consumed", -1)
            if consumed >= 0:
                t.battery_consumed_mah = float(consumed)
                t.battery_consumed_monotonic = time.monotonic()
        t.last_update_monotonic = time.monotonic()
