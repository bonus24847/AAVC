"""High-level async drone commands via MAVSDK.

Wraps the verbose MAVSDK calls into a small, mission-oriented API that the
executor calls. Designed for one connected vehicle; multi-vehicle would need
a separate Commander instance per drone.

All methods raise on failure — callers should wrap in try/except and let the
safety layer decide whether to abort, RTH, or ignore.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable, Mapping
from contextlib import asynccontextmanager
from dataclasses import dataclass

from loguru import logger
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionProgress
from mavsdk.mission import MissionPlan as _MissionPlan
from mavsdk.offboard import Attitude, AttitudeRate

# MissionItem is re-exported so the executor can build mission items without
# importing mavsdk directly. The Mission plugin's MissionItem is the right
# primitive for AAVC's static-plan upload: per-item speed_m_s,
# acceptance_radius_m, is_fly_through, loiter_time_s, and a vehicle_action enum.


@dataclass(frozen=True)
class ConnectionConfig:
    # `udpin://` (formerly `udp://`) listens for an incoming MAVLink heartbeat
    # from PX4 SITL or mavlink-router. The old `udp://` scheme is deprecated
    # since mavsdk 2.x; bare `udpin://:port` is rejected — must include the
    # interface (use 0.0.0.0 for all). 14540 is PX4's offboard port.
    system_address: str = "udpin://0.0.0.0:14540"
    connect_timeout_s: float = 15.0
    arming_timeout_s: float = 10.0
    # Release-actuator index — MAV_CMD_DO_SET_ACTUATOR / MAVSDK set_actuator
    # addressing (PX4 output function "Peripheral via Actuator Set N", N=1..6;
    # PX4 has NO MAV_CMD_DO_SET_SERVO handler, discovered 2026-08-11 reading
    # v1.17 source — the old value 9 assumed AUX9..12 DO_SET_SERVO numbering
    # that no code path ever honoured). payload_id 0..3 -> actuator set 1..4;
    # SITL maps them to gz via SIM_GZ_SV_FUNC1..4 (airframe 22000).
    drop_servo_channel: int = 1
    drop_servo_pwm_release: int = 1900
    drop_servo_pwm_hold: int = 1100
    drop_payload_count: int = 1   # cargo release channels onboard — bounds payload_id to
                                  # [0, drop_payload_count). One channel PER EGG SLOT
                                  # (M4, review 2026-07-24): drop_payload's servo channel is
                                  # drop_servo_channel + payload_id, so eggs_aboard eggs need
                                  # drop_payload_count == eggs_aboard release channels, not
                                  # one shared servo — this dataclass default (1) is the
                                  # single-egg-per-flight minimum; the shipping SITL/real
                                  # config sets it to match eggs_aboard (sitl/aavc_config.yaml).
    # Endpoint for the MAV_CMD_DO_SET_SERVO drop FALLBACK (used only if
    # action.set_actuator is rejected). The SITL default GCS side-channel port does
    # NOT exist on real/HITL hardware — override it to a mavlink-router endpoint
    # that reaches the FC (e.g. "udpout:127.0.0.1:14550"). The primary drop path is
    # set_actuator over the normal link, which works on hardware; this is the backstop.
    drop_fallback_endpoint: str = "udpout:127.0.0.1:18570"


# Outer-loop flight tuning applied once at mission start (orchestrator.main).
# Operator intent: fly stably (less overshoot), LIMIT max tilt (anti-flip), LIMIT
# yaw/rotation rate, UNLOCK speed — but still the fastest mission. These are the
# AUTO/position-controller LIMITS; the inner-loop MC_*RATE_* PID gains come from
# the pre-flight System-ID + Autotune module. Starting values — verify in SITL.
# Re-derived 2026-07-22 for the EFT X6100 hexacopter (7.17 kg) that replaced the
# ~2 kg quad; the egg/ceiling/scoring pins below are deliberately unchanged.
# MUST stay byte-identical to sitl/aavc_config.yaml px4_tuning — enforced by
# tests/test_px4_tuning_parity.py.
DEFAULT_PX4_TUNING: dict[str, float] = {
    "MPC_TILTMAX_AIR": 30.0,    # anti-flip: cap auto tilt (deg); PX4 default 45
    "MPC_MAN_TILT_MAX": 30.0,   # anti-flip: cap manual tilt (deg)
    "MC_YAWRATE_MAX": 45.0,     # limit body yaw rate (deg/s); default 200. The hexa's
                                # Izz dwarfs its yaw authority — it cannot chase 50.
    "MPC_YAWRAUTO_MAX": 25.0,   # gentler auto heading slews (deg/s) — the AUTO yaw cap
    "MPC_XY_VEL_MAX": 10.0,     # UNLOCK horizontal speed (m/s); default 12. Top of the
                                # 8-10 band the power-system document specifies — the
                                # PM03D is 60 A and hover already draws ~29 A.
    "MPC_XY_CRUISE": 8.0,       # auto cruise speed (m/s) — slower sweep = more frames
                                # per target = reliable blind discovery, and inside
                                # what 30 deg of tilt can accelerate to (the
                                # g*tan(tilt) bound is mass-independent)
    # No arm/disarm cycle per land-and-drop (operator decision 2026-06-11): the
    # vehicle touches down, drops, and takes off again WHILE ARMED. This also pins
    # PX4's home to the LAUNCH point (home is re-captured at every arming — a
    # re-arm on a target made the final RTL land right back on that target).
    "COM_DISARM_LAND": -1.0,    # disable auto-disarm on landing; default 2 s
    # Which of these AUTO actually reads was checked against the v1.17 source
    # 2026-07-22, correcting the note this comment used to carry: FlightTaskAuto
    # declares MPC_ACC_HOR (defined in multicopter_autonomous_params.c), while
    # MPC_ACC_HOR_MAX belongs to FlightTaskManualPosition. So the mission's real
    # accel cap is MPC_ACC_HOR — 3.0 is what every validated run flew, and it is
    # the knob to raise (toward the g*tan(30 deg) = 5.66 ceiling) if the window
    # ever needs time back. MPC_JERK_AUTO does shape AUTO and was the change that
    # took the transit legs off 58% of theoretical.
    "MPC_ACC_HOR": 3.0,         # THE AUTO horizontal accel cap (m/s²)
    "MPC_ACC_HOR_MAX": 5.0,     # manual position mode only; PX4 default 5
    "MPC_JERK_AUTO": 5.0,       # AUTO jerk (m/s³); PX4 default 4 — 3x the mass of
                                # the old quad spools in and out of cruise slower
    "NAV_ACC_RAD": 2.0,         # waypoint acceptance radius (m) — less overshoot.
                                # NOT widened for speed: the aircraft would cut the
                                # corner at the SCORED transit coordinates.
    # ── XY position/velocity loop: wind-rejection tune for the land-ON precision
    # (G6 touchdown scatter). MUST mirror sitl/aavc_config.yaml px4_tuning — the
    # config overrides this dict, but a config-absent run falls back HERE, and
    # dropping these silently reverts to PX4-stock XY gains. See test_px4_tuning_parity.
    "MPC_XY_P": 1.5,            # position P — stiffer hover hold (default 0.95)
    "MPC_XY_VEL_P_ACC": 3.0,    # velocity P — gust response (default 1.8)
    "MPC_XY_VEL_I_ACC": 0.8,    # velocity I — steady-wind offset (default 0.4)
    "MPC_XY_VEL_D_ACC": 0.3,    # velocity D — damping (default 0.2)
    # ── descent: FAST high, SLOW low. This is the fast CAP up high; tactical_align
    # steps MPC_Z_VEL_MAX_DN down per rung, then LAND_SPEED crawls the touchdown (no
    # slam — protects airframe + payload servo, stops the SITL model punching through). ──
    "MPC_Z_VEL_MAX_DN": 3.0,    # MANUAL/offboard descent cap (m/s); default 1.5 — PX4
                                # reads this only outside AUTO, so it does NOT shape
                                # the mission's descents. See MPC_Z_V_AUTO_DN below.
    "MPC_Z_V_AUTO_DN": 0.4,     # THE AUTO descent speed (m/s); PX4 default 1.5. The
                                # validated pad-approach descent — unpinned, a real 6X
                                # would drop onto the pad 4x faster than anything tested.
                                # mission.py raises it only for the L&R staged descent.
    "MPC_Z_VEL_MAX_UP": 2.0,    # cap AUTO climb (m/s; default 3) — 3 overshot to 19.68 m
                                # under the 20 m ceiling; 2.0 halves the overshoot (~v²)
    # Takeoff climbs to transit_alt - 2 m and mission.py stages the last 2 m at
    # 1 m/s, so this sits inside the margin the climb cap was tuned against.
    # Unpinned it rode the PX4 default 1.5 (measured ~1.0 m/s) — slower than the
    # 2.0 climb cap that was validated. Never above MPC_Z_VEL_MAX_UP.
    "MPC_TKO_SPEED": 2.0,       # takeoff climb speed (m/s); PX4 default 1.5
    "MPC_LAND_SPEED": 0.3,      # slow final touchdown (m/s); default 0.7 — NOT raised:
                                # a global bump made AUTO.LAND climb to 41 m (e02ffa3);
                                # the L&R descent is staged in mission.py instead
    "MPC_LAND_ALT1": 10.0,      # start slowing the descent at 10 m AGL
    "MPC_LAND_ALT2": 5.0,       # final crawl speed below 5 m AGL
    # Any failsafe RTL (geofence breach, datalink loss, watchdog) climbs to
    # RTL_RETURN_ALT first — the PX4 default is 60 m, which would smash through
    # the AAVC 20 m ceiling. Pin it AT the ceiling (rules: transit strictly 20 m).
    "RTL_RETURN_ALT": 20.0,     # failsafe return altitude (m); PX4 default 60
    # Downward rangefinder (Benewake TFmini-S) aids height through the delivery
    # descent and touchdown; 1 is already the 6X default but pin it so a param
    # reset cannot silently drop height aiding. The serial port assignment
    # (SENS_TFMINI_CFG) is a G5 bench decision, deliberately not set from here.
    "EKF2_RNG_CTRL": 1.0,       # fuse the downward rangefinder
    # Optical flow was cut from the project 2026-07-22 — no flow module aboard.
    # PX4 1.17 enables the fusion by default, which only invites a puzzling
    # "flow timeout" health failure at arming.
    "EKF2_OF_CTRL": 0.0,        # optical flow fusion OFF — no such sensor
}


# PX4 params this repo pushes that are declared INT32 on the FC. A float write
# to an INT32 param is rejected outright — seen twice: "applied 23/25" with two
# TIMEOUTs (EKF2_*, 2026-07-22) and SIM_BAT_ENABLE timing out on every SITL boot
# until it was listed here. ONE list for every block we push (tuning, sim
# battery, gimbal), because per-block lists is exactly how the last two were
# missed: a param added to config lands in whichever block, and only a single
# shared list can cover it.
_INT_PARAMS = frozenset({
    "EKF2_RNG_CTRL", "EKF2_OF_CTRL",          # height aiding / flow fusion
    "SIM_BAT_ENABLE",                          # SITL battery simulator on/off
    # Gimbal (VERIFY-AT-G5). MNT_MAN_PITCH is not an angle — PX4 types it INT32
    # because it selects the RC AUX channel that drives pitch (0 = disabled).
    "MNT_MODE_IN", "MNT_MODE_OUT", "MNT_DO_STAB", "MNT_MAN_PITCH",
})

# The pins whose PX4 DEFAULT is out of the competition envelope, i.e. the ones a
# silent apply-failure turns into a rules bust or a broken egg. verify_envelope_pins
# reads these back after the tuning block is pushed. Keep the list short: it is
# "what must be true to fly", not "what we tuned".
_ENVELOPE_PINS = (
    "RTL_RETURN_ALT",     # default 60 m vs the 20 m ceiling — busts it on any RTL
    "MPC_Z_V_AUTO_DN",    # default 1.5 m/s vs 0.4 validated onto the pad
    "COM_DISARM_LAND",    # default 2 s auto-disarms ON the pad mid-sortie
)


def _pwm_to_norm(pwm_us: float, *, pwm_min: float = 1000.0, pwm_max: float = 2000.0) -> float:
    """Convert a servo PWM pulse width (µs) to MAVSDK set_actuator's bipolar
    normalized command in [-1, 1] (-1 = pwm_min, 0 = centre, +1 = pwm_max).

    MAVSDK action.set_actuator expects [-1, 1], NOT pwm/2000: the old mapping
    put BOTH release (1900 -> 0.95) and hold (1100 -> 0.55) in the upper half of
    travel, so the servo may never have actually swung between the two
    positions. Default band is the standard 1000-2000 µs (1500 µs centre);
    confirm the drop servo's real band at the G6 bench test."""
    centre = (pwm_min + pwm_max) / 2.0
    half = (pwm_max - pwm_min) / 2.0
    return max(-1.0, min(1.0, (pwm_us - centre) / half))


class DroneCommander:
    """Thin async facade over MAVSDK for AAVC missions."""

    def __init__(self, config: ConnectionConfig | None = None) -> None:
        self.config = config or ConnectionConfig()
        self.system = System()
        self._connected = False
        # Home MSL altitude in metres, captured from telemetry.home() right
        # after MAVLink heartbeat. Needed because MAVSDK action.goto_location
        # takes ABSOLUTE (MSL) altitude while the rest of the orchestrator
        # speaks AGL relative-altitude. Conversion: msl = home_alt + alt_agl.
        self._home_alt_msl: float | None = None

    def close(self) -> None:
        """Tear down the embedded mavsdk_server subprocess so the process can exit.

        MAVSDK runs mavsdk_server as a subprocess and reads its stdout in a
        NON-daemon thread (``mavsdk.system._LoggingThread``). CPython joins
        non-daemon threads at interpreter shutdown BEFORE the atexit hook that
        would kill the server runs — so that reader blocks forever on the live
        server's pipe and the whole process hangs after the mission completes
        (the post-mission hang, confirmed by a faulthandler dump 2026-06-14:
        main thread parked in ``threading._shutdown`` joining system.py:59).
        Killing the server here closes the pipe → the reader hits EOF and
        exits → interpreter shutdown can finish. Idempotent + defensive: the
        private attr is absent if connect() never ran."""
        proc = getattr(self.system, "_server_process", None)
        kill = getattr(proc, "kill", None)
        if callable(kill):
            try:
                kill()
            except Exception:
                logger.debug("[mavlink] mavsdk_server teardown skipped", exc_info=True)

    async def connect(self) -> None:
        logger.info(f"[mavlink] connecting to {self.config.system_address}")

        # mavsdk-python's System.connect() starts mavsdk_server and then blocks in
        # AsyncPluginManager on `channel.channel_ready()` (gRPC → localhost:50051).
        # mavsdk_server 3.15 does NOT open that gRPC port until it discovers a
        # MAVLink system on the link — so with no vehicle present (PX4 SITL down /
        # mavlink-router not forwarding) channel_ready() never returns and this
        # call hangs FOREVER. connect_timeout_s used to guard only the heartbeat
        # wait below, leaving THIS call uncovered, so the operator got a silent
        # indefinite hang instead of an error. Bound it the same way: run as a
        # task, give up after the timeout, reap the orphaned server (it still
        # holds 14540), and raise a clear, actionable error.
        connect_task = asyncio.create_task(
            self.system.connect(system_address=self.config.system_address)
        )
        done, _ = await asyncio.wait(
            {connect_task}, timeout=self.config.connect_timeout_s
        )
        if connect_task not in done:
            connect_task.cancel()        # don't await — gRPC channel cleanup can hang
            self.close()                 # reap the orphaned mavsdk_server (holds 14540)
            raise RuntimeError(
                f"mavsdk_server gRPC backend not ready after "
                f"{self.config.connect_timeout_s:.0f}s on {self.config.system_address} — "
                "mavsdk_server opens its gRPC port only after it discovers a MAVLink "
                "system, so this usually means no vehicle is present: is PX4 SITL "
                "running, or is mavlink-router forwarding to 14540?"
            )
        connect_task.result()            # re-raise if connect() itself failed

        # NOTE: mavsdk-python's `connection_state()` async iterator does not
        # promptly honour asyncio cancellation — wrapping it with
        # `asyncio.wait_for` doesn't reliably raise TimeoutError when no
        # heartbeat ever arrives (the gRPC stream blocks past cancel). To
        # surface a timeout to the operator we poll the iterator manually
        # against a wall-clock deadline.
        logger.info(
            f"[mavlink] waiting up to {self.config.connect_timeout_s:.0f}s for heartbeat — "
            "start PX4 SITL or check mavlink-router if this hangs"
        )

        async def _watch() -> bool:
            async for state in self.system.core.connection_state():
                if state.is_connected:
                    logger.info("[mavlink] vehicle connection established")
                    return True
            return False

        watch_task = asyncio.create_task(_watch())
        done, _ = await asyncio.wait({watch_task}, timeout=self.config.connect_timeout_s)
        if watch_task not in done:
            watch_task.cancel()
            # Don't await the cancelled task; gRPC stream cleanup can hang.
            raise RuntimeError(
                f"No MAVLink heartbeat after {self.config.connect_timeout_s}s on "
                f"{self.config.system_address} — is PX4 SITL running, or is mavlink-router "
                "forwarding to 14540?"
            )
        if not watch_task.result():
            raise RuntimeError("connection_state() iterator closed without ever connecting")
        self._connected = True

        # Cache home MSL altitude. MAVSDK's telemetry.home() emits the home
        # position (latitude, longitude, absolute_altitude_m) — we keep the
        # MSL altitude so goto() can convert AGL inputs back to MSL.
        # Bounded wait so a SITL without GPS lock doesn't hang forever.
        async def _grab_home() -> float | None:
            async for home in self.system.telemetry.home():
                return float(home.absolute_altitude_m)
            return None

        try:
            self._home_alt_msl = await asyncio.wait_for(_grab_home(), timeout=10.0)
            logger.info(f"[mavlink] cached home MSL altitude: {self._home_alt_msl:.1f} m")
        except asyncio.TimeoutError:
            logger.warning(
                "[mavlink] telemetry.home() timed out after 10 s — goto() will "
                "assume home_alt_msl=0 (correct only at sea level)."
            )
            self._home_alt_msl = 0.0

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        await self.system.action.set_takeoff_altitude(altitude_m)
        # Landing ON a pad keeps the vehicle ARMED (COM_DISARM_LAND=-1), so a
        # mid-sortie climb-out is takeoff-only — arm just once per sortie.
        # (Skipping the mid-sortie re-arm also keeps PX4's home pinned to the
        # launch point while airborne.)
        if await self._is_armed():
            logger.info(f"[mavlink] takeoff (already armed) to {altitude_m} m AGL")
        else:
            logger.info(f"[mavlink] arming + takeoff to {altitude_m} m AGL")
            await self._arm_with_retry()
            # PX4 RE-CAPTURES its home position at this arming — the AGL→MSL
            # frame goto() rides on moves with it. A stale MSL cache from the
            # previous arming drifted the commanded transit altitude by up to
            # ~1.5 m across sorties (SITL G4 2026-07-04) — refresh it now.
            await asyncio.sleep(1.0)   # let the new home propagate to telemetry
            await self._refresh_home_alt()
        await self.system.action.takeoff()
        if not await self._wait_until_altitude_reached(altitude_m):
            # Don't fly on as if cruise altitude was achieved — surface it; the
            # orchestrator's emergency-RTH boundary brings the vehicle down.
            raise RuntimeError(
                f"takeoff did not reach {altitude_m} m AGL within timeout"
            )
        # After takeoff PX4 stays in AUTO.TAKEOFF mode hovering at target alt.
        # `MAV_CMD_DO_REPOSITION` (sent by action.goto_location) is rejected
        # from TAKEOFF mode — the vehicle silently ignores it. Switch to HOLD
        # so subsequent GOTO commands actually move the vehicle. See PX4 forum
        # "Proper way to do a GoTo" + MAVSDK guide (no wait_for_state exists).
        logger.info("[mavlink] takeoff complete — switching to HOLD")
        await self.system.action.hold()
        # Give PX4 ~1 s to settle into the new mode before the next reposition.
        await asyncio.sleep(1.0)

    async def _refresh_home_alt(self) -> None:
        """Re-cache the home MSL altitude after a (re-)arming. Best-effort +
        bounded: on timeout the previous cache is kept (a warning is logged) —
        never fatal to the sortie."""
        async def _grab() -> float | None:
            async for home in self.system.telemetry.home():
                return float(home.absolute_altitude_m)
            return None

        try:
            got = await asyncio.wait_for(_grab(), timeout=5.0)
        except asyncio.TimeoutError:
            logger.warning("[mavlink] home() re-capture timed out — keeping the "
                           "previous MSL cache")
            return
        if got is None:
            return
        if self._home_alt_msl is None or abs(got - self._home_alt_msl) > 0.05:
            logger.info(f"[mavlink] home MSL re-cached after arming: "
                        f"{self._home_alt_msl} → {got:.1f} m")
        self._home_alt_msl = got

    async def _arm_with_retry(
        self, attempts: int = 10, delay_s: float = 1.5
    ) -> None:
        """Arm, retrying a transient COMMAND_DENIED. PX4 briefly refuses to
        re-arm for a second or two right after the auto-disarm on landing (the
        land-detector / disarm sequence is still settling). Without this a
        single denial aborts the whole multi-target sortie at the climb-out
        between targets. Mirrors the retry the SITL verify harnesses use."""
        last: ActionError | None = None
        for i in range(attempts):
            try:
                await self.system.action.arm()
                if i:
                    logger.info(f"[mavlink] armed on attempt {i + 1}/{attempts}")
                return
            except ActionError as e:
                last = e
                logger.warning(
                    f"[mavlink] arm denied ({i + 1}/{attempts}): {e} — "
                    f"retrying in {delay_s:.1f}s"
                )
                await asyncio.sleep(delay_s)
        raise RuntimeError(f"arm failed after {attempts} attempts: {last}")

    async def goto(
        self, lat: float, lon: float, alt_m: float, yaw_deg: float = float("nan")
    ) -> None:
        """Reposition to (lat, lon, alt_m). `alt_m` is RELATIVE (AGL) — the
        orchestrator + plan speak relative altitudes throughout. We convert
        to absolute MSL here because action.goto_location() takes MSL.
        PX4 treats NaN yaw as "hold current heading".
        """
        if self._home_alt_msl is None:
            raise RuntimeError(
                "home altitude unknown — connect() must be awaited before goto()."
            )
        msl_alt = self._home_alt_msl + alt_m
        logger.info(
            f"[mavlink] goto ({lat:.6f}, {lon:.6f}, AGL={alt_m:.1f} m → "
            f"MSL={msl_alt:.1f} m, yaw={yaw_deg})"
        )
        await self.system.action.goto_location(lat, lon, msl_alt, yaw_deg)

    async def get_param_float(self, name: str) -> float:
        """Read a PX4 float parameter (e.g. MPC_XY_CRUISE)."""
        return float(await self.system.param.get_param_float(name))

    async def set_param_float(self, name: str, value: float) -> None:
        """Set a PX4 float parameter."""
        await self.system.param.set_param_float(name, float(value))

    async def get_param_int(self, name: str) -> int:
        """Read a PX4 int parameter (e.g. MIS_TKO_LAND_REQ). PX4 distinguishes
        int from float param types over MAVLink, so int params need a typed
        accessor — reading them via the float API returns a wrong value."""
        return int(await self.system.param.get_param_int(name))

    async def set_param_int(self, name: str, value: int) -> None:
        """Set a PX4 int parameter (e.g. MIS_TKO_LAND_REQ=0 to allow a
        mission with no landing waypoint)."""
        await self.system.param.set_param_int(name, int(value))

    async def _apply_params(self, params: Mapping[str, float], kind: str) -> int:
        """Push a block of PX4 params, one write per key, and return how many
        landed. INT32-typed names (`_INT_PARAMS`) go via set_param_int; the rest
        via set_param_float. Per-param failure is logged and tolerated so one
        unknown key never aborts the mission — the CALLER decides whether the
        count it gets back is acceptable."""
        applied = 0
        for name, value in params.items():
            try:
                if name in _INT_PARAMS:
                    await self.set_param_int(name, int(value))
                else:
                    await self.set_param_float(name, float(value))
                applied += 1
            except Exception as e:  # noqa: BLE001 — a bad/unknown param must not abort
                logger.warning(f"[mavlink] {kind} param {name}={value} failed: {e}")
        logger.info(f"[mavlink] applied {applied}/{len(params)} {kind} params")
        return applied

    async def apply_param_overrides(self, overrides: Mapping[str, float]) -> int:
        """Best-effort apply a set of PX4 params at mission start (the
        outer-loop flight tuning — anti-flip tilt cap, yaw-rate limit, speed
        unlock, tighter waypoint acceptance). Returns the count successfully
        applied — several of these are safety pins (RTL_RETURN_ALT,
        MPC_Z_V_AUTO_DN), so callers must check the count rather than assume."""
        return await self._apply_params(overrides, "PX4 tuning")

    async def verify_envelope_pins(
        self, expected: Mapping[str, float], *, tol: float = 1e-3,
    ) -> list[str]:
        """Read back the pins whose PX4 DEFAULTS would fly an illegal mission,
        and report the ones the FC is not actually holding.

        Applying params is best-effort by necessity (one unknown key must not
        abort a sortie), but that tolerance hid a real failure mode: a stale
        mavsdk_server holding the ports makes EVERY param RPC fail, the log says
        "applied 0/24", and the mission then flies on PX4 defaults —
        RTL_RETURN_ALT 60 m (3x the rules ceiling, and the safety watchdog's own
        RTH takes that path) and MPC_Z_V_AUTO_DN 1.5 m/s onto the pad, 4x
        anything validated with an egg aboard. So the pins that define the
        envelope get the same read-back treatment `set_battery_failsafe` already
        gives the failsafe action. Returns a list of human-readable mismatches —
        empty means the FC is holding the envelope.
        """
        bad: list[str] = []
        for name in _ENVELOPE_PINS:
            if name not in expected:
                continue
            want = float(expected[name])
            try:
                got = float(await self.get_param_float(name))
            except Exception as e:  # noqa: BLE001 — an unreadable pin IS a finding
                bad.append(f"{name} unreadable ({e})")
                continue
            if abs(got - want) > tol:
                bad.append(f"{name}={got:g} (want {want:g})")
        return bad

    # ── offboard primitives (pre-flight System-ID sweep; NOT the scored sortie) ──
    # Thin wrappers over the MAVSDK offboard plugin used by orchestrator.sysid_sweep
    # to inject a frequency-sweep excitation. Offboard streaming must be primed with
    # a setpoint BEFORE start() and kept fed at ≥2 Hz, or PX4 rejects/exits offboard.

    async def offboard_start(self) -> None:
        await self.system.offboard.start()

    async def offboard_stop(self) -> None:
        await self.system.offboard.stop()

    async def set_attitude(
        self, roll_deg: float, pitch_deg: float, yaw_deg: float, thrust: float
    ) -> None:
        """Attitude-ANGLE setpoint (self-levelling keeps thrust mostly vertical
        → altitude holds during an angle chirp)."""
        await self.system.offboard.set_attitude(
            Attitude(roll_deg, pitch_deg, yaw_deg, thrust)
        )

    async def set_attitude_rate(
        self, roll_dps: float, pitch_dps: float, yaw_dps: float, thrust: float
    ) -> None:
        """Body angular-rate setpoint (deg/s) — the sharper high-frequency chirp."""
        await self.system.offboard.set_attitude_rate(
            AttitudeRate(roll_dps, pitch_dps, yaw_dps, thrust)
        )

    async def run_mission(
        self,
        items: list[MissionItem],
        on_progress: "Callable[[MissionProgress], None] | None" = None,
        watchdog_should_stop: "Callable[[], bool] | None" = None,
    ) -> MissionProgress:
        """Upload and execute a Mission, blocking until finished or aborted.

        Architecture per research (2026-05-25): use the Mission plugin for
        the static plan because it provides FC-authoritative arrival
        signaling via PX4's MISSION_ITEM_REACHED MAVLink message
        (https://discuss.px4.io/t/check-if-goto-location-has-reached-target-position/27079).
        action.goto_location has no arrival callback and always sends
        param1=-1 (locked at MPC_XY_CRUISE) — wrong primitive for the
        static plan.

        Returns the final MissionProgress for caller inspection.
        Raises if upload/start fail, or if the watchdog says abort.

        `on_progress` is invoked on every progress update so the orchestrator
        can advance state.command_pointer and run safety checks.

        `watchdog_should_stop` is polled on every progress event; returning
        True triggers `mission.pause_mission()` + `clear_mission()` and the
        coroutine returns the latest progress. Caller (safety watchdog) is
        responsible for whatever recovery action follows (RTL, LAND, etc).
        """
        if not items:
            raise ValueError("run_mission: items must be non-empty")
        plan = _MissionPlan(items)
        # Clear any residual mission state in PX4 before uploading the new
        # plan. Without this, a second `upload_mission` in the same SITL
        # session can leave PX4 with `current_mission_index` pointing at
        # the previous mission's last item — `start_mission()` then jumps
        # straight to that item, reports it complete, and the vehicle
        # loiters with "No valid mission available". Observed mid-session
        # 2026-05-26 between debug reruns.
        try:
            await self.system.mission.clear_mission()
        except Exception as e:
            logger.debug(f"[mavlink] clear_mission pre-upload: {e}")
        logger.info(f"[mavlink] uploading mission ({len(items)} items)")
        await self.system.mission.set_return_to_launch_after_mission(False)
        await self.system.mission.upload_mission(plan)

        # Arm before starting — start_mission() does NOT auto-arm.
        if not (await self._is_armed()):
            # Wait for vehicle health to converge before arming. PX4 refuses
            # arm() with COMMAND_DENIED if EKF/GPS/preflight checks haven't
            # finished — which takes ~10-15s after first heartbeat on cold
            # SITL boot. Without this gate, an orchestrator launched right
            # after `make sitl` reliably fails on its very first arm.
            await self._wait_arm_ready(timeout_s=30.0)
            logger.info("[mavlink] arming for mission start")
            await self.system.action.arm()

        logger.info("[mavlink] starting mission")
        await self._start_mission_with_retry(self.system.mission.start_mission, "mission")

        latest = MissionProgress(0, len(items))
        async for progress in self.system.mission.mission_progress():
            latest = progress
            logger.info(
                f"[mavlink] mission_progress current={progress.current} total={progress.total}"
            )
            if on_progress is not None:
                try:
                    on_progress(progress)
                except Exception:
                    logger.exception("[mavlink] on_progress callback raised; continuing")
            if watchdog_should_stop is not None and watchdog_should_stop():
                logger.warning("[mavlink] watchdog requested mission stop")
                await self._abort_mission()
                return latest
            if (
                progress.current >= progress.total
                or await self.system.mission.is_mission_finished()
            ):
                logger.info("[mavlink] mission finished")
                return latest
        return latest

    async def _abort_mission(self) -> None:
        try:
            await self.system.mission.pause_mission()
        except Exception as e:
            logger.warning(f"[mavlink] pause_mission failed: {e}")
        try:
            await self.system.mission.clear_mission()
        except Exception as e:
            logger.warning(f"[mavlink] clear_mission failed: {e}")

    async def _start_mission_with_retry(
        self, start: "Callable[[], Awaitable[None]]", label: str,
        attempts: int = 5, backoff_s: float = 5.0,
    ) -> None:
        """Call a mission start() coroutine-factory, retrying on a transient
        rejection. PX4 returns DENIED on MISSION_START when the vehicle is briefly
        in a health/failsafe flap (e.g. an in-air EKF/compass wobble on a long
        flight) — which clears within seconds. Without the retry a single transient
        DENIED aborts the whole segment (observed on the fixed-wing landing approach,
        2026-06-01). ``start`` is the bound start_mission method (called fresh each
        attempt — a coroutine can't be awaited twice). Re-raises the last error if
        every attempt fails (a persistent rejection is a real failure)."""
        last: Exception | None = None
        for i in range(attempts):
            try:
                await start()
                return
            except Exception as e:    # noqa: BLE001 — surfaced after the retry budget
                last = e
                logger.warning(
                    f"[mavlink] start {label} attempt {i + 1}/{attempts} rejected: "
                    f"{e!s:.80} — retrying in {backoff_s:.0f}s"
                )
                if i < attempts - 1:
                    await asyncio.sleep(backoff_s)
        assert last is not None
        raise last

    async def _is_armed(self) -> bool:
        async for armed in self.system.telemetry.armed():
            return bool(armed)
        return False

    async def _is_landed(self) -> bool:
        async for in_air in self.system.telemetry.in_air():
            return not bool(in_air)
        return False

    async def _wait_until_landed(self, timeout_s: float = 90.0, poll_s: float = 0.5) -> bool:
        """Wait until PX4's land detector reports on-ground (in_air False).
        Same wall-clock-deadline poll as :meth:`_wait_until_disarmed` — needed
        because with COM_DISARM_LAND=-1 the vehicle never auto-disarms, so
        touchdown (not disarm) is the landing signal."""
        deadline = asyncio.get_running_loop().time() + timeout_s
        while True:
            try:
                if await self._is_landed():
                    return True
            except Exception:
                pass
            if asyncio.get_running_loop().time() > deadline:
                logger.warning(
                    f"[mavlink] touchdown not observed after {timeout_s:.0f}s — "
                    "abandoning the wait")
                return False
            await asyncio.sleep(poll_s)

    async def _wait_arm_ready(self, timeout_s: float = 30.0, poll_s: float = 0.5) -> None:
        """Block until PX4's preflight health checks pass, or timeout.

        Reads `telemetry.health()` which exposes EKF, GPS, accel, gyro,
        magnetometer, level calibration and home-position-set flags.
        SITL boot reaches health_all_ok ~10-15s after first heartbeat.

        The timeout wraps the whole stream wait (asyncio.wait_for): health() only
        advances a per-emission deadline when it emits, so a deadline tested
        *inside* the `async for` never fires if the stream stalls — the same
        stalled-telemetry trap that hung the climb-out wait (see
        :meth:`_wait_until_altitude_reached`). On timeout we warn and return so the
        caller still attempts the arm; the arm command itself is the real gate."""
        last_flags = "no telemetry received"

        async def _watch() -> None:
            nonlocal last_flags
            async for h in self.system.telemetry.health():
                last_flags = (
                    f"global_ok={h.is_global_position_ok}, "
                    f"home_ok={h.is_home_position_ok}, "
                    f"local_ok={h.is_local_position_ok}, "
                    f"armable={h.is_armable}"
                )
                if (
                    h.is_global_position_ok
                    and h.is_home_position_ok
                    and h.is_local_position_ok
                    and h.is_armable
                ):
                    logger.info("[mavlink] health checks pass — arm allowed")
                    return

        try:
            await asyncio.wait_for(_watch(), timeout=timeout_s)
        except asyncio.TimeoutError:
            logger.warning(
                "[mavlink] health-check timeout — attempting arm anyway "
                f"({last_flags})"
            )

    async def drop_payload(self, payload_id: int = 0) -> None:
        """Release a payload by pulsing the drop servo."""
        if not 0 <= payload_id < self.config.drop_payload_count:
            raise ValueError(
                f"payload_id {payload_id} out of range "
                f"[0, {self.config.drop_payload_count}) — refusing to address an "
                "unconfigured servo channel"
            )
        ch = self.config.drop_servo_channel + payload_id
        logger.info(f"[mavlink] drop payload {payload_id} (actuator set {ch})")
        # Primary path: MAVSDK action.set_actuator(index, value) — which is
        # MAV_CMD_DO_SET_ACTUATOR on the wire, the ONLY release command PX4
        # implements (FunctionActuatorSet; there is no DO_SET_SERVO handler).
        # index must be 1..6 (param7=0 addressing) — guaranteed by
        # drop_servo_channel=1 + the payload_id bounds check above.
        # The PWM is mapped to the bipolar [-1, 1] convention via _pwm_to_norm
        # (1900->+0.8 release, 1100->-0.8 hold); the hold re-latch keeps the
        # mechanism closed for the next resupply.
        try:
            await self.system.action.set_actuator(
                ch, _pwm_to_norm(self.config.drop_servo_pwm_release)
            )
            await asyncio.sleep(0.6)
            await self.system.action.set_actuator(
                ch, _pwm_to_norm(self.config.drop_servo_pwm_hold)
            )
        except ActionError as e:
            logger.warning(
                f"[mavlink] drop via set_actuator rejected: {e}; "
                "falling back to MAV_CMD_DO_SET_ACTUATOR (pymavlink side-channel)"
            )
            await self._drop_via_set_actuator(ch)

    async def _drop_via_set_actuator(
        self, actuator_index: int, target_endpoint: str | None = None,
    ) -> None:
        """Fallback drop: pulse the release actuator via a raw COMMAND_LONG
        MAV_CMD_DO_SET_ACTUATOR over the pymavlink GCS side-channel, for PX4
        builds that reject MAVSDK action.set_actuator when it arrives from a
        non-GCS MAVLink source. Same wire command as the primary path (PX4 has
        no DO_SET_SERVO handler — this used to send one, which PX4 silently
        dropped, making the 'fallback' a no-op). Sends release, dwell, hold.

        param7 = 0 (index page), param1..6 = actuator-set values with NaN for
        every channel this drop must NOT touch (FunctionActuatorSet skips
        non-finite params). Runs the blocking pymavlink I/O in a thread so the
        event loop is free."""
        endpoint = target_endpoint or self.config.drop_fallback_endpoint

        def _send() -> None:
            import math as _math
            import time as _time

            from pymavlink import mavutil
            mav = mavutil.mavlink_connection(
                endpoint, source_system=255, source_component=0,
            )
            try:
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
                )
                if mav.wait_heartbeat(timeout=5) is None:
                    raise RuntimeError(f"no heartbeat from {endpoint} in 5s")
                tgt_sys = mav.target_system or 1
                tgt_comp = mav.target_component or 1
                for pwm in (self.config.drop_servo_pwm_release,
                            self.config.drop_servo_pwm_hold):
                    params = [_math.nan] * 6
                    params[actuator_index - 1] = _pwm_to_norm(pwm)
                    mav.mav.command_long_send(
                        tgt_sys, tgt_comp,
                        mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR, 0,
                        *params, 0.0,
                    )
                    _time.sleep(0.6)
            finally:
                mav.close()

        await asyncio.to_thread(_send)

    async def land(self, *, disarm: bool = True) -> None:
        """AUTO.LAND at the current position.

        Auto-disarm-on-land is disabled fleet-wide (``COM_DISARM_LAND = -1``) so
        a land-and-drop needs NO arm/disarm cycle per target — and PX4's home
        stays pinned to the launch point (home is re-captured at every arming).

        ``disarm=False``: touch down and STAY ARMED (the per-target landing;
        the caller watches altitude/settle and takes off again after the drop).
        ``disarm=True``: the terminal landing — wait for touchdown, disarm
        explicitly, and confirm."""
        logger.info(f"[mavlink] landing (disarm={disarm})")
        await self.system.action.land()
        if not disarm:
            return
        await self._wait_until_landed()
        try:
            await self.system.action.disarm()
        except Exception as e:
            logger.warning(f"[mavlink] explicit disarm failed: {e}")
        await self._wait_until_disarmed(timeout_s=30.0)

    async def rth(self) -> None:
        """Return to the launch point AND land there, then disarm.

        PX4 RTL returns to the launch position and lands; with the default
        RTL_LAND_DELAY it may loiter at home instead of touching down, so we
        force a 0 s delay (land immediately at the launch point). With
        auto-disarm disabled (COM_DISARM_LAND=-1) the vehicle stays armed after
        touchdown, so disarm explicitly once landed. Blocking until disarm lets
        the mission terminate only once the vehicle is actually down — callers
        must NOT append a separate LAND after rth()."""
        logger.info("[mavlink] return-to-launch + land")
        try:
            await self.system.param.set_param_float("RTL_LAND_DELAY", 0.0)
        except Exception as e:
            logger.warning(f"[mavlink] could not set RTL_LAND_DELAY=0: {e}")
        await self.system.action.return_to_launch()
        if await self._wait_until_landed(timeout_s=180.0):
            try:
                await self.system.action.disarm()
            except Exception as e:
                logger.warning(f"[mavlink] explicit disarm failed: {e}")
        await self._wait_until_disarmed()

    async def abort(self) -> None:
        """Emergency stop — kill motors immediately. Use only when crash is imminent."""
        logger.warning("[mavlink] ABORT — killing motors")
        await self.system.action.kill()

    async def upload_geofence(
        self, polygon: list[tuple[float, float]], *, inclusion: bool = True,
    ) -> int:
        """Upload a per-mission onboard PX4 geofence (an INCLUSION polygon by
        default — the vehicle must stay inside it) via the geofence plugin.
        Returns the vertex count. Pairs with ``set_geofence_action_rtl`` so a
        breach triggers an FC-level RTL even if the companion dies.

        ``polygon`` is a list of (lat, lon) vertices (≥ 3). Clears any prior
        fence first so a stale geofence from a previous mission can't linger.
        """
        from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon

        if len(polygon) < 3:
            raise ValueError(f"geofence polygon needs >= 3 vertices, got {len(polygon)}")
        points = [Point(float(lat), float(lon)) for lat, lon in polygon]
        ftype = FenceType.INCLUSION if inclusion else FenceType.EXCLUSION
        data = GeofenceData([Polygon(points, ftype)], [])
        try:
            await self.system.geofence.clear_geofence()
        except Exception as e:
            logger.debug(f"[mavlink] geofence clear pre-upload: {e}")
        await self.system.geofence.upload_geofence(data)
        logger.info(
            f"[mavlink] uploaded {ftype.name} geofence ({len(points)} vertices) — "
            "pair with set_geofence_action_rtl() for FC-level breach RTL"
        )
        return len(points)

    async def set_geofence_action_rtl(self) -> None:
        """Make PX4 RTL on geofence breach (vs the default warn-only).

        Reads the param back after setting and raises if it didn't stick — a
        clean MAVSDK ACK does not guarantee PX4 stored the value. The caller
        records an anomaly on failure so a lost onboard geofence isn't silent."""
        await self.system.param.set_param_int("GF_ACTION", 2)  # 2 = RTL
        readback = await self.system.param.get_param_int("GF_ACTION")
        if readback != 2:
            raise RuntimeError(
                f"GF_ACTION readback={readback}, expected 2 (RTL) — PX4 did not "
                "store the geofence action; onboard breach enforcement is OFF."
            )

    async def set_datalink_loss_rtl(self, timeout_s: float = 10.0) -> None:
        """Make PX4 RTL on data-link (GCS/telemetry) loss — an FC-level failsafe
        that fires even if the companion process dies, so it does NOT depend on
        the software watchdog. NAV_DLL_ACT=2 (Return); COM_DL_LOSS_T sets how
        long the link must be gone first. Readback-confirmed like
        set_geofence_action_rtl (a clean MAVSDK ACK doesn't guarantee storage)."""
        await self.system.param.set_param_int("NAV_DLL_ACT", 2)  # 2 = Return (RTL)
        readback = await self.system.param.get_param_int("NAV_DLL_ACT")
        if readback != 2:
            raise RuntimeError(
                f"NAV_DLL_ACT readback={readback}, expected 2 (RTL) — PX4 did not "
                "store the data-link-loss action; FC-level link-loss RTL is OFF."
            )
        # COM_DL_LOSS_T is an INT32 param — set it as int. A set_param_float here
        # raises a PX4 type-mismatch (seen in SITL: "param types mismatch param:
        # COM_DL_LOSS_T") and the timeout silently stays at its boot default.
        try:
            await self.system.param.set_param_int("COM_DL_LOSS_T", int(round(timeout_s)))
        except Exception as e:
            logger.warning(f"[mavlink] could not set COM_DL_LOSS_T={timeout_s}: {e}")

    async def set_rc_loss_rtl(self) -> None:
        """Make PX4 RTL on RC (safety-pilot link) loss — an FC-level failsafe.

        The mission code never pinned this (only tuning mode zeroed it), so RC-
        loss behaviour on the real bird rode on whatever the FC/QGC default was.
        NAV_RCL_ACT=2 (Return); COM_RCL_EXCEPT=4 exempts Offboard/Mission so the
        autonomous sortie isn't RTL'd merely because no RC is bound (SITL has no
        RC). Readback-confirmed like the geofence/datalink pins. NOTE: the HITL
        bench preload deliberately uses NAV_RCL_ACT=1 (Hold) to test RC loss on
        the bench — the mission pins 2 (Return); they differ on purpose."""
        await self.system.param.set_param_int("NAV_RCL_ACT", 2)  # 2 = Return (RTL)
        readback = await self.system.param.get_param_int("NAV_RCL_ACT")
        if readback != 2:
            raise RuntimeError(
                f"NAV_RCL_ACT readback={readback}, expected 2 (RTL) — PX4 did not "
                "store the RC-loss action; FC-level RC-loss RTL is OFF."
            )
        # COM_RCL_EXCEPT is a bitmask INT — best-effort (a wrong value only makes
        # the failsafe slightly more conservative, never unsafe).
        try:
            await self.system.param.set_param_int("COM_RCL_EXCEPT", 4)  # 4 = Offboard
        except Exception as e:
            logger.warning(f"[mavlink] could not set COM_RCL_EXCEPT=4: {e}")

    async def set_battery_failsafe(
        self, *, low: float, crit: float, emergen: float, action: int,
    ) -> None:
        """Pin the FC-level battery failsafe thresholds + action.

        The companion watchdog RTHs at 30% / LANDs at 20% as the PRIMARY layer;
        these FC thresholds (fractions 0..1) sit BELOW that as the authoritative
        backstop if the companion dies, so the two layers never race. The action
        (COM_LOW_BAT_ACT) is an INT and readback-confirmed; the three float
        thresholds are best-effort. Confirm on the real pack before G7."""
        for name, value in (
            ("BAT_LOW_THR", low), ("BAT_CRIT_THR", crit), ("BAT_EMERGEN_THR", emergen),
        ):
            try:
                await self.system.param.set_param_float(name, float(value))
            except Exception as e:
                logger.warning(f"[mavlink] could not set {name}={value}: {e}")
        await self.system.param.set_param_int("COM_LOW_BAT_ACT", int(action))
        readback = await self.system.param.get_param_int("COM_LOW_BAT_ACT")
        if readback != int(action):
            raise RuntimeError(
                f"COM_LOW_BAT_ACT readback={readback}, expected {action} — PX4 did "
                "not store the low-battery action; FC-level battery failsafe is OFF."
            )

    async def set_gimbal_mount(self, params: Mapping[str, float]) -> int:
        """Best-effort push of the MNT_* mount-driver params at mission start.

        The single nadir camera rides a pitch servo that PX4's gimbal/mount
        driver keeps stabilized pointing straight down — the flight code only
        CONFIGURES it (config ``gimbal:`` block); stabilization then runs on
        the FC autonomously. Every failure is a WARN, never a raise: the gimbal
        is not flight-critical, and the AUX "Gimbal Pitch" actuator mapping is
        QGC-side G5 bench work regardless. Returns the count applied."""
        return await self._apply_params(params, "gimbal mount")

    # ----- internal helpers -----

    async def _wait_until_altitude_reached(
        self, target_m: float, tolerance_m: float = 1.0, timeout_s: float = 60.0
    ) -> bool:
        """Block until the vehicle climbs within tolerance of target_m. Returns
        True if reached, False on timeout. We return a status (not raise) so the
        command path can't hang — but the caller MUST act on False: arm_and_takeoff
        turns it into a surfaced failure → the orchestrator's emergency-RTH
        boundary, rather than flying transit legs at an altitude the vehicle never
        actually reached (a thrust-loss / EKF-drift takeoff).

        The timeout wraps the *whole* stream wait (asyncio.wait_for), not a
        per-emission deadline check. position() only advances a deadline when it
        emits, so a deadline tested *inside* the `async for` is unreachable once
        the stream stops emitting entirely — exactly what a SITL lockstep freeze
        or a telemetry-link stall mid-climb-out does. That silently hung the
        climb-out forever (observed 2026-06-13: sim froze ~3 s after the first
        drop, this wait never returned). wait_for fires on the wall clock
        regardless, so a stalled stream now surfaces as a clean failure."""
        last_alt = float("nan")

        async def _watch() -> bool:
            nonlocal last_alt
            async for pos in self.system.telemetry.position():
                last_alt = pos.relative_altitude_m
                if last_alt >= target_m - tolerance_m:
                    return True
            return False

        try:
            return await asyncio.wait_for(_watch(), timeout=timeout_s)
        except asyncio.TimeoutError:
            logger.error(
                f"[mavlink] altitude {target_m:.1f} m NOT reached within "
                f"{timeout_s:.0f}s (last={last_alt:.1f} m; telemetry may have "
                "stalled) — surfacing failure."
            )
            return False

    async def _wait_until_disarmed(self, timeout_s: float = 120.0, poll_s: float = 0.5) -> bool:
        """Wait until the vehicle reports disarmed. Returns True once disarm is
        observed, False on timeout.

        Terminal paths (land/rth) await this. MAVSDK's armed() stream emits
        only on *change*, so an unbounded `async for` over it never returns if
        the vehicle never disarms (RTL loiter, or disarm telemetry stalls) —
        that hung shutdown after land/RTH. We poll the current armed state on a
        wall-clock deadline instead. On timeout we surface a CRITICAL log and
        return False rather than raise (the caller is already terminal); a
        still-armed vehicle is then backstopped by _arm_with_retry refusing to
        re-arm + the orchestrator's emergency-RTH boundary."""
        deadline = asyncio.get_running_loop().time() + timeout_s
        while True:
            if not await self._is_armed():
                return True
            if asyncio.get_running_loop().time() > deadline:
                logger.critical(
                    f"[mavlink] disarm not observed after {timeout_s:.0f}s — "
                    "abandoning the wait (vehicle may still be airborne; check link)."
                )
                return False
            await asyncio.sleep(poll_s)


@asynccontextmanager
async def connected_drone(
    config: ConnectionConfig | None = None,
) -> AsyncIterator[DroneCommander]:
    """Context manager that establishes + tears down a drone connection."""
    cmd = DroneCommander(config)
    await cmd.connect()
    try:
        yield cmd
    finally:
        # mavsdk has no explicit disconnect; the System object is reclaimed on GC
        logger.info("[mavlink] commander released")
