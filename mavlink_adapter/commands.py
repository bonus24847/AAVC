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
from mavsdk.offboard import VelocityBodyYawspeed

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
    # EXPLICIT payload_id -> actuator-set index map, overriding the
    # drop_servo_channel + payload_id progression above; () keeps the
    # progression. Needed whenever the rack is not WIRED in delivery order —
    # KMUTNB 2026-08-15: the four latches came back from the bench on
    # AUX 4/1/2/3 for the front-left / rear-right / front-right / rear-left
    # corners, and the mission's release order must stay diagonal (CG), so
    # payload_id 0..3 -> (4, 1, 2, 3). PWM_AUX_FUNCn stays 300+n (pin ==
    # actuator set); only this map moves. See docs/SERVO_AUX_MAPPING.md.
    drop_servo_channels: tuple[int, ...] = ()
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

    def __post_init__(self) -> None:
        # YAML hands this over as a list; normalise to the frozen dataclass's
        # own tuple so equality/hashing keep working whoever built the config.
        chans = tuple(int(c) for c in self.drop_servo_channels)
        object.__setattr__(self, "drop_servo_channels", chans)
        if not chans:
            return
        if len(chans) < self.drop_payload_count:
            raise ValueError(
                f"drop_servo_channels {chans} covers {len(chans)} payloads but "
                f"drop_payload_count is {self.drop_payload_count} — every egg "
                "slot needs its own latch channel")
        if any(not 1 <= c <= 6 for c in chans):
            raise ValueError(
                f"drop_servo_channels {chans} out of range — MAV_CMD_DO_SET_ACTUATOR "
                "addresses actuator sets 1..6 (param1..6 with param7=0)")
        if len(set(chans)) != len(chans):
            raise ValueError(
                f"drop_servo_channels {chans} repeats a channel — two eggs sharing "
                "one latch is not an independent release mechanism (rules §7)")

    def actuator_index(self, payload_id: int) -> int:
        """Actuator-set index driving ``payload_id``'s latch.

        That index IS the real AUX pin number (``PWM_AUX_FUNCn = 300+n``, i.e.
        "Peripheral via Actuator Set n") and, in SITL, ``SIM_GZ_SV_FUNCn`` ->
        gz topic ``servo_<n-1>``. ``drop_servo_channels`` maps it explicitly
        when the rack is not wired in delivery order; otherwise it is the
        historical ``drop_servo_channel + payload_id`` progression.
        Bounds-checking of ``payload_id`` belongs to the caller
        (``drop_payload``), which refuses ids outside ``drop_payload_count``.
        """
        if self.drop_servo_channels:
            return self.drop_servo_channels[payload_id]
        return self.drop_servo_channel + payload_id


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
    # 5 = YAW FIXED in AUTO (v1.17 source-verified enum; 0-default yaws
    # towards every waypoint, spinning the nadir camera at each sweep turn
    # — rotational blur on top of the in-flight blur budget). The hexa
    # strafes its legs on one heading instead. Operator 2026-08-21.
    "MPC_YAW_MODE": 5.0,
    "MPC_XY_VEL_MAX": 5.0,      # horizontal speed ceiling (m/s); KMUTNB: the longest
                                # leg is ~52 m and accuracy is the brief — 10 m/s
                                # buys nothing here and doubles the overshoot.
    "MPC_XY_CRUISE": 3.0,       # auto cruise speed (m/s) — accuracy-over-speed:
                                # slower sweep = more frames per target = reliable
                                # blind discovery + tight leg tracking
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
    "MPC_ACC_HOR": 2.0,         # THE AUTO horizontal accel cap (m/s²) — KMUTNB
                                # accuracy set (3.0 was the KMITL value)
    "MPC_ACC_HOR_MAX": 5.0,     # manual position mode only; PX4 default 5
    "MPC_JERK_AUTO": 3.0,       # AUTO jerk (m/s³); PX4 default 4 — paired with
                                # the accel cap for the accuracy profile
    "NAV_ACC_RAD": 1.0,         # waypoint acceptance radius (m) — KMUTNB: transit
                                # legs are 10-20 m apart; 2 m would cut visible
                                # corners at the SCORED transit coordinates.
                                # mission.py _ARRIVAL_RADIUS_M = this + 1.
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
    "MPC_Z_VEL_MAX_DN": 1.5,    # MANUAL/offboard descent cap (m/s); default 1.5 — PX4
                                # reads this only outside AUTO, so it does NOT shape
                                # the mission's descents. See MPC_Z_V_AUTO_DN below.
    "MPC_Z_V_AUTO_DN": 0.4,     # THE AUTO descent speed (m/s); PX4 default 1.5. The
                                # validated pad-approach descent — unpinned, a real 6X
                                # would drop onto the pad 4x faster than anything tested.
                                # mission.py raises it only for the L&R staged descent.
    "MPC_Z_VEL_MAX_UP": 1.5,    # cap AUTO climb (m/s; default 3) — overshoot scales
                                # ~v² and the KMUTNB ceiling sits only 1 m above
                                # transit, so the band shrinks with the field
    # Takeoff climbs to transit_alt - 2 m and mission.py stages the last 2 m at
    # 1 m/s. Never above MPC_Z_VEL_MAX_UP.
    "MPC_TKO_SPEED": 1.5,       # takeoff climb speed (m/s); PX4 default 1.5
    "MPC_LAND_SPEED": 0.3,      # slow final touchdown (m/s); default 0.7 — NOT raised:
                                # a global bump made AUTO.LAND climb to 41 m (e02ffa3);
                                # the L&R descent is staged in mission.py instead
    "MPC_LAND_ALT1": 4.0,       # start slowing the descent at 4 m AGL (5 m band)
    "MPC_LAND_ALT2": 2.0,       # final crawl speed below 2 m AGL
    # Any failsafe RTL (geofence breach, datalink loss, watchdog) climbs to
    # RTL_RETURN_ALT first — the PX4 default is 60 m, which would smash through
    # the KMUTNB 10 m ceiling. Pin it just under the ceiling.
    "RTL_RETURN_ALT": 9.0,      # failsafe return altitude (m); PX4 default 60
    # FC-level ALTITUDE fence at the competition rules' 20 m (operator
    # 2026-08-18) — the outer net ABOVE the companion watchdog's 10 m
    # ceiling (warn 10.5 / RTH 12). Purely additive to the polygon fence
    # (unlike GF_MAX_HOR_DIST, which masked it and was removed); breach
    # response is the same pinned GF_ACTION=3 (Return).
    "GF_MAX_VER_DIST": 20.0,    # metres above home; PX4 default disabled
    # Downward rangefinder (Benewake TFmini-S) aids height through the delivery
    # descent and touchdown; 1 is already the 6X default but pin it so a param
    # reset cannot silently drop height aiding. The serial port assignment
    # (SENS_TFMINI_CFG) is a G5 bench decision, deliberately not set from here.
    "EKF2_RNG_CTRL": 1.0,       # fuse the downward rangefinder
    # Conditional aiding fuses range only while speed < EKF2_RNG_A_VMAX (1 m/s)
    # AND altitude < EKF2_RNG_A_HMAX. PX4's default HMAX 5.0 lands exactly on
    # the competition ladder's 5 m rung, where the aircraft is slowed to
    # re-centre — so the height reference could switch source right there.
    # 7.0 clears every rung, stays inside the TFmini-S band (0.1-12 m) and
    # inside PX4's own 1..10 limit.
    "EKF2_RNG_A_HMAX": 7.0,     # range aiding engages below this (m); default 5.0
    # 0 = BARO owns absolute height (operator decision 2026-08-20 after two
    # real KMUTNB night flights): the GPS vertical solution walked 12.7→36.6 m
    # parked and stepped +1.6/+1.3 m INSIDE a 30 s flight, so with 1 (GPS) the
    # reported AGL lied in both directions — one flight sank into the ground
    # off a stale home cache, the next was RTH'd by the ceiling watchdog while
    # physically fine. Baro is step-free over a <20 min flight; GPS keeps
    # horizontal; TFmini conditional aiding still pins the final metres.
    # NOT 2 (range): the local origin would follow ground level — a shed box
    # under the beam becomes "down". The competition config
    # (sitl/kmitl_config.yaml) still deliberately pins 1 for the 20 m site.
    # ⚠ reboot_required — applies from the next boot.
    "EKF2_HGT_REF": 0.0,        # height reference = BARO (2026-08-20)
    # Optical flow was cut from the project 2026-07-22 — no flow module aboard.
    # PX4 1.17 enables the fusion by default, which only invites a puzzling
    # "flow timeout" health failure at arming.
    "EKF2_OF_CTRL": 0.0,        # optical flow fusion OFF — no such sensor
    # MAV_1 = TELEM2 = the CM4 link. Forwarding carries the status beacon's
    # broadcast STATUSTEXT across to the radio instance, so the operator gets
    # the mission + camera summary with no WiFi (cm4/status_beacon.py). PX4's
    # per-instance default is [true, false, false]: the radio (MAV_0) already
    # forwards, this one does not. ⚠ reboot_required — pushing it here only
    # applies from the NEXT boot; the bench sets it once in QGC and this key
    # keeps it that way after a param reset.
    "MAV_1_FORWARD": 1.0,       # CM4 -> radio STATUSTEXT (reboot to apply)
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
    "MPC_YAW_MODE",                            # AUTO heading enum -> INT32
    # Missing from this list its whole life: the float write was rejected on
    # every connect ("EKF2_HGT_REF=1 failed: TIMEOUT" in every flight log), so
    # the pin never actually reached the board — found 2026-08-20 while
    # switching the practice site to baro height reference.
    "EKF2_HGT_REF",
    # Same defect, one entry down, its whole life too (boolean -> INT32;
    # "MAV_1_FORWARD=1 failed: TIMEOUT" in every flight log). The pin guards
    # the radio status beacon: had a param reset wiped it, the beacon would
    # have gone silent with no error anywhere. Caught by
    # tools/px4_type_audit.py on 2026-08-20 — run `make type-audit` whenever
    # a pushed param is added.
    "MAV_1_FORWARD",

    "SIM_BAT_ENABLE",                          # SITL battery simulator on/off
    # Gimbal (VERIFY-AT-G5). MNT_MAN_PITCH is not an angle — PX4 types it INT32
    # because it selects the RC AUX channel that drives pitch (0 = disabled).
    "MNT_MODE_IN", "MNT_MODE_OUT", "MNT_DO_STAB", "MNT_MAN_PITCH",
})

# Vertex-match tolerance when reading the onboard geofence back (deg). PX4 stores
# fence points as int32 at 1e-7 deg, so this is ~11x the storage quantisation and
# ~0.11 m on the ground — tight enough that a WRONG fence cannot pass, loose
# enough that a correctly stored one cannot fail. See _verify_geofence.
_FENCE_TOL_DEG = 1e-6

# The pins whose PX4 DEFAULT is out of the competition envelope, i.e. the ones a
# silent apply-failure turns into a rules bust or a broken egg. verify_envelope_pins
# reads these back after the tuning block is pushed. Keep the list short: it is
# "what must be true to fly", not "what we tuned".
# How many times to try uploading + reading back the fence before calling it
# do-not-fly. Three, with a short pause: the failure seen in the field is a
# MAVSDK RPC timeout on an otherwise healthy link, which clears on the next
# try; anything that survives three attempts is a real problem worth refusing.
_FENCE_UPLOAD_ATTEMPTS = 3
_FENCE_UPLOAD_BACKOFF_S = 1.0

_ENVELOPE_PINS = (
    "RTL_RETURN_ALT",     # default 60 m vs the 20 m ceiling — busts it on any RTL
    "GF_MAX_VER_DIST",    # the FC's 20 m altitude fence (rules) — default disabled
    "MPC_Z_V_AUTO_DN",    # default 1.5 m/s vs 0.4 validated onto the pad
    "COM_DISARM_LAND",    # default 2 s auto-disarms ON the pad mid-sortie
    # The ceiling under which the rangefinder is allowed to join the height
    # estimate: it decides where the aircraft thinks the ground is during the
    # land-ON descent, and a wrong value shows up only as a worse touchdown.
    # Applies live, so reading it back means something.
    "EKF2_RNG_A_HMAX",
    # ⚠ EKF2_HGT_REF is deliberately NOT here even though it matters just as
    # much: PX4 stores a new value immediately — so this read-back would say
    # PASS — while the estimator keeps running on the OLD reference. A gate that
    # can only report success is worse than no gate, because it is believed.
    # It is pinned in px4_tuning (so the value survives) and checked where the
    # answer is honest: at the bench, by tools/param_audit.py.
    # ⚠ REASON CORRECTED 2026-08-16: this used to say "it is reboot_required",
    # as if the metadata flag settled it. It does not — FD_ACT_EN and
    # BAT1_CAPACITY carry the same flag and both apply live. The real
    # disqualifier is that the EKF LATCHES its height reference when a source
    # starts fusing, and the one function that re-reads the param
    # (`checkHeightSensorRefFallback`) bails at its first line unless the
    # reference is UNKNOWN (height_control.cpp:63). Judge every candidate pin
    # by "does the owning module re-read it", never by the flag.
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


class PilotInControlError(RuntimeError):
    """Raised by a movement command once the pilot has taken the aircraft on RC.
    The safety layer latches it via ``DroneCommander.stand_down()``; nothing
    downstream should command the FC while the pilot is flying."""


class DroneCommander:
    """Thin async facade over MAVSDK for AAVC missions."""

    # Class default so a test double built with __new__ (skipping the real MAVSDK
    # System) reads a safe False before any stand_down().
    _pilot_in_control = False

    def __init__(self, config: ConnectionConfig | None = None) -> None:
        self.config = config or ConnectionConfig()
        self.system = System()
        self._connected = False
        # Home MSL altitude in metres, captured from telemetry.home() right
        # after MAVLink heartbeat. Needed because MAVSDK action.goto_location
        # takes ABSOLUTE (MSL) altitude while the rest of the orchestrator
        # speaks AGL relative-altitude. Conversion: msl = home_alt + alt_agl.
        self._home_alt_msl: float | None = None
        # Latched True by stand_down() when the pilot takes RC control; every
        # movement command then raises PilotInControlError instead of sending.
        self._pilot_in_control = False

    def stand_down(self) -> None:
        """Latch pilot-in-control: every subsequent movement command raises
        instead of reaching the FC. Called by the safety layer the instant it
        sees an RC takeover; never cleared — a takeover ends the mission and the
        aircraft is the pilot's until they land it."""
        self._pilot_in_control = True

    @property
    def pilot_in_control(self) -> bool:
        """True once the pilot has taken the aircraft (latched by stand_down).

        Public so callers OUTSIDE this class can refuse before they act rather
        than discovering it from an exception: the dashboard's raw dispatches
        reach ``system.action.*`` directly and never touch ``_guard_pilot``."""
        return self._pilot_in_control

    def _guard_pilot(self, what: str) -> None:
        if self._pilot_in_control:
            raise PilotInControlError(f"pilot has the aircraft — refusing to {what}")

    def _pilot_took_over_midway(self, what: str) -> bool:
        """True once the pilot has taken over DURING a long-running command.

        ``land()`` and ``rth()`` guard at entry and then block for 30-180 s
        waiting for touchdown. A takeover inside that window used to land on the
        unconditional ``action.disarm()`` that follows the wait — i.e. the one
        command that must never reach a FLYING aircraft, sent precisely when the
        pilot has just flown it away from the descent we asked for. These two
        call sites therefore re-check on the far side of the wait; they abandon
        the tail rather than raise, because the command they were asked to
        deliver was already delivered and the aircraft is now the pilot's."""
        if self._pilot_in_control:
            logger.warning(
                f"[mavlink] pilot took over mid-{what} — abandoning the disarm tail"
            )
            return True
        return False

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
        self._guard_pilot("arm/takeoff")
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
        # Re-guard (2026-08-21): between the entry guard and here sit the arm
        # retries, a 1 s settle and a home-alt refresh (1-6 s) — a takeover in
        # that window must not be followed by a takeoff command.
        self._guard_pilot("takeoff")
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
        # Guarded: commanding HOLD would yank a pilot who just took over
        # straight out of their POSCTL rescue.
        self._guard_pilot("switch to HOLD")
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
            # Re-checked EVERY attempt (2026-08-21): the retry window is up to
            # 15 s — a takeover landing mid-retry must abort the remaining
            # arms, not merely the next command after them. This is the exact
            # method the G7 zombie hammered against a parked aircraft.
            self._guard_pilot("arm")
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
        self._guard_pilot("goto")
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
        """Set a PX4 float parameter. Guarded: after a takeover nothing may
        rewrite the FC's behaviour under the pilot (reads stay allowed)."""
        self._guard_pilot(f"set param {name}")
        await self.system.param.set_param_float(name, float(value))

    async def get_param_int(self, name: str) -> int:
        """Read a PX4 int parameter (e.g. MIS_TKO_LAND_REQ). PX4 distinguishes
        int from float param types over MAVLink, so int params need a typed
        accessor — reading them via the float API returns a wrong value."""
        return int(await self.system.param.get_param_int(name))

    async def set_param_int(self, name: str, value: int) -> None:
        """Set a PX4 int parameter (e.g. MIS_TKO_LAND_REQ=0 to allow a
        mission with no landing waypoint). Guarded like set_param_float."""
        self._guard_pilot(f"set param {name}")
        await self.system.param.set_param_int(name, int(value))

    async def _apply_params(self, params: Mapping[str, float], kind: str) -> int:
        """Push a block of PX4 params, one write per key, and return how many
        landed. INT32-typed names (`_INT_PARAMS`) go via set_param_int; the rest
        via set_param_float. Per-param failure is logged and tolerated so one
        unknown key never aborts the mission — the CALLER decides whether the
        count it gets back is acceptable."""
        # Entry guard: raise BEFORE the loop, where the per-param try/except
        # would otherwise swallow PilotInControlError into warnings. (If a
        # stand_down lands mid-loop, the inner guards still keep every write
        # off the wire — just noisily.)
        self._guard_pilot("apply a param block")
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
            # RELATIVE, not absolute (2026-08-15). PX4 stores params as 32-bit
            # floats, whose spacing grows with magnitude: ~3.6e-6 at 60 but
            # ~5.5e-3 at 92160 — already past a 1e-3 absolute tol. Today's pins
            # are all small, so absolute happened to work; the failure it would
            # eventually cause is the WORSE direction for a gate that can stop a
            # flight — rejecting a CORRECT value, on the line, on competition
            # day. Fixed before the pin list grows, not after.
            if abs(got - want) > tol * max(1.0, abs(want)):
                bad.append(f"{name}={got:g} (want {want:g})")
        return bad

    # ── offboard priming (RC-GO) ──
    # The attitude/rate setpoint wrappers that lived here went out with
    # orchestrator.sysid_sweep (2026-08-15): they existed only to inject a
    # System-ID chirp, and the repo does not keep code with no caller. Only the
    # priming setpoint survives, because RC-GO needs it for a different reason —
    # PX4 will not let the PILOT select OFFBOARD unless a fresh offboard signal
    # (>2 Hz) is ALREADY arriving.

    async def prime_offboard_hold(self) -> None:
        """Send ONE zero-velocity offboard setpoint — no mode change, no
        ``offboard.start()``. The RC-GO preflight hold (orchestrator/main.py)
        streams this at ~5 Hz so the PILOT's OFFBOARD switch can engage: PX4
        refuses the mode unless a fresh offboard signal (>2 Hz) is already
        arriving. The mode change itself stays on the RC — in RC-GO the
        web/companion never launches the aircraft."""
        self._guard_pilot("stream offboard setpoints")
        await self.system.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

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
        # Guarded (2026-08-21): this method ARMS the vehicle and starts a
        # mission — it was the largest unguarded surface after stand_down().
        self._guard_pilot("upload/run a mission")
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
        self._guard_pilot("drop a payload")
        if not 0 <= payload_id < self.config.drop_payload_count:
            raise ValueError(
                f"payload_id {payload_id} out of range "
                f"[0, {self.config.drop_payload_count}) — refusing to address an "
                "unconfigured servo channel"
            )
        ch = self.config.actuator_index(payload_id)
        logger.info(
            f"[mavlink] drop payload {payload_id} (actuator set {ch} = AUX {ch})")
        # Primary path: MAVSDK action.set_actuator(index, value) — which is
        # MAV_CMD_DO_SET_ACTUATOR on the wire, the ONLY release command PX4
        # implements (FunctionActuatorSet; there is no DO_SET_SERVO handler).
        # index must be 1..6 (param7=0 addressing) — guaranteed by the
        # payload_id bounds check above plus ConnectionConfig.__post_init__,
        # which range-checks every explicit drop_servo_channels entry.
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
        # Guarded (2026-08-21): this is a raw side-channel that bypasses
        # MAVSDK entirely — it must respect stand_down() on its own, not only
        # via its guarded caller.
        self._guard_pilot("drop a payload (fallback)")
        endpoint = target_endpoint or self.config.drop_fallback_endpoint

        def _send() -> None:
            import math as _math
            import time as _time

            from pymavlink import mavutil

            # pymavlink 2.4.49 dropped the MAV_CMD_DO_SET_ACTUATOR name from
            # its generated enums (checked 2026-08-12) — the command itself is
            # still perfectly sendable as its raw id, 187.
            do_set_actuator = getattr(
                mavutil.mavlink, "MAV_CMD_DO_SET_ACTUATOR", 187)
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
                        do_set_actuator, 0,
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
        self._guard_pilot("land")
        logger.info(f"[mavlink] landing (disarm={disarm})")
        await self.system.action.land()
        if not disarm:
            return
        await self._wait_until_landed()
        if self._pilot_took_over_midway("land"):
            return
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
        self._guard_pilot("return-to-launch")
        logger.info("[mavlink] return-to-launch + land")
        try:
            await self.system.param.set_param_float("RTL_LAND_DELAY", 0.0)
        except Exception as e:
            logger.warning(f"[mavlink] could not set RTL_LAND_DELAY=0: {e}")
        await self.system.action.return_to_launch()
        if await self._wait_until_landed(timeout_s=180.0):
            if self._pilot_took_over_midway("return-to-launch"):
                return
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

        ⚠ READS THE FENCE BACK AND RAISES IF IT IS NOT THERE (2026-08-17).
        PX4 does NOT fail closed on a missing fence: ``Geofence::
        isInsidePolygonOrCircle`` opens with ``if (isEmpty()) { /* Empty fence
        -> accept all points */ return true; }``, so an upload that silently
        did not land leaves the FC accepting every point on earth. That used to
        be masked by ``GF_MAX_HOR_DIST`` — the radius fence, which is a stored
        param and needs no upload — but the operator removed it on 2026-08-17
        ("ตั้งผิดเมื่อไหร่ = ภารกิจตายทันที": a radius is a circle over a rotated
        rectangle, so it either clips legal airspace or leaks outside it, and a
        wrong one kills the mission mid-transit — which is exactly what a live
        ``GF_MAX_HOR_DIST=15`` did that day). With the radius gone this polygon
        is the ONLY FC-level fence, so "uploaded" has to mean verified, not
        attempted. An unverifiable fence counts as no fence and raises: callers
        must treat that as do-not-fly, not as a warning.
        """
        self._guard_pilot("upload a geofence")
        from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon

        if len(polygon) < 3:
            raise ValueError(f"geofence polygon needs >= 3 vertices, got {len(polygon)}")
        points = [Point(float(lat), float(lon)) for lat, lon in polygon]
        ftype = FenceType.INCLUSION if inclusion else FenceType.EXCLUSION
        data = GeofenceData([Polygon(points, ftype)], [])
        # RETRY the upload+readback, added 2026-08-23. This used to be one shot,
        # and on 2026-08-19 it cost two staged flights in one session: the
        # archived orchestrator.log shows `clear_geofence: TIMEOUT` followed by
        # the upload erroring, then "FC geofence NOT verified … refusing to
        # fly" — twice, twenty seconds apart, on a link that was otherwise fine.
        # Retrying weakens NOTHING: what makes the fence trustworthy is
        # _verify_geofence reading it back, and that still has to pass on the
        # attempt that succeeds. What it buys is the difference between a
        # transient MAVSDK timeout and a grounded aircraft — and the
        # competition gives 5 minutes of setup before the 20-minute clock runs.
        last: Exception | None = None
        for attempt in range(1, _FENCE_UPLOAD_ATTEMPTS + 1):
            try:
                await self.system.geofence.clear_geofence()
            except Exception as e:  # noqa: BLE001 — nothing to clear is fine
                logger.debug(f"[mavlink] geofence clear pre-upload: {e}")
            try:
                await self.system.geofence.upload_geofence(data)
                await self._verify_geofence(polygon, ftype)
                break
            except Exception as e:  # noqa: BLE001 — retried, then re-raised
                last = e
                if attempt < _FENCE_UPLOAD_ATTEMPTS:
                    logger.warning(
                        f"[mavlink] geofence upload attempt {attempt}/"
                        f"{_FENCE_UPLOAD_ATTEMPTS} failed ({e}) — retrying")
                    await asyncio.sleep(_FENCE_UPLOAD_BACKOFF_S)
        else:
            # Every attempt failed: the caller must still treat this as
            # do-not-fly. An unverified fence counts as no fence.
            raise RuntimeError(
                f"geofence not verified after {_FENCE_UPLOAD_ATTEMPTS} "
                f"attempts: {last}"
            ) from last
        logger.info(
            f"[mavlink] uploaded {ftype.name} geofence ({len(points)} vertices), "
            "read back from the FC and verified — the aircraft is fenced"
        )
        return len(points)

    async def _verify_geofence(self, polygon: list[tuple[float, float]], ftype: object) -> None:
        """Download the fence the FC actually holds and compare it to what we
        sent. Raises RuntimeError on any discrepancy — see ``upload_geofence``.

        Vertices are matched by proximity rather than by index: PX4 stores fence
        points as int32 at 1e-7 deg, and the order a fence comes back in is not
        contractual, but the SET of corners is. ``_FENCE_TOL_DEG`` (1e-6 deg,
        ~0.11 m) is far below any airspace boundary error that matters and far
        above the storage quantisation.
        """
        try:
            back = await self.system.geofence.download_geofence()
        except Exception as e:
            raise RuntimeError(
                f"geofence uploaded but could NOT be read back ({e}) — an "
                "unverified fence is treated as no fence, refusing to fly"
            ) from e

        polys = [p for p in (getattr(back, "polygons", None) or [])
                 if getattr(p, "fence_type", None) == ftype]
        if len(polys) != 1:
            raise RuntimeError(
                f"geofence readback holds {len(polys)} {getattr(ftype, 'name', ftype)} "
                f"polygon(s), expected exactly 1 — the FC is not fenced as commanded"
            )
        got = [(float(pt.latitude_deg), float(pt.longitude_deg)) for pt in polys[0].points]
        if len(got) != len(polygon):
            raise RuntimeError(
                f"geofence readback has {len(got)} vertices, uploaded {len(polygon)} "
                "— the FC is holding a different fence"
            )
        for lat, lon in polygon:
            if not any(abs(g_lat - lat) <= _FENCE_TOL_DEG and abs(g_lon - lon) <= _FENCE_TOL_DEG
                       for g_lat, g_lon in got):
                raise RuntimeError(
                    f"geofence vertex ({lat:.7f},{lon:.7f}) is absent from the FC's "
                    "own readback — the fence on the aircraft is not the one we drew"
                )

    async def set_geofence_action_rtl(self) -> None:
        """Make PX4 RETURN on geofence breach (vs the default warn-only).

        ⚠ FIXED 2026-08-16 — THIS WROTE THE WRONG ACTION FOR ITS WHOLE LIFE.
        It set ``GF_ACTION = 2`` and called that RTL in the value comment, the
        error text, the function name and CLAUDE.md. It is not. PX4's enum
        (``src/modules/navigator/geofence_params.c``) is:

            0 None · 1 Warning · 2 **Hold** · 3 **Return** · 4 Terminate · 5 Land

        So the FC-level geofence response was "stop and loiter" — executed at
        the breach point, i.e. potentially OUTSIDE the fence. That is the exact
        failure the design had already rejected once: safety.py's own comment
        records the move away from LAND-in-place because it "left the vehicle
        DOWN outside controlled airspace (a rules violation + recovery
        hazard)". Hold parks it there instead of landing it there — same
        problem, quieter. The companion watchdog's geofence check still RTHs,
        so the aircraft was not unprotected; what was missing is precisely the
        layer advertised as working when the CM4 does not.

        Fixing this also makes the breach DETECTABLE from the companion. PX4
        answers our own ``goto_location`` (DO_REPOSITION) with AUTO_LOITER,
        which MAVSDK reports as ``HOLD`` — the mode the mission flies in from
        takeoff to landing. A failsafe configured to Hold is therefore
        indistinguishable from normal flight and can never be spotted mode-side;
        one configured to Return shows up as RETURN_TO_LAUNCH. Any future
        geofence action must stay in {3 Return, 5 Land} for that reason — do
        NOT "restore" 2.

        Reads the param back after setting and raises if it didn't stick — a
        clean MAVSDK ACK does not guarantee PX4 stored the value. The caller
        records an anomaly on failure so a lost onboard geofence isn't silent."""
        self._guard_pilot("change the geofence action")
        await self.system.param.set_param_int("GF_ACTION", 3)  # 3 = Return
        readback = await self.system.param.get_param_int("GF_ACTION")
        if readback != 3:
            raise RuntimeError(
                f"GF_ACTION readback={readback}, expected 3 (Return) — PX4 did "
                "not store the geofence action; onboard breach enforcement is OFF."
            )

    async def set_datalink_loss_rtl(self, timeout_s: float = 10.0) -> None:
        """Make PX4 RTL on data-link (GCS/telemetry) loss — an FC-level failsafe
        that fires even if the companion process dies, so it does NOT depend on
        the software watchdog. NAV_DLL_ACT=2 (Return); COM_DL_LOSS_T sets how
        long the link must be gone first. Readback-confirmed like
        set_geofence_action_rtl (a clean MAVSDK ACK doesn't guarantee storage)."""
        self._guard_pilot("change the datalink-loss failsafe")
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
        self._guard_pilot("change the RC-loss failsafe")
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
        self._guard_pilot("change the battery failsafe")
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
        self, target_m: float, tolerance_m: float | None = None,
        timeout_s: float = 60.0,
    ) -> bool:
        # tolerance_m=None -> scale with the target. The old fixed 1.0 m was
        # sized for the 20 m KMITL profile (5% of the climb); on the KMUTNB 5 m
        # profile the whole climb-out is 3.0 m, so 1.0 m declared "at altitude"
        # a THIRD of the way short. The aircraft then began the transit legs at
        # ~2.0 m and finished the climb en route, which works in still air and
        # does not in wind: measured 2026-08-15, egress sat flat at 2.4 m
        # against a 3.5 m command for the whole leg at 10 m/s. Floored at 0.4 m
        # so a hover that settles a few centimetres low still counts.
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
        if tolerance_m is None:
            tolerance_m = min(1.0, max(0.4, 0.15 * abs(target_m)))
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
