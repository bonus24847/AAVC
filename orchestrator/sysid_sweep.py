"""Frequency-sweep (chirp) excitation for data-driven plant identification.

Pre-flight tuning aid — NOT the scored sortie (CLAUDE.md §2/§4). Streams a
tapered logarithmic chirp on ONE body axis per flight (clean SISO) so
``tuning.sysid`` can fit the rate-loop plant ``ω̇ = b·u`` from the resulting
PX4 ULog.

**numpy-only** (the reference used ``scipy.signal.chirp``; the log-chirp +
cosine taper are reimplemented in numpy here). MULTICOPTER only — the AAVC
hexacopter is the single airframe, and the hold altitude is clamped under the **20 m AGL**
competition ceiling (the reference's 80 m hover is out of envelope here).

Default mode is attitude-ANGLE chirp (self-levelling keeps thrust mostly
vertical → altitude holds); a ``rate`` chirp is available for a sharper
high-frequency fit. A cosine taper at both ends avoids step jolts; an altitude
P-loop on the collective arrests the chirp-induced sink and an absolute floor
aborts the stream. Always stops offboard + lands in a ``finally``.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from loguru import logger
from mavsdk.offboard import OffboardError

from mavlink_adapter.commands import DroneCommander
from mission_brain.profile import load_profile

_AXES = ("roll", "pitch", "yaw")
_STREAM_HZ = 50.0

# Altitude envelope — every number here derives from the ACTIVE mission
# profile's ceiling so the tuning tool follows the field it runs on: the KMITL
# competition profile (ceiling 20) reproduces the validated 15 / 6 / 19 set,
# while the KMUTNB sky-field profile (ceiling 5) scales the whole envelope
# into its band (hold 3, floor 1.5, abort 4) instead of commanding a 15 m
# hover through a 5 m ceiling. The hold altitude gives headroom for the
# chirp's vertical swing; the floor is an ABSOLUTE abort (a genuine fall
# toward the ground), not a deviation tolerance.
_PROFILE_CEILING_M = load_profile().altitude_ceiling_m
DEFAULT_HOLD_ALT_M = min(15.0, _PROFILE_CEILING_M * 0.6)
DEFAULT_ALT_FLOOR_ABORT_M = min(6.0, DEFAULT_HOLD_ALT_M * 0.5)
# The mirror of the floor abort. The original loop only guarded against sinking,
# but a hover base that over-estimates the aircraft's true hover thrust makes the
# chirp CLIMB — the more expensive direction to miss. Derived from the mission
# profile rather than restated: the ceiling is the number that moves (a rules
# revision, a lower practice-site cap), and a literal here would keep aborting
# at the old height after it did.
DEFAULT_ALT_CEILING_ABORT_M = _PROFILE_CEILING_M - 1.0
# Altitude-hold collective loop run DURING the chirp. It MUST be well damped: a
# P+I loop with no velocity damping AND no anti-windup went UNSTABLE in SITL — the
# chirp perturbed altitude, the integral wound up, and the aircraft oscillated
# 10 m → 21 m (through the 20 m ceiling) → into the ground (ULog-confirmed). The
# fix is a PD+I:
#   • KP on the altitude error,
#   • KD on the CLIMB RATE — the critical damper that stops the divergence,
#   • a small KI, FROZEN while the collective saturates (anti-windup), to trim the
#     hover-thrust bias (the FC's MPC_THR_HOVER can read low — the x500 reports
#     ~0.6 against a true ~0.7, and an un-converged MPC_USE_HTE reads low on any
#     airframe — so without a trim the aircraft sinks at chirp onset. The base
#     below adds a fixed margin rather than a floor, so this trim and the floor
#     abort are what cover an under-reading estimate.)
# Gains place the altitude loop at ω_n ≈ 0.9 rad/s, ζ ≈ 0.9 for the x500 plant.
_ALT_KP = 0.06
_ALT_KD = 0.12             # thrust per m/s of climb rate (+up) — damps the loop
_ALT_KI = 0.02
_ALT_I_CLAMP = 6.0
_THR_MIN, _THR_MAX = 0.40, 0.92
# The hover base is derived from the FC's own MPC_THR_HOVER rather than a fixed
# floor so the sweep works on any airframe (the x500 hovers near 0.70, the AAVC
# hexacopter near 0.51).
_HOVER_MARGIN = 1.06
_HOVER_BASE_MIN, _HOVER_BASE_MAX = 0.25, 0.85

# Per-axis chirp band + amplitudes. The band sits above the ~6 Hz
# rate-loop target so the FRF covers the crossover. Angle amplitudes are
# deliberately SMALL (6–8°): a large continuous attitude chirp saturates the
# motors differentially, robbing collective lift → the aircraft sinks out of its
# hold. Yaw gets a touch more (weaker authority).
_SWEEP_BAND_HZ = (0.6, 13.0)
_ANGLE_AMP_DEG = {"roll": 6.0, "pitch": 6.0, "yaw": 8.0}
_RATE_AMP_DPS = {"roll": 35.0, "pitch": 35.0, "yaw": 30.0}
_DEFAULT_THRUST = 0.66


def _alt_hold_thrust(base: float, target_alt: float, alt: float,
                     climb_rate: float = 0.0, integral: float = 0.0) -> float:
    """Altitude-hold collective: hover base + PD(+I) on the altitude error. Pure
    (unit-testable); the caller accumulates `integral` with anti-windup. KP raises
    thrust when below target; KD damps the climb RATE (the term that keeps the loop
    from oscillating/diverging during the chirp); KI trims a biased hover estimate.
    `climb_rate` is +up (m/s). Sinking below target raises thrust; rising above it,
    or climbing fast, lowers it."""
    return min(_THR_MAX, max(_THR_MIN,
                             base + _ALT_KP * (target_alt - alt)
                             - _ALT_KD * climb_rate + _ALT_KI * integral))


@dataclass
class SweepSpec:
    axis: str                       # roll | pitch | yaw — one axis per flight
    f0_hz: float
    f1_hz: float
    amp: float                      # attitude mode: deg ; rate mode: deg/s
    thrust: float                   # collective held through the sweep (hover)
    dur_s: float = 25.0
    mode: str = "attitude"          # attitude | rate


def sweep_spec_for(axis: str, mode: str = "attitude", dur_s: float = 25.0) -> SweepSpec:
    if axis not in _AXES:
        raise ValueError(f"unknown axis {axis!r}")
    amp = (_RATE_AMP_DPS if mode == "rate" else _ANGLE_AMP_DEG)[axis]
    return SweepSpec(axis=axis, f0_hz=_SWEEP_BAND_HZ[0], f1_hz=_SWEEP_BAND_HZ[1],
                     amp=amp, thrust=_DEFAULT_THRUST, dur_s=dur_s, mode=mode)


@dataclass
class SweepResult:
    axis: str
    mode: str = "attitude"
    band: tuple[float, float] = (0.0, 0.0)
    ok: bool = False
    reached_alt: bool = False
    offboard_window_epoch: tuple[float, float] | None = None
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis, "mode": self.mode, "band": list(self.band),
            "ok": self.ok, "reached_alt": self.reached_alt,
            "offboard_window_epoch": (list(self.offboard_window_epoch)
                                      if self.offboard_window_epoch else None),
            "detail": self.detail,
        }


def _chirp_samples(spec: SweepSpec, fs: float = _STREAM_HZ) -> np.ndarray:
    """Tapered logarithmic chirp on the target axis (amp-scaled), numpy-only.

    Instantaneous frequency f(t) = f0·(f1/f0)^(t/T); the phase is its integral
    φ(t) = 2π·f0·T/ln(f1/f0)·((f1/f0)^(t/T) − 1). A half-cosine taper over the
    first/last 0.5 s removes the step jolt at on/off."""
    t = np.arange(0.0, spec.dur_s, 1.0 / fs)
    k = spec.f1_hz / spec.f0_hz
    phase = 2.0 * np.pi * spec.f0_hz * spec.dur_s / np.log(k) * (k ** (t / spec.dur_s) - 1.0)
    s = spec.amp * np.sin(phase)
    n_taper = int(0.5 * fs)
    if len(s) > 2 * n_taper and n_taper > 0:
        ramp = 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, n_taper)))
        w = np.ones(len(s))
        w[:n_taper] = ramp
        w[-n_taper:] = ramp[::-1]
        s = s * w
    return s


async def _stream_sweep(
    commander: DroneCommander, spec: SweepSpec, tel: dict[str, float],
    target_alt: float, alt_floor_m: float, res: SweepResult,
    alt_ceiling_m: float = DEFAULT_ALT_CEILING_ABORT_M,
) -> tuple[bool, str]:
    """Prime + start offboard, stream the chirp at 50 Hz, abort on a floor OR
    ceiling breach — a sweep that climbs is as dangerous as one that sinks, and
    the AAVC envelope tops out at 20 m."""
    samples = _chirp_samples(spec)
    try:
        if spec.mode == "rate":
            await commander.set_attitude_rate(0.0, 0.0, 0.0, spec.thrust)
        else:
            await commander.set_attitude(0.0, 0.0, 0.0, spec.thrust)
        await commander.offboard_start()
    except OffboardError as e:
        logger.warning(f"[sweep] offboard {spec.mode} start rejected: {e!s:.80}")
        return False, ""

    t0 = time.time()
    res.offboard_window_epoch = (t0, t0)
    dt = 1.0 / _STREAM_HZ
    base_thrust = spec.thrust
    err_int = 0.0
    aborted = False
    abort_reason = ""
    for s in samples:
        vals = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        vals[spec.axis] = float(s)
        thr = _alt_hold_thrust(base_thrust, target_alt, tel["alt"], tel["climb"], err_int)
        # Anti-windup: only grow the integral while the collective is NOT saturated.
        # An always-on integral wound up during the initial sink and then threw the
        # aircraft up through the ceiling and back into the ground.
        if _THR_MIN < thr < _THR_MAX:
            err_int = min(_ALT_I_CLAMP, max(-_ALT_I_CLAMP,
                                            err_int + (target_alt - tel["alt"]) * dt))
        if spec.mode == "rate":
            await commander.set_attitude_rate(vals["roll"], vals["pitch"], vals["yaw"], thr)
        else:
            await commander.set_attitude(vals["roll"], vals["pitch"], vals["yaw"], thr)
        if tel["alt"] > alt_ceiling_m:
            # Logged like the floor breach below, and stated as an abort REASON
            # rather than only in res.detail: run_sweep replaces detail with a
            # generic message on any abort, so a ceiling climb used to be
            # indistinguishable from an offboard rejection — silent on exactly
            # the failure this guard was added to surface.
            logger.warning(f"[sweep] altitude ceiling breached "
                           f"(alt={tel['alt']:.1f} > {alt_ceiling_m:.0f} m) → abort")
            abort_reason = (f"climbed through the ceiling "
                            f"(alt={tel['alt']:.1f} > {alt_ceiling_m:.0f} m) — "
                            "the hover base is over-estimating true hover thrust")
            aborted = True
            break
        if tel["alt"] < alt_floor_m:
            logger.warning(f"[sweep] altitude floor breached "
                           f"(alt={tel['alt']:.1f} < {alt_floor_m:.0f} m) → abort")
            abort_reason = (f"sank through the floor "
                            f"(alt={tel['alt']:.1f} < {alt_floor_m:.0f} m) — "
                            "the hover base is under-estimating true hover thrust")
            aborted = True
            break
        await asyncio.sleep(dt)
    res.offboard_window_epoch = (t0, time.time())
    return not aborted, abort_reason


async def run_sweep(
    commander: DroneCommander, spec: SweepSpec, *,
    hold_alt_m: float = DEFAULT_HOLD_ALT_M,
    alt_floor_m: float = DEFAULT_ALT_FLOOR_ABORT_M,
    alt_ceiling_m: float = DEFAULT_ALT_CEILING_ABORT_M,
    arm_timeout_s: float = 30.0,
) -> SweepResult:
    """Arm → takeoff to the hold altitude → chirp on one axis → HOLD → land.

    Returns a SweepResult with the offboard window (for the ULog FRF). The
    actual plant fit is done by ``tuning.sysid`` from the PX4 log afterwards.
    """
    sys = commander.system
    target_alt = min(hold_alt_m, 18.0)   # never command above the 20 m ceiling band
    res = SweepResult(axis=spec.axis, mode=spec.mode, band=(spec.f0_hz, spec.f1_hz))
    tel: dict[str, float] = {"armed": 0.0, "alt": 0.0, "climb": 0.0}

    async def _w_armed() -> None:
        async for a in sys.telemetry.armed():
            tel["armed"] = 1.0 if a else 0.0

    async def _w_pos() -> None:
        async for p in sys.telemetry.position():
            tel["alt"] = float(p.relative_altitude_m)

    async def _w_vel() -> None:
        # +up climb rate for the altitude-hold D (damping) term. velocity_ned is
        # NED, so down_m_s > 0 means descending → negate for a +up convention.
        async for v in sys.telemetry.velocity_ned():
            tel["climb"] = -float(v.down_m_s)

    watchers = [asyncio.create_task(c) for c in (_w_armed(), _w_pos(), _w_vel())]

    async def _wait(pred: Any, timeout_s: float) -> bool:
        loop = asyncio.get_running_loop()
        t0 = loop.time()
        while loop.time() - t0 < timeout_s:
            if pred():
                return True
            await asyncio.sleep(0.25)
        return False

    try:
        try:
            await sys.action.arm()
        except Exception as e:  # noqa: BLE001 — preflight may deny; report
            logger.warning(f"[sweep] arm denied: {e!s:.60}")
        if not await _wait(lambda: tel["armed"] > 0.5, arm_timeout_s):
            res.detail = "never armed"
            return res

        try:
            await sys.action.set_takeoff_altitude(target_alt)
        except Exception as e:  # noqa: BLE001
            logger.debug(f"[sweep] set_takeoff_altitude: {e!s:.60}")
        await sys.action.takeoff()
        res.reached_alt = await _wait(lambda: tel["alt"] >= target_alt - 1.5, 60.0)
        if not res.reached_alt:
            res.detail = f"did not reach {target_alt:.0f} m (alt={tel['alt']:.1f})"
            return res

        # Base collective from the measured hover thrust; the alt-hold P-loop
        # rides on top of it to arrest the chirp-induced sink.
        try:
            hov = float(await commander.get_param_float("MPC_THR_HOVER"))
        except Exception:  # noqa: BLE001
            hov = 0.0
        # Bias the base slightly ABOVE the FC's hover estimate: it reads low, and a
        # too-low base makes the aircraft sink at chirp onset before the I-term can
        # trim in. The margin is relative, not the old fixed 0.66 floor — that
        # number came from the ~0.70-hover x500 and would command +0.15 of excess
        # collective on the 0.51-hover hexacopter, i.e. a climb toward the ceiling
        # at the very moment the chirp starts.
        spec.thrust = (min(_HOVER_BASE_MAX, max(_HOVER_BASE_MIN, hov * _HOVER_MARGIN))
                       if hov > 0 else _DEFAULT_THRUST)
        await asyncio.sleep(2.0)

        logger.info(f"[sweep] {spec.axis} chirp {spec.f0_hz}->{spec.f1_hz} Hz "
                    f"amp={spec.amp} mode={spec.mode} dur={spec.dur_s}s hold={target_alt:.0f}m")
        ok, why = await _stream_sweep(
            commander, spec, tel, target_alt, alt_floor_m, res,
            alt_ceiling_m=alt_ceiling_m)
        if ok:
            res.ok = True
            res.detail = f"{spec.axis} chirp {spec.f0_hz}-{spec.f1_hz} Hz ({spec.mode}) complete"
        else:
            # `why` is empty only when offboard itself refused the stream; an
            # altitude abort names the cause, which is the difference between
            # "retry it" and "fix the hover base first".
            res.detail = (f"{spec.axis} chirp aborted: {why}" if why
                          else f"{spec.axis} chirp aborted/rejected ({spec.mode})")
        try:
            await commander.offboard_stop()
            await sys.action.hold()
            await asyncio.sleep(2.0)
        except Exception as e:  # noqa: BLE001
            logger.debug(f"[sweep] post-sweep hold: {e!s:.60}")
        return res

    except Exception as e:  # noqa: BLE001 — report, never raise into the tuner
        logger.exception(f"[sweep] {spec.axis} error: {e}")
        res.detail = f"error: {e!s:.100}"
        return res
    finally:
        try:
            await commander.offboard_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            await sys.action.land()
        except Exception as e:  # noqa: BLE001
            logger.warning(f"[sweep] land(): {e!s:.60}")
        # The mission pins COM_DISARM_LAND=-1 (stay armed on the pad) and the
        # same param set is applied in tuning mode — the vehicle will NOT
        # auto-disarm here. Disarm EXPLICITLY once on the ground: it closes/
        # rotates the PX4 ULog per sweep (the fit right after each sweep
        # assumes that), and ends the old dead 120 s wait-for-a-disarm-that-
        # never-came. Tuning is not the scored sortie — a ground disarm is
        # safe, and PX4 refuses an in-air disarm anyway (the altitude gate
        # just avoids spamming it during the descent).
        if await _wait(lambda: tel["alt"] < 1.0, 60.0):
            for _ in range(3):
                try:
                    await sys.action.disarm()
                except Exception as e:  # noqa: BLE001
                    logger.debug(f"[sweep] disarm: {e!s:.60}")
                if await _wait(lambda: tel["armed"] < 0.5, 5.0):
                    break
        if not await _wait(lambda: tel["armed"] < 0.5, 15.0):
            logger.warning("[sweep] vehicle still ARMED after land+disarm — the "
                           "PX4 ULog stays open (the next fit may read a mixed log)")
        for w in watchers:
            w.cancel()
