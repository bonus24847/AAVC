"""Per-loop gain synthesis — the model-based control design.

Rate loops use a 2nd-order pole placement: a PI controller on an integrator
plant ``ω̇ = b·u`` closes to ``s² + b·P·s + b·I``; matching ``s² + 2ζω_c s + ω_c²``
gives ``P = 2ζω_c/b`` and ``I = ω_c²/b``. ω_c = 2π·f_bw. A derivative term
``D = τ_motor·P`` adds phase against the actuator lag (roll/pitch only).

Attitude loops are P-only (the PX4 attitude P-gain IS the attitude-loop
bandwidth in rad/s), placed a ``time_scale_factor`` below the rate loop for
cascade separation. Velocity/position (MC) are coarser (PX4's MPC is not a clean
2nd-order) and emitted with a warning to validate.

Every ComputedGain records its literal formula + assumptions. Absolute values
are STARTING estimates (the normalized-command/airspeed assumptions in plant.py)
— validate with empirical autotune.
"""

from __future__ import annotations

import math

from . import plant
from .schemas import ComputedGain, PerfSpec, PhysicalParams

# PI zero / integral crossover (rad/s). The textbook 2nd-order match (I=ω²/b)
# forces the integral into the dominant dynamics → an unrealistically large I.
# Practical rate loops (incl. PX4) keep I small: a low-frequency trim integral
# whose PI zero sits well below the rate crossover, so it rejects steady
# disturbances without eroding bandwidth/phase margin. I = P·ω_i.
_RATE_INTEGRAL_CROSSOVER_RADS = 1.5


def _eff_rate_bw_hz(spec: PerfSpec, axis: str | None = None) -> float:
    """Rate-loop bandwidth for an axis, clamped to the optional servo-slew cap
    (envelope). Yaw uses ``yaw_rate_bandwidth_hz`` when set (yaw is the weakest,
    slowest MC axis — see tuning/calibration.py). Both None ⇒ rate_bandwidth_hz
    unchanged (default design preserved)."""
    bw = spec.rate_bandwidth_hz
    if axis == "yaw" and spec.yaw_rate_bandwidth_hz is not None:
        bw = spec.yaw_rate_bandwidth_hz
    if spec.rate_bandwidth_hz_cap is not None:
        bw = min(bw, spec.rate_bandwidth_hz_cap)
    return bw


_MC_RATE_PREFIX = {"roll": "MC_ROLLRATE", "pitch": "MC_PITCHRATE", "yaw": "MC_YAWRATE"}
_MC_ATT_PARAM = {"roll": "MC_ROLL_P", "pitch": "MC_PITCH_P", "yaw": "MC_YAW_P"}
_FW_RATE_PREFIX = {"roll": "FW_RR", "pitch": "FW_PR", "yaw": "FW_YR"}
_FW_ATT_TC = {"roll": "FW_R_TC", "pitch": "FW_P_TC"}


def _g(param: str, value: float, loop: str, axis: str, formula: str,
       assumptions: list[str]) -> ComputedGain:
    return ComputedGain(param=param, value=round(float(value), 6), loop=loop,
                        axis=axis, formula=formula, assumptions=list(assumptions))


# ──────────────────────────── Multicopter ────────────────────────────

def mc_rate_gains(
    p: PhysicalParams, spec: PerfSpec, axis: str, b_override: float | None = None,
) -> list[ComputedGain]:
    omega = 2.0 * math.pi * _eff_rate_bw_hz(spec, axis)
    zeta = spec.rate_damping
    b_model = plant.mc_plant_gain(p, axis)
    use_meas = b_override is not None and b_override > 0
    b = float(b_override) if use_meas else b_model     # type: ignore[arg-type]
    pfx = _MC_RATE_PREFIX[axis]
    if use_meas:
        # Measured FRF b (frequency-sweep system-ID) replaces the τ_max/I model.
        # P ∝ 1/b, so anchoring to the (typically higher) measured yaw authority
        # removes the systematic yaw over-tune the model produces.
        asm = [
            "plant b = MEASURED FRF (frequency-sweep system-ID) — overrides the τ_max/I model",
            f"b_meas={b:.1f} (model τ_max/I={b_model:.1f}); P∝1/b → higher measured b lowers P",
        ]
    else:
        asm = [
            "plant b = τ_max/I (normalized-torque allocation); starting estimate, validate with autotune",
            f"τ_max≈{plant.mc_max_control_torque(p, axis):.3f} N·m, I={plant.mc_inertia(p, axis):.4f} kg·m², b={b:.1f}",
        ]
    pp = 2.0 * zeta * omega / b
    ii = pp * _RATE_INTEGRAL_CROSSOVER_RADS
    dd = (p.motor_time_constant_s * pp) if axis in ("roll", "pitch") else 0.0
    out = [
        _g(f"{pfx}_P", pp, "rate", axis,
           f"P = 2ζω/b = 2·{zeta}·{omega:.1f}/{b:.1f}  (P∝inertia: sluggish plant → more gain)", asm),
        _g(f"{pfx}_I", ii, "rate", axis,
           f"I = P·ω_i = P·{_RATE_INTEGRAL_CROSSOVER_RADS} (low-freq trim, PI zero below crossover)", asm),
        _g(f"{pfx}_D", dd, "rate", axis,
           (f"D = τ_motor·P = {p.motor_time_constant_s}·P" if dd else "D = 0 (yaw: no actuator-lag term)"),
           asm),
        _g(f"{pfx}_K", 1.0, "rate", axis, "K = 1 (gains map directly to P/I/D)", []),
        _g(f"{pfx}_FF", 0.0, "rate", axis, "FF = 0 (feedback-only rate design)", []),
    ]
    return out


def mc_attitude_gains(p: PhysicalParams, spec: PerfSpec) -> list[ComputedGain]:
    out = []
    for axis in ("roll", "pitch", "yaw"):
        # Per-axis rate bandwidth so a yaw de-rate (PerfSpec.yaw_rate_bandwidth_hz)
        # ALSO relaxes the yaw attitude P. Otherwise MC_YAW_P keeps the full
        # roll/pitch bandwidth and yaw is only half de-rated — the rate gains drop
        # but the attitude loop still commands the same aggressive rate setpoints.
        # axis=None ⇒ unchanged (rate_bandwidth_hz for every axis, the old design).
        bw = _eff_rate_bw_hz(spec, axis)
        omega_att = (2.0 * math.pi * bw) / spec.time_scale_factor
        asm = [f"ω_att = ω_rate/{spec.time_scale_factor} (cascade time-scale separation)"]
        out.append(_g(_MC_ATT_PARAM[axis], omega_att, "attitude", axis,
                      f"{_MC_ATT_PARAM[axis]} = ω_att = 2π·{bw}/{spec.time_scale_factor} "
                      f"= {omega_att:.2f} rad/s", asm))
    return out


def mc_velocity_position_gains(p: PhysicalParams, spec: PerfSpec) -> list[ComputedGain]:
    omega_v = 2.0 * math.pi * spec.velocity_bandwidth_hz
    omega_p = omega_v / spec.time_scale_factor
    asm = [
        "COARSE: PX4 MPC velocity loop is not a clean 2nd-order; validate in flight",
        f"vertical accel authority = (n·F_max)/m − g = {plant.mc_accel_authority(p):.1f} m/s²",
    ]
    return [
        _g("MPC_XY_VEL_P_ACC", omega_v, "velocity", "xy", f"≈ ω_v = 2π·{spec.velocity_bandwidth_hz}", asm),
        _g("MPC_XY_VEL_I_ACC", 0.2 * omega_v, "velocity", "xy", "≈ 0.2·ω_v (integral trim)", asm),
        _g("MPC_XY_VEL_D_ACC", 0.1, "velocity", "xy", "≈ 0.1 (light velocity damping)", asm),
        _g("MPC_Z_VEL_P_ACC", 1.5 * omega_v, "velocity", "z", "≈ 1.5·ω_v (vertical, higher authority)", asm),
        _g("MPC_Z_VEL_I_ACC", 0.5 * omega_v, "velocity", "z", "≈ 0.5·ω_v", asm),
        _g("MPC_XY_P", omega_p, "position", "xy", f"= ω_v/{spec.time_scale_factor}", asm),
        _g("MPC_Z_P", omega_p, "position", "z", f"= ω_v/{spec.time_scale_factor}", asm),
    ]


# ──────────────────────────── Fixed-wing ────────────────────────────

def fw_rate_gains(
    p: PhysicalParams, spec: PerfSpec, axis: str, b_override: float | None = None,
) -> list[ComputedGain]:
    omega = 2.0 * math.pi * _eff_rate_bw_hz(spec)
    zeta = spec.rate_damping
    b_model = plant.fw_plant_gain(p, axis)
    use_meas = b_override is not None and b_override > 0
    b = float(b_override) if use_meas else b_model    # type: ignore[arg-type]
    pfx = _FW_RATE_PREFIX[axis]
    if use_meas:
        # Measured cruise FRF b (normalized-command) overrides the q̄·S·C_δ/I
        # surface-rad model. PX4's FW rate controller acts on a NORMALIZED command
        # (like MC), so the measured normalized-command b is the unit-correct plant
        # gain; the model is only a per-command proxy (assumes ~1 rad per command).
        asm = [
            "plant b = MEASURED cruise FRF (normalized-command) — overrides the q̄·S·C_δ/I surface-rad model",
            f"b_meas={b:.1f} (model M_δ/I={b_model:.1f}); P∝1/b, PX4 FW rate cmd is normalized like MC",
        ]
    else:
        asm = [
            "plant b = M_δ/I, M_δ = q̄·S·ref·C_δ; gains valid at ref_airspeed (q̄-dependent), degrade off-design",
            f"q̄={plant.fw_dynamic_pressure(p):.1f} Pa, M_δ≈{plant.fw_control_moment_derivative(p, axis):.3f} N·m/rad, b={b:.1f}",
        ]
    pp = 2.0 * zeta * omega / b
    ii = pp * _RATE_INTEGRAL_CROSSOVER_RADS
    return [
        _g(f"{pfx}_P", pp, "rate", axis, f"P = 2ζω/b = 2·{zeta}·{omega:.1f}/{b:.1f}", asm),
        _g(f"{pfx}_I", ii, "rate", axis,
           f"I = P·ω_i = P·{_RATE_INTEGRAL_CROSSOVER_RADS} (low-freq trim)", asm),
        _g(f"{pfx}_D", 0.0, "rate", axis, "D = 0 (PX4 FW convention; lag handled by FF)", asm),
        _g(f"{pfx}_FF", 0.5, "rate", axis,
           "FF = 0.5 (airspeed-scaled steady-state surface; PX4-conventional, retune off-design)", asm),
    ]


def fw_attitude_gains(p: PhysicalParams, spec: PerfSpec) -> list[ComputedGain]:
    omega_rate = 2.0 * math.pi * _eff_rate_bw_hz(spec)
    omega_att = omega_rate / spec.time_scale_factor
    tc = 1.0 / omega_att
    asm = [f"attitude time-constant TC = 1/ω_att, ω_att = ω_rate/{spec.time_scale_factor}"]
    return [_g(_FW_ATT_TC[axis], tc, "attitude", axis,
               f"{_FW_ATT_TC[axis]} = 1/ω_att = {tc:.3f} s", asm)
            for axis in ("roll", "pitch")]
