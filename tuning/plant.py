"""Plant-model constants derived from measured physical parameters.

Multirotor: rigid-body rotational dynamics. PX4's rate controller emits a
NORMALIZED torque command u∈[-1,1]; control allocation normalizes u≈1 to the
max achievable control torque, so the plant from command to angular rate is an
integrator with gain ``b = τ_max / I_axis`` (rad/s² per unit command). This is
an APPROXIMATION (it assumes ideal, normalized allocation) — it gives correct
gain *trends* + ballpark scale; absolute values are validated by autotune.

Fixed-wing: the rate-loop plant gain is ``M_δ / I_axis`` where the control
moment ``M_δ = q̄·S·ref·C_δ`` and dynamic pressure ``q̄ = ½ρV²``. FW gains are
valid at the reference airspeed (q̄-dependent) and degrade off-design.
"""

from __future__ import annotations

import math

from .schemas import PhysicalParams

GRAVITY = 9.80665
_QUAD_X_ARM_FACTOR = math.sin(math.radians(45.0))   # standard quad-X moment-arm projection

# Differential-couple factors k, per rotor layout: τ_max = k·F_max·arm, where
# k = ½·Σ|p̂| over the unit-circle rotor offsets on the axis that produces the
# moment (|ŷ| for roll, |x̂| for pitch). Quad-X reduces to the historical
# (n/2)·sin45° = √2. Hexa-X (rotors at ±30°, ±90°, ±150°) is NOT symmetric:
# roll ½(4·0.5 + 2·1) = 2.0 but pitch ½(4·cos30°) = √3, ~15 % less authority.
# ⚠ Keyed by MOTOR COUNT, which stands in for the layout only because this lab
# flies one aircraft per count. It is not a general truth: a hexa-PLUS has these
# two exactly swapped, and a coax Y6 shares n=6 with different arms entirely — so
# adding an airframe means checking the layout, not just the count. The honest
# fix, if the fleet ever grows, is to derive k from the rotor offsets that already
# exist in the CA_ROTOR* table rather than tabulating them here.
_MC_ARM_FACTORS: dict[int, tuple[float, float]] = {
    4: (math.sqrt(2.0), math.sqrt(2.0)),          # quad-X
    6: (2.0, math.sqrt(3.0)),                     # hexa-X (AAVC EFT X6100)
}


def mc_inertia(p: PhysicalParams, axis: str) -> float:
    return {"roll": p.ixx, "pitch": p.iyy, "yaw": p.izz}[axis]


def mc_max_control_torque(p: PhysicalParams, axis: str) -> float:
    """Max control torque (N·m) about a body axis for a multirotor.

    roll/pitch: differential-thrust couple ≈ k_axis·F_max·arm, with k_axis from
    the rotor layout (_MC_ARM_FACTORS); layouts with no entry fall back to the
    quad-X projection, which is only a rough scale for an unknown geometry.
    yaw: rotor reaction-torque differential ≈ n·F_max·(k_M/k_F).
    """
    n = p.n_motors or 0
    fmax = p.max_thrust_per_motor_n or 0.0
    arm = p.arm_length_m or 0.0
    if axis in ("roll", "pitch"):
        k_roll, k_pitch = _MC_ARM_FACTORS.get(
            n, ((n / 2.0) * _QUAD_X_ARM_FACTOR,) * 2
        )
        return (k_roll if axis == "roll" else k_pitch) * fmax * arm
    if axis == "yaw":
        return n * fmax * (p.prop_torque_coeff or 0.0)
    raise ValueError(f"unknown MC axis {axis!r}")


def mc_plant_gain(p: PhysicalParams, axis: str) -> float:
    """b = τ_max / I  (rad/s² per unit normalized torque command)."""
    return mc_max_control_torque(p, axis) / mc_inertia(p, axis)


def mc_accel_authority(p: PhysicalParams) -> float:
    """Net vertical acceleration authority above hover (m/s²) = (n·F_max)/m − g."""
    n = p.n_motors or 0
    fmax = p.max_thrust_per_motor_n or 0.0
    return (n * fmax) / p.mass_kg - GRAVITY


def fw_inertia(p: PhysicalParams, axis: str) -> float:
    return {"roll": p.ixx, "pitch": p.iyy, "yaw": p.izz}[axis]


def fw_dynamic_pressure(p: PhysicalParams) -> float:
    """q̄ = ½ ρ V²  (Pa)."""
    v = p.ref_airspeed_mps or 0.0
    return 0.5 * p.air_density_kgm3 * v * v


def fw_control_moment_derivative(p: PhysicalParams, axis: str) -> float:
    """M_δ = q̄·S·ref·C_δ  (N·m/rad). ref = wingspan (roll/yaw) | mean chord (pitch)."""
    q = fw_dynamic_pressure(p)
    s = p.wing_area_m2 or 0.0
    if axis == "roll":
        return q * s * (p.wingspan_m or 0.0) * (p.cl_delta or 0.0)
    if axis == "pitch":
        return q * s * (p.mean_chord_m or 0.0) * (p.cm_delta or 0.0)
    if axis == "yaw":
        return q * s * (p.wingspan_m or 0.0) * (p.cn_delta or 0.0)
    raise ValueError(f"unknown FW axis {axis!r}")


def fw_plant_gain(p: PhysicalParams, axis: str) -> float:
    """b_fw = M_δ / I  (rad/s² per rad of surface command)."""
    return fw_control_moment_derivative(p, axis) / fw_inertia(p, axis)
