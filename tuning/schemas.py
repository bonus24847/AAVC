"""Pydantic models for the model-based PID tuning engine (System ID + Tuner tab).

The engine maps a measured PHYSICAL plant model (mass, moments of inertia,
multirotor actuation geometry, and for fixed-wing the aerodynamics) plus a
desired PERFORMANCE spec (per-loop bandwidth + damping) to PX4 controller gains.

PX4 itself never consumes mass/inertia — its own autotune is empirical
signal-injection — so this is OUR control-design logic. Outputs are STARTING
gains to validate with empirical autotune + a cautious hover before autonomous
flight. Commercial-grade: airframe-general (the AAVC quad is one config), strict
validation, every gain carries its derivation formula + assumptions.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from mission_brain.schemas import Airframe

# Airframe families: which physical sub-models each needs. TWINBOOM is a
# quad-plane VTOL (4 lift rotors + pusher + wing/A-tail) → needs BOTH the MC
# (hover) and FW (cruise) tuning blocks, same as VTOL.
_NEEDS_MC = (Airframe.QUADCOPTER, Airframe.HEXACOPTER, Airframe.VTOL,
             Airframe.TILT_VTOL, Airframe.TAILSITTER, Airframe.TWINBOOM)
_NEEDS_FW = (Airframe.FIXED_WING, Airframe.VTOL, Airframe.TILT_VTOL,
             Airframe.TAILSITTER, Airframe.TWINBOOM)
_MC_FIELDS = ("arm_length_m", "n_motors", "max_thrust_per_motor_n", "prop_torque_coeff")
_FW_FIELDS = ("wing_area_m2", "mean_chord_m", "wingspan_m", "ref_airspeed_mps",
              "cl_delta", "cm_delta", "cn_delta")


class PhysicalParams(BaseModel):
    """Measured / estimated vehicle physical parameters, SI units. The MC block
    is required for multirotor + VTOL-hover tuning; the FW block for fixed-wing +
    VTOL-cruise tuning (enforced per-airframe by DesignRequest)."""

    model_config = ConfigDict(extra="forbid")

    mass_kg: float = Field(..., gt=0, le=200, description="Total vehicle mass (kg)")
    ixx: float = Field(..., gt=0, le=100, description="Roll inertia about body x (kg·m²)")
    iyy: float = Field(..., gt=0, le=100, description="Pitch inertia about body y (kg·m²)")
    izz: float = Field(..., gt=0, le=100, description="Yaw inertia about body z (kg·m²)")

    # ── Multirotor actuation geometry (MC / VTOL-hover) ──
    arm_length_m: float | None = Field(None, gt=0, le=5, description="Hub→rotor arm length (m)")
    n_motors: int | None = Field(None, ge=3, le=12, description="Number of lift rotors")
    max_thrust_per_motor_n: float | None = Field(None, gt=0, le=500, description="Max thrust / rotor (N)")
    prop_torque_coeff: float | None = Field(None, gt=0, le=0.2,
                                            description="Reaction-torque ratio k_M/k_F (N·m/N), yaw authority")
    motor_time_constant_s: float = Field(0.02, gt=0, le=0.5,
                                         description="Motor+ESC first-order lag (s) — rate-loop D term")

    # ── Fixed-wing aerodynamics (FW / VTOL-cruise) ──
    wing_area_m2: float | None = Field(None, gt=0, le=50, description="Reference wing area S (m²)")
    mean_chord_m: float | None = Field(None, gt=0, le=10, description="Mean aerodynamic chord (m), pitch ref")
    wingspan_m: float | None = Field(None, gt=0, le=30, description="Wingspan b (m), roll/yaw ref")
    ref_airspeed_mps: float | None = Field(None, gt=0, le=120, description="Trim/cruise airspeed for tuning (m/s)")
    air_density_kgm3: float = Field(1.225, gt=0, le=2.0, description="Air density ρ (kg/m³)")
    cl_delta: float | None = Field(None, gt=0, le=20, description="Roll control deriv dCl/dδa (1/rad)")
    cm_delta: float | None = Field(None, gt=0, le=20, description="Pitch control deriv dCm/dδe (1/rad)")
    cn_delta: float | None = Field(None, gt=0, le=20, description="Yaw control deriv dCn/dδr (1/rad)")

    # ── Flight-envelope inputs (AoA / stall floor + servo slew; orchestrator/envelope.py) ──
    cl_alpha: float | None = Field(None, gt=0, le=12,
                                   description="Lift-curve slope dCL/dα (1/rad), from AVL — for the stall/AoA floor")
    cl_max: float | None = Field(None, gt=0, le=3.0,
                                 description="Max usable CL (linear-AoA ceiling). Overrides cl_alpha·α_max if set")
    alpha_max_deg: float | None = Field(None, gt=0, le=20,
                                        description="AVL linear-range angle-of-attack ceiling (deg)")
    servo_slew_deg_s: float | None = Field(None, gt=0, le=2000,
                                           description="Control-surface servo slew rate (deg/s) — rate-loop bandwidth cap")
    servo_max_deflection_deg: float | None = Field(None, gt=0, le=60,
                                                   description="Max control-surface deflection (deg)")


class PerfSpec(BaseModel):
    """Desired closed-loop performance. The inner rate-loop bandwidth + damping
    drive the 2nd-order pole placement; outer loops are derived by the cascade
    time-scale separation (each outer loop ``time_scale_factor`` slower)."""

    model_config = ConfigDict(extra="forbid")

    rate_bandwidth_hz: float = Field(6.0, gt=0, le=50,
                                     description="Inner rate-loop target natural freq ω_n/2π (Hz)")
    yaw_rate_bandwidth_hz: float | None = Field(
        None, gt=0, le=50,
        description="Optional SEPARATE yaw rate-loop bandwidth (Hz). Yaw is the weakest, "
        "slowest multicopter axis — SITL system-ID measured b_yaw ~4x below the model "
        "(2026-06-01), so designing yaw at the roll/pitch bandwidth over-tunes it. Set this "
        "below rate_bandwidth_hz to de-rate yaw; None = use rate_bandwidth_hz (unchanged).")
    rate_bandwidth_hz_cap: float | None = Field(
        None, gt=0, le=50,
        description="Optional ceiling on the rate-loop bandwidth (servo-slew envelope, "
        "orchestrator/envelope.py). When set, synthesis clamps rate_bandwidth_hz to this "
        "so designed gains never demand surface motion faster than the servo can deliver.")
    rate_damping: float = Field(0.7, ge=0.3, le=1.5, description="Rate-loop damping ratio ζ")
    velocity_bandwidth_hz: float = Field(0.5, gt=0, le=10, description="Velocity-loop bandwidth (MC, Hz)")
    time_scale_factor: float = Field(4.0, ge=2, le=10, description="Cascade separation (outer = inner / this)")


class ComputedGain(BaseModel):
    """One PX4 parameter the engine designed, with its derivation."""

    model_config = ConfigDict(extra="forbid")

    param: str                                   # PX4 param name, e.g. "MC_ROLLRATE_P"
    value: float
    loop: str                                    # rate|attitude|velocity|position|guidance|tecs|mixer
    axis: str = ""                               # roll|pitch|yaw|xy|z|""
    formula: str                                 # human-readable derivation
    assumptions: list[str] = Field(default_factory=list)


class DesignRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    airframe: Airframe
    physical: PhysicalParams
    spec: PerfSpec = Field(default_factory=PerfSpec)

    @model_validator(mode="after")
    def _require_airframe_fields(self) -> DesignRequest:
        missing: list[str] = []
        if self.airframe in _NEEDS_MC:
            missing += [f for f in _MC_FIELDS if getattr(self.physical, f) is None]
        if self.airframe in _NEEDS_FW:
            missing += [f for f in _FW_FIELDS if getattr(self.physical, f) is None]
        if missing:
            raise ValueError(
                f"{self.airframe.value} tuning requires physical params: {', '.join(missing)}"
            )
        return self


class DesignResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    airframe: Airframe
    gains: list[ComputedGain]
    warnings: list[str] = Field(default_factory=list)
    plant_summary: dict[str, float] = Field(default_factory=dict)
