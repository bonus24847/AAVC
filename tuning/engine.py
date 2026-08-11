"""design_gains() — dispatch the model-based gain synthesis by airframe.

Multirotor + VTOL-hover → MC rate/attitude/velocity/position loops.
Fixed-wing + VTOL-cruise → FW rate + attitude loops.
VTOL family runs BOTH sets. Pure + deterministic (no I/O).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from . import plant, synthesis
from .schemas import (
    _NEEDS_FW,
    _NEEDS_MC,
    ComputedGain,
    DesignRequest,
    DesignResult,
)

if TYPE_CHECKING:
    from .calibration import PlantCalibration


def design_gains(req: DesignRequest, calibration: PlantCalibration | None = None) -> DesignResult:
    """Design PX4 gains from the physical/perf model.

    ``calibration`` (optional) supplies measured rate-loop plant gains ``b`` from a
    frequency-sweep system-ID; when given, the MULTICOPTER rate synthesis uses the
    measured ``b`` instead of the first-principles ``τ_max/I`` (fixes the yaw
    over-tune). ``calibration=None`` ⇒ byte-identical to the first-principles
    design — so the competition quad path is unchanged unless a calibration is
    explicitly supplied. Fixed-wing rate gains use the measured CRUISE plant ``b``
    from ``calibration.b_measured_fw`` when present (a normalized-command FRF — the
    unit-correct quantity), else the model ``M_δ/I``. The FW (cruise) measurement is
    kept separate from the MC (hover) ``b_measured`` so a VTOL's regimes never mix."""
    p, spec, af = req.physical, req.spec, req.airframe
    gains: list[ComputedGain] = []
    warnings: list[str] = []
    plant_summary: dict[str, float] = {}

    if af in _NEEDS_MC:
        for axis in ("roll", "pitch", "yaw"):
            b_meas = calibration.b_for(axis) if calibration is not None else None
            # Yaw-authority guard (SITL 2026-06-01: measured b_yaw ~0.25x the model).
            # A measured b_yaw far BELOW the model would RAISE designed yaw P (P∝1/b)
            # and over-tune the weakest axis. Don't apply the yaw override unless a
            # yaw bandwidth de-rate is set to compensate — fall back to the model b
            # for yaw and surface a warning so the over-tune can't slip in silently.
            if (axis == "yaw" and b_meas is not None
                    and spec.yaw_rate_bandwidth_hz is None):
                b_model_yaw = plant.mc_plant_gain(p, "yaw")
                if b_model_yaw > 0 and b_meas < 0.5 * b_model_yaw:
                    warnings.append(
                        f"Measured b_yaw ({b_meas:.1f}) is {b_meas / b_model_yaw:.0%} of the model "
                        f"({b_model_yaw:.1f}) — the model OVER-estimates yaw authority. Applying it "
                        "would raise yaw P (P∝1/b) and over-tune yaw; kept the model b for yaw. To "
                        "use the measured yaw plant, set PerfSpec.yaw_rate_bandwidth_hz (de-rate), "
                        "or tune yaw empirically (safe-optimizer / stock)."
                    )
                    b_meas = None
            gains += synthesis.mc_rate_gains(p, spec, axis, b_override=b_meas)
            plant_summary[f"mc_tau_max_{axis}_Nm"] = round(plant.mc_max_control_torque(p, axis), 4)
            plant_summary[f"mc_plant_b_{axis}"] = round(plant.mc_plant_gain(p, axis), 3)
            if b_meas is not None:
                plant_summary[f"mc_plant_b_meas_{axis}"] = round(float(b_meas), 3)
        gains += synthesis.mc_attitude_gains(p, spec)
        gains += synthesis.mc_velocity_position_gains(p, spec)
        authority = plant.mc_accel_authority(p)
        plant_summary["mc_accel_authority_mps2"] = round(authority, 3)
        if authority < 5.0:
            warnings.append(
                f"Low vertical accel authority ({authority:.1f} m/s²) — thrust-to-weight is "
                "marginal; velocity/position gains may be unachievable."
            )
        warnings.append("MC velocity/position gains are coarse approximations — validate in flight.")

    if af in _NEEDS_FW:
        fw_meas_applied = False
        for axis in ("roll", "pitch", "yaw"):
            # FW cruise-regime measured b (normalized-command FRF) overrides the
            # surface-rad model M_δ/I when present; mirrors the MC override but
            # reads the SEPARATE b_measured_fw so a VTOL's hover b can't leak in.
            b_meas_fw = calibration.b_fw_for(axis) if calibration is not None else None
            gains += synthesis.fw_rate_gains(p, spec, axis, b_override=b_meas_fw)
            plant_summary[f"fw_Mdelta_{axis}_Nm"] = round(plant.fw_control_moment_derivative(p, axis), 4)
            plant_summary[f"fw_plant_b_{axis}"] = round(plant.fw_plant_gain(p, axis), 3)
            if b_meas_fw is not None:
                plant_summary[f"fw_plant_b_meas_{axis}"] = round(float(b_meas_fw), 3)
                fw_meas_applied = True
        gains += synthesis.fw_attitude_gains(p, spec)
        plant_summary["fw_dynamic_pressure_pa"] = round(plant.fw_dynamic_pressure(p), 2)
        if fw_meas_applied:
            warnings.append(
                "FW rate gains use a MEASURED cruise plant b (normalized-command FRF) on the "
                "measured axes, overriding the q̄·S·C_δ/I surface-rad model (a per-command proxy); "
                "unmeasured axes stay model-based."
            )
        warnings.append(
            f"FW gains valid at ref_airspeed={p.ref_airspeed_mps} m/s (q̄-dependent) — "
            "retune for off-design speeds."
        )

    warnings.append(
        "Model gains are STARTING values — validate with empirical autotune + a cautious "
        "manual hover/flight before autonomous missions."
    )
    return DesignResult(airframe=af, gains=gains, warnings=warnings, plant_summary=plant_summary)
