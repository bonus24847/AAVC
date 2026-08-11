"""Unit tests for the model-based gain synthesis (tuning.engine / synthesis / plant).

Checks the pole-placement formula (``P = 2ζωₙ/b``), the measured-plant
calibration override (the "identify the TF, then tune" path), the yaw-authority
guard, and the per-airframe required-field validation.
"""

import math

import pytest

from mission_brain.schemas import Airframe
from tuning.calibration import PlantCalibration
from tuning.engine import design_gains
from tuning.gains_io import load_gains, save_gains
from tuning.schemas import DesignRequest, PerfSpec, PhysicalParams

# AAVC x500-class quad.
QUAD = dict(
    mass_kg=2.0, ixx=0.02, iyy=0.02, izz=0.04,
    arm_length_m=0.25, n_motors=4, max_thrust_per_motor_n=10.0,
    prop_torque_coeff=0.05, motor_time_constant_s=0.02,
)


def _gains(res) -> dict[str, float]:
    return {g.param: g.value for g in res.gains}


def _req(spec: PerfSpec | None = None) -> DesignRequest:
    return DesignRequest(airframe=Airframe.QUADCOPTER, physical=PhysicalParams(**QUAD),
                         spec=spec or PerfSpec())


def test_rate_P_matches_pole_placement():
    spec = PerfSpec()
    g = _gains(design_gains(_req(spec)))
    # b_roll = τ_max/I = (n/2·F·arm·sin45)/ixx
    tau_max = (4 / 2) * 10.0 * 0.25 * math.sin(math.radians(45.0))
    b = tau_max / 0.02
    omega = 2.0 * math.pi * spec.rate_bandwidth_hz
    p_expected = 2.0 * spec.rate_damping * omega / b
    assert g["MC_ROLLRATE_P"] == pytest.approx(p_expected, rel=1e-4)
    # D = τ_motor·P on roll/pitch, zero on yaw.
    assert g["MC_ROLLRATE_D"] == pytest.approx(QUAD["motor_time_constant_s"] * p_expected, rel=1e-4)
    assert g["MC_YAWRATE_D"] == 0.0


def test_measured_calibration_raises_roll_P():
    # Measured b_roll (121) is below the model (176.8) ⇒ P ∝ 1/b raises roll P.
    cal = PlantCalibration(airframe="quadcopter",
                           b_measured={"roll": 121.0, "pitch": 207.0})
    g_model = _gains(design_gains(_req()))
    g_meas = _gains(design_gains(_req(), calibration=cal))
    assert g_meas["MC_ROLLRATE_P"] > g_model["MC_ROLLRATE_P"]
    # pitch measured b (207) is above model ⇒ lower P.
    assert g_meas["MC_PITCHRATE_P"] < g_model["MC_PITCHRATE_P"]


def test_yaw_authority_guard_keeps_model_b():
    # Measured b_yaw far below model → guard keeps model b + warns (no over-tune).
    cal = PlantCalibration(airframe="quadcopter", b_measured={"yaw": 15.0})
    res = design_gains(_req(), calibration=cal)
    g_meas = _gains(res)
    g_model = _gains(design_gains(_req()))
    assert g_meas["MC_YAWRATE_P"] == pytest.approx(g_model["MC_YAWRATE_P"], rel=1e-6)
    assert any("OVER-estimates yaw" in w for w in res.warnings)


def test_yaw_derate_allows_measured_b():
    # With a yaw bandwidth de-rate set, the measured yaw b IS applied (guard off).
    cal = PlantCalibration(airframe="quadcopter", b_measured={"yaw": 15.0})
    spec = PerfSpec(yaw_rate_bandwidth_hz=2.0)
    g = _gains(design_gains(_req(spec), calibration=cal))
    # yaw P uses measured b=15 at the de-rated bandwidth.
    omega = 2.0 * math.pi * 2.0
    assert g["MC_YAWRATE_P"] == pytest.approx(2.0 * spec.rate_damping * omega / 15.0, rel=1e-4)


def test_missing_quad_fields_rejected():
    with pytest.raises(ValueError):
        DesignRequest(airframe=Airframe.QUADCOPTER,
                      physical=PhysicalParams(mass_kg=2.0, ixx=0.02, iyy=0.02, izz=0.04))


def test_gains_io_round_trip(tmp_path):
    # Persisted tuned gains survive a save/load (the mission auto-load path).
    save_gains({"MC_ROLLRATE_P": 0.436, "MC_YAWRATE_P": 1.06}, source="test", out_dir=tmp_path)
    g = load_gains(out_dir=tmp_path)
    assert g == {"MC_ROLLRATE_P": 0.436, "MC_YAWRATE_P": 1.06}


def test_gains_io_missing_is_empty(tmp_path):
    # A missing/never-tuned file must NOT break a mission launch — returns {}.
    assert load_gains(out_dir=tmp_path / "absent") == {}
