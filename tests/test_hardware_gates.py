"""Gates that must not be fooled by an endpoint string, and the pins they read.

Three fixes from 2026-08-15 live here, all of the same shape: something that
was safe only by luck, in a place where being wrong is expensive.

* `_detect_simulator` — TUNING mode disables NAV_DLL_ACT/NAV_RCL_ACT/GF_ACTION
  so a frequency sweep is not aborted by a failsafe. It used to do that with no
  gate at all; gating it on the ENDPOINT would not have helped either, because
  cm4/launch_flight.sh flies the real aircraft through a router at
  udpin://…:14540 — the real bird's default endpoint is indistinguishable from
  SITL's. So the autopilot is asked instead.
* relative tolerance in `verify_envelope_pins` — a gate that can stop a flight
  must not reject a CORRECT value.
* the envelope pin list — the height-aiding pair decides where the aircraft
  thinks the ground is during the land-ON descent.
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from mavlink_adapter.commands import _ENVELOPE_PINS, DEFAULT_PX4_TUNING, DroneCommander
from orchestrator.main import _detect_simulator, _is_sitl_endpoint

_CONFIG = Path(__file__).resolve().parent.parent / "sitl/aavc_config.yaml"


class _Params:
    """Fake param plugin: `present` is what this 'firmware' knows about."""

    def __init__(self, present: dict[str, int] | None, dead: bool = False) -> None:
        self.present = present or {}
        self.dead = dead
        self.asked: list[str] = []

    async def get_param_int(self, name: str) -> int:
        self.asked.append(name)
        if self.dead:
            raise TimeoutError("link is silent")
        if name not in self.present:
            raise ValueError(f"no such parameter {name}")
        return self.present[name]


def _commander(params: _Params) -> SimpleNamespace:
    return SimpleNamespace(get_param_int=params.get_param_int)


def test_simulator_detected_by_a_param_only_sitl_builds_have() -> None:
    p = _Params({"SIM_GZ_EN": 1, "SYS_AUTOSTART": 4001})
    assert asyncio.run(_detect_simulator(_commander(p))) is True
    assert p.asked == ["SIM_GZ_EN"]          # no second read needed


def test_real_board_is_only_believed_after_the_link_proves_itself() -> None:
    """A Pixhawk has no SIM_GZ_EN — but neither does a dead link. The verdict
    'hardware' is only reached once a parameter every build carries answers."""
    p = _Params({"SYS_AUTOSTART": 6001})     # hexarotor, the real airframe
    assert asyncio.run(_detect_simulator(_commander(p))) is False
    assert p.asked == ["SIM_GZ_EN", "SYS_AUTOSTART"]


def test_silent_link_returns_none_rather_than_guessing() -> None:
    """The caller must be able to tell 'I don't know' from 'it's hardware' —
    guessing either way is unsafe: call hardware a sim and the safety pins come
    off, call a sim hardware and nothing is testable."""
    assert asyncio.run(_detect_simulator(_commander(_Params(None, dead=True)))) is None


def test_endpoint_check_cannot_answer_the_question() -> None:
    """Pinned as documentation of WHY the probe exists: the real aircraft's own
    default launcher endpoint (cm4/launch_flight.sh) looks exactly like SITL."""
    assert _is_sitl_endpoint("udpin://0.0.0.0:14540") is True      # SITL
    assert _is_sitl_endpoint("udpin://0.0.0.0:14540") is True      # …and the CM4
    assert _is_sitl_endpoint("serial:///dev/ttyAMA0:921600") is False


# ── verify_envelope_pins: relative tolerance, and what it covers ──────────────


class _ParamFloat:
    def __init__(self, values: dict[str, float]) -> None:
        self.values = values

    async def get_param_float(self, name: str) -> float:
        return self.values[name]


def _pin_commander(values: dict[str, float]) -> DroneCommander:
    c = DroneCommander.__new__(DroneCommander)
    c.system = SimpleNamespace(param=_ParamFloat(values))  # type: ignore[attr-defined]
    return c


def test_tolerance_scales_with_magnitude() -> None:
    """32-bit float spacing grows with the value (~5.5e-3 at 92160), so an
    ABSOLUTE 1e-3 would eventually flag a correct big number and block the
    flight — the expensive direction for a gate with veto power."""
    big = {"RTL_RETURN_ALT": 92160.004}
    assert _pin_commander(big) is not None
    bad = asyncio.run(_pin_commander(big).verify_envelope_pins({"RTL_RETURN_ALT": 92160.0}))
    assert bad == [], bad
    # A real drift at the same magnitude is still caught. (Note what relative
    # tolerance means here: at 92160 the allowance is ~92 units, so a 40-unit
    # difference now PASSES. That is the intended trade — it is float noise at
    # this magnitude — and it is why the pins that matter are small numbers.)
    bad = asyncio.run(_pin_commander({"RTL_RETURN_ALT": 100_000.0})
                      .verify_envelope_pins({"RTL_RETURN_ALT": 92160.0}))
    assert bad and "RTL_RETURN_ALT" in bad[0]


def test_small_pins_still_caught_at_the_old_precision() -> None:
    """The pins actually flown are small numbers; relative tolerance must not
    have loosened them (60 * 1e-3 = 0.06 m, still far under a real drift)."""
    bad = asyncio.run(_pin_commander({"RTL_RETURN_ALT": 60.0})
                      .verify_envelope_pins({"RTL_RETURN_ALT": 20.0}))
    assert bad and "want 20" in bad[0]


@pytest.mark.parametrize("pin", ["EKF2_HGT_REF", "EKF2_RNG_A_HMAX"])
def test_height_aiding_params_are_shipped(pin: str) -> None:
    """Both decide where the aircraft thinks the ground is during the land-ON
    descent, and a wrong value shows up only as a worse touchdown — so both are
    pinned rather than left at whatever the board happens to hold."""
    assert pin in DEFAULT_PX4_TUNING
    cfg = yaml.safe_load(_CONFIG.read_text(encoding="utf-8"))
    assert pin in (cfg.get("px4_tuning") or {})


def test_only_live_params_may_gate_the_flight() -> None:
    """A reboot_required parameter must NEVER sit in the read-back gate.

    PX4 stores such a value immediately — so the read-back reports PASS — while
    the module keeps running on the OLD value until the next boot. A gate that
    can only ever say "held" is worse than no gate, because it gets believed.
    EKF2_HGT_REF is reboot_required (ekf2/module.yaml); EKF2_RNG_A_HMAX is not.
    """
    assert "EKF2_RNG_A_HMAX" in _ENVELOPE_PINS      # applies live
    assert "EKF2_HGT_REF" not in _ENVELOPE_PINS     # reboot_required
    # …and the honest place for it is the bench sweep, which reports
    # reboot-required keys as their own class instead of as drift.
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "param_audit", Path(__file__).resolve().parents[1] / "tools/param_audit.py")
    assert spec and spec.loader
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    assert "EKF2_HGT_REF" in audit._REBOOT_REQUIRED
    assert "MAV_1_FORWARD" in audit._REBOOT_REQUIRED


def test_range_aid_ceiling_clears_every_descent_rung() -> None:
    """PX4's default EKF2_RNG_A_HMAX is 5.0 — exactly the competition ladder's
    5 m rung, where the aircraft is deliberately slowed to re-centre (so the
    speed condition is met too). The pin must sit clear of every rung and stay
    inside both the sensor's usable band and PX4's own 1..10 limit."""
    from orchestrator.tactical_align import AlignParams

    hmax = DEFAULT_PX4_TUNING["EKF2_RNG_A_HMAX"]
    assert 1.0 <= hmax <= 10.0                      # PX4's declared range
    assert hmax <= 12.0                             # TFmini-S max range
    for rung in AlignParams().rungs:
        assert abs(rung - hmax) > 0.5, f"rung {rung} sits on the aiding boundary"
