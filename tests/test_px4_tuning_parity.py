"""DEFAULT_PX4_TUNING must match the shipped config's px4_tuning block (L4).

The config OVERRIDES DEFAULT_PX4_TUNING when present, and DEFAULT is the fallback
when the config is missing/corrupt. If DEFAULT silently omits a key the config
carries (it used to drop the whole XY precision-landing tune + the climb cap),
a config-absent run flies with PX4-stock gains and nobody notices. Pin parity.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from mavlink_adapter.commands import DEFAULT_PX4_TUNING

_CONFIG = Path(__file__).resolve().parent.parent / "sitl/aavc_config.yaml"


def _config_px4_tuning() -> dict[str, float]:
    cfg = yaml.safe_load(_CONFIG.read_text(encoding="utf-8"))
    return {k: float(v) for k, v in (cfg.get("px4_tuning") or {}).items()}


def test_default_tuning_keys_match_config() -> None:
    cfg = _config_px4_tuning()
    assert set(DEFAULT_PX4_TUNING) == set(cfg), (
        "DEFAULT_PX4_TUNING and config px4_tuning have drifted: "
        f"only in DEFAULT={set(DEFAULT_PX4_TUNING) - set(cfg)}, "
        f"only in config={set(cfg) - set(DEFAULT_PX4_TUNING)}"
    )


def test_default_tuning_values_match_config() -> None:
    cfg = _config_px4_tuning()
    for key, default_val in DEFAULT_PX4_TUNING.items():
        assert float(default_val) == cfg[key], f"{key}: DEFAULT {default_val} != config {cfg[key]}"


def test_auto_mode_accel_and_jerk_are_pinned() -> None:
    """Pinning MPC_ACC_HOR alone does not shape an AUTO mission.

    The mission flies AUTO end to end, and AUTO reads MPC_ACC_HOR_MAX and
    MPC_JERK_AUTO — not the MPC_ACC_HOR that was tuned. Leaving them at the
    PX4 defaults is why transit ran at 58% of its theoretical time
    (2026-07-20). If one of the pair is pinned, its AUTO counterpart must be.
    """
    cfg = _config_px4_tuning()
    if "MPC_ACC_HOR" in cfg:
        assert "MPC_ACC_HOR_MAX" in cfg, "MPC_ACC_HOR pinned but AUTO's cap is not"
        assert cfg["MPC_ACC_HOR_MAX"] >= cfg["MPC_ACC_HOR"], (
            "the AUTO accel cap must not sit below the tuned accel")
    assert "MPC_JERK_AUTO" in cfg, "AUTO jerk left at the PX4 default"


def test_takeoff_speed_never_exceeds_the_tuned_climb_cap() -> None:
    """MPC_TKO_SPEED is a ceiling risk: the takeoff climbs to within 2 m of the
    20 m transit altitude, so a takeoff faster than the climb cap that was
    tuned against a 19.68 m overshoot would spend that margin (b858abd)."""
    cfg = _config_px4_tuning()
    assert "MPC_TKO_SPEED" in cfg, (
        "unpinned MPC_TKO_SPEED rides the PX4 default and caps every takeoff")
    assert cfg["MPC_TKO_SPEED"] <= cfg["MPC_Z_VEL_MAX_UP"]


def test_autonomous_descent_speed_is_pinned() -> None:
    """MPC_Z_V_AUTO_DN governs every AUTO descent — including the one onto the
    pad with the egg aboard.

    Unpinned it rides the PX4 default of 1.5 m/s. The SITL runs that validated
    the 0.15-0.22 m release accuracy actually flew 0.4 m/s (a value that had
    persisted in the sim's parameter file), so a real 6X left unpinned would
    descend onto the pad ~4x faster than anything that was ever validated.
    tactical_align's rung ladder does NOT cover this: it steps
    MPC_Z_VEL_MAX_DN, which PX4 reads only in manual/offboard modes.
    """
    cfg = _config_px4_tuning()
    assert "MPC_Z_V_AUTO_DN" in cfg, (
        "the AUTO descent speed is unpinned — the real bird would use PX4's 1.5 m/s")
    assert cfg["MPC_Z_V_AUTO_DN"] <= cfg["MPC_Z_VEL_MAX_DN"]


def test_mission_restores_the_pinned_pad_descent_speed() -> None:
    """The L&R staging speed-up must hand back exactly the pinned value."""
    from orchestrator.mission import _LAND_STAGE_MPS, _PAD_DESCENT_MPS

    cfg = _config_px4_tuning()
    assert _PAD_DESCENT_MPS == cfg["MPC_Z_V_AUTO_DN"], (
        "mission.py restores a descent speed the config does not pin")
    assert _LAND_STAGE_MPS > _PAD_DESCENT_MPS
    assert _LAND_STAGE_MPS <= cfg["MPC_Z_VEL_MAX_DN"]
