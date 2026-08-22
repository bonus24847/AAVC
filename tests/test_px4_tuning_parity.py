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


# ── the height stance is a decision, not a per-field preference ────────────
#
# 2026-08-20 at KMUTNB, on EKF2_HGT_REF=1 (GPS, PX4's default): baro-vs-GPS
# divergence 10.8 m peak-to-peak inside one flight, reported AGL inflating to
# 12.0 m while the aircraft physically held ~8.5 m, and the ceiling watchdog —
# correct on its inputs — RTH'd an aircraft that was tracking transit to 1.7 m.
# Flight 3 the same afternoon on =0 (baro): transit 3/3 at 1.4-2.0 m, no
# altitude event at all.
#
# The practice config moved to baro on that evidence; kmitl_config.yaml kept
# GPS for three more days on the reasoning that a 20 m ceiling has more margin.
# It has 2.5 m (transit commanded 19.5, watchdog RTH at 22) against a measured
# 10.8 m. Operator closed it 2026-08-23: baro everywhere, lidar still pinning
# the last few metres. This walks EVERY field config so the next one cannot
# quietly ship PX4's default again.

_FIELD_CONFIGS = sorted((Path(__file__).resolve().parent.parent / "sitl")
                        .glob("*config.yaml"))


def test_every_field_flies_baro_height_with_lidar_aiding() -> None:
    assert len(_FIELD_CONFIGS) >= 2, f"expected both fields, found {_FIELD_CONFIGS}"
    for path in _FIELD_CONFIGS:
        tune = (yaml.safe_load(path.read_text(encoding="utf-8")) or {})["px4_tuning"]
        assert tune["EKF2_HGT_REF"] == 0, (
            f"{path.name}: height reference must be BARO (0). 1 = GPS is PX4's "
            "default and cost a flight on 2026-08-20; 2 = range makes the local "
            "origin ride ground level, so a shed cargo box would move 'down'")
        assert tune["EKF2_RNG_CTRL"] == 1, f"{path.name}: lidar aiding off"
        assert tune["EKF2_RNG_A_HMAX"] == 7.0, f"{path.name}: aiding ceiling moved"
        assert tune["EKF2_OF_CTRL"] == 0, (
            f"{path.name}: optical flow on, and there is no flow module aboard")


def test_the_code_fallback_agrees_with_the_fields() -> None:
    """DEFAULT_PX4_TUNING is what a config-absent run flies. It has carried
    baro since 2026-08-20 — pin that it cannot drift back while the configs
    move on."""
    assert DEFAULT_PX4_TUNING["EKF2_HGT_REF"] == 0.0
    assert DEFAULT_PX4_TUNING["EKF2_RNG_CTRL"] == 1.0
    assert DEFAULT_PX4_TUNING["EKF2_OF_CTRL"] == 0.0
