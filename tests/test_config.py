"""Regression-lock the competition configuration constants (C3).

profile.py's docstring promises tests/test_config.py locks the COMPETITION
profile so its values can't silently drift from the watchdog/detector/config
copies. This is that test — plus the terminal-align competition values and the
egg-release servo config, the other rules-derived numbers with no other single
guard.
"""

from __future__ import annotations

import pytest

from mavlink_adapter.commands import ConnectionConfig
from mission_brain.profile import COMPETITION
from orchestrator.constants import (
    CEILING_BREACH_M,
    CEILING_WARN_M,
    TOUCHDOWN_ALT_GUARD_M,
)
from orchestrator.tactical_align import AlignParams


def test_competition_profile_is_locked() -> None:
    p = COMPETITION
    assert p.name == "competition"
    assert p.operation_window_s == 1200.0        # AAVC 20-min window
    assert p.min_time_remaining_s == 180.0
    assert p.rth_battery_pct == 15.0    # 30 -> 15, operator 2026-08-27
    assert p.land_battery_pct == 10.0
    assert p.datalink_loss_threshold_s == 5.0
    assert p.gps_loss_threshold_s == 5.0
    assert p.telemetry_stale_threshold_s == 10.0
    assert p.geofence_margin_m == 5.0
    assert p.altitude_floor_m == 3.0
    assert p.altitude_ceiling_m == 30.0          # 30 m ceiling (briefing 2026-08-28; was 20)
    assert p.transit_alt_m == 20.0               # transit strictly at 20 m (briefing left it)
    assert p.search_floor_m == 10.0              # search ≥ 10 m
    assert p.drop_count_max == 1                 # one egg aboard
    assert p.max_sorties == 4                    # ≤4 pads, one flight each
    assert p.eggs_aboard == 4                    # briefing: carry all 4 in one flight
    assert p.drop_without_confirmation is False  # never land on an unconfirmed pad
    assert p.default_target == "aruco landing pad"


def test_align_params_competition_defaults_are_locked() -> None:
    a = AlignParams()
    # Descend rungs high→low; the FINAL tolerance is 0.2 m (0.35 is the 3 m rung).
    assert a.rungs == (12.0, 8.0, 5.0, 3.0, 2.0)   # 1.5 m dropped 2026-08-28 (KMITL trial)
    assert a.rung_tol_m == (1.5, 1.0, 0.6, 0.35, 0.25)
    assert a.rung_tol_m[-1] == 0.25
    assert a.settle_after_land_s == 2.0          # gentle post-touchdown pause
    assert a.land_alt_threshold_m == 1.5         # touchdown-confirm altitude
    assert a.gps_fallback is False               # defer, never land blind
    assert a.require_id_votes == 1               # decoded assigned id before LAND
    assert a.target_radius_m == 0.2              # marker-equivalent size prior
    assert a.frame_max_age_s == 2.0              # in-flight staleness gate (S2)


def test_egg_release_servo_config_is_locked() -> None:
    c = ConnectionConfig()
    # KMUTNB 2026-08-11: base actuator-set index 1 (DO_SET_ACTUATOR 1..4 —
    # PX4 has no DO_SET_SERVO handler; the old AUX-9 numbering addressed a
    # command that was never implemented).
    assert c.drop_servo_channel == 1
    assert c.drop_servo_pwm_release == 1900
    assert c.drop_servo_pwm_hold == 1100
    assert c.drop_servo_relatch is False         # latch stays open after release (2026-08-28)
    assert c.drop_payload_count == 1             # one release mechanism → payload_id 0
    # Empty map = the historical base+offset progression; the shipped config
    # (below) supplies the real rack's as-wired order.
    assert c.drop_servo_channels == ()
    assert c.actuator_index(0) == 1


def test_shipped_drop_servo_channels_match_the_as_wired_rack() -> None:
    """AS-WIRED 2026-08-15 (docs/SERVO_AUX_MAPPING.md): the four egg latches
    are on AUX 4 / 1 / 2 / 3 for the front-left / rear-right / front-right /
    rear-left corners, and the mission releases in that DIAGONAL order so the
    CG moment stays ~zero full, after two eggs, and empty. The map lives in
    config, not in code — but a silent edit of it releases the wrong egg on
    the right pad, which nothing else in the suite would notice."""
    from pathlib import Path

    import yaml

    cfg = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml").read_text())
    conn = cfg["connection"]
    assert conn["drop_servo_channels"] == [4, 1, 2, 3]
    assert conn["drop_payload_count"] == 4
    c = ConnectionConfig(drop_payload_count=4,
                         drop_servo_channels=tuple(conn["drop_servo_channels"]))
    assert [c.actuator_index(p) for p in range(4)] == [4, 1, 2, 3]


def test_drop_servo_channels_reject_an_unflyable_map() -> None:
    """The map addresses MAV_CMD_DO_SET_ACTUATOR sets 1..6 and must give every
    egg its own latch (rules §7: independent release mechanisms). All three
    ways of getting it wrong fail at config-build time — on the ground — not
    at the release, 1 m over a pad with an egg aboard."""
    with pytest.raises(ValueError, match="out of range"):
        ConnectionConfig(drop_payload_count=4, drop_servo_channels=(4, 1, 2, 7))
    with pytest.raises(ValueError, match="repeats a channel"):
        ConnectionConfig(drop_payload_count=4, drop_servo_channels=(4, 1, 2, 1))
    with pytest.raises(ValueError, match="every egg slot needs its own"):
        ConnectionConfig(drop_payload_count=4, drop_servo_channels=(4, 1, 2))
    # A YAML list is normalised to the frozen dataclass's tuple.
    assert ConnectionConfig(drop_payload_count=4,
                            drop_servo_channels=[4, 1, 2, 3]).drop_servo_channels \
        == (4, 1, 2, 3)


def test_gimbal_is_off_because_none_is_fitted() -> None:
    """No gimbal on the aircraft (operator, 2026-08-16) — the camera is hard
    mounted. Pushing MNT_* at a board with no mount only fills the anomaly log,
    so the block must ship DISABLED. The params stay so re-fitting is one line.
    """
    from pathlib import Path

    import yaml

    cfg = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml").read_text())
    gimbal = cfg["gimbal"]
    assert gimbal["enabled"] is False
    assert gimbal["params"] == {
        # MODE_OUT 0 = drive the AUX PWM servo. It shipped as 1 (= MAVLink
        # gimbal protocol v1, i.e. talk to a gimbal device this aircraft does
        # not have) behind a comment that claimed 1 meant AUX — which would have
        # left the nadir camera unstabilized and the G5 check rubber-stamping it.
        "MNT_MODE_IN": 4, "MNT_MODE_OUT": 0, "MNT_DO_STAB": 1,
        "MNT_RANGE_PITCH": 90.0, "MNT_OFF_PITCH": 0.0,
        # INT32 channel selector (0 = no manual RC pitch channel), not an angle.
        "MNT_MAN_PITCH": 0,
    }


def test_shared_envelope_constants() -> None:
    # The touchdown guard is the touchdown threshold (1.5) + 1.0 m frame-drift.
    assert TOUCHDOWN_ALT_GUARD_M == AlignParams().land_alt_threshold_m + 1.0
    assert CEILING_WARN_M == 0.5
    # KMUTNB: breach band 1.5 (RTH at ceiling+1.5 = 6.5 m on the 5 m field —
    # proportionate to the small band; 2.0 tolerated a 40%-over hold).
    assert CEILING_BREACH_M == 1.5


def test_field_configs_keep_the_latch_open_after_release() -> None:
    """Both field configs spell the 2026-08-28 decision out, and the loader
    honours the key (a typo'd key would silently keep the dataclass default —
    which happens to be the same value today, so pin the config text too)."""
    import yaml

    from orchestrator.main import _build_connection
    for path in ("sitl/kmitl_config.yaml", "sitl/aavc_config.yaml"):
        cfg = yaml.safe_load(open(path, encoding="utf-8"))
        assert cfg["connection"]["drop_servo_relatch"] is False, path
        cc = _build_connection(cfg["connection"], None)
        assert cc.drop_servo_relatch is False
        # The pymavlink fallback must aim at the CM4 router's loopback server
        # (what the status beacon uses), not PX4 SITL's 18570 (dead on the bird).
        assert cc.drop_fallback_endpoint == "udpout:127.0.0.1:14550", path
    assert _build_connection({"drop_servo_relatch": True}, None).drop_servo_relatch is True


def test_align_rungs_are_altitude_gated() -> None:
    """2026-08-28 (KMITL 17:28 flight): a rung counts only when the aircraft is AT
    its altitude, within max(0.3 m, 12 %). Without it LAND was commanded from
    5-9 m and the eggs landed 0.5-0.7 m off the marker."""
    a = AlignParams()
    assert a.rung_alt_tol_m == 0.3
    assert a.rung_alt_tol_frac == 0.12
    assert a.rung_bias_max_m == 2.0
    import yaml
    for path in ("sitl/kmitl_config.yaml", "sitl/aavc_config.yaml"):
        blk = yaml.safe_load(open(path, encoding="utf-8"))["align"]
        assert blk["rung_alt_tol_m"] == 0.3 and blk["rung_alt_tol_frac"] == 0.12, path

