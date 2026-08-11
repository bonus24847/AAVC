"""Regression-lock the competition configuration constants (C3).

profile.py's docstring promises tests/test_config.py locks the COMPETITION
profile so its values can't silently drift from the watchdog/detector/config
copies. This is that test — plus the terminal-align competition values and the
egg-release servo config, the other rules-derived numbers with no other single
guard.
"""

from __future__ import annotations

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
    assert p.rth_battery_pct == 30.0
    assert p.land_battery_pct == 20.0
    assert p.datalink_loss_threshold_s == 5.0
    assert p.gps_loss_threshold_s == 5.0
    assert p.telemetry_stale_threshold_s == 10.0
    assert p.geofence_margin_m == 5.0
    assert p.altitude_floor_m == 3.0
    assert p.altitude_ceiling_m == 20.0          # hard 20 m ceiling
    assert p.transit_alt_m == 20.0               # transit strictly at 20 m
    assert p.search_floor_m == 10.0              # search ≥ 10 m
    assert p.drop_count_max == 1                 # one egg aboard
    assert p.max_sorties == 4                    # ≤4 pads, one flight each
    assert p.eggs_aboard == 4                    # briefing: carry all 4 in one flight
    assert p.drop_without_confirmation is False  # never land on an unconfirmed pad
    assert p.default_target == "aruco landing pad"


def test_align_params_competition_defaults_are_locked() -> None:
    a = AlignParams()
    # Descend rungs high→low; the FINAL tolerance is 0.2 m (0.35 is the 3 m rung).
    assert a.rungs == (12.0, 8.0, 5.0, 3.0, 2.0, 1.5)
    assert a.rung_tol_m == (1.5, 1.0, 0.6, 0.35, 0.25, 0.2)
    assert a.rung_tol_m[-1] == 0.2
    assert a.settle_after_land_s == 2.0          # gentle post-touchdown pause
    assert a.land_alt_threshold_m == 1.5         # touchdown-confirm altitude
    assert a.gps_fallback is False               # defer, never land blind
    assert a.require_id_votes == 1               # decoded assigned id before LAND
    assert a.target_radius_m == 0.2              # marker-equivalent size prior
    assert a.frame_max_age_s == 2.0              # in-flight staleness gate (S2)


def test_egg_release_servo_config_is_locked() -> None:
    c = ConnectionConfig()
    assert c.drop_servo_channel == 9             # AUX 9 (confirm at G5)
    assert c.drop_servo_pwm_release == 1900
    assert c.drop_servo_pwm_hold == 1100
    assert c.drop_payload_count == 1             # one release mechanism → payload_id 0


def test_gimbal_config_block_is_locked() -> None:
    """The stabilized-nadir gimbal ships as a config block whose MNT_* values
    are VERIFY-AT-G5 candidates — lock the shipped schema so it can't silently
    drift before the bench pass."""
    from pathlib import Path

    import yaml

    cfg = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml").read_text())
    gimbal = cfg["gimbal"]
    assert gimbal["enabled"] is True
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
    assert CEILING_BREACH_M == 2.0
