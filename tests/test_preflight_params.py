"""``_board_ok`` decides whether a live FC reading passes the BOARD check.

The exact-match rule is right for every param EXCEPT ``BAT1_CAPACITY``. Any
value ``<= 0`` selects PX4's voltage-only state-of-charge branch
(``estimateStateOfCharge``'s ``else``), which is what this airframe flies on
after the PM03D was removed (the PM02D now feeding the FC senses avionics
draw only, so nothing changed). The expected value is written as ``-1``, but a
board reading ``0`` — or any non-positive value, e.g. after a QGC battery-cal
reset — is equally correct, and the runtime gate (``main.py``:
``if fc_capacity <= 0``) accepts it. So the field-day check must not STOP the
flight over a ``0`` that flies fine (the old exact-``-1`` compare did).
"""

from tools.preflight_params import BOARD, _board_ok


def test_board_checks_the_17000_pack_endpoints() -> None:
    # After the 2026-08-19 pack swap the field-day check must verify the voltage
    # endpoints too: with BAT1_CAPACITY=-1 the whole gauge is interpolate(cell_v,
    # V_EMPTY, V_CHARGED), so a board still holding the LiPo defaults reads a
    # wrong %. 25.1 V / 6 = 4.18; 22.6 V / 6 = 3.77.
    assert BOARD["BAT1_V_CHARGED"] == 4.18
    assert BOARD["BAT1_V_EMPTY"] == 3.77


def test_wrong_endpoint_is_flagged() -> None:
    # the LiPo-default 4.05 the board held before 2026-08-19 must NOT pass as 4.18
    assert not _board_ok("BAT1_V_CHARGED", 4.05, 4.18)
    # but float32 storage of 4.18 (reads back 4.17999983) still passes
    assert _board_ok("BAT1_V_CHARGED", 4.179999828, 4.18)


def test_bat1_capacity_accepts_any_non_positive() -> None:
    assert _board_ok("BAT1_CAPACITY", -1.0, -1.0)
    assert _board_ok("BAT1_CAPACITY", 0.0, -1.0)
    assert _board_ok("BAT1_CAPACITY", -5.0, -1.0)


def test_bat1_capacity_rejects_positive() -> None:
    # a positive capacity re-arms the current-fused gauge — the optimistic
    # branch the PM03D removal was meant to leave behind (and the PM02D's
    # avionics-only current would feed, wrongly, if this ever flipped)
    assert not _board_ok("BAT1_CAPACITY", 500.0, -1.0)
    assert not _board_ok("BAT1_CAPACITY", 17000.0, -1.0)


def test_exact_match_params_unchanged() -> None:
    assert _board_ok("PWM_MAIN_FUNC1", 101.0, 101.0)
    assert not _board_ok("PWM_MAIN_FUNC1", 0.0, 101.0)
    assert _board_ok("SYS_AUTOSTART", 6001.0, 6001.0)


def test_sys_hitl_zero_is_a_board_check() -> None:
    # The HITL firmware has no real actuator output (docs/HITL.md) — a board
    # left flagged SYS_HITL=1 is unflyable in exactly the silent way the BOARD
    # block exists to catch, and no mission-start pin touches it (G-7,
    # 2026-08-20).
    assert BOARD.get("SYS_HITL") == 0
    assert _board_ok("SYS_HITL", 0.0, 0.0)
    assert not _board_ok("SYS_HITL", 1.0, 0.0)


# ── boot-latched params: the config decides, and the BOARD must already hold it ──
#
# EKF2_HGT_REF picks the sensor the whole flight's altitude is measured against,
# and the EKF latches it the first time any height source fuses
# (EKF/height_control.cpp:61 returns early once _height_sensor_ref is set), so
# apply_param_overrides writing it at mission start changes the NEXT flight, not
# this one. It sat in neither list until 2026-08-23 — while already having cost
# a flight: 2026-08-20 on GPS reference, 10.8 m of baro-vs-GPS divergence, a
# 12.0 m "ceiling breach" against a 10 m ceiling, and a watchdog RTH on a flight
# that was tracking transit to 1.7 m.

def test_boot_latched_value_comes_from_the_field_config_not_a_constant(tmp_path) -> None:
    """The expectation is READ, not compiled in.

    Written when the two fields disagreed (practice baro, competition GPS) —
    which made the property self-evident and the test lazy. The operator closed
    that on 2026-08-23 and both now say baro, so the mechanism is proved the
    honest way instead: hand it a config saying something else and it must come
    back with that. A constant would sail through and be wrong the first time a
    field needs its own answer."""
    from pathlib import Path

    import yaml

    from tools.preflight_params import BOOT_LATCHED, boot_latched_expected

    assert "EKF2_HGT_REF" in BOOT_LATCHED
    root = Path(__file__).resolve().parents[1]
    for name in ("aavc_config.yaml", "kmitl_config.yaml"):
        assert boot_latched_expected(root / "sitl" / name)["EKF2_HGT_REF"] == 0.0

    odd = tmp_path / "somewhere_else.yaml"
    odd.write_text(yaml.safe_dump({"px4_tuning": {"EKF2_HGT_REF": 2}}))
    assert boot_latched_expected(odd) == {"EKF2_HGT_REF": 2.0}


def test_an_unreadable_config_skips_the_check_instead_of_inventing_a_value() -> None:
    """A preflight tool that reports a mismatch it cannot justify is worse than
    one that says nothing — the whole reason this file exists (see its
    docstring on the raw-pymavlink version that cried wolf)."""
    from pathlib import Path

    from tools.preflight_params import boot_latched_expected

    assert boot_latched_expected(None) == {}
    assert boot_latched_expected(Path("/nonexistent/config.yaml")) == {}


def test_the_site_marker_names_the_config_this_repo_flies() -> None:
    """Deliberately NOT pinned to a filename: this suite is synced verbatim to
    the competition repo, whose .aavc_site names the other field. What must
    hold in both is that the tool checks the config THIS repo actually flies,
    and reads the expectation out of that file rather than a constant."""
    import yaml

    from tools.preflight_params import _active_config_path, boot_latched_expected

    p = _active_config_path(None)
    assert p is not None and p.exists(), ".aavc_site names no readable config"
    want = (yaml.safe_load(p.read_text()) or {})["px4_tuning"]["EKF2_HGT_REF"]
    assert boot_latched_expected(p) == {"EKF2_HGT_REF": float(want)}
