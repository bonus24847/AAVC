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
