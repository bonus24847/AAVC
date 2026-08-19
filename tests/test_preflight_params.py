"""``_board_ok`` decides whether a live FC reading passes the BOARD check.

The exact-match rule is right for every param EXCEPT ``BAT1_CAPACITY``. Any
value ``<= 0`` selects PX4's voltage-only state-of-charge branch
(``estimateStateOfCharge``'s ``else``), which is what this airframe flies on
after the PM03D was removed. The expected value is written as ``-1``, but a
board reading ``0`` — or any non-positive value, e.g. after a QGC battery-cal
reset — is equally correct, and the runtime gate (``main.py``:
``if fc_capacity <= 0``) accepts it. So the field-day check must not STOP the
flight over a ``0`` that flies fine (the old exact-``-1`` compare did).
"""

from tools.preflight_params import _board_ok


def test_bat1_capacity_accepts_any_non_positive() -> None:
    assert _board_ok("BAT1_CAPACITY", -1.0, -1.0)
    assert _board_ok("BAT1_CAPACITY", 0.0, -1.0)
    assert _board_ok("BAT1_CAPACITY", -5.0, -1.0)


def test_bat1_capacity_rejects_positive() -> None:
    # a positive capacity re-arms the current-fused gauge — the optimistic
    # branch the PM03D removal was meant to leave behind
    assert not _board_ok("BAT1_CAPACITY", 500.0, -1.0)
    assert not _board_ok("BAT1_CAPACITY", 17000.0, -1.0)


def test_exact_match_params_unchanged() -> None:
    assert _board_ok("PWM_MAIN_FUNC1", 101.0, 101.0)
    assert not _board_ok("PWM_MAIN_FUNC1", 0.0, 101.0)
    assert _board_ok("SYS_AUTOSTART", 6001.0, 6001.0)
