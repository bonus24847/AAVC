"""Unit tests for the per-sortie energy budget (mirrors test_time_policy)."""

import math
from types import SimpleNamespace

import pytest

from orchestrator.energy_policy import (
    EnergyPolicy,
    baseline_for_pack,
    detect_battery_swap,
    energy_consumed_mah,
)

POLICY = EnergyPolicy(capacity_mah=7500.0, reserve_frac=0.25,
                      seed_sortie_mah=1700.0, margin_mah=150.0)


def test_usable_excludes_the_fc_reserve():
    assert POLICY.usable_mah() == 5625.0


def test_cost_uses_the_seed_until_a_sortie_has_been_measured():
    assert POLICY.sortie_cost_mah([]) == 1700.0


def test_seed_scales_with_the_eggs_carried_in_one_flight():
    """A FLIGHT is no longer one delivery. The 1700 mAh seed is 29 A x 3.5 min
    = ONE old sortie; a 4-egg flight adds three more ~110 s serves at the same
    hover current, so the seed must grow with eggs_aboard or the pre-flight
    gate compares against a cost ~3x too low and shows a falsely green card."""
    p = EnergyPolicy(capacity_mah=7500.0, reserve_frac=0.25,
                     seed_sortie_mah=1700.0, seed_delivery_mah=900.0,
                     margin_mah=150.0, eggs_aboard=4)
    assert p.seed_flight_mah() == 1700.0 + 3 * 900.0
    assert p.sortie_cost_mah([]) == 4400.0
    # eggs_aboard=1 is the original one-delivery flight, unchanged.
    assert POLICY.seed_flight_mah() == 1700.0


def test_a_pack_that_cannot_finish_a_four_egg_flight_is_refused():
    """The gate must not approve a flight the pack cannot finish. 5625 usable
    - 2000 already drawn = 3625 left, under the 4400 + 150 a 4-egg flight
    needs — even though it comfortably covers a single old-style sortie."""
    p = EnergyPolicy(capacity_mah=7500.0, reserve_frac=0.25,
                     seed_sortie_mah=1700.0, seed_delivery_mah=900.0,
                     margin_mah=150.0, eggs_aboard=4)
    ok, reason = p.can_start_sortie(consumed_mah=2000.0, history=[])
    assert not ok
    assert "swap" in reason.lower()
    # The same pack state IS enough for a one-egg flight.
    assert POLICY.can_start_sortie(consumed_mah=2000.0, history=[])[0]


def test_measured_flights_still_beat_the_seed():
    """The seed only covers the first flight; once a whole 4-egg flight has
    been measured, the median of measurements wins (they already include
    every delivery)."""
    p = EnergyPolicy(capacity_mah=7500.0, reserve_frac=0.25,
                     seed_sortie_mah=1700.0, seed_delivery_mah=900.0,
                     margin_mah=150.0, eggs_aboard=4)
    assert p.sortie_cost_mah([3800.0, 4000.0, 4200.0]) == 4000.0


def test_cost_is_the_median_of_measured_sorties():
    assert POLICY.sortie_cost_mah([1500.0, 1600.0, 2000.0]) == 1600.0


def test_cost_ignores_junk_entries():
    assert POLICY.sortie_cost_mah([float("nan"), -5.0, 1600.0]) == 1600.0


def test_allows_a_sortie_when_the_pack_can_cover_it():
    ok, reason = POLICY.can_start_sortie(consumed_mah=1000.0, history=[])
    assert ok
    assert "1700" in reason


def test_blocks_when_the_remaining_charge_cannot_cover_the_next_sortie():
    # 5625 usable - 4200 used = 1425 left, under 1700 + 150 margin
    ok, reason = POLICY.can_start_sortie(consumed_mah=4200.0, history=[])
    assert not ok
    assert "swap" in reason.lower()


def test_boundary_is_cost_plus_margin():
    exact = POLICY.usable_mah() - (1700.0 + 150.0)
    assert POLICY.can_start_sortie(exact, [])[0]
    assert not POLICY.can_start_sortie(exact + 1.0, [])[0]


def test_unknown_consumption_allows_the_sortie_rather_than_grounding_it():
    ok, reason = POLICY.can_start_sortie(float("nan"), [])
    assert ok
    assert "unknown" in reason.lower()


def test_sorties_remaining_is_the_headline_number():
    # 5625 usable, 1700 per sortie, nothing used yet
    assert round(POLICY.sorties_remaining(0.0, []), 2) == 3.31
    assert POLICY.sorties_remaining(5625.0, []) == 0.0
    assert math.isnan(POLICY.sorties_remaining(float("nan"), []))


# ── two-tier consumption reading ──

def _telem(consumed=float("nan"), percent=float("nan"), stamped=1000.0):
    return SimpleNamespace(battery_consumed_mah=consumed, battery_percent=percent,
                           battery_consumed_monotonic=stamped)


def test_tier_a_prefers_the_flight_controller_coulomb_count():
    mah, tier = energy_consumed_mah(_telem(consumed=1234.0, percent=80.0), 7500.0,
                                    now_monotonic=1001.0)
    assert (round(mah), tier) == (1234, "A")


def test_a_stale_coulomb_count_is_demoted_to_the_percent_estimate():
    """The count rides the OPTIONAL raw-MAVLink listener. If that feed dies the
    number freezes, and a frozen 'measured' value would shadow the percentage
    estimate that is still updating — so the budget would plan against a pack
    that appears to have stopped draining."""
    mah, tier = energy_consumed_mah(_telem(consumed=1234.0, percent=80.0), 7500.0,
                                    now_monotonic=1000.0 + 60.0)
    assert (round(mah), tier) == (1500, "B")


def test_an_unstamped_coulomb_count_is_not_trusted_as_measured():
    t = SimpleNamespace(battery_consumed_mah=1234.0, battery_percent=80.0)
    assert energy_consumed_mah(t, 7500.0)[1] == "B"


def test_tier_b_falls_back_to_percent_times_capacity():
    mah, tier = energy_consumed_mah(_telem(percent=80.0), 7500.0)
    assert (round(mah), tier) == (1500, "B")


def test_no_tier_when_neither_signal_is_available():
    mah, tier = energy_consumed_mah(_telem(), 7500.0)
    assert math.isnan(mah)
    assert tier == "none"


def test_a_negative_coulomb_count_is_not_trusted():
    # PX4 reports a negative sentinel when the module cannot measure; SITL
    # sends -383, which as a raw number would read as charge PUT BACK.
    mah, tier = energy_consumed_mah(_telem(consumed=-383.0, percent=90.0), 7500.0)
    assert (round(mah), tier) == (750, "B")


# ── battery swap ──

def test_swap_detected_when_the_coulomb_count_resets():
    assert detect_battery_swap(prev_mah=4200.0, now_mah=12.0,
                               prev_pct=30.0, now_pct=98.0)


def test_swap_detected_on_a_large_percent_jump_alone():
    assert detect_battery_swap(prev_mah=float("nan"), now_mah=float("nan"),
                               prev_pct=28.0, now_pct=95.0)


def test_normal_discharge_is_not_a_swap():
    assert not detect_battery_swap(prev_mah=1000.0, now_mah=1400.0,
                                   prev_pct=80.0, now_pct=74.0)


def test_noise_on_a_full_pack_is_not_a_swap():
    assert not detect_battery_swap(prev_mah=20.0, now_mah=8.0,
                                   prev_pct=99.0, now_pct=100.0)


# ── the FORCE escape must stay alive (the 2026-07-15 dead-path lesson) ──

def test_energy_preflight_row_is_advisory_not_critical(tmp_path):
    """A critical energy row would freeze the board exactly when FORCE is
    needed: the GO endpoint checks all_critical_pass WITHOUT a force escape, so
    a critical row cannot be overridden. The refusal lives in
    state.sortie_energy_ok instead, which the endpoint checks `or force`."""
    from tests.test_preflight import _report_for_healthy_state

    report = _report_for_healthy_state(tmp_path, sortie_energy_ok=False,
                                       energy_detail="pack is flat")
    row = next(i for i in report.items if i.id == "energy")
    assert row.critical is False
    assert row.detail == "pack is flat"
    # the board must still be green so FORCE has something to override
    assert report.all_critical_pass


# ── pack swaps: the baseline must describe the pack that is actually fitted ──


def test_a_full_spare_starts_from_the_meter_reading():
    # Fresh pack reads 100%: nothing of it has been used yet.
    assert baseline_for_pack(now_mah=0.0, now_pct=100.0, capacity_mah=7500.0) == 0.0


def test_a_part_used_spare_is_not_budgeted_as_full():
    """The bug this guards: rebasing to the raw meter reading assumes every
    spare arrives full, so a 60% pack would be planned as if it held 7,500 mAh —
    the gate approves a sortie it cannot finish and the FC's low-battery failsafe
    ends it mid-flight with the egg aboard."""
    base = baseline_for_pack(now_mah=0.0, now_pct=60.0, capacity_mah=7500.0)
    assert base == pytest.approx(-3000.0)
    # …so a reading of 0 mAh on this pack already counts as 3,000 mAh used,
    # leaving 2,625 of the 5,625 usable — not the whole pack.
    assert POLICY.usable_mah() - (0.0 - base) == pytest.approx(2625.0)


def test_a_part_used_spare_is_refused_when_it_cannot_cover_a_sortie():
    base = baseline_for_pack(now_mah=0.0, now_pct=30.0, capacity_mah=7500.0)
    ok, reason = POLICY.can_start_sortie(0.0 - base, [])
    assert not ok and "swap" in reason.lower()


def test_no_charge_signal_falls_back_to_assuming_the_spare_is_full():
    # Coarse, but the alternative (refusing to fly) grounds a serviceable
    # aircraft; the FC's own failsafe is still the hard stop.
    assert baseline_for_pack(1234.0, float("nan"), 7500.0) == 1234.0


def test_swap_detection_spans_the_resupply_hold():
    """Exit of sortie N vs entry of sortie N+1 — the only window in which the
    crew can touch the pack. Checked within one sortie instead, as it was until
    2026-07-22, the aircraft is armed and airborne throughout and no real swap
    could ever be seen."""
    # Sortie N ended at 3,400 mAh drawn / 38%; a fresh pack reads 0 / 100%.
    assert detect_battery_swap(3400.0, 0.0, 38.0, 100.0)
    # …and simply flying the next sortie is not a swap.
    assert not detect_battery_swap(3400.0, 3500.0, 38.0, 37.0)
