"""Test the flight-chunking logic for multi-egg deliveries."""

from __future__ import annotations

from mission_brain.flights import (
    budgeted_flights_for,
    chunk_flights,
    max_flights_for,
    remaining_owed,
)


def test_eggs_aboard_1_is_one_delivery_per_flight():
    assert chunk_flights([3, 1, 4, 6], 1) == [[3], [1], [4], [6]]


def test_eggs_aboard_4_is_a_single_flight():
    assert chunk_flights([3, 1, 4, 6], 4) == [[3, 1, 4, 6]]


def test_eggs_aboard_2_pairs_in_order():
    assert chunk_flights([3, 1, 4, 6], 2) == [[3, 1], [4, 6]]


def test_ragged_last_chunk():
    assert chunk_flights([3, 1, 4], 2) == [[3, 1], [4]]


def test_empty_queue_is_no_flights():
    assert chunk_flights([], 4) == []


def test_eggs_aboard_below_one_is_treated_as_one():
    assert chunk_flights([3, 1], 0) == [[3], [1]]


def test_max_flights_for():
    assert max_flights_for(4, 1) == 4
    assert max_flights_for(4, 4) == 1
    assert max_flights_for(4, 2) == 2
    assert max_flights_for(3, 2) == 2
    assert max_flights_for(0, 4) == 0


# ── I5 (review 2026-07-24): recovery-flight budgeting ──────────────────────


def test_budgeted_flights_for_adds_a_recovery_flight_at_the_shipping_default():
    """eggs_aboard=4, 4 deliveries: max_flights_for alone is 1 (the whole
    queue fits one flight) — no spare capacity if that flight comes home
    with undelivered eggs. Budget one recovery flight beyond it."""
    assert max_flights_for(4, 4) == 1
    assert budgeted_flights_for(4, 4) == 2


def test_budgeted_flights_for_eggs_aboard_1_is_unchanged():
    """eggs_aboard=1 already needs >= 2 flights for any realistic delivery
    count, so the recovery-flight floor changes nothing here."""
    assert budgeted_flights_for(4, 1) == 4
    assert budgeted_flights_for(1, 1) == 2  # the one case the floor DOES lift
    assert budgeted_flights_for(2, 1) == 2


def test_budgeted_flights_for_zero_deliveries_is_zero_flights():
    """Nothing to ever deliver — no recovery flight to budget either. The
    gate already ends the mission on an empty chunk before any flight
    starts, so a spare flight here would only inflate a number nothing
    reads."""
    assert budgeted_flights_for(0, 4) == 0
    assert budgeted_flights_for(0, 1) == 0


def test_remaining_owed_is_the_queue_minus_what_was_delivered():
    assert remaining_owed([3, 1, 4, 6], [3, 1]) == [4, 6]


def test_remaining_owed_preserves_queue_order():
    assert remaining_owed([3, 1, 4, 6], [4, 6]) == [3, 1]


def test_remaining_owed_nothing_delivered_owes_everything():
    assert remaining_owed([3, 1, 4, 6], []) == [3, 1, 4, 6]


def test_remaining_owed_everything_delivered_owes_nothing():
    assert remaining_owed([3, 1, 4, 6], [3, 1, 4, 6]) == []


def test_remaining_owed_is_a_multiset_subtraction_for_duplicate_ids():
    """A duplicate-id queue (e.g. headless --assigned-ids "3,3,3,3") must
    only cancel as many owed 3's as were actually delivered, not collapse
    to a single distinct id via set subtraction."""
    assert remaining_owed([3, 3, 3, 3], [3, 3]) == [3, 3]


def test_remaining_owed_ignores_deliveries_outside_the_queue():
    """A manual per-flight GO override can serve an id that was never
    queued at all (state.eggs_aboard == 1, or no chunk to fly) — that
    delivery must not cancel a DIFFERENT, still-owed queue entry."""
    assert remaining_owed([3, 1], [5]) == [3, 1]
