"""Serve-order routing (mission_brain/serve_order.py).

The operator's click order is not a route. These lock the properties the flight
depends on: the route actually gets shorter, unknown pads never displace known
ones, and a missing position leaves the order alone instead of guessing.
"""

from __future__ import annotations

from mission_brain.serve_order import (
    distance_m,
    order_by_nearest,
    route_length_m,
)

# A field-sized layout, ~100 m across, in the KMUTNB latitude band.
HOME = (13.8228, 100.5116)
PADS = {
    1: (13.8228, 100.5126),      # ~108 m east of home
    2: (13.8228, 100.5118),      # ~22 m east  — nearest
    3: (13.8231, 100.5122),      # north-east, middle
    6: (13.8225, 100.5120),      # south-east, middle
}


def test_the_queue_order_is_replaced_by_a_shorter_route() -> None:
    """The point of the exercise: whatever the operator typed, the aircraft
    flies less. A reordering that does not shorten the route is a bug."""
    queue = [1, 2, 3, 6]                       # worst-ish: far pad first
    ordered = order_by_nearest(queue, PADS, HOME)
    assert sorted(ordered) == sorted(queue)    # same pads, no losses
    assert (route_length_m(ordered, PADS, HOME)
            < route_length_m(queue, PADS, HOME))


def test_it_finds_the_true_optimum_not_just_a_greedy_chain() -> None:
    """With four stops the search is exhaustive (24 routes), so the result must
    be the shortest of ALL orders — greedy nearest-neighbour can be beaten and
    this pins that we do not settle for it."""
    from itertools import permutations
    queue = [1, 2, 3, 6]
    ordered = order_by_nearest(queue, PADS, HOME)
    best = min(route_length_m(o, PADS, HOME) for o in permutations(queue))
    assert route_length_m(ordered, PADS, HOME) == best


def test_the_egress_leg_is_part_of_the_route() -> None:
    """Cheapest-to-reach-all and cheapest-to-reach-all-then-leave are different
    routes. The aircraft always flies the second one, so the ordering must be
    chosen against it: ending far from the egress point is a real cost."""
    egress = (13.8225, 100.5127)               # south-east corner
    with_end = order_by_nearest([1, 2, 3, 6], PADS, HOME, egress)
    assert (route_length_m(with_end, PADS, HOME, egress)
            <= route_length_m(order_by_nearest([1, 2, 3, 6], PADS, HOME),
                              PADS, HOME, egress))


def test_pads_the_sweep_never_found_go_last_in_queue_order() -> None:
    """An id with no registry entry has no position, so it cannot be placed on
    a route. The serve loop will record it not-found; letting it keep a middle
    slot would interleave a dead stop into a route picked to be short."""
    ordered = order_by_nearest([4, 1, 5, 2], PADS, HOME)
    assert ordered[-2:] == [4, 5]              # unknown, in the order given
    assert set(ordered[:2]) == {1, 2}


def test_no_position_means_no_reordering() -> None:
    """Without a fix there is nothing to measure from. Returning the queue
    untouched is honest; inventing an order would look like an optimisation and
    be a guess."""
    queue = [1, 2, 3, 6]
    assert order_by_nearest(queue, PADS, None) == queue
    assert order_by_nearest(queue, PADS, (float("nan"), 100.5)) == queue


def test_a_single_known_pad_is_returned_unchanged() -> None:
    assert order_by_nearest([3], PADS, HOME) == [3]
    assert order_by_nearest([], PADS, HOME) == []


def test_distance_is_metres_at_field_scale() -> None:
    """Sanity on the flat-earth approximation: 0.001 deg of latitude is ~111 m,
    and the longitude scale shrinks by cos(lat)."""
    assert 110.0 < distance_m((13.8228, 100.5116), (13.8238, 100.5116)) < 112.0
    east = distance_m((13.8228, 100.5116), (13.8228, 100.5126))
    assert 107.0 < east < 109.0
