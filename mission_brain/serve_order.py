"""Order the pads of one flight by flight distance, not by the click order.

The operator picks four ids out of six in the GCS queue editor. Until now that
click order WAS the delivery order (``orchestrator/mission.py`` iterated
``flight_ids`` directly), which is an arbitrary basis for a route: the pads are
scattered across the search area, and the aircraft can easily cross the field
twice serving them in the order they happened to be typed. Operator, 2026-08-18:
"ให้ส่งตาม path ที่ใกล้ที่สุดก่อน".

Distance bought here is not a nicety. Every metre saved is battery the sweep has
already spent most of, and time inside a 20-minute window that also pays a
per-minute overtime penalty — a shorter route is directly more eggs delivered.

**The ordering can only be computed after the sweep.** Pad coordinates come from
the registry, which learns them by decoding markers in flight; at the moment the
operator clicks, nobody knows where anything is. So this runs at the start of
the serve phase, from the aircraft's position at that moment.

What it does NOT change: ``payload_id`` stays the serve SLOT, so the physical
release order remains the diagonal AUX 4/1/2/3 the rack is wired for (the order
that keeps the centre of gravity balanced as eggs leave). Re-ordering which PAD
each slot goes to leaves the release sequence untouched.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from itertools import permutations

_R_EARTH_M = 6_371_000.0

# Above this many stops the exact search stops being free (8! = 40 320 routes,
# each scored over 8 legs). The mission carries at most four eggs, so the exact
# branch is what actually runs; the greedy fallback exists so a future
# larger-payload airframe degrades in speed rather than hanging.
_EXACT_MAX = 7


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Local flat-earth metres between two (lat, lon) points.

    Same approximation as ``orchestrator/target_tracker._dist_m``: over a
    100 m field the curvature error is far below the GPS noise this is
    ordering against.
    """
    dn = math.radians(b[0] - a[0]) * _R_EARTH_M
    de = math.radians(b[1] - a[1]) * _R_EARTH_M * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def _valid(p: tuple[float, float] | None) -> bool:
    return (p is not None and not math.isnan(p[0]) and not math.isnan(p[1]))


def route_length_m(order: Sequence[int], coords: Mapping[int, tuple[float, float]],
                   start: tuple[float, float],
                   end: tuple[float, float] | None = None) -> float:
    """Total flown distance for one serve order: start → every pad → end."""
    total, here = 0.0, start
    for pad in order:
        nxt = coords[pad]
        total += distance_m(here, nxt)
        here = nxt
    if end is not None:
        total += distance_m(here, end)
    return total


def order_by_nearest(ids: Iterable[int],
                     coords: Mapping[int, tuple[float, float]],
                     start: tuple[float, float] | None,
                     end: tuple[float, float] | None = None) -> list[int]:
    """The flight's ids re-ordered so the served route is shortest.

    ``coords`` holds only the pads the registry has actually confirmed. Ids
    missing from it keep their queue order and go LAST: their position is
    unknown, so they cannot be placed on a route, and the serve loop will
    record them as not-found anyway — putting them first would only interleave
    dead stops into a route chosen to be short.

    ``end`` is the point the aircraft leaves for after the last delivery (the
    egress transit point). Including it matters: the cheapest way to reach four
    pads and the cheapest way to reach four pads AND get home are different
    routes, and only the second one is the flight actually flown.

    A missing or NaN ``start`` returns the queue order unchanged — with no idea
    where the aircraft is, any reordering would be a guess dressed up as an
    optimisation.
    """
    queue = list(ids)
    if not _valid(start):
        return queue
    assert start is not None  # _valid() ruled out None (and NaN); narrows for mypy
    known = [i for i in queue if _valid(coords.get(i))]
    unknown = [i for i in queue if i not in known]
    if len(known) < 2:
        return known + unknown

    if len(known) <= _EXACT_MAX:
        # Exhaustive: with four stops this is 24 routes, so take the true
        # optimum rather than the greedy chain's first-step-looks-cheapest
        # answer, which can be arbitrarily worse than optimal.
        best = min(permutations(known),
                   key=lambda o: route_length_m(o, coords, start, end))
        return list(best) + unknown

    # Greedy nearest-neighbour chain for payloads this airframe cannot carry.
    remaining, chain, here = list(known), [], start
    while remaining:
        nxt = min(remaining, key=lambda i: distance_m(here, coords[i]))
        chain.append(nxt)
        remaining.remove(nxt)
        here = coords[nxt]
    return chain + unknown
