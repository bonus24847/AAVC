"""Split the committee's assigned-id queue into flights.

A FLIGHT is one arm→disarm cycle carrying up to ``eggs_aboard`` eggs; a
DELIVERY is one pad served within a flight. ``eggs_aboard=1`` yields one
delivery per flight — the original per-sortie behaviour. Pure: no I/O.
"""

from __future__ import annotations

import math
from collections import Counter


def chunk_flights(ids: list[int], eggs_aboard: int) -> list[list[int]]:
    """Consecutive chunks of ``ids`` of at most ``eggs_aboard`` (order kept).
    ``eggs_aboard < 1`` is treated as 1 (one delivery per flight)."""
    n = max(1, int(eggs_aboard))
    return [ids[i : i + n] for i in range(0, len(ids), n)]


def max_flights_for(n_ids: int, eggs_aboard: int) -> int:
    """Number of flights ``n_ids`` deliveries need at ``eggs_aboard`` per flight."""
    n = max(1, int(eggs_aboard))
    return math.ceil(n_ids / n) if n_ids > 0 else 0


def budgeted_flights_for(n_ids: int, eggs_aboard: int) -> int:
    """The flight-count CEILING the orchestrator seeds ``state.max_sorties``
    with (I5, review 2026-07-24): ``max_flights_for`` — what the deliveries
    need in the BEST case, zero failures — floored at 2 whenever there is
    anything to deliver.

    PRECISELY what that floor buys (M3, review 2026-07-24 — the previous
    wording overstated this): ``max(2, n)`` only RAISES the count when
    ``max_flights_for(...)`` is 0 or 1 — i.e. only when every assigned id
    already fits in a single best-case flight (``eggs_aboard >= n_ids``, the
    shipping ``eggs_aboard=4`` default, or any 1-delivery mission). In that
    case, and ONLY that case, a flight that comes home with undelivered eggs
    (a pad never found, a per-delivery budget abort, a release-channel
    shortage) gets exactly ONE extra recovery flight, no operator action
    needed beyond the ordinary per-flight GO.

    For ``eggs_aboard`` small enough that ``max_flights_for(...) >= 2`` on its
    own (e.g. ``eggs_aboard=1/2/3`` at 4 deliveries), ``max(2, n) == n`` — this
    adds NO spare flight beyond the positional ones the queue already needs.
    A flight anywhere in THAT schedule that comes home partial has nowhere to
    go; each flight still budgets only its own positional chunk. This is a
    known, accepted scope boundary (the review's own example formula), not
    something this function silently fixes for every config.

    ``n_ids == 0`` stays 0: nothing to ever deliver, so nothing to recover
    either — the gate already ends the mission on an empty chunk before any
    flight starts (mission.py's own ``max(1, ...)`` floor covers the "must
    launch a takeoff-less flight 1 to observe that" bookkeeping case).
    """
    n = max_flights_for(n_ids, eggs_aboard)
    return n if n == 0 else max(2, n)


def remaining_owed(queue: list[int], delivered: list[int]) -> list[int]:
    """``queue`` minus ``delivered``, as a MULTISET subtraction (order kept).

    The ids a RECOVERY flight — one past the queue's own positional chunks —
    still owes. Delivered ids are tracked explicitly
    (``state.delivered_marker_ids``) rather than inferred from
    ``dropped_stops``/``stop_index``: a manual per-flight GO override can
    serve an id that was never in the queue at all, and ``stop_index`` is a
    flat ordinal across the WHOLE mission, not a queue position — neither
    maps back to "which queue id got delivered" unambiguously. A duplicate id
    in the queue (e.g. headless ``--assigned-ids "3,3,3,3"``) is only
    cancelled by that many actual deliveries, not collapsed by a set.
    """
    left = Counter(delivered)
    owed: list[int] = []
    for marker_id in queue:
        if left[marker_id] > 0:
            left[marker_id] -= 1
        else:
            owed.append(marker_id)
    return owed
