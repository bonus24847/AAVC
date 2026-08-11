"""Headless per-FLIGHT gate ↔ the mission-id queue (orchestrator.main).

Both the dashboard GO and headless runs consume ONE queue —
``state.assigned_id_queue`` (seeded from --assigned-ids / config headlessly,
or from POST /api/cmd/mission_ids on the GCS). The gate owns the CHUNKING:
it hands the mission loop flight i's list of ≤``state.eggs_aboard`` ids. These
lock the headless side: chunks come in order, the mission ends at queue
exhaustion, and the time-policy refusal path is unchanged.
"""

from __future__ import annotations

import asyncio
from typing import cast

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.main import _sortie_gate_factory
from orchestrator.mission import FlightGate
from orchestrator.state import OrchestratorMode, OrchestratorState
from orchestrator.target_tracker import TargetTracker
from orchestrator.time_policy import TimePolicy

_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


class _Tracker:
    def confirmed_by_marker(self, marker_id: int):  # registry-known? → no
        return None


def _state(queue: list[int], eggs_aboard: int = 1) -> OrchestratorState:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )
    state.assigned_id_queue = list(queue)
    state.eggs_aboard = eggs_aboard
    return state


def _gate_for(state: OrchestratorState) -> FlightGate:
    # skip_preflight=True exercises the queue-pop without the readiness board
    # (preflight itself is covered by test_preflight / the dashboard tests).
    return _sortie_gate_factory(
        state, dash=None, home=_HOME, geofence=[tuple(v) for v in _AREA],
        cfg={}, profile=COMPETITION, policy=TimePolicy(),
        tracker=cast(TargetTracker, _Tracker()),
        skip_preflight=True,
    )


def _go(gate: FlightGate, flight: int) -> list[int] | None:
    async def _inner() -> list[int] | None:
        return await gate(flight)
    return asyncio.run(_inner())


def _mark_delivered(state: OrchestratorState, ids: list[int]) -> None:
    """These gate-only tests never run the real mission loop — the thing
    that normally appends to this list is orchestrator/mission.py::_serve,
    the moment a delivery's touchdown+release is confirmed — so simulate
    "the flight just flown actually delivered these ids" directly."""
    state.delivered_marker_ids.extend(ids)


def test_headless_gate_pops_queue_in_order() -> None:
    state = _state([3, 1, 4, 6])
    gate = _gate_for(state)
    ids = [_go(gate, flight) for flight in (1, 2, 3, 4)]
    assert ids == [[3], [1], [4], [6]]       # one egg aboard → one id per flight


def test_headless_gate_chunks_the_queue_by_eggs_aboard() -> None:
    """Four eggs aboard → the whole queue is ONE flight's chunk. Once every
    id it carried is actually DELIVERED, there is no second flight to fly
    (I5, review 2026-07-24: a flight that comes home with UNdelivered eggs
    gets a recovery flight instead — see test_recovery_flight_* below)."""
    state = _state([3, 1, 4, 6], eggs_aboard=4)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3, 1, 4, 6]
    _mark_delivered(state, [3, 1, 4, 6])
    assert _go(gate, 2) is None              # every delivery already assigned


def test_headless_gate_chunks_a_partial_last_flight() -> None:
    state = _state([3, 1, 4, 6], eggs_aboard=3)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3, 1, 4]
    _mark_delivered(state, [3, 1, 4])
    assert _go(gate, 2) == [6]               # short final flight
    _mark_delivered(state, [6])
    assert _go(gate, 3) is None


def test_headless_gate_ends_mission_at_queue_exhaustion() -> None:
    state = _state([3, 1])
    gate = _gate_for(state)
    assert _go(gate, 1) == [3]
    _mark_delivered(state, [3])
    assert _go(gate, 2) == [1]
    _mark_delivered(state, [1])
    assert _go(gate, 3) is None              # queue exhausted → mission over


# ── I5 (review 2026-07-24): recovery flight for undelivered eggs ───────────
#
# Before this fix, a flight index PAST the queue's positional chunks always
# ended the mission (`_chunk_for` returned None) — so a flight that came
# home with undelivered eggs (a pad never found, a per-delivery budget
# abort, a release-channel shortage) had NO way to get them flown, and at
# the shipping eggs_aboard=4 default (ONE positional chunk) that was every
# partial flight. Past the positional chunks, the gate now chunks the queue
# MINUS what state.delivered_marker_ids says was actually delivered.


def test_recovery_flight_carries_exactly_the_undelivered_ids() -> None:
    """2 of 4 delivered → the next flight carries exactly the other 2,
    automatically — no operator re-queue needed."""
    state = _state([3, 1, 4, 6], eggs_aboard=4)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3, 1, 4, 6]
    _mark_delivered(state, [3, 1])            # 4 and 6 came home undelivered
    assert _go(gate, 2) == [4, 6]


def test_recovery_flight_respects_eggs_aboard_chunking() -> None:
    """A recovery flight is chunked the SAME way a positional one is — it
    doesn't dump everything owed into one oversized flight."""
    state = _state([3, 1, 4, 6], eggs_aboard=2)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3, 1]
    assert _go(gate, 2) == [4, 6]
    _mark_delivered(state, [3, 4])            # 1 and 6 came home undelivered
    assert _go(gate, 3) == [1, 6]
    _mark_delivered(state, [1, 6])
    assert _go(gate, 4) is None


def test_recovery_flight_ignores_a_delivery_outside_the_queue() -> None:
    """A manual per-flight GO override can serve an id that was never queued
    at all — that must not be mistaken for progress against a DIFFERENT
    still-queued id."""
    state = _state([3, 1], eggs_aboard=2)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3, 1]
    _mark_delivered(state, [5])               # served OUTSIDE the queue
    assert _go(gate, 2) == [3, 1]             # both still owed


def test_eggs_aboard_1_standard_four_delivery_path_is_unchanged() -> None:
    """I5(c): the standard eggs_aboard=1 / 4-delivery config already needs 4
    flights on its own (one id per flight) — budgeted_flights_for's recovery
    floor (>= 2) changes nothing here; matches pre-fix behaviour exactly."""
    state = _state([3, 1, 4, 6], eggs_aboard=1)
    gate = _gate_for(state)
    assert [_go(gate, f) for f in (1, 2, 3, 4)] == [[3], [1], [4], [6]]


def test_eggs_aboard_1_single_delivery_also_gets_a_recovery_flight() -> None:
    """A single-delivery eggs_aboard=1 mission is the SAME gap I5 fixes, just
    at a smaller scale (max_flights_for(1, 1) == 1, no spare before this fix)
    — the recovery floor applies uniformly, not only at the eggs_aboard=4
    shipping default."""
    state = _state([3], eggs_aboard=1)
    gate = _gate_for(state)
    assert _go(gate, 1) == [3]
    # Flight 1 came home without delivering pad 3 (nothing marked delivered).
    assert _go(gate, 2) == [3]
    _mark_delivered(state, [3])
    assert _go(gate, 3) is None


def test_headless_gate_reads_queue_live() -> None:
    """A queue set/extended mid-mission (e.g. from the GCS) applies at the
    next hold — the gate re-reads state each call."""
    state = _state([3])
    gate = _gate_for(state)
    assert _go(gate, 1) == [3]
    state.assigned_id_queue = [3, 5]         # extended between flights
    assert _go(gate, 2) == [5]


# ── interactive (dashboard) hold: GO resolves the same chunk ──


class _Broadcaster:
    def push_preflight(self, report: dict) -> None:  # the gate's can_push probe
        pass


class _Dash:
    broadcaster = _Broadcaster()


def _gate_with_dash(state: OrchestratorState) -> FlightGate:
    return _sortie_gate_factory(
        state, dash=_Dash(), home=_HOME, geofence=[tuple(v) for v in _AREA],
        cfg={}, profile=COMPETITION, policy=TimePolicy(),
        tracker=cast(TargetTracker, _Tracker()),
    )


def _go_with_click(gate: FlightGate, flight: int, state: OrchestratorState,
                   *, manual: int | None) -> list[int] | None:
    """Run the interactive hold and fire /api/cmd/preflight/go's state writes
    (assigned_marker_id then the resume event) from a concurrent task."""
    async def _inner() -> list[int] | None:
        async def _click() -> None:
            await asyncio.sleep(0.02)
            state.assigned_marker_id = manual
            state.preflight_resume_event.set()
        clicker = asyncio.create_task(_click())
        try:
            return await gate(flight)
        finally:
            await clicker
    return asyncio.run(_inner())


def test_interactive_go_serves_the_flights_whole_chunk() -> None:
    """4 eggs aboard: GO flies the queue's chunk, not the endpoint's single
    resolved id (which is only the eggs_aboard=1 manual override)."""
    state = _state([3, 1, 4, 6], eggs_aboard=4)
    gate = _gate_with_dash(state)
    assert _go_with_click(gate, 1, state, manual=3) == [3, 1, 4, 6]
    assert state.flight_ids == [3, 1, 4, 6]


def test_interactive_go_honours_the_manual_override_at_one_egg() -> None:
    """1 egg aboard: a flight IS one delivery, so the operator's manual pick
    overrides the queue slot for THIS flight."""
    state = _state([3, 1], eggs_aboard=1)
    gate = _gate_with_dash(state)
    assert _go_with_click(gate, 1, state, manual=5) == [5]
    assert state.flight_ids == [5]


def test_interactive_go_without_a_queue_flies_the_manual_pick() -> None:
    """No queue at all (the endpoint accepted a manual id): that id is the
    flight — returning None would end the mission on a green GO."""
    state = _state([], eggs_aboard=4)
    gate = _gate_with_dash(state)
    assert _go_with_click(gate, 1, state, manual=2) == [2]


def test_interactive_go_flies_the_recovery_chunk_automatically() -> None:
    """I5: the operator does not need to re-post anything to /mission_ids —
    GO for the recovery flight resolves to the undelivered remainder on its
    own, the same one-click flow as any other flight.

    Both holds run inside ONE asyncio.run() (unlike _go_with_click, which is
    single-call-only): state.preflight_resume_event is an asyncio.Event bound
    to whichever loop first uses it, and two flights on the same state need
    to share one — _gate itself clears it at the top of each hold, so no
    manual reset is needed between the two clicks."""
    state = _state([3, 1, 4, 6], eggs_aboard=4)
    gate = _gate_with_dash(state)

    async def _click_and_go(flight: int) -> list[int] | None:
        async def _click() -> None:
            await asyncio.sleep(0.02)
            state.assigned_marker_id = None
            state.preflight_resume_event.set()
        clicker = asyncio.create_task(_click())
        try:
            return await gate(flight)
        finally:
            await clicker

    async def _run() -> tuple[list[int] | None, list[int] | None]:
        first = await _click_and_go(1)
        _mark_delivered(state, [3, 1])          # flight 1 came home with 2 of 4
        second = await _click_and_go(2)
        return first, second

    first, second = asyncio.run(_run())
    assert first == [3, 1, 4, 6]
    assert second == [4, 6]
    assert state.flight_ids == [4, 6]


# ── I1 (review 2026-07-24): a PERFECT flight must not hold for a pointless GO ──


def test_go_after_a_fully_served_queue_does_not_launch_a_redundant_flight() -> None:
    """Before this fix, `_chunk_for(2)` already returned None once flight 1
    delivered everything — but the interactive hold ran anyway (the GCS kept
    showing a live GO board), and dashboard/commands.py:524-528 resolves the
    GO's id POSITIONALLY (``assigned_id_queue[sortie_index]``) with NO notion
    of ``state.delivered_marker_ids``. So the documented one-click GO handed
    back id 1 — already delivered — which main.py's manual-override branch
    (``eggs_aboard == 1 or not chunk``) then turned into a bogus one-egg
    flight 2. The click below supplies EXACTLY what the real endpoint would
    compute (no manual pick — the queue fills it in), not the unconditional
    ``manual=None`` the OTHER interactive tests use, which the endpoint can
    only produce when the queue can't cover this slot at all."""
    state = _state([3, 1, 4, 6], eggs_aboard=4)
    gate = _gate_with_dash(state)

    async def _click_and_go(flight: int) -> list[int] | None:
        async def _click() -> None:
            await asyncio.sleep(0.02)
            # dashboard/commands.py:524-528, no manual pick supplied: resolve
            # positionally from the queue at the CURRENT sortie_index — the
            # endpoint has no notion of state.delivered_marker_ids at all.
            if len(state.assigned_id_queue) > state.sortie_index:
                state.assigned_marker_id = (
                    state.assigned_id_queue[state.sortie_index])
            state.preflight_resume_event.set()
        clicker = asyncio.create_task(_click())
        try:
            return await gate(flight)
        finally:
            await clicker

    async def _run() -> tuple[list[int] | None, list[int] | None]:
        first = await _click_and_go(1)
        # What mission.py sets after flight 1 actually flies (state.sortie_index
        # = flight) — these gate-only tests bypass the real mission loop, so it
        # is set here exactly as _mark_delivered stands in for _serve's ledger.
        state.sortie_index = 1
        _mark_delivered(state, [3, 1, 4, 6])      # flight 1 delivered everything
        second = await _click_and_go(2)
        return first, second

    first, second = asyncio.run(_run())
    assert first == [3, 1, 4, 6]
    assert second is None, (
        "a fully-served queue must not launch a redundant flight even when "
        "the GO's positionally-resolved id (already delivered) is supplied "
        "exactly as dashboard/commands.py computes it")
