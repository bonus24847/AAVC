"""orchestrator.vision_worker — the GCS DetectedObjectEvent feed (V1.3 FLIGHT
⊃ DELIVERY).

CLAUDE.md §5 defines ``is_designated_match`` as "decoded id == the assigned
id". A FLIGHT can now carry ``eggs_aboard`` eggs, so "the assigned id" has to
mean ANY pad this flight will serve (``state.flight_ids``), not only whichever
one delivery is CURRENTLY in progress (``state.assigned_marker_id``) — else
the live map paints pads 2..N as non-designated while the aircraft is
actually landing on them (I3, review 2026-07-24).
"""

from __future__ import annotations

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState
from orchestrator.vision_worker import TargetFix, VisionWorker

_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


def _state() -> OrchestratorState:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    return OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )


def _fix(marker_id: int | None) -> TargetFix:
    return TargetFix(lat=13.7303, lon=100.7880, pixel_xy=(320, 240),
                      confidence=0.9, radius_px=3.0, camera="nadir",
                      ground_dist_m=1.0, slant_range_m=12.0, t_monotonic=0.0,
                      marker_id=marker_id)


def test_any_pad_in_the_flights_chunk_is_designated() -> None:
    """A multi-egg flight sets state.flight_ids = [3, 1, 4, 6]; a decode of
    ANY of them is designated — not only whichever one is currently being
    served (state.assigned_marker_id)."""
    state = _state()
    state.flight_ids = [3, 1, 4, 6]
    state.assigned_marker_id = 6      # the delivery CURRENTLY being served
    worker = VisionWorker(state)

    events = worker._detected_object_events([_fix(3), _fix(1), _fix(6), _fix(5)])

    by_id = {e.label: e.is_designated_match for e in events}
    assert by_id["aruco pad 3"] is True
    assert by_id["aruco pad 1"] is True
    assert by_id["aruco pad 6"] is True
    assert by_id["aruco pad 5"] is False       # not in this flight's chunk


def test_falls_back_to_assigned_marker_id_when_flight_ids_is_empty() -> None:
    """Pre-flight / eggs_aboard=1 paths that never populated state.flight_ids
    must keep working off the single assigned id."""
    state = _state()
    assert state.flight_ids == []
    state.assigned_marker_id = 3
    worker = VisionWorker(state)

    events = worker._detected_object_events([_fix(3), _fix(5)])

    by_id = {e.label: e.is_designated_match for e in events}
    assert by_id["aruco pad 3"] is True
    assert by_id["aruco pad 5"] is False


def test_undecoded_hit_is_never_designated() -> None:
    state = _state()
    state.flight_ids = [3]
    worker = VisionWorker(state)

    events = worker._detected_object_events([_fix(None)])

    assert events[0].is_designated_match is False


def test_no_assignment_at_all_designates_nothing() -> None:
    """Before any GO (flight_ids empty AND assigned_marker_id None) nothing
    is designated — no false positive from a stale/absent id."""
    state = _state()
    worker = VisionWorker(state)

    events = worker._detected_object_events([_fix(3)])

    assert events[0].is_designated_match is False
