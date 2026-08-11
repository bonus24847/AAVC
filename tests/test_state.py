"""Tests for orchestrator.state — flight/delivery fields (Task 4)."""

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState

# Use the same area/home/transit as the integration tests
SEARCH_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
HOME = Coordinate(lat=13.730250, lon=100.787300)
TRANSIT = [Coordinate(lat=13.730322, lon=100.787446),
           Coordinate(lat=13.730397, lon=100.788694),
           Coordinate(lat=13.730712, lon=100.788755)]


def _state() -> OrchestratorState:
    """Construct a minimal OrchestratorState for testing."""
    spec = build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0)
    plan = render_live_plan(HOME, spec, discovered=[], profile=COMPETITION,
                            transit_route=TRANSIT)
    telem = CurrentTelemetry()
    telem.lat, telem.lon, telem.relative_alt_m = HOME.lat, HOME.lon, 0.0
    telem.is_armed = False
    return OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem)


def test_state_has_flight_delivery_fields() -> None:
    """Task 4: OrchestratorState carries flight/delivery model fields."""
    s = _state()
    assert s.eggs_aboard == 1
    assert s.max_deliveries == 4
    assert s.delivery_index == 0
    assert s.flight_ids == []
