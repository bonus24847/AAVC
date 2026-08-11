"""Every POST the Svelte GCS issues must have a backend route.

Guards against the class of bug where the frontend calls `/api/cmd/kill`
(the emergency motor-cut button) but no backend handler exists, so the click
silently 404s. Parses cmd-client.ts and compares against the live route table.
"""

from __future__ import annotations

import re
from pathlib import Path

from dashboard.commands import CommandSession, make_command_router
from dashboard.realtime import RealtimeBroadcaster
from dashboard.tuner import make_tuner_router
from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState

_CMD_CLIENT = Path(__file__).resolve().parent.parent / "dashboard/web/src/lib/cmd-client.ts"
_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


def _backend_post_paths() -> set[str]:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )
    bc = RealtimeBroadcaster(state)
    session = CommandSession()
    routers = [
        make_command_router(state, object(), bc, session),
        make_tuner_router(state, object(), bc, session),
    ]
    paths: set[str] = set()
    for router in routers:
        for route in router.routes:
            if "POST" in getattr(route, "methods", set()):
                paths.add(route.path)  # type: ignore[attr-defined]
    return paths


def _frontend_post_paths() -> set[str]:
    text = _CMD_CLIENT.read_text(encoding="utf-8")
    return set(re.findall(r"post\('(/api/cmd/[^']+)'", text))


def test_every_frontend_post_has_a_backend_route() -> None:
    frontend = _frontend_post_paths()
    assert "/api/cmd/kill" in frontend, "sanity: cmd-client should POST /api/cmd/kill"
    missing = frontend - _backend_post_paths()
    assert not missing, f"frontend POSTs with no backend route (would 404): {sorted(missing)}"
