"""What the orchestrator does when its mission loop dies mid-flight.

The normal answer is an orderly RTH so a crashed process never leaves the
aircraft flying itself. The exception is a pilot takeover, and it was found the
hard way: on the bench (props off, 2026-08-18) safety.py announced "PILOT
TAKEOVER (POSCTL) — orchestrator standing down, no further commands" and 37
seconds later the crash handler sent an RTL anyway, because the loop was still
inside a 60 s takeoff wait and only surfaced afterwards.

With no props that changed nothing. Airborne it is the aircraft being pulled
into AUTO.RTL out from under a pilot who took manual control to stop it doing
something — the one thing a stand-down promises will not happen.
"""

from __future__ import annotations

import asyncio

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.main import emergency_recover
from orchestrator.state import (
    MissionPhase,
    OrchestratorMode,
    OrchestratorState,
    TerminalState,
)


def _state() -> OrchestratorState:
    """The recovery path touches only terminal/phase/audit, so an empty plan
    keeps the test about the safety property rather than about mission setup."""
    home = Coordinate(lat=13.730250, lon=100.787300)
    area = [[13.730723, 100.787840], [13.730703, 100.789776],
            [13.731359, 100.789916], [13.731239, 100.787824]]
    plan = render_live_plan(home, build_search_pattern(area, home, sweep_alt_m=12.0),
                            discovered=[], profile=COMPETITION)
    return OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan,
                             telemetry=CurrentTelemetry())


class RecordingCommander:
    """Counts what the recovery path actually sends to the aircraft."""

    def __init__(self, rth_fails: bool = False, land_fails: bool = False) -> None:
        self.calls: list[str] = []
        self._rth_fails = rth_fails
        self._land_fails = land_fails

    async def rth(self) -> None:
        self.calls.append("rth")
        if self._rth_fails:
            raise RuntimeError("no link")

    async def land(self) -> None:
        self.calls.append("land")
        if self._land_fails:
            raise RuntimeError("no link")


def test_pilot_takeover_sends_nothing_at_all() -> None:
    """The property the whole fix exists for: after takeover the aircraft
    belongs to the pilot, so the recovery path must issue ZERO commands."""
    state = _state()
    state.set_terminal(TerminalState.PILOT_TAKEOVER)
    cmd = RecordingCommander()

    asyncio.run(emergency_recover(state, cmd))

    assert cmd.calls == [], f"commanded the aircraft after takeover: {cmd.calls}"


def test_pilot_takeover_stays_the_terminal_state() -> None:
    """FAILED would overwrite the reason the flight ended — the operator's
    readout, the audit trail and the post-flight verifier all key off it."""
    state = _state()
    state.set_terminal(TerminalState.PILOT_TAKEOVER)

    asyncio.run(emergency_recover(state, RecordingCommander()))

    assert state.terminal is TerminalState.PILOT_TAKEOVER
    assert any("PILOT TAKEOVER" in a and "suppressed" in a
               for a in state.anomalies), state.anomalies


def test_an_ordinary_crash_still_gets_its_rth() -> None:
    """The suppression must be narrow. Every other failure keeps the behaviour
    that stops a dead process leaving the aircraft airborne."""
    state = _state()
    cmd = RecordingCommander()

    asyncio.run(emergency_recover(state, cmd))

    assert cmd.calls == ["rth"]
    assert state.terminal is TerminalState.FAILED
    assert state.phase is MissionPhase.RTH


def test_land_is_the_fallback_when_rth_cannot_be_sent() -> None:
    state = _state()
    cmd = RecordingCommander(rth_fails=True)

    asyncio.run(emergency_recover(state, cmd))

    assert cmd.calls == ["rth", "land"]


def test_both_failing_is_survived_not_raised() -> None:
    """This runs inside an except: block during teardown — raising here would
    lose the original error and skip the cleanup that follows it."""
    state = _state()
    cmd = RecordingCommander(rth_fails=True, land_fails=True)

    asyncio.run(emergency_recover(state, cmd))       # must not raise

    assert cmd.calls == ["rth", "land"]
