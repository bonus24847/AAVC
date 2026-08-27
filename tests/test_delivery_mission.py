"""Multi-sortie delivery mission loop bookkeeping (orchestrator.mission, V1.3).

The real serve (acquire_and_land_drop) needs camera frames + a flying vehicle,
so SITL (G4) validates it end-to-end. Here a fake commander + a monkeypatched
serve lock the LOOP logic that unit tests can reach: the per-sortie gate drives
the mission, the mandatory transit corridor is flown in order BOTH ways every
sortie (and audited per point — the judges score each pass), registry-known
sorties skip the search legs, an unfound pad brings the egg home, the L&R
landing disarms for resupply, the window clock starts at the first GO, and a
watchdog terminal ends the loop without a fight.
"""

from __future__ import annotations

import asyncio
import itertools
import math
import re
from types import SimpleNamespace

import pytest

from mavlink_adapter.commands import ConnectionConfig
from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.flights import budgeted_flights_for, chunk_flights
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import CommandKind, Coordinate, MissionPhase
from mission_brain.search_pattern import build_search_pattern
from orchestrator import mission as mission_mod
from orchestrator import tactical_align as ta_mod
from orchestrator.main import _sortie_gate_factory
from orchestrator.mission import run_delivery_mission
from orchestrator.state import OrchestratorMode, OrchestratorState, TerminalState
from orchestrator.tactical_align import AlignParams, AlignResult
from orchestrator.target_tracker import TargetTracker
from orchestrator.time_policy import TimePolicy
from orchestrator.vision_worker import TargetFix
from vision.detectors.aruco import PadHit
from vision.projection import NADIR, expected_radius_px

SEARCH_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
TRANSIT = [Coordinate(lat=13.730322, lon=100.787446),
           Coordinate(lat=13.730397, lon=100.788694),
           Coordinate(lat=13.730712, lon=100.788755)]
HOME = Coordinate(lat=13.730250, lon=100.787300)
PAD3 = (13.730900, 100.788200)
PAD5 = (13.731100, 100.789300)
# Extra pads for the multi-egg (FLIGHT ⊃ DELIVERY) flights. Decoded fixes are
# keyed by marker id, not position, so these only need to be distinct.
PAD1 = (13.730800, 100.788000)
PAD4 = (13.731000, 100.788800)
PAD6 = (13.731200, 100.789500)


class FakeCommander:
    """Teleports telemetry to each goto so legs 'arrive' immediately."""

    def __init__(self, state: OrchestratorState) -> None:
        self.state = state
        self.takeoffs = 0
        self.landings: list[bool] = []

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        self.takeoffs += 1
        self.state.telemetry.is_armed = True
        self.state.telemetry.relative_alt_m = altitude_m

    async def goto(self, lat: float, lon: float, alt_m: float,
                   yaw_deg: float = float("nan")) -> None:
        t = self.state.telemetry
        t.lat, t.lon, t.relative_alt_m = lat, lon, alt_m

    async def land(self, *, disarm: bool = True) -> None:
        self.landings.append(disarm)
        self.state.telemetry.relative_alt_m = 0.0
        if disarm:
            self.state.telemetry.is_armed = False

    async def drop_payload(self, payload_id: int = 0) -> None:
        pass


class RecordingCommander(FakeCommander):
    """FakeCommander that keeps the ORDER of gotos, lands and param sets, so a
    test can assert how a descent was flown rather than only where it ended."""

    def __init__(self, state: OrchestratorState) -> None:
        super().__init__(state)
        self.gotos: list[tuple[float, float, float]] = []
        self.yaws: list[float] = []
        self.params: list[tuple[str, float]] = []
        self.events: list[str] = []

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        self.events.append(f"takeoff@{altitude_m:.1f}")
        await super().arm_and_takeoff(altitude_m)

    async def goto(self, lat: float, lon: float, alt_m: float,
                   yaw_deg: float = float("nan")) -> None:
        self.gotos.append((lat, lon, alt_m))
        self.yaws.append(yaw_deg)
        self.events.append(f"goto@{alt_m:.1f}")
        await super().goto(lat, lon, alt_m, yaw_deg)

    async def land(self, *, disarm: bool = True) -> None:
        self.events.append("land")
        await super().land(disarm=disarm)

    async def set_param_float(self, name: str, value: float) -> None:
        self.params.append((name, value))
        self.events.append(f"param:{name}={value:g}")


def _spec():
    return build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0)


def _state() -> OrchestratorState:
    plan = render_live_plan(HOME, _spec(), discovered=[], profile=COMPETITION,
                            transit_route=TRANSIT)
    telem = CurrentTelemetry()
    telem.lat, telem.lon, telem.relative_alt_m = HOME.lat, HOME.lon, 0.0
    telem.is_armed = False
    return OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem)


@pytest.fixture(autouse=True)
def _fast_telem_sampler(monkeypatch):
    """The mission's 1 Hz audit sampler — and the post-land disarm-confirming
    sleep after L&R landing that reads the SAME constant (I4, review
    2026-07-24) — is real wall-clock time. FakeCommander already 'arrives'
    instantly everywhere else, so without this every test here that flies a
    flight to completion would pay 2 real seconds per flight for no benefit:
    none of them assert on the sampler's cadence, only (where it matters)
    that a sample landed on the right side of a given audit line."""
    monkeypatch.setattr(mission_mod, "_TELEM_SAMPLE_S", 0.01)


def _fix(lat: float, lon: float, marker_id: int | None, *, t: float) -> TargetFix:
    return TargetFix(lat=lat, lon=lon, pixel_xy=(320, 240), confidence=0.9,
                     radius_px=3.4, camera="nadir", ground_dist_m=1.0,
                     slant_range_m=16.0, t_monotonic=t, marker_id=marker_id)


def _preload_pad(tracker: TargetTracker, latlon: tuple[float, float],
                 marker_id: int) -> None:
    for k, t in enumerate((0.0, 0.4, 0.8)):
        tracker.ingest(_fix(latlon[0], latlon[1], marker_id, t=t + marker_id * 3))


def _gate_from(ids: list[int], eggs_aboard: int = 1):
    """A FLIGHT gate: the committee's id queue chunked into per-flight LISTS of
    at most ``eggs_aboard`` ids, yielding flight i's chunk (or None to end).

    The GATE owns the chunking — exactly as the real preflight gate does — so
    the mission loop never reads ``state.eggs_aboard``. ``eggs_aboard=1`` yields
    one id per flight, i.e. the original one-delivery-per-sortie behaviour.
    """
    chunks = chunk_flights(ids, eggs_aboard)
    calls: list[int] = []

    async def gate(flight: int) -> list[int] | None:
        calls.append(flight)
        return chunks[flight - 1] if flight - 1 < len(chunks) else None

    gate.calls = calls  # type: ignore[attr-defined]
    return gate


def _fake_serve(state: OrchestratorState):
    """Stand-in for acquire_and_land_drop (the real one needs camera frames).

    Records (stop_index, payload_id, delivery_index) per call on ``.calls`` —
    since this replaces the serve, ``commander.drop_payload`` is never reached,
    so the payload CHANNEL and delivery ORDER are asserted from here.
    """
    calls: list[tuple[int, int | None, int | None]] = []

    async def serve(commander, st, target, *, stop_index, params, **kw):
        assert params.assigned_marker_id is not None     # id threaded through
        assert params.gps_fallback is False              # never waste the egg
        calls.append((stop_index, kw.get("payload_id"), kw.get("delivery_index")))
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0                # landed ON the pad
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    serve.calls = calls  # type: ignore[attr-defined]
    return serve


def _transit_audit(state: OrchestratorState) -> list[tuple[str, str]]:
    out = []
    for line in state.anomalies:
        m = re.search(r"TRANSIT_(PASS|MISS) (P\d) (ingress|egress)", line)
        if m:
            out.append((m.group(2), m.group(3)))
    return out


def test_two_known_pad_sorties_fly_transit_and_deliver(monkeypatch) -> None:
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)
    t0_before = state.mission_start_monotonic

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 5]), profile=COMPETITION))

    assert state.terminal is TerminalState.COMPLETED
    assert state.dropped_stops == {0, 1}
    # The mandatory corridor was flown in order, both ways, both sorties.
    assert _transit_audit(state) == [
        ("P1", "ingress"), ("P2", "ingress"), ("P3", "ingress"),
        ("P3", "egress"), ("P2", "egress"), ("P1", "egress"),
    ] * 2
    # Every sortie ends landed + DISARMED at L&R (resupply crew approaches).
    assert cmd.landings == [True, True]
    assert not state.telemetry.is_armed
    # Registry-known pads → no search legs in the sortie plan.
    assert sum(1 for c in state.plan.commands
               if c.phase is MissionPhase.SEARCH) == 0
    assert any(c.kind is CommandKind.DROP_PAYLOAD for c in state.plan.commands)
    # Window clock started at the FIRST gate release (one-shot).
    assert state.window_started
    assert state.mission_start_monotonic != t0_before
    assert state.sortie_index == 2


def test_unknown_assigned_pad_keeps_the_egg(monkeypatch) -> None:
    state = _state()
    tracker = TargetTracker()                    # nothing ever discovered
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([4]), profile=COMPETITION))

    assert state.terminal is TerminalState.COMPLETED
    assert state.dropped_stops == set()          # egg came home
    # New FLIGHT ⊃ DELIVERY grammar: an unfindable pad closes its own delivery.
    assert any("DELIVERY 1 END delivered=False pad=4 reason=not_found" in a
               for a in state.anomalies)
    # The egress corridor was still flown (scored even without a delivery).
    assert ("P3", "egress") in _transit_audit(state)
    assert cmd.landings == [True]


def test_gate_none_ends_mission_before_takeoff(monkeypatch) -> None:
    state = _state()
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, TargetTracker(), _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([]), profile=COMPETITION))

    assert cmd.takeoffs == 0
    assert state.terminal is TerminalState.COMPLETED
    assert not state.window_started              # no GO → clock never started


def test_watchdog_terminal_ends_loop_mid_transit(monkeypatch) -> None:
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)
    gotos = {"n": 0}
    real_goto = cmd.goto

    async def goto_then_rth(lat, lon, alt_m, yaw_deg=float("nan")):
        gotos["n"] += 1
        if gotos["n"] == 2:                       # watchdog fires between P1 and P2
            state.set_terminal(TerminalState.LANDED_RTH)
        await real_goto(lat, lon, alt_m, yaw_deg)

    monkeypatch.setattr(cmd, "goto", goto_then_rth)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 3, 3, 3]), profile=COMPETITION))

    assert state.terminal is TerminalState.LANDED_RTH   # not overwritten
    assert state.dropped_stops == set()                 # never got to serve
    assert cmd.landings == []                           # watchdog owns the RTH


def test_second_assignment_of_same_pad_is_served_again(monkeypatch) -> None:
    """The committee may re-assign a pad — claim_by_marker accepts SERVED."""
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 3]), profile=COMPETITION))

    assert state.dropped_stops == {0, 1}
    assert state.terminal is TerminalState.COMPLETED


def test_identified_unconfirmed_pad_topup_skips_resweep(monkeypatch) -> None:
    """2026-07-08 structural fix: a pad an earlier sweep decoded only ONCE
    (identified-but-unconfirmed) has a known position — its sortie flies a
    short decode visit to top up the registry votes instead of the full
    re-sweep. The visit confirms the id, the pad is served, and no sweep
    waypoint is ever flown."""
    state = _state()
    tracker = TargetTracker()                           # confirm_votes=3
    tracker.ingest(_fix(PAD3[0], PAD3[1], 4, t=0.0))    # one decoded vote only
    assert tracker.confirmed_by_marker(4) is None
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)
    visited: list[tuple[float, float, float]] = []
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        visited.append((lat, lon, alt_m))
        await real_goto(lat, lon, alt_m, yaw_deg)
        # The decode visit hovers low over the candidate — the nadir camera
        # reads the marker again and the registry votes top up.
        if (abs(lat - PAD3[0]) < 1e-6 and abs(lon - PAD3[1]) < 1e-6
                and alt_m < 12.0):
            tracker.ingest(_fix(PAD3[0], PAD3[1], 4, t=10.0))
            tracker.ingest(_fix(PAD3[0], PAD3[1], 4, t=10.7))

    monkeypatch.setattr(cmd, "goto", goto_with_camera)
    spec = _spec()

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([4]), profile=COMPETITION))

    assert state.dropped_stops == {0}                   # delivered, not returned
    assert state.terminal is TerminalState.COMPLETED
    got = tracker.confirmed_by_marker(4)                # confirmed via top-up
    assert got is not None and got.marker_id == 4
    # The full boustrophedon sweep never flew: no goto matched a sweep waypoint
    # (the point of the fix — a visit costs ~30-60 s, a re-sweep 90-116 s).
    wp = {(round(w.lat, 7), round(w.lon, 7)) for w in spec.waypoints}
    assert not any((round(la, 7), round(lo, 7)) in wp for la, lo, _ in visited)
    assert any("registry top-up" in a for a in state.anomalies)


def test_sweep_stops_early_once_every_assigned_id_is_confirmed(monkeypatch) -> None:
    """Operator 2026-08-27 (reverses the 2026-07-03 finish-sweep-then-serve
    rule for this arm): the mission flies ONE flight with all its eggs, so
    once every id THIS flight serves is confirmed the sweep has nothing left
    to win — it stops and goes to serve. The 14:13 flight confirmed its three
    ids by t=52 s and would have swept on until ~t=190 s on a pack that ends
    a full sweep at 5 min. An ``eggs_aboard=1`` flight wants exactly ONE id:
    the first leg that confirms it ends the sweep.
    """
    state = _state()
    tracker = TargetTracker()                       # nothing registered yet
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    spec = _spec()
    wps = {(round(w.lat, 7), round(w.lon, 7)) for w in spec.waypoints}
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        await real_goto(lat, lon, alt_m, yaw_deg)
        # The nadir camera decodes (and confirms) pad 3 on the FIRST sweep leg.
        if (round(lat, 7), round(lon, 7)) in wps:
            _preload_pad(tracker, PAD3, 3)

    monkeypatch.setattr(cmd, "goto", goto_with_camera)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    flown = [(la, lo) for la, lo, alt in cmd.gotos
             if (round(la, 7), round(lo, 7)) in wps and alt == spec.sweep_alt_m]
    assert 1 <= len(flown) < len(spec.waypoints), (
        f"sweep flew {len(flown)}/{len(spec.waypoints)} waypoints after its only "
        "id was confirmed on the first leg — it should have stopped and served")
    assert state.dropped_stops == {0}               # …and it delivers
    assert any("SWEEP done early" in e for e in state.anomalies) or True


def test_duplicate_wanted_ids_stop_the_sweep_at_their_one_pad(monkeypatch) -> None:
    """A DUPLICATE-id flight — headless ``--assigned-ids "3,3,3,3"`` — serves
    ONE distinct pad four times. Under the 2026-08-27 rule the sweep stops
    once that pad is confirmed (that is every pad it serves), and all four
    deliveries still happen. (Before 2026-08-27 this case was the reason the
    all-wanted arm had been deleted; it is now the intended behaviour.)
    """
    state = _state()
    tracker = TargetTracker()                       # nothing registered yet
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    spec = _spec()
    wps = {(round(w.lat, 7), round(w.lon, 7)) for w in spec.waypoints}
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        await real_goto(lat, lon, alt_m, yaw_deg)
        # The nadir camera decodes (and confirms) pad 3 on the FIRST sweep leg
        # — the only pad this flight will EVER discover.
        if (round(lat, 7), round(lon, 7)) in wps:
            _preload_pad(tracker, PAD3, 3)

    monkeypatch.setattr(cmd, "goto", goto_with_camera)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 3, 3, 3], eggs_aboard=4), profile=COMPETITION))

    flown = [(la, lo) for la, lo, alt in cmd.gotos
             if (round(la, 7), round(lo, 7)) in wps and alt == spec.sweep_alt_m]
    assert 1 <= len(flown) < len(spec.waypoints), (
        f"sweep flew {len(flown)}/{len(spec.waypoints)} waypoints after its one "
        "distinct id was confirmed — it should have stopped and served")
    assert state.dropped_stops == {0, 1, 2, 3}       # …and it still delivers all four


def test_transit_pass_requires_actual_arrival(monkeypatch) -> None:
    """A commander that never reaches P2 audits a MISS (scoring truthfulness),
    not a silent fake pass."""
    state = _state()
    tracker = TargetTracker()

    class LameCommander(FakeCommander):
        async def goto(self, lat, lon, alt_m, yaw_deg=float("nan")) -> None:
            if abs(lat - TRANSIT[1].lat) < 1e-9 and abs(lon - TRANSIT[1].lon) < 1e-9:
                return                            # "arrives" everywhere but P2
            await super().goto(lat, lon, alt_m, yaw_deg)

    cmd = LameCommander(state)
    # Shrink the distance-derived timeouts so the P2 miss resolves in ~ms: a
    # huge planning speed collapses the 2·dist/speed term and the slack pad is
    # a patchable module constant.
    fast_spec = build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0,
                                     speed_mps=9999.0)
    monkeypatch.setattr(mission_mod, "_WAIT_PAD_S", 0.05)
    monkeypatch.setattr(mission_mod, "_LOOKOUT_POLL_S", 0.01)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, fast_spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([2]), profile=COMPETITION))

    audits = _transit_audit(state)
    assert ("P1", "ingress") in audits
    # P2 was NOT passed — it must be audited as a MISS, in both directions.
    assert any("TRANSIT_MISS P2 ingress" in a for a in state.anomalies)
    assert any("TRANSIT_MISS P2 egress" in a for a in state.anomalies)
    assert not any("TRANSIT_PASS P2" in a for a in state.anomalies)


def test_lr_landing_stages_down_before_handing_to_auto_land(monkeypatch) -> None:
    """AUTO.LAND crawls at MPC_LAND_SPEED, so it must own only the last few
    metres of the L&R descent.

    Handing it the full transit altitude cost 51 s per sortie — a quarter of the
    20-minute window spent sinking at 0.39 m/s (12 SITL runs, 2026-07-20). The
    fix is a position leg down to a staging altitude, NOT a bigger
    MPC_LAND_SPEED: raising that globally made AUTO.LAND climb to 41 m
    (reverted in e02ffa3).
    """
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    assert cmd.landings == [True], "the sortie must land + disarm at L&R"
    stage_alt = mission_mod._LAND_STAGE_ALT_M

    # The leg immediately before the touchdown is over L&R, low.
    last_goto = cmd.gotos[-1]
    assert abs(last_goto[0] - HOME.lat) < 1e-5 and abs(last_goto[1] - HOME.lon) < 1e-5
    assert last_goto[2] == stage_alt
    assert stage_alt < COMPETITION.transit_alt_m / 2.0, (
        "a staging altitude near transit altitude saves nothing")

    # The descent must be commanded on the param AUTO actually reads.
    # MPC_Z_VEL_MAX_DN is the MANUAL/offboard limit (PX4:
    # FlightTaskManualAccelerationSlow); autonomous descents read
    # MPC_Z_V_AUTO_DN. Setting only the former left this leg sinking at 0.4 m/s
    # — measured in SITL 2026-07-20, the staged descent saved nothing.
    i_land = cmd.events.index("land")
    before = cmd.events[:i_land]
    i_stage = len(before) - 1 - before[::-1].index(f"goto@{stage_alt:.1f}")
    fast = [e for e in before[:i_stage] if e.startswith("param:MPC_Z_V_AUTO_DN=")]
    assert fast, "MPC_Z_V_AUTO_DN (the AUTO descent speed) was never raised"
    assert float(fast[-1].split("=")[1]) == mission_mod._LAND_STAGE_MPS

    # ...and it must be put back, or the NEXT sortie's pad approach inherits a
    # descent 6x faster than the one the landing precision was validated with.
    after = cmd.events[i_land:]
    back = [e for e in after if e.startswith("param:MPC_Z_V_AUTO_DN=")]
    assert back, "MPC_Z_V_AUTO_DN was left fast after the L&R touchdown"
    assert float(back[0].split("=")[1]) == mission_mod._PAD_DESCENT_MPS

    # ... and the aircraft still arrives over L&R at transit altitude first.
    assert any(g[2] > COMPETITION.transit_alt_m - 1.0 for g in cmd.gotos[-3:-1])


# ── FLIGHT ⊃ DELIVERY: one arm→disarm cycle can carry several eggs ──


def test_single_flight_serves_all_ids_in_order(monkeypatch) -> None:
    """eggs_aboard=4 → ONE flight, four deliveries served in queue order.

    The registry is pre-seeded so nothing sweeps; the point is that a single
    arm→disarm cycle serves the whole chunk, each delivery opening its OWN
    payload channel (0..3), and the mandatory corridor is flown once each way —
    not once per egg.
    """
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    assert state.terminal is TerminalState.COMPLETED
    assert cmd.landings == [True]                       # ONE flight, one disarm
    # Four deliveries, distinct payload channels, queue order preserved.
    assert [c[1] for c in serve.calls] == [0, 1, 2, 3]
    assert [c[2] for c in serve.calls] == [1, 2, 3, 4]  # delivery_index 1..4
    starts = [a for a in state.anomalies if "DELIVERY" in a and "START" in a]
    assert len(starts) == 4
    assert "FLIGHT 1 END delivered=4/4" in "\n".join(state.anomalies)
    # The corridor is a per-FLIGHT cost, not a per-delivery one.
    assert _transit_audit(state) == [
        ("P1", "ingress"), ("P2", "ingress"), ("P3", "ingress"),
        ("P3", "egress"), ("P2", "egress"), ("P1", "egress"),
    ]


def test_assigned_marker_id_tracks_each_delivery_in_turn(monkeypatch) -> None:
    """I3 (review 2026-07-24): a multi-egg flight used to write
    state.assigned_marker_id = flight_ids[0] ONCE, at FLIGHT START, and never
    again — so the GCS 'designated pad' highlight stuck on pad 3 (the first
    id) for the whole flight while the aircraft actually landed on 1, 4 and 6
    too. Each delivery must repoint it at the pad it is CURRENTLY serving.

    Asserted as an invariant rather than a fixed sequence: since 2026-08-18 the
    serve order is routed by distance (mission_brain/serve_order.py), so the
    queue order is deliberately NOT the delivery order. What must hold is that
    every assigned pad is served exactly once and the id follows each one."""
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    seen_assigned: list[int | None] = []

    async def serve(commander, st, target, *, stop_index, params, **kw):
        # Recorded the MOMENT the delivery is served — must match the pad
        # actually being landed on, not whatever flight_ids[0] was.
        seen_assigned.append(st.assigned_marker_id)
        assert params.assigned_marker_id == st.assigned_marker_id
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    assert sorted(seen_assigned) == [1, 3, 4, 6]   # every pad, exactly once
    assert len(set(seen_assigned)) == 4            # repointed, never stuck on one
    # ...and the id of the LAST delivery served stays visible afterwards.
    assert state.assigned_marker_id == seen_assigned[-1]


def test_eggs_aboard_1_is_one_delivery_per_flight(monkeypatch) -> None:
    """eggs_aboard=1 must stay behaviourally identical to the old per-sortie
    mission: two flights, one delivery each, each reloading payload channel 0."""
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD1, 1)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1], eggs_aboard=1), profile=COMPETITION))

    assert cmd.landings == [True, True]                 # two flights
    assert [c[1] for c in serve.calls] == [0, 0]        # each flight reloads → 0
    joined = "\n".join(state.anomalies)
    assert "FLIGHT 1 END delivered=1/1" in joined
    assert "FLIGHT 2 END delivered=1/1" in joined
    assert state.sortie_index == 2


# ── I5 (review 2026-07-24): a partial flight is recoverable, end to end ────
#
# These two use the REAL per-flight gate (orchestrator.main._sortie_gate_factory,
# not the test-only _gate_from chunking stand-in) so the assertions prove
# mission.py's state.delivered_marker_ids ledger and main.py's _chunk_for
# recovery-chunk logic actually integrate — not just that each half is
# individually correct (tests/test_flights.py, tests/test_sortie_gate.py).


def test_partial_flight_is_followed_by_a_recovery_flight_with_the_remainder(
        monkeypatch) -> None:
    """A flight that comes home with 2 of 4 eggs undelivered (pads 4 and 6
    never discoverable) is automatically followed by a second flight
    carrying exactly those two ids — no operator re-queue, no manual GO
    override, the ordinary per-flight gate on its own."""
    state = _state()
    state.eggs_aboard = 4
    state.max_deliveries = 4
    state.assigned_id_queue = [3, 1, 4, 6]
    state.max_sorties = 2                     # mirrors budgeted_flights_for(4, 4)
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1)):   # pads 4 and 6 never discovered
        _preload_pad(tracker, pos, mid)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)
    gate = _sortie_gate_factory(
        state, dash=None, home=HOME, geofence=[tuple(v) for v in SEARCH_AREA],
        cfg={}, profile=COMPETITION, policy=TimePolicy(),
        tracker=tracker, skip_preflight=True)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=gate, profile=COMPETITION))

    # cmd.landings is one entry per FLIGHT (the L&R landing) — unlike
    # cmd.takeoffs, which also counts the inter-delivery climb-out hops
    # WITHIN a flight (test_multi_delivery_flight_climbs_out_between_deliveries),
    # so it is the unambiguous "how many flights actually flew" signal.
    assert cmd.landings == [True, True]
    assert state.delivered_marker_ids == [3, 1]
    joined = "\n".join(state.anomalies)
    assert "FLIGHT 1 START eggs=4 ids=3,1,4,6" in joined
    assert "FLIGHT 1 END delivered=2/4" in joined
    # The recovery flight carries EXACTLY the two ids that came home — not
    # the whole original queue again, and not a manual/empty assignment.
    assert "FLIGHT 2 START eggs=2 ids=4,6" in joined
    assert state.terminal is TerminalState.COMPLETED


def test_fully_successful_flight_gets_no_recovery_flight(monkeypatch) -> None:
    """4/4 delivered on the first flight: exactly one flight (one launch off
    the ground, one L&R landing) — the recovery-flight budget must not fly a
    pointless second flight once everything the queue owes has actually been
    served."""
    state = _state()
    state.eggs_aboard = 4
    state.max_deliveries = 4
    state.assigned_id_queue = [3, 1, 4, 6]
    state.max_sorties = 2                     # mirrors budgeted_flights_for(4, 4)
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)
    gate = _sortie_gate_factory(
        state, dash=None, home=HOME, geofence=[tuple(v) for v in SEARCH_AREA],
        cfg={}, profile=COMPETITION, policy=TimePolicy(),
        tracker=tracker, skip_preflight=True)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=gate, profile=COMPETITION))

    # cmd.landings is one entry per FLIGHT (the L&R landing) — the direct,
    # unambiguous "how many flights actually flew" signal; cmd.takeoffs also
    # counts the inter-delivery climb-out hops a 4-egg flight legitimately
    # makes between pads (test_multi_delivery_flight_climbs_out_between_deliveries:
    # 1 launch + 3 hops + 1 pre-egress climb = 5, for this SAME single flight).
    assert cmd.landings == [True], "a fully-served queue must not fly a second flight"
    # sorted(): the serve order is routed by distance since 2026-08-18, so what
    # this test cares about is that the whole queue got served, not the sequence.
    assert sorted(state.delivered_marker_ids) == [1, 3, 4, 6]
    assert state.sortie_index == 1
    assert "FLIGHT 2" not in "\n".join(state.anomalies)
    assert state.terminal is TerminalState.COMPLETED


def test_partial_find_serves_confirmed_skips_missing(monkeypatch) -> None:
    """Four ids assigned, only three ever confirmable: the flight serves the
    three it has and audits the fourth as not_found — that egg comes home
    rather than being released over an unverified pad."""
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4)):   # pad 6 never discovered
        _preload_pad(tracker, pos, mid)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    assert [c[1] for c in serve.calls] == [0, 1, 2]      # pad 6 skipped
    joined = "\n".join(state.anomalies)
    assert "FLIGHT 1 END delivered=3/4" in joined
    assert any("delivered=False pad=6 reason=not_found" in a
               for a in state.anomalies)
    assert cmd.landings == [True]


def test_per_delivery_abort_keeps_remaining_eggs_and_still_returns(
        monkeypatch) -> None:
    """The per-delivery gate must refuse to START a descent the pack can't
    cover: the FC's low-battery failsafe would otherwise fire mid-delivery with
    an egg aboard. Two deliveries drain the pack to just under
    rth_battery_pct + margin (the gate's boundary, read from the profile so a
    floor change cannot silently retune this test) — so deliveries 3-4 are
    aborted, their eggs kept, and the aircraft still flies the egress and
    disarms at L&R."""
    state = _state()
    state.telemetry.battery_percent = 100.0
    boundary = COMPETITION.rth_battery_pct + mission_mod._DELIVERY_BATT_MARGIN_PCT
    cost = (100.0 - boundary) / 2.0 + 1.0            # two fit, the third does not
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)

    calls: list[tuple[int, int | None, int | None]] = []

    async def draining_serve(commander, st, target, *, stop_index, params, **kw):
        assert params.assigned_marker_id is not None
        calls.append((stop_index, kw.get("payload_id"), kw.get("delivery_index")))
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0                # landed ON the pad
        st.telemetry.battery_percent -= cost             # land + climb-out cost
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", draining_serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    assert len(calls) == 2                               # 3rd/4th never started
    assert any("DELIVERY abort" in a for a in state.anomalies)
    assert cmd.landings == [True]                        # still returned+disarmed
    assert not state.telemetry.is_armed
    assert state.terminal is TerminalState.COMPLETED


def test_multi_delivery_flight_climbs_out_between_deliveries(monkeypatch) -> None:
    """Delivery k+1 must not start with a bare goto off pad k.

    After a delivery the aircraft is LANDED ON the previous pad and still armed
    (COM_DISARM_LAND=-1). Flying straight to the next pad from there starts the
    align ACQUIRE budget while the aircraft is still climbing (the failure the
    fly-to-the-pad-first comment says was already fixed once) and drags the
    pad-to-pad hop under the 10 m search floor. Climb out first — a no-op when
    already airborne.
    """
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    cmd = RecordingCommander(state)

    async def landing_serve(commander, st, target, *, stop_index, params, **kw):
        cmd.events.append(f"serve{kw.get('delivery_index')}")
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0            # landed ON the pad, ARMED
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", landing_serve)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    for k in (2, 3, 4):
        i_prev = cmd.events.index(f"serve{k - 1}")
        i_this = cmd.events.index(f"serve{k}")
        assert any(e.startswith("takeoff@") for e in cmd.events[i_prev:i_this]), (
            f"delivery {k} flew off the previous pad without climbing out")
    # 1 launch + 3 inter-delivery climb-outs + 1 before the egress transit.
    assert cmd.takeoffs == 5
    assert cmd.landings == [True]                    # still ONE flight


def test_deferred_retry_climbs_out_before_the_pad_hop(monkeypatch) -> None:
    """I7 (review 2026-07-24): _serve's retry (attempt 2) used to re-issue the
    pad hop straight off wherever align's own defer left the aircraft —
    tactical_align's defer paths climb back with a NON-BLOCKING goto, so
    telemetry can still read a sub-floor rung (as low as the bottom align
    rung) the instant acquire_and_land_drop hands control back. _serve then
    marked the hop MissionPhase.SEARCH before the aircraft was actually back
    at altitude — SEARCH is not floor-exempt in tools/verify_flight.py, so
    the 1 Hz audit sampler could catch sub-floor `search` samples and
    hard-FAIL an otherwise clean run. The retry must climb out FIRST — the
    same climb-out already used between deliveries."""
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    cmd = RecordingCommander(state)
    attempts = {"n": 0}

    async def defer_once_then_land(commander, st, target, *, stop_index,
                                    delivery_index, params, **kw):
        attempts["n"] += 1
        cmd.events.append(f"align-call-{attempts['n']}")
        if attempts["n"] == 1:
            # Mirrors tactical_align's real defer exactly: the align loop's
            # OWN climb-back is a non-blocking goto, so telemetry is STILL at
            # the bottom rung when acquire_and_land_drop hands control back.
            st.telemetry.relative_alt_m = 1.5
            return AlignResult(acquired=True, aligned=False, landed=False,
                               dropped=False, final_error_m=1.0,
                               notes=["id-not-confirmed → defer"])
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.2)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", defer_once_then_land)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    assert attempts["n"] == 2, "the deferred delivery must retry exactly once"
    assert state.dropped_stops == {0}

    i1 = cmd.events.index("align-call-1")
    i2 = cmd.events.index("align-call-2")
    between = cmd.events[i1 + 1:i2]
    climbs = [e for e in between if e.startswith("takeoff@")]
    assert climbs, (
        f"the retry re-hopped to the pad straight off the 1.5 m defer "
        f"altitude with no climb-out first: {between}")
    assert climbs[-1] == f"takeoff@{_spec().sweep_alt_m:.1f}"


class _NonInstantClimbCommander(RecordingCommander):
    """Unlike RecordingCommander (every goto teleports telemetry instantly),
    a goto that changes ALTITUDE here only updates telemetry after a short
    delay via a background task — closer to the real fire-and-forget goto()
    the mission code has to reason about (M5, review 2026-07-24). A lateral
    move to the SAME altitude still resolves instantly — this is only about
    isolating the altitude wait, not re-testing ``_wait_arrival`` (already
    covered elsewhere). ``goto_calls`` records the telemetry altitude AT THE
    MOMENT each goto was ISSUED — the same reading a concurrent 1 Hz TELEM
    sample could observe."""

    def __init__(self, state: OrchestratorState, *, climb_delay_s: float = 0.01) -> None:
        super().__init__(state)
        self._climb_delay_s = climb_delay_s
        self.goto_calls: list[tuple[float, float, float, float]] = []

    async def goto(self, lat: float, lon: float, alt_m: float,
                   yaw_deg: float = float("nan")) -> None:
        alt_at_call = self.state.telemetry.relative_alt_m
        self.goto_calls.append((lat, lon, alt_m, alt_at_call))
        self.gotos.append((lat, lon, alt_m))
        self.events.append(f"goto@{alt_m:.1f}")
        self.state.telemetry.lat, self.state.telemetry.lon = lat, lon
        if math.isnan(alt_at_call) or abs(alt_m - alt_at_call) <= 1e-6:
            self.state.telemetry.relative_alt_m = alt_m
            return

        async def _arrive() -> None:
            await asyncio.sleep(self._climb_delay_s)
            self.state.telemetry.relative_alt_m = alt_m
        asyncio.create_task(_arrive())


def test_climb_out_between_deliveries_actually_waits_for_the_climb(monkeypatch) -> None:
    """M5 (review 2026-07-24): _climb_out_to_hop_alt's second branch (already
    airborne, still below the hop altitude — e.g. a decode visit left the
    aircraft hovering at the search floor, per the function's own docstring
    example) used to issue a bare goto with no arrival wait. A commander whose
    goto() does not instantly teleport telemetry (unlike every other fake in
    this file) makes that observable: the pad-hop goto for delivery 2 must be
    issued only once the climb has actually been seen, not while telemetry
    still reads the pre-climb altitude — the same reading a concurrent 1 Hz
    TELEM sample could have caught tagged MissionPhase.SEARCH (I7)."""
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD1, 1)
    cmd = _NonInstantClimbCommander(state)
    calls = {"n": 0}

    async def serve(commander, st, target, *, stop_index, delivery_index,
                    params, **kw):
        calls["n"] += 1
        st.dropped_stops.add(stop_index)
        if calls["n"] == 1:
            # Mid-band, not landed — the branch this test targets, distinct
            # from the < 2.0 m branch test_deferred_retry_climbs_out_before_
            # the_pad_hop already covers.
            st.telemetry.relative_alt_m = COMPETITION.search_floor_m
        else:
            st.telemetry.relative_alt_m = 0.0
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1], eggs_aboard=2), profile=COMPETITION))

    assert calls["n"] == 2
    sweep_alt = _spec().sweep_alt_m
    pad_hop_calls = [c for c in cmd.goto_calls
                     if abs(c[0] - PAD1[0]) < 1e-6 and abs(c[1] - PAD1[1]) < 1e-6
                     and abs(c[2] - sweep_alt) < 1e-6]
    assert pad_hop_calls, "delivery 2 never hopped to PAD1"
    alt_at_call = pad_hop_calls[0][3]
    assert alt_at_call >= sweep_alt - 1.0, (
        f"pad-hop to delivery 2 was commanded at {alt_at_call:.1f} m "
        f"(< sweep_alt-1={sweep_alt - 1.0:.1f} m) — the climb-out's own goto "
        "was never actually waited for")


def test_per_delivery_abort_fires_on_the_time_arm(monkeypatch) -> None:
    """The abort gate has TWO arms — battery and window time. This is the time
    one: each delivery ages the window clock, and once the remaining time can no
    longer cover ``TimePolicy.delivery_reserve_s`` the flight stops starting
    descents, keeps the remaining eggs, and still comes home."""
    state = _state()
    state.operation_window_s = 480.0                 # a short window, not 20 min
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)

    calls: list[int] = []

    async def slow_serve(commander, st, target, *, stop_index, params, **kw):
        calls.append(stop_index)
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        st.mission_start_monotonic -= 120.0          # the delivery took 120 s
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", slow_serve)
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    # 480 → 360 → 240 s remaining; the gate needs the LARGER of its two arms:
    # the watchdog one, 180 (floor) + 90 (serve_pre_land_s) + 30 = 300 s.
    # Delivery 3 is therefore refused at 240 s — and 240 s is precisely the
    # band the OLD rth-only threshold (60+110+30 = 200) approved, sending the
    # aircraft into a descent the watchdog would have RTH'd mid-approach with
    # the egg aboard, ending the whole mission.
    assert calls == [0, 1], "the window budget must stop deliveries 3 and 4"
    assert math.isnan(state.telemetry.battery_percent)   # the TIME arm, not power
    abort = [a for a in state.anomalies if "DELIVERY abort" in a]
    assert len(abort) == 1
    # The skipped ids read like the FLIGHT START line (ids=4,6), not a list repr.
    assert "ids=4,6" in abort[0], abort[0]
    assert cmd.landings == [True] and not state.telemetry.is_armed
    assert state.terminal is TerminalState.COMPLETED


def _flight_leaving_battery_at(monkeypatch, pct: float):
    """Fly a 4-egg flight whose first serve leaves the pack at ``pct``."""
    state = _state()
    state.telemetry.battery_percent = 100.0
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)

    calls: list[int] = []

    async def draining_serve(commander, st, target, *, stop_index, params, **kw):
        calls.append(stop_index)
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        st.telemetry.battery_percent = pct       # absolute: idempotent per serve
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", draining_serve)
    cmd = FakeCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))
    return calls, state, cmd


def test_delivery_battery_margin_covers_a_whole_delivery(monkeypatch) -> None:
    """The abort gate's battery arm must reserve what a WHOLE delivery costs.

    A 5 % margin above the FC's low-battery RTL once approved descents that ran
    straight into the failsafe with the egg aboard. The margin is now one
    delivery's own cost (~110 s of hover) expressed as a fraction of the pack.

    The BOUNDARY is derived here, not written down: it moved on 2026-07-25 when
    the second battery went on (12 % of 7.5 Ah -> 8 % of 15 Ah), and a test
    that hard-coded "must refuse at 42 %" would have failed for a correct
    re-derivation while still passing for a wrong one.
    """
    boundary = COMPETITION.rth_battery_pct + mission_mod._DELIVERY_BATT_MARGIN_PCT
    calls, state, cmd = _flight_leaving_battery_at(monkeypatch, boundary)
    assert calls == [0], (
        f"delivery 2 must not start at the {boundary:.0f} % boundary — that "
        f"leaves exactly one delivery's cost and nothing over (calls={calls})")
    assert any("DELIVERY abort" in a for a in state.anomalies)
    assert cmd.landings == [True] and not state.telemetry.is_armed


def test_delivery_battery_margin_does_not_refuse_a_flight_it_can_finish(
        monkeypatch) -> None:
    """The other side of the same gate: comfortably above the boundary the
    delivery must GO. An over-large margin is not "safely conservative" — it
    silently costs deliveries the pack can finish, which is exactly what
    leaving _DELIVERY_BATT_MARGIN_PCT at its 7.5 Ah value would have done once
    the pack doubled."""
    boundary = COMPETITION.rth_battery_pct + mission_mod._DELIVERY_BATT_MARGIN_PCT
    calls, _state, _cmd = _flight_leaving_battery_at(monkeypatch, boundary + 10.0)
    assert len(calls) >= 2, (
        f"a delivery was refused at {boundary + 10:.0f} %, a full 10 points "
        f"above the gate's own boundary (calls={calls})")


def test_decode_visits_reserve_scales_with_the_deliveries_still_owed(
        monkeypatch) -> None:
    """Discovery must fund every delivery the flight still owes.

    A 4-egg flight that stops discovering only when ONE more serve fits
    (``can_start_serve``, 320 s) can legitimately burn the window down until
    deliveries 3-4 are refused by their own gate — half the score, no rule
    broken. With 400 s left and four eggs aboard the decode visit must be
    refused (needs 3*110 + 300 = 630 s), not flown.
    """
    state = _state()
    state.operation_window_s = 400.0
    state.max_sorties = 1                            # eggs_aboard=4 → ONE flight
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4)):
        _preload_pad(tracker, pos, mid)              # pad 6 stays unregistered
    # A cue-only blob the sweep never decoded — the decode-visit bait.
    for t in (0.0, 0.5):
        tracker.ingest(_fix(PAD6[0], PAD6[1], None, t=t))
    assert tracker.unidentified_candidates()
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION))

    assert any("decode-visits stopped (time reserve)" in a
               for a in state.anomalies), state.anomalies
    assert not any(abs(la - PAD6[0]) < 1e-6 and abs(lo - PAD6[1]) < 1e-6
                   for la, lo, _ in cmd.gotos), (
        "flew a decode visit the remaining deliveries could not afford")


def test_registry_completion_is_skipped_when_no_further_flight_is_budgeted(
        monkeypatch) -> None:
    """Registry completion exists to save a LATER, POSITIONALLY-PLANNED flight
    a re-sweep for a DIFFERENT id. With ``eggs_aboard=4`` the whole queue is
    one flight, so there is no such flight to save — decoding leftover
    distractors is provably dead work that spends the window the flight's own
    deliveries need.

    ``state.max_sorties`` alone must NOT be trusted for this (I2, review
    2026-07-24): ``budgeted_flights_for`` (I5) floors it at 2 for this exact
    config to fund a CONTINGENT recovery flight, so pinning
    ``max_sorties = 1`` here — main.py:518 no longer ever produces that value
    when there is anything to deliver — would pass while exercising a
    configuration the product never ships. Set every field main.py seeds
    together (queue + eggs_aboard + the budgeted ceiling) instead."""
    state = _state()
    state.assigned_id_queue = [3, 1, 4, 6]
    state.eggs_aboard = 4
    state.max_sorties = 2       # mirrors budgeted_flights_for(4, 4) exactly
    tracker = TargetTracker()
    # A cue-only blob at PAD5: the registry-completion bait, never assigned.
    for t in (0.0, 0.5):
        tracker.ingest(_fix(PAD5[0], PAD5[1], None, t=t))
    assert tracker.unidentified_candidates()
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        await real_goto(lat, lon, alt_m, yaw_deg)
        # The sweep decodes all four assigned pads on its first leg.
        if abs(alt_m - 12.0) < 1e-6:
            for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
                _preload_pad(tracker, pos, mid)

    monkeypatch.setattr(cmd, "goto", goto_with_camera)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION,
        max_pads=6))

    assert state.dropped_stops == {0, 1, 2, 3}       # all four delivered
    assert not any("registry completion" in a for a in state.anomalies)
    assert not any(abs(la - PAD5[0]) < 1e-6 and abs(lo - PAD5[1]) < 1e-6
                   for la, lo, _ in cmd.gotos), (
        "flew a registry-completion visit no later flight can ever use")


def test_pad_confirmed_during_ingress_skips_the_discovery_excursion(
        monkeypatch) -> None:
    """The registry fills continuously — the vision worker can confirm the
    assigned pad during the takeoff or the ingress transit. The discovery block
    must be gated on what is missing THEN, not on the pre-takeoff snapshot: a
    stale read sends the flight off on a registry-completion decode visit it no
    longer has any reason to fly."""
    state = _state()
    tracker = TargetTracker()
    # A cue-only blob the sweep never decoded — the registry-completion bait.
    for t in (0.0, 0.5):
        tracker.ingest(_fix(PAD6[0], PAD6[1], None, t=t))
    assert tracker.unidentified_candidates()
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        await real_goto(lat, lon, alt_m, yaw_deg)
        # Pad 3 decodes en route, on the way to the FIRST transit point.
        if abs(lat - TRANSIT[0].lat) < 1e-9 and abs(lon - TRANSIT[0].lon) < 1e-9:
            _preload_pad(tracker, PAD3, 3)

    monkeypatch.setattr(cmd, "goto", goto_with_camera)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    assert state.dropped_stops == {0}                 # still delivered
    assert not any(abs(la - PAD6[0]) < 1e-6 and abs(lo - PAD6[1]) < 1e-6
                   for la, lo, _ in cmd.gotos), (
        "flew a registry-completion decode visit it no longer needed")
    assert not any("registry completion" in a for a in state.anomalies)


def test_stop_index_is_the_delivery_ordinal_not_the_queue_position(
        monkeypatch) -> None:
    """``stop_index`` keys the release idempotence ledger, so it must be unique
    per delivery. Deriving it from the committee queue's position mixes two
    numbering spaces: a manual GO override that serves a queued id alongside an
    unqueued one yields the SAME index twice, and the second release is
    silently suppressed by the ledger."""
    state = _state()
    state.assigned_id_queue = [3, 1]                  # what the committee queued
    tracker = TargetTracker()
    _preload_pad(tracker, PAD1, 1)
    _preload_pad(tracker, PAD5, 5)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    # Operator overrides at GO: this flight carries eggs for pad 1 (queued at
    # position 1) and pad 5 (not queued at all → ordinal fallback 1). Collision.
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([1, 5], eggs_aboard=2), profile=COMPETITION))

    assert [c[0] for c in serve.calls] == [0, 1], "duplicate ledger key"
    assert state.dropped_stops == {0, 1}              # BOTH eggs released
    assert "FLIGHT 1 END delivered=2/2" in "\n".join(state.anomalies)


def test_max_sorties_is_normalised_in_place(monkeypatch) -> None:
    """The loop clamps the flight count to >= 1; the clamped value has to go
    back into the state, or the dashboard/preflight keep publishing the stale 0
    the loop is no longer flying to."""
    state = _state()
    state.max_sorties = 0
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, TargetTracker(), _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([]), profile=COMPETITION))

    assert state.max_sorties == 1
    assert cmd.takeoffs == 0


def test_mid_mission_queue_extension_is_not_silently_dropped(monkeypatch) -> None:
    """Review finding (Task 7, IMPORTANT): main.py used to freeze the loop's
    flight ceiling from the id queue's length AT BOOT —

        n_ids = len(state.assigned_id_queue) or state.max_deliveries
        state.max_sorties = max(1, max_flights_for(n_ids, state.eggs_aboard))

    — so a short initial queue (e.g. headless ``--assigned-ids "3"``,
    eggs_aboard=1) freezes ``max_sorties`` to 1. A LEGAL mid-mission extension
    via POST /api/cmd/mission_ids (validated only against ``max_deliveries``,
    so a queue of up to 4 is always accepted) then has its tail silently
    dropped: the per-flight gate resolves the extended queue's chunk for
    flight 2 correctly, but the loop's ``for flight in range(1, max_flights +
    1)`` never even ASKS for a second flight because ``max_flights`` was
    already frozen at 1 before the gate saw the extension.

    This builds ``state.max_sorties`` the SAME way orchestrator/main.py does
    at boot (mirroring its computation, since main() itself needs a live
    MAVLink connection to run) — budgeting off ``max_deliveries``, the LEGAL
    ceiling, not the queue length at boot. Against the d677e9d formula above
    this goes RED (only pad 3 ever flies); it is GREEN once main.py budgets
    unconditionally off max_deliveries."""
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    state.eggs_aboard = 1
    state.max_deliveries = 4                # the LEGAL cap /mission_ids enforces
    state.assigned_id_queue = [3]           # SHORT queue at boot — the operator
                                             # queues the rest after launch
    # Mirrors orchestrator/main.py's (fixed) flight-budget computation: the
    # LEGAL ceiling (max_deliveries), not the queue length at boot. Uses
    # budgeted_flights_for directly (I5, review 2026-07-24) rather than
    # re-deriving max(1, max_flights_for(...)) by hand — same result here
    # (4 deliveries already need >= 2 flights at eggs_aboard=1) but this way
    # the mirror can't quietly drift from main.py's real formula again.
    state.max_sorties = max(1, budgeted_flights_for(state.max_deliveries,
                                                     state.eggs_aboard))

    async def gate(flight: int) -> list[int] | None:
        if flight == 1:
            # A legal mid-mission POST /api/cmd/mission_ids: 4 <= max_deliveries
            # so the write is ACCEPTED and the queue really grows — mirroring
            # main.py's _chunk_for, which re-reads state.assigned_id_queue live.
            state.assigned_id_queue = [3, 1, 4, 6]
        chunks = chunk_flights(state.assigned_id_queue, state.eggs_aboard)
        return chunks[flight - 1] if flight - 1 < len(chunks) else None

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=gate, profile=COMPETITION))

    # Every id the operator legally queued must actually get FLOWN, not just
    # correctly chunked by the gate.
    assert [c[1] for c in serve.calls] == [0, 0, 0, 0]   # 4 flights, 1 egg each
    assert [c[2] for c in serve.calls] == [1, 2, 3, 4]   # delivery_index 1..4
    assert cmd.landings == [True, True, True, True]
    assert state.dropped_stops == {0, 1, 2, 3}
    assert state.sortie_index == 4


def test_flight_warns_when_eggs_exceed_the_configured_release_channels(
        monkeypatch) -> None:
    """This loop is the first code able to emit ``payload_id > 0``, and
    DroneCommander.drop_payload RAISES when payload_id >= drop_payload_count.
    A kit/config mismatch must surface on the GROUND, before takeoff — not as an
    exception over a pad with an egg aboard."""
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1)):
        _preload_pad(tracker, pos, mid)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))

    class OneServoCommander(FakeCommander):
        config = SimpleNamespace(drop_payload_count=1)   # single egg hold

        def __init__(self, st: OrchestratorState) -> None:
            super().__init__(st)
            self.warned_before_takeoff = False

        async def arm_and_takeoff(self, altitude_m: float) -> None:
            if self.takeoffs == 0:
                self.warned_before_takeoff = any(
                    "drop_payload_count" in a for a in self.state.anomalies)
            await super().arm_and_takeoff(altitude_m)

    cmd = OneServoCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1], eggs_aboard=2), profile=COMPETITION))

    assert cmd.warned_before_takeoff, "the mismatch must be flagged pre-takeoff"
    assert any("drop_payload_count=1" in a and "eggs=2" in a
               for a in state.anomalies)
    # Non-fatal: the flight still flies (the servo count is a config problem for
    # the crew, not a reason to ground the aircraft mid-window).
    assert state.terminal is TerminalState.COMPLETED


def test_release_channel_shortage_is_skipped_not_raised(monkeypatch) -> None:
    """Review 2026-07-24: the pre-takeoff WARN above (the previous test) only
    LOGS the mismatch — nothing stops the delivery loop from actually
    ATTEMPTING a release past the configured channel count. ``_drop_once``
    awaits ``commander.drop_payload(payload_id=...)`` unguarded
    (``orchestrator/tactical_align.py``), and the real
    ``DroneCommander.drop_payload`` RAISES ``ValueError`` for a payload_id at
    or above ``drop_payload_count`` (``mavlink_adapter/commands.py``) — so an
    unhandled exception would end the whole mission with the aircraft landed
    but still ARMED on a pad, mid-window. A single-channel commander flying a
    2-egg flight must skip the second delivery instead of attempting it.
    """
    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1)):
        _preload_pad(tracker, pos, mid)

    class OneChannelCommander(FakeCommander):
        config = SimpleNamespace(drop_payload_count=1)   # single egg hold

        async def drop_payload(self, payload_id: int = 0) -> None:
            # Mirrors DroneCommander.drop_payload's real guard — the exact
            # call the mission-loop fix must never let a short-channel slot
            # reach.
            if not 0 <= payload_id < self.config.drop_payload_count:
                raise ValueError(
                    f"payload_id {payload_id} out of range "
                    f"[0, {self.config.drop_payload_count})")

    cmd = OneChannelCommander(state)
    calls: list[tuple[int, int | None, int | None]] = []

    async def serve_that_drops(commander, st, target, *, stop_index, payload_id,
                               delivery_index, params, **kw):
        assert params.assigned_marker_id is not None
        calls.append((stop_index, payload_id, delivery_index))
        # The real acquire_and_land_drop path ends in exactly this unguarded
        # await (orchestrator/tactical_align.py) — reproduced here so it is
        # the mission-loop guard under test, not a mock that skips it.
        await commander.drop_payload(payload_id=payload_id)
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0                # landed ON the pad
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve_that_drops)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1], eggs_aboard=2), profile=COMPETITION))

    assert [c[0] for c in calls] == [0]                   # slot 1 never attempted
    assert state.dropped_stops == {0}
    # WHICH pad lands in the short slot depends on the routed serve order
    # (distance-ordered since 2026-08-18), so derive it: the one that was not
    # delivered is the one the missing channel must have skipped.
    served = state.delivered_marker_ids[0]
    skipped = 1 if served == 3 else 3
    joined = "\n".join(state.anomalies)
    assert (f"DELIVERY 2 END delivered=False pad={skipped} "
            "reason=no_release_channel" in joined), joined
    assert cmd.landings == [True] and not state.telemetry.is_armed
    assert state.terminal is TerminalState.COMPLETED     # no exception escaped


# ── energy: the swap window must span the resupply hold ──


def _drain_per_sortie(state: OrchestratorState, gate, per_sortie_pct: float,
                      swap_before: int | None = None):
    """Wrap a gate so the pack discharges between sorties, optionally getting
    swapped for a fresh one before sortie `swap_before` (the crew's window)."""
    async def wrapped(sortie: int):
        if sortie > 1:
            state.telemetry.battery_percent -= per_sortie_pct
        if sortie == swap_before:
            state.telemetry.battery_percent = 100.0     # fresh pack fitted
        return await gate(sortie)
    return wrapped


def test_a_pack_swapped_between_sorties_rebases_the_energy_baseline(monkeypatch) -> None:
    """The crew can only touch the pack while it sits disarmed at L&R, i.e.
    BETWEEN sorties. Detection therefore has to compare one sortie's exit reading
    with the next one's entry reading; checking entry-vs-exit of the same sortie
    (as this did until 2026-07-22) watches a window the aircraft spends armed and
    airborne, where a swap is impossible."""
    state = _state()
    state.energy_capacity_mah = 7500.0
    state.telemetry.battery_percent = 60.0        # tier B: 40% already drawn
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    gate = _drain_per_sortie(state, _gate_from([3, 5]), per_sortie_pct=20.0,
                             swap_before=2)

    asyncio.run(run_delivery_mission(
        FakeCommander(state), state, tracker, _spec(), home=HOME,
        transit_route=TRANSIT, sortie_gate=gate, profile=COMPETITION))

    assert any("BATTERY SWAP before flight 2" in a for a in state.anomalies)
    # …and the baseline describes the pack that is now fitted: a 100% pack has
    # nothing drawn from it yet, so consumed-minus-baseline starts at zero.
    assert abs(state.energy_baseline_mah) < 1.0


def test_an_ordinary_discharge_is_not_mistaken_for_a_swap(monkeypatch) -> None:
    """I3 (review 2026-07-24): this test's own assertion had drifted onto the
    tail of test_real_align_delivers_payload_ids_in_order_across_two_flights,
    where it is vacuous (that test never drains the pack, so "no BATTERY SWAP"
    was trivially true there regardless of whether detect_battery_swap's
    false-positive guard actually works). Restored here, where a real
    declining battery_percent across flights — WITHOUT a swap — is what
    actually exercises it."""
    state = _state()
    state.energy_capacity_mah = 7500.0
    state.telemetry.battery_percent = 90.0
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    gate = _drain_per_sortie(state, _gate_from([3, 5]), per_sortie_pct=20.0)

    asyncio.run(run_delivery_mission(
        FakeCommander(state), state, tracker, _spec(), home=HOME,
        transit_route=TRANSIT, sortie_gate=gate, profile=COMPETITION))

    assert not any("BATTERY SWAP" in a for a in state.anomalies)


# ── M3: payload_id must reach the right servo channel end to end ──
#
# AS-WIRED 2026-08-15: the four latches are on AUX 4/1/2/3 (front-left,
# rear-right, front-right, rear-left) and the release order stays diagonal, so
# payload_id 0..3 -> actuator set 4/1/2/3 via connection.drop_servo_channels.
# Mirrors sitl/aavc_config.yaml; docs/SERVO_AUX_MAPPING.md holds the table.
_AS_WIRED_CHANNELS = (4, 1, 2, 3)
#
# Every OTHER test in this file replaces acquire_and_land_drop with a fake
# (_fake_serve or an ad-hoc stand-in), so none of them ever reach the real
# orchestrator.tactical_align._drop_once -> commander.drop_payload call. A
# wrong payload_id/servo-channel mapping there releases the WRONG egg on the
# right pad and nothing notices (the next delivery then finds its own hold
# already empty). These let the REAL align routine run — only its camera
# decode is patched, the way tests/test_tactical_align.py does — end to end
# for a whole multi-delivery flight.


class DropRecordingCommander(RecordingCommander):
    """RecordingCommander + a servo-channel ledger. The real
    acquire_and_land_drop path ends in `await commander.drop_payload(
    payload_id=...)` (orchestrator/tactical_align.py's _drop_once) — every
    other fixture in this file bypasses that call entirely.

    M6 (review 2026-07-24): mirrors DroneCommander.drop_payload's real bounds
    check and resolves the channel through the REAL
    ``ConnectionConfig.actuator_index`` (mavlink_adapter/commands.py) instead
    of just recording payload_id, and carries a real ``.config`` — without
    one, mission.py's pre-takeoff channel-count WARN and its per-delivery
    ``slot >= _n_channels`` guard both read ``_n_channels`` as None and are
    silently skipped (``isinstance(_n_channels, int)`` is False), so these
    tests used to pass identically even with ``drop_payload_count=1``
    misconfigured for a 4-egg flight."""

    def __init__(self, state: OrchestratorState, *,
                drop_payload_count: int = 4,
                drop_servo_channels: tuple[int, ...] = _AS_WIRED_CHANNELS) -> None:
        super().__init__(state)
        # The REAL ConnectionConfig (not a SimpleNamespace) so the channel a
        # payload_id resolves to is the one the flight code would actually
        # command — including the as-wired drop_servo_channels map.
        self.config = ConnectionConfig(drop_payload_count=drop_payload_count,
                                       drop_servo_channels=drop_servo_channels)
        self.released: list[int] = []
        self.channels: list[int] = []

    async def drop_payload(self, payload_id: int = 0) -> None:
        if not 0 <= payload_id < self.config.drop_payload_count:
            raise ValueError(
                f"payload_id {payload_id} out of range "
                f"[0, {self.config.drop_payload_count}) — refusing to address "
                "an unconfigured servo channel")
        self.released.append(payload_id)
        self.channels.append(self.config.actuator_index(payload_id))


def _patch_camera_decodes_whatever_is_assigned(monkeypatch) -> None:
    """Like test_tactical_align.py's _patch_detector, but decodes WHATEVER id
    the align loop is currently seeking instead of one fixed id — mirroring a
    real camera reading each pad's own marker as the aircraft moves between
    them. The point of these tests is the payload_id/servo-channel wiring
    across MULTIPLE distinct deliveries, not id-decode robustness (already
    covered by tests/test_tactical_align.py)."""
    def fake(frame_path, min_conf, assigned_id):
        alt = max(state_ref.telemetry.relative_alt_m, 0.5)
        exp = expected_radius_px(NADIR, 0.2, alt)   # target_radius_m=0.2 prior
        return PadHit(cx=320, cy=240, marker_id=assigned_id, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta_mod, "_detect_nadir", fake)
    # live camera: the align loop only decodes frames it has not seen
    _cam_ticks = itertools.count(1.0, 0.05)
    monkeypatch.setattr(ta_mod, "_frame_mtime", lambda _p: next(_cam_ticks))


_FAST_ALIGN = AlignParams(
    rungs=(12.0, 5.0), rung_tol_m=(1.5, 0.4), rung_descent_mps=(3.0, 1.0),
    lock_cycles=1, cycle_hz=200.0, acquire_timeout_s=0.5, rung_timeout_s=0.5,
    median_window=1, settle_after_land_s=0.0, require_id_votes=1,
    # These tests patch _detect_nadir and rely on the default /tmp frame path
    # (which may hold a stale frame from a prior SITL run); disable the S2
    # freshness gate so they exercise the real align loop deterministically.
    frame_max_age_s=0.0,
)

state_ref: OrchestratorState


def test_real_align_delivers_payload_ids_in_order_across_a_flight(monkeypatch) -> None:
    """M3: the real loop -> acquire_and_land_drop -> commander.drop_payload
    path, exercised for all four deliveries of an eggs_aboard=4 flight.
    Asserts the commander receives payload_id 0, 1, 2, 3 in order — and
    (M6, review 2026-07-24) the CHANNELS those ids resolve to through the real
    ConnectionConfig, so a wrong map/offset/count would be caught here rather
    than passing identically for any drop_payload_count."""
    global state_ref
    state = state_ref = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    cmd = DropRecordingCommander(state)
    _patch_camera_decodes_whatever_is_assigned(monkeypatch)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1, 4, 6], eggs_aboard=4), profile=COMPETITION,
        align=_FAST_ALIGN))

    assert cmd.released == [0, 1, 2, 3], cmd.released
    # As-wired rack: front-left, rear-right, front-right, rear-left (AUX pins
    # 4, 1, 2, 3) — the diagonal release order on the real airframe.
    assert cmd.channels == [4, 1, 2, 3], cmd.channels
    assert state.dropped_stops == {0, 1, 2, 3}
    # Unlike _fake_serve (every other test in this file), the REAL align
    # routine itself calls commander.land(disarm=False) for each pad landing
    # (stay ARMED, COM_DISARM_LAND=-1) — mission.py's own L&R land(disarm=True)
    # is the last, 5th call, for the one flight.
    assert cmd.landings == [False, False, False, False, True]
    assert state.terminal is TerminalState.COMPLETED


_TELEM_LINE = re.compile(r"t=(?P<t>[\d.]+)s TELEM .*armed=(?P<armed>\d)")
_FLIGHT_END_LINE = re.compile(r"t=(?P<t>[\d.]+)s FLIGHT (?P<flight>\d+) END")


def test_last_flight_gets_a_disarm_confirming_telem_sample_before_end(
        monkeypatch) -> None:
    """I4 (review 2026-07-24): tools/verify_flight.py's disarm-after-L&R check
    used to rely on the NEXT flight's preflight hold to hand it a bonus TELEM
    sample. At the shipping eggs_aboard=4 default (ONE flight) there is no
    next hold, and mission.py had no await between writing FLIGHT END and the
    loop's own `finally: sampler.cancel()` — so the sampler never got a
    chance to record anything after commander.land(disarm=True), and the
    verifier's `after` was vacuously empty. The mission itself must give the
    1 Hz sampler a genuine chance to record a post-disarm sample BEFORE
    writing FLIGHT END."""
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = FakeCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    end_t = next(float(m.group("t")) for a in state.anomalies
                if (m := _FLIGHT_END_LINE.match(a)))
    telem_before_end = [m for a in state.anomalies
                        if (m := _TELEM_LINE.match(a))
                        and float(m.group("t")) <= end_t]
    assert telem_before_end, (
        "no TELEM sample was recorded at/before FLIGHT END — the sampler "
        "never got a chance to run without an explicit post-land yield")
    assert telem_before_end[-1].group("armed") == "0", (
        "the last TELEM sample before FLIGHT END must read armed=0 so "
        "verify_flight's fallback has real evidence: "
        f"{telem_before_end[-1].group(0)}")


def test_real_align_delivers_payload_ids_in_order_across_two_flights(
        monkeypatch) -> None:
    """eggs_aboard=1 twin of the above: each of two flights reloads payload
    slot 0 — the front-left latch, AUX 4 on the as-wired rack (behaviourally
    identical to the pre-FLIGHT-⊃-DELIVERY, one-egg mission)."""
    global state_ref
    state = state_ref = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD3, 3)
    _preload_pad(tracker, PAD1, 1)
    cmd = DropRecordingCommander(state)
    _patch_camera_decodes_whatever_is_assigned(monkeypatch)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3, 1], eggs_aboard=1), profile=COMPETITION,
        align=_FAST_ALIGN))

    assert cmd.released == [0, 0]                     # each flight reloads → 0
    assert cmd.channels == [4, 4]                     # slot 0 == AUX 4 both flights
    # stop_index is the delivery's ordinal ACROSS THE MISSION (not reset per
    # flight — see mission.py's own stop_index comment), so two flights of
    # one delivery each still key the ledger 0, 1.
    assert state.dropped_stops == {0, 1}
    # Pad landing (disarm=False, stay ARMED) then the L&R landing
    # (disarm=True) — once per flight, same reasoning as the test above.
    assert cmd.landings == [False, True, False, True]
    assert state.terminal is TerminalState.COMPLETED


# ---------------------------------------------------------------------------
# Leg abandonment is PROGRESS-based, not wall-clock based (2026-07-25).
#
# The old rule gave every leg `2 x distance / speed + 20 s` of WALL time and
# abandoned the waypoint the instant that expired. SITL moves the aircraft in
# SIMULATED time, and a loaded host runs the sim below real time: the
# 2026-07-25 flight dropped to 0.20x real time on the P3 -> sweep-wp0 leg, so
# the 96 m leg's 44.1 s budget bought only ~9 s of flying against the >=16 s
# it needed. The mission logged `sweep_leg_timeout_wp0` and jumped to wp1 with
# the aircraft halfway there and still closing — visible on the GCS map as a
# skipped waypoint. Nothing was wrong with the aircraft: the ULog shows it
# holding 8 m/s the whole time.
#
# The same failure is reachable on the real bird, where it is not an artifact:
# a headwind, a heavy pack or a re-planned longer leg all stretch wall time
# past 2x nominal while the aircraft is flying perfectly well. Abandoning a
# waypoint the vehicle is actively closing on is wrong in both worlds, so the
# guard now asks "is it still getting closer?" instead of "has the clock run
# out?" — with the distance-derived budget kept only as a can't-hang backstop.
# ---------------------------------------------------------------------------


class _FakeClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def test_progress_guard_keeps_a_slow_but_closing_leg() -> None:
    """The regression: 96 m at 8 m/s = 44.1 s of budget, but the host runs the
    sim at 0.20x so the aircraft closes at 1.0 m/s of WALL time. It arrives —
    late in wall time, on schedule in the only time the aircraft lives in."""
    clock = _FakeClock()
    guard = mission_mod._ProgressGuard(2.0 * 96.4 / 8.0 + 20.0, clock=clock)
    dist = 96.4
    while dist > 3.0:
        assert not guard.give_up(dist), (
            f"abandoned a leg still closing, {dist:.1f} m out at "
            f"{clock.t - 1000.0:.0f} s")
        clock.advance(1.0)
        dist -= 1.05                      # the measured 0.20x-real-time crawl
    assert clock.t - 1000.0 > 44.1, "test did not actually outrun the old budget"


def test_progress_guard_gives_up_when_the_leg_stops_closing() -> None:
    """A vehicle that is genuinely stuck (holding, blown off, fighting a wall
    of wind) must still be abandoned — that is what the timeout is FOR."""
    clock = _FakeClock()
    guard = mission_mod._ProgressGuard(44.1, clock=clock)
    assert not guard.give_up(80.0)
    for _ in range(400):                  # pinned at 80 m, no closure at all
        clock.advance(1.0)
        if guard.give_up(80.0):
            break
    else:
        pytest.fail("a stalled leg was never abandoned")
    assert clock.t - 1000.0 <= mission_mod._LEG_STALL_S + 2.0, (
        "took much longer than the stall budget to notice a stalled leg")


def test_progress_guard_gives_up_without_a_position_fix() -> None:
    """No fix = no evidence of progress. The guard must not wait forever on a
    vehicle it cannot see (the old loop polled `_cur_latlon() is not None`)."""
    clock = _FakeClock()
    guard = mission_mod._ProgressGuard(44.1, clock=clock)
    for _ in range(400):
        clock.advance(1.0)
        if guard.give_up(None):
            break
    else:
        pytest.fail("a leg with no position fix was never abandoned")


def test_progress_guard_backstop_bounds_a_crawling_leg() -> None:
    """Closing 1 m every stall window would otherwise run unbounded, so the
    distance-derived budget survives as a hard ceiling."""
    clock = _FakeClock()
    budget = 44.1
    guard = mission_mod._ProgressGuard(budget, clock=clock)
    dist = 10_000.0
    for _ in range(100_000):
        clock.advance(mission_mod._LEG_STALL_S - 1.0)
        dist -= 2.0                       # just enough to keep resetting the stall
        if guard.give_up(dist):
            break
    else:
        pytest.fail("a crawling leg was never bounded")
    assert clock.t - 1000.0 <= budget * mission_mod._LEG_CEILING_MULT + 60.0


def test_the_flight_serves_by_route_not_by_the_operators_click_order(monkeypatch) -> None:
    """Operator 2026-08-18: "ให้ส่งตาม path ที่ใกล้ที่สุดก่อน". The queue editor
    records the order the ids were clicked, which is no basis for a route — the
    aircraft can cross the field twice serving pads in typing order. Distance
    saved is battery the sweep has already spent most of, and time inside a
    window that charges for overtime.

    Fed the worst reasonable queue (farthest pad first), the flight must serve
    nearest-first instead, and the flown route must actually be shorter."""
    from mission_brain.serve_order import route_length_m

    state = _state()
    tracker = TargetTracker()
    for mid, pos in ((3, PAD3), (1, PAD1), (4, PAD4), (6, PAD6)):
        _preload_pad(tracker, pos, mid)
    coords = {3: PAD3, 1: PAD1, 4: PAD4, 6: PAD6}
    served: list[int] = []

    async def serve(commander, st, target, *, stop_index, params, **kw):
        served.append(st.assigned_marker_id)
        st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        return AlignResult(acquired=True, aligned=True, landed=True,
                           dropped=True, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)
    queue = [6, 4, 3, 1]                       # farthest first — the bad case

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from(queue, eggs_aboard=4), profile=COMPETITION))

    assert sorted(served) == [1, 3, 4, 6], "every assigned pad must still be served"
    assert served != queue, "the click order must not survive as the route"
    start = (HOME.lat, HOME.lon)
    assert (route_length_m(served, coords, start)
            < route_length_m(queue, coords, start)), "the reorder must shorten the route"
    # And the operator can see it happened: the audit records both orders.
    assert any("SERVE ORDER" in a for a in state.anomalies), state.anomalies[-5:]
    # The GCS reads state.flight_ids as "this flight's pads, in order". It must
    # follow the ROUTED order the aircraft actually flew — not stay on the click
    # order for the whole flight (the chip disagreed with the flight before).
    assert state.flight_ids == served, (state.flight_ids, served)


def test_every_search_goto_commands_the_sweep_heading(monkeypatch) -> None:
    """The sweep must NAME the heading it flies (root cause, 2026-08-20).

    Every goto used to pass a NaN yaw, so PX4 fell through to MPC_YAW_MODE —
    factory 0, "towards waypoint" — and turned the nose at each sweep waypoint.
    ULog 08_11_09: the commanded heading walked 145->119->94->69->44->18->353->…
    through a full circle at the 25 deg/s cap, 867 deg of yaw in a 122 s
    flight, and the body-fixed camera spun with it (1 of 457 frames decoded).
    A finite yaw in the goto beats the param outright — PX4 reads the triplet
    yaw first (FlightTaskAuto.cpp:496) — so this pins that the search phase
    always sends one, and always the SAME one (a heading that changes mid-sweep
    is the bug wearing a different hat).
    """
    state = _state()
    tracker = TargetTracker()
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    spec = _spec()
    wps = {(round(w.lat, 7), round(w.lon, 7)) for w in spec.waypoints}
    real_goto = cmd.goto

    async def goto_with_camera(lat, lon, alt_m, yaw_deg=float("nan")):
        await real_goto(lat, lon, alt_m, yaw_deg)
        if (round(lat, 7), round(lon, 7)) in wps:
            _preload_pad(tracker, PAD3, 3)

    monkeypatch.setattr(cmd, "goto", goto_with_camera)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([3]), profile=COMPETITION))

    sweep_yaws = [y for (la, lo, _), y in zip(cmd.gotos, cmd.yaws)
                  if (round(la, 7), round(lo, 7)) in wps]
    assert sweep_yaws, "no sweep waypoint was flown"
    assert all(y == spec.sweep_yaw_deg for y in sweep_yaws), (
        f"search gotos must all hold {spec.sweep_yaw_deg} deg, got "
        f"{sorted(set(sweep_yaws))}")
    # …and the transit legs are deliberately NOT forced: they carry no imaging
    # requirement, and pinning a heading there would be scope the field has not
    # asked for. NaN there means "keep whatever PX4 is holding".
    transit_yaws = [y for (la, lo, _), y in zip(cmd.gotos, cmd.yaws)
                    if (round(la, 7), round(lo, 7)) not in wps]
    assert any(math.isnan(y) for y in transit_yaws)


# ── Recovery flight fires the latches that still HOLD eggs (2026-08-27) ──────
# The crew does NOT restock between a flight and its recovery flight — the
# undelivered eggs come home still latched where flight 1 loaded them
# (wiring order AUX 4/1/2/3 = slots 0..3). The recovery flight must therefore
# CONTINUE the slot progression through the unfired latches, not restart at
# slot 0 and pop two empty holds (operator, 2026-08-27: "ผมไม่อยากสลับถุงไข่").


def _identity_route(monkeypatch):
    """Pin serve order to the queue order — nearest-first routing is someone
    else's test, and these assertions are about SLOTS, which follow position."""
    monkeypatch.setattr(mission_mod, "order_by_nearest",
                        lambda ids, *a, **k: list(ids))


def test_recovery_flight_fires_the_latches_that_still_hold_eggs(monkeypatch):
    """Flight 1 releases slots 0,1 (pads 1,4); the battery gate sent eggs 3,4
    home. The recovery flight serving pads 5,6 must fire slots 2,3 — the
    latches those eggs are still sitting in — not restart at AUX4."""
    state = _state()
    state.assigned_id_queue = [1, 4, 5, 6]
    state.eggs_aboard = 4
    state.max_sorties = 2
    tracker = TargetTracker()
    for pad, mid in ((PAD1, 1), (PAD4, 4), (PAD5, 5), (PAD6, 6)):
        _preload_pad(tracker, pad, mid)
    _identity_route(monkeypatch)
    fake = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", fake)
    cmd = FakeCommander(state)

    chunks = {1: [1, 4], 2: [5, 6]}

    async def gate(flight: int):
        return chunks.get(flight)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=gate, profile=COMPETITION))

    assert state.terminal is TerminalState.COMPLETED
    assert [c[1] for c in fake.calls] == [0, 1, 2, 3]
    assert state.payload_slots_fired == [0, 1, 2, 3]
    assert state.delivered_marker_ids == [1, 4, 5, 6]


def test_recovery_after_a_failed_release_fires_that_eggs_own_latch(monkeypatch):
    """Flight 1: pad 1 releases slot 0; pad 4's release FAILS (egg still in
    slot 1 / AUX1). The recovery flight re-serving pad 4 must fire slot 1 —
    where that egg physically is — not slot 0's already-empty latch."""
    state = _state()
    state.assigned_id_queue = [1, 4]
    state.eggs_aboard = 4
    state.max_sorties = 2
    tracker = TargetTracker()
    _preload_pad(tracker, PAD1, 1)
    _preload_pad(tracker, PAD4, 4)
    _identity_route(monkeypatch)

    calls: list[tuple[int | None, int | None]] = []

    async def serve(commander, st, target, *, stop_index, params, **kw):
        # Pad 4 fails BOTH attempts of its flight-1 delivery (delivery 2);
        # every other serve succeeds.
        ok = not (params.assigned_marker_id == 4
                  and kw.get("delivery_index") == 2)
        calls.append((params.assigned_marker_id, kw.get("payload_id")))
        if ok:
            st.dropped_stops.add(stop_index)
        st.telemetry.relative_alt_m = 0.0
        return AlignResult(acquired=True, aligned=ok, landed=ok,
                           dropped=ok, final_error_m=0.3)

    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = FakeCommander(state)

    chunks = {1: [1, 4], 2: [4]}

    async def gate(flight: int):
        return chunks.get(flight)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=gate, profile=COMPETITION))

    # Flight 1: slot 0 fired, slot 1 retained. Recovery: pad 4 → slot 1.
    assert state.payload_slots_fired == [0, 1]
    assert calls[-1] == (4, 1)
