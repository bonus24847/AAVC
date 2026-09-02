"""Keep-out routing + the hand-laid sweep (2026-08-28, KMITL competition day).

The briefing turned the KMITL search area into an L: the main building and
the east end became no-fly bands, and the building band sits in the MIDDLE of
the rules' polygon. Every move the mission makes is a straight goto, so
(a) the sweep can no longer be generated from a polygon's bounding box —
``build_explicit_pattern`` flies a hand-laid ENU list instead — and (b) a
goto whose straight line crosses a keep-out polygon detours through one
configured gateway (``mission.py::_goto_routed``), while a decode candidate
or a claimed pad inside a band is refused. These tests pin the geometry
helpers, the explicit pattern, the routing behaviour on the fake commander,
and — against the shipped ``sitl/kmitl_config.yaml`` — that the KMITL legs,
corridor, entry/exit and gateway never cross the bands.
"""

from __future__ import annotations

import asyncio
import math
from pathlib import Path

import pytest
import yaml

from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_explicit_pattern
from orchestrator import mission as mission_mod
from orchestrator import tactical_align as ta_mod
from orchestrator.main import _build_spec
from orchestrator.mission import (
    _point_in_polygon,
    _point_near_polygon_m,
    _segment_crosses_polygon,
    gateway_route,
    run_delivery_mission,
)
from orchestrator.target_tracker import TargetTracker
from tests.test_delivery_mission import (  # noqa: F401 — the fixture is autouse
    HOME,
    PAD5,
    TRANSIT,
    RecordingCommander,
    _fake_serve,
    _fast_telem_sampler,
    _gate_from,
    _preload_pad,
    _spec,
    _state,
)
from vision.projection import CameraModel

_R = 6378137.0
_ROOT = Path(__file__).resolve().parents[1]

# A square keep-out straddling the straight line from P3 (13.730712,
# 100.788755) to PAD5 (13.7311, 100.7893); the gateway sits east of P3 and
# south of the pad so both hops around it are clear.
BOX = [(13.7308, 100.7889), (13.7308, 100.7891), (13.7310, 100.7891), (13.7310, 100.7889)]
GATE = (13.730700, 100.789300)


def _m_to_dlon(m: float, lat: float) -> float:
    return math.degrees(m / (_R * math.cos(math.radians(lat))))


# ── geometry ────────────────────────────────────────────────────────────────

def test_segment_crossing_detection() -> None:
    cross = _segment_crosses_polygon
    assert cross((13.7307, 100.7888), (13.7311, 100.7892), BOX)       # diagonal through
    assert cross((13.7309, 100.7890), (13.7312, 100.7895), BOX)       # starts inside
    assert not cross((13.7307, 100.7888), (13.7307, 100.7895), BOX)   # passes south
    assert not cross((13.7311, 100.7888), (13.7311, 100.7895), BOX)   # passes north
    assert not cross((13.7307, 100.7888), (13.7311, 100.7888), BOX)   # passes west


def test_point_near_polygon_uses_a_metre_margin() -> None:
    lat = 13.7309
    assert _point_near_polygon_m(lat, 100.7890, BOX, 0.0)                       # inside
    assert _point_near_polygon_m(lat, 100.7891 + _m_to_dlon(2.0, lat), BOX, 3.0)  # 2 m outside
    assert not _point_near_polygon_m(lat, 100.7891 + _m_to_dlon(10.0, lat), BOX, 3.0)


# ── the hand-laid sweep ─────────────────────────────────────────────────────

def test_explicit_pattern_round_trips_its_enu_list_and_holds_the_sweep_heading() -> None:
    origin = Coordinate(lat=13.730322, lon=100.787446)
    pts = [(112.0, 58.0), (46.0, 58.0), (46.0, 88.0), (205.0, 88.0)]
    cam = CameraModel(name="nadir", mount_yaw_rad=math.radians(180.0))
    sp = build_explicit_pattern(pts, origin, sweep_alt_m=20.0, speed_mps=3.0,
                                camera=cam, ceiling_m=30.0, leg_bearing_deg=87.0)
    assert sp.leg_count == 2
    assert sp.sweep_alt_m == 20.0
    for (e, n), wp in zip(pts, sp.waypoints):
        de = math.radians(wp.lon - origin.lon) * _R * math.cos(math.radians(origin.lat))
        dn = math.radians(wp.lat - origin.lat) * _R
        assert abs(de - e) < 0.05 and abs(dn - n) < 0.05
        assert wp.alt_m == 20.0
    # Same held-heading rule as the generated pattern: at the measured 180 deg
    # mount the nose-first flip brings the heading back to the leg bearing.
    assert sp.sweep_yaw_deg == pytest.approx(87.0)
    assert sp.est_duration_s == pytest.approx((66.0 + 30.0 + 159.0) / 3.0 + 3 * 3.0)
    # Clamped under the ceiling exactly like build_search_pattern.
    assert build_explicit_pattern(pts, origin, sweep_alt_m=40.0, speed_mps=3.0,
                                  ceiling_m=30.0).sweep_alt_m == 29.0
    with pytest.raises(ValueError):
        build_explicit_pattern(pts[:1], origin, sweep_alt_m=20.0, speed_mps=3.0)


def _kmitl() -> dict:
    return yaml.safe_load((_ROOT / "sitl" / "kmitl_config.yaml").read_text(encoding="utf-8"))


def test_the_kmitl_sweep_stays_in_the_airspace_and_out_of_the_bands() -> None:
    """The shipped KMITL list: every leg inside the geofence, every waypoint
    >= 5 m from a keep-out, no leg / transition / entry / exit / corridor
    segment crossing a band, and the gateway with a clear straight line to
    every waypoint and to the corridor gate (the routing invariant)."""
    cfg = _kmitl()
    sc = cfg["search"]
    assert sc.get("sweep_waypoints_enu"), "KMITL flies the hand-laid sweep since 2026-08-28"
    origin = tuple(float(v) for v in cfg["ground_operation"]["launch_recovery"])
    spec = _build_spec(cfg["search_area"], Coordinate(lat=origin[0], lon=origin[1]),
                       sc, COMPETITION.altitude_ceiling_m, origin=origin)
    assert len(spec.waypoints) == len(sc["sweep_waypoints_enu"])
    assert spec.sweep_alt_m == float(sc["sweep_alt_m"])
    airspace = [tuple(float(x) for x in v) for v in cfg["controlled_airspace"]]
    keepouts = [[tuple(float(x) for x in v) for v in poly]
                for poly in cfg["routing"]["keepout_zones"]]
    gws = [tuple(float(v) for v in g) for g in cfg["routing"]["gateways"]]
    p1, p2, p3 = [tuple(float(x) for x in v) for v in cfg["transit_route"]]
    wps = [(w.lat, w.lon) for w in spec.waypoints]
    for w in wps:
        assert _point_in_polygon(w[0], w[1], airspace), f"waypoint {w} outside the airspace"
        for poly in keepouts:
            assert not _point_near_polygon_m(w[0], w[1], poly, 4.0), f"waypoint {w} hugs a band"
    # every leg / transition, the corridor and the entry stay clear of EVERY band
    segments = list(zip(wps, wps[1:])) + [(p1, p2), (p2, p3), (p3, wps[0])]
    for a, b in segments:
        for poly in keepouts:
            assert not _segment_crosses_polygon(a, b, poly), f"{a}->{b} crosses a band"
    for g in gws:
        assert _point_in_polygon(g[0], g[1], airspace)
    # the routing invariant: from every waypoint AND the corridor gate, a chain
    # of gateways with clear segments reaches every other waypoint and the gate
    def clear(a, b):
        return not any(_segment_crosses_polygon(a, b, poly) for poly in keepouts)

    def reachable(src, dst):
        nodes = [src] + gws + [dst]
        seen = {0}
        frontier = [0]
        while frontier:
            i = frontier.pop()
            for j in range(len(nodes)):
                if j not in seen and clear(nodes[i], nodes[j]):
                    seen.add(j)
                    frontier.append(j)
        return len(nodes) - 1 in seen
    for w in wps:
        assert reachable(w, p3), f"no clear gateway chain from {w} to P3'"
        assert reachable(p3, w), f"no clear gateway chain from P3' to {w}"
    assert reachable(wps[-1], wps[0])


# ── routing on the fake commander ───────────────────────────────────────────

def _pts(cmd: RecordingCommander) -> list[tuple[float, float]]:
    return [(round(la, 6), round(lo, 6)) for la, lo, _ in cmd.gotos]


def test_a_hop_that_would_cross_a_keepout_goes_through_the_gateway(monkeypatch) -> None:
    p3 = (TRANSIT[-1].lat, TRANSIT[-1].lon)
    assert _segment_crosses_polygon(p3, PAD5, BOX)          # the scenario is real
    assert not _segment_crosses_polygon(p3, GATE, BOX)
    assert not _segment_crosses_polygon(GATE, PAD5, BOX)
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)

    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION,
        keepout_zones=[BOX], gateways=[GATE]))

    assert state.dropped_stops == {0}
    pts = _pts(cmd)
    g = (round(GATE[0], 6), round(GATE[1], 6))
    pad = (round(PAD5[0], 6), round(PAD5[1], 6))
    p3r = (round(p3[0], 6), round(p3[1], 6))
    assert g in pts and pad in pts
    i_pad = pts.index(pad)
    assert pts[i_pad - 1] == g, "the hop to the pad must go through the gateway first"
    # …and the egress from the pad back to P3 crosses the box the other way.
    after = pts[i_pad + 1:]
    assert g in after and p3r in after and after.index(g) < after.index(p3r)
    assert sum(1 for e in state.anomalies if "ROUTE via gateway" in e) == 2


def test_without_keepouts_the_same_hop_is_the_straight_line_it_always_was(monkeypatch) -> None:
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION))
    assert state.dropped_stops == {0}
    assert (round(GATE[0], 6), round(GATE[1], 6)) not in _pts(cmd)
    assert not any("ROUTE via gateway" in e for e in state.anomalies)


def test_a_pad_registered_inside_a_keepout_is_refused(monkeypatch) -> None:
    """The committee places pads in the open; a 'pad' inside a band is a false
    hit and landing there would put the aircraft in the band."""
    around_pad = [(PAD5[0] - 0.0002, PAD5[1] - 0.0002), (PAD5[0] - 0.0002, PAD5[1] + 0.0002),
                  (PAD5[0] + 0.0002, PAD5[1] + 0.0002), (PAD5[0] + 0.0002, PAD5[1] - 0.0002)]
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = RecordingCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION,
        keepout_zones=[around_pad], gateways=[GATE]))
    assert state.dropped_stops == set()
    assert serve.calls == []
    assert any("REFUSED" in e and "keep-out" in e for e in state.anomalies)
    assert (round(PAD5[0], 6), round(PAD5[1], 6)) not in _pts(cmd)


def _kmitl_sweep_geometry():
    """(free-ground grid, legs, keep-outs, usable half-swath) in ENU metres.

    The grid is the FLOWN `search_area` minus the keep-out bands.

    2026-08-29 it was `search_area_rules`, because the drawn L stopped at
    N 75.3 east of the building while marker 4 sat at ENU (187.0, 72.1).
    2026-08-30 the operator redrew the whole field on the satellite map and the
    drawn polygon now REACHES FURTHER NORTH than the rules rectangle (N 118 vs
    N 115) while stopping ~10 m short of it in the south (N 54 vs N 44) — the
    two no longer nest either way, and the polygon he drew is the one flown.
    ⚠ The strip N 44-54 of the rules polygon is therefore NOT swept.
    """
    cfg = _kmitl()
    sc = cfg["search"]
    lat0, lon0 = (float(v) for v in cfg["ground_operation"]["launch_recovery"])
    k = _R * math.cos(math.radians(lat0))

    def enu(p: tuple[float, float]) -> tuple[float, float]:
        return (math.radians(p[1] - lon0) * k, math.radians(p[0] - lat0) * _R)

    area = [enu((float(v[0]), float(v[1]))) for v in cfg["search_area"]]
    keepouts = [[enu((float(v[0]), float(v[1]))) for v in poly]
                for poly in cfg["routing"]["keepout_zones"]]
    pts = [tuple(map(float, p)) for p in sc["sweep_waypoints_enu"]]
    legs = list(zip(pts, pts[1:]))          # every flown segment sees the ground
    # usable half-swath at the CONFIGURED sweep altitude: the measured 74.2 deg
    # lens minus ~1.3 m so a 1 m pad sits wholly inside the frame
    fov = math.radians(float(cfg["cameras"]["nadir"]["fov_deg"]))
    usable = float(sc["sweep_alt_m"]) * math.tan(fov / 2.0) - 1.3
    grid = []
    es = [v[0] for v in area]
    ns = [v[1] for v in area]
    for e in range(int(min(es)) + 1, int(max(es)), 1):
        for n in range(int(min(ns)) + 1, int(max(ns)), 1):
            if not _point_in_polygon(e, n, area):
                continue
            # a pad within _KEEPOUT_MARGIN_M of a band is refused anyway
            if any(min(x for x, _ in ko) - 3.0 <= e <= max(x for x, _ in ko) + 3.0
                   and min(y for _, y in ko) - 3.0 <= n <= max(y for _, y in ko) + 3.0
                   for ko in keepouts):
                continue
            grid.append((e, n))
    return grid, legs, keepouts, usable


def _seg_dist(p, a, b):
    L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    if L2 == 0.0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / L2))
    return math.hypot(p[0] - (a[0] + t * (b[0] - a[0])), p[1] - (a[1] + t * (b[1] - a[1])))


def test_the_kmitl_sweep_covers_the_whole_rules_polygon() -> None:
    """Every square metre of free ground inside the FLOWN polygon is swept.

    2026-08-30: the operator redrew the search area and the no-fly bands on the
    satellite map the morning of the scored day, so the polygon this grids over
    is now his YELLOW loop (config search_area), not the rules rectangle.

    Two flights taught this. 2026-08-28 noon: a 30 m seam between two legs (one
    swath, zero overlap) left 6 % of the area unseen. 2026-08-29, scored flight
    1: marker 4 was found at ENU (187.0, 72.1) — OUTSIDE the drawn L — and the
    17-waypoint sweep of that morning still left 160 m² (2.2 %) of the rules
    polygon unseen, 65 m² of it at the bottom of the pocket where a pad can
    sit and 62 m² under the trees. The redrawn sweep leaves under 0.2 %.
    """
    grid, legs, _keepouts, usable = _kmitl_sweep_geometry()
    assert len(grid) > 7000, "the drawn polygon minus the bands is ~8000 m²"
    unseen = [p for p in grid
              if min(_seg_dist(p, a, b) for a, b in legs) > usable]
    assert len(unseen) / len(grid) < 0.002, (
        f"{len(unseen)}/{len(grid)} m² of the rules polygon unseen at a "
        f"{usable:.1f} m usable half-swath: {unseen[:10]}")


def test_the_pocket_between_the_building_and_the_courtyard_is_swept_to_its_south_end() -> None:
    """Marker 4 sat in the pocket at ENU (187.0, 72.1) on 2026-08-29 — the free
    gap between the building band (E <= 180.1) and the courtyard (E >= 192.1).
    The pass that found it turned at N 64, leaving the pocket's south end
    unseen. Pin the whole pocket, N 46 to N 72."""
    _grid, legs, _keepouts, usable = _kmitl_sweep_geometry()
    # 2026-08-30: the operator's redrawn band puts the pocket's south end at
    # N 59 (it was N 46 against the 29-Aug boxes), so N 60-72 IS the pocket.
    for n in range(60, 73, 2):
        p = (186.0, float(n))
        d = min(_seg_dist(p, a, b) for a, b in legs)
        assert d <= usable, f"pocket point ENU (186, {n}) is {d:.1f} m from the nearest leg"


def test_no_sweep_leg_comes_within_10_m_of_the_18_m_tree_block() -> None:
    """The trees are 18 m and the sweep flies at 15 m — BELOW the canopy — so
    horizontal margin is the only margin there is, and a no-RTK fix is worth
    ±1-2 m on its own. The 2026-08-29-morning layout passed 5.0 m from the box
    (the E 102 tree-side pass and the N 66 transition). Keep 10 m."""
    _grid, legs, keepouts, _usable = _kmitl_sweep_geometry()
    trees = min(keepouts, key=lambda ko: min(x for x, _ in ko))
    # the tree block is the westernmost band; guard the premise
    x0, x1 = min(x for x, _ in trees), max(x for x, _ in trees)
    y0, y1 = min(y for _, y in trees), max(y for _, y in trees)
    assert 75 < x0 < 85 and 65 < y0 < 75, (x0, y0, "not the tree block")

    def clearance(a, b):
        best = math.inf
        for i in range(101):
            t = i / 100.0
            px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            dx = max(x0 - px, 0.0, px - x1)
            dy = max(y0 - py, 0.0, py - y1)
            best = min(best, math.hypot(dx, dy))
        return best

    worst = min(clearance(a, b) for a, b in legs)
    assert worst >= 9.9, f"a sweep leg passes {worst:.1f} m from the 18 m tree block"


# ── two gateways chained around two boxes ──────────────────────────────────

def test_a_hop_needing_two_gateways_chains_them(monkeypatch) -> None:
    """P3 → PAD5 blocked by BOX; the direct line from GATE to PAD5 is blocked
    by a second box, so the only clear chain is P3 → GATE → GATE2 → PAD5."""
    p3 = (TRANSIT[-1].lat, TRANSIT[-1].lon)
    box2 = [(13.7308, 100.78925), (13.7308, 100.78935),
            (13.73095, 100.78935), (13.73095, 100.78925)]
    gate2 = (13.7311, 100.7896)
    # premises: GATE→PAD5 and P3→gate2 are blocked, GATE→gate2→PAD5 is clear
    assert _segment_crosses_polygon(GATE, PAD5, box2)
    assert _segment_crosses_polygon(p3, gate2, BOX)
    for a, b in ((p3, GATE), (GATE, gate2), (gate2, PAD5)):
        assert not _segment_crosses_polygon(a, b, BOX) and not _segment_crosses_polygon(a, b, box2)
    state = _state()
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION,
        keepout_zones=[BOX, box2], gateways=[GATE, gate2]))
    pts = _pts(cmd)
    pad = (round(PAD5[0], 6), round(PAD5[1], 6))
    g1 = (round(GATE[0], 6), round(GATE[1], 6))
    g2 = (round(gate2[0], 6), round(gate2[1], 6))
    i_pad = pts.index(pad)
    assert pts[i_pad - 2:i_pad] == [g1, g2], pts[max(0, i_pad - 4):i_pad + 1]
    assert state.dropped_stops == {0}


# ── battery: cost-aware gate, no retry, abort hook ─────────────────────────

def test_a_delivery_is_refused_below_floor_plus_cost(monkeypatch) -> None:
    """36 % read the trial into a descent that ended at 20 %: a delivery may
    only start when the pack can pay ~12 points and still sit above the 30 %
    egress floor afterwards."""
    state = _state()
    state.telemetry.battery_percent = 38.0
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    serve = _fake_serve(state)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = RecordingCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION))
    assert serve.calls == []
    assert state.dropped_stops == set()
    assert any("BATTERY EGRESS" in e and "delivery cost" in e for e in state.anomalies)


def test_no_retry_and_abort_hook_once_the_floor_is_crossed(monkeypatch) -> None:
    """The serve gets the planned-egress test as ``abort_if``; if the first
    attempt comes back undropped with the pack under the floor there is no
    second attempt."""
    state = _state()
    state.telemetry.battery_percent = 60.0
    tracker = TargetTracker()
    _preload_pad(tracker, PAD5, 5)
    calls = []

    async def serve(commander, st, target, *, stop_index, params, **kw):
        calls.append(kw.get("abort_if"))
        st.telemetry.battery_percent = 25.0            # drained during the approach
        return ta_mod.AlignResult(acquired=True, aligned=True, landed=False,
                                  dropped=False, final_error_m=0.4)
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", serve)
    cmd = RecordingCommander(state)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, _spec(), home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION))
    assert len(calls) == 1, "no retry under the egress floor"
    assert callable(calls[0]) and calls[0]() is True     # the hook reads the live pack
    assert any("no retry" in e for e in state.anomalies)


# ── the sweep flies its own cruise speed ───────────────────────────────────

def test_the_sweep_sets_its_cruise_and_hands_the_pin_back(monkeypatch) -> None:
    from mission_brain.search_pattern import build_search_pattern
    from tests.test_delivery_mission import SEARCH_AREA
    state = _state()
    tracker = TargetTracker()                     # nothing known → the sweep flies
    monkeypatch.setattr(mission_mod, "acquire_and_land_drop", _fake_serve(state))
    cmd = RecordingCommander(state)
    spec = build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0, speed_mps=3.5)
    asyncio.run(run_delivery_mission(
        cmd, state, tracker, spec, home=HOME, transit_route=TRANSIT,
        sortie_gate=_gate_from([5]), profile=COMPETITION, cruise_pin_mps=5.0))
    sets = [v for n, v in cmd.params if n == "MPC_XY_CRUISE"]
    assert sets[:1] == [3.5] and sets[-1] == 5.0, cmd.params
    i_set = cmd.events.index("param:MPC_XY_CRUISE=3.5")
    first_sweep_goto = next(i for i, e in enumerate(cmd.events)
                            if e == "goto@12.0")
    assert i_set < first_sweep_goto


# ── the COMPANION's return-to-home routes around the bands ──────────────────

def _rth_commander(vias_from):
    """A DroneCommander with only the pieces rth() touches, built the way the
    field does: a return_route_provider plus a stubbed MAVSDK surface."""
    import asyncio as _asyncio

    from mavlink_adapter.commands import DroneCommander

    class _Act:
        def __init__(self):
            self.rtl_calls = 0
            self.disarm_calls = 0

        async def return_to_launch(self):
            self.rtl_calls += 1

        async def disarm(self):
            self.disarm_calls += 1

    class _Param:
        async def set_param_float(self, name, v):
            pass

    act = _Act()
    c = DroneCommander.__new__(DroneCommander)
    c.system = type("_Sys", (), {"action": act, "param": _Param()})()
    c.gotos = []
    c.return_route_provider = vias_from
    c._rth_home = (HOME.lat, HOME.lon)
    c.rtl_return_alt_m = 25.0
    c.rth_route_detour_max = 2.0

    async def _pos():
        return (PAD5[0], PAD5[1]) if not c.gotos else c.gotos[-1][:2]

    async def _goto(lat, lon, alt, yaw_deg=float("nan")):
        c.gotos.append((lat, lon, alt))

    async def _arrive(lat, lon, *, timeout_s, radius_m=3.0):
        return True

    async def _climbed(target_m, tolerance_m=None, timeout_s=60.0):
        return True

    async def _landed(**kw):
        return True

    async def _disarmed(**kw):
        return True
    c._current_position = _pos
    c._wait_until_altitude_reached = _climbed
    c.goto = _goto
    c._wait_arrival = _arrive
    c._wait_until_landed = _landed
    c._wait_until_disarmed = _disarmed
    _asyncio.run(c.rth())
    return c, act


def test_the_companion_rth_flies_the_gateway_chain_before_px4_takes_over() -> None:
    """Operator, 2026-08-29: "ถ้า RTL ก็ช่วยทำให้มันหลบด้วยได้ไหม". PX4's RTL is
    a straight line home and knows nothing about the bands drawn to keep the
    scan off the trees and the building. When a clear chain exists, the
    companion flies it first — at RTL altitude, so the vertical margin PX4
    would have given is not given up — and only then hands over."""
    c, act = _rth_commander(
        lambda lat, lon: gateway_route((lat, lon), (HOME.lat, HOME.lon),
                                       [BOX], [GATE]))
    # climb IN PLACE first (PX4's RTL does; a goto climbs and translates at the
    # same time, which from a 2 m rung would cross the ground below the return
    # altitude — and the bands are 18 m trees and a building), then the chain.
    assert [g[:2] for g in c.gotos] == [(PAD5[0], PAD5[1]), GATE], c.gotos
    assert all(g[2] == 25.0 for g in c.gotos), "vias must be flown at RTL altitude"
    assert act.rtl_calls == 1, "PX4 RTL still flies the last leg and lands"


def test_the_rth_flies_straight_when_the_line_home_is_already_clear() -> None:
    """No band in the way, no detour: the routing must not add waypoints to an
    emergency that did not need them."""
    c, act = _rth_commander(lambda lat, lon: [])
    assert c.gotos == []
    assert act.rtl_calls == 1


def test_an_absurd_detour_is_refused_and_the_rth_flies_straight() -> None:
    """An RTH is already an emergency and the pack may be the reason for it, so
    a chain that more than doubles the distance home is not worth the bands:
    PX4's straight line still climbs above them. Fails OPEN, like every other
    branch of this helper."""
    far = (HOME.lat + 0.02, HOME.lon + 0.02)      # ~2.9 km out of the way
    c, act = _rth_commander(lambda lat, lon: [far])
    assert c.gotos == [], "the detour cap did not fire"
    assert act.rtl_calls == 1


def test_a_provider_that_raises_never_blocks_the_return() -> None:
    def boom(lat, lon):
        raise RuntimeError("router exploded")
    c, act = _rth_commander(boom)
    assert c.gotos == []
    assert act.rtl_calls == 1, "a broken router must not cost the aircraft its RTL"
