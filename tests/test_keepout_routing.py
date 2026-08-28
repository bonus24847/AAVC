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
from orchestrator.main import _build_spec
from orchestrator.mission import (
    _point_in_polygon,
    _point_near_polygon_m,
    _segment_crosses_polygon,
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
    gw = tuple(float(v) for v in cfg["routing"]["gateway"])
    p1, p2, p3 = [tuple(float(x) for x in v) for v in cfg["transit_route"]]
    wps = [(w.lat, w.lon) for w in spec.waypoints]
    for w in wps:
        assert _point_in_polygon(w[0], w[1], airspace), f"waypoint {w} outside the airspace"
        for poly in keepouts:
            assert not _point_near_polygon_m(w[0], w[1], poly, 5.0), f"waypoint {w} hugs a band"
    segments = list(zip(wps, wps[1:])) + [(p1, p2), (p2, p3), (p3, wps[0]), (wps[-1], p3)]
    segments += [(gw, w) for w in wps] + [(gw, p3)]
    for a, b in segments:
        for poly in keepouts:
            assert not _segment_crosses_polygon(a, b, poly), f"{a}->{b} crosses a band"
    assert _point_in_polygon(gw[0], gw[1], airspace)


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
        keepout_zones=[BOX], gateway=GATE))

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
        keepout_zones=[around_pad], gateway=GATE))
    assert state.dropped_stops == set()
    assert serve.calls == []
    assert any("REFUSED" in e and "keep-out" in e for e in state.anomalies)
    assert (round(PAD5[0], 6), round(PAD5[1], 6)) not in _pts(cmd)


def test_the_kmitl_sweep_covers_the_L_with_overlap() -> None:
    """The operator's 28-Aug-noon call: the first cut had two legs 30 m apart —
    exactly one 20 m swath, zero overlap — and a 2 m grid of the L showed 6 %
    of the area unseen at a realistic 13 m usable half-swath (a pad must sit
    wholly inside the frame), all along the seam. Pin the fix: every grid
    point of the flown search polygon that is outside the keep-outs lies
    within 13 m of some leg (< 1 % unseen; points within 3 m of a band are
    left out — a candidate there is refused as a false hit anyway)."""
    cfg = _kmitl()
    sc = cfg["search"]
    lat0, lon0 = (float(v) for v in cfg["ground_operation"]["launch_recovery"])
    k = _R * math.cos(math.radians(lat0))

    def enu(p: tuple[float, float]) -> tuple[float, float]:
        return (math.radians(p[1] - lon0) * k, math.radians(p[0] - lat0) * _R)
    area = [enu((float(v[0]), float(v[1]))) for v in cfg["search_area"]]
    keepouts = [[enu((float(v[0]), float(v[1]))) for v in poly]
                for poly in cfg["routing"]["keepout_zones"]]
    legs = [(tuple(map(float, a)), tuple(map(float, b)))
            for a, b in zip(sc["sweep_waypoints_enu"][0::2], sc["sweep_waypoints_enu"][1::2])]
    assert len(legs) >= 4, "four legs since the noon fix — three left a 30 m seam"

    def seg_dist(p, a, b):
        L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        t = max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / L2))
        return math.hypot(p[0] - (a[0] + t * (b[0] - a[0])), p[1] - (a[1] + t * (b[1] - a[1])))
    es = [v[0] for v in area]
    ns = [v[1] for v in area]
    total = 0
    unseen = []
    for e in range(int(min(es)) + 2, int(max(es)), 2):
        for n in range(int(min(ns)) + 2, int(max(ns)), 2):
            if not _point_in_polygon(e, n, area):
                continue
            # a pad within _KEEPOUT_MARGIN_M of a band is refused anyway
            if any(min(x for x, _ in ko) - 3.0 <= e <= max(x for x, _ in ko) + 3.0
                   and min(y for _, y in ko) - 3.0 <= n <= max(y for _, y in ko) + 3.0
                   for ko in keepouts):
                continue
            total += 1
            if min(seg_dist((e, n), a, b) for a, b in legs) > 13.0:
                unseen.append((e, n))
    assert total > 1500
    assert len(unseen) / total < 0.01, f"{len(unseen)}/{total} grid points unseen: {unseen[:10]}"
    # and no two consecutive parallel legs further apart than the validated
    # 0.30-overlap spacing (21.2 m) plus a metre of slack
    for (a1, b1), (a2, b2) in zip(legs, legs[1:]):
        assert abs((a2[1] + b2[1]) / 2 - (a1[1] + b1[1]) / 2) <= 22.5
