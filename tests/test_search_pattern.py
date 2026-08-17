"""Boustrophedon search pattern (mission_brain.search_pattern).

The blind-search front half of the mission turns the controlled-airspace polygon
into a coverage sweep. These lock the invariants the flight depends on: enough
overlapping legs to cover the inset field, every waypoint inside the fence, a
snaking (boustrophedon) order, and the ceiling clamp.
"""

from __future__ import annotations

import math

from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.safety import _point_in_polygon

# The real AAVC controlled airspace (ENU bbox x=-5..235, y=-35..35 about the
# site centre) and a home near the SW corner, as in sitl/aavc_config.yaml.
GEOFENCE = [
    [14.6521856, 101.1874536],   # ENU (-5, -35)  SW
    [14.6521856, 101.1896820],   # ENU (235, -35) SE
    [14.6528144, 101.1896820],   # ENU (235,  35) NE
    [14.6528144, 101.1874536],   # ENU (-5,  35)  NW
]
HOME = Coordinate(lat=14.65235, lon=101.18748)
_R = 6_378_137.0


def _centroid(geo):
    return sum(v[0] for v in geo) / len(geo), sum(v[1] for v in geo) / len(geo)


def _enu(lat, lon, lat0, lon0):
    e = math.radians(lon - lon0) * _R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * _R
    return e, n


def test_three_legs_cover_the_field_at_16m() -> None:
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0, overlap_frac=0.3)
    # 4 legs since the MEASURED 74.2° lens (2026-08-17): swath 24.2 m at 16 m
    # vs the old placeholder's 38.1 m — narrower lens, tighter legs, same field.
    assert spec.leg_count == 4
    assert len(spec.waypoints) == 2 * spec.leg_count
    # Swath = 2·alt·tan(fov/2); spacing leaves the 30% overlap.
    assert math.isclose(spec.swath_m, 2 * 16.0 * math.tan(1.295 / 2), rel_tol=1e-6)
    assert math.isclose(spec.spacing_m, spec.swath_m * 0.7, rel_tol=1e-6)
    assert spec.est_duration_s > 0.0


def test_legs_span_the_inset_cross_axis() -> None:
    """leg_count·spacing must cover the inset field width (no uncovered strip)."""
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0, overlap_frac=0.3,
                                margin_m=8.0)
    inset_cross = (35.0 - 8.0) - (-35.0 + 8.0)   # y span after 8 m inset = 54 m
    assert spec.leg_count * spec.spacing_m >= inset_cross


def test_all_waypoints_inside_the_geofence() -> None:
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0)
    for wp in spec.waypoints:
        assert _point_in_polygon(wp.lat, wp.lon, [list(v) for v in GEOFENCE])
        assert wp.alt_m == spec.sweep_alt_m


def test_boustrophedon_snake_order() -> None:
    """Consecutive legs reverse: the connector between leg k and k+1 is short
    (a cross-step), not a full diagonal back to the same end."""
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0)
    lat0, lon0 = _centroid(GEOFENCE)
    enu = [_enu(w.lat, w.lon, lat0, lon0) for w in spec.waypoints]
    # Waypoints come in (start,end) pairs per leg. The end of leg k and the start
    # of leg k+1 share the along-axis end → their separation ≈ one leg spacing,
    # far less than a leg's length.
    leg_len = math.hypot(enu[1][0] - enu[0][0], enu[1][1] - enu[0][1])
    connector = math.hypot(enu[2][0] - enu[1][0], enu[2][1] - enu[1][1])
    assert connector < 0.5 * leg_len


def test_sweep_starts_nearest_home() -> None:
    """First waypoint is the field corner closest to home (least transit in)."""
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0)
    lat0, lon0 = _centroid(GEOFENCE)
    he, hn = _enu(HOME.lat, HOME.lon, lat0, lon0)
    first = _enu(spec.waypoints[0].lat, spec.waypoints[0].lon, lat0, lon0)
    last = _enu(spec.waypoints[-1].lat, spec.waypoints[-1].lon, lat0, lon0)
    d_first = math.hypot(first[0] - he, first[1] - hn)
    d_last = math.hypot(last[0] - he, last[1] - hn)
    assert d_first < d_last


def test_sweep_altitude_clamped_under_ceiling() -> None:
    spec = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=25.0, ceiling_m=20.0)
    assert spec.sweep_alt_m == 19.0
    assert all(wp.alt_m == 19.0 for wp in spec.waypoints)


def test_lower_sweep_alt_needs_more_legs() -> None:
    """A lower sweep → smaller swath → more legs (still covers the field)."""
    hi = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0)
    lo = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=10.0)
    assert lo.leg_count >= hi.leg_count
    assert lo.swath_m < hi.swath_m


# ── axis_deg: rotated-field sweep (KMUTNB sky-field, pitch axis 143.8°) ──────
# The KMUTNB search polygon is a rectangle rotated ~36° off ENU. The legacy
# grid sweeps the polygon's ENU bounding box, whose corners fall OUTSIDE such
# a polygon (and the airspace around it); with axis_deg the identical grid
# logic runs in the field frame instead. These use the REAL config polygons.

def _kmutnb_cfg():
    from pathlib import Path

    import yaml
    repo = Path(__file__).resolve().parents[1]
    return yaml.safe_load((repo / "sitl" / "aavc_config.yaml").read_text())


def test_axis_deg_keeps_every_waypoint_inside_the_rotated_airspace() -> None:
    cfg = _kmutnb_cfg()
    sc = cfg["search"]
    home = Coordinate(lat=cfg["site"]["center_lat"],
                      lon=cfg["site"]["center_lon"])
    spec = build_search_pattern(
        cfg["search_area"], home,
        sweep_alt_m=float(sc["sweep_alt_m"]),
        overlap_frac=float(sc["overlap_frac"]),
        margin_m=float(sc["margin_m"]),
        speed_mps=float(sc["speed_mps"]),
        ceiling_m=float(cfg["mission"]["altitude_ceiling_m"]),
        axis_deg=float(sc["sweep_axis_deg"]),
    )
    airspace = [(float(v[0]), float(v[1])) for v in cfg["controlled_airspace"]]
    for wp in spec.waypoints:
        assert _point_in_polygon(wp.lat, wp.lon, airspace), (
            f"waypoint {wp.lat:.7f},{wp.lon:.7f} escaped the rotated airspace")


def test_without_axis_deg_the_rotated_field_sweep_would_breach() -> None:
    """Documents WHY axis_deg exists: on the rotated KMUTNB polygon the legacy
    ENU-bbox sweep provably leaves the airspace — if this ever starts passing,
    the fields aren't rotated any more and axis_deg can be retired."""
    cfg = _kmutnb_cfg()
    home = Coordinate(lat=cfg["site"]["center_lat"],
                      lon=cfg["site"]["center_lon"])
    spec = build_search_pattern(
        cfg["search_area"], home, sweep_alt_m=4.0, overlap_frac=0.4,
        margin_m=4.0, speed_mps=3.0, ceiling_m=5.0,
    )
    airspace = [(float(v[0]), float(v[1])) for v in cfg["controlled_airspace"]]
    escaped = sum(
        0 if _point_in_polygon(wp.lat, wp.lon, airspace) else 1
        for wp in spec.waypoints)
    assert escaped > 0


def test_axis_deg_none_is_bit_for_bit_legacy() -> None:
    """axis_deg=None must not perturb the existing ENU-aligned behaviour."""
    a = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0, overlap_frac=0.3)
    b = build_search_pattern(GEOFENCE, HOME, sweep_alt_m=16.0, overlap_frac=0.3,
                             axis_deg=None)
    assert [(w.lat, w.lon, w.alt_m) for w in a.waypoints] == \
           [(w.lat, w.lon, w.alt_m) for w in b.waypoints]
