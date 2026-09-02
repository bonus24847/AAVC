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
from vision.projection import CameraModel

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


# ── the heading the sweep holds (2026-08-22) ───────────────────────────────
#
# Field root cause, from ULog 08_11_09 of 2026-08-20: every goto passed a NaN
# yaw, so PX4 fell through to MPC_YAW_MODE — factory 0, "towards waypoint" —
# and turned the nose at each sweep waypoint. The commanded heading walked
# 145->119->94->69->44->18->353->... through a full circle at the 25 deg/s cap:
# 867 deg of yaw in 122 s. The camera is bolted to the body, so 1 of 457
# recorded frames decoded. The sweep now names the heading it wants.

def test_sweep_holds_a_heading_that_puts_the_wide_axis_across_the_legs() -> None:
    """Wide (1280 px) axis across track <=> nose along the legs, at mount 0."""
    area = [(13.731239, 100.787824), (13.731359, 100.789916),
            (13.730703, 100.789776), (13.730723, 100.787840)]
    home = Coordinate(lat=13.730322, lon=100.787446)
    spec = build_search_pattern(area, home, sweep_alt_m=12.0, overlap_frac=0.44,
                                margin_m=5.0, speed_mps=3.0, ceiling_m=20.0,
                                axis_deg=87.0)
    assert abs(spec.leg_bearing_deg - 87.0) < 1e-6
    assert abs(spec.sweep_yaw_deg - 87.0) < 1e-6


def test_a_rotated_camera_mount_turns_the_held_heading_with_it() -> None:
    """The heading is derived, not assumed: bolt the camera 90 deg round and
    the aircraft must fly the legs sideways to keep the same ground footprint.

    A 180 deg mount is the exception, and deliberately so: `leg_bearing - 180`
    and `leg_bearing` BOTH lay the wide axis across track (the footprint is a
    rectangle), so the plan takes the nose-first one and the held heading comes
    back to the leg bearing itself. See
    ``test_the_held_heading_prefers_flying_the_leg_nose_first``.
    """
    area = [(13.731239, 100.787824), (13.731359, 100.789916),
            (13.730703, 100.789776), (13.730723, 100.787840)]
    home = Coordinate(lat=13.730322, lon=100.787446)
    for mount, want in ((90.0, 357.0), (180.0, 87.0), (270.0, 177.0)):
        cam = CameraModel(name="nadir", mount_yaw_rad=math.radians(mount))
        spec = build_search_pattern(area, home, sweep_alt_m=12.0,
                                    overlap_frac=0.44, margin_m=5.0,
                                    speed_mps=3.0, camera=cam, ceiling_m=20.0,
                                    axis_deg=87.0)
        assert abs(spec.sweep_yaw_deg - want) < 1e-6, f"mount {mount}"


def _wrap180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def test_the_held_heading_prefers_flying_the_leg_nose_first() -> None:
    """Two headings put the wide axis across track and they are 180 deg apart.
    Whichever mount the camera is bolted at, the plan must pick the one that
    flies the leg forwards — and must still put the wide axis across track,
    which is the whole reason the heading is derived at all.

    The 180 deg mount MEASURED on this aircraft is what makes this matter: the
    bare formula would fly every sweep leg backwards. A 90 deg mount has no
    nose-first option (both perpendiculars are 90 deg off the leg), so the
    guarantee there is only "no worse than 90".
    """
    area = [(13.731239, 100.787824), (13.731359, 100.789916),
            (13.730703, 100.789776), (13.730723, 100.787840)]
    home = Coordinate(lat=13.730322, lon=100.787446)
    for mount in range(0, 360, 10):
        cam = CameraModel(name="nadir", mount_yaw_rad=math.radians(mount))
        spec = build_search_pattern(area, home, sweep_alt_m=12.0,
                                    overlap_frac=0.44, margin_m=5.0,
                                    speed_mps=3.0, camera=cam, ceiling_m=20.0,
                                    axis_deg=87.0)
        off_leg = abs(_wrap180(spec.sweep_yaw_deg - spec.leg_bearing_deg))
        assert off_leg <= 90.0 + 1e-6, (
            f"mount {mount}: heading {spec.sweep_yaw_deg:.0f} flies the "
            f"{spec.leg_bearing_deg:.0f} deg leg {off_leg:.0f} deg off — "
            "backwards when it did not have to be")
        # …and the flip must not have cost the coverage it exists to protect:
        # the camera's WIDE image axis lies at body bearing 90 + mount, so in
        # the world it points at heading + 90 + mount, and that must be square
        # across the leg — swath_m assumes exactly this.
        wide_axis = spec.sweep_yaw_deg + 90.0 + mount
        assert abs(abs(_wrap180(wide_axis - spec.leg_bearing_deg)) - 90.0) < 1e-6


def test_an_enu_aligned_polygon_still_names_its_leg_bearing() -> None:
    """No axis_deg: legs run along the longer bbox side — East or North."""
    wide = [(13.8000, 100.5000), (13.8000, 100.5030),
            (13.8004, 100.5030), (13.8004, 100.5000)]     # ~325 m E x 44 m N
    spec = build_search_pattern(wide, Coordinate(lat=13.8000, lon=100.5000),
                                sweep_alt_m=12.0, overlap_frac=0.44,
                                margin_m=5.0, speed_mps=3.0, ceiling_m=20.0)
    assert abs(spec.leg_bearing_deg - 90.0) < 1e-6      # legs run East
    assert abs(spec.sweep_yaw_deg - 90.0) < 1e-6


def test_overlap_044_makes_the_strips_survive_a_sideways_camera() -> None:
    """The reason the shipped configs moved 0.30 -> 0.44: at 0.30 the legs are
    spaced at 70% of the WIDE axis, which is only the cross-track footprint if
    that axis is across track. If it is not, the real swath is the 720 px axis
    and a 1 m pad fits inside the gap with room to spare."""
    area = [(13.731239, 100.787824), (13.731359, 100.789916),
            (13.730703, 100.789776), (13.730723, 100.787840)]
    home = Coordinate(lat=13.730322, lon=100.787446)
    # The REAL sensor, not the module default: the gap is a property of the
    # 16:9 shape (720/1280 = 0.5625, so anything looser than 0.4375 can leave
    # one). The 640x480 default is 4:3 and would hide this entirely.
    cam = CameraModel(name="ov9281", width_px=1280, height_px=720)
    kw = dict(sweep_alt_m=12.0, margin_m=5.0, speed_mps=3.0, ceiling_m=20.0,
              axis_deg=87.0, camera=cam)
    narrow = cam.height_px / cam.fx_px * 12.0           # 720 px axis on ground
    loose = build_search_pattern(area, home, overlap_frac=0.30, **kw)
    tight = build_search_pattern(area, home, overlap_frac=0.44, **kw)
    assert loose.spacing_m - narrow > 1.0, "0.30 leaves a pad-sized gap"
    assert tight.spacing_m <= narrow + 1e-6, "0.44 must close it at any mount"
    assert tight.leg_count > loose.leg_count             # paid for in legs
