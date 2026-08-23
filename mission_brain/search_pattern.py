"""Boustrophedon (lawnmower) search pattern over the field — the blind-search
front half of the mission.

The competition does NOT hand us the target coordinates; the drone must find the
landing pads itself. This module turns the search-area polygon
into a coverage sweep: inset the polygon, lay parallel legs spaced so the nadir
camera's ground swath overlaps, and order them boustrophedon (snaking) from the
end nearest home. The mission loop (``orchestrator.mission``) flies these legs,
watching both cameras; ``orchestrator.target_tracker`` confirms what they see.

Pure geometry (flat-earth ENU about the polygon centroid) — no telemetry, no
MAVSDK — so it is trivially unit-testable and deterministic. Design reference:
coverage path planning (CLAUDE.md §9 E).

Mounting assumption: the camera's WIDE image axis (the horizontal FOV) is across
track, so the guaranteed cross-track swath is ``2·alt·tan(fov/2)``. Validate the
real footprint in SITL (G3) and tune ``sweep_alt_m`` / ``overlap_frac`` if needed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vision.projection import NADIR, CameraModel

from .schemas import Coordinate

_R_EARTH_M = 6_378_137.0


@dataclass(frozen=True)
class SearchPlanSpec:
    """A coverage sweep ready to fly. ``waypoints`` are in boustrophedon order,
    each at ``sweep_alt_m`` AGL; the rest are diagnostics for the dashboard /
    time budget."""

    waypoints: list[Coordinate]
    leg_count: int
    sweep_alt_m: float
    speed_mps: float
    swath_m: float
    spacing_m: float
    est_duration_s: float
    # True bearing the legs run along, and the heading to HOLD while flying
    # them so the camera's wide (1280 px) image axis lies ACROSS track — the
    # orientation ``swath_m`` above assumes. See the sweep-heading note in
    # ``build_search_pattern``. ``sweep_yaw_deg`` is what the mission passes to
    # every search-phase goto.
    leg_bearing_deg: float = 0.0
    sweep_yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        # A non-positive or NaN speed makes the mission's leg/RTH timeouts
        # (2·dist / speed) collapse to ~0 or NaN — fail fast at construction
        # rather than letting a mis-configured sweep fly with broken timing.
        if not (self.speed_mps > 0.0):
            raise ValueError(f"speed_mps must be > 0, got {self.speed_mps!r}")


def _enu(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    east = math.radians(lon - lon0) * _R_EARTH_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * _R_EARTH_M
    return east, north


def _latlon(east: float, north: float, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + math.degrees(north / _R_EARTH_M)
    lon = lon0 + math.degrees(east / (_R_EARTH_M * math.cos(math.radians(lat0))))
    return lat, lon


def build_search_pattern(
    geofence: list[tuple[float, ...]] | list[list[float]],
    home: Coordinate,
    *,
    sweep_alt_m: float = 12.0,   # matches config search.sweep_alt_m (all callers pass it)
    overlap_frac: float = 0.3,
    margin_m: float = 8.0,
    speed_mps: float = 12.0,
    camera: CameraModel = NADIR,
    ceiling_m: float = 20.0,
    turn_penalty_s: float = 3.0,
    axis_deg: float | None = None,
) -> SearchPlanSpec:
    """Lay a boustrophedon sweep covering the inset ``geofence`` polygon.

    Legs run along the polygon's LONG axis (fewer turns), spaced ``swath·(1−overlap)``
    so adjacent strips overlap; the sweep starts from the corner nearest ``home``.
    ``sweep_alt_m`` is clamped to ``ceiling_m − 1``. Raises ``ValueError`` for a
    degenerate polygon (< 3 vertices).

    ``axis_deg`` (config ``search.sweep_axis_deg``): true heading of the field's
    long axis for a search polygon that is NOT axis-aligned in ENU. The sweep
    grid below covers the polygon's axis-aligned BOUNDING BOX — fine for the
    KMITL field whose rectangles hug ENU, but a rotated rectangle (KMUTNB: the
    pitch bears ~144°) has bbox corners far OUTSIDE the polygon, i.e. outside
    the geofence. With ``axis_deg`` set, every vertex (and home) is rotated
    into the field frame first, the identical grid logic runs there, and the
    waypoints are rotated back before the lat/lon conversion — so each leg
    runs along the true field axis and stays inside the polygon (+margin).
    ``None`` keeps the legacy ENU-aligned behaviour bit-for-bit.
    """
    verts = [(float(v[0]), float(v[1])) for v in geofence]
    if len(verts) < 3:
        raise ValueError("search pattern needs a polygon of >= 3 vertices")

    alt = max(1.0, min(sweep_alt_m, ceiling_m - 1.0))

    # ENU about the polygon centroid (flat-earth; the AAVC field is sub-km).
    lat0 = sum(v[0] for v in verts) / len(verts)
    lon0 = sum(v[1] for v in verts) / len(verts)
    enu = [_enu(lat, lon, lat0, lon0) for lat, lon in verts]
    home_e, home_n = _enu(home.lat, home.lon, lat0, lon0)

    # Field-frame rotation (see docstring). u = along-axis unit vector,
    # v = u rotated +90° CCW; forward maps ENU -> (s, t), inverse restores ENU.
    if axis_deg is not None:
        _h = math.radians(float(axis_deg))
        _ux, _uy = math.sin(_h), math.cos(_h)
        enu = [(e * _ux + n * _uy, -e * _uy + n * _ux) for e, n in enu]
        home_e, home_n = (home_e * _ux + home_n * _uy,
                          -home_e * _uy + home_n * _ux)

    emin = min(e for e, _ in enu) + margin_m
    emax = max(e for e, _ in enu) - margin_m
    nmin = min(n for _, n in enu) + margin_m
    nmax = max(n for _, n in enu) - margin_m
    # Guard a margin that swallows the field — fall back to the un-inset bbox.
    if emax <= emin or nmax <= nmin:
        emin = min(e for e, _ in enu)
        emax = max(e for e, _ in enu)
        nmin = min(n for _, n in enu)
        nmax = max(n for _, n in enu)

    # Swath = nadir cross-track ground footprint; spacing leaves the overlap.
    swath = 2.0 * alt * math.tan(camera.fov_rad / 2.0)
    spacing = max(1.0, swath * (1.0 - overlap_frac))

    he, hn = home_e, home_n

    # Sweep along the longer span; step across the shorter one.
    sweep_is_east = (emax - emin) >= (nmax - nmin)
    if sweep_is_east:
        along_lo, along_hi, home_along = emin, emax, he
        cross_lo, cross_hi, home_cross = nmin, nmax, hn
    else:
        along_lo, along_hi, home_along = nmin, nmax, hn
        cross_lo, cross_hi, home_cross = emin, emax, he

    cross_span = cross_hi - cross_lo
    n_legs = max(1, math.ceil(cross_span / spacing)) if cross_span > 0 else 1
    # Leg centres evenly spread so each strip is <= spacing wide (>= the overlap).
    crosses = [cross_lo + (i + 0.5) * cross_span / n_legs for i in range(n_legs)]
    # Start from the side nearest home (cross + along).
    if home_cross > (cross_lo + cross_hi) / 2.0:
        crosses.reverse()
    first_forward = home_along <= (along_lo + along_hi) / 2.0

    enu_wps: list[tuple[float, float]] = []
    for k, cross in enumerate(crosses):
        forward = first_forward if (k % 2 == 0) else (not first_forward)
        a0, a1 = (along_lo, along_hi) if forward else (along_hi, along_lo)
        if sweep_is_east:
            enu_wps += [(a0, cross), (a1, cross)]
        else:
            enu_wps += [(cross, a0), (cross, a1)]

    # Undo the field-frame rotation before geodetic conversion (path length is
    # rotation-invariant, so it may be computed in either frame).
    if axis_deg is not None:
        enu_wps = [(s * _ux - t * _uy, s * _uy + t * _ux) for s, t in enu_wps]

    waypoints = [
        Coordinate(lat=lat, lon=lon, alt_m=alt)
        for lat, lon in (_latlon(e, n, lat0, lon0) for e, n in enu_wps)
    ]

    path_len = sum(
        math.hypot(enu_wps[i + 1][0] - enu_wps[i][0], enu_wps[i + 1][1] - enu_wps[i][1])
        for i in range(len(enu_wps) - 1)
    )
    est = path_len / max(speed_mps, 0.1) + turn_penalty_s * max(0, len(enu_wps) - 1)

    # ── the heading to hold while sweeping ──────────────────────────────────
    # ``swath`` above is the WIDE image axis on the ground, which is only the
    # cross-track footprint if that axis actually lies across the legs. The
    # camera is body-fixed with no gimbal, so that is decided by the heading
    # the aircraft holds — and until 2026-08-22 nothing chose one: every
    # ``goto`` passed a NaN yaw, so PX4 fell through to MPC_YAW_MODE. On the
    # 2026-08-20 flights that param sat at its factory 0 = "towards waypoint",
    # and the aircraft dutifully turned its nose to face each new sweep
    # waypoint: the commanded heading walked 145->119->94->69->44->18->353->…
    # through a full circle at the 25 deg/s cap, 867 deg of yaw in one 122 s
    # flight (ULog 08_11_09). A body-fixed camera spinning like that smears
    # every frame — 1 of 457 recorded frames decoded.
    # Holding this heading fixes both halves: the image stops rotating, and the
    # wide axis is deliberately placed across track instead of wherever the
    # aircraft happened to be pointing when it armed. A finite yaw in the goto
    # BEATS MPC_YAW_MODE in PX4 (FlightTaskAuto.cpp:496 takes the triplet yaw
    # before ever consulting the param), so this works whatever that param says.
    if axis_deg is None:
        leg_bearing = 90.0 if sweep_is_east else 0.0
    else:
        leg_bearing = float(axis_deg) if sweep_is_east else float(axis_deg) - 90.0
    # Wide axis across track <=> nose along the legs, offset by however the
    # camera is bolted (CameraModel.mount_yaw_rad; 0 = image-up at the nose).
    sweep_yaw = (leg_bearing - math.degrees(camera.mount_yaw_rad)) % 360.0
    # TWO headings put the wide axis across track. They are 180 deg apart, and
    # the footprint is a rectangle, so the ground covered is identical either
    # way — take the one that flies the leg NOSE-FIRST. Without this the 180 deg
    # mount measured on the aircraft 2026-08-23 (the camera is bolted upside
    # down) would send it down every sweep leg backwards: legal, identically
    # covered, and unlike every validated run — not a thing for a safety pilot
    # to meet for the first time in the air. At a 90 deg mount there is no
    # nose-first option (both perpendiculars sit 90 deg off the leg) and this
    # is correctly a no-op.
    if abs(((sweep_yaw - leg_bearing + 180.0) % 360.0) - 180.0) > 90.0:
        sweep_yaw = (sweep_yaw + 180.0) % 360.0

    return SearchPlanSpec(
        waypoints=waypoints,
        leg_count=n_legs,
        leg_bearing_deg=leg_bearing % 360.0,
        sweep_yaw_deg=sweep_yaw,
        sweep_alt_m=alt,
        speed_mps=speed_mps,
        swath_m=swath,
        spacing_m=spacing,
        est_duration_s=est,
    )
