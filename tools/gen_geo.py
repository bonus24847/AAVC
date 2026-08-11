#!/usr/bin/env python3
"""Single source of truth for the KMUTNB sky-field mission geometry.

The whole SITL stack keeps a hard invariant: aavc_config.yaml `site.center`
== world <spherical_coordinates> == launch_sitl.sh PX4_HOME_* ==
spawn_targets ENU origin == fetch_sat LAT0/LON0 == gcs/kmutnb_field.yaml
local_origin.  Every one of those numbers is generated HERE and pasted (or
imported) — never hand-derived, so the invariant cannot drift.

Frame definitions
-----------------
* Field centre C = centre circle of the KMUTNB rooftop football pitch
  (สนามฟุตบอลลอยฟ้า มจพ. บางซื่อ), measured off Google z20 imagery 2026-08-11.
* The pitch long axis points AXIS_DEG (143.8° true, NW goal -> SE goal).
  Field frame: s = metres along that axis (+ toward the SE goal),
  t = metres across it (+ toward the NE touchline).
* ENU: X = east, Y = north, origin = the Launch & Recovery point (which is
  also the world origin and PX4 home).  All mission ENU values are L&R-relative.
* The user-mandated operating area is the 60 x 44 m rotated rectangle around
  C (2,640 m^2, inside the 2,400-2,800 m^2 brief) — pitch only, no track.

Run `python3 tools/gen_geo.py` to print every derived block (config YAML,
world SDF includes, launcher exports, GCS field yaml, spawn baseline).
`fetch_sat.py` imports GEO from here instead of duplicating constants.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

# Import the ENU<->lat/lon conversion from spawn_targets so there is exactly
# one WGS84 formula in the project (it must match gz's EARTH_WGS84 frame).
_SITL = Path(__file__).resolve().parent.parent / "sitl"
sys.path.insert(0, str(_SITL))
from spawn_targets import _local_enu_to_latlon, _wgs84_m_per_deg  # noqa: E402

# ---------------------------------------------------------------------------
# Measured field anchors (Google z20 tiles, ~0.145 m/px, fetched 2026-08-11)
# ---------------------------------------------------------------------------
FIELD_CENTER_LAT = 13.822669   # pitch centre circle
FIELD_CENTER_LON = 100.512146
AXIS_DEG = 143.8               # pitch long-axis true heading (NW goal -> SE goal)
GROUND_ALT_M = 15.0            # rooftop pitch surface above street level

# Field-frame layout (s along axis from C, t across; metres) ----------------
AIRSPACE_S = (-30.0, 30.0)     # 60 m — user brief: field only, no track
AIRSPACE_T = (-22.0, 22.0)     # 44 m
SEARCH_S = (-28.0, 2.0)        # NW two-thirds: pads + boustrophedon sweep
SEARCH_T = (-20.0, 20.0)
LNR_ST = (24.0, 0.0)           # Launch & Recovery, on-axis near the SE edge
TRANSIT_ST = ((18.0, -6.0), (8.0, -10.0), (2.0, 0.0))  # P1, P2, P3 (ingress)

# Baseline pads: (marker_id, s, t, yaw_deg).  Two columns x three rows,
# min pairwise separation 14.5 m (>= 12 m rule), >= 4.5 m inside the search
# polygon boundary.  ids/yaws keep the old aavc_field variety.
BASELINE_PADS_ST = (
    (1, -8.0, -14.5, 0.0),
    (3, -8.0, 0.0, 55.0),
    (4, -8.0, 14.5, 120.0),
    (6, -23.5, -14.5, 240.0),
    (2, -23.5, 0.0, 170.0),
    (5, -23.5, 14.5, 300.0),
)

# Satellite ground plane (visual): centred on the pitch, big enough to show
# the track + surrounding rooftops for realism.  E x N metres.
SAT_PLANE_W = 250.0
SAT_PLANE_H = 220.0


def _unit_vectors() -> tuple[tuple[float, float], tuple[float, float]]:
    h = math.radians(AXIS_DEG)
    u = (math.sin(h), math.cos(h))          # along-axis, ENU components
    v = (-u[1], u[0])                       # +90° CCW = toward NE touchline
    return u, v


def _field_to_enu_about_c(s: float, t: float) -> tuple[float, float]:
    u, v = _unit_vectors()
    return (s * u[0] + t * v[0], s * u[1] + t * v[1])


class Geo:
    """All derived geometry, computed once at import."""

    def __init__(self) -> None:
        u, v = _unit_vectors()
        self.axis_deg = AXIS_DEG
        self.ground_alt_m = GROUND_ALT_M

        # L&R (= ENU origin = world origin = PX4 home) in lat/lon
        lnr_off = _field_to_enu_about_c(*LNR_ST)
        self.lnr_lat, self.lnr_lon = _local_enu_to_latlon(
            lnr_off[0], lnr_off[1], FIELD_CENTER_LAT, FIELD_CENTER_LON)

        def to_enu(s: float, t: float) -> tuple[float, float]:
            e, n = _field_to_enu_about_c(s, t)
            return (round(e - lnr_off[0], 2), round(n - lnr_off[1], 2))

        def to_latlon(s: float, t: float) -> tuple[float, float]:
            e, n = to_enu(s, t)
            lat, lon = _local_enu_to_latlon(e, n, self.lnr_lat, self.lnr_lon)
            return (round(lat, 7), round(lon, 7))

        self.field_center_enu = to_enu(0.0, 0.0)

        # CCW quads (SW-ish start) in field frame, then converted
        a_s, a_t = AIRSPACE_S, AIRSPACE_T
        s_s, s_t = SEARCH_S, SEARCH_T
        self.airspace_st = [(a_s[0], a_t[0]), (a_s[1], a_t[0]),
                            (a_s[1], a_t[1]), (a_s[0], a_t[1])]
        self.search_st = [(s_s[0], s_t[0]), (s_s[1], s_t[0]),
                          (s_s[1], s_t[1]), (s_s[0], s_t[1])]
        self.airspace_enu = [to_enu(s, t) for s, t in self.airspace_st]
        self.airspace_ll = [to_latlon(s, t) for s, t in self.airspace_st]
        self.search_enu = [to_enu(s, t) for s, t in self.search_st]
        self.search_ll = [to_latlon(s, t) for s, t in self.search_st]

        self.transit_enu = [to_enu(s, t) for s, t in TRANSIT_ST]
        self.transit_ll = [to_latlon(s, t) for s, t in TRANSIT_ST]

        self.pads = [
            {"id": pid, "enu": to_enu(s, t), "ll": to_latlon(s, t),
             "yaw_deg": yaw}
            for pid, s, t, yaw in BASELINE_PADS_ST
        ]

        self.sat_center_enu = self.field_center_enu
        self.sat_plane_wh = (SAT_PLANE_W, SAT_PLANE_H)

    # -- render helpers -----------------------------------------------------
    def report(self) -> str:
        L: list[str] = []
        add = L.append
        add("# KMUTNB sky-field geometry (generated by tools/gen_geo.py)")
        add(f"L&R / origin / home : lat {self.lnr_lat:.7f}  lon {self.lnr_lon:.7f}"
            f"  alt {self.ground_alt_m:g} m")
        add(f"pitch centre ENU    : {self.field_center_enu}")
        add(f"axis heading        : {self.axis_deg} deg")
        add("")
        add("## aavc_config.yaml blocks")
        add("controlled_airspace:")
        for (lat, lon), (e, n) in zip(self.airspace_ll, self.airspace_enu):
            add(f"  - [{lat:.7f}, {lon:.7f}]   # ENU ({e:+.1f}, {n:+.1f})")
        add("search_area:")
        for (lat, lon), (e, n) in zip(self.search_ll, self.search_enu):
            add(f"  - [{lat:.7f}, {lon:.7f}]   # ENU ({e:+.1f}, {n:+.1f})")
        add("transit_route:")
        for i, ((lat, lon), (e, n)) in enumerate(
                zip(self.transit_ll, self.transit_enu), start=1):
            add(f"  - [{lat:.7f}, {lon:.7f}]   # P{i} ENU ({e:+.1f}, {n:+.1f})")
        add(f"launch_recovery: [{self.lnr_lat:.7f}, {self.lnr_lon:.7f}]")
        add("")
        add("## world SDF pad includes (pose = E N 0 0 0 yaw_rad)")
        for i, p in enumerate(self.pads, start=1):
            e, n = p["enu"]
            add(f"pad_{i}: landing_pad_id_{p['id']}  pose {e:.1f} {n:.1f} 0 0 0 "
                f"{math.radians(p['yaw_deg']):.5f}")
        add("")
        add("## spawn_targets.BASELINE_PADS (id, east, north, yaw_deg)")
        add("BASELINE_PADS = (")
        for p in self.pads:
            e, n = p["enu"]
            add(f"    ({p['id']}, {e:.1f}, {n:.1f}, {p['yaw_deg']:.1f}),")
        add(")")
        add("")
        add("## launch_sitl.sh")
        add(f"PX4_HOME_LAT={self.lnr_lat:.7f}")
        add(f"PX4_HOME_LON={self.lnr_lon:.7f}")
        add(f"PX4_HOME_ALT={self.ground_alt_m:g}")
        add("")
        add("## ground_sat / fetch_sat")
        add(f"LAT0, LON0 = {self.lnr_lat:.7f}, {self.lnr_lon:.7f}")
        add(f"CENTER_E, CENTER_N = {self.sat_center_enu[0]}, {self.sat_center_enu[1]}")
        add(f"PLANE_W, PLANE_H = {SAT_PLANE_W}, {SAT_PLANE_H}")
        m_lat, m_lon = _wgs84_m_per_deg(self.lnr_lat)
        add(f"(m/deg lat {m_lat:.4f}, m/deg lon {m_lon:.4f})")
        add("")
        add("## gcs/kmutnb_field.yaml transit_waypoints")
        for i, (lat, lon) in enumerate(self.transit_ll, start=1):
            add(f"  - {{id: {i}, lat: {lat:.7f}, lon: {lon:.7f}}}")
        return "\n".join(L)


GEO = Geo()

if __name__ == "__main__":
    print(GEO.report())
