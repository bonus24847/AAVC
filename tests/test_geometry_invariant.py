"""Every consumer of the field geometry against the one source (tools/gen_geo).

Born 2026-08-18, the morning after the operator caught what no check had:
the survey migration moved the world origin, the pads and every polygon into
the drone-drawn frame — and left the ground_sat photo pose in the old frame,
parking the whole satellite picture ~77 m adrift. Coordinates were right; the
wallpaper was wrong; and the ONE leg of the invariant that drifted was the one
leg no report printed and no test read. This file reads them all:

    gen_geo.GEO  ==  sitl/aavc_config.yaml   (what the aircraft flies)
                 ==  gcs/kmutnb_field.yaml   (what the real console draws)
                 ==  sitl/launch_sitl.sh     (where PX4 spawns)
                 ==  kmutnb_skyfield.sdf     (gz origin AND the photo pose)

A frame move that forgets any consumer now fails `make test` instead of
waiting for an operator to notice the drone parked on the wrong grass.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "tools"))
from gen_geo import GEO  # noqa: E402

_DEG = 1e-6      # ~0.1 m — the paste precision of the generated blocks
_M = 0.01        # metres, for ENU poses


def _pairs_match(actual, expected, tol=_DEG) -> bool:
    return len(actual) == len(expected) and all(
        abs(float(a[0]) - float(b[0])) <= tol and abs(float(a[1]) - float(b[1])) <= tol
        for a, b in zip(actual, expected))


def test_mission_config_carries_the_generated_frame() -> None:
    cfg = yaml.safe_load((_ROOT / "sitl" / "aavc_config.yaml").read_text())
    lr = cfg["ground_operation"]["launch_recovery"]
    assert abs(lr[0] - GEO.lnr_lat) <= _DEG and abs(lr[1] - GEO.lnr_lon) <= _DEG
    assert _pairs_match(cfg["controlled_airspace"], GEO.airspace_ll)
    assert _pairs_match(cfg["search_area"], GEO.search_ll)
    assert _pairs_match(cfg["transit_route"], GEO.transit_ll)


def test_real_console_field_carries_the_generated_frame() -> None:
    fy = yaml.safe_load((_ROOT / "gcs" / "kmutnb_field.yaml").read_text())
    gf = fy["geofence"]
    assert abs(gf["local_origin"][0] - GEO.lnr_lat) <= _DEG
    assert abs(gf["local_origin"][1] - GEO.lnr_lon) <= _DEG
    assert _pairs_match(gf["controlled_airspace"], GEO.airspace_ll)
    assert _pairs_match(gf["search_area"], GEO.search_ll)
    tw = [(t["lat"], t["lon"]) for t in fy["transit_waypoints"]]
    assert _pairs_match(tw, GEO.transit_ll)


def test_sitl_launcher_spawns_at_the_generated_lnr() -> None:
    sh = (_ROOT / "sitl" / "launch_sitl.sh").read_text()
    lat = float(re.search(r'PX4_HOME_LAT="([\d.]+)"', sh).group(1))
    lon = float(re.search(r'PX4_HOME_LON="([\d.]+)"', sh).group(1))
    assert abs(lat - GEO.lnr_lat) <= _DEG and abs(lon - GEO.lnr_lon) <= _DEG


def test_world_origin_and_photo_pose_carry_the_generated_frame() -> None:
    w = (_ROOT / "sitl" / "worlds" / "kmutnb_skyfield.sdf").read_text()
    wlat = float(re.search(r"<latitude_deg>([\d.]+)", w).group(1))
    wlon = float(re.search(r"<longitude_deg>([\d.]+)", w).group(1))
    assert abs(wlat - GEO.lnr_lat) <= _DEG and abs(wlon - GEO.lnr_lon) <= _DEG
    # The leg that actually drifted (2026-08-17→18): the satellite photo's
    # include pose. It must sit at gen_geo's sat centre or the whole picture
    # lies about where everything is.
    m = re.search(r"<name>ground_sat</name>.*?<pose>([-\d. ]+)</pose>", w, re.S)
    assert m, "ground_sat include (with pose) missing from the world"
    pe, pn = (float(v) for v in m.group(1).split()[:2])
    assert abs(pe - GEO.sat_center_enu[0]) <= _M
    assert abs(pn - GEO.sat_center_enu[1]) <= _M
