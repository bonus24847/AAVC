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

import math
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


def test_every_field_config_calls_its_L_and_R_one_thing() -> None:
    """``site.center`` and ``ground_operation.launch_recovery`` are ONE point.

    Every conversion from a pad's lat/lon into the metres the console and the
    radio beacon carry measures from that point, and the console re-anchors
    what it receives at the VEHICLE's origin — so if the config names the
    place twice and the two names disagree, every pad marker slides by the
    difference while each individual number still looks reasonable.

    Added 2026-08-22, after ``sitl/kmitl_config.yaml`` was found shipping the
    practice field's entire ``site:`` block over the competition field's L&R —
    **31.5 km apart**, with the comment on that very line asserting the
    equality it broke. It survived because it was the one leg nothing read
    out loud: the aircraft takes its home from GPS and flew fine. Same shape
    as the ``ground_sat`` drift this file was born for; this is the check that
    was missing. Field-agnostic on purpose — it needs no ``gen_geo``, so it
    covers the competition config too, and any field added later.
    """
    cfgs = sorted((_ROOT / "sitl").glob("*config.yaml"))
    assert len(cfgs) >= 2, f"expected both field configs, found {cfgs}"
    for path in cfgs:
        cfg = yaml.safe_load(path.read_text())
        lr = cfg["ground_operation"]["launch_recovery"]
        site = cfg["site"]
        dn = math.radians(site["center_lat"] - lr[0]) * 6371000.0
        de = (math.radians(site["center_lon"] - lr[1]) * 6371000.0
              * math.cos(math.radians(lr[0])))
        off = math.hypot(dn, de)
        assert off <= 1.0, (
            f"{path.name}: site.center is {off:,.0f} m from "
            f"ground_operation.launch_recovery — they name the same point")


def test_mission_config_carries_the_generated_frame() -> None:
    cfg = yaml.safe_load((_ROOT / "sitl" / "aavc_config.yaml").read_text())
    lr = cfg["ground_operation"]["launch_recovery"]
    assert abs(lr[0] - GEO.lnr_lat) <= _DEG and abs(lr[1] - GEO.lnr_lon) <= _DEG
    assert _pairs_match(cfg["controlled_airspace"], GEO.airspace_ll)
    assert _pairs_match(cfg["search_area"], GEO.search_ll)
    assert _pairs_match(cfg["transit_route"], GEO.transit_ll)
    # The sweep axis rotates the boustrophedon grid, so it is part of the frame.
    # The polygons were regenerated from gen_geo's measured 143.2 deg, but
    # sweep_axis_deg was left at the old satellite-traced 143.8 (code-review
    # 2026-08-19) — a 0.6 deg drift that walked the legs ~1 m off the survey.
    assert abs(float(cfg["search"]["sweep_axis_deg"]) - GEO.axis_deg) <= 1e-6


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
