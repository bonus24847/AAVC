"""The map's static layers come from the field file (2026-08-28, operator:
"แสดง path ที่บินเลย"): the corridor, the hand-laid sweep, the routing gateway and
the keep-out bands. Also pins the fix for the corridor never drawing — both
field files write ``transit_waypoints`` at the ROOT as ``{id, lat, lon}`` rows and
the old reader only looked inside ``geofence:`` for bare pairs."""

import os
import sys
import textwrap

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402

_HERE = os.path.dirname(__file__)
_COMP_FIELD = os.path.join(_HERE, "..", "aavc_field.yaml")
_FLIGHT_CFG = os.path.expanduser("~/Desktop/aavc-comp/sitl/kmitl_config.yaml")


def _zones(path, monkeypatch):
    monkeypatch.setattr(aavc_gcs, "AAVC_FIELD", path)
    return aavc_gcs.load_zones()


def test_competition_field_carries_the_whole_planned_route(monkeypatch):
    z = _zones(_COMP_FIELD, monkeypatch)
    assert z["transit"] and len(z["transit"]) == 3            # root-level {id,lat,lon} rows
    assert z["transit"][0] == z["home"]                        # P1 = L&R
    assert len(z["sweep"]) == 14                               # 7 legs + the tree-side pass (2026-08-29)
    assert len(z["keepout"]) == 4 and all(len(p) == 4 for p in z["keepout"])   # building, courtyard, east, trees (2026-08-29)
    assert len(z["gateways"]) == 3 and z["gateway"] == z["gateways"][0]
    # the sweep's east ends stop short of the east band, its west ends inside the airspace
    lons = [p[1] for p in z["sweep"]]
    assert max(lons) < min(p[1] for p in z["keepout"][2])   # west of the EAST band (index 2 since 2026-08-29: building, courtyard, east, trees)
    assert min(lons) > min(p[1] for p in z["airspace"])


def test_planned_route_matches_the_flight_config_digit_for_digit():
    """Same numbers as what the aircraft flies, or the map lies. Skipped where the
    flight repo is not checked out beside this one."""
    if not os.path.exists(_FLIGHT_CFG):
        return
    cfg = yaml.safe_load(open(_FLIGHT_CFG))
    field = yaml.safe_load(open(_COMP_FIELD))
    assert [[r["lat"], r["lon"]] for r in field["transit_waypoints"]] == \
        [[float(a), float(b)] for a, b in cfg["transit_route"]]
    assert field["keepout_zones"] == [[list(map(float, v)) for v in poly]
                                      for poly in cfg["routing"]["keepout_zones"]]
    assert field["planned_path"]["gateways"] == [[float(v) for v in g] for g in cfg["routing"]["gateways"]]
    # the sweep is ENU in the flight config; the field file carries it as lat/lon
    import math
    lat0, lon0 = (float(v) for v in cfg["ground_operation"]["launch_recovery"])
    R = 6378137.0
    for (e, n), (lat, lon) in zip(cfg["search"]["sweep_waypoints_enu"], field["planned_path"]["sweep"]):
        assert abs(lat - (lat0 + math.degrees(n / R))) < 2e-6
        assert abs(lon - (lon0 + math.degrees(e / (R * math.cos(math.radians(lat0)))))) < 2e-6


def test_old_style_field_file_still_loads(tmp_path, monkeypatch):
    """Bare pairs under geofence: (the form the reader always accepted) keep working,
    and a file without the planned-route keys yields no sweep/keepout/gateway."""
    f = tmp_path / "f.yaml"
    f.write_text(textwrap.dedent("""
        geofence:
          controlled_airspace: [[13.7313, 100.7872], [13.7314, 100.7899], [13.7300, 100.7898], [13.7298, 100.7872]]
          search_area: [[13.7312, 100.7878], [13.7314, 100.7899], [13.7307, 100.7898], [13.7307, 100.7878]]
          transit_waypoints: [[13.730322, 100.787446], [13.730397, 100.788694]]
          local_origin: [13.730322, 100.787446]
    """))
    z = _zones(str(f), monkeypatch)
    assert z["transit"] == [[13.730322, 100.787446], [13.730397, 100.788694]]
    assert "sweep" not in z and "keepout" not in z and "gateway" not in z
