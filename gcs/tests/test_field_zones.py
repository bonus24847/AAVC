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
# The flight config lives in this same repo since the three repos became one
# (gcs/ + mission/ + aruco/), so the digit-for-digit test below RUNS instead of
# skipping. AAVC_FLIGHT_CFG still overrides it for a split checkout.
_FLIGHT_CFG = os.environ.get("AAVC_FLIGHT_CFG") or os.path.join(
    _HERE, "..", "..", "mission", "sitl", "kmitl_config.yaml")


def _zones(path, monkeypatch):
    monkeypatch.setattr(aavc_gcs, "AAVC_FIELD", path)
    return aavc_gcs.load_zones()


def test_competition_field_carries_the_whole_planned_route(monkeypatch):
    z = _zones(_COMP_FIELD, monkeypatch)
    assert z["transit"] and len(z["transit"]) == 3            # root-level {id,lat,lon} rows
    assert z["transit"][0] == z["home"]                        # P1 = L&R
    # 29 since the 2026-08-30 morning redraw — the operator drew the whole
    # field again on the satellite map before the scored flight, and the sweep
    # became E-W legs with a dip into the building/courtyard pocket and a
    # detour around the north object. (It was 15 after the 2026-08-29 evening
    # redraw, 17 before that.) The digit-for-digit check against the flight
    # config is the test below; this one just pins the shape.
    assert len(z["sweep"]) == 29
    # 6 since 2026-08-30: trees, building-west, pocket-south, building-east,
    # east band, north object (was 4: building, courtyard, east, trees).
    assert len(z["keepout"]) == 6 and all(len(p) == 4 for p in z["keepout"])
    assert len(z["gateways"]) == 4 and z["gateway"] == z["gateways"][0]
    # the sweep's east ends stop short of the east band, its west ends inside the airspace
    lons = [p[1] for p in z["sweep"]]
    assert max(lons) < min(p[1] for p in z["keepout"][4])   # west of the EAST band (index 4 since 2026-08-30)
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


def test_a_gcs_only_checkout_boots_on_a_field_file_that_exists(tmp_path, monkeypatch):
    """A registry entry whose field yaml is not on disk is not selectable.

    `git clone --filter=blob:none --sparse` + `sparse-checkout set gcs` is the
    37 MB way to get just the console (the whole repo is 330 MB of flight data).
    In that checkout mission/ does not exist, so the `kmutnb` entry's field file
    is absent — and before 2026-09-02 the console booted it anyway and drew a map
    with tiles and an aircraft but no geofence, no search area, no corridor and
    no keep-outs, saying nothing. The bundled gcs/aavc_field.yaml is always
    there, so an availability rule that checks the file makes the boot land on it.
    """
    present = tmp_path / "here.yaml"
    present.write_text("geofence: {}\n")
    missing = tmp_path / "gone.yaml"        # never created

    monkeypatch.setattr(aavc_gcs, "MISSIONS", {
        "absent": {"label": "field not checked out",
                   "field": str(missing), "mission_cmd": "run {ids}"},
        "bundled": {"label": "bundled field",
                    "field": str(present), "mission_cmd": "run {ids}"},
    })

    assert aavc_gcs.mission_available(aavc_gcs.MISSIONS["absent"]) is False
    assert aavc_gcs.mission_available(aavc_gcs.MISSIONS["bundled"]) is True

    snap = {row["name"]: row["available"] for row in aavc_gcs.mission_registry_snapshot()}
    assert snap == {"absent": False, "bundled": True}

    # and picking it by hand is refused with the path it could not find
    monkeypatch.setattr(aavc_gcs, "REAL_CONSOLE", False)
    err = aavc_gcs.apply_mission("absent")
    assert err and str(missing) in err


def test_every_shipped_registry_entry_resolves_in_a_full_checkout():
    """The three entries in missions.yaml point at files this repo really has."""
    aavc_gcs.load_missions()
    assert set(aavc_gcs.MISSIONS) == {"kmutnb", "aavc", "bangbo"}
    for name, m in aavc_gcs.MISSIONS.items():
        assert os.path.exists(m["field"]), f"{name}: {m['field']}"
        assert "{repo}" not in str(m), f"{name}: unexpanded placeholder"
