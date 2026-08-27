#!/usr/bin/env python3
"""Fill the console's offline map-tile cache for a field, ahead of a field day.

    python3 scripts/prefetch_tiles.py --field aavc_field.yaml
    python3 scripts/prefetch_tiles.py --field aavc_field.yaml \
        --field "/home/bonus-linux/Desktop/mission AAVC in kmutnb/gcs/kmutnb_field.yaml"
    python3 scripts/prefetch_tiles.py --bbox 13.7298 100.7872 13.7314 100.7899

Reads the field yaml's geofence.controlled_airspace, grows it by --margin-m,
and downloads every tile at --zooms from aavc_gcs.TILE_URL (Esri World
Imagery — satellite JPEG) into aavc_gcs.TILE_DIR (repo tiles/), skipping tiles
already there. A field is a few hundred tiles (KMITL at 300 m margin, z15-19:
229). Run it while the laptop is ONLINE; the console then serves the map with
no internet at all. Anything the server returns that is not a PNG/JPEG is
refused (2026-08-27: OSM served a "403 blocked" placeholder as HTTP 200 and
poisoned the whole cache — wipe tiles/ and re-run if that ever recurs).
"""
import argparse
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import aavc_gcs  # noqa: E402


def _bbox_of_field(path):
    gf = (yaml.safe_load(open(path)) or {}).get("geofence", {})
    pts = [(float(a), float(b)) for a, b in gf.get("controlled_airspace", [])]
    if len(pts) < 3:
        raise SystemExit(f"{path}: no geofence.controlled_airspace polygon")
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    return min(lats), min(lons), max(lats), max(lons)


def _zooms(spec):
    lo, _, hi = spec.partition("-")
    lo = int(lo)
    hi = int(hi or lo)
    return list(range(lo, hi + 1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--field", action="append", default=[],
                    help="field yaml (repeatable); bbox = geofence.controlled_airspace")
    ap.add_argument("--bbox", nargs=4, type=float, metavar=("LAT_MIN", "LON_MIN", "LAT_MAX", "LON_MAX"))
    ap.add_argument("--zooms", default="15-19", help="zoom range, e.g. 15-19 (default)")
    ap.add_argument("--margin-m", type=float, default=300.0,
                    help="grow the bbox by this many metres all round (default 300)")
    ap.add_argument("--sleep", type=float, default=0.05, help="pause between fetches (s)")
    args = ap.parse_args(argv)

    boxes = [(f, _bbox_of_field(f)) for f in args.field]
    if args.bbox:
        boxes.append(("--bbox", tuple(args.bbox)))
    if not boxes:
        ap.error("give at least one --field or a --bbox")

    total_have = total_new = total_fail = 0
    for name, (lat_min, lon_min, lat_max, lon_max) in boxes:
        print(f"[prefetch] {name}: bbox {lat_min:.6f},{lon_min:.6f} .. "
              f"{lat_max:.6f},{lon_max:.6f} (+{args.margin_m:.0f} m) -> {aavc_gcs.TILE_DIR}")
        for z in _zooms(args.zooms):
            tiles = aavc_gcs.tiles_covering(lat_min, lon_min, lat_max, lon_max, z,
                                            margin_m=args.margin_m)
            have = new = fail = 0
            for x, y in tiles:
                path = aavc_gcs.tile_cache_path(f"/tiles/{z}/{x}/{y}.png")
                if path is None:
                    continue
                if os.path.isfile(path):
                    have += 1
                    continue
                data = aavc_gcs.fetch_tile(z, x, y)
                if data is None:
                    fail += 1
                    continue
                aavc_gcs.save_tile(path, data)
                new += 1
                time.sleep(args.sleep)
            print(f"[prefetch]   z{z}: {len(tiles)} tiles — cached {have}, fetched {new}, failed {fail}")
            total_have += have
            total_new += new
            total_fail += fail
    print(f"[prefetch] done: {total_have} already cached, {total_new} fetched, {total_fail} failed")
    return 1 if total_fail else 0


if __name__ == "__main__":
    sys.exit(main())
