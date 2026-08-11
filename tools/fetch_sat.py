#!/usr/bin/env python3
"""Fetch the satellite backdrop for the KMUTNB sky-field and stitch it into
sitl/models/ground_sat/materials/textures/sat_real.png, georeferenced to the
exact world extent of the ground_sat plane.

Geometry (origin, plane centre/size) is imported from tools/gen_geo.py — the
project's single source of truth — so the imagery can never drift from the
world/config/spawn georeference.  ENU→lat/lon uses the same WGS84
metres-per-degree formula as spawn_targets.py (gz EARTH_WGS84 frame), not the
old spherical M=111320 approximation (~0.7 m error at the plane edges).

Providers:
  google -> mt1.google.com/vt/lyrs=s   (z20 here, ~14.5 cm/px, sharpest)
  esri   -> ArcGIS World_Imagery       (z19, ~29 cm/px; license-clean)
Grey "not yet available" placeholder tiles are detected (low pixel variance)
and trigger the next fallback in the chain.

Usage:  python3 tools/fetch_sat.py            # google z20 -> esri fallback
        python3 tools/fetch_sat.py --provider esri --zoom 19
Restart the sim AND clear the ~/.gz cache to see a refreshed texture.
"""
import argparse
import io
import math
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image

from gen_geo import GEO, _local_enu_to_latlon

LAT0, LON0 = GEO.lnr_lat, GEO.lnr_lon                 # L&R = world origin
CENTER_E, CENTER_N = GEO.sat_center_enu               # plane centre (pitch centre)
PLANE_W, PLANE_H = GEO.sat_plane_wh                   # metres (east, north)

_ROOT = Path(__file__).resolve().parent.parent
OUT = str(_ROOT / "sitl/models/ground_sat/materials/textures/sat_real.png")
UA = {"User-Agent": "Mozilla/5.0 (KMUTNB-sim georef fetch)"}
PROV = {
    "google": lambda z, x, y: f"https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
    "esri":   lambda z, x, y: ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                               f"World_Imagery/MapServer/tile/{z}/{y}/{x}"),
}


def _latlon(n, e):
    return _local_enu_to_latlon(e, n, LAT0, LON0)


def _deg2px(lat, lon, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n * 256
    lr = math.radians(lat)
    y = (1 - math.asinh(math.tan(lr)) / math.pi) / 2 * n * 256
    return x, y


def _is_placeholder(im):
    return np.asarray(im).astype(int).std() < 12.0


def _fetch(url):
    for _ in range(4):
        try:
            data = urllib.request.urlopen(urllib.request.Request(url, headers=UA),
                                          timeout=25).read()
            im = Image.open(io.BytesIO(data)).convert("RGB")
            if im.size == (256, 256):
                return im
        except Exception:
            pass
    return None


def build(provider, z):
    e0, e1 = CENTER_E - PLANE_W / 2, CENTER_E + PLANE_W / 2
    n0, n1 = CENTER_N - PLANE_H / 2, CENTER_N + PLANE_H / 2
    latS, lonW = _latlon(n0, e0)
    latN, lonE = _latlon(n1, e1)
    xw, yn = _deg2px(latN, lonW, z)           # NW corner (top-left)
    xe, ys = _deg2px(latS, lonE, z)           # SE corner (bottom-right)
    tx0, tx1 = int(xw // 256), int(xe // 256)
    ty0, ty1 = int(yn // 256), int(ys // 256)
    ntx, nty = tx1 - tx0 + 1, ty1 - ty0 + 1
    canvas = Image.new("RGB", (ntx * 256, nty * 256))
    jobs = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]

    def work(j):
        tx, ty = j
        return tx, ty, _fetch(PROV[provider](z, tx, ty))

    bad = 0
    with ThreadPoolExecutor(max_workers=8) as ex:
        for tx, ty, im in ex.map(work, jobs):
            if im is None or _is_placeholder(im):
                bad += 1
                continue
            canvas.paste(im, ((tx - tx0) * 256, (ty - ty0) * 256))
    if bad:
        print(f"  {provider} z{z}: {bad}/{len(jobs)} bad/placeholder tiles -> skip")
        return None
    ox, oy = tx0 * 256, ty0 * 256
    crop = canvas.crop((round(xw - ox), round(yn - oy), round(xe - ox), round(ys - oy)))
    print(f"  {provider} z{z}: OK {ntx}x{nty} tiles -> {crop.size} "
          f"({PLANE_W / crop.size[0] * 100:.1f} cm/px)")
    return crop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--provider", default=None, help="google|esri (default: try both)")
    ap.add_argument("--zoom", type=int, default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    if a.provider and a.zoom:
        chain = [(a.provider, a.zoom)]
    else:
        chain = [("google", 20), ("esri", 19), ("google", 19), ("esri", 18)]
    for provider, z in chain:
        crop = build(provider, z)
        if crop is not None:
            Path(a.out).parent.mkdir(parents=True, exist_ok=True)
            crop.save(a.out)
            print(f"SAVED {a.out} {crop.size} from {provider} z{z}")
            return 0
    print("ERROR: no provider/zoom returned complete imagery")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
