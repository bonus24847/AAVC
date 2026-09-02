#!/usr/bin/env python3
"""Turn a ground survey track into mission geometry.

Reads the jsonl written by tools/survey_logger.py, finds every spot where
the aircraft STOOD STILL (that's how the operator marks a corner: carry,
stop, wait), averages each dwell into one measured point, then fits the
field frame those points describe: centre, long-axis heading, and the s/t
extents used by tools/gen_geo.py.

Averaging a 30 s dwell kills the second-to-second noise; the residual
absolute bias is COMMON to every point and to the flying aircraft, which is
exactly why the survey is done with the aircraft's own receiver.

Usage:
    tools/survey_extract.py captures/survey_track.jsonl \
        [--names "NW,NE,SE,SW,LR"] [--dwell 20] [--radius 1.5]

Output: the measured points (with scatter + satellite count so a bad dwell
is visible), the fitted frame, and the exact gen_geo.py constants to paste.
Nothing is written automatically — the geometry invariant (CLAUDE.md §8)
says every derived number comes from gen_geo.py, so the operator reviews
these numbers and gen_geo regenerates the rest.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

_R_EARTH = 6_378_137.0


def _m_per_deg(lat: float) -> tuple[float, float]:
    """metres per degree (lat, lon) — WGS84 ellipsoidal, matching
    sitl/spawn_targets.py so every frame in the project agrees."""
    a, f = 6_378_137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    s = math.sin(math.radians(lat))
    w = math.sqrt(1 - e2 * s * s)
    m_lat = math.pi * a * (1 - e2) / (180 * w ** 3)
    m_lon = math.pi * a * math.cos(math.radians(lat)) / (180 * w)
    return m_lat, m_lon


def load(path: Path, min_fix: int) -> list[dict]:
    pts = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if r.get("fix", 0) >= min_fix:
            pts.append(r)
    return pts


def dwells(samples: list[dict], min_s: float, radius_m: float) -> list[dict]:
    """Group consecutive samples that stay within `radius_m` of their running
    mean for at least `min_s` — one group per corner the operator held."""
    out: list[dict] = []
    cur: list[dict] = []

    def flush() -> None:
        if not cur:
            return
        dur = cur[-1]["t"] - cur[0]["t"]
        if dur < min_s:
            return
        lat = sum(p["lat"] for p in cur) / len(cur)
        lon = sum(p["lon"] for p in cur) / len(cur)
        ml, mo = _m_per_deg(lat)
        scat = max(math.hypot((p["lat"] - lat) * ml, (p["lon"] - lon) * mo)
                   for p in cur)
        out.append({"lat": lat, "lon": lon, "n": len(cur), "dur_s": dur,
                    "scatter_m": scat,
                    "sats": min(p.get("sats", 0) for p in cur),
                    "alt_m": sum(p["alt_m"] for p in cur) / len(cur)})

    for s in samples:
        if not cur:
            cur = [s]
            continue
        lat = sum(p["lat"] for p in cur) / len(cur)
        lon = sum(p["lon"] for p in cur) / len(cur)
        ml, mo = _m_per_deg(lat)
        if math.hypot((s["lat"] - lat) * ml, (s["lon"] - lon) * mo) <= radius_m:
            cur.append(s)
        else:
            flush()
            cur = [s]
    flush()
    return out


def _hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convex hull (Andrew monotone chain)."""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def half(seq):
        out: list[tuple[float, float]] = []
        for p in seq:
            while len(out) >= 2:
                (x1, y1), (x2, y2) = out[-2], out[-1]
                if (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) > 0:
                    break
                out.pop()
            out.append(p)
        return out

    return half(pts)[:-1] + half(reversed(pts))[:-1]


def fit_frame(pts: list[dict]) -> dict:
    """Centre + long-axis heading from the measured corners via the
    MINIMUM-AREA bounding rectangle (rotating calipers over the hull).

    Not the widest point pair: on a rectangle that is the DIAGONAL, which
    fitted a 4-corner test walk at 115° instead of its true 143.8° axis and
    inflated the area by 2x (caught by the synthetic-walk test, 2026-08-14).
    """
    lat0 = sum(p["lat"] for p in pts) / len(pts)
    lon0 = sum(p["lon"] for p in pts) / len(pts)
    ml, mo = _m_per_deg(lat0)
    en = [((p["lon"] - lon0) * mo, (p["lat"] - lat0) * ml) for p in pts]
    hull = _hull(en) or en

    best: tuple[float, float, float, float] | None = None
    for i in range(len(hull)):
        (x1, y1), (x2, y2) = hull[i], hull[(i + 1) % len(hull)]
        ang = math.atan2(y2 - y1, x2 - x1)          # rotate this edge flat
        ca, sa = math.cos(-ang), math.sin(-ang)
        rot = [(e * ca - n * sa, e * sa + n * ca) for e, n in en]
        w = max(p[0] for p in rot) - min(p[0] for p in rot)
        h = max(p[1] for p in rot) - min(p[1] for p in rot)
        if best is None or w * h < best[0]:
            best = (w * h, ang, w, h)
    assert best is not None, "hull is empty — no points to fit"
    _, ang, w, h = best
    # +s runs along the LONGER side of that rectangle
    long_ang = ang if w >= h else ang + math.pi / 2
    de, dn = math.cos(long_ang), math.sin(long_ang)
    axis = math.degrees(math.atan2(de, dn)) % 180.0        # heading of +s
    th = math.radians(axis)
    u = (math.sin(th), math.cos(th))
    v = (math.cos(th), -math.sin(th))
    st = [(e * u[0] + n * u[1], e * v[0] + n * v[1]) for e, n in en]
    return {"lat0": lat0, "lon0": lon0, "axis_deg": axis, "st": st,
            "span_m": max(w, h)}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("track", type=Path)
    p.add_argument("--names", default="",
                   help="comma-separated labels in dwell order, e.g. "
                        "'NW,NE,SE,SW,LR' (LR = the Launch & Recovery point)")
    p.add_argument("--dwell", type=float, default=20.0,
                   help="minimum seconds standing still to count (default 20)")
    p.add_argument("--radius", type=float, default=1.5,
                   help="how far the fix may wander and still count as still")
    p.add_argument("--min-fix", type=int, default=3)
    a = p.parse_args()

    samples = load(a.track, a.min_fix)
    if not samples:
        print("ไม่มีตัวอย่างที่ fix >= 3 ในไฟล์นี้", file=sys.stderr)
        return 1
    marks = dwells(samples, a.dwell, a.radius)
    if not marks:
        print(f"ไม่พบจุดที่ยืนนิ่ง >= {a.dwell:.0f}s "
              "(ลอง --dwell 10 หรือ --radius 2.5)", file=sys.stderr)
        return 1
    names = [s.strip() for s in a.names.split(",") if s.strip()]

    print(f"\n=== จุดที่วัดได้ {len(marks)} จุด "
          f"(จาก {len(samples)} ตัวอย่าง) ===")
    for k, m in enumerate(marks):
        nm = names[k] if k < len(names) else f"P{k + 1}"
        flag = ""
        if m["scatter_m"] > 1.5:
            flag += "  ⚠ กระจายเกิน 1.5 m — ยืนนิ่งกว่านี้/รอสัญญาณดีขึ้น"
        if m["sats"] < 8:
            flag += f"  ⚠ sats {m['sats']}"
        print(f"  {nm:>4} : {m['lat']:.7f}, {m['lon']:.7f}   "
              f"ยืน {m['dur_s']:4.0f}s ({m['n']:3d} ตัวอย่าง) "
              f"กระจาย {m['scatter_m']:.2f} m sats {m['sats']}{flag}")

    lr = None
    corners = marks
    if names and names[-1].upper() in ("LR", "L&R", "HOME") and len(marks) >= 2:
        lr, corners = marks[-1], marks[:-1]
    if len(corners) < 3:
        print("\n(ต้องมีมุมอย่างน้อย 3 จุดถึงจะ fit กรอบสนามได้)")
        return 0

    fr = fit_frame(corners)
    ss = [s for s, _ in fr["st"]]
    ts = [t for _, t in fr["st"]]
    print("\n=== กรอบสนามที่ fit ได้ ===")
    print(f"  แกนยาวสนาม  : {fr['axis_deg']:.1f}°  (ระยะมุมไกลสุด {fr['span_m']:.1f} m)")
    print(f"  ขนาดใช้งาน  : {max(ss) - min(ss):.1f} m (ตามแนว) × "
          f"{max(ts) - min(ts):.1f} m (ขวาง)")
    cs, ct = (max(ss) + min(ss)) / 2, (max(ts) + min(ts)) / 2
    th = math.radians(fr["axis_deg"])
    ml, mo = _m_per_deg(fr["lat0"])
    ce = cs * math.sin(th) + ct * math.cos(th)
    cn = cs * math.cos(th) - ct * math.sin(th)
    clat, clon = fr["lat0"] + cn / ml, fr["lon0"] + ce / mo
    print(f"  ศูนย์กลางสนาม: {clat:.7f}, {clon:.7f}")

    print("\n=== ค่าที่จะเอาไปใส่ tools/gen_geo.py ===")
    print(f"FIELD_CENTER_LAT = {clat:.6f}")
    print(f"FIELD_CENTER_LON = {clon:.6f}")
    print(f"AXIS_DEG = {fr['axis_deg']:.1f}")
    print(f"AIRSPACE_S = ({min(ss) - cs:+.1f}, {max(ss) - cs:+.1f})")
    print(f"AIRSPACE_T = ({min(ts) - ct:+.1f}, {max(ts) - ct:+.1f})")
    if lr is not None:
        e = (lr["lon"] - fr["lon0"]) * mo
        n = (lr["lat"] - fr["lat0"]) * ml
        s = e * math.sin(th) + n * math.cos(th) - cs
        t = e * math.cos(th) - n * math.sin(th) - ct
        print(f"LNR_ST = ({s:+.1f}, {t:+.1f})     # จากจุดที่วัดไว้ "
              f"({lr['lat']:.7f}, {lr['lon']:.7f})")
    print("\n(ตัวเลขพวกนี้ยังไม่ถูกเขียนลงไฟล์ — ตรวจก่อน แล้วค่อยให้ gen_geo "
          "สร้าง config/world/launcher/GCS ใหม่ทั้งชุด)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
