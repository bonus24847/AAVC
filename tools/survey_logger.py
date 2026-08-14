#!/usr/bin/env python3
"""Field survey recorder — log the AIRCRAFT'S OWN GPS while it is carried
around the field, so the mission geometry is measured with the same sensor
that flies it (operator + professor, 2026-08-14: the practice area grows to
the WHOLE KMUTNB field, and the boundary is surveyed on the ground rather
than traced off satellite imagery).

Why the aircraft's GPS and not the picture: a no-RTK fix carries 1-2.5 m of
slowly-drifting ABSOLUTE bias. Trace the boundary off imagery and that bias
lands between the map and the drone — the geofence sits where the picture
says, not where the drone thinks it is. Survey with the SAME receiver and
the common-mode part cancels: the corner the drone measured is the corner
the drone will fly to.

Field procedure (see docs/FIELD_SURVEY.md):
  1. start this logger (drone powered, 3D fix, ideally sats >= 12)
  2. carry the aircraft to each boundary corner and STAND STILL >= 30 s
  3. finish at the intended Launch & Recovery spot, stand still >= 30 s
  4. Ctrl-C, then run tools/survey_extract.py on the file it wrote

Usage:
    env -u PYTHONPATH .venv/bin/python tools/survey_logger.py [--url URL]
        [--out captures/survey_track.jsonl]

The default endpoint is the GCS link (udpin:0.0.0.0:14550). If the AAVC GCS
console already holds that port, point this at another mavlink-router
output (e.g. --url udpin:0.0.0.0:14552) or stop the console first.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

_FIX = {0: "no-gps", 1: "no-fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-float",
        6: "RTK-fixed"}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="udpin:0.0.0.0:14550")
    p.add_argument("--out", default="captures/survey_track.jsonl")
    p.add_argument("--min-sats", type=int, default=8,
                   help="warn below this satellite count (default 8)")
    a = p.parse_args()

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"[survey] linking to {a.url} …")
    m = mavutil.mavlink_connection(a.url)
    if m.wait_heartbeat(timeout=30) is None:
        print("[survey] no heartbeat — is the aircraft powered and linked?",
              file=sys.stderr)
        return 1
    print(f"[survey] linked (sys {m.target_system}). Recording → {out}")
    print("[survey] carry the aircraft to each corner and STAND STILL >= 30 s.")
    print("[survey] Ctrl-C when the last point is done.\n")

    fh = out.open("w", encoding="utf-8")
    n = 0
    last_print = 0.0
    sats = fix = 0
    eph = float("nan")
    still_since: float | None = None
    last_ll: tuple[float, float] | None = None
    try:
        while True:
            msg = m.recv_match(
                type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"], blocking=True,
                timeout=5)
            if msg is None:
                print("[survey] …no GPS messages for 5 s")
                continue
            t = msg.get_type()
            if t == "GPS_RAW_INT":
                sats, fix = msg.satellites_visible, msg.fix_type
                eph = msg.eph / 100.0 if msg.eph not in (0, 65535) else float("nan")
                continue
            lat, lon = msg.lat / 1e7, msg.lon / 1e7
            rec = {"t": round(time.time(), 2), "lat": lat, "lon": lon,
                   "alt_m": msg.alt / 1000.0, "rel_alt_m": msg.relative_alt / 1000.0,
                   "fix": fix, "sats": sats, "eph": None if math.isnan(eph) else eph}
            fh.write(json.dumps(rec) + "\n")
            fh.flush()
            n += 1

            # live quality line + a "standing still" hint so the operator knows
            # when a corner has been held long enough
            if last_ll is not None:
                d = math.hypot((lat - last_ll[0]) * 111_320,
                               (lon - last_ll[1]) * 111_320
                               * math.cos(math.radians(lat)))
                if d < 0.8:
                    still_since = still_since or rec["t"]
                else:
                    still_since = None
            last_ll = (lat, lon)
            if rec["t"] - last_print >= 1.0:
                last_print = rec["t"]
                held = f"  นิ่งมา {rec['t'] - still_since:4.0f}s" if still_since else ""
                warn = "" if (fix >= 3 and sats >= a.min_sats) else "  ⚠ สัญญาณอ่อน"
                print(f"\r[survey] {n:5d} จุด | {_FIX.get(fix, fix)} sats {sats:2d}"
                      f" | {lat:.7f}, {lon:.7f}{held}{warn}   ", end="", flush=True)
    except KeyboardInterrupt:
        print("\n[survey] stopped.")
    finally:
        fh.close()
    print(f"[survey] wrote {n} samples → {out}")
    print("[survey] next: env -u PYTHONPATH .venv/bin/python "
          f"tools/survey_extract.py {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
