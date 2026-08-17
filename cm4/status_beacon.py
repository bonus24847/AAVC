#!/usr/bin/env python3
"""Mission + camera status over the RADIO, as MAVLink STATUSTEXT.

Why this exists (operator 2026-08-15): the console's phase stepper, pad ✓ ticks
and camera panel all arrive over **WiFi** (status_sync rsync's captures/ and the
nadir frame off the CM4). At the field WiFi does not reach the aircraft, so the
one moment the operator wants those readouts — mid-flight — is exactly when they
freeze. Telemetry, meanwhile, rides the NOMAD radio the whole way.

So put the *summary* on the radio and leave the *detail* on WiFi:

    AAVC p=SERVE d=2/4 m=3 ok=1,3           ~30 bytes
    AAVC pads 1:12.3,-8.1 3:5.0,14.2        ~35 bytes (chunked, <=50 each)
    AAVC cam=OK 0.9s                        ~16 bytes

That is well under 100 B/s against ~250 KB/s for the frame pull — and it
answers the three questions the operator actually asked: *which pads are done*,
*WHERE the mapped pads are* (ENU about the field origin, 0.1 m — the map draws
the same marker with WiFi or without it; operator 2026-08-17), and *is the
camera alive or dead*. The frame IMAGE stays on WiFi for whoever is near the
L&R point — the radio physically cannot carry it.

HOW IT REACHES THE GROUND: this sends into the CM4's mavlink-router, which
routes to the FC over TELEM2. STATUSTEXT carries no target field, so it is a
BROADCAST message, and PX4 forwards broadcast traffic between MAVLink instances
when the receiving instance has ``MAV_<i>_FORWARD = 1``
(src/modules/mavlink/module.yaml — its own documented example is "a GCS talking
to a camera connected to the autopilot on a different link"). So the TELEM2
instance needs MAV_x_FORWARD=1 (reboot required) for the radio to see these.
Without it nothing breaks — the lines simply never leave the aircraft.

Identity: sysid 1 / component 191 (MAV_COMP_ID_ONBOARD_COMPUTER) — the honest
identity for a companion computer, and not the 255 a GCS uses.

Runs OFF the flight-critical path: every failure is swallowed and retried, the
flight core neither imports nor knows about this file (same contract as
sitl/payload_detach_bridge.py). Losing it costs a readout, never a mission.

    python3 cm4/status_beacon.py                       # defaults, on the CM4
    python3 cm4/status_beacon.py --dry-run             # print, do not send
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# STATUSTEXT carries 50 chars; longer text is chunked by MAVLink v2, but a
# chunked line costs several packets on a link this narrow — keep every line
# inside one packet instead.
_MAX_TEXT = 50
_SEV_INFO = 6      # MAV_SEVERITY_INFO
_SEV_WARN = 4      # MAV_SEVERITY_WARNING

# A frame older than this means the grabber is wedged or dead, not merely slow
# (the real grabber writes at several Hz; the mission's own vision gate is 2 s).
_CAM_DEAD_S = 6.0


def _fmt_ids(ids: list) -> str:
    """``[1, 3]`` -> ``"1,3"``; empty -> ``"-"``. Trimmed to keep the line short."""
    out = ",".join(str(i) for i in ids)
    return out or "-"


def compose_lines(status: dict | None, frame_age_s: float | None) -> list[tuple[int, str]]:
    """``[(severity, text)]`` for one beacon tick. Pure — unit-testable without
    a link, a simulator or a camera.

    ``status`` is captures/mission_status.json as the orchestrator writes it
    (phase/assigned/delivered/pads_mapped/updated); ``None`` before the first
    mission of the session. ``frame_age_s`` is the age of the nadir frame, or
    ``None`` when the file does not exist at all.
    """
    lines: list[tuple[int, str]] = []

    if status:
        phase = str(status.get("phase") or "?")[:9]
        assigned = list(status.get("assigned") or [])
        delivered = list(status.get("delivered") or [])
        seen = len(status.get("pads_mapped") or {})
        # d=<delivered>/<assigned> is the number the operator is counting;
        # m=<n> says how many pads the search has actually put on the map,
        # which is the difference between "still looking" and "stuck".
        lines.append((_SEV_INFO,
                      f"AAVC p={phase} d={len(delivered)}/{len(assigned)} "
                      f"m={seen} ok={_fmt_ids(delivered)}"[:_MAX_TEXT]))
        # Pad COORDINATES over the radio (operator 2026-08-17: "เอาแค่พิกัดพอ
        # ที่ใช้วิทยุ") — ~13 chars per pad, chunked so every line stays one
        # STATUSTEXT packet. Values are relayed verbatim from mission_status's
        # pads_mapped (ENU metres about the field yaml's local_origin), so the
        # console draws the SAME marker with WiFi or without it. 0.1 m
        # resolution: an order finer than the no-RTK GPS the fix came from.
        pads = status.get("pads_mapped") or {}
        entries = [f"{pid}:{en[0]:.1f},{en[1]:.1f}"
                   for pid, en in sorted(
                       pads.items(),
                       key=lambda kv: (not str(kv[0]).isdigit(), str(kv[0])))]
        line = "AAVC pads"
        for ent in entries:
            if len(line) + 1 + len(ent) > _MAX_TEXT:
                lines.append((_SEV_INFO, line))
                line = "AAVC pads"
            line += " " + ent
        if line != "AAVC pads":
            lines.append((_SEV_INFO, line))
    else:
        lines.append((_SEV_INFO, "AAVC p=idle (no mission yet)"))

    if frame_age_s is None:
        lines.append((_SEV_WARN, "AAVC cam=NONE no frame file"))
    elif frame_age_s > _CAM_DEAD_S:
        lines.append((_SEV_WARN, f"AAVC cam=DEAD {frame_age_s:.0f}s stale"))
    else:
        lines.append((_SEV_INFO, f"AAVC cam=OK {frame_age_s:.1f}s"))
    return lines


def _read_status(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _frame_age(path: Path) -> float | None:
    try:
        return max(0.0, time.time() - path.stat().st_mtime)
    except OSError:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14550",
                    help="mavlink-router endpoint to inject into "
                         "(default: the qgc server port on the CM4)")
    ap.add_argument("--captures", type=Path, default=Path("captures"),
                    help="dir holding mission_status.json (default: ./captures)")
    ap.add_argument("--frame", type=Path, default=Path("/tmp/aavc_nadir.png"),
                    help="nadir frame the camera grabber writes")
    ap.add_argument("--interval-s", type=float, default=5.0)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the lines instead of sending (no MAVLink at all)")
    args = ap.parse_args()

    status_path = args.captures / "mission_status.json"
    mav = None
    if not args.dry_run:
        try:
            from pymavlink import mavutil
        except Exception as e:                       # pymavlink missing/broken
            print(f"[beacon] pymavlink unavailable ({e}) — beacon disabled",
                  flush=True)
            return 0
        try:
            mav = mavutil.mavlink_connection(
                args.endpoint, source_system=1, source_component=191)
        except Exception as e:
            print(f"[beacon] cannot open {args.endpoint} ({e}) — beacon disabled",
                  flush=True)
            return 0
        print(f"[beacon] {status_path} + {args.frame} -> {args.endpoint} "
              f"every {args.interval_s}s (sysid 1 / comp 191)", flush=True)

    while True:
        lines = compose_lines(_read_status(status_path), _frame_age(args.frame))
        for sev, text in lines:
            if args.dry_run:
                print(f"[beacon] sev={sev} {text}", flush=True)
                continue
            try:
                mav.mav.statustext_send(sev, text.encode("ascii", "replace")[:_MAX_TEXT])
            except Exception as e:                   # link down, router restarting…
                print(f"[beacon] send failed ({e}) — retrying next tick", flush=True)
                break
        time.sleep(args.interval_s)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("[beacon] stopped", flush=True)
