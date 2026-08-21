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
import re
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


# How old the mission status may be before the beacon says so out loud. The
# orchestrator writes at ~1 Hz while flying, so anything past this means nobody
# is writing any more — the mission ended, or its process died.
_STATUS_STALE_S = 30.0


def compose_lines(status: dict | None, frame_age_s: float | None,
                  status_age_s: float | None = None) -> list[tuple[int, str]]:
    """``[(severity, text)]`` for one beacon tick. Pure — unit-testable without
    a link, a simulator or a camera.

    ``status`` is captures/mission_status.json as the orchestrator writes it
    (phase/assigned/delivered/pads_mapped/updated); ``None`` before the first
    mission of the session. ``frame_age_s`` is the age of the nadir frame, or
    ``None`` when the file does not exist at all.
    """
    lines: list[tuple[int, str]] = []

    if status:
        # 17, not 9 (2026-08-18). The console does not merely PRINT the phase —
        # it keys behaviour off the text: the 🚀 button shows "staged" (the
        # aircraft has confirmed the mission) only when the phase CONTAINS
        # "preflight". Truncated to 9 it arrived as "recon (pr", the match
        # failed, and the radio path jumped straight to "flying" — a button
        # lying about whether the mission ever reached the drone. 17 is a
        # line-WIDTH budget, not the longest phase: transit phases run longer
        # ("deliver (transit_egress)" is 24, truncating to "deliver (transit_"),
        # but the console's substring matches — "preflight"/"recon"/"deliver" —
        # all fall inside the first 17 chars, which a test pins.
        phase = str(status.get("phase") or "?")[:17]
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
        # Identified-but-unconfirmed ids (operator request 2026-08-21): the
        # moment a marker id is first DECODED it must reach the screen over
        # the radio — the WiFi sync dies in flight, and the pads line above
        # carries CONFIRMED pads only, so G7 flight 1 was pulled down while
        # ids 4,5 were being identified live behind a blank console. Ids
        # only (all six = "AAVC seen=1,2,3,4,5,6", 21 chars — one packet);
        # omitted entirely when none, like the pads line.
        ident = status.get("pads_identified") or {}
        ident_ids = sorted(ident, key=lambda p: (not str(p).isdigit(), str(p)))
        if ident_ids:
            lines.append((_SEV_INFO,
                          ("AAVC seen=" +
                           ",".join(str(i) for i in ident_ids))[:_MAX_TEXT]))
        # Progress bar + milestone strip over the radio (operator 2026-08-18:
        # "ที่เราคุยกันว่าข้อมูลทั้งหมดบนวิทยุ มันหายไปไหน"). `progress` is not
        # one readout among many — it is the SWITCH the console tests to decide
        # whether to draw the awareness pack at all, so without it the whole
        # %-bar + milestone strip collapses back to the old three-step stepper.
        # It costs one 34-char line to carry, so it rides the radio like the
        # rest. The two derived fields exist because the console reads them out
        # of Thai strings that STATUSTEXT (ascii-only) cannot carry:
        #   tp=110  which transit points have been passed — the P1·P2·P3 chip
        #           reads this out of the event feed's "ผ่านจุด Pn" lines
        #   cur=3   the pad being served right now — the console finds it by
        #           searching progress_label for "pad N "
        # The console rebuilds both back into the shapes its widgets expect.
        prg = status.get("progress")
        if isinstance(prg, (int, float)):
            texts = " ".join(str(e.get("text", "")) for e in (status.get("events") or []))
            tp = "".join("1" if f"ผ่านจุด P{n}" in texts else "0" for n in (1, 2, 3))
            cur = re.search(r"pad (\d+)", str(status.get("progress_label") or ""))
            eta = status.get("eta_s")
            lines.append((_SEV_INFO,
                          f"AAVC prg={int(prg)} eta={int(eta) if eta else 0} "
                          f"tp={tp} cur={cur.group(1) if cur else '-'}"[:_MAX_TEXT]))
    else:
        lines.append((_SEV_INFO, "AAVC p=idle (no mission yet)"))

    # WHY the aircraft came home unfinished — an ascii code the console maps
    # back to operator text (operator 2026-08-18). One short WARN line, only
    # while a reason stands; gcs_status clears it at every FLIGHT START.
    # How old the STATUS ITSELF is — not how long ago we sent it. The console
    # dates the mission by when the beacon line arrived, and this beacon re-reads
    # and re-sends the same file every 5 s forever, so a finished mission kept
    # arriving "2.6 s fresh" while its contents were 32 minutes old: a console
    # opened the next morning showed the last flight sitting at 100 % as though
    # it were happening now (operator 2026-08-18, after a console restart failed
    # to clear it — restarting cannot help, the staleness is on the wire).
    # Sent only once the data stops being current, so a live flight pays nothing.
    # WARN, not INFO: "the mission data stopped updating" is a something-is-wrong
    # line like why=/cam=DEAD, and on a QGC bound to 14550 an INFO line blends in.
    if status_age_s is not None and status_age_s > _STATUS_STALE_S:
        lines.append((_SEV_WARN, f"AAVC stale={int(status_age_s)}"[:_MAX_TEXT]))

    why = (status or {}).get("home_reason_code")
    if why:
        lines.append((_SEV_WARN, f"AAVC why={why}"[:_MAX_TEXT]))

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


def _status_age(status: dict | None, path: Path) -> float | None:
    """Seconds since the mission status was last WRITTEN, or None if unknown.

    Prefers the writer's own ``updated`` stamp over the file mtime: the
    orchestrator writes atomically via rename, and a status copied between
    machines keeps ``updated`` but not the mtime. Falls back to the mtime, then
    to None if even that cannot be read — the beacon then omits the stale line
    rather than inventing an age."""
    if status is None:
        return None
    stamped = status.get("updated")
    if isinstance(stamped, (int, float)):
        return max(0.0, time.time() - float(stamped))
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
    ap.add_argument("--frame", type=Path, default=Path("/tmp/aavc_nadir.jpg"),
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
        try:
            status = _read_status(status_path)
            lines = compose_lines(status, _frame_age(args.frame),
                                  _status_age(status, status_path))
            for sev, text in lines:
                if args.dry_run:
                    print(f"[beacon] sev={sev} {text}", flush=True)
                    continue
                try:
                    mav.mav.statustext_send(sev, text.encode("ascii", "replace")[:_MAX_TEXT])
                except Exception as e:               # link down, router restarting…
                    print(f"[beacon] send failed ({e}) — retrying next tick", flush=True)
                    break
        except Exception as e:
            # One malformed status (a bad eta_s, a stray pads_mapped entry from
            # the sibling writer) must NOT kill the loop: on the 🚀 path the
            # beacon runs unsupervised, so a crash goes radio-silent for the rest
            # of the flight — the one window the operator most needs it.
            print(f"[beacon] tick failed ({e}) — continuing", flush=True)
        time.sleep(args.interval_s)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("[beacon] stopped", flush=True)
