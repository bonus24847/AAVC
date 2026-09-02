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
_SEV_ERR = 3       # MAV_SEVERITY_ERROR — the console's loudest line
# The mission publishes its status continuously while it runs, so a status
# this old ALONGSIDE a stand-down reason means one thing only: the
# orchestrator has ended and is not coming back by itself.
_STOOD_DOWN_S = 45.0

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
                  status_age_s: float | None = None,
                  decode: dict | None = None) -> list[tuple[int, str]]:
    """``[(severity, text)]`` for one beacon tick. Pure — unit-testable without
    a link, a simulator or a camera.

    ``status`` is captures/mission_status.json as the orchestrator writes it
    (phase/assigned/delivered/pads_mapped/updated); ``None`` before the first
    mission of the session. ``frame_age_s`` is the age of the nadir frame, or
    ``None`` when the file does not exist at all.

    ``decode`` is ``tools/hover_decode.py``'s rolling summary, or ``None`` when
    that tool is not running — which is the normal case, so the line simply does
    not appear. The beacon deliberately does NOT decode anything itself: it is
    part of the flight stack, and a JPEG decode per tick would compete with the
    vision worker for the CM4 the whole time the aircraft is flying. It reads a
    file, exactly as it already reads mission_status.json.
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

    # THE ONE LINE THAT WAS MISSING ON 2026-08-29. After the pilot killed the
    # flight the orchestrator stood down for good; the crew swapped the pack and
    # armed twice, and both times PX4 answered "Switching to Offboard is
    # currently not available" — because nothing was publishing setpoints any
    # more. Everything the crew could see said the aircraft was fine: this
    # beacon was still arriving every 5 s, the camera line still read OK, and
    # the console's control bar just sat there waiting. The two facts that
    # spelled it out, `stale=266` and `why=pilot`, were two separate WARN lines
    # among a dozen, and neither says what to DO. Say it, once, at ERROR.
    if (why and status_age_s is not None and status_age_s > _STOOD_DOWN_S):
        lines.append((_SEV_ERR, "AAVC STOOD DOWN - RESTART STACK, THEN ARM"))

    if frame_age_s is None:
        lines.append((_SEV_WARN, "AAVC cam=NONE no frame file"))
    elif frame_age_s > _CAM_DEAD_S:
        lines.append((_SEV_WARN, f"AAVC cam=DEAD {frame_age_s:.0f}s stale"))
    else:
        lines.append((_SEV_INFO, f"AAVC cam=OK {frame_age_s:.1f}s"))

    # `cam=OK` says the grabber is still writing. It does NOT say the pictures
    # are usable, and the difference has already cost a flight: G7 attempt 1 ran
    # a healthy camera for a whole sortie and decoded 0 of 402 frames. This line
    # carries the missing half.
    #
    # It leads with a WORD. The numbers behind it need two reference values held
    # in the operator's head (the bench scored 680-780, the failed flight 41-76)
    # and a verdict needs none — and the four failing words point at four
    # different fixes: fly lower (BLUR), you are simply too high (HIGH), raise
    # the gain (DARK), it is working, remember this height (GOOD). STATUSTEXT is
    # ASCII here, so the sentence is assembled on the console; the radio carries
    # the token.
    verdict = (decode or {}).get("verdict")
    if verdict:
        frames = int(decode.get("frames") or 0)
        decodes = int(decode.get("decodes") or 0)
        sharp = int(decode.get("sharpness") or 0)
        severity = _SEV_INFO if verdict in ("GOOD", "WEAK") else _SEV_WARN
        lines.append((severity,
                      f"AAVC cam={verdict} dec={decodes}/{frames} "
                      f"sh={sharp}"[:_MAX_TEXT]))
    return lines


def _read_status(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


_DECODE_STALE_S = 30.0


def _read_decode(path: Path) -> dict | None:
    """``tools/hover_decode.py``'s rolling summary, or None.

    Missing is the normal case and must be silent — that tool runs only for a
    deliberate decode test. A STALE file is treated as missing too: the summary
    is rewritten every frame while the tool lives, so anything older than
    ``_DECODE_STALE_S`` is a verdict about a moment that has passed, and a radio
    line saying GOOD about thirty seconds ago is worse than no line at all.
    """
    try:
        age = time.time() - path.stat().st_mtime
        if age > _DECODE_STALE_S:
            return None
        return json.loads(path.read_text())
    except Exception:                                # noqa: BLE001
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


def send_lines(mav: object, lines: list[tuple[int, str]], *,
               dry_run: bool, pace_s: float) -> int:
    """Send one tick's STATUSTEXTs, SPACED — returns how many went out.

    A tick composes 6-8 lines and used to hand them all to the radio in one
    go: ~8 x 60 B into TELEM1's 1200 B/s budget is a third of a second of
    solid air every 5 s, dumped into the ground unit's TX buffer where it sits
    AHEAD of whatever the operator presses next. The console's own notes
    already record this link flapping under load, and the servo-release press
    that took 8 s to move a latch was measured on exactly this air. The gap
    costs nothing: the tick still finishes ~1 s into a 5 s period.

    A failed send breaks the tick rather than retrying — the next tick carries
    the same summary 5 s later, so nothing is lost by giving up early on a
    link that is down."""
    sent = 0
    for i, (sev, text) in enumerate(lines):
        if dry_run:
            print(f"[beacon] sev={sev} {text}", flush=True)
            sent += 1
            continue
        if i and pace_s > 0:
            time.sleep(pace_s)
        try:
            mav.mav.statustext_send(  # type: ignore[attr-defined]
                sev, text.encode("ascii", "replace")[:_MAX_TEXT])
            sent += 1
        except Exception as e:               # link down, router restarting…
            print(f"[beacon] send failed ({e}) — retrying next tick", flush=True)
            break
    return sent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14550",
                    help="mavlink-router endpoint to inject into "
                         "(default: the qgc server port on the CM4)")
    ap.add_argument("--captures", type=Path, default=Path("captures"),
                    help="dir holding mission_status.json (default: ./captures)")
    ap.add_argument("--frame", type=Path, default=Path("/tmp/aavc_nadir.jpg"),
                    help="nadir frame the camera grabber writes")
    ap.add_argument("--decode-summary", type=Path,
                    default=Path("/tmp/aavc_decode.json"),
                    help="tools/hover_decode.py's rolling verdict; the "
                         "cam=<VERDICT> line is omitted when it is absent or "
                         "stale, which is the normal flight case")
    ap.add_argument("--interval-s", type=float, default=5.0)
    ap.add_argument("--pace-s", type=float, default=0.15,
                    help="gap between the STATUSTEXTs of one tick (default 0.15 s) "
                         "so a tick trickles instead of bursting into the radio's "
                         "TX buffer; 0 restores the old all-at-once behaviour")
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
                                  _status_age(status, status_path),
                                  _read_decode(args.decode_summary))
            send_lines(mav, lines, dry_run=args.dry_run, pace_s=args.pace_s)
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
