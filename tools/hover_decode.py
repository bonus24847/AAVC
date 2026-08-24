#!/usr/bin/env python3
"""Watch the nadir frames during a hand-flown hover and say whether they READ.

    .venv/bin/python tools/hover_decode.py                 # on the CM4, alongside the grabber
    .venv/bin/python tools/hover_decode.py --no-mavlink    # bench, no vehicle

The open question after G7 attempt 1 is narrow and empirical: the camera is
sharp on the bench (Laplacian 680-780, decoding a 38 cm marker from 1.9 to 14 m)
and produced 41-76 in flight with 0 of 402 frames decoding. Three fixes have
landed since — a held sweep heading, a forced 2 ms exposure, 20 Hz frames — and
none of them has been tested in the air.

This is the instrument for that test. It runs beside the camera grabber, reads
every new frame, and records what the flight core would have got out of it:

  * ``sharpness``  Laplacian variance — the same number the bench measured, so
                   the comparison is like-for-like.
  * ``brightness`` mean level. A 2 ms exposure on a sensor with NO auto-gain can
                   simply underexpose, which looks like "blur" in every metric
                   that is not this one.
  * ``decoded``    what ``find_landing_pads`` returns — the actual question.
  * ``marker_px``  the marker's size on the sensor, so a failure at altitude can
                   be told from a failure at any altitude (400 mm at fx 847 is
                   42 px at 8 m, 28 px at 12 m, 21 px at 16 m).
  * ``agl_m``      altitude at that frame, from the vehicle, so the output is a
                   decode rate PER HEIGHT rather than one number for the flight.

Two outputs, because there are two readers:

  * ``--jsonl`` gets every frame, for the post-flight analysis.
  * ``--summary`` gets a small rolling verdict JSON that ``cm4/status_beacon.py``
    turns into one radio line, so the operator can read it MID-HOVER and change
    height instead of landing, pulling logs and guessing. The verdict is a WORD
    (GOOD / WEAK / BLUR / HIGH / DARK), not a number, because the number needs
    two reference values held in your head and the word does not.

WiFi does not reach the aircraft in flight — the CM4's access point is ON the
airframe — so nothing here streams anywhere. It writes files and the radio
carries the summary.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from vision.detectors.aruco import find_landing_pads  # noqa: E402

DEFAULT_FRAME = Path("/tmp/aavc_nadir.jpg")
DEFAULT_SUMMARY = Path("/tmp/aavc_decode.json")

# Thresholds, with the measurements they come from. They only ever choose the
# WORD; every number is still written to the jsonl, so a bad threshold here can
# never destroy the evidence — it can only mislabel a line on the radio.
#
# SHARP_MIN: the bench decoded at 680-780 and the failed flight scored 41-76.
# 200 sits well above the failure and well below the success, so "sharp enough
# to be worth asking why it did not decode" is a claim about a real gap, not a
# hairline. Anything between roughly 120 and 400 would do the same job.
SHARP_MIN = 200.0
# BRIGHT_MIN: mean level out of 255. The forced 2 ms exposure has NO auto-gain
# behind it (the OV9281 has none), so falling light darkens frames with nothing
# to compensate. Below this the frame is underexposed, which every sharpness
# metric reports as blur — a different fix (CAM_GAIN) from a different cause.
BRIGHT_MIN = 40.0
# Fraction of a window that must decode before the hover is called usable.
GOOD_FRAC = 0.25


def verdict(*, frames: int, decodes: int, sharpness: float,
            brightness: float) -> str:
    """One word for what the last window of frames means, and it is chosen in
    the order the fixes differ, not in the order the numbers are convenient.

    A decode is the ground truth: if frames read, nothing else matters. Only
    when nothing reads does the reason matter, and then DARK comes before BLUR
    because underexposure lowers the sharpness score too and would otherwise be
    misread as motion blur — sending someone to the camera mount when the
    answer is gain.
    """
    if frames == 0:
        return "NOFRAMES"
    if decodes >= max(1, int(frames * GOOD_FRAC)):
        return "GOOD"
    if decodes > 0:
        return "WEAK"
    if brightness < BRIGHT_MIN:
        return "DARK"
    if sharpness < SHARP_MIN:
        return "BLUR"
    return "HIGH"


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class _Altitude:
    """Vehicle AGL, sampled in the background so a frame can be stamped with it.

    TWO sources, because one of them is not always there. ``GLOBAL_POSITION_INT``
    carries ``relative_alt`` (height above home) and is the natural answer — but
    PX4 only publishes it once the GLOBAL position estimate is valid, i.e. once
    GPS has fixed. Measured on the bench 2026-08-24: the link was healthy and
    busy (GPS_RAW_INT, ATTITUDE, LOCAL_POSITION_NED all streaming) with
    **zero** GLOBAL_POSITION_INT, so every frame would have logged
    ``agl_m: null`` and the hover test would have produced decode rates with no
    altitude axis — which is the entire point of it.

    ``LOCAL_POSITION_NED`` needs no GPS: its ``z`` is DOWN from the EKF's local
    origin, so ``-z`` is height above wherever the estimator started, which for
    a hover flown from the ground is the number we want. It is the fallback and
    the source is recorded per frame so the analysis never has to guess which
    one it got.

    Optional on purpose: the bench has no vehicle, and a hover log without
    altitude is still worth having. Failure here degrades the record, it never
    stops it.
    """

    def __init__(self, endpoint: str) -> None:
        self._agl: float | None = None
        self._src: str | None = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, args=(endpoint,),
                                        daemon=True)
        self._thread.start()

    @property
    def agl_m(self) -> float | None:
        return self._agl

    @property
    def source(self) -> str | None:
        """Which message the current reading came from, or None."""
        return self._src

    def stop(self) -> None:
        self._stop = True

    def _run(self, endpoint: str) -> None:
        try:
            from pymavlink import mavutil
            link = mavutil.mavlink_connection(endpoint, source_system=204)
            link.mav.heartbeat_send(6, 8, 0, 0, 0)   # so the router routes to us
        except Exception as exc:                     # noqa: BLE001
            print(f"[hover] no altitude source ({exc}) — frames log agl=null")
            return
        last_beat = 0.0
        announced = False
        while not self._stop:
            try:
                # Guarded: pymavlink 2.4.49 throws on some PX4 1.17 instanced
                # messages and drops whatever arrived with it.
                msg = link.recv_match(
                    type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                    blocking=True, timeout=2)
            except Exception:                        # noqa: BLE001
                continue
            now = time.time()
            if msg is None:
                if now - last_beat > 1.0:            # keep the route alive
                    try:
                        link.mav.heartbeat_send(6, 8, 0, 0, 0)
                        last_beat = now
                    except Exception:                # noqa: BLE001
                        pass
                continue
            kind = msg.get_type()
            if kind == "GLOBAL_POSITION_INT":
                self._agl, self._src = msg.relative_alt / 1000.0, "relative_alt"
            elif self._src != "relative_alt":
                # Only fall back while the better source has never appeared —
                # once GPS fixes mid-flight, relative_alt takes over and keeps it.
                self._agl, self._src = -msg.z, "local_ned"
            if not announced and self._agl is not None:
                announced = True
                print(f"[hover] altitude from {self._src}", flush=True)


def measure(frame_bgr) -> tuple[float, float, list[int], float]:
    """Everything one frame can say: sharpness, brightness, ids, marker size."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 \
        else frame_bgr
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    brightness = float(gray.mean())
    hits = find_landing_pads(frame_bgr)
    ids = [h.marker_id for h in hits if h.marker_id is not None]
    # radius_px is marker-equivalent end to end (one 0.2 m size prior), so
    # doubling it gives the marker's side in pixels.
    px = max((2.0 * h.radius_px for h in hits if h.marker_id is not None),
             default=0.0)
    return sharpness, brightness, ids, px


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frame", type=Path, default=DEFAULT_FRAME)
    ap.add_argument("--jsonl", type=Path,
                    default=Path("/tmp/aavc_hover_decode.jsonl"),
                    help="per-frame record for the post-flight analysis")
    ap.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY,
                    help="rolling verdict the status beacon reads")
    ap.add_argument("--window", type=int, default=200,
                    help="frames the rolling verdict is computed over")
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14550",
                    help="MAVLink endpoint for altitude")
    ap.add_argument("--no-mavlink", action="store_true",
                    help="skip the altitude thread (bench)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long (0 = until killed)")
    args = ap.parse_args()

    alt = None if args.no_mavlink else _Altitude(args.endpoint)
    print(f"[hover] watching {args.frame} -> {args.jsonl} (+ {args.summary})")

    window: list[tuple[int, float, float]] = []      # (decoded, sharp, bright)
    last_mtime = 0.0
    started = time.time()
    total = decoded_total = 0
    try:
        with args.jsonl.open("a") as sink:
            while args.seconds <= 0 or time.time() - started < args.seconds:
                try:
                    mtime = args.frame.stat().st_mtime
                except OSError:
                    time.sleep(0.05)
                    continue
                if mtime == last_mtime:
                    time.sleep(0.01)
                    continue
                last_mtime = mtime
                img = cv2.imread(str(args.frame))
                if img is None:
                    continue
                sharp, bright, ids, px = measure(img)
                total += 1
                if ids:
                    decoded_total += 1
                window.append((1 if ids else 0, sharp, bright))
                if len(window) > args.window:
                    window.pop(0)

                sink.write(json.dumps({
                    "t": round(time.time(), 3),
                    "agl_m": None if alt is None else alt.agl_m,
                    "agl_src": None if alt is None else alt.source,
                    "sharpness": round(sharp, 1),
                    "brightness": round(bright, 1),
                    "ids": ids,
                    "marker_px": round(px, 1),
                }) + "\n")
                sink.flush()

                hits = sum(w[0] for w in window)
                med_sharp = _median([w[1] for w in window])
                med_bright = _median([w[2] for w in window])
                state = verdict(frames=len(window), decodes=hits,
                                sharpness=med_sharp, brightness=med_bright)
                payload = {
                    "verdict": state,
                    "frames": len(window),
                    "decodes": hits,
                    "sharpness": round(med_sharp),
                    "brightness": round(med_bright),
                    "agl_m": None if alt is None else alt.agl_m,
                    "agl_src": None if alt is None else alt.source,
                    "t": round(time.time(), 1),
                }
                tmp = args.summary.with_suffix(".tmp")
                tmp.write_text(json.dumps(payload))
                os.replace(tmp, args.summary)
    except KeyboardInterrupt:
        pass
    finally:
        if alt is not None:
            alt.stop()
    print(f"[hover] {total} frames, {decoded_total} decoded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
