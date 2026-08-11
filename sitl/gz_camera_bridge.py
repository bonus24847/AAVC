#!/usr/bin/python3
"""Gazebo camera bridge — single onboard camera + spectator, file-only output.

Subscribes to the onboard NADIR camera on the x500_mono_cam model AND the
static world spectator camera, writing BGR PNGs:

  NADIR     topic `camera`           -> /tmp/aavc_nadir.png  AND  /tmp/aavc_frame.png
  SPECTATOR topic `spectator_camera` -> /tmp/aavc_spectator.png

The nadir frame is mirrored to /tmp/aavc_frame.png because that is the
dashboard camera endpoint (the flight core's contract). The spectator camera
is a fixed third-person ground camera (sitl/models/spectator_cam) that watches
the launch pad / sys-ID hover column for the Tuning view's right rail; it is
best-effort (a world without it just gets no spectator frames).

No GStreamer / H.264 / RTP — this bridge writes PNG files only. The flight
core's vision_worker reads the files; the dashboard serves /tmp/aavc_frame.png.

INVOKE WITH /usr/bin/python3 (NOT .venv/bin/python). `python3-gz-transport13`
is an apt-installed Debian package at /usr/lib/python3/dist-packages/gz/, not
visible from the project's pip .venv.

Topics: confirmed via `gz topic -l` on Gazebo Harmonic + PX4 v1.15.4 — PX4's
gz_bridge config trims the verbose world/model path to plain `/camera`. If
your gz/PX4 version drifts, run `gz topic -l | grep image` and pass
--nadir-topic.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node

DEFAULT_NADIR_TOPIC = "/camera"
DEFAULT_SPECTATOR_TOPIC = "/spectator_camera"

# Rate limit ~10 Hz per camera. The gz cameras publish at 30 Hz; the vision
# pipeline reads the latest file far slower than 10 Hz, so this is plenty.
DEFAULT_MIN_INTERVAL_S = 0.1

_state: dict = {
    "nadir_rx": 0, "nadir_file": 0,
    "spectator_rx": 0, "spectator_file": 0,
    "nadir_last_ts": 0.0, "spectator_last_ts": 0.0,
}
_lock = threading.Lock()


def _decode(msg: GzImage) -> np.ndarray:
    """Decode gz.msgs.Image (RGB_INT8, pixel_format_type=3) to BGR ndarray."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)


def _write_png(bgr: np.ndarray, out_path: Path) -> None:
    """Atomic-ish PNG write: write to a temp sibling, set owner-only perms, then
    rename. 0o600 + /tmp's sticky bit stops another local user reading or
    swapping the frame the flight loop / dashboard trust (defense-in-depth)."""
    tmp = out_path.with_suffix(".tmp.png")
    cv2.imwrite(str(tmp), bgr)
    try:
        tmp.chmod(0o600)
    except OSError:
        pass
    tmp.replace(out_path)


def _make_nadir_cb(nadir_path: Path, frame_path: Path, min_interval_s: float,
                   mono: bool = False):
    """Callback for the NADIR camera: write /tmp/aavc_nadir.png AND mirror to
    the dashboard endpoint /tmp/aavc_frame.png. ``mono=True`` collapses the gz
    RGB render to replicated-gray (R=G=B) before writing — SITL fidelity for
    the real OV9281 MONO sensor, so a G4 run exercises the mono decode path."""
    def cb(msg: GzImage) -> None:
        with _lock:
            _state["nadir_rx"] += 1
            now = time.monotonic()
            if now - _state["nadir_last_ts"] < min_interval_s:
                return
            try:
                bgr = _decode(msg)
                if mono:
                    bgr = cv2.cvtColor(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY),
                                       cv2.COLOR_GRAY2BGR)
                _write_png(bgr, nadir_path)
                _write_png(bgr, frame_path)
                _state["nadir_last_ts"] = now
                _state["nadir_file"] += 1
            except Exception as e:
                print(f"[bridge] nadir write failed: {e}", file=sys.stderr)
    return cb


def _make_spectator_cb(spectator_path: Path, min_interval_s: float):
    """Callback for the SPECTATOR camera: write /tmp/aavc_spectator.png (the
    fixed third-person view for the Tuning right rail)."""
    def cb(msg: GzImage) -> None:
        with _lock:
            _state["spectator_rx"] += 1
            now = time.monotonic()
            if now - _state["spectator_last_ts"] < min_interval_s:
                return
            try:
                bgr = _decode(msg)
                _write_png(bgr, spectator_path)
                _state["spectator_last_ts"] = now
                _state["spectator_file"] += 1
            except Exception as e:
                print(f"[bridge] spectator write failed: {e}", file=sys.stderr)
    return cb


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nadir-out", type=Path, default=Path("/tmp/aavc_nadir.png"),
                    help="NADIR PNG path (default: /tmp/aavc_nadir.png)")
    ap.add_argument("--frame-out", type=Path, default=Path("/tmp/aavc_frame.png"),
                    help="Dashboard endpoint; mirror of the nadir frame "
                         "(default: /tmp/aavc_frame.png)")
    ap.add_argument("--spectator-out", type=Path, default=Path("/tmp/aavc_spectator.png"),
                    help="SPECTATOR PNG path; fixed third-person view for the "
                         "Tuning UI (default: /tmp/aavc_spectator.png)")
    ap.add_argument("--nadir-topic", default=DEFAULT_NADIR_TOPIC,
                    help=f"gz nadir camera topic (default: {DEFAULT_NADIR_TOPIC})")
    ap.add_argument("--spectator-topic", default=DEFAULT_SPECTATOR_TOPIC,
                    help=f"gz spectator camera topic (default: {DEFAULT_SPECTATOR_TOPIC})")
    ap.add_argument("--min-interval-s", type=float, default=DEFAULT_MIN_INTERVAL_S,
                    help="Per-camera write rate limit (>=0.1s = <=10 Hz).")
    ap.add_argument("--mono", action="store_true",
                    help="write the nadir frame as replicated-gray (R=G=B) — "
                         "matches the real OV9281 MONO sensor for a fidelity run")
    args = ap.parse_args()

    node = Node()

    ok_nadir = node.subscribe(
        GzImage, args.nadir_topic,
        _make_nadir_cb(args.nadir_out, args.frame_out, args.min_interval_s,
                       mono=args.mono),
    )
    ok_spectator = node.subscribe(
        GzImage, args.spectator_topic,
        _make_spectator_cb(args.spectator_out, args.min_interval_s),
    )

    if not ok_nadir:
        print(f"[bridge] subscribe failed for nadir topic={args.nadir_topic}",
              file=sys.stderr)
        print("  Hint: `gz topic -l | grep image` while SITL is running.",
              file=sys.stderr)
        return 2
    if not ok_spectator:
        # Spectator is best-effort (Tuning view only); a flight-only world may
        # not include the camera. Warn but keep the nadir feed alive.
        print(f"[bridge] WARNING: subscribe failed for spectator "
              f"topic={args.spectator_topic}; no spectator feed.", file=sys.stderr)

    print(f"[bridge] subscribed: nadir={args.nadir_topic} "
          f"-> {args.nadir_out} (+ {args.frame_out}); "
          f"spectator={args.spectator_topic} -> {args.spectator_out} "
          f"(rate-limit >= {args.min_interval_s}s)")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    try:
        while not stop.is_set():
            time.sleep(2.0)
            with _lock:
                print(f"[bridge] nadir rx={_state['nadir_rx']} "
                      f"file={_state['nadir_file']} | "
                      f"spectator rx={_state['spectator_rx']} "
                      f"file={_state['spectator_file']}")
    finally:
        print("[bridge] shutting down")
    return 0


if __name__ == "__main__":
    sys.exit(main())
