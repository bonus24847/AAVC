#!/usr/bin/env python3
"""Synthetic camera for HITL — feeds the REAL vision pipeline when there are no
gz cameras.

HITL runs on jMAVSim (gz Harmonic can't drive a real FC), which renders NO cameras.
The AAVC V1.3 mission needs nadir frames to discover the ArUco landing
pads, so this stand-in watches the flight controller's position over MAVLink and,
whenever the vehicle is over (or approaching) a KNOWN pad, pastes the REAL pad
face (vision.detectors.aruco.render_pad_bgr — the same renderer that bakes the
SITL textures) scaled to the vehicle's altitude into ``/tmp/aavc_nadir.png`` /
(nadir only) — otherwise it writes plain ground. The unmodified
detector (`vision.detectors.aruco`) + projection + tracker then run on the real
FC's telemetry, so HITL validates the whole decode→confirm→serve→release SEQUENCE
on real hardware, including the id-verified LAND gate.

SCOPE (be honest): the pad is drawn CENTERED when within the lock radius, so
this exercises the mission sequence + timing, NOT the lateral align-loop
magnitude or projection precision (those are validated in gz-SITL with real
rendering, and on real cameras at the G6 tethered gate). See docs/HITL.md.

Run with the SAME interpreter as the camera bridge (system python3 is fine — needs
opencv + numpy + pymavlink). Point --mavlink at a telemetry feed that is SEPARATE
from the orchestrator's offboard link (e.g. a dedicated mavlink-router UDP endpoint)
so the two don't fight over packets.

Usage:
  python3 sitl/hitl_synthetic_camera.py \
      --mavlink udpin:0.0.0.0:14541 --targets /tmp/aavc_targets.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from vision.detectors.aruco import render_pad_bgr  # noqa: E402

# Match the SITL camera intrinsics so the detector/projection behave identically
# (nadir = the single decode camera — kept in sync with sitl/aavc_config.yaml
# cameras: and the x500_mono_cam model).
FOV_RAD = 1.74
WIDTH_PX = 1280
HEIGHT_PX = 720
FX = (WIDTH_PX / 2.0) / math.tan(FOV_RAD / 2.0)

GROUND_BGR = (45, 110, 60)   # grass green — low-V so the white-pad cue can't fire
PAD_SIZE_M = 1.0             # rules Fig. 6: 1x1 m white pad

NADIR = Path("/tmp/aavc_nadir.png")
FRAME_MIRROR = Path("/tmp/aavc_frame.png")   # dashboard camera endpoint
# How long the MAVLink position feed may go quiet before this camera stops
# writing. Must stay well under the orchestrator's own frame-age gate
# (vision_worker.DEFAULT_FRAME_MAX_AGE_S) so the gate is what fires, not this.
POS_MAX_AGE_S = 2.0


def _ground_frame(w: int = WIDTH_PX, h: int = HEIGHT_PX) -> np.ndarray:
    img = np.full((h, w, 3), GROUND_BGR, dtype=np.uint8)
    # a little texture so it doesn't look like a flat test card (harmless to detector)
    noise = np.random.randint(0, 18, (h, w, 1), dtype=np.uint8)
    return cv2.subtract(img, noise.repeat(3, axis=2))


def _target_frame(pad_px: int, yaw_deg: float, marker_id: int,
                  w: int = WIDTH_PX, h: int = HEIGHT_PX) -> np.ndarray:
    """The assigned landing pad centred in the frame — the AAVC V1.3 target.

    ``pad_px`` is the 1 m pad's size in pixels at the vehicle's altitude;
    ``yaw_deg`` rotates it (decode must be rotation-invariant); ``marker_id``
    picks the real DICT_4X4_50 face via render_pad_bgr."""
    img = _ground_frame(w, h)
    px = max(16, min(pad_px, h - 8))
    pad = cv2.resize(render_pad_bgr(marker_id, 512), (px, px),
                     interpolation=cv2.INTER_AREA)
    if yaw_deg:
        # Rotate INTO AN ENLARGED CANVAS so the pad's corners are not clipped —
        # a tight px×px warp fills the rotated square's corners with ground and
        # turns it into an octagon (~29% area lost at 45°), which breaks the
        # detector's white-square blob cue exactly on the non-axis-aligned pads.
        rad = math.radians(yaw_deg % 180.0)
        side = int(math.ceil(px * (abs(math.cos(rad)) + abs(math.sin(rad)))))
        m = cv2.getRotationMatrix2D((px / 2.0, px / 2.0), yaw_deg, 1.0)
        m[0, 2] += (side - px) / 2.0
        m[1, 2] += (side - px) / 2.0
        pad = cv2.warpAffine(pad, m, (side, side), borderValue=GROUND_BGR)
        px = side
    cx, cy = w // 2, h // 2
    x0, y0 = cx - px // 2, cy - px // 2
    # Clip to the frame (the enlarged/low-altitude pad may exceed the border).
    xa, ya, xb, yb = max(0, x0), max(0, y0), min(w, x0 + px), min(h, y0 + px)
    if xb > xa and yb > ya:
        img[ya:yb, xa:xb] = pad[ya - y0:yb - y0, xa - x0:xb - x0]
    return img


def _atomic_write(path: Path, img: np.ndarray) -> None:
    # Encode by FORMAT (path.suffix), then write raw bytes to the temp file — a
    # ".tmp"-suffixed path has no extension cv2.imwrite can map to a codec.
    ok, buf = cv2.imencode(path.suffix, img)
    if not ok:
        return
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(buf.tobytes())
    tmp.chmod(0o600)        # match camera_grabber / gz_camera_bridge (2026-06-13)
    os.replace(tmp, path)   # atomic — readers never see a half-written PNG


def _load_targets(path: Path) -> list[tuple[float, float, int]]:
    data = json.loads(path.read_text())
    # marker_id is REQUIRED (V1.3 truth always carries it) — a missing key means
    # a stale/pre-V1.3 file; defaulting it to 1 would silently render EVERY pad
    # as id 1 and defeat the id-verified LAND gate this rig exists to validate.
    return [(float(t["lat"]), float(t["lon"]), int(t["marker_id"]))
            for t in data.get("targets", [])]


def _ground_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = (lat2 - lat1) * 111_320.0
    de = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def main() -> int:
    ap = argparse.ArgumentParser(description="HITL synthetic nadir camera")
    ap.add_argument("--mavlink", default="udpin:0.0.0.0:14541",
                    help="telemetry feed (separate from the orchestrator's 14540 link)")
    ap.add_argument("--targets", default="/tmp/aavc_targets.json")
    ap.add_argument("--interval", type=float, default=0.2)
    ap.add_argument("--lock-radius-m", type=float, default=8.0,
                    help="draw the target in NADIR within this ground distance")
    ap.add_argument("--pad-size-m", type=float, default=PAD_SIZE_M,
                    help="physical pad side (m) → pixel size scales with altitude")
    args = ap.parse_args()

    try:
        targets = _load_targets(Path(args.targets))
    except Exception as e:  # noqa: BLE001 — startup; report and bail
        print(f"[hitl-cam] cannot read targets {args.targets}: {e}", file=sys.stderr)
        return 2
    if not targets:
        print(f"[hitl-cam] no targets in {args.targets}", file=sys.stderr)
        return 2
    print(f"[hitl-cam] {len(targets)} targets; feed={args.mavlink}; "
          f"lock<{args.lock_radius_m}m nadir")

    from pymavlink import mavutil
    mav = mavutil.mavlink_connection(args.mavlink)
    print("[hitl-cam] waiting for first GLOBAL_POSITION_INT…")

    last_pos: tuple[float, float, float] | None = None
    last_pos_at = 0.0
    last_log = 0.0
    stale_warned = False
    while True:
        # drain all pending position msgs, keep the latest (non-blocking).
        # Wrapped because pymavlink 2.4.49 raises TypeError out of its
        # instanced-message bookkeeping on some PX4 1.17 messages — unguarded,
        # one such message kills the camera mid-run.
        while True:
            try:
                m = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            except TypeError:
                break        # leave the drain; the next tick retries after a sleep
            if m is None:
                break
            last_pos = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
            last_pos_at = time.time()

        # A dead position feed must STOP the frames, not keep painting the last
        # known one. Every write refreshes the file's mtime, which is exactly what
        # the orchestrator's frame-staleness gate reads — so a camera that keeps
        # writing after its input died defeats that gate on the very rig built to
        # exercise it, and the vision worker would go on decoding a pad the
        # aircraft has long since flown away from.
        if last_pos is not None and time.time() - last_pos_at > POS_MAX_AGE_S:
            if not stale_warned:
                print(f"[hitl-cam] position feed silent >{POS_MAX_AGE_S:.0f}s — "
                      "holding frames stale so the vision gate can see it")
                stale_warned = True
            time.sleep(args.interval)
            continue
        stale_warned = False

        if last_pos is None:
            ground = _ground_frame()
            _atomic_write(NADIR, ground)
            _atomic_write(FRAME_MIRROR, ground)   # the dashboard view, too
            time.sleep(args.interval)
            continue

        vlat, vlon, valt = last_pos
        alt = max(valt, 0.5)
        dists = [_ground_dist_m(vlat, vlon, tlat, tlon)
                 for tlat, tlon, _mid in targets]
        idx = min(range(len(dists)), key=dists.__getitem__)
        nearest = dists[idx]
        marker_id = targets[idx][2]
        pad_px = int(args.pad_size_m * FX / alt)
        yaw = float((idx * 47) % 180)            # stable, varied heading per pad

        nadir = (_target_frame(pad_px, yaw, marker_id)
                 if nearest <= args.lock_radius_m else _ground_frame())
        _atomic_write(NADIR, nadir)
        _atomic_write(FRAME_MIRROR, nadir)   # dashboard view (atomic + 0600)

        now = time.time()
        if now - last_log > 2.0:
            tag = f"PAD {marker_id}" if nearest <= args.lock_radius_m else "ground"
            print(f"[hitl-cam] alt={alt:5.1f}m nearest={nearest:6.1f}m "
                  f"pad={pad_px}px {tag}")
            last_log = now
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
