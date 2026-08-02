#!/usr/bin/env python3
"""ArUco scanner for the AAVC touch-and-go mission camera.

Detects the AAVC pad markers — **DICT_4X4_50, IDs 1-6** — from a USB camera
(e.g. the WSD-9781-V12) and reports each marker's ID + pixel centre + normalised
offset from the frame centre (for visual centring during landing).

Built to run **smoothly headless on the Raspberry Pi CM4** (the Pixhawk
companion):
- a **threaded grabber** always holds the freshest frame, so detection never
  lags behind the camera or piles up a buffer — the key to a low-latency feed;
- **MJPG** transport + a **1-frame** driver buffer + a 30 fps camera cap;
- a **`--fps` detection cap** so CPU stays low and steady on the Pi;
- the USB camera's `/dev/videoN` number is **auto-detected** (it isn't stable
  across reboots on the Pi), so no reconfiguring after a power-cycle.

    python aruco_scan.py                       # auto-detect camera, print detections
    python aruco_scan.py --device /dev/video1  # force a specific camera node
    python aruco_scan.py --fps 15              # lower the CPU budget

Only needs OpenCV (with the aruco module) + the Python standard library.
"""
import argparse
import glob
import threading
import time

import cv2

MARKER_DICT = cv2.aruco.DICT_4X4_50      # AAVC pads
PAD_IDS = range(1, 7)                    # valid pad IDs 1..6


class Camera:
    """Threaded frame grabber. Reads the camera in its own thread and keeps only
    the newest frame; `read()` returns that. This decouples slow USB I/O from the
    detection loop so nothing stalls and no stale frames queue up — the key to a
    smooth, low-latency feed on the Pi."""

    def __init__(self, device, width, height):
        # index (e.g. 0) or path (e.g. /dev/video0) both work with V4L2
        dev = int(device) if str(device).isdigit() else device
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        # MJPG => the USB2 cam sends compressed frames => far higher FPS than raw YUYV
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # minimise latency
        self.cap.set(cv2.CAP_PROP_FPS, 30)           # don't let the cam run 60 fps
        self.ok = self.cap.isOpened()
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._frame = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = False

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _loop(self):
        while not self._stop:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.03)
                continue
            with self._lock:
                self._frame = f
                self._seq += 1

    def read(self):
        with self._lock:
            return self._frame, self._seq

    def release(self):
        self._stop = True
        time.sleep(0.05)
        self.cap.release()


def _probe(device):
    """True if `device` opens AND yields one real frame — i.e. a live capture node, not a
    metadata/codec node (bcm2835-isp etc.) that merely opens."""
    dev = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    ok = bool(cap.isOpened() and cap.read()[0])
    cap.release()
    return ok


def find_camera(preferred):
    """Return a device path that actually delivers frames. A USB camera's /dev/videoN number is
    NOT stable across reboots on the Pi (it comes up video0 one boot, video1 the next, …), so if
    `preferred` isn't delivering, scan the low /dev/video* nodes and take the first that does."""
    if _probe(preferred):
        return preferred
    for d in sorted(glob.glob("/dev/video[0-9]"),
                    key=lambda p: int(p.rsplit("video", 1)[1])):
        if d != preferred and _probe(d):
            return d
    return None


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(MARKER_DICT)
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def detect(detector, gray, w, h):
    """Return `(corners, ids, found)` where `found` is a list of detected markers — each with its
    id, pixel centre, and normalised offset from the frame centre (dx,dy in [-1,1], right/down
    positive). Importable for mission integration: feed dx,dy to the centring loop."""
    corners, ids, _ = detector.detectMarkers(gray)
    found = []
    if ids is not None:
        for c, mid in zip(corners, ids.flatten()):
            p = c.reshape(-1, 2)
            cx, cy = float(p[:, 0].mean()), float(p[:, 1].mean())
            found.append({
                "id": int(mid),
                "cx": round(cx, 1), "cy": round(cy, 1),
                "dx": round((cx - w / 2) / (w / 2), 3),   # -1 left .. +1 right
                "dy": round((cy - h / 2) / (h / 2), 3),   # -1 top  .. +1 down
                "pad": int(mid) in PAD_IDS,
            })
    return corners, ids, found


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video0",
                    help="camera device or index; auto-detected if this one has no frames")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=20,
                    help="max detections/sec — caps CPU on the Pi (0 = uncapped)")
    args = ap.parse_args()

    dev = find_camera(args.device)
    if dev is None:
        print(f"[aruco] ❌ no working camera (tried {args.device} + /dev/video0-9)")
        return 1
    if str(dev) != str(args.device):
        print(f"[aruco] {args.device} has no frames → auto-detected {dev}")
    cam = Camera(dev, args.width, args.height)
    if not cam.ok:
        print(f"[aruco] ❌ cannot open camera {dev}")
        return 1
    cam.start()
    print(f"[aruco] camera {dev} {cam.width}x{cam.height} — dict DICT_4X4_50, pads 1-6")

    det = make_detector()
    last_ids, tprint, last_seq = None, 0.0, -1
    target_dt = 1.0 / args.fps if args.fps > 0 else 0.0
    while True:
        tloop = time.time()
        frame, seq = cam.read()
        if frame is None or seq == last_seq:
            time.sleep(0.003)             # only run detection on NEW frames -> saves Pi CPU
            continue
        last_seq = seq
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        _corners, _ids, found = detect(det, gray, cam.width, cam.height)

        now = time.time()
        cur = tuple(sorted(m["id"] for m in found))
        if cur != last_ids or (found and now - tprint > 1.0):
            if found:
                print("[aruco] " + "  ".join(
                    f"ID {m['id']}{'' if m['pad'] else '(not pad)'} "
                    f"@({m['cx']:.0f},{m['cy']:.0f}) dx{m['dx']:+.2f} dy{m['dy']:+.2f}"
                    for m in found), flush=True)
            elif last_ids:
                print("[aruco] (no marker)", flush=True)
            last_ids, tprint = cur, now

        if target_dt:                     # rate-limit detection -> steady, low Pi CPU
            rest = target_dt - (time.time() - tloop)
            if rest > 0:
                time.sleep(rest)


if __name__ == "__main__":
    raise SystemExit(main())
