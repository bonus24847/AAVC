#!/usr/bin/env python3
"""ArUco scanner for the AAVC touch-and-go mission camera.

Detects the AAVC pad markers — **DICT_4X4_50, IDs 1-6** — from a USB camera
(e.g. the WSD-9781-V12) and reports each marker's ID + pixel center + offset
from frame centre (for visual centring). Built to run **smoothly headless on the
Raspberry Pi CM4**: a threaded grabber always holds the freshest frame (so
detection never lags behind the camera or piles up a buffer), MJPG transport,
and a 1-frame driver buffer.

    python aruco_scan.py                          # headless, prints detections
    python aruco_scan.py --device /dev/video0     # pick the camera
    python aruco_scan.py --web 8090               # + browser view at :8090
    python aruco_scan.py --device /dev/video4 --web 8090   # (laptop test)

Only needs OpenCV (with the aruco module) + the Python stdlib.
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(MARKER_DICT)
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def detect(detector, gray, w, h):
    """Return a list of detected markers: id, pixel centre, and normalised offset
    from the frame centre (dx,dy in [-1,1], right/down positive)."""
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


# ---- shared state for the optional web view --------------------------------
STATE = {"jpeg": None, "markers": [], "fps": 0.0, "size": "?", "device": "?"}
STATE_LOCK = threading.Lock()

PAGE = b"""<!doctype html><meta charset=utf-8><title>ArUco scan</title>
<style>body{margin:0;background:#0e1116;color:#e6e6e6;font-family:system-ui,sans-serif;text-align:center}
h1{font-size:18px;padding:10px;margin:0;background:#161b22}
img{max-width:100%;height:auto;background:#000}
#m{font-size:22px;padding:12px;min-height:30px}.b{display:inline-block;background:#238636;color:#fff;
border-radius:10px;padding:6px 14px;margin:4px;font-weight:700}.none{color:#8b98a5}
#s{font-size:13px;color:#8b98a5;padding:6px}</style>
<h1>ArUco scan (DICT_4X4_50, pads 1-6)</h1>
<img id=cam alt="(camera)">
<div id=m>-</div><div id=s>-</div>
<script>
async function t(){try{let d=await(await fetch('/markers')).json();
 document.getElementById('s').textContent=d.device+' | '+d.size+' | '+d.fps.toFixed(0)+' fps';
 let m=d.markers||[];
 document.getElementById('m').innerHTML=m.length?m.map(x=>'<span class=b>ID '+x.id+(x.pad?'':' (not pad)')+
  ' <small>dx '+x.dx+' dy '+x.dy+'</small></span>').join(''):'<span class=none>no marker</span>';
}catch(e){}}
setInterval(t,300);t();
var cam=document.getElementById('cam');
setInterval(function(){cam.src='/snapshot?'+Date.now();},120);   // ~8 fps, works in any browser
</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/markers":
            with STATE_LOCK:
                body = json.dumps({"markers": STATE["markers"], "fps": STATE["fps"],
                                   "size": STATE["size"], "device": STATE["device"]}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/snapshot"):
            with STATE_LOCK:
                jpg = STATE["jpeg"]
            if jpg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with STATE_LOCK:
                        jpg = STATE["jpeg"]
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(1 / 15.0)          # cap the stream at ~15 fps
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", default="/dev/video0",
                    help="camera device or index (Pi: /dev/video0)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--web", type=int, default=0,
                    help="serve a browser view on this port (0 = off)")
    ap.add_argument("--fps", type=int, default=20,
                    help="max detections/sec — caps CPU on the Pi (0 = uncapped)")
    args = ap.parse_args()

    cam = Camera(args.device, args.width, args.height)
    if not cam.ok:
        print(f"[aruco] ❌ cannot open camera {args.device}")
        return 1
    cam.start()
    print(f"[aruco] camera {args.device} {cam.width}x{cam.height} — dict DICT_4X4_50")
    if args.web:
        srv = ThreadingHTTPServer(("0.0.0.0", args.web), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[aruco] web view -> http://<pi-ip>:{args.web}/")

    det = make_detector()
    last_ids, tprint, last_seq = None, 0.0, -1
    n, t0 = 0, time.time()
    target_dt = 1.0 / args.fps if args.fps > 0 else 0.0
    while True:
        tloop = time.time()
        frame, seq = cam.read()
        if frame is None or seq == last_seq:
            time.sleep(0.003)             # only run detection on NEW frames -> saves Pi CPU
            continue
        last_seq = seq
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, found = detect(det, gray, cam.width, cam.height)
        n += 1
        now = time.time()
        fps = n / (now - t0) if now > t0 else 0.0
        if now - t0 > 2:                          # reset the FPS window
            n, t0 = 0, now

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

        if args.web:
            vis = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with STATE_LOCK:
                    STATE["jpeg"] = buf.tobytes()
                    STATE["markers"] = found
                    STATE["fps"] = fps
                    STATE["size"] = f"{cam.width}x{cam.height}"
                    STATE["device"] = str(args.device)

        if target_dt:                     # rate-limit detection -> steady, low Pi CPU
            rest = target_dt - (time.time() - tloop)
            if rest > 0:
                time.sleep(rest)


if __name__ == "__main__":
    raise SystemExit(main())
