#!/usr/bin/env python3
"""Real-camera grabber for the CM4 — file-only output, mirrors the SITL bridge.

Feeds the flight-core vision pipeline the SAME ``/tmp/aavc_*.png`` frames that
``sitl/gz_camera_bridge.py`` writes in SITL, but from the REAL nadir camera on
the Raspberry Pi CM4. Two pluggable backends, chosen by ``--backend``:

  * ``v4l2``       — USB / UVC webcams via OpenCV (``cv2.VideoCapture``). Runs in
    the project ``.venv`` (opencv-python-headless is a flight dep).
  * ``picamera2``  — CSI-ribbon Pi cameras via libcamera. Run with SYSTEM python3
    (``/usr/bin/python3``); picamera2 is an apt package, not in the ``.venv``.

Contract (identical to ``gz_camera_bridge``): BGR PNG, atomic temp+rename, ``0600``
perms, rate-limited; the NADIR frame mirrors to ``/tmp/aavc_frame.png`` (the
dashboard endpoint + the flight core's contract).

COLOR ORDER: OpenCV (v4l2) yields BGR already; picamera2 RGB888 is converted
RGB->BGR here so downstream code sees the OpenCV convention. NOTE the white/black
ArUco pad is itself R/B-swap-invariant (grayscale decode + a low-S/high-V white
cue), so the pad canNOT reveal a channel-order mistake — VERIFY channel order at
G5 against a COLOURED reference (not the pad); ``--swap-rb`` flips it if wrong.

Usage:
  # USB webcam (venv):     .venv/bin/python sitl/camera_grabber.py --backend v4l2 \
  #       --nadir-device 0
  # CSI Pi cam (system):   /usr/bin/python3 sitl/camera_grabber.py --backend picamera2 \
  #       --nadir-device 0
"""
from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# The Meige OV9281 UVC module: MONO global shutter, 1280x720 @ <=120 fps. The
# 1280 width is the decode requirement (400 mm marker ~18 px @ the 12 m sweep).
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_INTERVAL_S = 0.2   # ~5 Hz file writes; the vision worker reads at ~3 Hz
# Capture rate requested from the driver. The OV9281 can do 120 fps but the
# pipeline only consumes ~5 Hz — a moderate rate keeps USB bandwidth + CPU sane
# while the global shutter still freezes motion per frame. Bench-tune at G5.
DEFAULT_FPS = 50.0


def _to_bgr(frame: np.ndarray) -> np.ndarray:
    """Normalise a captured frame to 3-channel BGR.

    A mono UVC camera (OV9281 with the GREY fourcc) can hand OpenCV a 2-D
    single-channel array; the /tmp PNG contract and the detector both expect
    3 channels (a replicated-gray PNG round-trips identically through
    cv2.imread). 3-channel frames pass through untouched."""
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


class V4l2Backend:
    """USB / UVC camera via OpenCV. ``device`` = index (int) or /dev/videoN (str)."""

    def __init__(self, device: str, width: int, height: int,
                 fps: float | None = None, fourcc: str | None = None) -> None:
        cap_arg: int | str = int(device) if str(device).isdigit() else device
        self._cap = cv2.VideoCapture(cap_arg)
        # ONE-DEEP QUEUE (2026-08-21). V4L2 hands OpenCV a queue of captured
        # frames; reading slower than the sensor produces (5-10 Hz reads vs a
        # 10-120 fps sensor) means every read can return a frame several
        # periods OLD. That is not just latency: vision_worker stamps each
        # decode with the CURRENT telemetry, so a stale frame geolocates the
        # pad where the aircraft ISN'T — at 6 m/s, 3 queued frames is ~1.8 m
        # of pure error fed into the pad registry. Ask the driver for the
        # shallowest queue it will give us; the grab-drain below covers
        # backends that ignore this.
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:      # noqa: BLE001 — not all backends expose it
            pass
        # FOURCC first: switching the pixel format can reset the frame size on
        # some UVC drivers, so size/rate are applied after it. "GREY" selects
        # the OV9281's native mono stream; "MJPG" unlocks high fps on colour cams.
        if fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps and fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._w, self._h = width, height
        if not self._cap.isOpened():
            raise RuntimeError(f"v4l2 device {device!r} did not open")

    def grab(self) -> np.ndarray | None:
        # NOTE: do NOT "drain" the queue with a fixed grab() loop — grab()
        # BLOCKS when no frame is pending, so draining N frames costs N frame
        # periods on an idle queue and makes the grabber slower, not fresher.
        # Freshness comes from the one-deep queue above plus reading at the
        # sensor's own rate (--interval-s ~= 1/fps), which never lets a
        # backlog build in the first place.
        ok, frame = self._cap.read()            # OpenCV returns BGR (or 2-D mono)
        if not ok or frame is None:
            return None
        frame = _to_bgr(frame)
        if frame.shape[1] != self._w or frame.shape[0] != self._h:
            frame = cv2.resize(frame, (self._w, self._h))
        return frame

    def close(self) -> None:
        self._cap.release()


class Picamera2Backend:
    """CSI Pi camera via libcamera / picamera2. ``device`` = camera_num (int)."""

    def __init__(self, device: str, width: int, height: int) -> None:
        from picamera2 import Picamera2  # system-only import; lazy

        self._cam = Picamera2(camera_num=int(device))
        cfg = self._cam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"})
        self._cam.configure(cfg)
        self._cam.start()

    def grab(self) -> np.ndarray | None:
        rgb = self._cam.capture_array()          # RGB
        if rgb is None:
            return None
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)   # -> BGR for the red detector

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:  # noqa: BLE001 — teardown best-effort
            pass


def _make_backend(name: str, device: str, width: int, height: int,
                  fps: float | None = None, fourcc: str | None = None):
    if name == "v4l2":
        return V4l2Backend(device, width, height, fps=fps, fourcc=fourcc)
    if name == "picamera2":
        return Picamera2Backend(device, width, height)
    raise ValueError(f"unknown backend {name!r}")


def _force_short_exposure(device: str, n_100us: int, gain: int) -> None:
    """Force a fixed short exposure (and optionally a fixed gain) on the V4L2
    nadir camera via ``v4l2-ctl`` — the in-flight-blur lever (G7 2026-08-21:
    flight frames scored Laplacian 41-76 vs 680-780 static while the OV9281
    sat in auto_exposure=3 "Aperture Priority" at 16.6 ms; a translating,
    hard-mounted camera needs ~1-3 ms). Subprocess over CAP_PROP
    deliberately: the UVC control names are explicit, they apply to the node
    even while cv2 holds it open, and the readback shows what the driver kept.

    FAIL-SOFT by design: a missing binary/control logs and continues — this
    must never join the nadir open's fail-hard class (auto exposure beats no
    mission). ``device`` may be a bare index ("0"): v4l2-ctl needs a path, so
    it is rewritten to /dev/video<N>. ``n_100us`` is the UVC
    exposure_time_absolute unit (100 µs): 20 = 2 ms; 0 = leave auto exposure
    alone. ``gain`` < 0 = leave the sensor gain alone (this module has NO
    auto-gain, so a short exposure darkens frames with no automatic
    compensation — raise the gain if the bench A/B comes out too dark)."""
    controls: list[str] = []
    if n_100us > 0:
        controls += ["auto_exposure=1", f"exposure_time_absolute={int(n_100us)}"]
    if gain >= 0:
        controls.append(f"gain={int(gain)}")
    if not controls:
        return
    dev = f"/dev/video{device}" if str(device).isdigit() else str(device)
    cmd = ["v4l2-ctl", "-d", dev]
    for ctl in controls:
        cmd += ["-c", ctl]
    names = ",".join(ctl.split("=")[0] for ctl in controls)
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=5.0)
        back = subprocess.run(["v4l2-ctl", "-d", dev, "-C", names],
                              check=False, capture_output=True, text=True,
                              timeout=5.0)
        readback = " ".join((back.stdout or "").split()) or "n/a"
        print(f"[grabber] exposure forced on {dev}: {' '.join(controls)} "
              f"(readback: {readback})")
    except FileNotFoundError:
        print("[grabber] v4l2-ctl not installed — exposure left on AUTO")
    except subprocess.TimeoutExpired:
        print(f"[grabber] v4l2-ctl timed out on {dev} — exposure left on AUTO")
    except subprocess.CalledProcessError as e:
        err = (e.stderr or b"").decode(errors="replace").strip()
        print(f"[grabber] v4l2-ctl failed on {dev} ({err or e}) — "
              "exposure left on AUTO")


def _write_png(bgr: np.ndarray, out_path: Path) -> None:
    """Atomic BGR-PNG write: temp sibling (``.tmp.png`` so cv2 maps the codec),
    ``0600`` perms, then rename. Identical to ``sitl/gz_camera_bridge.py``."""
    tmp = out_path.with_suffix(".tmp.png")
    cv2.imwrite(str(tmp), bgr)
    try:
        tmp.chmod(0o600)
    except OSError:
        pass
    tmp.replace(out_path)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--backend", choices=("v4l2", "picamera2"), required=True)
    ap.add_argument("--nadir-device", default="0",
                    help="nadir camera index/path (control-authority; fails hard)")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS,
                    help="requested driver capture rate (v4l2 only; 0 = driver default)")
    ap.add_argument("--fourcc", default=None,
                    help="v4l2 pixel format, e.g. GREY (OV9281 mono) or MJPG")
    ap.add_argument("--interval-s", type=float, default=DEFAULT_INTERVAL_S,
                    help="per-camera grab/write period (>= the worker's 0.3s read)")
    ap.add_argument("--nadir-out", type=Path, default=Path("/tmp/aavc_nadir.png"))
    ap.add_argument("--frame-out", type=Path, default=Path("/tmp/aavc_frame.png"),
                    help="dashboard endpoint; mirror of the nadir frame")
    ap.add_argument("--no-mirror", action="store_true",
                    help="skip the --frame-out mirror. It is a SECOND full PNG "
                         "encode (measured 62 ms per frame on the CM4 — as much "
                         "as the entire ArUco decode) and only the web dashboard "
                         "reads it; the GCS console and the vision worker both "
                         "use the nadir frame. Use on the real aircraft")
    ap.add_argument("--swap-rb", action="store_true",
                    help="swap R/B on every frame (use if a coloured reference looks wrong)")
    ap.add_argument("--exposure-100us", type=int, default=0,
                    help="v4l2 only: force manual exposure via v4l2-ctl "
                         "(auto_exposure=1 + exposure_time_absolute=N, unit 100 µs "
                         "— 20 = 2 ms). 0 = leave the driver's auto exposure alone. "
                         "Fail-soft when the tool/control is missing")
    ap.add_argument("--gain", type=int, default=-1,
                    help="v4l2 only: fixed sensor gain (OV9281: 0-128, no auto-gain "
                         "— pair with --exposure-100us if frames come out dark). "
                         "-1 = leave unchanged")
    args = ap.parse_args()

    # NADIR is the sole control-authority camera — fail hard if it won't open.
    nadir = _make_backend(args.backend, args.nadir_device, args.width, args.height,
                          fps=args.fps, fourcc=args.fourcc)
    if args.backend == "v4l2":
        _force_short_exposure(args.nadir_device, args.exposure_100us, args.gain)
    mirror = not args.no_mirror
    print(f"[grabber] nadir {args.backend} dev={args.nadir_device} "
          f"-> {args.nadir_out}"
          + (f" (+ {args.frame_out})" if mirror else " (mirror OFF)")
          + f" every {args.interval_s:.2f}s")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))

    def _emit(frame: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if args.swap_rb else frame

    n_ok = n_fail = 0
    last_log = 0.0
    try:
        while not stop["flag"]:
            t0 = time.monotonic()
            frame = nadir.grab()
            if frame is not None:
                frame = _emit(frame)
                _write_png(frame, args.nadir_out)
                if mirror:
                    _write_png(frame, args.frame_out)
                n_ok += 1
            else:
                n_fail += 1
            now = time.monotonic()
            if now - last_log > 2.0:
                print(f"[grabber] nadir ok={n_ok} fail={n_fail}")
                last_log = now
            dt = args.interval_s - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        nadir.close()
        print("[grabber] shutting down")
    return 0


if __name__ == "__main__":
    sys.exit(main())
