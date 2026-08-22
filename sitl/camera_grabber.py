#!/usr/bin/env python3
"""Real-camera grabber for the CM4 — file-only output, mirrors the SITL bridge.

Feeds the flight-core vision pipeline the SAME ``/tmp/aavc_*.jpg`` frames that
``sitl/gz_camera_bridge.py`` writes in SITL, but from the REAL nadir camera on
the Raspberry Pi CM4. Two pluggable backends, chosen by ``--backend``:

  * ``v4l2``       — USB / UVC webcams via OpenCV (``cv2.VideoCapture``). Runs in
    the project ``.venv`` (opencv-python-headless is a flight dep).
  * ``picamera2``  — CSI-ribbon Pi cameras via libcamera. Run with SYSTEM python3
    (``/usr/bin/python3``); picamera2 is an apt package, not in the ``.venv``.

Contract (identical to ``gz_camera_bridge``): BGR frame, atomic temp+rename, ``0600``
perms, rate-limited; the NADIR frame mirrors to ``/tmp/aavc_frame.jpg`` (the
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
DEFAULT_INTERVAL_S = 0.1   # 10 Hz file writes = the sensor's own rate at
                           # 1280x720; the vision worker decodes every new one
# Capture rate requested from the driver. The OV9281 can do 120 fps but the
# pipeline consumes every frame written — a moderate rate keeps USB + CPU sane
# while the global shutter still freezes motion per frame. Bench-tune at G5.
DEFAULT_FPS = 50.0


def _to_bgr(frame: np.ndarray) -> np.ndarray:
    """Normalise a captured frame to 3-channel BGR.

    A mono UVC camera (OV9281 with the GREY fourcc) can hand OpenCV a 2-D
    single-channel array; the /tmp frame contract and the detector both expect
    3 channels (a replicated-gray frame round-trips identically through
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


class MjpegPassthroughBackend:
    """UVC camera in MJPG mode, handing back the camera's OWN JPEG bytes.

    The normal path costs three codec passes per frame — the camera compresses,
    OpenCV decodes to BGR, we re-encode to write the file — and the middle two
    are pure waste when the consumer wants a JPEG anyway. With
    ``CAP_PROP_CONVERT_RGB=0`` the V4L2 backend returns the raw MJPEG buffer,
    so the grabber becomes a file write: measured on the CM4, 121 fps captured
    (vs 8.7 for YUYV) and the encode cost gone entirely. The frame on disk is
    then EXACTLY what the sensor produced — no double compression to soften the
    marker's cell edges.

    ``grab_bytes`` returns the JPEG payload; callers that need pixels decode it
    themselves. Trimmed at the EOI marker because the driver pads the buffer."""

    def __init__(self, device: str, width: int, height: int,
                 fps: float | None = None) -> None:
        cap_arg: int | str = int(device) if str(device).isdigit() else device
        self._cap = cv2.VideoCapture(cap_arg)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps and fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, fps)
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:      # noqa: BLE001
            pass
        if not self._cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
            self._cap.release()
            raise RuntimeError(
                "this OpenCV/V4L2 backend will not hand back raw MJPEG "
                "(CAP_PROP_CONVERT_RGB=0 refused) — drop --mjpeg-passthrough")
        if not self._cap.isOpened():
            raise RuntimeError(f"v4l2 device {device!r} did not open")

    def verify_resolution(self) -> tuple[int, int] | None:
        """Decode ONE frame and return its true (w, h), or None if unreadable.

        The plain V4L2 path resizes when the driver ignores the requested size;
        passthrough cannot, because it never decodes. That silence is
        dangerous: vision/projection derives BOTH the focal length and the
        principal point from the CONFIGURED width/height, and nothing
        downstream ever looks at the real frame. A camera that only offers
        MJPG at 1920x1080 would make fx 1.5x wrong and the centre off by
        (320, 180) px — every pixel->lat/lon silently scaled and shifted, with
        the size prior and the accept radius both wide enough to let it
        through (2026-08-21 review)."""
        payload = self.grab_bytes()
        if payload is None:
            return None
        img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        return int(img.shape[1]), int(img.shape[0])

    def grab_bytes(self) -> bytes | None:
        ok, raw = self._cap.read()
        if not ok or raw is None:
            return None
        buf = raw.reshape(-1).tobytes()
        if buf[:2] != b"\xff\xd8":       # not a JPEG — the driver converted
            return None
        end = buf.rfind(b"\xff\xd9")     # strip the driver's zero padding
        return buf[:end + 2] if end > 0 else buf

    def grab(self) -> np.ndarray | None:
        payload = self.grab_bytes()
        if payload is None:
            return None
        return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)

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


def _write_frame(bgr: np.ndarray, out_path: Path, jpeg_quality: int = 95) -> None:
    """Atomic BGR frame write: temp sibling carrying the SAME suffix (cv2 maps
    the codec from it), ``0600`` perms, then rename. Identical to
    ``sitl/gz_camera_bridge.py``.

    The suffix chooses the codec, and that choice is the pipeline's single
    biggest CPU knob — measured on the CM4 at 1280x720: PNG encodes in 48 ms
    and decodes in 33, JPEG q95 in 12 and 15, for a file 280 KB -> 62 KB (which
    also shrinks the WiFi frame sync). q95 costs 0.5 ms over q85 and keeps the
    marker's black/white edges crisp, so the quality is spent where the decode
    needs it."""
    suffix = out_path.suffix or ".jpg"
    tmp = out_path.with_suffix(f".tmp{suffix}")
    params = ([int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
              if suffix.lower() in (".jpg", ".jpeg") else [])
    cv2.imwrite(str(tmp), bgr, params)
    try:
        tmp.chmod(0o600)
    except OSError:
        pass
    tmp.replace(out_path)


def _write_bytes(payload: bytes, out_path: Path) -> None:
    """Atomic write of an ALREADY-encoded frame (the camera's own JPEG).

    Same temp+chmod+rename contract as _write_frame — readers must never see a
    half-written frame — but with no codec pass at all."""
    tmp = out_path.with_suffix(f".tmp{out_path.suffix or '.jpg'}")
    tmp.write_bytes(payload)
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
    ap.add_argument("--nadir-out", type=Path, default=Path("/tmp/aavc_nadir.jpg"))
    ap.add_argument("--frame-out", type=Path, default=Path("/tmp/aavc_frame.jpg"),
                    help="dashboard endpoint; mirror of the nadir frame")
    ap.add_argument("--jpeg-quality", type=int, default=95,
                    help="JPEG quality when the output path ends in .jpg "
                         "(ignored for .png). 95 keeps the marker's edges "
                         "crisp and costs 0.5 ms more than 85 on the CM4")
    ap.add_argument("--mjpeg-passthrough", action="store_true",
                    help="v4l2 only: capture in MJPG and write the camera's OWN "
                         "JPEG bytes — no decode, no re-encode. CM4-measured "
                         "121 fps captured (YUYV manages 8.7) and the encode "
                         "cost disappears; the file is exactly what the sensor "
                         "produced, so nothing is compressed twice. ⚠ the "
                         "camera picks the JPEG quality, so VALIDATE a real "
                         "marker decodes from these frames before flying it. "
                         "Incompatible with --swap-rb (no pixels to swap)")
    ap.add_argument("--no-mirror", action="store_true",
                    help="skip the --frame-out mirror. It is a SECOND full "
                         "encode (CM4-measured: 12 ms per frame as JPEG q95, "
                         "62 as PNG) and only the web dashboard reads it; the "
                         "GCS console and the vision worker both use the nadir "
                         "frame. Use on the real aircraft")
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
    passthrough = bool(args.mjpeg_passthrough) and args.backend == "v4l2"
    if passthrough and args.swap_rb:
        raise SystemExit("[grabber] --mjpeg-passthrough writes the camera's own "
                         "JPEG; there are no pixels to --swap-rb. Pick one.")
    if passthrough:
        nadir = MjpegPassthroughBackend(args.nadir_device, args.width,
                                        args.height, fps=args.fps)
        got = nadir.verify_resolution()
        if got is None:
            nadir.close()
            raise SystemExit("[grabber] MJPEG passthrough produced no decodable "
                             "frame — rerun with CAM_PASSTHROUGH=0")
        if got != (args.width, args.height):
            nadir.close()
            raise SystemExit(
                f"[grabber] camera delivered {got[0]}x{got[1]}, not "
                f"{args.width}x{args.height}. Passthrough cannot resize, and "
                "the projection derives fx AND the principal point from the "
                "configured size — flying this would scale and shift every "
                "pixel->lat/lon silently. Fix the config/camera, or use "
                "CAM_PASSTHROUGH=0 to fall back to the resizing path.")
        print(f"[grabber] passthrough resolution verified {got[0]}x{got[1]}")
    else:
        nadir = _make_backend(args.backend, args.nadir_device, args.width,
                              args.height, fps=args.fps, fourcc=args.fourcc)
    if args.backend == "v4l2":
        _force_short_exposure(args.nadir_device, args.exposure_100us, args.gain)
    mirror = not args.no_mirror
    print(f"[grabber] nadir {args.backend}"
          + (" MJPEG-passthrough" if passthrough else "")
          + f" dev={args.nadir_device} -> {args.nadir_out}"
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
            if passthrough:
                payload = nadir.grab_bytes()
                if payload is not None:
                    _write_bytes(payload, args.nadir_out)
                    if mirror:
                        _write_bytes(payload, args.frame_out)
                    n_ok += 1
                else:
                    n_fail += 1
            else:
                frame = nadir.grab()
                if frame is not None:
                    frame = _emit(frame)
                    _write_frame(frame, args.nadir_out, args.jpeg_quality)
                    if mirror:
                        _write_frame(frame, args.frame_out, args.jpeg_quality)
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
