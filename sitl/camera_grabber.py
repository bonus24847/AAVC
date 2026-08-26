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
# UVC menu value for "Aperture Priority" — the camera picks the exposure time.
# Measured on the OV9281 outdoors 2026-08-24: it selects 1 ms, and a moving
# marker decodes 65 % of frames at the competition's own pixel size.
_AUTO_EXPOSURE_MODE = 3
# The module's own factory gain, restored alongside auto so a bench sweep that
# left gain at 4 or 128 cannot follow the aircraft into a flight.
_DEFAULT_GAIN = 64

# ── highlight-priority auto exposure (2026-08-26) ──────────────────────────
# The camera's own AE meters the WHOLE frame: sunlit grass lands at ~130 and a
# white pad — 4-5x the albedo — clips at 255; at ~28 px the marker's black
# modules bleed into 1-3 px lines and nothing decodes (2788 flight frames,
# 0 decodes, pad in view: nadir_000203 white 255 / "black" 170-190). ArUco
# needs the WHITE to be unclipped and the black to be black; it does not care
# how dark the grass is. So: meter the brightest 0.2 % of the frame (the pad,
# the field lines) and hold that in [AE_HI_LO, AE_HI_HI]; when nothing white
# is in view, cap the ground mean so the NEXT pad cannot clip on entry.
AE_HI_LO, AE_HI_HI, AE_HI_TARGET = 190.0, 225.0, 208.0
AE_MEAN_MAX = 55.0          # grass mean cap: 55 x 4.5 (pad/grass albedo) ≈ 250
AE_MEAN_TARGET = 49.5       # where a decrease aims and below which an increase starts
AE_PCTL = 99.8              # a 0.5 m pad at 8 m is 0.35 % of the frame; 1 m at
AE_STRIDE = 2               # 12 m is 0.5 % — the top 0.2 % is INSIDE the pad
AE_PERIOD_S = 0.5
AE_STEP_UP, AE_STEP_DOWN = 1.6, 0.5    # per-step ratio bounds
AE_DAMP = 0.7                          # log-domain damping against overshoot
AE_CLIP = 250.0                        # ≥ this the true highlight is unknown
AE_DEFAULT_INIT_100US = 10             # 1 ms — what the driver's AE picked outdoors
AE_DEFAULT_MAX_100US = 40              # 4 ms: 1.3 px of smear at 3 m/s from 8 m


def meter_gray(gray: np.ndarray) -> tuple[float, float]:
    """(mean, highlight) of a mono frame — highlight = the AE_PCTL percentile
    over a strided sample (a percentile needs no more pixels than that)."""
    s = gray[::AE_STRIDE, ::AE_STRIDE]
    return float(s.mean()), float(np.percentile(s, AE_PCTL))


def ae_step(exp: float, mean: float, hi: float, *,
            max_100us: float = AE_DEFAULT_MAX_100US, min_100us: float = 1.0) -> float:
    """Next exposure (100 µs units) from this frame's metering. Pure.

    Brightness is ~linear in exposure until it clips, so the correction is a
    RATIO (damped, bounded per step): one or two steps land in the band, and
    a clipped highlight — whose true value is unknown — takes a bigger step
    down. Decrease wins over increase; inside the band the value holds."""
    # Both directions aim at the SAME points (AE_HI_TARGET, AE_MEAN_TARGET)
    # and the hold band around them is wider than one 100 µs exposure step
    # (14 % at 0.7 ms) — otherwise the loop hunts between two targets forever
    # (CM4 bench 2026-08-26: exp 9 → 7.1 → 7.6 → 7.0, ten steps in 8 s).
    if hi > AE_HI_HI or mean > AE_MEAN_MAX:
        r = 1.0
        if hi > AE_HI_HI:
            r = min(r, AE_HI_TARGET / hi)
        if mean > AE_MEAN_MAX:
            r = min(r, AE_MEAN_TARGET / mean)
        if hi >= AE_CLIP:
            r = min(r, 0.6)
    elif hi < AE_HI_LO and mean < AE_MEAN_TARGET:
        r = min(AE_HI_TARGET / max(hi, 1.0), AE_MEAN_TARGET / max(mean, 1.0))
    else:
        return float(exp)
    r = max(AE_STEP_DOWN, min(AE_STEP_UP, r ** AE_DAMP))
    new = float(max(min_100us, min(max_100us, exp * r)))
    # The driver takes integers: a correction too small to change the applied
    # value must not creep into the float either, or a string of 1 % nudges
    # eventually flips the rounding and the loop hunts one unit up and down.
    return new if int(round(new)) != int(round(exp)) else float(exp)


class HighlightAE:
    """Runs ``ae_step`` on the live frame every AE_PERIOD_S and pushes the
    result to the V4L2 node. Fail-soft like _force_short_exposure: a missing
    v4l2-ctl leaves the camera wherever it is and says so."""

    def __init__(self, device: str, *, init_100us: int = AE_DEFAULT_INIT_100US,
                 max_100us: int = AE_DEFAULT_MAX_100US,
                 period_s: float = AE_PERIOD_S) -> None:
        self.device = device
        self.exp = float(init_100us)
        self.max_100us = float(max_100us)
        self.period_s = period_s
        self.mean = float("nan")
        self.hi = float("nan")
        self.steps = 0
        self.next_at = 0.0
        self._applied: int | None = None

    def apply(self) -> None:
        """Push the current exposure (manual mode) — used at open/reopen too."""
        n = int(round(self.exp))
        if n == self._applied:
            return
        self._applied = n
        _apply_exposure(self.device, n)

    def maybe_step(self, gray: "np.ndarray | None", now: float) -> bool:
        """Meter + step if the period has elapsed. Returns True when it ran."""
        if now < self.next_at or gray is None:
            return False
        self.next_at = now + self.period_s
        self.mean, self.hi = meter_gray(gray)
        new = ae_step(self.exp, self.mean, self.hi, max_100us=self.max_100us)
        stepped = int(round(new)) != int(round(self.exp))
        self.exp = new
        if stepped:
            self.steps += 1
            self.apply()
        return True

    def status(self) -> str:
        return (f"exp={self.exp:.1f}x100us mean={self.mean:.0f} hi={self.hi:.0f} "
                f"steps={self.steps}")


def _apply_exposure(device: str, n_100us: int) -> None:
    """Quiet manual-exposure write for the AE loop (no readback, prints only
    on failure — this can run twice a second)."""
    dev = f"/dev/video{device}" if str(device).isdigit() else str(device)
    cmd = ["v4l2-ctl", "-d", dev, "-c", "auto_exposure=1",
           "-c", f"exposure_time_absolute={int(n_100us)}"]
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=2.0)
    except (FileNotFoundError, subprocess.TimeoutExpired,
            subprocess.CalledProcessError) as e:
        print(f"[grabber] AE: v4l2-ctl failed on {dev} ({e}) — exposure unchanged",
              flush=True)


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
    """Set the V4L2 nadir camera's exposure — or hand it back to AUTO.

    ⚠ THE PREMISE THIS WAS WRITTEN ON WAS WRONG, measured 2026-08-24. It was
    added on 2026-08-21 to force ~2 ms because flight frames scored Laplacian
    41-76 against 680-780 static, and auto_exposure=3 was blamed for sitting at
    16.6 ms. Read back OUTDOORS on the aircraft, auto picks **1 ms** — SHORTER
    than the 2 ms that was forced on it. Auto was never the blur.

    Forcing 2 ms is what broke the decode. Bench A/B at 8-10 m, same scene,
    same marker (the KMITL pixel size, 29-32 px):

        auto (1 ms)        brightness 124   sharpness  929   DECODED
        forced 2 ms g64    brightness 190   sharpness  934   no decode
        forced 2 ms g16    brightness 114   sharpness  377   no decode

    Sharpness was never the problem — EXPOSURE was. At 2 ms with this gain the
    frame lands at ~190 mean and the marker's white quiet zone blows out into
    the background, so the black/white boundary ArUco needs is gone. On auto,
    a marker CARRIED AT RUNNING SPEED across 8-10 m decoded 145 of 222 frames
    (65 %) at brightness 112-125.

    So ``n_100us <= 0`` now RESTORES auto exposure rather than merely declining
    to touch it. "Leave it alone" was a trap: a previous run that forced manual
    left the camera stuck in manual with a stale exposure, and nothing said so.
    Restoring the gain default with it keeps a bench sweep from leaking into a
    flight.

    Subprocess over CAP_PROP deliberately: the UVC control names are explicit,
    they apply to the node even while cv2 holds it open, and the readback shows
    what the driver kept. FAIL-SOFT by design: a missing binary/control logs and
    continues — this must never join the nadir open's fail-hard class. ``device``
    may be a bare index ("0"); v4l2-ctl needs a path, so it is rewritten to
    /dev/video<N>. ``gain`` < 0 leaves the sensor gain alone EXCEPT in the
    restore-auto case, where the default is put back."""
    controls: list[str] = []
    if n_100us > 0:
        controls += ["auto_exposure=1", f"exposure_time_absolute={int(n_100us)}"]
    else:
        # Hand the camera back to auto, and undo any gain a bench sweep left.
        controls.append(f"auto_exposure={_AUTO_EXPOSURE_MODE}")
        if gain < 0:
            controls.append(f"gain={_DEFAULT_GAIN}")
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


def _should_reopen(*, now: float, last_ok: float, next_reopen_at: float,
                   after_s: float) -> bool:
    """Is it time to throw the camera handle away and open a fresh one?

    Three conditions, and the third is the one that matters under a camera that
    stays gone: the frame file has been quiet for longer than ``after_s``, the
    feature is enabled at all, and the backoff from the previous failed attempt
    has expired — otherwise a camera that is unplugged turns into a re-open
    storm at the loop rate.
    """
    return (after_s > 0.0
            and (now - last_ok) > after_s
            and now >= next_reopen_at)


def _open_nadir(args: argparse.Namespace, passthrough: bool,
                ae: "HighlightAE | None" = None):
    """Open the nadir camera and hand back a backend ready to grab.

    Raises ``RuntimeError`` for every way the camera can be unusable — will not
    open, will not decode, or delivers a size other than the configured one.
    That last case is a refusal on purpose: the projection derives fx AND the
    principal point from the configured size, so a silently different frame
    would scale and shift every pixel->lat/lon answer.

    Factored out of ``main`` 2026-08-23 so the run loop can call it AGAIN. A UVC
    camera that browns out re-enumerates under a new device node — measured on
    the CM4 the same day, ``usb 1-1.1: USB disconnect`` moving the OV9281 from
    video0 to video1 — and the already-open descriptor stays dead for good. The
    process kept running, the frame file simply stopped changing, and nothing
    said so for 14 minutes. Re-opening the ``by-id`` path picks up whatever node
    the camera came back on.
    """
    if passthrough:
        cam = MjpegPassthroughBackend(args.nadir_device, args.width,
                                      args.height, fps=args.fps)
        got = cam.verify_resolution()
        if got is None:
            cam.close()
            raise RuntimeError("MJPEG passthrough produced no decodable frame "
                               "— rerun with CAM_PASSTHROUGH=0")
        if got != (args.width, args.height):
            cam.close()
            raise RuntimeError(
                f"camera delivered {got[0]}x{got[1]}, not "
                f"{args.width}x{args.height}. Passthrough cannot resize, and "
                "the projection derives fx AND the principal point from the "
                "configured size — flying this would scale and shift every "
                "pixel->lat/lon silently. Fix the config/camera, or use "
                "CAM_PASSTHROUGH=0 to fall back to the resizing path.")
        print(f"[grabber] passthrough resolution verified {got[0]}x{got[1]}")
    else:
        cam = _make_backend(args.backend, args.nadir_device, args.width,
                            args.height, fps=args.fps, fourcc=args.fourcc)
    if args.backend == "v4l2":
        # A re-enumerated node comes back on the driver's defaults, so the
        # exposure has to be re-forced every time — not just at startup.
        if ae is not None and args.exposure_100us <= 0:
            # highlight AE owns the exposure: manual mode at its current value
            # (the readback in _force_short_exposure proves the node took it)
            _force_short_exposure(args.nadir_device, int(round(ae.exp)), args.gain)
            ae._applied = int(round(ae.exp))
        else:
            _force_short_exposure(args.nadir_device, args.exposure_100us, args.gain)
    return cam


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
    ap.add_argument("--ae-highlight", action="store_true",
                    help="v4l2 only: software highlight-priority auto exposure "
                         "— meter the brightest 0.2 %% of the frame (a white pad, "
                         "the field lines) and hold it under clipping, capping "
                         "the ground mean when nothing white is in view. The "
                         "driver's AE meters the grass and clips the pad "
                         "(2026-08-26: 2788 flight frames, 0 decodes). Ignored "
                         "when --exposure-100us > 0 (manual wins)")
    ap.add_argument("--ae-init-100us", type=int, default=AE_DEFAULT_INIT_100US,
                    help="starting exposure for --ae-highlight (100 µs units)")
    ap.add_argument("--ae-max-100us", type=int, default=AE_DEFAULT_MAX_100US,
                    help="exposure ceiling for --ae-highlight — motion smear: "
                         "40 = 4 ms = 1.3 px at 3 m/s from 8 m")
    ap.add_argument("--reopen-after-s", type=float, default=3.0,
                    help="reopen the camera when this many seconds pass with no "
                         "frame written (a browned-out UVC camera re-enumerates "
                         "under a new device node and the open descriptor stays "
                         "dead). 0 disables")
    args = ap.parse_args()

    # NADIR is the sole control-authority camera — fail hard if it won't open.
    passthrough = bool(args.mjpeg_passthrough) and args.backend == "v4l2"
    if passthrough and args.swap_rb:
        raise SystemExit("[grabber] --mjpeg-passthrough writes the camera's own "
                         "JPEG; there are no pixels to --swap-rb. Pick one.")
    ae: HighlightAE | None = None
    if args.ae_highlight and args.backend == "v4l2" and args.exposure_100us <= 0:
        ae = HighlightAE(args.nadir_device, init_100us=args.ae_init_100us,
                         max_100us=args.ae_max_100us)
    try:
        nadir = _open_nadir(args, passthrough, ae)
    except RuntimeError as exc:
        raise SystemExit(f"[grabber] {exc}") from exc
    mirror = not args.no_mirror
    print(f"[grabber] nadir {args.backend}"
          + (" MJPEG-passthrough" if passthrough else "")
          + (f" highlight-AE(init {args.ae_init_100us}, max {args.ae_max_100us})"
             if ae is not None else "")
          + f" dev={args.nadir_device} -> {args.nadir_out}"
          + (f" (+ {args.frame_out})" if mirror else " (mirror OFF)")
          + f" every {args.interval_s:.2f}s")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))

    def _emit(frame: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if args.swap_rb else frame

    n_ok = n_fail = n_reopen = 0
    last_log = 0.0
    # Frame-liveness watchdog. Keyed on the last frame actually WRITTEN, not on
    # a failure count: the symptom is the frame file going quiet, and it must be
    # caught whether the backend says so (grab returns None) or just stops
    # producing. A read() that blocks forever is NOT covered — that needs a
    # second thread, and it is not the failure this aircraft has had.
    last_ok = time.monotonic()
    next_reopen_at = 0.0
    reopen_backoff_s = 1.0
    try:
        while not stop["flag"]:
            t0 = time.monotonic()
            gray: np.ndarray | None = None
            if passthrough:
                payload = nadir.grab_bytes()
                wrote = payload is not None
                if payload is not None:
                    _write_bytes(payload, args.nadir_out)
                    if mirror:
                        _write_bytes(payload, args.frame_out)
                    if ae is not None and time.monotonic() >= ae.next_at:
                        # one decode per AE period (~15 ms on the CM4), not per frame
                        gray = cv2.imdecode(np.frombuffer(payload, np.uint8),
                                            cv2.IMREAD_GRAYSCALE)
            else:
                frame = nadir.grab()
                wrote = frame is not None
                if frame is not None:
                    frame = _emit(frame)
                    _write_frame(frame, args.nadir_out, args.jpeg_quality)
                    if mirror:
                        _write_frame(frame, args.frame_out, args.jpeg_quality)
                    if ae is not None:
                        gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                if frame.ndim == 3 else frame)
            if ae is not None:
                ae.maybe_step(gray, time.monotonic())
            n_ok, n_fail = (n_ok + 1, n_fail) if wrote else (n_ok, n_fail + 1)
            now = time.monotonic()
            if wrote:
                last_ok = now
                reopen_backoff_s = 1.0
                next_reopen_at = 0.0
            elif _should_reopen(now=now, last_ok=last_ok,
                                next_reopen_at=next_reopen_at,
                                after_s=args.reopen_after_s):
                print(f"[grabber] no frame for {now - last_ok:.1f}s — reopening "
                      f"{args.nadir_device}", flush=True)
                # Release FIRST: a camera that came back on a new node can be
                # blocked by the stale handle, and on failure the released
                # backend keeps returning None, which is the honest state — a
                # frame file that stops ageing is what cm4/status_beacon.py
                # turns into `AAVC cam=DEAD <n>s stale` over the radio. Exiting
                # instead would remove the only process still trying, and
                # nothing restarts it in flight.
                try:
                    nadir.close()
                except Exception as exc:                     # noqa: BLE001
                    print(f"[grabber] close before reopen failed: {exc}",
                          flush=True)
                try:
                    nadir = _open_nadir(args, passthrough, ae)
                except Exception as exc:                     # noqa: BLE001
                    # Re-read the clock: opening is not instant (verify_
                    # resolution grabs a frame, and _force_short_exposure shells
                    # out to v4l2-ctl with a 5 s timeout twice). Reusing the
                    # pre-attempt `now` would date both the backoff and the
                    # quiet-since mark, and a SLOW open would then look like a
                    # long silence and re-open again immediately — the storm
                    # the backoff exists to prevent.
                    after = time.monotonic()
                    next_reopen_at = after + reopen_backoff_s
                    reopen_backoff_s = min(reopen_backoff_s * 2.0, 15.0)
                    print(f"[grabber] reopen failed ({exc}) — retrying in "
                          f"{next_reopen_at - after:.0f}s", flush=True)
                else:
                    n_reopen += 1
                    last_ok = time.monotonic()
                    reopen_backoff_s = 1.0
                    next_reopen_at = 0.0
                    print(f"[grabber] reopened (#{n_reopen})", flush=True)
            if now - last_log > 2.0:
                print(f"[grabber] nadir ok={n_ok} fail={n_fail} "
                      f"reopen={n_reopen}"
                      + (f" AE {ae.status()}" if ae is not None else ""),
                      flush=True)      # the launchers do not run python -u
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
