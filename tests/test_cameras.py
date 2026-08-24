"""Camera calibration-from-config (vision.projection.configure_cameras) + the
real-camera grabber's frame-write contract (sitl/camera_grabber.py)."""
from __future__ import annotations

import importlib.util
import inspect
import math
from pathlib import Path

import cv2
import numpy as np

from vision import projection as P

# ── 1b: camera calibration as config ─────────────────────────────────────────

def test_cam_overrides_deg_to_rad() -> None:
    o = P._cam_overrides(
        {"fov_deg": 90.0, "depression_deg": 45.0, "width_px": 1280, "height_px": 720})
    assert math.isclose(o["fov_rad"], math.radians(90.0))
    assert math.isclose(o["depression_rad"], math.radians(45.0))
    assert o["width_px"] == 1280 and o["height_px"] == 720


def test_cam_overrides_radians_take_precedence() -> None:
    o = P._cam_overrides({"fov_rad": 1.5, "fov_deg": 90.0, "depression_rad": 1.2})
    assert o["fov_rad"] == 1.5 and o["depression_rad"] == 1.2


def test_cam_overrides_empty() -> None:
    assert P._cam_overrides({}) == {}


def test_configure_cameras_applies_in_place() -> None:
    """configure_cameras mutates the shared NADIR singleton; save + restore so
    other tests still see the SITL defaults."""
    saved = (P.NADIR.fov_rad, P.NADIR.depression_rad, P.NADIR.width_px, P.NADIR.height_px)
    try:
        P.configure_cameras(
            nadir={"fov_deg": 80.0, "width_px": 800, "height_px": 600,
                   "depression_deg": 88.0})
        assert math.isclose(P.NADIR.fov_rad, math.radians(80.0))
        assert P.NADIR.width_px == 800 and P.NADIR.height_px == 600
        # fx_px recomputes from the updated fov/width
        assert math.isclose(P.NADIR.fx_px, 400.0 / math.tan(math.radians(80.0) / 2.0))
        # depression is calibratable too (G6 gimbal residual-pitch trim)
        assert math.isclose(P.NADIR.depression_rad, math.radians(88.0))
    finally:
        fov, depr, w, h = saved
        object.__setattr__(P.NADIR, "fov_rad", fov)
        object.__setattr__(P.NADIR, "depression_rad", depr)
        object.__setattr__(P.NADIR, "width_px", w)
        object.__setattr__(P.NADIR, "height_px", h)


def test_configure_cameras_none_is_noop() -> None:
    before = (P.NADIR.fov_rad, P.NADIR.depression_rad)
    P.configure_cameras()
    assert (P.NADIR.fov_rad, P.NADIR.depression_rad) == before


# ── 1a: grabber frame-write contract ─────────────────────────────────────────

def _load_grabber():
    path = Path(__file__).resolve().parents[1] / "sitl" / "camera_grabber.py"
    spec = importlib.util.spec_from_file_location("camera_grabber", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_config_nadir_matches_ov9281_profile() -> None:
    """Parity-lock the shipped camera profile (the Meige OV9281 UVC module):
    1280×720, nadir 90°, fov 74.2° — MEASURED on the real lens 2026-08-17
    (50 mm marker at 0.495 m → 85.5 px → fx 847; replaces the 99.7 unmeasured
    placeholder) — and no second camera block (the oblique cue was retired).

    ``mount_yaw_deg`` joined the block 2026-08-22 locked at 0.0 — an ASSUMPTION
    nobody had measured — with a note that the day someone read it off the
    aircraft this test should fail and be updated with the reading. That day was
    2026-08-23: it measured **180**, the camera is bolted upside down, and the
    assumption had been wrong by ~175 deg in every direction. The evidence (four
    placements round the airframe) lives in
    ``tests/test_measure_mount_yaw.py::_BENCH_2026_08_23``. It moves the
    pixel->lat/lon bearing and the heading the sweep holds; SITL cannot check
    it, because the gz camera shares the same assumption.
    """
    import yaml

    cfg_path = Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml"
    cams = yaml.safe_load(cfg_path.read_text())["cameras"]
    assert set(cams) == {"nadir"}
    assert cams["nadir"] == {
        "fov_deg": 74.2, "width_px": 1280, "height_px": 720,
        "depression_deg": 90.0, "mount_yaw_deg": 180.0,
    }


def test_grabber_gray_frame_written_as_bgr(tmp_path: Path) -> None:
    """A mono (2-D) frame from the OV9281 must be normalised to 3-channel BGR
    before hitting the PNG contract (cv2.imread of the result is (h,w,3))."""
    g = _load_grabber()
    gray = np.full((720, 1280), 200, dtype=np.uint8)
    out = g._to_bgr(gray)
    assert out.shape == (720, 1280, 3)
    assert (out[..., 0] == out[..., 1]).all() and (out[..., 1] == out[..., 2]).all()
    # A 3-channel frame passes through untouched.
    bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    assert g._to_bgr(bgr) is bgr


def test_grabber_write_frame_atomic_bgr_0600(tmp_path: Path) -> None:
    g = _load_grabber()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :, 2] = 255           # pure red in BGR (channel 2 = red)
    out = tmp_path / "aavc_nadir.png"

    g._write_frame(img, out)

    assert out.exists()
    assert not (tmp_path / "aavc_nadir.tmp.png").exists()   # atomic: no temp left behind
    assert (out.stat().st_mode & 0o777) == 0o600            # owner-only perms
    back = cv2.imread(str(out))
    assert back is not None and back.shape == (480, 640, 3)
    # red must survive as red (BGR), not swapped to blue — the detector keys on it
    assert int(back[0, 0, 2]) == 255 and int(back[0, 0, 0]) == 0


# ── 1c: forced short exposure — the G7 2026-08-21 in-flight-blur lever ───────
# Flight frames scored Laplacian 41-76 vs 680-780 static because the OV9281
# sat in auto_exposure=3 (Aperture Priority) at 16.6 ms. The flag must
# translate to exactly the UVC controls measured on the real module, and it
# must be fail-soft: exposure never joins the nadir open's fail-hard class.


def test_force_short_exposure_builds_the_v4l2_ctl_command(monkeypatch) -> None:
    """auto_exposure=1 (Manual Mode — activates exposure_time_absolute, unit
    100 µs) on the given device path; a bare index is rewritten to
    /dev/video<N> (v4l2-ctl cannot take "0"); gain is untouched unless asked."""
    g = _load_grabber()
    calls: list[list[str]] = []

    class _Done:
        stdout = "auto_exposure: 1 exposure_time_absolute: 20"
        stderr = ""

    def _fake_run(cmd, **kw):
        calls.append(list(cmd))
        return _Done()

    monkeypatch.setattr(g.subprocess, "run", _fake_run)
    g._force_short_exposure("/dev/v4l/by-id/usb-cam-video-index0", 20, -1)
    set_cmd = calls[0]
    assert set_cmd[:3] == ["v4l2-ctl", "-d", "/dev/v4l/by-id/usb-cam-video-index0"]
    assert "auto_exposure=1" in set_cmd
    assert "exposure_time_absolute=20" in set_cmd
    assert not any(c.startswith("gain=") for c in set_cmd)

    calls.clear()
    g._force_short_exposure("0", 30, 96)
    assert calls[0][2] == "/dev/video0"
    assert "gain=96" in calls[0]


def test_force_short_exposure_is_fail_soft(monkeypatch) -> None:
    """A laptop with no v4l2-ctl (FileNotFoundError), a hung tool (timeout),
    or an unknown control (non-zero exit) must LOG and return — never raise
    into the grabber's startup."""
    g = _load_grabber()

    def _missing(cmd, **kw):
        raise FileNotFoundError("v4l2-ctl")

    monkeypatch.setattr(g.subprocess, "run", _missing)
    g._force_short_exposure("/dev/video0", 20, -1)          # must not raise

    def _denied(cmd, **kw):
        raise g.subprocess.CalledProcessError(1, cmd, stderr=b"unknown control")

    monkeypatch.setattr(g.subprocess, "run", _denied)
    g._force_short_exposure("/dev/video0", 20, -1)          # must not raise

    def _hangs(cmd, **kw):
        raise g.subprocess.TimeoutExpired(cmd, 5.0)

    monkeypatch.setattr(g.subprocess, "run", _hangs)
    g._force_short_exposure("/dev/video0", 20, -1)          # must not raise


def test_force_short_exposure_zero_RESTORES_auto(monkeypatch) -> None:
    """--exposure-100us 0 must hand the camera back to AUTO, not merely decline
    to touch it.

    It used to be a no-op, and that was a trap with real cost. A run that forced
    manual exposure left the camera in manual, and the next run — asking for
    "auto" by passing 0 — silently inherited the stale manual value. That is how
    the decode stayed broken from 2026-08-21 to 2026-08-24: the forced 2 ms sat
    on the camera across restarts while every launcher believed it was on auto.
    The gain default goes back with it so a bench sweep that left gain at 4 or
    128 cannot follow the aircraft into a flight.
    """
    g = _load_grabber()
    calls: list[list[str]] = []

    class _Done:
        stdout = ""
        stderr = ""

    monkeypatch.setattr(g.subprocess, "run",
                        lambda cmd, **kw: (calls.append(list(cmd)), _Done())[1])
    g._force_short_exposure("/dev/video0", 0, -1)

    assert calls, "0 must still talk to the camera — that is the whole fix"
    set_cmd = calls[0]
    assert f"auto_exposure={g._AUTO_EXPOSURE_MODE}" in set_cmd, set_cmd
    assert f"gain={g._DEFAULT_GAIN}" in set_cmd, set_cmd
    # and it must NOT pin an exposure time while asking for auto
    assert not any(c.startswith("exposure_time_absolute=") for c in set_cmd), set_cmd


def test_an_explicit_gain_still_wins_when_restoring_auto(monkeypatch) -> None:
    """Restoring auto puts the DEFAULT gain back, but an operator who passed
    --gain explicitly asked for that number and must still get it."""
    g = _load_grabber()
    calls: list[list[str]] = []

    class _Done:
        stdout = ""
        stderr = ""

    monkeypatch.setattr(g.subprocess, "run",
                        lambda cmd, **kw: (calls.append(list(cmd)), _Done())[1])
    g._force_short_exposure("/dev/video0", 0, 96)
    set_cmd = calls[0]
    assert f"auto_exposure={g._AUTO_EXPOSURE_MODE}" in set_cmd
    assert "gain=96" in set_cmd, set_cmd


def test_the_launchers_default_the_real_camera_to_auto() -> None:
    """The premise the 2 ms default was built on was measured FALSE on
    2026-08-24 (auto picks 1 ms outdoors — shorter than what was forced), and
    the forcing is what broke the decode. Pin the default so it cannot drift
    back without someone reading why."""
    root = Path(__file__).resolve().parents[1]
    for rel in ("sitl/run_mission.sh", "cm4/launch_flight.sh"):
        txt = (root / rel).read_text()
        assert 'CAM_EXPOSURE="${CAM_EXPOSURE:-0}"' in txt, rel
        assert 'CAM_EXPOSURE="${CAM_EXPOSURE:-20}"' not in txt, rel


# ── frame transport: the SUFFIX picks the codec (JPEG since 2026-08-21) ──────
# Measured on the CM4 at 1280x720: PNG encodes in 48 ms and decodes in 33,
# JPEG q95 in 12 and 15, for 280 KB -> 62 KB on disk (which also shrinks the
# WiFi frame sync). That 54 ms per analysed frame was the pipeline's single
# largest cost — larger than the ArUco detection it exists to feed.


def test_write_frame_codec_follows_the_suffix(tmp_path: Path) -> None:
    g = _load_grabber()
    img = np.zeros((64, 96, 3), dtype=np.uint8)
    img[:, :, 2] = 255                        # red in BGR

    jpg = tmp_path / "f.jpg"
    g._write_frame(img, jpg)
    assert jpg.read_bytes()[:2] == b"\xff\xd8"        # JPEG SOI
    assert (jpg.stat().st_mode & 0o777) == 0o600
    assert not (tmp_path / "f.tmp.jpg").exists()      # atomic: temp cleaned up

    png = tmp_path / "f.png"
    g._write_frame(img, png)
    assert png.read_bytes()[:4] == b"\x89PNG"         # PNG magic — still works
    back = cv2.imread(str(png))
    assert back is not None and back.shape == (64, 96, 3)


def test_jpeg_quality_is_high_enough_for_marker_edges(tmp_path: Path) -> None:
    """The decode reads black/white cell edges, so the quality has to protect
    THEM, not the average pixel. q95 costs 0.5 ms over q85 on the CM4 — cheap
    insurance. Pinned as a floor so nobody trades it away for a millisecond."""
    g = _load_grabber()
    sig = inspect.signature(g._write_frame)
    assert sig.parameters["jpeg_quality"].default >= 92

    # a hard-edged checkerboard survives the round trip essentially intact
    board = np.zeros((64, 64, 3), dtype=np.uint8)
    board[::2, ::2] = 255
    board[1::2, 1::2] = 255
    out = tmp_path / "edges.jpg"
    g._write_frame(board, out)
    back = cv2.imread(str(out))
    assert back is not None
    assert float(np.abs(back.astype(int) - board.astype(int)).mean()) < 12.0


def test_the_frame_contract_paths_are_jpeg() -> None:
    """Writer and reader must agree on the same file, or the worker silently
    decodes a frame nobody is writing any more."""
    from orchestrator.preflight import NADIR_FRAME
    from orchestrator.vision_worker import DEFAULT_NADIR_FRAME

    g = _load_grabber()
    assert str(DEFAULT_NADIR_FRAME) == "/tmp/aavc_nadir.jpg"
    assert str(NADIR_FRAME) == str(DEFAULT_NADIR_FRAME)
    ap_defaults = {a.dest: a.default for a in g._build_parser()._actions} \
        if hasattr(g, "_build_parser") else {}
    if ap_defaults:
        assert str(ap_defaults["nadir_out"]) == str(DEFAULT_NADIR_FRAME)


# ── 1d: the camera that goes away and comes back (2026-08-23) ────────────────
# A UVC brown-out re-enumerates the OV9281 under a NEW device node (`usb 1-1.1:
# USB disconnect` → video0 becomes video1, measured on the CM4). The open
# descriptor stays dead for good, the grabber process keeps running, and the
# frame file simply stops changing — for 14 minutes, in the incident. The
# beacon says `cam=DEAD` over the radio; nothing used to fix it.


def test_should_reopen_needs_quiet_enabled_and_past_the_backoff() -> None:
    g = _load_grabber()
    k = dict(now=100.0, last_ok=90.0, next_reopen_at=0.0, after_s=3.0)
    assert g._should_reopen(**k)                       # 10 s of quiet
    assert not g._should_reopen(**{**k, "after_s": 0.0})       # disabled
    assert not g._should_reopen(**{**k, "last_ok": 99.0})      # only 1 s quiet
    assert not g._should_reopen(**{**k, "next_reopen_at": 105.0})  # backoff
    # exactly at the threshold is not yet stale — strictly greater
    assert not g._should_reopen(**{**k, "last_ok": 97.0})


def test_open_nadir_refuses_a_camera_that_changed_resolution(monkeypatch) -> None:
    """The projection derives fx AND the principal point from the configured
    size, so a camera that comes back at a different resolution must NOT be
    flown — silently scaling and shifting every pixel->lat/lon."""
    import argparse

    g = _load_grabber()
    closed: list[bool] = []

    class _Cam:
        def __init__(self, *a, **k) -> None:
            pass

        def verify_resolution(self):
            return (640, 480)

        def close(self) -> None:
            closed.append(True)

    monkeypatch.setattr(g, "MjpegPassthroughBackend", _Cam)
    args = argparse.Namespace(nadir_device="/dev/video0", width=1280,
                              height=720, fps=30, fourcc=None, backend="v4l2",
                              exposure_100us=20, gain=-1)
    try:
        g._open_nadir(args, True)
    except RuntimeError as exc:
        assert "640x480" in str(exc)
    else:                                              # pragma: no cover
        raise AssertionError("a resolution change must not be accepted")
    assert closed, "the rejected handle must be released"


def test_the_grabber_reopens_a_camera_that_stopped_delivering(
        monkeypatch, tmp_path) -> None:
    """End to end through main(): the camera delivers, then goes quiet, and the
    loop must build a SECOND backend rather than sitting on the dead one. The
    fake stops the run once that has happened."""
    import sys

    g = _load_grabber()
    opened: list[object] = []
    jpeg = cv2.imencode(".jpg", np.zeros((720, 1280, 3), np.uint8))[1].tobytes()

    class _Cam:
        def __init__(self, *a, **k) -> None:
            self.n = 0
            self.first = not opened
            opened.append(self)

        def verify_resolution(self):
            return (1280, 720)

        def grab_bytes(self):
            self.n += 1
            if not self.first:
                # The re-opened camera works again — and having proved that,
                # end the run so the test does not spin.
                raise SystemExit(0)
            return jpeg if self.n <= 2 else None       # then the camera goes

        def close(self) -> None:
            pass

    monkeypatch.setattr(g, "MjpegPassthroughBackend", _Cam)
    monkeypatch.setattr(g, "_force_short_exposure", lambda *a, **k: None)
    out = tmp_path / "nadir.jpg"
    monkeypatch.setattr(sys, "argv", [
        "camera_grabber", "--backend", "v4l2", "--mjpeg-passthrough",
        "--nadir-device", "/dev/v4l/by-id/cam-video-index0",
        "--nadir-out", str(out), "--no-mirror",
        "--interval-s", "0", "--reopen-after-s", "0.01"])

    try:
        g.main()
    except SystemExit as exc:
        assert exc.code == 0
    assert len(opened) == 2, (
        f"the dead camera should have been reopened once, opened={len(opened)}")
    assert out.exists(), "the frames written before the failure must survive"


def test_reopen_is_off_when_asked(monkeypatch) -> None:
    """`--reopen-after-s 0` must leave the old behaviour exactly as it was."""
    g = _load_grabber()
    assert not g._should_reopen(now=1e6, last_ok=0.0, next_reopen_at=0.0,
                                after_s=0.0)


def test_a_slow_reopen_does_not_trigger_another_one_immediately(
        monkeypatch, tmp_path) -> None:
    """Opening is not instant — verify_resolution grabs a frame and
    _force_short_exposure shells out to v4l2-ctl with a 5 s timeout, twice. If
    the loop marks "last frame seen" with the clock reading from BEFORE the
    attempt, a slow open reads back as a long silence and re-opens again on the
    very next iteration: a tight loop of opens on a camera that is merely slow.

    Driven on a fake clock so it is a property, not a race.
    """
    import sys

    g = _load_grabber()
    clock = {"t": 0.0}
    opened: list[object] = []
    jpeg = cv2.imencode(".jpg", np.zeros((720, 1280, 3), np.uint8))[1].tobytes()

    class _Clock:
        @staticmethod
        def monotonic() -> float:
            return clock["t"]

        @staticmethod
        def sleep(_s: float) -> None:
            pass

    class _Cam:
        def __init__(self, *a, **k) -> None:
            self.grabs = 0
            self.first = not opened
            if not self.first:
                clock["t"] += 10.0        # a SLOW open
            opened.append(self)
            if len(opened) == 3:
                raise SystemExit(0)       # third open = the storm; stop here

        def verify_resolution(self):
            return (1280, 720)

        def grab_bytes(self):
            self.grabs += 1
            clock["t"] += 0.1
            return jpeg if (self.first and self.grabs == 1) else None

        def close(self) -> None:
            pass

    monkeypatch.setattr(g, "time", _Clock)
    monkeypatch.setattr(g, "MjpegPassthroughBackend", _Cam)
    monkeypatch.setattr(g, "_force_short_exposure", lambda *a, **k: None)
    monkeypatch.setattr(sys, "argv", [
        "camera_grabber", "--backend", "v4l2", "--mjpeg-passthrough",
        "--nadir-device", "/dev/v4l/by-id/cam-video-index0",
        "--nadir-out", str(tmp_path / "n.jpg"), "--no-mirror",
        "--interval-s", "0", "--reopen-after-s", "1.0"])

    try:
        g.main()
    except SystemExit:
        pass
    assert len(opened) == 3
    served = opened[1].grabs
    assert served >= 5, (
        "the second camera was replaced after only "
        f"{served} grab(s) — the 10 s spent OPENING it was counted as silence")
