"""Camera calibration-from-config (vision.projection.configure_cameras) + the
real-camera grabber's frame-write contract (sitl/camera_grabber.py)."""
from __future__ import annotations

import importlib.util
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
    placeholder) — and no second camera block (the oblique cue was retired)."""
    import yaml

    cfg_path = Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml"
    cams = yaml.safe_load(cfg_path.read_text())["cameras"]
    assert set(cams) == {"nadir"}
    assert cams["nadir"] == {
        "fov_deg": 74.2, "width_px": 1280, "height_px": 720, "depression_deg": 90.0,
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


def test_grabber_write_png_atomic_bgr_0600(tmp_path: Path) -> None:
    g = _load_grabber()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :, 2] = 255           # pure red in BGR (channel 2 = red)
    out = tmp_path / "aavc_nadir.png"

    g._write_png(img, out)

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


def test_force_short_exposure_zero_is_a_noop(monkeypatch) -> None:
    """--exposure-100us 0 with gain -1 (the default, and the SITL/laptop
    path) = leave the driver's auto exposure alone — no subprocess at all."""
    g = _load_grabber()

    def _boom(cmd, **kw):
        raise AssertionError("subprocess must not run for the no-op defaults")

    monkeypatch.setattr(g.subprocess, "run", _boom)
    g._force_short_exposure("/dev/video0", 0, -1)
