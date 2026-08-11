"""Landing-pad detector (vision.detectors.aruco) — AAVC 2026 V1.3 target.

Synthetic scenes are drawn with render_pad_bgr — the SAME renderer that bakes
the SITL textures and the HITL synthetic camera frames — scaled to altitude-
realistic pixel sizes for the 1280x720 / 99.7 deg nadir camera (OV9281 profile;
the decode math is width-only): GSD = alt * 2.372 / 1280 m/px  ->  the 1 m pad
is ~45 px at the 12 m sweep altitude (marker ~18 px, the decoder floor) and
~108 px at the 5 m rung.
"""

from __future__ import annotations

import cv2
import numpy as np

from vision.detectors.aruco import (
    MARKER_TO_PAD,
    VALID_MARKER_IDS,
    PadDetector,
    find_landing_pads,
    render_pad_bgr,
)

GRASS = (60, 120, 70)   # BGR, matches the SITL grass albedo tone


def _scene(pad_px: int, marker_id: int = 3, *, angle_deg: float = 0.0,
           blur: int = 0, at: tuple[int, int] | None = None,
           w: int = 1280, h: int = 720) -> np.ndarray:
    """A grass frame with one pad of ``pad_px`` square pixels pasted in."""
    img = np.full((h, w, 3), GRASS, np.uint8)
    pad = cv2.resize(render_pad_bgr(marker_id, 512), (pad_px, pad_px),
                     interpolation=cv2.INTER_AREA)
    if angle_deg:
        m = cv2.getRotationMatrix2D((pad_px / 2, pad_px / 2), angle_deg, 1.0)
        pad = cv2.warpAffine(pad, m, (pad_px, pad_px), borderValue=GRASS)
    cx, cy = at if at else (w // 2, h // 2)
    x0, y0 = cx - pad_px // 2, cy - pad_px // 2
    img[y0:y0 + pad_px, x0:x0 + pad_px] = pad
    if blur:
        img = cv2.GaussianBlur(img, (blur, blur), 0)
    return img


def _pad_px(alt_m: float, width_px: int = 1280) -> int:
    """Pad size in px at a given altitude (1280 px, 99.7 deg HFOV nadir)."""
    return int(round(1.0 / (alt_m * 2.372 / width_px)))


# ── decode: id + centroid at the mission's working altitudes ──

def test_decodes_id_and_centroid_across_altitudes() -> None:
    for alt in (12.0, 10.0, 8.0, 5.0):
        px = _pad_px(alt)
        hits = find_landing_pads(_scene(px, marker_id=4, angle_deg=25.0))
        assert hits, f"no hit at {alt} m ({px} px pad)"
        top = hits[0]
        assert top.marker_id == 4, f"wrong id at {alt} m"
        assert abs(top.cx - 640) <= 4 and abs(top.cy - 360) <= 4
        # radius_px = marker half-side = pad_px * 0.4 / 2 within ~15 %
        expected = px * MARKER_TO_PAD / 2.0
        assert abs(top.radius_px - expected) <= 0.15 * expected


def test_rejects_wrong_dictionary_id() -> None:
    # id 7 is a valid DICT_4X4_50 marker but NOT a competition id (1..6).
    img = np.full((960, 1280, 3), GRASS, np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 80)
    img[440:520, 600:680] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    assert all(h.marker_id is None for h in find_landing_pads(img))


# ── cue: blob-only fallback when the marker cannot decode ──

def test_blurred_pad_falls_back_to_cue_with_marker_equivalent_radius() -> None:
    px = _pad_px(12.0)
    hits = find_landing_pads(_scene(px, marker_id=2, blur=7))
    assert hits
    top = hits[0]
    assert top.marker_id is None            # too blurred to decode
    assert top.confidence >= 0.5
    assert abs(top.cx - 640) <= 5 and abs(top.cy - 360) <= 5
    # blob radius_px is marker-EQUIVALENT: 0.4 x half the observed pad side
    assert abs(top.radius_px - MARKER_TO_PAD * top.pad_side_px / 2.0) < 0.5
    assert abs(top.pad_side_px - px) <= 0.2 * px


def test_mild_blur_still_decodes_via_booster() -> None:
    hits = find_landing_pads(_scene(_pad_px(12.0), marker_id=2, blur=3))
    assert hits and hits[0].marker_id == 2


# ── multi-pad frames: the registry wants every pad ──

def test_reports_every_pad_in_frame() -> None:
    img = _scene(50, marker_id=1)
    pad = cv2.resize(render_pad_bgr(6, 512), (50, 50),
                     interpolation=cv2.INTER_AREA)
    img[200:250, 200:250] = pad
    ids = {h.marker_id for h in find_landing_pads(img)}
    assert {1, 6} <= ids


# ── distractor rejection ──

def test_ignores_grass_grey_box_and_plain_white_square() -> None:
    grass = np.full((960, 1280, 3), GRASS, np.uint8)
    assert find_landing_pads(grass) == []

    grey = grass.copy()   # the 10 m launch pad — mid-grey, no marker
    cv2.rectangle(grey, (600, 400), (680, 480), (128, 128, 128), -1)
    assert find_landing_pads(grey) == []

    white = grass.copy()  # white car roof — bright square, NO dark centre
    cv2.rectangle(white, (600, 400), (660, 460), (255, 255, 255), -1)
    assert find_landing_pads(white) == []


def test_handles_empty_and_tiny_frames() -> None:
    assert find_landing_pads(np.zeros((0, 0, 3), np.uint8)) == []
    assert find_landing_pads(np.zeros((8, 8, 3), np.uint8)) == []


# ── renderer contract ──

def test_renderer_rejects_invalid_ids_and_undecodable_geometry() -> None:
    import pytest
    with pytest.raises(ValueError):
        render_pad_bgr(0)
    with pytest.raises(ValueError):
        render_pad_bgr(7)
    with pytest.raises(ValueError):   # ring would swallow the marker corners
        render_pad_bgr(1, circle_diameter_m=0.60)


def test_every_competition_texture_decodes_as_itself() -> None:
    for mid in sorted(VALID_MARKER_IDS):
        face = render_pad_bgr(mid, 512)
        got = [h.marker_id for h in find_landing_pads(face) if h.marker_id]
        assert got == [mid]


# ── BaseDetector wrapper ──

def test_pad_detector_wrapper(tmp_path) -> None:
    p = tmp_path / "frame.png"
    cv2.imwrite(str(p), _scene(_pad_px(10.0), marker_id=5))
    res = PadDetector().detect(p, "aruco landing pad")
    assert res.ok and res.matched
    assert res.best is not None and res.best.label == "aruco pad 5"


# ── monochrome camera (OV9281): the mono proof ──────────────────────────────
# The real camera is a MONO global-shutter sensor: every frame arrives with
# R=G=B. The saturation gate of the white-pad cue (S<=60) becomes trivially
# true at S=0; brightness (V>=170) + squareness + the dark-centre contrast
# carry the discrimination, and the ArUco decode path (BGR2GRAY) is an
# identity on replicated channels. This test locks that whole story.


def test_decode_on_grayscale_replicated_frame() -> None:
    scene = _scene(_pad_px(12.0))                       # the 12 m sweep view
    gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
    mono3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)      # R=G=B, like the OV9281
    hits = find_landing_pads(mono3)
    assert hits and hits[0].marker_id == 3
