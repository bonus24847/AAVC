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
    _PAD_V_FLOOR,
    _PAD_V_MIN,
    MARKER_TO_PAD,
    VALID_MARKER_IDS,
    PadDetector,
    _adaptive_v_min,
    _pad_blobs,
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


# ── the cue must survive pad ROTATION and the frame's own exposure ──────────
# (2026-08-21 review, both measured) The rim probes used to sit at fixed
# IMAGE-axis offsets, so a pad rotated toward 45° put all four of them on the
# grass: PASS at 0-20° and 70-90°, FAIL at 21-69° — 54% of orientations, and
# pads are laid at arbitrary yaw. Losing the blob also silently disabled the
# ×4 ROI booster and the "revisit undecoded pads" recovery, which are fed only
# by these blobs. Separately, the fixed V>=170 gate sat on the real flight
# frames' own brightness ceiling, so the cue fired on 0 of 457 of them.


def _rotated_pad_frame(marker_px: int, rot_deg: float, marker_id: int = 2):
    """A pad of the real artwork, rotated on textured ground."""
    rng = np.random.default_rng(7)
    bg = rng.integers(60, 110, (720, 1280, 3), dtype=np.uint8)
    pad_side = int(marker_px * 2.5)
    big = int(pad_side * 1.5)
    canvas = np.zeros((big, big, 3), np.uint8)
    o = (big - pad_side) // 2
    canvas[o:o + pad_side, o:o + pad_side] = render_pad_bgr(marker_id, pad_side)
    m = cv2.getRotationMatrix2D((big / 2, big / 2), rot_deg, 1.0)
    rot = cv2.warpAffine(canvas, m, (big, big), flags=cv2.INTER_LINEAR)
    mask = cv2.warpAffine(np.full((big, big), 255, np.uint8), m, (big, big))
    y, x = 360 - big // 2, 640 - big // 2
    roi = bg[y:y + big, x:x + big]
    roi[mask > 128] = rot[mask > 128]
    return bg


def test_white_pad_cue_survives_every_pad_rotation() -> None:
    for marker_px in (42, 28):                    # the 8 m and 12 m bands
        failed = [rot for rot in range(0, 91, 5)
                  if not _pad_blobs(
                      (f := _rotated_pad_frame(marker_px, rot)),
                      cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))]
        assert not failed, f"{marker_px}px: cue lost the pad at {failed}°"


def test_brightness_gate_tracks_the_frames_exposure() -> None:
    """Never stricter than the historical fixed value, never loose enough to
    let sunlit ground in — but it must MOVE, or a short-exposure frame has its
    whole histogram below the gate."""
    bright = np.full((720, 1280), 250, np.uint8)
    dim = np.full((720, 1280), 120, np.uint8)
    assert _adaptive_v_min(bright) == _PAD_V_MIN          # capped at the old value
    assert _adaptive_v_min(dim) == _PAD_V_FLOOR           # clamped at the floor
    mid = np.full((720, 1280), 180, np.uint8)
    assert _PAD_V_FLOOR < _adaptive_v_min(mid) < _PAD_V_MIN


def test_a_dim_pad_is_still_found_when_the_whole_frame_is_dim() -> None:
    """The exposure fix and the cue must not fight each other: forcing a short
    exposure to kill motion blur darkens the pad too."""
    frame = _rotated_pad_frame(42, 30)
    dim = (frame.astype(np.float32) * 0.55).astype(np.uint8)   # ~2 stops down
    assert _pad_blobs(dim, cv2.cvtColor(dim, cv2.COLOR_BGR2GRAY)), \
        "cue lost the pad once the frame was darkened"
