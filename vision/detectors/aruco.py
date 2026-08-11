"""AAVC 2026 landing-pad detector — ArUco DICT_4X4_50 + white-pad cue.

The V1.3 competition targets are **landing pads**: a 1000×1000 mm white pad
with a black circle (⌀750 mm ring) and a central **400×400 mm ArUco marker**
(chev.me/arucogen "4x4" ⇒ OpenCV ``DICT_4X4_50``), ids **1–6**, up to 4 pads
on the field. Each sortie must deliver to the pad matching the committee-
assigned marker id, so the detector's job is (a) find pads, (b) **decode the
id**. Deterministic OpenCV only — a few ms per frame on the CM4.

Two complementary signals, fused per frame:

* **ArUco decode** (the identity authority): at the 12 m sweep altitude the
  400 mm marker is only ≈18 px in the 1280 px nadir frame — right at the
  decoder's floor — so the detector params accept small candidates and an
  ROI **upscale-retry booster** re-decodes undecoded pad candidates at 4×.
  Decode is easy from the descent rungs (≥27 px below 8 m).
* **White-pad cue** (the search-altitude workhorse): the 1 m white square with
  its dark marker centre is a high-contrast ≈45 px blob at 12 m — detectable
  long before the marker decodes. Cue-only hits carry ``marker_id=None`` and
  seed *unidentified candidates* that the mission revisits lower to decode.

``PadHit.radius_px`` is the **marker-equivalent half-side** in px for BOTH hit
kinds (blob hits scale by the known marker:pad ratio 0.4), so one metric size
prior — ``target_radius_m = 0.2`` — flows through ``expected_radius_px`` /
the tracker / the align layer unchanged.

Public surface:
  * ``find_landing_pads(img_bgr) -> list[PadHit]`` — all pads in a frame
    (every decoded id matters: the cross-sortie registry wants them all).
  * ``render_pad_bgr(marker_id, size_px)`` — the ONE pad renderer, shared by
    the SITL texture generator, the HITL synthetic camera, and the tests so
    fixtures can never drift from the sim.
  * ``PadDetector`` — BaseDetector wrapper for the router/benchmark plumbing.

Tuning knobs live at module top for the field team (real pad print geometry
may differ from Figure 6 — re-measure at the event and re-generate textures).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .base import BaseDetector, Detection

# ── marker dictionary + field constants (rules Figure 6/7) ──
PAD_DICT = cv2.aruco.DICT_4X4_50      # arucogen "4x4" = OpenCV DICT_4X4_50
VALID_MARKER_IDS = frozenset(range(1, 7))   # ids 1..6; ≤4 pads placed
PAD_SIZE_M = 1.0                      # white pad side
MARKER_SIZE_M = 0.4                   # marker side (incl. its black border)
CIRCLE_DIAMETER_M = 0.75              # black ring on the pad
CIRCLE_STROKE_M = 0.025               # ring line width (the "25" in Figure 6)
MARKER_TO_PAD = MARKER_SIZE_M / PAD_SIZE_M   # 0.4 — blob→marker-equivalent scale

# ── white-pad cue thresholds ──
# The pad is printed white: low saturation, high value. The SITL grey launch
# pad (V≈128) and grass stay below the V floor; white car roofs pass the colour
# test but fail the dark-centre contrast check (no marker).
_PAD_S_MAX, _PAD_V_MIN = 60, 170
_MIN_PAD_SIDE_PX = 18.0        # pad ≈45 px at 12 m/1280 px; floor rejects speckle
                               # while keeping detection out to ~30 m slant
_MIN_PAD_AREA_PX = 250.0
_SQUARE_ASPECT_BAND = (0.70, 1.43)   # minAreaRect side ratio — a square-ish blob
_MIN_PAD_EXTENT = 0.80         # contour area / rect area — a filled square
# Dark-centre contrast: mean gray of the inner (marker) region must sit well
# below the outer white rim. The 400 mm marker is ~half black, so the inner
# mean drops hard; a plain white rectangle does not.
_RIM_MIN_GRAY = 160.0
_CENTRE_RIM_CONTRAST = 40.0

# ── decode/booster geometry ──
_ROI_PAD_FRAC = 0.8            # ROI half-size = (1 + this) × blob half-side
_BOOST_SCALE = 4               # upscale factor for the re-decode pass
_BOOST_MAX_SIDE = 220.0        # only boost candidates smaller than this (px)

_CONF_DECODED = 0.95
_CONF_CUE_MAX = 0.80


@dataclass(frozen=True)
class PadHit:
    cx: int                 # pad/marker centroid (the aim point), pixels
    cy: int
    marker_id: int | None   # decoded ArUco id (1..6), or None = cue-only blob
    radius_px: float        # marker-equivalent HALF-SIDE (px). Decoded: half the
    #                         mean marker edge; blob-only: 0.4 × half pad side.
    #                         Named radius_px so the projection size-prior +
    #                         tracker + TargetFix contract (expected_radius_px,
    #                         target_radius_m=0.2) are reused unchanged.
    confidence: float
    corners: tuple[tuple[float, float], ...]   # marker corners (decoded) or
    #                                            pad minAreaRect box (blob)
    pad_side_px: float      # observed blob square side (0.0 for decode-only hits
    #                         — the pad may extend beyond the marker crop)

    @property
    def decoded(self) -> bool:
        return self.marker_id is not None


def _aruco_detector() -> cv2.aruco.ArucoDetector:
    """A fresh detector per call — cheap, and safe across the vision-worker
    thread + the asyncio align loop (cv2 objects are not thread-safe)."""
    p = cv2.aruco.DetectorParameters()
    # Accept small candidates: an 18 px marker in a 1280 px frame has perimeter
    # rate 72/1280 ≈ 0.056, but the sweep must also survive partial blur +
    # distant frames — drop the floor well below the default 0.03.
    p.minMarkerPerimeterRate = 0.02
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(PAD_DICT), p)


def _marker_hits(
    gray: np.ndarray,
    *,
    valid_ids: frozenset[int],
    offset_xy: tuple[float, float] = (0.0, 0.0),
    scale: float = 1.0,
) -> list[PadHit]:
    """Run one decode pass; map corners back through the ROI offset/scale."""
    corners, ids, _ = _aruco_detector().detectMarkers(gray)
    hits: list[PadHit] = []
    if ids is None:
        return hits
    ox, oy = offset_xy
    for c, i in zip(corners, ids.flatten(), strict=False):
        mid = int(i)
        if mid not in valid_ids:
            continue
        pts = c[0] / scale
        pts[:, 0] += ox
        pts[:, 1] += oy
        side = float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4])
                              for k in range(4)]))
        cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        hits.append(PadHit(
            cx=int(round(cx)), cy=int(round(cy)), marker_id=mid,
            radius_px=round(side / 2.0, 2), confidence=_CONF_DECODED,
            corners=tuple((float(x), float(y)) for x, y in pts),
            pad_side_px=0.0,
        ))
    return hits


def _pad_blobs(img_bgr: np.ndarray, gray: np.ndarray) -> list[PadHit]:
    """White-square-with-dark-centre candidates (marker not yet decodable)."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] <= _PAD_S_MAX) & (hsv[:, :, 2] >= _PAD_V_MIN))
    mask: np.ndarray = white.astype(np.uint8) * 255
    # CLOSE first: the dark marker punches a hole in the white square — merge
    # the rim back into ONE filled blob before shape-testing it.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8),
                            iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hits: list[PadHit] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < _MIN_PAD_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        (rcx, rcy), (w, h), _ang = rect
        if min(w, h) < _MIN_PAD_SIDE_PX:
            continue
        aspect = min(w, h) / max(w, h)
        if not (_SQUARE_ASPECT_BAND[0] <= aspect <= 1.0 / _SQUARE_ASPECT_BAND[0]):
            continue
        if area / (w * h) < _MIN_PAD_EXTENT:
            continue
        side = float((w + h) / 2.0)

        # Dark-centre vs white-rim contrast — the marker signature at any scale.
        centre = _region_mean(gray, rcx, rcy, side * 0.20)
        rim = (_region_mean(gray, rcx - side * 0.40, rcy - side * 0.40, side * 0.08)
               + _region_mean(gray, rcx + side * 0.40, rcy + side * 0.40, side * 0.08)
               + _region_mean(gray, rcx - side * 0.40, rcy + side * 0.40, side * 0.08)
               + _region_mean(gray, rcx + side * 0.40, rcy - side * 0.40, side * 0.08)
               ) / 4.0
        if rim < _RIM_MIN_GRAY or (rim - centre) < _CENTRE_RIM_CONTRAST:
            continue

        conf = min(_CONF_CUE_MAX,
                   0.55 + 0.25 * min(1.0, (rim - centre) / 120.0))
        hits.append(PadHit(
            cx=int(round(rcx)), cy=int(round(rcy)), marker_id=None,
            radius_px=round(MARKER_TO_PAD * side / 2.0, 2),
            confidence=round(conf, 3),
            corners=tuple((float(x), float(y)) for x, y in cv2.boxPoints(rect)),
            pad_side_px=round(side, 1),
        ))
    return hits


def _region_mean(gray: np.ndarray, cx: float, cy: float, half: float) -> float:
    h, w = gray.shape[:2]
    x0, x1 = max(0, int(cx - half)), min(w, int(cx + half) + 1)
    y0, y1 = max(0, int(cy - half)), min(h, int(cy + half) + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0
    return float(gray[y0:y1, x0:x1].mean())


def _boost_decode(
    gray: np.ndarray, blob: PadHit, *, valid_ids: frozenset[int],
) -> PadHit | None:
    """Re-decode a small undecoded pad candidate from an upscaled ROI crop."""
    if blob.pad_side_px <= 0 or blob.pad_side_px > _BOOST_MAX_SIDE:
        return None
    h, w = gray.shape[:2]
    half = blob.pad_side_px * (1.0 + _ROI_PAD_FRAC) / 2.0
    x0, x1 = max(0, int(blob.cx - half)), min(w, int(blob.cx + half) + 1)
    y0, y1 = max(0, int(blob.cy - half)), min(h, int(blob.cy + half) + 1)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    roi = cv2.resize(gray[y0:y1, x0:x1], None, fx=_BOOST_SCALE, fy=_BOOST_SCALE,
                     interpolation=cv2.INTER_CUBIC)
    hits = _marker_hits(roi, valid_ids=valid_ids,
                        offset_xy=(float(x0), float(y0)), scale=float(_BOOST_SCALE))
    if not hits:
        return None
    best = max(hits, key=lambda p: p.confidence)
    # Keep the blob's pad-side observation on the upgraded hit.
    return PadHit(cx=best.cx, cy=best.cy, marker_id=best.marker_id,
                  radius_px=best.radius_px, confidence=best.confidence,
                  corners=best.corners, pad_side_px=blob.pad_side_px)


def find_landing_pads(
    img_bgr: np.ndarray,
    *,
    valid_ids: frozenset[int] = VALID_MARKER_IDS,
) -> list[PadHit]:
    """All landing pads in a BGR frame — decoded ids first, then cue-only blobs.

    Returns every pad, not just the best: the mission's cross-sortie registry
    records each decoded id it ever sees, and the sweep revisits ``None``-id
    candidates to decode them.
    """
    if img_bgr is None or img_bgr.size == 0:
        return []
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    decoded = _marker_hits(gray, valid_ids=valid_ids)
    blobs = _pad_blobs(img_bgr, gray)

    # Upgrade undecoded blobs via the upscale booster; drop blobs that overlap
    # a decoded marker (same pad seen by both passes — decode wins).
    out = list(decoded)
    for b in blobs:
        near = any(abs(b.cx - d.cx) <= b.pad_side_px and
                   abs(b.cy - d.cy) <= b.pad_side_px for d in decoded)
        if near:
            continue
        boosted = _boost_decode(gray, b, valid_ids=valid_ids)
        out.append(boosted if boosted is not None else b)

    out.sort(key=lambda p: (not p.decoded, -p.confidence))
    return out


def render_pad_bgr(
    marker_id: int,
    size_px: int = 1024,
    *,
    pad_size_m: float = PAD_SIZE_M,
    circle_diameter_m: float = CIRCLE_DIAMETER_M,
    circle_stroke_m: float = CIRCLE_STROKE_M,
    marker_size_m: float = MARKER_SIZE_M,
    quiet_zone_m: float = 0.05,
) -> np.ndarray:
    """Render the official pad face (Figure 6) as a BGR image.

    The single source of pad pixels: the SITL texture generator, the HITL
    synthetic camera, and the detector tests all draw with this, so what the
    detector is tested on IS what the sim shows. Raises if the geometry leaves
    less than ``quiet_zone_m`` of white between the marker's own black border
    and the ring (an undecodable pad texture must never be generated).
    """
    if marker_id not in VALID_MARKER_IDS:
        raise ValueError(f"marker_id {marker_id} outside {sorted(VALID_MARKER_IDS)}")
    corner_gap_m = (circle_diameter_m - marker_size_m * 2.0 ** 0.5) / 2.0
    if corner_gap_m < quiet_zone_m:
        raise ValueError(
            f"ring ⌀{circle_diameter_m} m leaves {corner_gap_m * 1000:.0f} mm to the "
            f"marker corners < quiet zone {quiet_zone_m * 1000:.0f} mm — undecodable")

    scale = size_px / pad_size_m
    img = np.full((size_px, size_px, 3), 255, np.uint8)
    centre = (size_px // 2, size_px // 2)
    cv2.circle(img, centre, int(round(circle_diameter_m / 2.0 * scale)),
               (0, 0, 0), thickness=max(1, int(round(circle_stroke_m * scale))),
               lineType=cv2.LINE_AA)
    marker_px = int(round(marker_size_m * scale))
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(PAD_DICT), marker_id, marker_px)
    x0 = centre[0] - marker_px // 2
    y0 = centre[1] - marker_px // 2
    img[y0:y0 + marker_px, x0:x0 + marker_px] = cv2.cvtColor(marker,
                                                             cv2.COLOR_GRAY2BGR)
    return img


class PadDetector(BaseDetector):
    """BaseDetector wrapper around :func:`find_landing_pads`."""

    name = "landing_pad"

    def __init__(self, valid_ids: frozenset[int] = VALID_MARKER_IDS) -> None:
        self.valid_ids = valid_ids

    def _detect(
        self, frame_path: Path, target_description: str,
    ) -> tuple[list[Detection], str]:
        img = cv2.imread(str(frame_path))
        if img is None:
            return [], "landing_pad: frame unreadable"
        hits = find_landing_pads(img, valid_ids=self.valid_ids)
        dets = []
        for p in hits:
            half = int(p.radius_px / MARKER_TO_PAD)   # ≈ half pad side
            dets.append(Detection(
                label=f"aruco pad {p.marker_id}" if p.decoded else "landing pad",
                centroid_pixel_xy=(p.cx, p.cy),
                confidence=p.confidence,
                bbox_xywh=(p.cx - half, p.cy - half, 2 * half, 2 * half),
                is_match=p.decoded,
            ))
        ids = sorted(p.marker_id for p in hits if p.marker_id is not None)
        return dets, f"landing_pad: ids={ids} cues={sum(1 for p in hits if not p.decoded)}"
