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
# ids 0..6 — SEVEN markers (2026-08-27). The rules PDF says "1 through 6", but
# its Figure 7 (what everyone prints from) encodes 1, 2, 0, 4, 5, 6: the third
# picture IS id 0, and the field pad printed from it decoded as 0 on
# 2026-08-27 while this set threw it away. Operator: 0 and 3 are both real.
VALID_MARKER_IDS = frozenset(range(0, 7))
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
# ⚠ _PAD_V_MIN is a CEILING on the gate, not the gate itself (2026-08-21).
# Running the shipped detector over the 457 real KMUTNB flight frames: the cue
# fired on ZERO of them, because those frames' own brightness ceiling (per-frame
# V p99.5: min 144 / median 176 / max 255) sits right on this threshold — and
# they are pure mono (mean S = 0.0), so the saturation half of "white" is a
# no-op and this bare brightness number is the whole test. Forcing a short
# exposure to fight motion blur pushes the pad further under it, i.e. the blur
# fix and the cue were fighting each other.
# So the gate now TRACKS the frame's own exposure, bounded on both sides: never
# stricter than the original 170, never looser than the floor (below which
# sunlit grass starts passing). Sampled on a stride so it costs ~0.1 ms.
_PAD_V_FLOOR = 110             # below this, bright ground passes the colour test
_PAD_V_PCTL = 99.0             # the pad is ~0.2% of frame area at sweep altitude
_PAD_V_HEADROOM = 25.0         # sit this far under the bright tail's shoulder
_PAD_V_STRIDE = 8              # every Nth pixel — a percentile needs no more
_MIN_PAD_SIDE_PX = 18.0        # pad ≈45 px at 12 m/1280 px; floor rejects speckle
                               # while keeping detection out to ~30 m slant
_MIN_PAD_AREA_PX = 250.0
_SQUARE_ASPECT_BAND = (0.70, 1.43)   # minAreaRect side ratio — a square-ish blob
_MIN_PAD_EXTENT = 0.80         # contour area / rect area — a filled square
# Dark-centre contrast: mean gray of the inner (marker) region must sit well
# below the outer white rim. The 400 mm marker is ~half black, so the inner
# mean drops hard; a plain white rectangle does not.
# ⚠ These were absolute gray levels, and absolute levels do not survive an
# exposure change (2026-08-21). A 2-stop-shorter exposure — exactly what the
# in-flight blur fix forces — takes a white rim from ~255 to ~140 and the pad
# was rejected by the 160 floor while still being perfectly decodable. The
# discriminative signal is the CONTRAST between the white rim and the marker,
# and contrast is best expressed as a RATIO, which is invariant to exposure:
# printed black-on-white gives 0.5-0.9 at any brightness, while a plain grey
# pad or a grass blob gives ~0. The absolute terms survive only as noise
# floors, low enough to pass a dim frame.
_RIM_MIN_GRAY = 60.0           # noise floor; the white mask's own gate is stricter
_CENTRE_RIM_CONTRAST = 25.0    # absolute floor, to reject sensor noise
_CENTRE_RIM_RATIO = 0.30       # (rim - centre) / rim — the exposure-invariant test

# ── decode/booster geometry ──
_ROI_PAD_FRAC = 0.8            # ROI half-size = (1 + this) × blob half-side
_BOOST_SCALE = 4               # upscale factor for the re-decode pass
_BOOST_MAX_SIDE = 220.0        # only boost candidates smaller than this (px)

_CONF_DECODED = 0.95
_CONF_CUE_MAX = 0.80

# ── decode provenance counters (diagnostic only — no flight path reads them) ──
# WHICH pass produced each hit, counted for the life of the process. The ROI
# upscale booster (``_boost_decode``) costs a resize plus a second full detector
# pass per undecoded blob, and 32 synthetic conditions measured 2026-08-22 found
# ZERO decodes that were its alone — but a synthetic render is exactly where a
# booster written for REAL optics (motion blur, a marker printed on cloth, sun
# glare washing the quiet zone) cannot show what it is for. So rather than
# delete a decode path on evidence that cannot cover the case, count it:
# ``boosted`` still 0 after a real hover over a printed marker is the
# measurement that settles keeping or removing it.
# ``orchestrator/vision_worker.py`` logs this once at shutdown. Incremented from
# the decode threads without a lock on purpose — a miscount of one in a
# diagnostic counter is cheaper than a lock on the per-frame path.
DECODE_STATS: dict[str, int] = {
    "frames": 0,        # frames that reached the detector
    "direct": 0,        # ids decoded by the first, full-frame pass
    "boosted": 0,       # ids that ONLY the 4x ROI re-decode produced
    "cue_only": 0,      # white-pad blobs that stayed undecoded either way
}


@dataclass(frozen=True)
class PadHit:
    cx: int                 # pad/marker centroid (the aim point), pixels
    cy: int
    marker_id: int | None   # decoded ArUco id (0..6), or None = cue-only blob
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
    v_min = _adaptive_v_min(hsv[:, :, 2])
    white = ((hsv[:, :, 1] <= _PAD_S_MAX) & (hsv[:, :, 2] >= v_min))
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
        # ⚠ The rim probes MUST follow the pad's own orientation (fixed
        # 2026-08-21). They used to sit at fixed IMAGE-axis offsets of
        # ±0.40·side in x and y — 0.566·side from the centre along the image
        # diagonal. minAreaRect returns a ROTATED square, whose support in that
        # fixed direction collapses to 0.5·side as the pad approaches 45°, so
        # all four probes landed on grass and the blob was dropped: measured
        # PASS at 0-20° and 70-90°, FAIL at 21-69° — 54% of orientations, at
        # every altitude. Pads are laid at arbitrary yaw, so roughly half of
        # them were invisible to the cue, which also silently disabled the ×4
        # ROI booster and the "revisit undecoded pads" recovery (both are fed
        # ONLY by these blobs).
        # Walking 0.8 of the way to each ROTATED corner keeps the original
        # intent — the pad's white corners, outside the ⌀750 ring — for any
        # rotation: 0.566·side in the PAD's frame, clear of the ring (0.39)
        # and inside the pad edge (0.71) even with the probe's own half-width.
        centre = _region_mean(gray, rcx, rcy, side * 0.20)
        rim = sum(
            _region_mean(gray, rcx + 0.8 * (bx - rcx), rcy + 0.8 * (by - rcy),
                         side * 0.08)
            for bx, by in cv2.boxPoints(rect)
        ) / 4.0
        contrast = rim - centre
        if (rim < max(_RIM_MIN_GRAY, v_min)
                or contrast < _CENTRE_RIM_CONTRAST
                or contrast / max(rim, 1.0) < _CENTRE_RIM_RATIO):
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


def _adaptive_v_min(v: np.ndarray) -> float:
    """Brightness floor for the white-pad mask, tracking THIS frame's exposure.

    A fixed floor cannot work across the exposure range the aircraft actually
    flies: the same number that keeps sunlit grass out of the mask at noon sits
    above the whole histogram of a short-exposure frame, and the cue then finds
    nothing at all (measured: 0 cue hits over 457 real flight frames). Anchored
    to a high percentile — the pad is a fraction of a percent of the frame —
    and clamped so it can never be stricter than the original fixed value nor
    loose enough to let the ground in."""
    sample = v[::_PAD_V_STRIDE, ::_PAD_V_STRIDE]
    shoulder = float(np.percentile(sample, _PAD_V_PCTL)) - _PAD_V_HEADROOM
    return float(min(_PAD_V_MIN, max(_PAD_V_FLOOR, shoulder)))


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
    DECODE_STATS["frames"] += 1
    DECODE_STATS["direct"] += len(decoded)

    # Upgrade undecoded blobs via the upscale booster; drop blobs that overlap
    # a decoded marker (same pad seen by both passes — decode wins).
    out = list(decoded)
    for b in blobs:
        near = any(abs(b.cx - d.cx) <= b.pad_side_px and
                   abs(b.cy - d.cy) <= b.pad_side_px for d in decoded)
        if near:
            continue
        boosted = _boost_decode(gray, b, valid_ids=valid_ids)
        DECODE_STATS["boosted" if boosted is not None else "cue_only"] += 1
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
