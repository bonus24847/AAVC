"""AAVC vision detector — the deterministic OpenCV landing-pad detector.

Lightweight competition build (no torch / ultralytics / VLM):

    PadDetector    the AAVC 2026 V1.3 target — ArUco DICT_4X4_50 + white-pad cue.

`PadDetector` is the sole detector; `find_landing_pads` is the raw decode used by
the flight core. `BaseDetector`/`Detector` are the interface the tests/benchmark
plumbing type against.
"""

from __future__ import annotations

from .aruco import (
    VALID_MARKER_IDS,
    PadDetector,
    PadHit,
    find_landing_pads,
    render_pad_bgr,
)
from .base import BaseDetector, Detection, DetectionResult, Detector

__all__ = [
    "VALID_MARKER_IDS",
    "BaseDetector",
    "Detection",
    "DetectionResult",
    "Detector",
    "PadDetector",
    "PadHit",
    "find_landing_pads",
    "render_pad_bgr",
]
