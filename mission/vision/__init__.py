"""AAVC vision package — deterministic landing-pad detection + projection.

Lightweight competition build: OpenCV only, no torch/ultralytics/VLM.
"""

from __future__ import annotations

from .detectors import (
    PadDetector,
    PadHit,
    find_landing_pads,
    render_pad_bgr,
)
from .projection import (
    NADIR,
    CameraModel,
    GroundFix,
    pixel_to_world,
    project_pixel,
)

__all__ = [
    "NADIR",
    "CameraModel",
    "GroundFix",
    "PadDetector",
    "PadHit",
    "find_landing_pads",
    "pixel_to_world",
    "project_pixel",
    "render_pad_bgr",
]
