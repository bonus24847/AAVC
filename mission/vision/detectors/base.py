"""Common interface for all AAVC vision detectors.

Every detector — classical OpenCV, ArUco fiducials, YOLO11n, and the VLM
detectors (Moondream2 / Qwen2.5-VL via llama-cpp) — implements the same
`Detector` protocol and returns a uniform `DetectionResult`. This lets the
router pick the right detector per situation and the benchmark run them all
side by side on the same frame.

A `BaseDetector` template handles latency timing + error capture so each
concrete detector only implements `_detect()`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, runtime_checkable


@dataclass
class Detection:
    """One detected object in a frame.

    `centroid_pixel_xy` is the image-frame centroid (fed to
    `vision.projection.pixel_to_world`); `bbox_xywh` is the pixel bounding box
    (x, y, w, h) when the detector provides one. `is_match` is True when this
    detection matches the requested target description."""

    label: str
    centroid_pixel_xy: tuple[int, int]
    confidence: float
    bbox_xywh: tuple[int, int, int, int] | None = None
    is_match: bool = True


@dataclass
class DetectionResult:
    """Uniform output of one detector on one frame."""

    detector: str
    detections: list[Detection] = field(default_factory=list)
    latency_ms: float = 0.0
    ok: bool = True
    error: str = ""
    raw: str = ""              # optional raw model text (VLMs) for debugging

    @property
    def matched(self) -> bool:
        return any(d.is_match for d in self.detections)

    @property
    def best(self) -> Detection | None:
        """Highest-confidence matching detection (or highest overall)."""
        cands = [d for d in self.detections if d.is_match] or self.detections
        return max(cands, key=lambda d: d.confidence) if cands else None


@runtime_checkable
class Detector(Protocol):
    """Structural type every detector satisfies."""

    name: str

    def available(self) -> bool:
        """True if this detector can run now (deps + weights present)."""
        ...

    def detect(self, frame_path: Path, target_description: str) -> DetectionResult:
        ...


class BaseDetector:
    """Template: times `_detect`, captures errors, never raises out of detect().

    Subclasses set `name` and implement `_detect` returning
    (detections, raw_text). `available()` defaults True; override when the
    detector needs optional deps / weights."""

    name: str = "base"

    def available(self) -> bool:
        return True

    def _detect(
        self, frame_path: Path, target_description: str,
    ) -> tuple[list[Detection], str]:
        raise NotImplementedError

    def detect(self, frame_path: Path, target_description: str) -> DetectionResult:
        t0 = time.monotonic()
        try:
            dets, raw = self._detect(Path(frame_path), target_description)
            return DetectionResult(
                detector=self.name, detections=dets,
                latency_ms=(time.monotonic() - t0) * 1000.0, ok=True, raw=raw,
            )
        except Exception as e:  # never let a detector crash the pipeline
            return DetectionResult(
                detector=self.name, detections=[],
                latency_ms=(time.monotonic() - t0) * 1000.0,
                ok=False, error=f"{type(e).__name__}: {e}",
            )
