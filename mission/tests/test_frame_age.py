"""In-flight camera-frame staleness gate (S2).

If the camera bridge/grabber dies mid-descent the last PNG persists on /tmp; a
reader that only checks existence keeps 'seeing' a frozen pad and the
id-verified LAND gate can't tell. Both the monitoring worker and the terminal
align loop must reject a frame older than the age threshold.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import pytest

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState
from orchestrator.vision_worker import VisionWorker, frame_too_old
from vision.projection import NADIR

cv2 = pytest.importorskip("cv2")

_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


def _state() -> OrchestratorState:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    return OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )


def _write_frame(path: Path, age_s: float = 0.0) -> None:
    cv2.imwrite(str(path), np.zeros((64, 64, 3), dtype=np.uint8))
    if age_s:
        past = time.time() - age_s
        os.utime(path, (past, past))


# ── the pure helper ──


def test_frame_too_old_fresh_stale_and_missing(tmp_path: Path) -> None:
    fresh = tmp_path / "fresh.png"
    _write_frame(fresh, age_s=0.0)
    assert frame_too_old(fresh, 2.0) is False

    stale = tmp_path / "stale.png"
    _write_frame(stale, age_s=10.0)
    assert frame_too_old(stale, 2.0) is True

    # A missing frame is NOT 'stale' — the existing not-exists path returns [].
    assert frame_too_old(tmp_path / "nope.png", 2.0) is False

    # A zero/negative threshold disables the gate.
    assert frame_too_old(stale, 0.0) is False


# ── VisionWorker integration ──


def test_stale_nadir_frame_returns_empty_and_records_anomaly(tmp_path: Path) -> None:
    frame = tmp_path / "aavc_nadir.png"
    _write_frame(frame, age_s=10.0)               # older than the 2 s gate
    state = _state()
    worker = VisionWorker(state, nadir_frame=frame, frame_max_age_s=2.0)
    assert worker._detect_one(frame, NADIR) == []
    assert any("nadir_frame_stale" in a for a in state.anomalies)


def test_fresh_frame_does_not_record_stale_anomaly(tmp_path: Path) -> None:
    frame = tmp_path / "aavc_nadir.png"
    _write_frame(frame, age_s=0.0)                # fresh; blank → no pad, but gate OK
    state = _state()
    worker = VisionWorker(state, nadir_frame=frame, frame_max_age_s=2.0)
    assert worker._detect_one(frame, NADIR) == []  # blank image, no pad
    assert not any("frame_stale" in a for a in state.anomalies)
