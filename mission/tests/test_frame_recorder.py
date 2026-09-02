"""Mission frame recorder — the 'record AND transmit' compliance half (R2).

The rules require the imaging system to RECORD and transmit. We already transmit
(live dashboard); this persists a ~1 Hz JPEG trail into runs/<id>/frames/. It
runs strictly OFF the flight-critical path: any failure is a single WARN +
anomaly, never mission-fatal.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import numpy as np
import pytest

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.frame_recorder import FrameRecorder
from orchestrator.state import OrchestratorMode, OrchestratorState, TerminalState

cv2 = pytest.importorskip("cv2")

_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
_HOME = Coordinate(lat=13.730250, lon=100.787300)


def _state(window_started: bool = True) -> OrchestratorState:
    spec = build_search_pattern(_AREA, _HOME, sweep_alt_m=12.0)
    plan = render_live_plan(_HOME, spec, discovered=[], profile=COMPETITION)
    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )
    if window_started:
        state.start_window()
    return state


def _write(path: Path, val: int = 0) -> None:
    cv2.imwrite(str(path), np.full((16, 16, 3), val, dtype=np.uint8))


def _rec(state: OrchestratorState, frames: Path, nadir: Path,
         **kw: object) -> FrameRecorder:
    return FrameRecorder(state, frames, nadir=nadir, **kw)  # type: ignore[arg-type]


# ── the capture logic ──


def test_capture_writes_nadir_jpeg(tmp_path: Path) -> None:
    nadir = tmp_path / "nadir.png"
    _write(nadir)
    frames = tmp_path / "frames"
    frames.mkdir()
    rec = _rec(_state(), frames, nadir)
    wrote = rec._capture(0)
    assert wrote and all(p.suffix == ".jpg" for p in wrote)
    assert list(frames.glob("nadir_*.jpg"))


def test_recorder_saves_nadir_only(tmp_path: Path) -> None:
    """Single-camera rig: every tick records the nadir frame and nothing else."""
    nadir = tmp_path / "n.png"
    _write(nadir)
    frames = tmp_path / "frames"
    frames.mkdir()
    rec = _rec(_state(), frames, nadir)
    rec._capture(0)
    _write(nadir, 1)                 # bump mtime so nadir isn't skipped
    rec._capture(1)
    files = sorted(p.name for p in frames.glob("*.jpg"))
    assert len(files) == 2 and all(n.startswith("nadir_") for n in files)


def test_unchanged_frame_is_skipped(tmp_path: Path) -> None:
    nadir = tmp_path / "n.png"
    _write(nadir)
    frames = tmp_path / "frames"
    frames.mkdir()
    rec = _rec(_state(), frames, nadir)
    rec._capture(0)
    rec._capture(1)                  # same mtime → duplicate skipped
    assert len(list(frames.glob("nadir_*.jpg"))) == 1


def test_missing_source_writes_nothing_and_does_not_raise(tmp_path: Path) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    rec = _rec(_state(), frames, tmp_path / "nope.png")
    assert rec._capture(0) == []


def test_unwritable_dir_records_anomaly_and_survives(tmp_path: Path) -> None:
    nadir = tmp_path / "n.png"
    _write(nadir)
    state = _state()
    frames = tmp_path / "does_not_exist"   # not created → imwrite fails
    rec = _rec(state, frames, nadir)
    rec._capture(0)                         # must NOT raise
    assert any("frame_recorder_failed" in a for a in state.anomalies)


# ── lifecycle ──


def test_disabled_recorder_starts_no_task(tmp_path: Path) -> None:
    frames = tmp_path / "frames"
    rec = _rec(_state(), frames, tmp_path / "n.png", enabled=False)
    asyncio.run(rec.start())
    assert rec._task is None
    assert not frames.exists()


def test_run_idles_until_window_then_records(tmp_path: Path) -> None:
    nadir = tmp_path / "n.png"
    frames = tmp_path / "frames"
    state = _state(window_started=False)
    rec = _rec(state, frames, nadir, hz=50.0)

    async def drive() -> int:
        await rec.start()
        _write(nadir)
        await asyncio.sleep(0.06)                 # window not started → idle
        before = len(list(frames.glob("*.jpg"))) if frames.exists() else 0
        state.start_window()
        await asyncio.sleep(0.12)                 # now capturing
        state.terminal = TerminalState.COMPLETED
        await rec.stop()
        return before

    before = asyncio.run(drive())
    assert before == 0
    assert len(list(frames.glob("nadir_*.jpg"))) >= 1
