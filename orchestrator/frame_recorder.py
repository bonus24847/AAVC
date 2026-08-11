"""Mission frame recorder — the RECORD half of the rules' "record AND transmit"
imaging requirement (R2).

The system already transmits imagery live (the dashboard camera endpoint). The
rules also require the imaging system to RECORD; this task persists a low-rate
JPEG trail of the nadir frames into ``runs/<mission_id>/frames/`` for
post-flight review and as scoring evidence.

Design constraints (this must NEVER endanger the flight):
  * Runs as its own asyncio task, all image IO offloaded to a thread.
  * Idles until the operation window starts (the first GO), stops at terminal.
  * Skips a frame whose mtime hasn't changed (a dead camera → no 1200 dupes).
  * ANY failure is a single WARN + one deduped anomaly — it is caught, never
    raised into the mission loop, and the task keeps running.

Storage: ~1 Hz × 20 min × ~150-250 KB ≈ 200-300 MB per window. Size the CM4's
SD card accordingly (see docs/FLIGHT.md). Config `recording:` gates it.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

from loguru import logger

from .state import OrchestratorState, TerminalState
from .vision_worker import DEFAULT_NADIR_FRAME

try:  # cv2 is a flight dep, but keep the import soft for headless tests
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

DEFAULT_HZ = 1.0
DEFAULT_JPEG_QUALITY = 80


class FrameRecorder:
    """Persists a low-rate JPEG frame trail into ``frames_dir``."""

    def __init__(
        self,
        state: OrchestratorState,
        frames_dir: Path,
        *,
        nadir: Path = DEFAULT_NADIR_FRAME,
        hz: float = DEFAULT_HZ,
        jpeg_quality: int = DEFAULT_JPEG_QUALITY,
        enabled: bool = True,
    ) -> None:
        self.state = state
        self.frames_dir = frames_dir
        self.nadir = nadir
        self.hz = hz if hz > 0 else DEFAULT_HZ
        self.jpeg_quality = int(jpeg_quality)
        self.enabled = enabled and cv2 is not None
        self._interval_s = 1.0 / self.hz
        self._task: asyncio.Task[None] | None = None
        # Last-seen source mtime, so an unchanged (frozen) frame isn't written twice.
        self._last_mtime: dict[str, float] = {}

    async def start(self) -> None:
        if not self.enabled:
            logger.info("[frames] recorder disabled (config recording.enabled=false)")
            return
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self._task = asyncio.get_running_loop().create_task(self._run())
        logger.info(f"[frames] recording ~{self.hz:.1f} Hz → {self.frames_dir}")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None

    async def _run(self) -> None:
        seq = 0
        while self.state.terminal == TerminalState.RUNNING:
            await asyncio.sleep(self._interval_s)
            # Idle until the operation window is actually running (first GO) —
            # don't archive the pre-mission idle frames.
            if not self.state.window_started:
                continue
            await asyncio.to_thread(self._capture, seq)
            seq += 1

    def _capture(self, seq: int) -> list[Path]:
        """Save the nadir frame (every tick). Returns the paths written. Never
        raises — failures are absorbed as a deduped anomaly so a bad SD card or
        missing frame can't stall the loop."""
        written: list[Path] = []
        self._save(self.nadir, "nadir", seq, written)
        return written

    def _save(self, src: Path, name: str, seq: int, written: list[Path]) -> None:
        if cv2 is None:
            return
        try:
            if not src.exists():
                return
            mtime = src.stat().st_mtime
            if self._last_mtime.get(name) == mtime:
                return                                  # frozen frame — skip dupe
            img = cv2.imread(str(src))
            if img is None:
                return
            out = self.frames_dir / f"{name}_{seq:06d}.jpg"
            ok = cv2.imwrite(str(out), img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                raise OSError(f"cv2.imwrite returned False for {out}")
            self._last_mtime[name] = mtime
            written.append(out)
        except Exception as e:  # noqa: BLE001 — recording must never abort the mission
            logger.warning(f"[frames] failed to record {name} frame: {e}")
            self.state.record_anomaly("frame_recorder_failed")
