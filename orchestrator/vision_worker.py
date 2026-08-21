"""Single-camera landing-pad vision worker (AAVC 2026 V1.3).

Background asyncio task. While the mission is in an active phase it reads the
NADIR camera frame written by ``sitl/gz_camera_bridge.py`` (real flight: the
UVC grabber on the CM4):

  * NADIR (/tmp/aavc_nadir.png) — straight-down (gimbal-stabilized on the real
    bird), drives the world fix used to seed the precise approach and the
    dashboard map pin. The SOLE control-authority sensor.

Each cycle it detects EVERY landing pad in frame (decoded ArUco ids + white-pad
cues), projects each centroid (attitude-composed) to a ground (lat, lon) via
:mod:`vision.projection`, and fires callbacks: dashboard (record_vision + map
events) and ``on_fix`` per pad (the cross-sortie pad registry in
:mod:`orchestrator.target_tracker` wants every decoded id it ever sees — pads
spotted while serving one assignment are next sorties' direct gotos).

The tight visual-servo loop that actually lands + releases lives in
``orchestrator/tactical_align.py`` and detects inline for low latency — this
worker is the monitoring + acquisition feed, deliberately decoupled.
"""

from __future__ import annotations

import asyncio
import collections
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from loguru import logger

from mission_brain.schemas import DetectedTarget, MissionPhase, VisionAnalysis
from vision.detectors.aruco import find_landing_pads
from vision.projection import NADIR, CameraModel, project_pixel

from .state import OrchestratorState, TerminalState

try:  # cv2 is a hard dep in flight, but keep the import soft for headless tests
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

DEFAULT_NADIR_FRAME = Path("/tmp/aavc_nadir.png")
# Poll period, NOT the decode rate. Since 2026-08-21 a new-frame guard sits in
# front of the decode, so this only bounds how long a freshly written frame
# waits before it is looked at; the actual decode rate is the CAMERA's write
# rate. 0.3 s used to mean "~3 Hz decoding" and, worse, up to 300 ms of extra
# staleness on every frame — at 6 m/s that is 1.8 m of pad-position error for
# free. Polling at 20 Hz costs one stat() per poll and nothing else.
DEFAULT_INTERVAL_S = 0.05
# In-flight frame-staleness gate (S2): the camera bridge/grabber writes each
# frame by atomic replace, so a dead writer leaves the LAST frame frozen on
# disk. A reader that only checks existence would keep decoding that stale pad —
# defeating the id-verified LAND gate. Frames arrive at 5-15 Hz, so 2 s is many
# missed frames: comfortably not a transient hiccup.
DEFAULT_FRAME_MAX_AGE_S = 2.0


def frame_too_old(frame_path: Path, max_age_s: float) -> bool:
    """True if the frame's mtime is older than ``max_age_s`` (a dead camera).

    A non-positive ``max_age_s`` disables the gate. A missing frame is NOT
    'stale' — the existing not-exists path already returns no detection."""
    if max_age_s <= 0:
        return False
    try:
        age = time.time() - frame_path.stat().st_mtime
    except OSError:
        return False
    return age > max_age_s
DEFAULT_OBSERVATION_MAXLEN = 10
# Phases where the camera is worth running. Acquisition starts on ingress so a
# pad passing under the nadir footprint is registered before the sweep begins.
DEFAULT_ACTIVE_PHASES: frozenset[MissionPhase] = frozenset({
    MissionPhase.TRANSIT_INGRESS,
    MissionPhase.SEARCH,
    MissionPhase.LOCALIZE,
    MissionPhase.DROP,
    MissionPhase.TRACK,
})


@dataclass(frozen=True)
class TargetFix:
    """A projected world estimate of one landing pad from one camera."""

    lat: float
    lon: float
    pixel_xy: tuple[int, int]
    confidence: float
    radius_px: float          # marker-equivalent half-side px (PadHit contract)
    camera: str
    ground_dist_m: float
    slant_range_m: float
    t_monotonic: float
    marker_id: int | None = None   # decoded ArUco id (1..6); None = cue-only blob

    def age_s(self) -> float:
        return time.monotonic() - self.t_monotonic


class VisionWorker:
    """Background single-camera worker. Detects pads + notifies the GCS."""

    def __init__(
        self,
        state: OrchestratorState,
        target_description: str = "aruco landing pad",
        nadir_frame: Path = DEFAULT_NADIR_FRAME,
        interval_s: float = DEFAULT_INTERVAL_S,
        active_phases: frozenset[MissionPhase] = DEFAULT_ACTIVE_PHASES,
        observation_maxlen: int = DEFAULT_OBSERVATION_MAXLEN,
        frame_max_age_s: float = DEFAULT_FRAME_MAX_AGE_S,
    ) -> None:
        self.state = state
        self.target_description = target_description
        self.nadir_frame = nadir_frame
        self.interval_s = interval_s
        self.frame_max_age_s = frame_max_age_s
        self.active_phases = active_phases
        self._observations: collections.deque[VisionAnalysis] = collections.deque(
            maxlen=observation_maxlen
        )
        self._on_observation: list[Callable[[VisionAnalysis], None]] = []
        self._on_detected_objects: list[Callable[[list[Any]], None]] = []
        self._on_fix: list[Callable[[TargetFix], None]] = []
        self._task: asyncio.Task[None] | None = None
        # mtime of the last frame actually decoded. The worker polls a FILE
        # the grabber overwrites, so its poll rate and the write rate are
        # independent: without this it re-decoded whichever frame happened to
        # be on disk (measured 67 ms of CM4 CPU per pass — 26 ms of PNG decode
        # plus 41 ms of detection) even when nothing had changed, while a
        # faster poll used to be the only way to shorten the worst-case wait
        # for a NEW frame. With the guard, polling fast is nearly free and the
        # effective decode rate becomes the CAMERA's rate, not the poll rate.
        self._last_frame_mtime: float = -1.0

    # ---------- subscriber registry ----------

    def on_observation(self, cb: Callable[[VisionAnalysis], None]) -> None:
        self._on_observation.append(cb)

    def on_detected_objects(self, cb: Callable[[list[Any]], None]) -> None:
        self._on_detected_objects.append(cb)

    def on_fix(self, cb: Callable[[TargetFix], None]) -> None:
        """Subscribe to every fresh per-camera :class:`TargetFix`.

        WARNING: the callback fires on the worker THREAD (``_tick`` runs under
        ``asyncio.to_thread``), not the event loop. The search-and-serve target
        tracker's ``ingest`` is built for this (it holds its own lock); a callback
        that schedules work on the asyncio loop must hop threads itself."""
        self._on_fix.append(cb)

    # ---------- public state ----------

    def recent_observations(self) -> list[VisionAnalysis]:
        return list(self._observations)

    # ---------- lifecycle ----------

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run())
        logger.info(
            f"[vision] camera worker started; nadir={self.nadir_frame} "
            f"interval={self.interval_s}s"
        )

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)

    async def _run(self) -> None:
        while self.state.terminal == TerminalState.RUNNING:
            await asyncio.sleep(self.interval_s)
            if self.state.phase not in self.active_phases:
                continue
            if not self.has_new_frame():
                continue          # same frame as last pass — nothing to learn
            try:
                await asyncio.to_thread(self._tick)
            except Exception as e:  # never let a frame kill the worker
                logger.warning(f"[vision] tick failed: {e}")

    # ---------- per-cycle work ----------

    def _tick(self) -> None:
        # find_landing_pads sorts decoded-then-confidence, so element 0 is the
        # strongest hit in frame.
        nadir_fixes = self._detect_one(self.nadir_frame, NADIR)

        # Feed the pad registry with EVERY pad seen. Fires on THIS worker thread.
        for fix in nadir_fixes:
            for cb_fix in self._on_fix:
                try:
                    cb_fix(fix)
                except Exception:
                    logger.exception("[vision] on_fix callback raised")

        best: TargetFix | None = nadir_fixes[0] if nadir_fixes else None
        analysis = self._to_analysis(best)
        self._observations.append(analysis)
        for cb in self._on_observation:
            try:
                cb(analysis)
            except Exception:
                logger.exception("[vision] on_observation callback raised")
        if nadir_fixes and self._on_detected_objects:
            events = self._detected_object_events(nadir_fixes)
            if events:
                for cb_obj in self._on_detected_objects:
                    try:
                        cb_obj(events)
                    except Exception:
                        logger.exception("[vision] on_detected_objects callback raised")

    def has_new_frame(self) -> bool:
        """True when the grabber has written a frame we have not decoded yet.

        Cheap (one stat) compared with a decode pass, so the run loop can poll
        far faster than the camera writes without paying for it."""
        try:
            mtime = self.nadir_frame.stat().st_mtime
        except OSError:
            return False
        return mtime != self._last_frame_mtime

    def _detect_one(self, frame_path: Path, camera: CameraModel) -> list[TargetFix]:
        if cv2 is None or not frame_path.exists():
            return []
        # Reject a frozen frame from a dead camera writer (S2) BEFORE decoding —
        # otherwise a stale-but-decodable pad keeps feeding the registry and the
        # id-vote LAND gate. Deduped anomaly so the operator sees the dropout.
        if frame_too_old(frame_path, self.frame_max_age_s):
            self.state.record_anomaly(f"{camera.name}_frame_stale")
            return []
        try:                       # claim this frame BEFORE the expensive part
            self._last_frame_mtime = frame_path.stat().st_mtime
        except OSError:
            pass
        img = cv2.imread(str(frame_path))
        if img is None:
            return []
        hits = find_landing_pads(img)
        if not hits:
            return []
        t = self.state.telemetry
        if math.isnan(t.lat) or math.isnan(t.lon):
            return []
        alt = t.relative_alt_m if not math.isnan(t.relative_alt_m) else 0.0
        yaw = t.heading_deg if not math.isnan(t.heading_deg) else 0.0
        roll = t.roll_deg if not math.isnan(t.roll_deg) else 0.0
        pitch = t.pitch_deg if not math.isnan(t.pitch_deg) else 0.0
        now = time.monotonic()
        fixes: list[TargetFix] = []
        for hit in hits:
            gf = project_pixel((hit.cx, hit.cy), t.lat, t.lon, alt, yaw, camera,
                               roll_deg=roll, pitch_deg=pitch)
            if gf is None:
                continue
            fixes.append(TargetFix(
                lat=gf.lat, lon=gf.lon, pixel_xy=(hit.cx, hit.cy),
                confidence=hit.confidence, radius_px=hit.radius_px,
                camera=camera.name, ground_dist_m=gf.ground_dist_m,
                slant_range_m=gf.slant_range_m,
                t_monotonic=now, marker_id=hit.marker_id,
            ))
        return fixes

    def _to_analysis(self, fix: TargetFix | None) -> VisionAnalysis:
        if fix is None:
            return VisionAnalysis(
                targets_detected=[],
                matches_designated_description=False,
                matched_target_index=None,
                rationale="no landing pad in either camera",
                confidence=0.5,
            )
        target = DetectedTarget(
            clothing_color="unknown", pose="unknown", member_count=1,
            centroid_pixel_xy=fix.pixel_xy, confidence=fix.confidence,
        )
        pad = (f"aruco pad {fix.marker_id}" if fix.marker_id is not None
               else "landing pad (id undecoded)")
        return VisionAnalysis(
            targets_detected=[target],
            matches_designated_description=fix.marker_id is not None,
            matched_target_index=0,
            rationale=f"{fix.camera}: {pad}, ground_dist={fix.ground_dist_m:.1f}m",
            confidence=fix.confidence,
        )

    def _detected_object_events(self, fixes: list[TargetFix]) -> list[Any]:
        try:
            from dashboard.payloads import DetectedObjectEvent
        except Exception:
            return []
        # I3 (review 2026-07-24): a FLIGHT can carry several eggs
        # (eggs_aboard > 1), so "designated" must cover every pad THIS flight
        # will serve, not only whichever one is CURRENTLY being landed on
        # (state.assigned_marker_id) — else the map paints pads 2..N as
        # non-designated while the aircraft is actually landing on them.
        # Falls back to assigned_marker_id when flight_ids hasn't been
        # populated yet (pre-flight) or is empty (a caller that predates
        # flight_ids, e.g. bench/tuning use of VisionWorker directly).
        flight_ids = getattr(self.state, "flight_ids", None)
        if flight_ids:
            designated_ids = set(flight_ids)
        else:
            assigned = getattr(self.state, "assigned_marker_id", None)
            designated_ids = {assigned} if assigned is not None else set()
        return [DetectedObjectEvent(
            t_monotonic=fix.t_monotonic,
            label=(f"aruco pad {fix.marker_id}" if fix.marker_id is not None
                   else "landing pad"),
            clothing_color="unknown",
            member_count=1,
            pose="unknown",
            confidence=fix.confidence,
            lat=fix.lat,
            lon=fix.lon,
            is_designated_match=(fix.marker_id is not None
                                 and fix.marker_id in designated_ids),
        ) for fix in fixes]
