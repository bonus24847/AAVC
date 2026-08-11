"""Live ``mission_status.json`` feed for the AAVC GCS console (KMUTNB).

The console (~/Desktop/aavc-gcs/src/aavc_gcs.py) polls
``<captures>/mission_status.json`` and draws one map marker per entry in
``pads_mapped`` — so a pad appears on the operator's map exactly when THIS
feed says so. The user's contract (2026-08-12): pads show up **when the
drone actually scans/confirms them**, never from a stale file — which is
also why the constructor clobbers any pre-existing status file immediately
(the stale pads the operator saw came from another project's captures dir
holding a finished mission's layout).

File contract (aavc_gcs.py header + its ``aavcEN`` map helper)::

    {"phase": str, "assigned": [ids], "delivered": [ids],
     "pads_mapped": {"<marker_id>": [east_m, north_m]}, "updated": epoch}

``pads_mapped`` ENU is about the field yaml's ``local_origin`` (== this
repo's ``site.center``), and the console converts it back with a SPHERICAL
earth (``aavcEN``, R=6378137) — so this module inverts that exact formula
rather than using the WGS84 metres-per-degree the rest of the repo flies
with: matching the consumer puts the marker on the true pad; "better" math
here would land it ~0.3 m off at this field's scale.

Threading/robustness: confirmations arrive on the VisionWorker THREAD and
releases on the asyncio loop, so state is lock-guarded; writes are atomic
(tmp + os.replace, the same pattern as the camera bridge) and every failure
is swallowed — this is a display aid, never flight-critical.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from pathlib import Path
from typing import Any, Callable

from loguru import logger

_R_EARTH_M = 6_378_137.0

# Same discriminator as sitl/payload_detach_bridge.parse_release, minus the
# payload capture: pad=None (an id-unverified touchdown release) deliberately
# fails the \d+ and is not shown as a delivered pad.
_RELEASE = re.compile(r"DELIVERY \d+ RELEASE pad=(?P<pad>\d+)")

# MissionPhase.value -> the console's 4-step mission bar (its phaseIdx() does
# SUBSTRING matching on [recon, load, deliver, done], so the label carries the
# raw phase in parentheses for operator detail without breaking the stepper).
# "load" = eggs aboard / operator GO; the recon steps are the blind sweep; the
# whole serve-egress-land tail lives under "deliver" (4 steps is the console's
# vocabulary, not ours). Terminal must be EXACTLY "done" — the console stops
# its mission clock only on that exact string (MCLOCK.done check).
_STEP_OF = {
    "preflight": "load",
    "takeoff": "recon",
    "transit_ingress": "recon",
    "search": "recon",
    "localize": "deliver",
    "drop": "deliver",
    "track": "deliver",
    "transit_egress": "deliver",
    "land": "deliver",
    "rth": "deliver",
    "abort": "deliver",
}


class GcsMissionStatus:
    """Best-effort writer of the AAVC GCS console's mission_status.json."""

    def __init__(self, path: Path | str, origin_lat: float, origin_lon: float,
                 assigned: list[int]) -> None:
        self.path = Path(path)
        self._origin = (float(origin_lat), float(origin_lon))
        self._lock = threading.Lock()
        self._pads: dict[str, list[float]] = {}
        self._delivered: list[int] = []
        self._assigned = [int(i) for i in assigned]
        self._phase = "load (preflight)"
        self._mission_time: float | None = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        # Startup write = the stale-pads fix: whatever mission_status.json a
        # previous run (or another project) left behind is replaced by an
        # empty pads_mapped before the console's next 1 Hz poll.
        self._write()
        logger.info(f"[gcs_status] live pad feed → {self.path}")

    # ── inverse of the console's aavcEN (spherical, see module docstring) ──
    def _enu(self, lat: float, lon: float) -> list[float]:
        lat0, lon0 = self._origin
        e = math.radians(lon - lon0) * _R_EARTH_M * math.cos(math.radians(lat0))
        n = math.radians(lat - lat0) * _R_EARTH_M
        return [round(e, 2), round(n, 2)]

    def pad_confirmed(self, marker_id: int, lat: float, lon: float) -> None:
        with self._lock:
            self._pads[str(int(marker_id))] = self._enu(lat, lon)
        self._write()

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self._phase = str(phase)
        self._write()

    def set_progress(self, raw_phase: str, mission_time_s: float) -> None:
        """1 Hz heartbeat from the orchestrator's REAL MissionPhase: drives the
        console's mission stepper + ⏱ clock, and keeps ``updated`` fresh so
        the console's 45 s staleness gate never hides a live mission between
        sparse events (the original event-only writes did exactly that)."""
        step = _STEP_OF.get(str(raw_phase), "recon")
        with self._lock:
            self._phase = f"{step} ({raw_phase})"
            self._mission_time = float(mission_time_s)
        self._write()

    def set_done(self, mission_time_s: float) -> None:
        """Terminal write — exactly "done" (see _STEP_OF note)."""
        with self._lock:
            self._phase = "done"
            self._mission_time = float(mission_time_s)
        self._write()

    def on_audit(self, entry: str) -> None:
        """Audit-sink tee: mark a pad delivered on its DELIVERY … RELEASE line."""
        m = _RELEASE.search(entry)
        if m is None:
            return
        pad = int(m.group("pad"))
        with self._lock:
            if pad not in self._delivered:
                self._delivered.append(pad)
        self._write()

    def tracker_pusher(self, tracker: Any) -> Callable[[Any], None]:
        """VisionWorker ``on_fix`` callback: mirror each newly CONFIRMED,
        id-DECODED pad to the map the moment the tracker promotes it —
        independent of the web dashboard's own pusher, so headless
        (--no-dashboard) runs feed the console too. Unidentified blob-only
        clusters are withheld: the operator's map shows pads, not maybes."""
        from .target_tracker import TargetState
        show = (TargetState.CONFIRMED, TargetState.SERVING, TargetState.SERVED)
        seen: set[int] = set()

        def _push(_fix: Any) -> None:
            for t in tracker.snapshot():
                if (t.marker_id is None or t.target_id in seen
                        or t.state not in show):
                    continue
                seen.add(t.target_id)
                self.pad_confirmed(t.marker_id, t.lat, t.lon)
                logger.info(f"[gcs_status] pad {t.marker_id} on the GCS map "
                            f"({t.lat:.7f}, {t.lon:.7f})")

        return _push

    def _write(self) -> None:
        try:
            with self._lock:
                doc = {
                    "phase": self._phase,
                    "assigned": list(self._assigned),
                    "delivered": list(self._delivered),
                    "pads_mapped": dict(self._pads),
                    "updated": time.time(),
                }
                if self._mission_time is not None:
                    doc["mission_time"] = round(self._mission_time, 1)
            tmp = self.path.with_name(self.path.name + ".tmp")
            tmp.write_text(json.dumps(doc), encoding="utf-8")
            os.replace(tmp, self.path)
        except Exception as e:  # display aid — never raise into the flight path
            logger.debug(f"[gcs_status] write skipped: {e}")
