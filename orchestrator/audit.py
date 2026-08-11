"""Persisted run artifacts (C3): a loguru file sink + an append-only audit JSONL
under ``runs/<mission_id>/``, so a mid-flight crash keeps the full log + the
audit trail for post-flight review (``tools/verify_flight.py`` parses it) — one
flushed JSON object per line.

Before C3 loguru was stdout-only (no file sink) and ``state.anomalies`` lived
only in memory, so a crash lost the whole audit trail. The file sink + JSONL
fix that for every G-gate flight.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING

from loguru import logger

if TYPE_CHECKING:
    from .target_tracker import TrackedTarget

RUNS_DIR = Path("runs")


def setup_run_logging(mission_id: str, runs_dir: Path = RUNS_DIR) -> Path:
    """Create ``runs/<mission_id>/`` and attach a rotating loguru file sink for
    the full orchestrator log (stdout sink is left untouched). Returns the run
    directory. Call once per process at startup, after the mission_id is known."""
    run_dir = runs_dir / mission_id
    run_dir.mkdir(parents=True, exist_ok=True)
    logger.add(
        run_dir / "orchestrator.log",
        level="DEBUG",
        rotation="20 MB",
        retention=5,
        enqueue=True,          # async-safe: writes are off the flight loop
        backtrace=False,
        diagnose=False,
    )
    logger.info(f"[audit] run logging → {run_dir / 'orchestrator.log'}")
    return run_dir


class AuditLog:
    """Append-only JSONL of anomalies + operator-audit events under
    ``runs/<mission_id>/audit.jsonl``. One JSON object per line, opened per-write
    so a crash never loses an already-flushed line. Persistence is best-effort —
    it must never raise into the flight path. ``path=None`` makes it a no-op
    (tests / quick runs)."""

    def __init__(self, path: Path | None) -> None:
        self.path = Path(path) if path is not None else None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._rotate_stale()

    def _rotate_stale(self) -> None:
        """Rotate a pre-existing audit file aside as ``audit_<mtime>.jsonl``.

        A re-used mission_id used to APPEND run after run into one
        audit.jsonl, which fed tools/verify_flight.py a concatenation of
        flights: measured 2026-08-12, a clean 13-check PASS read as 17
        VIOLATIONS because run A's releases were scored against run B's
        seeded truth. One file per process keeps every consumer
        (verify_flight, payload_detach_bridge's EOF tail, the post-flight
        truth audit) single-run by construction; the rotated copies keep the
        history. Best-effort like everything here — a failed rename falls
        back to the old append behaviour rather than raising."""
        try:
            if self.path is None or not self.path.exists():
                return
            stamp = datetime.fromtimestamp(
                self.path.stat().st_mtime, tz=timezone.utc
            ).strftime("%Y%m%dT%H%M%SZ")
            backup = self.path.with_name(f"audit_{stamp}.jsonl")
            # A same-second collision only happens on rapid restarts; keep
            # both by suffixing rather than clobbering the earlier history.
            n = 1
            while backup.exists():
                backup = self.path.with_name(f"audit_{stamp}_{n}.jsonl")
                n += 1
            self.path.rename(backup)
            logger.info(f"[audit] rotated stale audit trail → {backup.name}")
        except Exception as e:
            logger.warning(f"[audit] could not rotate stale audit file: {e}")

    def record(self, entry: str) -> None:
        if self.path is None:
            return
        try:
            row = {"ts": datetime.now(timezone.utc).isoformat(), "entry": entry}
            with self.path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(row, default=str))
                f.write("\n")
        except Exception as e:  # persistence is best-effort, never fatal
            logger.warning(f"[audit] failed to persist entry: {e}")

    @staticmethod
    def read(path: Path | str) -> list[str]:
        """Read back the ``entry`` strings in order (post-flight audit)."""
        p = Path(path)
        out: list[str] = []
        if not p.exists():
            return out
        for line in p.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                out.append(str(json.loads(line).get("entry", "")))
            except (ValueError, AttributeError, TypeError):
                continue
        return out


# ---------------- discovered-vs-truth scoring (SITL audit only) ----------------
#
# In SITL we know the ground-truth target positions (sitl/spawn_targets.py writes
# them to /tmp/aavc_targets.json). The blind-search mission NEVER reads them for
# planning; this compares what the search DISCOVERED against truth purely for the
# post-flight debrief (localisation error, served vs missed). Pure + testable.

_R_EARTH_M = 6_378_137.0


@dataclass(frozen=True)
class TruthComparison:
    """Result of scoring discovered targets against ground truth."""

    lines: list[str]
    matched: int
    served: int
    total_truth: int
    missed: list[str]


def _truth_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = math.radians(lat2 - lat1) * _R_EARTH_M
    de = math.radians(lon2 - lon1) * _R_EARTH_M * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def read_truth_targets(path: Path | str) -> list[dict[str, float | str | int | None]]:
    """Load ground-truth pads (``{name?, marker_id?, lat, lon}`` list, or
    ``{"targets": [...]}``). Returns ``[]`` if the file is missing/unreadable —
    truth is optional."""
    p = Path(path)
    if not p.exists():
        return []
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    items = data.get("targets", data) if isinstance(data, dict) else data
    out: list[dict[str, float | str | int | None]] = []
    for i, t in enumerate(items):
        try:
            mid = t.get("marker_id")
            out.append({"name": str(t.get("name", f"T{i+1}")),
                        "marker_id": int(mid) if mid is not None else None,
                        "lat": float(t["lat"]), "lon": float(t["lon"])})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def compare_with_truth(
    discovered: "list[TrackedTarget]",
    truth: list[dict[str, float | str | int | None]],
    *,
    match_radius_m: float = 10.0,
) -> TruthComparison:
    """Score the registry against ground truth. Identity is the decoded marker
    id when BOTH sides carry one (V1.3: the id IS the assignment — a position
    match under the wrong id is an ID-MISMATCH, not a success); pads without
    ids fall back to nearest-distance matching. ``discovered`` is duck-typed
    (needs ``lat``/``lon``/``target_id``/``state``; ``marker_id`` optional) so
    this stays decoupled from the tracker import."""
    lines: list[str] = []
    matched = served = 0
    ids_total = ids_correct = 0
    missed: list[str] = []

    def _nearest(tlat: float, tlon: float):
        best, best_d = None, math.inf
        for c in discovered:
            if math.isnan(c.lat) or math.isnan(c.lon):
                continue
            d = _truth_dist_m(tlat, tlon, c.lat, c.lon)
            if d < best_d:
                best_d, best = d, c
        return best, best_d

    for tg in truth:
        name = str(tg.get("name", "?"))
        tlat, tlon = float(tg["lat"]), float(tg["lon"])  # type: ignore[arg-type]
        tmid = tg.get("marker_id")
        by_id = None
        if tmid is not None:
            ids_total += 1
            for c in discovered:
                if getattr(c, "marker_id", None) == tmid:
                    by_id = c
                    break
        if by_id is not None and not math.isnan(by_id.lat):
            d = _truth_dist_m(tlat, tlon, by_id.lat, by_id.lon)
            matched += 1
            ids_correct += 1
            is_served = str(getattr(by_id.state, "value", by_id.state)) == "served"
            served += 1 if is_served else 0
            lines.append(f"truth {name} (pad {tmid}): id-matched cluster="
                         f"{by_id.target_id} err={d:.2f}m served={is_served}")
            continue
        best, best_d = _nearest(tlat, tlon)
        if best is not None and best_d <= match_radius_m:
            got_mid = getattr(best, "marker_id", None)
            if tmid is not None and got_mid is not None and got_mid != tmid:
                # Position says this cluster IS the pad, but it decoded another
                # id — a delivery keyed on it would go to the WRONG pad.
                missed.append(name)
                lines.append(f"truth {name} (pad {tmid}): ID-MISMATCH — nearest "
                             f"cluster {best.target_id} decoded {got_mid} "
                             f"at {best_d:.2f}m")
                continue
            matched += 1
            is_served = str(getattr(best.state, "value", best.state)) == "served"
            served += 1 if is_served else 0
            lines.append(f"truth {name}: matched id={best.target_id} "
                         f"err={best_d:.2f}m served={is_served}")
        else:
            missed.append(name)
            near = f"nearest {best_d:.1f}m" if best is not None else "no detections"
            lines.append(f"truth {name}{f' (pad {tmid})' if tmid else ''}: "
                         f"MISSED ({near})")
    lines.append(f"truth audit: matched {matched}/{len(truth)}, "
                 f"ids correct {ids_correct}/{ids_total}, "
                 f"served {served}, missed {len(missed)}")
    return TruthComparison(lines=lines, matched=matched, served=served,
                           total_truth=len(truth), missed=missed)
