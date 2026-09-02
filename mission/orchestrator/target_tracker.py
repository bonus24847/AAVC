"""Cross-sortie landing-pad registry — confirm pads from streaming camera fixes.

The blind search has no pad list — it discovers landing pads visually while it
sweeps, and the committee only names the ASSIGNED marker id per sortie. This
tracker turns the :class:`TargetFix` stream from
:class:`~orchestrator.vision_worker.VisionWorker` into a registry of CONFIRMED
pad positions keyed by decoded ArUco id. The registry lives for the whole
operation window (single long-running process): a pad decoded while serving
sortie 1 becomes sortie 3's direct goto.

Identity model (V1.3):
  * A fix with a decoded ``marker_id`` carries EXPLICIT identity — it merges
    into the cluster holding that id at any distance (field ids are unique),
    and NEVER into a cluster decoded as a different id.
  * A ``marker_id=None`` fix (white-pad cue, marker not yet decodable) merges
    spatially, exactly like the legacy tracker; such *unidentified candidates*
    are what the mission revisits at decode altitude. The first decoded fix
    landing on one upgrades ("identifies") it in place.

How a cluster earns CONFIRMED (CLAUDE.md §9, papers B + C):
  * **Gate** each fix: nadir-camera-only (the sole control authority), detector
    confidence, a SIZE prior (marker-equivalent radius vs the known 0.4 m
    marker at the fix's slant range), max ground distance (kills near-horizon
    junk from a tilted frame edge).
  * **Confirm** on temporal consistency — >= k NADIR votes spanning >= a short
    time; identified clusters count only DECODED nadir votes (the id must be
    read k times, not guessed).
  * **Fuse** the reported position as the component-wise MEDIAN of the
    cluster's nadir fixes (robust to a single outlier).

Threading: ``ingest`` runs on the VisionWorker's worker THREAD; the mission loop
reads/claims on the asyncio loop thread. A single lock guards the cluster list;
every method is short and sync. Reads return copies. ``ingest`` never touches
``state`` (its audit sink isn't thread-safe) — events are queued and the loop
drains them with :meth:`drain_events`.
"""

from __future__ import annotations

import math
import statistics
import threading
from dataclasses import dataclass, field
from enum import Enum

from vision.projection import NADIR, expected_radius_px

from .vision_worker import TargetFix

_R_EARTH_M = 6_378_137.0


class TargetState(str, Enum):
    CANDIDATE = "candidate"    # seen, not yet enough consistent nadir votes
    CONFIRMED = "confirmed"    # ready to serve (land-and-drop)
    SERVING = "serving"        # the mission loop has claimed it
    SERVED = "served"          # a payload was dropped
    FAILED = "failed"          # gave up after retries (or a suppressed duplicate)


@dataclass(frozen=True)
class TrackedTarget:
    """Immutable snapshot of one cluster handed to the mission loop."""

    target_id: int
    lat: float
    lon: float
    votes_nadir: int
    best_confidence: float
    first_t: float
    last_t: float
    state: TargetState
    attempts: int
    marker_id: int | None = None   # decoded ArUco id; None = unidentified pad


@dataclass
class _Cluster:
    target_id: int
    marker_id: int | None = None   # adopted from the first decoded nadir fix
    id_votes: int = 0              # NADIR fixes that DECODED this cluster's id
    nadir_pts: list[tuple[float, float]] = field(default_factory=list)
    best_confidence: float = 0.0
    first_t: float = math.inf      # min over fixes; inf until the first lands
    last_t: float = -math.inf      # max over fixes
    state: TargetState = TargetState.CANDIDATE
    attempts: int = 0

    def centre(self) -> tuple[float, float] | None:
        """Fused position: median of the nadir fixes (paper C)."""
        if self.nadir_pts:
            return (statistics.median(p[0] for p in self.nadir_pts),
                    statistics.median(p[1] for p in self.nadir_pts))
        return None

    def view(self) -> TrackedTarget:
        c = self.centre() or (float("nan"), float("nan"))
        return TrackedTarget(
            target_id=self.target_id, lat=c[0], lon=c[1],
            votes_nadir=len(self.nadir_pts),
            best_confidence=self.best_confidence,
            first_t=self.first_t if math.isfinite(self.first_t) else 0.0,
            last_t=self.last_t if math.isfinite(self.last_t) else 0.0,
            state=self.state, attempts=self.attempts,
            marker_id=self.marker_id,
        )


def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = math.radians(lat2 - lat1) * _R_EARTH_M
    de = math.radians(lon2 - lon1) * _R_EARTH_M * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


_FINISHED = (TargetState.SERVED, TargetState.FAILED)


class TargetTracker:
    """Thread-safe confirmation of discovered targets (see module docstring)."""

    def __init__(
        self,
        *,
        cluster_radius_m: float = 8.0,
        confirm_votes: int = 3,
        min_confidence: float = 0.5,
        min_span_s: float = 0.6,
        target_radius_m: float = 0.2,
        radius_band: tuple[float, float] = (0.4, 2.5),
        max_fix_ground_dist_m: float = 50.0,
        max_candidates: int = 12,
        serve_dedupe_m: float = 12.0,
    ) -> None:
        self.cluster_radius_m = cluster_radius_m
        self.confirm_votes = confirm_votes
        self.min_confidence = min_confidence
        self.min_span_s = min_span_s
        self.target_radius_m = target_radius_m
        self.radius_band = radius_band
        self.max_fix_ground_dist_m = max_fix_ground_dist_m
        self.max_candidates = max_candidates
        self.serve_dedupe_m = serve_dedupe_m
        self._clusters: list[_Cluster] = []
        self._events: list[str] = []
        self._next_id = 0
        self._lock = threading.Lock()

    # ---------- ingest (worker thread) ----------

    def ingest(self, fix: TargetFix) -> None:
        """Add one nadir camera fix. Gated, clustered, and possibly confirmed.
        Safe to call from the VisionWorker thread.

        The nadir camera is the SOLE control authority (single-camera rig): a
        fix labelled with any other camera is ignored outright, so nothing but
        nadir geometry can ever seed, vote, or steer the registry."""
        if fix.camera != "nadir":
            return
        if not self._gate(fix):
            return
        mid = getattr(fix, "marker_id", None)
        with self._lock:
            cl = self._match(fix, mid)
            if cl is None:                      # near a finished target / at cap
                return
            if mid is not None and cl.marker_id is None:
                cl.marker_id = mid              # a cue-only pad gains its identity
                self._events.append(
                    f"cluster_identified id={cl.target_id} marker={mid}")
            cl.nadir_pts.append((fix.lat, fix.lon))
            if mid is not None and mid == cl.marker_id:
                cl.id_votes += 1
            cl.best_confidence = max(cl.best_confidence, fix.confidence)
            cl.first_t = min(cl.first_t, fix.t_monotonic)
            cl.last_t = max(cl.last_t, fix.t_monotonic)
            if cl.state is TargetState.CANDIDATE and self._confirmable(cl):
                cl.state = TargetState.CONFIRMED
                c = cl.centre() or (float("nan"), float("nan"))
                self._events.append(
                    f"target_confirmed id={cl.target_id} marker={cl.marker_id} "
                    f"({c[0]:.7f},{c[1]:.7f}) votes={len(cl.nadir_pts)} "
                    f"id_votes={cl.id_votes} conf={cl.best_confidence:.2f}")

    def _gate(self, fix: TargetFix) -> bool:
        if fix.confidence < self.min_confidence:
            return False
        if fix.ground_dist_m > self.max_fix_ground_dist_m:
            return False
        exp = expected_radius_px(NADIR, self.target_radius_m, fix.slant_range_m)
        if exp > 0.0:
            ratio = fix.radius_px / exp
            lo, hi = self.radius_band
            if not (lo <= ratio <= hi):
                return False
        return True

    def _confirmable(self, cl: _Cluster) -> bool:
        # An IDENTIFIED cluster confirms only on decoded votes — the marker id
        # must actually be READ k times. An unidentified (cue-only) cluster
        # keeps the legacy positional rule; it is surfaced only through
        # unidentified_candidates(), never as a servable identity.
        votes = cl.id_votes if cl.marker_id is not None else len(cl.nadir_pts)
        return (votes >= self.confirm_votes
                and (cl.last_t - cl.first_t) >= self.min_span_s)

    def _match(self, fix: TargetFix, mid: int | None) -> _Cluster | None:
        """The cluster this fix belongs to.

        Decoded fixes match their id's cluster at ANY distance (field ids are
        unique) and never a cluster decoded differently; undecoded fixes match
        the nearest id-compatible cluster within ``cluster_radius_m``. Returns
        None when the fix falls on an already finished target, or a new
        candidate can't be made (capacity, with nothing prunable)."""
        if mid is not None:
            for cl in self._clusters:
                if cl.marker_id == mid:
                    return None if cl.state in _FINISHED else cl
        best: _Cluster | None = None
        best_d = self.cluster_radius_m
        for cl in self._clusters:
            if mid is not None and cl.marker_id not in (None, mid):
                continue                    # decoded differently — never merge
            c = cl.centre()
            if c is None:
                continue
            d = _dist_m(fix.lat, fix.lon, c[0], c[1])
            if d <= best_d:
                best_d, best = d, cl
        if best is not None:
            return None if best.state in _FINISHED else best
        return self._new_cluster()

    def _new_cluster(self) -> _Cluster | None:
        active = [c for c in self._clusters if c.state not in _FINISHED]
        if len(active) >= self.max_candidates:
            # Make room by dropping the weakest CANDIDATE (fewest votes); refuse
            # if everything is confirmed/serving (don't evict committed work).
            cands = [c for c in active if c.state is TargetState.CANDIDATE]
            if not cands:
                return None
            weakest = min(cands, key=lambda c: (len(c.nadir_pts), c.last_t))
            self._clusters.remove(weakest)
        cl = _Cluster(target_id=self._next_id)
        self._next_id += 1
        self._clusters.append(cl)
        return cl

    # ---------- read / claim (loop thread) ----------

    def confirmed_pending(self) -> list[TrackedTarget]:
        """Copies of every CONFIRMED (unclaimed) target, for the mission loop."""
        with self._lock:
            return [c.view() for c in self._clusters if c.state is TargetState.CONFIRMED]

    def confirmed_by_marker(self, marker_id: int) -> TrackedTarget | None:
        """The registry entry for a decoded pad id, if it ever confirmed.

        SERVED entries are included — the committee may re-assign a pad that
        was already delivered to (the mission re-claims it), and a later sortie
        needs the stored position either way. FAILED entries are not."""
        with self._lock:
            for cl in self._clusters:
                if (cl.marker_id == marker_id
                        and cl.state is not TargetState.FAILED
                        and cl.state is not TargetState.CANDIDATE
                        and cl.centre() is not None):
                    return cl.view()
            return None

    def distinct_confirmed_ids(self) -> set[int]:
        """Every decoded pad id that has reached CONFIRMED (or beyond). The
        sweep early-stops when this covers the field's max_pads."""
        with self._lock:
            return {cl.marker_id for cl in self._clusters
                    if cl.marker_id is not None
                    and cl.state not in (TargetState.CANDIDATE, TargetState.FAILED)}

    def unidentified_candidates(self, min_votes: int = 2) -> list[TrackedTarget]:
        """Active cue-only clusters (marker never decoded) worth revisiting at
        decode altitude, strongest first."""
        with self._lock:
            out = [cl.view() for cl in self._clusters
                   if cl.marker_id is None
                   and cl.state in (TargetState.CANDIDATE, TargetState.CONFIRMED)
                   and len(cl.nadir_pts) >= min_votes
                   and cl.centre() is not None]
            out.sort(key=lambda t: (-t.votes_nadir, -t.best_confidence))
            return out

    def identified_unconfirmed(self, marker_id: int | None = None) -> list[TrackedTarget]:
        """Active identified clusters (marker decoded at least once) still short
        of the confirm-vote threshold — the cheap top-up targets: their position
        is already known, so a short decode visit that reads the id again
        confirms them without the full re-sweep an unregistered assignment
        costs. ``marker_id`` filters to one id (the sortie's assignment)."""
        with self._lock:
            out = [cl.view() for cl in self._clusters
                   if cl.marker_id is not None
                   and (marker_id is None or cl.marker_id == marker_id)
                   and cl.state is TargetState.CANDIDATE
                   and cl.centre() is not None]
            out.sort(key=lambda t: (-t.votes_nadir, -t.best_confidence))
            return out

    def claim_by_marker(self, marker_id: int) -> TrackedTarget | None:
        """Move the pad with this decoded id to SERVING (attempts += 1).

        Unlike :meth:`claim` it accepts a SERVED entry (re-assignment) and
        needs no positional dedupe — the identity is explicit."""
        with self._lock:
            for cl in self._clusters:
                if cl.marker_id != marker_id:
                    continue
                if cl.state not in (TargetState.CONFIRMED, TargetState.SERVED):
                    return None
                cl.state = TargetState.SERVING
                cl.attempts += 1
                self._events.append(
                    f"pad_claimed marker={marker_id} id={cl.target_id} "
                    f"attempt={cl.attempts}")
                return cl.view()
            return None

    def claim(self, target_id: int) -> TrackedTarget | None:
        """Move a CONFIRMED target to SERVING (attempts += 1) and return it.

        Returns None if it isn't claimable, or if its position duplicates an
        already SERVED target (within ``serve_dedupe_m``) — in which case it is
        suppressed so the same body isn't dropped on twice."""
        with self._lock:
            cl = self._by_id(target_id)
            if cl is None or cl.state is not TargetState.CONFIRMED:
                return None
            c = cl.centre()
            if c is not None:
                for other in self._clusters:
                    if other.state is not TargetState.SERVED:
                        continue
                    oc = other.centre()
                    if oc is not None and _dist_m(c[0], c[1], oc[0], oc[1]) <= self.serve_dedupe_m:
                        cl.state = TargetState.FAILED
                        self._events.append(
                            f"duplicate_cluster_suppressed id={cl.target_id} "
                            f"near served={other.target_id}")
                        return None
            cl.state = TargetState.SERVING
            cl.attempts += 1
            return cl.view()

    def defer(self, target_id: int) -> None:
        """Return a SERVING target to CONFIRMED for a later attempt."""
        with self._lock:
            cl = self._by_id(target_id)
            if cl is not None and cl.state is TargetState.SERVING:
                cl.state = TargetState.CONFIRMED
                self._events.append(f"target_deferred id={cl.target_id} attempts={cl.attempts}")

    def mark_served(self, target_id: int) -> None:
        with self._lock:
            cl = self._by_id(target_id)
            if cl is not None:
                cl.state = TargetState.SERVED
                self._events.append(f"target_served id={cl.target_id}")

    def mark_failed(self, target_id: int, reason: str = "") -> None:
        with self._lock:
            cl = self._by_id(target_id)
            if cl is not None:
                cl.state = TargetState.FAILED
                self._events.append(f"target_failed id={cl.target_id} {reason}".rstrip())

    def drain_events(self) -> list[str]:
        """Pop the queued audit strings (the mission loop records them)."""
        with self._lock:
            out, self._events = self._events, []
            return out

    def snapshot(self) -> list[TrackedTarget]:
        """Copies of ALL clusters (any state) — used for the truth audit."""
        with self._lock:
            return [c.view() for c in self._clusters]

    def _by_id(self, target_id: int) -> _Cluster | None:
        for cl in self._clusters:
            if cl.target_id == target_id:
                return cl
        return None
