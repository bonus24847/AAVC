"""Vision-guided precision landing ON the assigned ArUco landing pad.

This is the close-in controller the mission loop hands each delivery to. It
owns a tight detect→project→reposition loop (the GPS-coarse, no-RTK approach
is refined by the nadir camera) and a **descend gate**: the drone steps down
only while it is confidently centred, and backs off a rung if it loses the
lock. That gate is the "fast but safe" trade — aggressive descent when sure,
caution when not.

The target is the 1×1 m landing pad whose ArUco id matches THIS sortie's
committee assignment (AAVC 2026 V1.3). Scoring pays for landing ON the correct
pad and releasing the egg cargo only after touchdown, so two identity guards
run through the descent:

  * a hit decoded as a DIFFERENT id is rejected outright (never merged);
  * the **id-verified LAND gate** — the assigned id must have been decoded at
    least ``require_id_votes`` times during ACQUIRE/ALIGN, or the routine
    climbs back to the top rung and defers instead of landing. Landing on the
    wrong pad (or off-pad on coarse GPS) wastes the sortie's only egg, so the
    old "an attempt beats no drop" doctrine is deliberately reversed
    (``gps_fallback=False``).

Per-sortie terminal sequence (land-ON-and-release):

    ACQUIRE → ALIGN ↘ (descend rungs over the pad, re-centring at each)
            → LAND on the pad (stay ARMED) → RELEASE after touchdown
            → (caller climbs out and flies the egress transit)

The loop reads the nadir frame inline (``find_landing_pads``) for low latency;
the background :class:`VisionWorker` is a separate monitoring feed.

All altitudes are clamped to the profile ceiling (20 m) by the mission layer;
this module never commands above ``rungs[0]``. The descent below the 10 m
search floor happens only here, over the pad — exactly the rules' carve-out.
"""

from __future__ import annotations

import asyncio
import math
import statistics
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from loguru import logger

from mavlink_adapter.commands import DroneCommander
from mission_brain.schemas import Coordinate, MissionPhase
from vision.detectors.aruco import PadHit, find_landing_pads
from vision.projection import (
    NADIR,
    GroundFix,
    expected_radius_px,
    ground_sample_distance_m_per_px,
    project_pixel,
)

from . import drop_trajectory
from .state import OrchestratorState, TerminalState
from .vision_worker import DEFAULT_NADIR_FRAME, frame_too_old

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]


@dataclass(frozen=True)
class AlignParams:
    """Tunables for the approach. Defaults suit the SITL field + the 1×1 m
    V1.3 landing pad; the field team adjusts after the first live runs."""

    # Descend rungs (m AGL), high → low. First is the cruise/approach altitude;
    # the last is the final hover before LAND. All ≤ profile ceiling (20 m).
    # Only this descent may sink below the 10 m search floor (rules carve-out).
    rungs: tuple[float, ...] = (12.0, 8.0, 5.0, 3.0, 2.0, 1.5)
    # Horizontal lock tolerance (m) required to descend FROM each rung. Tighter
    # as we get lower (the footprint shrinks so the same px error is fewer m).
    # The final 0.35 m is the land-ON precision driver: touchdown must put the
    # gear on a 1 m pad, so centre within ~a third of a metre before LAND.
    rung_tol_m: tuple[float, ...] = (1.5, 1.0, 0.6, 0.35, 0.25, 0.2)
    # Per-rung DESCENT speed cap (m/s), paralleling ``rungs`` high → low: descend
    # FAST up high, ease off near the ground (operator request). Applied by setting
    # MPC_Z_VEL_MAX_DN before descending INTO each rung; index 0 also caps the
    # acquire descent to the cruise rung. The FINAL touchdown is PX4's MPC_LAND_SPEED
    # crawl, not this. Indexed defensively (min(i, len-1)) so a shorter ``rungs``
    # (low ceiling) still maps cleanly.
    rung_descent_mps: tuple[float, ...] = (3.0, 3.0, 1.5, 0.8, 0.5, 0.4)
    lock_cycles: int = 3              # consecutive in-tolerance cycles to advance
    cycle_hz: float = 5.0             # detect→reposition loop rate
    acquire_timeout_s: float = 12.0   # search budget before deferring
    rung_timeout_s: float = 18.0      # per-rung align budget
    min_confidence: float = 0.45      # pad-hit acceptance
    max_lost_cycles: int = 8          # lost detections before climbing a rung
    search_radius_m: float = 4.0      # expanding-box search step if not acquired
    settle_after_land_s: float = 2.0  # pause after touchdown before the release
    #                                   opens (gentle for the egg; also lets the
    #                                   airframe stop rocking on the pad)
    # "Touched down" when the RELATIVE altitude reads below this. Generous on
    # purpose: the EKF/home altitude frame drifts up to ~±0.7 m across a flight
    # (SITL G4 measured a pad touchdown reading 1.36 m), and a too-tight
    # threshold left the vehicle SITTING on the pad for the full 40 s wait.
    # PX4's LAND has completed by then; the physical settle is the 2 s pause.
    land_alt_threshold_m: float = 1.5
    # Target-IDENTITY gate: reject a detected pad whose projected world position
    # is farther than this from the commanded target. Pads are ≥25 m apart, so
    # 15 m cleanly separates neighbours while absorbing coarse no-RTK GPS.
    accept_radius_m: float = 15.0
    # Anti-overshoot: suppress a goto re-issue when the new setpoint is within an
    # ALTITUDE-SCALED deadband of the last one at the same rung. The deadband is
    # ``reissue_px`` pixels of ground (max(0.3, reissue_px·GSD(alt))) so it tracks
    # detector noise (a few px) instead of a flat metre value — the old flat 0.4 m
    # exceeded the final rung's 0.35 m tolerance and could deadlock the descent.
    reissue_px: float = 4.0
    # Median-fuse the last N accepted world fixes before commanding (paper C):
    # kills single-frame projection outliers at ~0.2-0.4 s of added latency.
    median_window: int = 3
    # Tilt gate (deg): skip a cycle while |roll| or |pitch| exceeds this. A
    # near-hover rung shouldn't hold large tilt; a mid-correction transient is
    # where the projection is least trustworthy. Skipped cycles do NOT count as
    # lost (else a multi-cycle correction would spuriously trip the climb-back).
    tilt_gate_deg: float = 10.0
    # Size prior (paper B): reject a hit whose marker-equivalent radius (px) is
    # outside this band times the size expected for a target_radius_m object at
    # the fix's slant range — geometry a real pad can't produce (a far blob, a
    # near artefact). target_radius_m = half the 0.4 m marker side; PadHit
    # reports marker-equivalent radius_px for blob hits too, so ONE prior fits.
    target_radius_m: float = 0.2
    radius_band: tuple[float, float] = (0.4, 2.5)
    # THIS sortie's committee-assigned ArUco id. None accepts any decoded pad
    # (bench/tuning use); the mission always sets it.
    assigned_marker_id: int | None = None
    # Id-verified LAND gate: the assigned id must have been DECODED at least
    # this many times during ACQUIRE/ALIGN before LAND is allowed — otherwise
    # climb back to the top rung and defer. Positional (cue-only) centring can
    # steer the descent, but it must never commit the egg to an unread pad.
    require_id_votes: int = 1
    # When ACQUIRE times out: False (default) → return without landing so the
    # mission can defer/re-try — landing on coarse GPS wastes the sortie's only
    # egg and scores nothing (V1.3 reverses "an attempt beats no drop").
    gps_fallback: bool = False
    # In-flight frame-staleness gate (S2): reject a nadir frame older than this
    # (a dead camera writer freezes the last frame — see vision_worker). A stale
    # frame reads as NO detection so the loss/climb-back machinery kicks in and
    # the id-vote gate can't be fooled by a frozen pad. 0 disables (bench/tuning).
    frame_max_age_s: float = 2.0


@dataclass
class AlignResult:
    acquired: bool = False
    aligned: bool = False
    landed: bool = False
    dropped: bool = False
    final_error_m: float = float("nan")
    notes: list[str] = field(default_factory=list)


# Optional hooks the mission layer can inject (dashboard, audit) — all no-ops by
# default so this module has zero hard dependency on the dashboard.
PhaseCb = Callable[[MissionPhase], None]
DropPredCb = Callable[[drop_trajectory.DropPrediction], None]


def _hit_world_fix(hit: PadHit, state: OrchestratorState) -> GroundFix | None:
    """Project a nadir pad centroid to a full ground fix, composing the
    drone's roll/pitch so a tilted correction doesn't shift the projected point."""
    t = state.telemetry
    if math.isnan(t.lat) or math.isnan(t.lon):
        return None
    alt = t.relative_alt_m if not math.isnan(t.relative_alt_m) else 0.0
    yaw = t.heading_deg if not math.isnan(t.heading_deg) else 0.0
    roll = t.roll_deg if not math.isnan(t.roll_deg) else 0.0
    pitch = t.pitch_deg if not math.isnan(t.pitch_deg) else 0.0
    return project_pixel((hit.cx, hit.cy), t.lat, t.lon, alt, yaw, NADIR,
                         roll_deg=roll, pitch_deg=pitch)


def _radius_ok(hit: PadHit, gf: GroundFix, params: AlignParams) -> bool:
    """Size prior: the hit's marker-equivalent radius (px) must be near what a
    target_radius_m object would subtend at this fix's slant range (paper B)."""
    exp = expected_radius_px(NADIR, params.target_radius_m, gf.slant_range_m)
    if exp <= 0.0:
        return True
    ratio = hit.radius_px / exp
    lo, hi = params.radius_band
    return lo <= ratio <= hi


def _detect_nadir(frame_path: Path, min_conf: float,
                  assigned_id: int | None) -> PadHit | None:
    """Best acceptable pad hit in the nadir frame for THIS sortie.

    A hit decoded as the assigned id always wins; an undecoded (cue-only) hit
    is acceptable as a positional candidate; a hit decoded as a DIFFERENT id
    is rejected outright — the descent must never be steered by a wrong pad."""
    if cv2 is None or not frame_path.exists():
        return None
    img = cv2.imread(str(frame_path))
    if img is None:
        return None
    best: PadHit | None = None
    for hit in find_landing_pads(img):          # decoded-first, conf-sorted
        if hit.confidence < min_conf:
            continue
        if (assigned_id is not None and hit.marker_id is not None
                and hit.marker_id != assigned_id):
            continue                            # wrong pad — never steer by it
        if hit.marker_id is not None:
            return hit                          # assigned (or any, if None) id
        if best is None:
            best = hit                          # strongest cue-only fallback
    return best


def _running(state: OrchestratorState) -> bool:
    return state.terminal == TerminalState.RUNNING


async def acquire_and_land_drop(
    commander: DroneCommander,
    state: OrchestratorState,
    target: Coordinate,
    stop_index: int,
    *,
    payload_id: int = 0,
    delivery_index: int = 0,
    nadir_frame: Path = DEFAULT_NADIR_FRAME,
    params: AlignParams = AlignParams(),
    on_phase: PhaseCb | None = None,
    on_drop_prediction: DropPredCb | None = None,
) -> AlignResult:
    """Fly the full vision-guided land-ON-pad + release for ONE sortie.

    Returns an :class:`AlignResult`. Honours the safety watchdog: if
    ``state.terminal`` leaves RUNNING (RTH/abort) the routine returns early so
    the watchdog's RTH is not fought.
    """
    res = AlignResult()
    dt = 1.0 / max(params.cycle_hz, 1.0)
    cruise_alt = params.rungs[0]

    def _phase(p: MissionPhase) -> None:
        state.phase = p
        if on_phase is not None:
            try:
                on_phase(p)
            except Exception:
                logger.exception("[align] on_phase hook raised")

    last_goto: tuple[float, float, float] | None = None

    async def _goto(lat: float, lon: float, alt: float) -> None:
        """Reposition, suppressing a near-duplicate re-issue at the same rung so
        the vehicle settles instead of chasing a jittering setpoint. The deadband
        is altitude-scaled (``reissue_px`` px of ground) — a fixed metre value is
        either too tight up high or wider than the final rung's lock tolerance."""
        nonlocal last_goto
        if last_goto is not None:
            deadband = max(0.3, params.reissue_px
                           * ground_sample_distance_m_per_px(NADIR, alt))
            d = _latlon_dist_m(lat, lon, last_goto[0], last_goto[1])
            if d < deadband and abs(alt - last_goto[2]) < 0.3:
                return
        await commander.goto(lat, lon, alt)
        last_goto = (lat, lon, alt)

    async def _set_descent_cap(mps: float) -> None:
        """Cap the AUTO vertical descent speed (PX4 ``MPC_Z_VEL_MAX_DN``) so the
        rung descent is fast up high and gentle near the ground. Non-fatal: a
        failed param set just leaves the previous cap in place."""
        try:
            await commander.set_param_float("MPC_Z_VEL_MAX_DN", float(mps))
        except Exception:
            logger.debug("[align] MPC_Z_VEL_MAX_DN set failed (non-fatal)")

    def _on_target(gf: GroundFix) -> bool:
        """True if a projected pad fix is near the COMMANDED target (identity gate)."""
        return _latlon_dist_m(gf.lat, gf.lon, target.lat, target.lon) <= params.accept_radius_m

    # Decoded sightings of THIS sortie's assigned id — the LAND gate's evidence.
    id_seen = 0

    def _accept(hit: PadHit | None) -> GroundFix | None:
        """A hit that projects, sits on the commanded target, AND is the right
        apparent size — else None (treated as a lost detection). Counts decoded
        assigned-id sightings for the LAND gate."""
        nonlocal id_seen
        if hit is None:
            return None
        gf = _hit_world_fix(hit, state)
        if gf is None or not _on_target(gf) or not _radius_ok(hit, gf, params):
            return None
        if (hit.marker_id is not None
                and (params.assigned_marker_id is None
                     or hit.marker_id == params.assigned_marker_id)):
            id_seen += 1
        return gf

    def _too_tilted() -> bool:
        roll, pitch = state.telemetry.roll_deg, state.telemetry.pitch_deg
        roll = 0.0 if math.isnan(roll) else roll
        pitch = 0.0 if math.isnan(pitch) else pitch
        return abs(roll) > params.tilt_gate_deg or abs(pitch) > params.tilt_gate_deg

    async def _read_nadir() -> PadHit | None:
        """Detect the pad in the nadir frame, but reject a frozen frame from a
        dead camera writer first (S2). A stale frame returns None so the loop's
        lost-detection/climb-back path runs — the id-vote LAND gate must never be
        satisfied by a frame the camera stopped refreshing."""
        if frame_too_old(nadir_frame, params.frame_max_age_s):
            state.record_anomaly("nadir_frame_stale")
            return None
        return await asyncio.to_thread(
            _detect_nadir, nadir_frame, params.min_confidence, params.assigned_marker_id)

    # Median-fused world fixes (component-wise) smooth the commanded setpoint.
    fix_window: deque[tuple[float, float]] = deque(maxlen=max(1, params.median_window))

    def _commanded_latlon(gf: GroundFix) -> tuple[float, float]:
        fix_window.append((gf.lat, gf.lon))
        return (statistics.median(p[0] for p in fix_window),
                statistics.median(p[1] for p in fix_window))

    # ── 1. ACQUIRE ───────────────────────────────────────────────────────────
    _phase(MissionPhase.SEARCH)
    logger.info(f"[align] sortie #{stop_index}: acquiring pad "
                f"{params.assigned_marker_id} near "
                f"({target.lat:.7f},{target.lon:.7f}) at {cruise_alt:.0f} m")
    await _set_descent_cap(params.rung_descent_mps[0])   # fast drop to the cruise rung
    await _goto(target.lat, target.lon, cruise_alt)
    best_latlon = (target.lat, target.lon)
    t_start = state.now()
    search_ring = 0
    while _running(state) and (state.now() - t_start) < params.acquire_timeout_s:
        hit = await _read_nadir()
        # Accept ONLY a pad that projects near the commanded target AND is the
        # right apparent size — a pad seen while still over a neighbour (or a
        # wrong-size blob) is rejected so the loop keeps flying to THIS one.
        fix = _accept(hit)
        if fix is not None:
            best_latlon = (fix.lat, fix.lon)
            res.acquired = True
            res.notes.append(f"acquired conf={hit.confidence if hit else 0.0}")
            break
        # Expanding-box search around the coarse GPS while not yet acquired.
        search_ring += 1
        off = params.search_radius_m * ((search_ring % 4) // 2 or 1)
        ang = math.radians(90.0 * (search_ring % 4))
        slat, slon = _offset_latlon(target.lat, target.lon,
                                    off * math.cos(ang), off * math.sin(ang))
        await _goto(slat, slon, cruise_alt)
        await asyncio.sleep(dt)
    if not res.acquired:
        if not params.gps_fallback:
            # Defer instead of landing on coarse GPS: an unread pad + a blind
            # landing would spend the sortie's only egg for zero score.
            logger.warning(f"[align] sortie #{stop_index}: not acquired — deferring "
                           "(no coarse-GPS fallback)")
            res.notes.append("acquire-timeout: deferred")
            return res
        logger.warning(f"[align] sortie #{stop_index}: not acquired — "
                       "falling back to coarse GPS land")
        res.notes.append("acquire-timeout: GPS fallback")

    # ── 2. ALIGN + DESCEND GATE ──────────────────────────────────────────────
    _phase(MissionPhase.LOCALIZE)
    last_err = float("nan")
    final_locked = False
    for rung_i, rung_alt in enumerate(params.rungs):
        tol = params.rung_tol_m[min(rung_i, len(params.rung_tol_m) - 1)]
        # Descend INTO this rung at its scheduled speed (fast high → slow low).
        await _set_descent_cap(
            params.rung_descent_mps[min(rung_i, len(params.rung_descent_mps) - 1)])
        in_tol = 0
        lost = 0
        t_rung = state.now()
        while _running(state) and (state.now() - t_rung) < params.rung_timeout_s:
            # Tilt gate: while the quad is banked past the gate the projection is
            # least trustworthy — wait it out WITHOUT counting it as a lost
            # detection (else a multi-cycle correction trips the climb-back).
            if _too_tilted():
                await asyncio.sleep(dt)
                continue
            hit = await _read_nadir()
            fix = _accept(hit)
            if fix is not None:
                last_err = fix.ground_dist_m
                lost = 0
                mlat, mlon = _commanded_latlon(fix)
                best_latlon = (mlat, mlon)
                await _goto(mlat, mlon, rung_alt)
                in_tol = in_tol + 1 if last_err <= tol else 0
                if in_tol >= params.lock_cycles:
                    break        # locked at this rung → descend to next
                await asyncio.sleep(dt)
                continue
            # lost / unprojectable / off-target / wrong-size
            lost += 1
            if lost >= params.max_lost_cycles and rung_i > 0:
                # back off one rung to re-acquire (the descend gate's safety arm)
                climb = params.rungs[rung_i - 1]
                logger.info(f"[align] #{stop_index}: lost lock at {rung_alt:.0f} m "
                            f"→ climb to {climb:.0f} m")
                await _goto(best_latlon[0], best_latlon[1], climb)
                res.notes.append(f"lost@{rung_alt:.0f}m→climb")
                lost = 0
            else:
                await _goto(best_latlon[0], best_latlon[1], rung_alt)
            await asyncio.sleep(dt)
        final_locked = in_tol >= params.lock_cycles
        logger.info(f"[align] #{stop_index}: rung {rung_alt:.0f} m err="
                    f"{last_err:.2f} m locked={final_locked}")
    res.aligned = res.acquired and not math.isnan(last_err)
    res.final_error_m = last_err
    # Restore the fast descent cap for the climb-out / next target / RTH — the
    # per-rung slow caps above were only for THIS target's near-ground approach.
    await _set_descent_cap(params.rung_descent_mps[0])

    if not _running(state):
        res.notes.append("aborted by watchdog before land")
        return res

    # ── 3. ID-VERIFIED LAND GATE, then LAND ON the pad (stay ARMED — no re-arm) ──
    # The vehicle is hovering centred over the pad at the lowest rung. Before
    # committing the landing (and with it the sortie's only egg), the assigned
    # marker id must have actually been DECODED during this approach — a purely
    # positional descent (cue blobs, coarse GPS) may have centred on the WRONG
    # pad. In that case climb back to the top rung and defer to the mission.
    if params.assigned_marker_id is not None and id_seen < params.require_id_votes:
        logger.warning(
            f"[align] sortie #{stop_index}: pad id {params.assigned_marker_id} was "
            f"never decoded during the approach (seen {id_seen}/"
            f"{params.require_id_votes}) — NOT landing; climbing to defer")
        state.record_anomaly("land_gate_id_not_confirmed")
        res.notes.append("id-not-confirmed → defer")
        await _goto(best_latlon[0], best_latlon[1], params.rungs[0])
        return res

    # Centred-LAND gate (found by the 2026-07-15 GCS run): a rung TIMEOUT used
    # to fall straight through — the vehicle descended and LANDED even when the
    # final rung never locked, releasing 2.46 m off-pad on a biased low-altitude
    # fix streak. Landing off the pad forfeits the "landed on the pad BEFORE
    # releasing" scoring line, so refuse and defer exactly like the id gate —
    # the mission un-ledgers the pad and re-approaches once if time allows
    # (a fresh acquire from the top rung usually breaks the bias streak).
    # gps_fallback=True (bench-only) deliberately allows the blind/uncentred
    # landing, matching its acquire-timeout semantics above.
    if not params.gps_fallback and not final_locked:
        logger.warning(
            f"[align] sortie #{stop_index}: final rung never locked "
            f"(err={last_err:.2f} m > tol {params.rung_tol_m[-1]:.2f} m) — "
            "NOT landing off-centre; climbing to defer")
        state.record_anomaly("land_gate_not_centred")
        res.notes.append(f"not-centred (err={last_err:.2f} m) → defer")
        await _goto(best_latlon[0], best_latlon[1], params.rungs[0])
        return res

    _phase(MissionPhase.LAND)
    logger.info(f"[align] sortie #{stop_index}: final err={last_err:.2f} m "
                f"id_seen={id_seen} → LAND on pad {params.assigned_marker_id}")
    await commander.land(disarm=False)
    landed = await _wait_until(
        state, lambda t: (not math.isnan(t.relative_alt_m)
                          and t.relative_alt_m <= params.land_alt_threshold_m),
        timeout_s=40.0,
    )
    res.landed = landed
    await asyncio.sleep(params.settle_after_land_s)

    # A safety RTH/abort during the landing wait takes priority over the release —
    # don't open the cargo hold while the watchdog is bringing the vehicle home.
    if not _running(state):
        res.notes.append("aborted by watchdog before release")
        logger.warning(f"[align] sortie #{stop_index}: watchdog terminal before "
                       "release — keeping the egg")
        return res

    # ── 4. RELEASE (touchdown-gated, idempotent per sortie) ─────────────────
    # Scoring pays for "landed on the pad BEFORE releasing" and for an intact
    # egg — so the release is gated on a confirmed touchdown. If touchdown did
    # not confirm but telemetry shows the vehicle essentially on the ground
    # (flaky AGL near zero), release with an audit trail; if it reads clearly
    # airborne, KEEP the egg — dropping it from height breaks it for zero score
    # (V1.3 reverses the old "an attempt beats no drop" doctrine).
    _phase(MissionPhase.DROP)
    alt_now = state.telemetry.relative_alt_m
    near_ground = (not math.isnan(alt_now)
                   and alt_now <= params.land_alt_threshold_m + 1.0)
    if not landed:
        if not near_ground:
            logger.critical(
                f"[align] sortie #{stop_index}: touchdown NOT confirmed and "
                f"alt={alt_now:.1f} m reads airborne — KEEPING the egg (audited)")
            state.record_anomaly("release_skipped_touchdown_unconfirmed")
            res.notes.append("release skipped: touchdown unconfirmed")
            return res
        logger.critical(
            f"[align] sortie #{stop_index}: touchdown NOT confirmed "
            f"(alt={alt_now:.1f} m, thr={params.land_alt_threshold_m:.1f} m) but "
            "near-ground — releasing (audited)")
        state.record_anomaly("touchdown_unconfirmed_before_release")
    dropped = await _drop_once(
        commander, state, stop_index,
        payload_id=payload_id, delivery_index=delivery_index,
        marker_id=params.assigned_marker_id,
        on_drop_prediction=on_drop_prediction,
    )
    res.dropped = dropped
    logger.info(f"[align] sortie #{stop_index}: released={dropped} landed={landed} "
                f"err={last_err:.2f} m pad={params.assigned_marker_id}")
    return res


async def _drop_once(
    commander: DroneCommander,
    state: OrchestratorState,
    stop_index: int,
    *,
    payload_id: int = 0,
    delivery_index: int = 0,
    marker_id: int | None = None,
    on_drop_prediction: DropPredCb | None = None,
) -> bool:
    """Release exactly once for ``stop_index`` (guards the shared drop lock).

    ``payload_id`` selects the release mechanism for THIS flight (0..N-1 →
    servo channel drop_servo_channel + payload_id); ``stop_index`` (the id's
    position in the mission queue) keys the idempotence ledger so a retried
    serve can't double-open the same hold. ``delivery_index`` is the 1-based
    delivery number across the mission, for the audit line only."""
    async with state.drop_lock:
        if stop_index in state.dropped_stops:
            return False
        # Ballistic record (≈0 drift since we land first) for the GCS overlay.
        if on_drop_prediction is not None:
            try:
                t = state.telemetry
                pred = drop_trajectory.predict(
                    release_lat=t.lat, release_lon=t.lon,
                    release_alt_agl_m=max(0.0, t.relative_alt_m)
                    if not math.isnan(t.relative_alt_m) else 0.0,
                    vehicle_ground_speed_mps=t.ground_speed_mps
                    if not math.isnan(t.ground_speed_mps) else 0.0,
                    vehicle_heading_deg=t.heading_deg
                    if not math.isnan(t.heading_deg) else 0.0,
                )
                on_drop_prediction(pred)
            except Exception:
                logger.exception("[align] drop prediction failed (non-fatal)")
        await commander.drop_payload(payload_id=payload_id)
        state.dropped_stops.add(stop_index)
        state.record_audit(
            f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} RELEASE "
            f"pad={marker_id} payload={payload_id} "
            f"lat={state.telemetry.lat:.7f} lon={state.telemetry.lon:.7f}")
        return True


async def _wait_until(
    state: OrchestratorState,
    pred: Callable[[Any], bool],
    timeout_s: float,
    poll_s: float = 0.25,
) -> bool:
    t0 = state.now()
    while state.now() - t0 < timeout_s:
        if not _running(state):
            return False
        try:
            if pred(state.telemetry):
                return True
        except (AttributeError, TypeError) as e:
            # The predicate only reads telemetry fields; a missing/non-numeric
            # field is the one expected (transient) error. Anything else is a
            # real bug — let it propagate rather than silently masking it.
            logger.debug(f"[align] _wait_until predicate error (transient): {e}")
        await asyncio.sleep(poll_s)
    return False


def _offset_latlon(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    r = 6_378_137.0
    dlat = math.degrees(north_m / r)
    dlon = math.degrees(east_m / (r * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def _latlon_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth metric distance (m) between two lat/lon — the target-identity gate
    + goto-reissue gate both compare sub-100 m separations on the AAVC field."""
    r = 6_378_137.0
    dn = math.radians(lat2 - lat1) * r
    de = math.radians(lon2 - lon1) * r * math.cos(math.radians(lat1))
    return math.hypot(dn, de)
