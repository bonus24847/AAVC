"""The multi-flight delivery mission loop (AAVC 2026 rules V1.3).

A **FLIGHT** is one arm→disarm cycle; a **DELIVERY** is one pad served within
it. The committee assigns landing-pad marker ids at resupply, and the flight
gate hands the loop the ids for THIS flight — its chunk of the queue, at most
``eggs_aboard`` of them. Everything runs inside a single long-running process
(operator decision 2026-07-03) whose pad registry (the target tracker) and
20-minute window clock persist across flights:

    for flight 1..max_flights:
        per-flight gate (dashboard: operator confirms the eggs + GO; headless:
            --assigned-ids) → the LIST of ids for this flight, or None to end
            [window clock starts at the FIRST GO]
        arm + takeoff → transit P1→P2→P3 at 20 m (mandatory, scored per point)
        for any assigned pad NOT in the registry:
            if a sweep already decoded it once (identified-but-unconfirmed):
                short decode visit to top up its votes (2026-07-08 structural
                fix — far cheaper than a re-sweep); sweep only if that fails
            full boustrophedon sweep of the search area — finish-sweep-then-
            serve (operator 2026-07-03): the sweep runs to completion, feeding
            EVERY decoded pad into the registry (later flights fly direct);
            early-stop only once max_pads distinct ids are confirmed
            → if still undecoded: revisit identified-but-unconfirmed and
              unidentified candidates at the search floor to read their ids
        for each assigned id, in queue order:
            per-delivery abort gate (window reserve + a battery margin above
                the FC's low-battery RTL) — refuse to START a descent the
                budget can't cover, and come home with the remaining eggs
            climb out (a no-op unless the previous delivery left the aircraft
                landed-but-armed on a pad, or a decode visit left it low)
            serve: vision-guided land-ON-pad (id-verified LAND gate) → release
                that delivery's egg after touchdown
        transit P3→P2→P1 → explicit goto L&R → land + DISARM (resupply safety)

``eggs_aboard=1`` yields one delivery per flight — behaviourally identical to
the per-sortie mission this grew out of.

Landings ON the pad stay ARMED (COM_DISARM_LAND=-1, no re-arm mid-flight —
which also pins PX4 home to the launch point); only the L&R landing disarms.
The final approach is an explicit goto + land, NOT RTL: PX4 re-captures home
at every arming, and the vehicle re-arms at L&R each flight.

Watchdog-aware: any safety RTH (``state.terminal`` leaves RUNNING) ends the
loop cleanly without fighting it — every wait re-checks the terminal state.

This deliberately does NOT use PX4 AUTO mission segmentation — a direct
goto/align loop keeps the camera in authority over discovery and the final
metres, and lets the plan be rebuilt live as pads are found.
"""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Awaitable, Callable
from dataclasses import replace

from loguru import logger

from mavlink_adapter.commands import DroneCommander
from mission_brain.flights import chunk_flights
from mission_brain.live_plan import ServedStop, pointer_for, render_live_plan
from mission_brain.profile import MissionProfile, load_profile
from mission_brain.schemas import Coordinate, MissionPhase, MissionPlan
from mission_brain.search_pattern import SearchPlanSpec
from mission_brain.serve_order import order_by_nearest

from .energy_policy import (
    baseline_for_pack,
    detect_battery_swap,
    energy_consumed_mah,
)
from .state import OrchestratorState, TerminalState
from .tactical_align import AlignParams, DropPredCb, acquire_and_land_drop
from .target_tracker import TargetTracker
from .time_policy import TimePolicy

_R_EARTH_M = 6_378_137.0
_ARRIVAL_RADIUS_M = 2.0       # NAV_ACC_RAD (1 m) + 1 m; also the transit-pass radius
                              # (KMUTNB: NAV_ACC_RAD tightened 2 -> 1 for the
                              # short 10-20 m legs — keep these two in step)
_LOOKOUT_POLL_S = 0.25        # 4 Hz fly-leg-with-lookout
_SERVE_ACCEPT_RADIUS_M = 5.0  # registry coords are vision-accurate (<= ~4 m);
                              # KMUTNB: 8 -> 5 so it stays well under the
                              # 14.5 m baseline pad separation (was 25 m)
_TELEM_SAMPLE_S = 1.0         # audit-trail telemetry sample cadence (verify_flight)
_WAIT_PAD_S = 20.0            # slack added to every distance-derived arrival timeout
# Leg abandonment (2026-07-25). `2 x distance / speed + _WAIT_PAD_S` is still
# computed per leg, but it is now the BACKSTOP, not the verdict — see
# _ProgressGuard for why elapsed wall time is the wrong question to ask.
_LEG_STALL_S = 25.0           # no measurable closure for this long => give up
_LEG_PROGRESS_EPS_M = 1.0     # closure that counts as progress (above GPS noise)
_LEG_CEILING_MULT = 8.0       # hard ceiling = this x the distance-derived budget
_ALT_BIAS_MARGIN_M = 0.5      # commanded-vs-EKF altitude-frame bias headroom (ceiling)
# Altitude the L&R descent is flown DOWN TO as a normal position leg before
# AUTO.LAND takes the touchdown. PX4 sinks at MPC_LAND_SPEED (0.3 m/s) for the
# whole of an AUTO.LAND, so handing it the full transit altitude cost ~51 s per
# sortie — 25% of the 20-minute window (12 SITL runs, 2026-07-20). The position
# leg runs at MPC_Z_VEL_MAX_DN instead, leaving LAND only the last few metres.
# NOT fixed by raising MPC_LAND_SPEED: that was tried and reverted (e02ffa3 —
# AUTO.LAND climbed to 41 m after the L&R touchdown).
_LAND_STAGE_ALT_M = 5.0       # AGL hand-over altitude for the L&R touchdown
_LAND_STAGE_TOL_M = 1.0       # altitude-frame wander (±~0.7 m) headroom
_LAND_STAGE_MIN_MPS = 0.5     # pessimistic sink rate used to size the wait
# The AUTO descent speed. PX4 splits this by mode family: MPC_Z_V_AUTO_DN drives
# autonomous descents, MPC_Z_VEL_MAX_DN only manual/offboard
# (FlightTaskManualAccelerationSlow) — the mission flies AUTO end to end, so
# AUTO_DN is the one that moves the aircraft. _PAD_DESCENT_MPS must stay equal
# to the pinned MPC_Z_V_AUTO_DN (tests/test_px4_tuning_parity.py): it is the
# descent the release accuracy was validated at, and it is restored the moment
# the L&R touchdown is done so the next sortie's pad approach is unchanged.
_LAND_STAGE_MPS = 1.5         # L&R staged-descent speed (m/s). KMUTNB: 2.5 ->
                              # 1.5 — transit is 4 m and _LAND_STAGE_ALT_M is
                              # 5, so the staged leg is a near-no-op here; keep
                              # it <= MPC_Z_VEL_MAX_DN (parity test invariant)
_PAD_DESCENT_MPS = 0.4        # validated pad-approach descent (m/s)
# Mid-flight per-delivery battery guard: don't START a new descent if the pack
# is already at/near the FC's low-battery RTL threshold — the FC failsafe would
# otherwise fire mid-delivery with the egg aboard. Percentage, above rth_battery_pct.
# Charge (percentage points ABOVE the FC's own low-battery RTL, profile
# rth_battery_pct) the per-delivery gate reserves before committing to another
# descent. It must cover a WHOLE delivery, because the watchdog's battery RTH
# is NOT exempt in the SEARCH/LOCALIZE phases a delivery spends its first
# ~90 s in (safety.py) — a descent begun on a thinner margin runs into the
# failsafe with the egg still aboard, and the RTH ends the whole mission.
# Derivation: one delivery is ~110 s at the ~35.6 A hover the energy module
# cites for the 8.22 kg X6100 => 35.6 A x 110 s = 1,088 mAh ~= 8 % of the
# 15,000 mAh pack. So 8 points is one delivery's own cost with nothing left
# over: at rth_battery_pct=30 the gate refuses at or below 38 %.
#
# This is a PERCENTAGE of the pack, so it had to be re-derived when the second
# battery went on (2026-07-25): the delivery got ~23 % more expensive in mAh,
# but the pack doubled, so the fraction FELL 12 -> 8. Leaving it at 12 would
# not have been "safely conservative" — it would have refused deliveries the
# pack can comfortably finish. Re-derive it again if either number moves.
_DELIVERY_BATT_MARGIN_PCT = 8.0


class _ProgressGuard:
    """Decide when to abandon a leg by PROGRESS, not by elapsed wall time.

    Every leg is still sized `2 x distance / speed + _WAIT_PAD_S`, but that
    number answers the wrong question. It asks "should the aircraft have
    arrived by now?"; what actually justifies giving up is "has it stopped
    getting closer?". The two only agree while wall time and flight time run
    at the same rate.

    They stop agreeing in SITL, which advances the aircraft in SIMULATED time
    while the mission counts `time.monotonic()`. On 2026-07-25 the host fell
    to 0.20x real time during the P3 -> sweep-wp0 leg (ULog 04_51_49: 443 s of
    sim across 776 s of wall, dipping to 0.20 on that leg). The 96.4 m leg's
    44.1 s budget therefore bought ~9 s of flying against the >=16 s it
    needed, `sweep_leg_timeout_wp0` fired with the aircraft halfway there and
    still closing at a steady 8 m/s, and the sweep jumped to wp1 — a visibly
    skipped waypoint on the GCS map, with nothing wrong with the aircraft.

    The real bird reaches the same failure without any clock trickery: a
    headwind, a heavier pack or a re-planned longer leg all stretch a leg past
    2x nominal while it is flying perfectly well. So the verdict is now the
    stall test, and the distance-derived budget survives only as a hard
    ceiling — a leg that closes just `eps_m` per stall window would otherwise
    run unbounded.
    """

    __slots__ = ("_clock", "_stall_s", "_eps_m", "_deadline", "_best_m",
                 "_t_progress")

    def __init__(self, budget_s: float, *,
                 clock: Callable[[], float] = time.monotonic,
                 stall_s: float = _LEG_STALL_S,
                 eps_m: float = _LEG_PROGRESS_EPS_M,
                 ceiling_mult: float = _LEG_CEILING_MULT) -> None:
        self._clock = clock
        self._stall_s = stall_s
        self._eps_m = eps_m
        now = clock()
        self._deadline = now + max(budget_s, 0.0) * ceiling_mult
        self._best_m = math.inf
        self._t_progress = now

    def give_up(self, dist_m: float | None) -> bool:
        """Feed the current distance to the target (None = no position fix).

        A missing fix is deliberately NOT treated as progress: with no
        evidence the aircraft is closing, the stall clock keeps running.
        """
        now = self._clock()
        if dist_m is not None and dist_m < self._best_m - self._eps_m:
            self._best_m = dist_m
            self._t_progress = now
        return (now - self._t_progress) > self._stall_s or now > self._deadline


PlanUpdateCb = Callable[[MissionPlan, int], None]
# Per-FLIGHT gate: awaits operator GO (or headless auto-GO) and returns the
# committee-assigned marker ids for flight i — its <= eggs_aboard chunk of the
# queue — or None to end the mission. The GATE owns the chunking; the loop just
# serves the list it is handed.
FlightGate = Callable[[int], Awaitable[list[int] | None]]


def _latlon_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = math.radians(lat2 - lat1) * _R_EARTH_M
    de = math.radians(lon2 - lon1) * _R_EARTH_M * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


async def run_delivery_mission(
    commander: DroneCommander,
    state: OrchestratorState,
    tracker: TargetTracker,
    spec: SearchPlanSpec,
    *,
    home: Coordinate,
    transit_route: list[Coordinate],
    sortie_gate: FlightGate,
    profile: MissionProfile | None = None,
    align: AlignParams | None = None,
    policy: TimePolicy | None = None,
    max_pads: int = 6,  # SIX pads in the field; a 4 truncates the sweep at 4/6
    decode_dwell_s: float = 4.0,
    on_phase: Callable[[MissionPhase], None] | None = None,
    on_drop_prediction: DropPredCb | None = None,
    on_plan_update: PlanUpdateCb | None = None,
    refresh_energy: Callable[[], None] | None = None,
) -> None:
    """Fly the full multi-flight delivery mission, mutating ``state.terminal``."""
    # Bind non-optional locals so the nested closures below type-check (a
    # reassigned `X | None` parameter stays optional inside closures).
    # The keyword stays `sortie_gate` (main.py + the tests call it that); what
    # changed is what it RETURNS — the flight's id list, not a single id.
    flight_gate = sortie_gate
    prof: MissionProfile = profile or load_profile()
    align_p: AlignParams = align or AlignParams()
    pol: TimePolicy = policy or TimePolicy(watchdog_floor_s=prof.min_time_remaining_s)
    sweep_alt = min(spec.sweep_alt_m, prof.altitude_ceiling_m)
    # Command the transit half a metre UNDER the strict altitude: the goto
    # AGL→MSL conversion rides on the home MSL cached at connect, and SITL
    # measured a ~+0.6 m bias between that frame and the EKF relative altitude
    # — commanding 20.0 cruised at 20.6, over the ceiling. 19.5 commanded
    # reads 19.5-20.1 actual: "at 20 m" within estimation accuracy, never over.
    transit_alt = min(prof.transit_alt_m, prof.altitude_ceiling_m) - _ALT_BIAS_MARGIN_M
    # Two-stage climb: a full-rate climb straight to the 20 m transit altitude
    # overshoots ~+1.8 m even at MPC_Z_VEL_MAX_UP=2 (measured in SITL) — through
    # the rules ceiling. Take off / climb out to transit_alt - this margin, then
    # let the first transit goto close the last metres gently en-route.
    # KMUTNB (2026-08-12): floor-clamped. At the 5 m profile transit_alt-2 =
    # 2.0 m sits UNDER the 2.5 m search floor, so the phase flips to
    # transit_ingress while still below the floor and verify_flight flags 9
    # below-floor samples on a rules-clean flight (measured, run A). Stage no
    # lower than floor + the same bias margin the decode hover uses; at the
    # KMITL 20 m profile this clamp is inert (18.0 ≫ 10.5).
    climb_alt = max(transit_alt - 2.0, 2.0,
                    prof.search_floor_m + _ALT_BIAS_MARGIN_M)
    # Revisit-decode hovers ABOVE the search floor by the same frame-bias margin
    # the transit uses under the ceiling — commanding exactly at the floor let
    # the ±0.7 m EKF/home drift dip the reading below 10 m and trip the floor
    # advisory / verify_flight on a rules-compliant hover (G4 2026-07-04).
    _floor = max(prof.search_floor_m, 0.0)
    decode_alt = (_floor + _ALT_BIAS_MARGIN_M) if _floor else sweep_alt
    # The serve ledger renders the plan's GOTO+DROP section; it spans flights.
    discovered: list[ServedStop] = []
    delivered = 0
    include_search = True

    def _running() -> bool:
        return state.terminal == TerminalState.RUNNING

    def _phase(p: MissionPhase) -> None:
        state.phase = p
        if on_phase is not None:
            try:
                on_phase(p)
            except Exception:
                logger.exception("[mission] on_phase hook raised")

    def _rebuild_plan(flight: int) -> None:
        plan = render_live_plan(
            home, spec, discovered=list(discovered), profile=prof,
            transit_route=transit_route, include_search=include_search,
            sortie=flight)
        state.plan = plan
        if on_plan_update is not None:
            try:
                on_plan_update(plan, state.command_pointer)
            except Exception:
                logger.exception("[mission] on_plan_update hook raised")

    def _drain_tracker() -> None:
        for ev in tracker.drain_events():
            state.record_audit(f"t={state.time_elapsed_s():.1f}s {ev}")

    def _cur_latlon() -> tuple[float, float] | None:
        t = state.telemetry
        if math.isnan(t.lat) or math.isnan(t.lon):
            return None
        return (t.lat, t.lon)

    async def _wait_arrival(point: tuple[float, float], timeout_s: float) -> bool:
        """``timeout_s`` is the distance-derived budget; _ProgressGuard turns
        it into a hard ceiling and abandons the leg on a STALL instead."""
        guard = _ProgressGuard(timeout_s, clock=state.now)
        while _running():
            cur = _cur_latlon()
            d = (_latlon_dist_m(cur[0], cur[1], point[0], point[1])
                 if cur is not None else None)
            if d is not None and d <= _ARRIVAL_RADIUS_M:
                return True
            if guard.give_up(d):
                return False
            await asyncio.sleep(_LOOKOUT_POLL_S)
        return False

    async def _telemetry_sampler() -> None:
        """1 Hz flight samples into the audit trail — the data source for the
        post-flight drone-response verifier (tools/verify_flight.py).

        Also the mission's heartbeat for the energy readout. Without it the
        pack figures on the GCS would only move at a pre-flight hold, so they
        would sit at their GO-time values for a whole 4-6 minute sortie while
        the panel presented them as live.
        """
        while _running():
            t = state.telemetry
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s TELEM phase={state.phase.value} "
                f"flight={state.sortie_index} lat={t.lat:.7f} lon={t.lon:.7f} "
                f"alt={t.relative_alt_m:.2f} armed={int(bool(t.is_armed))}")
            if refresh_energy is not None:
                try:
                    refresh_energy()
                except Exception:  # noqa: BLE001 — a readout must never end a sortie
                    logger.exception("[mission] energy refresh failed")
            await asyncio.sleep(_TELEM_SAMPLE_S)

    async def _set_climb_cap(mps: float) -> None:
        """Cap the AUTO climb speed (PX4 MPC_Z_VEL_MAX_UP). The staged 2 m
        closure onto the 20 m transit altitude overshoots ~+0.6 m at the fast
        2 m/s cap — through the ceiling; at 1 m/s the peak is ~+0.15 m.
        Non-fatal: a failed set leaves the previous cap."""
        setter = getattr(commander, "set_param_float", None)
        if setter is None:
            return
        try:
            await setter("MPC_Z_VEL_MAX_UP", float(mps))
        except Exception:
            logger.debug("[mission] MPC_Z_VEL_MAX_UP set failed (non-fatal)")

    async def _set_descent_speed(mps: float) -> None:
        """Set the AUTO descent speed (PX4 MPC_Z_V_AUTO_DN).

        This is the parameter autonomous descents actually read;
        MPC_Z_VEL_MAX_DN is the manual/offboard limit and leaving the mission
        to it was why a first staged-descent attempt sank at 0.4 m/s and saved
        nothing (SITL 2026-07-20).

        A failed set is only harmless in ONE direction, which this used to get
        wrong (the old comment claimed "never faster"): going UP to the staging
        speed, a failure leaves the slower pad value and merely wastes time —
        but coming back DOWN to the pad value, a failure leaves the aircraft
        pinned at the staging speed, 3.75x the descent every land-ON was
        validated at, for the NEXT flight's approach onto a 1 m pad with an egg
        aboard. The tuning push happens once at mission start, so nothing else
        would correct it until the mission ends. So the restore direction gets
        a read-back and a retry, and shouts if it still cannot get there;
        the staging direction stays best-effort.
        """
        setter = getattr(commander, "set_param_float", None)
        if setter is None:
            return
        slowing = mps <= _PAD_DESCENT_MPS
        for attempt in (1, 2, 3):
            try:
                await setter("MPC_Z_V_AUTO_DN", float(mps))
                if not slowing:
                    return
                getter = getattr(commander, "get_param_float", None)
                if getter is None:
                    return
                got = float(await getter("MPC_Z_V_AUTO_DN"))
                if abs(got - mps) <= 1e-3:
                    return
                logger.warning(
                    f"[mission] MPC_Z_V_AUTO_DN read back {got:.2f} after "
                    f"setting {mps:.2f} (attempt {attempt}/3)")
            except Exception as e:  # noqa: BLE001
                if not slowing:
                    logger.debug("[mission] MPC_Z_V_AUTO_DN set failed (non-fatal)")
                    return
                logger.warning(
                    f"[mission] MPC_Z_V_AUTO_DN restore failed "
                    f"(attempt {attempt}/3): {e}")
            await asyncio.sleep(0.2)
        state.record_anomaly(
            f"MPC_Z_V_AUTO_DN stuck above the validated pad descent "
            f"({_PAD_DESCENT_MPS} m/s) — the next pad approach would descend at "
            "the staging speed; land and fix before flying another egg")

    async def _wait_descent(target_alt_m: float, *, timeout_s: float) -> bool:
        """Wait until the vehicle has actually sunk to `target_alt_m`.

        goto() only publishes a setpoint and returns, so issuing land()
        straight after it would cancel the descent that was just commanded —
        the altitude has to be observed, not assumed. On timeout the caller
        lands from wherever it is: a sluggish descent must never strand the
        aircraft airborne over the L&R site."""
        t0 = state.now()
        while _running() and (state.now() - t0) < timeout_s:
            alt = state.telemetry.relative_alt_m
            if alt is not None and alt <= target_alt_m + _LAND_STAGE_TOL_M:
                return True
            await asyncio.sleep(_LOOKOUT_POLL_S)
        return False

    async def _wait_climb(target_alt_m: float, *, timeout_s: float) -> bool:
        """Wait until the vehicle has actually climbed to `target_alt_m` (M5,
        review 2026-07-24) — the climb-out mirror of ``_wait_descent`` above.

        goto() only publishes a setpoint and returns; without this,
        ``_climb_out_to_hop_alt``'s own elif branch (already airborne, still
        below the hop altitude) would hand control back to its caller before
        the climb was actually observed — the exact non-blocking-goto failure
        the climb-out was added (I7) to guard against, one level up. On
        timeout the caller proceeds anyway from wherever it is: a sluggish
        climb must never strand the flight mid-delivery."""
        t0 = state.now()
        while _running() and (state.now() - t0) < timeout_s:
            alt = state.telemetry.relative_alt_m
            if (alt is not None and not math.isnan(alt)
                    and alt >= target_alt_m - _LAND_STAGE_TOL_M):
                return True
            await asyncio.sleep(_LOOKOUT_POLL_S)
        return False

    async def _fly_transit(flight: int, *, egress: bool) -> None:
        """Fly the mandatory transit corridor in order (scored per point).
        Every pass/miss is audited — the judges score each coordinate. The
        first point closes the last ~2 m of climb at a gentle cap so the
        transit altitude is captured without busting the ceiling."""
        _phase(MissionPhase.TRANSIT_EGRESS if egress else MissionPhase.TRANSIT_INGRESS)
        pts = list(reversed(transit_route)) if egress else list(transit_route)
        n_route = len(transit_route)
        await _set_climb_cap(1.0)
        for k, p in enumerate(pts, start=1):
            if not _running():
                return
            n = (n_route - k + 1) if egress else k        # P-number as published
            state.command_pointer = pointer_for(
                state.plan, transit_index=k, egress=egress)
            await commander.goto(p.lat, p.lon, transit_alt)
            cur = _cur_latlon()
            dist = _latlon_dist_m(cur[0], cur[1], p.lat, p.lon) if cur else 200.0
            ok = await _wait_arrival(
                (p.lat, p.lon),
                timeout_s=2.0 * dist / max(spec.speed_mps, 0.1) + _WAIT_PAD_S)
            cur = _cur_latlon()
            d = (_latlon_dist_m(cur[0], cur[1], p.lat, p.lon)
                 if cur else float("nan"))
            tag = "TRANSIT_PASS" if ok else "TRANSIT_MISS"
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s {tag} P{n} "
                f"{'egress' if egress else 'ingress'} flight={flight} d={d:.1f}m")
            if not ok:
                state.record_anomaly(
                    f"transit_{'egress' if egress else 'ingress'}_P{n}_missed")
            if k == 1:
                await _set_climb_cap(2.0)   # captured — restore the fast cap

    async def _sweep_for(flight: int) -> None:
        """Fly the FULL boustrophedon sweep (finish-sweep-then-serve, operator
        decision 2026-07-03): discovery only, feeding every decoded pad into
        the cross-flight registry. Early-stop only once there is nothing more
        the sweep can win — see _done()."""

        def _done() -> bool:
            # Locked decision (CLAUDE.md §2): finish-sweep-then-serve — the
            # sweep runs to COMPLETION so later flights fly direct; early-stop
            # is legitimate ONLY once the registry covers the whole field
            # (max_pads distinct confirmed ids) — never on "this flight's own
            # ids happen to be confirmed", which strands the registry and
            # makes the next flight pay a whole re-sweep.
            # (Review 2026-07-24: this used to also carry
            # `len(wanted) >= max_pads and all(confirmed for a in wanted)`.
            # For a list of DISTINCT ids that arm was dead — the max_pads
            # check above already returns True whenever it would — and for a
            # DUPLICATE-id flight (e.g. headless --assigned-ids "3,3,3,3") it
            # was actively harmful: all(...) collapses onto a single distinct
            # id and truncates the sweep after just one confirmed pad, the
            # exact regression finish-sweep-then-serve exists to prevent.
            # Deleted rather than guarded further — nothing legitimately
            # needs it.)
            return len(tracker.distinct_confirmed_ids()) >= max_pads

        for orig_i, wp in enumerate(spec.waypoints):
            if not _running():
                return
            if _done():
                logger.info(f"[mission] flight {flight}: registry complete "
                            "— sweep done early")
                return
            _phase(MissionPhase.SEARCH)
            state.command_pointer = pointer_for(state.plan, wp_index=orig_i)
            await commander.goto(wp.lat, wp.lon, sweep_alt)
            cur = _cur_latlon()
            leg_len = _latlon_dist_m(cur[0], cur[1], wp.lat, wp.lon) if cur else 0.0
            leg_timeout = 2.0 * leg_len / max(spec.speed_mps, 0.1) + _WAIT_PAD_S
            # Stall-based, not clock-based: the 2026-07-25 flight abandoned
            # this very leg while still closing on it at 8 m/s. _ProgressGuard.
            guard = _ProgressGuard(leg_timeout, clock=state.now)
            while _running():
                _drain_tracker()
                if _done():
                    return
                cur = _cur_latlon()
                d = (_latlon_dist_m(cur[0], cur[1], wp.lat, wp.lon)
                     if cur is not None else None)
                if d is not None and d <= _ARRIVAL_RADIUS_M:
                    break
                if guard.give_up(d):
                    state.record_anomaly(f"sweep_leg_timeout_wp{orig_i}")
                    break
                await asyncio.sleep(_LOOKOUT_POLL_S)

    async def _decode_visits(flight: int, assigned: int, *,
                             identified_only: bool = False,
                             owed: int = 1) -> None:
        """The sweep saw pads it could not fully decode — revisit them at the
        search floor (the lowest legal search altitude) and dwell until the
        marker reads. Identified-but-unconfirmed clusters (decoded, but short
        of confirm_votes) are visited FIRST: topping up their votes is the
        cheap alternative to a full re-sweep (2026-07-08 structural fix);
        ``identified_only`` restricts the visit list to just those (the
        pre-sweep top-up). ``assigned`` = a specific id to hunt (stop as soon
        as it confirms), or -1 to decode every leftover candidate (registry
        completion for later flights)."""
        cands = tracker.identified_unconfirmed(assigned if assigned > 0 else None)
        if not identified_only:
            cands += tracker.unidentified_candidates()
        for cand in cands:
            if not _running() or (assigned > 0 and
                                  tracker.confirmed_by_marker(assigned) is not None):
                return
            # Fund every delivery this flight still OWES, not just one. The
            # old can_start_serve guard sized discovery for a single serve —
            # correct when a flight was one delivery, but a 4-egg flight could
            # legitimately discover until only one serve fit and then have its
            # own per-delivery gate refuse deliveries 3-4: half the score, no
            # rule broken, no anomaly that reads as a problem.
            if not pol.can_start_discovery(state.time_remaining_s(), owed):
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s decode-visits stopped "
                    "(time reserve)")
                return
            _phase(MissionPhase.SEARCH)
            logger.info(f"[mission] decode visit → candidate #{cand.target_id} "
                        f"({cand.lat:.7f},{cand.lon:.7f}) @ {decode_alt:.0f} m")
            await commander.goto(cand.lat, cand.lon, decode_alt)
            await _wait_arrival((cand.lat, cand.lon), timeout_s=60.0)
            t0 = state.now()
            while _running() and (state.now() - t0) < decode_dwell_s:
                _drain_tracker()
                if assigned > 0 and tracker.confirmed_by_marker(assigned) is not None:
                    return
                await asyncio.sleep(_LOOKOUT_POLL_S)
            _drain_tracker()

    async def _climb_out_to_hop_alt() -> None:
        """Climb back to the pad-hop altitude (``sweep_alt``) before flying to
        a pad. A bare goto from landed-but-armed (COM_DISARM_LAND=-1) would
        start the align ACQUIRE budget mid-climb and fly the hop under the
        10 m search floor; a bare goto from a low align rung does the same.
        No-op once already there.

        Two callers: between deliveries (the aircraft is landed ON the
        previous pad) and the top of ``_serve``'s attempt loop — a no-op on
        attempt 1 (the between-deliveries call just above already ran), but
        NOT on the retry (attempt 2): align's own defer climb
        (``tactical_align._goto``) is a NON-BLOCKING goto, so telemetry can
        still read a sub-floor rung the instant ``acquire_and_land_drop``
        hands control back. Without this, the retry's pad-hop goto got
        tagged ``MissionPhase.SEARCH`` while still under the floor, and
        ``tools/verify_flight.py`` hard-FAILs the resulting sub-floor
        ``search`` samples on an otherwise clean run (I7, review 2026-07-24).

        The target is ``sweep_alt`` — the altitude the pad hop itself is
        flown at — NOT the egress ``climb_alt``: the pre-egress climb-out
        uses ``climb_alt`` because the transit needs it, but climbing an
        extra ~5 m here would immediately be given back at the pinned
        ``MPC_Z_V_AUTO_DN`` of 0.4 m/s (~14 s per delivery) for no gain.
        """
        t = state.telemetry
        if (not t.is_armed) or (not math.isnan(t.relative_alt_m)
                                and t.relative_alt_m < 2.0):
            await commander.arm_and_takeoff(sweep_alt)
        elif (not math.isnan(t.relative_alt_m)
                and t.relative_alt_m < sweep_alt - 1.0):
            # e.g. a decode visit left the aircraft hovering at the search
            # floor, or the align defer's own climb-back hadn't finished yet.
            # M5 (review 2026-07-24): goto() alone is non-blocking — the same
            # failure this whole function exists to prevent (see docstring) —
            # so block on the climb actually being observed before returning
            # control to the caller. Not reachable through today's align (its
            # own defer climb always lands below the 2.0 m branch above, not
            # in this band), but the next caller that leaves the aircraft
            # airborne mid-band should not have to rediscover I7 from scratch.
            await commander.goto(t.lat, t.lon, sweep_alt)
            if not await _wait_climb(sweep_alt, timeout_s=30.0):
                logger.warning(
                    "[mission] climb-out to hop altitude timed out at "
                    f"{state.telemetry.relative_alt_m:.1f} m (target "
                    f"{sweep_alt:.1f} m) — proceeding anyway")

    async def _serve(flight: int, assigned: int, *, stop_index: int,
                     payload_id: int, delivery_index: int) -> bool:
        """Land ON the assigned pad + release ONE egg (with a single retry if
        time allows). Leaves the aircraft landed on the pad; the CALLER owns
        the climb-out — before the next delivery, or for the egress transit.

        ``payload_id`` is THIS flight's release channel (0..eggs_aboard-1);
        ``stop_index`` keys the idempotence ledger + the plan's GOTO/DROP pair;
        ``delivery_index`` is the 1-based delivery across the whole mission and
        is what the DELIVERY audit lines are numbered by."""
        # I3 (review 2026-07-24): repoint the GCS "designated pad" highlight
        # at the pad CURRENTLY being served — a flight can carry several eggs,
        # and the old flight-start-only write left every id but flight_ids[0]
        # painted as non-designated while the aircraft was landing on it.
        # Safe to mutate here: the only OTHER writers of assigned_marker_id
        # are the per-flight gate's PREFLIGHT-hold reset (orchestrator/main.py)
        # and the dashboard GO endpoint (dashboard/commands.py) — both run
        # only during the PREFLIGHT hold, never once a flight is airborne, so
        # they can't race this mid-flight write.
        state.assigned_marker_id = assigned
        for attempt in (1, 2):
            if not _running():
                return False
            known = tracker.confirmed_by_marker(assigned)
            if known is None:
                return False
            claimed = tracker.claim_by_marker(assigned)
            _drain_tracker()
            if claimed is None:
                return False
            # I7: on the retry (attempt 2) this is where align's non-blocking
            # defer climb gets finished off before the hop is flown — see
            # _climb_out_to_hop_alt's docstring. No-op on attempt 1.
            await _climb_out_to_hop_alt()
            ledger = ServedStop(stop_index=stop_index, lat=claimed.lat,
                                lon=claimed.lon, name=f"PAD{assigned}",
                                payload_id=payload_id)
            discovered.append(ledger)
            state.command_pointer = pointer_for(
                state.plan, stop_index=stop_index, kind="goto")
            _rebuild_plan(flight)
            _phase(MissionPhase.SEARCH)   # goto-to-pad; align owns LOCALIZE/LAND/DROP
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} START "
                f"pad={assigned} payload={payload_id} stop_index={stop_index}")
            logger.info(
                f"[mission] flight {flight} delivery {delivery_index}: serving pad "
                f"{assigned} ({claimed.lat:.7f},{claimed.lon:.7f}) attempt={attempt}")
            # Fly TO the pad first: the align's ACQUIRE budget times the visual
            # lock, not the transit — starting it early burned the timeout (and
            # the single retry) on flight time every sortie.
            cur0 = _cur_latlon()
            d0 = (_latlon_dist_m(cur0[0], cur0[1], claimed.lat, claimed.lon)
                  if cur0 else 200.0)
            await commander.goto(claimed.lat, claimed.lon, sweep_alt)
            await _wait_arrival(
                (claimed.lat, claimed.lon),
                timeout_s=2.0 * d0 / max(spec.speed_mps, 0.1) + _WAIT_PAD_S)
            serve_params = replace(
                align_p, accept_radius_m=_SERVE_ACCEPT_RADIUS_M,
                assigned_marker_id=assigned)
            res = await acquire_and_land_drop(
                commander, state, Coordinate(lat=claimed.lat, lon=claimed.lon),
                stop_index=stop_index, payload_id=payload_id,
                delivery_index=delivery_index, params=serve_params,
                on_phase=on_phase, on_drop_prediction=on_drop_prediction)
            if res.dropped:
                tracker.mark_served(claimed.target_id)
                # I5: explicit delivered-ids ledger the recovery-flight gate
                # (orchestrator/main.py::_chunk_for) reads — see state.py's
                # field comment for why this can't be inferred from
                # dropped_stops/stop_index instead.
                state.delivered_marker_ids.append(assigned)
                _drain_tracker()
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} END "
                    f"delivered=True pad={assigned} err={res.final_error_m:.2f}m "
                    f"landed={res.landed}")
                return True
            # Not delivered: un-ledger, defer, maybe retry once.
            discovered.pop()
            tracker.defer(claimed.target_id)
            _drain_tracker()
            _rebuild_plan(flight)
            if attempt == 1 and pol.can_start_serve(state.time_remaining_s()):
                logger.warning(f"[mission] flight {flight} delivery "
                               f"{delivery_index}: serve deferred "
                               f"({'; '.join(res.notes)}) — one retry")
                continue
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} END "
                f"delivered=False pad={assigned} notes={'; '.join(res.notes)}")
            return False
        return False

    # ── mission ────────────────────────────────────────────────────────────
    # state.max_sorties IS the flight count (main.py seeds it via
    # mission_brain.flights.budgeted_flights_for — max_flights_for's
    # best-case count PLUS at least one recovery flight, I5 review
    # 2026-07-24; the test harness sets it directly). It is authoritative —
    # do NOT fall back to prof.max_sorties, which counts DELIVERIES (<=4
    # pads), not flights: with eggs_aboard=4 the whole queue is one flight.
    # Written back so the dashboard/preflight readouts publish the normalised
    # value rather than a stale 0.
    max_flights = state.max_sorties = max(1, state.max_sorties)
    logger.info(
        f"[mission] delivery mission: ≤{max_flights} flights "
        f"(eggs_aboard={state.eggs_aboard}), transit {len(transit_route)} pts @ "
        f"{transit_alt:.0f} m, sweep {spec.leg_count} legs @ {sweep_alt:.0f} m, "
        f"window {state.operation_window_s:.0f} s")
    sampler = asyncio.create_task(_telemetry_sampler())
    delivery_no = 0                   # 1-based delivery counter across the mission
    try:
        for flight in range(1, max_flights + 1):
            if not _running():
                break
            # The gate owns the chunking and hands back THIS flight's id list.
            flight_ids = await flight_gate(flight)
            if not flight_ids:
                logger.info(f"[mission] no assignment for flight {flight} — "
                            "mission ends")
                break
            state.start_window()          # first GO starts the 20-min clock
            state.sortie_index = flight
            state.flight_ids = list(flight_ids)
            # Pack reading at launch — the flight's energy cost is the delta at
            # FLIGHT END.
            _entry_mah, _ = energy_consumed_mah(
                state.telemetry, state.energy_capacity_mah)
            _entry_pct = state.telemetry.battery_percent
            # …and the far side of the swap window. The crew could only have
            # changed the pack since the last flight ended, so compare against
            # what it read then. The baseline is rebased from the NEW pack's own
            # charge: assuming every spare arrives full would budget a half-used
            # one as if it held 7,500 mAh.
            if flight > 1 and detect_battery_swap(
                    state.energy_exit_mah, _entry_mah,
                    state.energy_exit_pct, _entry_pct):
                state.energy_baseline_mah = baseline_for_pack(
                    _entry_mah, _entry_pct, state.energy_capacity_mah)
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s BATTERY SWAP before flight "
                    f"{flight} baseline={state.energy_baseline_mah:.0f}mAh")
                logger.info(f"[mission] pack change detected before flight {flight} "
                            f"— energy baseline rebased to "
                            f"{state.energy_baseline_mah:.0f} mAh")
            # The first id still drives the single-assignment readouts (GCS chip,
            # align's id gate default); every id is in state.flight_ids.
            state.assigned_marker_id = flight_ids[0]
            missing = [a for a in flight_ids
                       if tracker.confirmed_by_marker(a) is None]
            include_search = bool(missing)
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} START "
                f"eggs={len(flight_ids)} ids={','.join(map(str, flight_ids))} "
                f"remaining={state.time_remaining_s():.0f}s")
            # Defensive config check: this loop is the first code able to emit
            # payload_id > 0, and DroneCommander.drop_payload REFUSES a
            # payload_id at/above the configured release count. Surface a
            # kit/config mismatch here, on the ground, instead of as an
            # exception over a pad with an egg aboard. Non-fatal — the servo
            # count is the crew's problem to fix, not a reason to ground the
            # aircraft mid-window. Read defensively: the test/HITL commanders
            # need not carry a config.
            _n_channels = getattr(
                getattr(commander, "config", None), "drop_payload_count", None)
            if isinstance(_n_channels, int) and len(flight_ids) > _n_channels:
                state.record_anomaly(f"flight{flight}_eggs_exceed_drop_channels")
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} CONFIG WARN "
                    f"eggs={len(flight_ids)} > drop_payload_count={_n_channels} "
                    "— releases beyond the configured servo count will be refused")
                logger.warning(
                    f"[mission] flight {flight}: {len(flight_ids)} eggs assigned "
                    f"but only {_n_channels} release channel(s) configured — "
                    "deliveries past the first will fail to release")
            _rebuild_plan(flight)

            _phase(MissionPhase.TAKEOFF)
            await commander.arm_and_takeoff(climb_alt)
            await _fly_transit(flight, egress=False)

            # Find every missing id BEFORE serving any of them
            # (finish-sweep-then-serve). This is the original per-sortie
            # discovery ladder generalised over the flight's ids, so
            # eggs_aboard=1 is behaviourally identical: vote top-up → full
            # sweep → per-id decode visits → opportunistic registry completion.
            # Recomputed HERE, not reused from the pre-takeoff read: the vision
            # worker feeds the registry continuously, so a pad can confirm
            # during the takeoff or the ingress transit — and a stale `missing`
            # would then fly the whole discovery ladder for nothing.
            missing = [a for a in flight_ids
                       if tracker.confirmed_by_marker(a) is None]
            if _running() and missing:
                # Vote top-up (2026-07-08 structural fix): a pad an earlier
                # sweep decoded but not often enough to CONFIRM has a KNOWN
                # position — a short decode visit tops up its votes far cheaper
                # than the full re-sweep an unregistered assignment otherwise
                # costs. Falls through to the sweep if the visit doesn't
                # confirm (a false decode must never block discovery).
                # Every discovery excursion below is funded for the deliveries
                # this flight still OWES (len(flight_ids) here — nothing has
                # been served yet at this point in the flight).
                owed = len(flight_ids)
                for a in missing:
                    if tracker.identified_unconfirmed(a):
                        state.record_audit(
                            f"t={state.time_elapsed_s():.1f}s registry top-up: "
                            f"pad={a} identified-unconfirmed — decode "
                            f"visit before sweep")
                        await _decode_visits(flight, a, identified_only=True,
                                             owed=owed)
                if _running() and any(tracker.confirmed_by_marker(a) is None
                                      for a in flight_ids):
                    await _sweep_for(flight)
                for a in flight_ids:
                    if _running() and tracker.confirmed_by_marker(a) is None:
                        await _decode_visits(flight, a, owed=owed)
                # Registry completion (opportunistic): a pad the sweep saw but
                # never decoded costs a WHOLE re-sweep on the flight it gets
                # assigned (seed-99 G4: 310 s) — a 30-60 s decode visit now is
                # far cheaper; an identified pad one vote short is the same
                # cheap top-up. Only while the window comfortably covers
                # another full sortie — AND only while a LATER FLIGHT exists to
                # spend the saving: this pass buys nothing for the flight doing
                # it, so at eggs_aboard=4 (one positional chunk) it is provably
                # dead work chasing distractors nobody will ever be assigned,
                # paid for out of the window this flight's own deliveries need.
                #
                # "LATER FLIGHT" here means a POSITIONALLY-PLANNED one — another
                # queue chunk this flight does NOT already own — not a I5
                # recovery flight (I2, review 2026-07-24): budgeted_flights_for
                # floors state.max_sorties at >=2 to fund a recovery flight even
                # at eggs_aboard=4, so `flight < state.max_sorties` went TRUE
                # again on flight 1 and re-enabled this pass. But a recovery
                # flight only ever re-attempts a SUBSET of ids THIS flight's own
                # per-id decode-visit loop (just above) already chased — it can
                # never be assigned an id outside state.assigned_id_queue — so
                # unlike a genuinely different positional flight's DIFFERENT
                # ids, it gets nothing from pre-decoding OTHER leftover
                # candidates. A recovery flight is also contingent (it may
                # never fly at all if this flight succeeds), so betting
                # guaranteed budget on it is strictly worse than betting it on
                # a flight the queue has already committed to flying.
                positional_flights = len(
                    chunk_flights(state.assigned_id_queue, state.eggs_aboard))
                if (_running() and flight < positional_flights
                        and (tracker.unidentified_candidates()
                             or tracker.identified_unconfirmed())
                        and len(tracker.distinct_confirmed_ids()) < max_pads
                        and pol.can_start_sortie(state.time_remaining_s())):
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s registry completion: "
                        f"decoding leftover candidates")
                    await _decode_visits(flight, assigned=-1, owed=owed)

            # Serve by ROUTE, not by the order the operator happened to click
            # (operator 2026-08-18: "ให้ส่งตาม path ที่ใกล้ที่สุดก่อน"). This is
            # the first moment it can be decided: pad positions come from the
            # registry, which only learns them by decoding markers during the
            # sweep that just finished. Distance saved here is battery the sweep
            # has already spent most of, and time inside a window that charges
            # for overtime. `payload_id` stays the serve SLOT below, so the
            # physical release keeps the diagonal AUX 4/1/2/3 order the rack is
            # wired for — only which pad each slot flies to changes.
            if len(flight_ids) > 1:
                t_now = state.telemetry
                here = ((t_now.lat, t_now.lon)
                        if not (math.isnan(t_now.lat) or math.isnan(t_now.lon))
                        else None)
                pad_xy = {}
                for a in flight_ids:
                    tgt = tracker.confirmed_by_marker(a)
                    if tgt is not None:
                        pad_xy[a] = (tgt.lat, tgt.lon)
                # Where the aircraft leaves for after the last egg: ending far
                # from the egress point is a real cost, so it is part of the
                # route being minimised rather than someone else's problem.
                egress_pt = (transit_route[-1].lat, transit_route[-1].lon) \
                    if transit_route else None
                routed = order_by_nearest(flight_ids, pad_xy, here, egress_pt)
                if routed != list(flight_ids):
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s SERVE ORDER flight "
                        f"{flight} queue={','.join(map(str, flight_ids))} "
                        f"routed={','.join(map(str, routed))}")
                    logger.info(f"[mission] flight {flight}: serving nearest-"
                                f"first {routed} (queue was {list(flight_ids)})")
                    flight_ids = routed

            # Serve each assigned id, re-checking the budget before EVERY
            # descent. The aircraft is already committed to this flight, so the
            # gate reserves only the egress + L&R landing — but it must never
            # start a delivery it cannot finish, and never one the FC's
            # low-battery failsafe would interrupt with an egg aboard.
            n_delivered = 0
            for slot, assigned in enumerate(flight_ids):
                if not _running():
                    break
                pct = state.telemetry.battery_percent
                batt_ok = (math.isnan(pct)
                           or pct > prof.rth_battery_pct + _DELIVERY_BATT_MARGIN_PCT)
                if not (pol.can_start_delivery(state.time_remaining_s()) and batt_ok):
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s DELIVERY abort: flight "
                        f"{flight} skipping remaining "
                        f"ids={','.join(map(str, flight_ids[slot:]))} "
                        f"(remaining={state.time_remaining_s():.0f}s "
                        f"batt={pct:.0f}%) — returning with the egg(s)")
                    logger.warning(
                        f"[mission] flight {flight}: budget exhausted before "
                        f"pad {assigned} — {len(flight_ids) - slot} egg(s) come home")
                    break
                if tracker.confirmed_by_marker(assigned) is None:
                    # Never released blind: an undiscovered pad keeps its egg.
                    delivery_no += 1
                    state.delivery_index = delivery_no
                    state.record_anomaly(f"flight{flight}_pad{assigned}_not_found")
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_no} END "
                        f"delivered=False pad={assigned} reason=not_found")
                    continue
                if isinstance(_n_channels, int) and slot >= _n_channels:
                    # The same config mismatch the pre-takeoff WARN above
                    # flags (_n_channels, read once per flight) — this is the
                    # guard that keeps it non-fatal. Unguarded, _serve() would
                    # reach _drop_once's `await commander.drop_payload(...)`
                    # (orchestrator/tactical_align.py), which RAISES
                    # ValueError for a payload_id >= drop_payload_count
                    # (mavlink_adapter/commands.py) — an unhandled exception
                    # here would end the whole mission with the aircraft
                    # landed but still ARMED on a pad, mid-window. Degrade
                    # like an undiscovered pad instead: keep the egg, audit
                    # why, keep flying. Unknown channel count (fakes/tests
                    # without a `config`) is never treated as a shortage —
                    # only an isinstance-confirmed int gates this branch.
                    delivery_no += 1
                    state.delivery_index = delivery_no
                    state.record_anomaly(
                        f"flight{flight}_pad{assigned}_no_release_channel")
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_no} END "
                        f"delivered=False pad={assigned} reason=no_release_channel")
                    continue
                delivery_no += 1
                state.delivery_index = delivery_no
                # Climb out before flying to the next pad — see
                # _climb_out_to_hop_alt's docstring for the full rationale.
                # Placed AFTER the abort gate and the not-found skip so a
                # delivery that never happens doesn't pay for a climb.
                await _climb_out_to_hop_alt()
                # Ledger key: the delivery's own ordinal across the mission.
                # Unique by construction — deriving it from the committee
                # queue's position could collide with the fallback numbering
                # when a manual GO override re-serves a queued id, and
                # _drop_once's idempotence ledger would then silently suppress
                # the second release.
                stop_index = delivery_no - 1
                if await _serve(flight, assigned, stop_index=stop_index,
                                payload_id=slot, delivery_index=delivery_no):
                    n_delivered += 1
                    delivered += 1

            if not _running():
                break
            # Climb out (from the last pad, or from wherever the search left us)
            # to the staged altitude; the egress transit gotos finish the climb.
            t = state.telemetry
            if (not t.is_armed) or (not math.isnan(t.relative_alt_m)
                                    and t.relative_alt_m < 2.0):
                await commander.arm_and_takeoff(climb_alt)
            else:
                await commander.goto(t.lat, t.lon, climb_alt)
            # FINISH the climb before translating (2026-08-15). The comment
            # above — "the egress transit gotos finish the climb" — held only in
            # still air. In the wind actually measured at both fields (8-12 m/s)
            # the aircraft stops climbing the moment it starts translating: the
            # egress sat flat at 2.4 m against a 3.5 m command for the entire
            # leg, so the SCORED transit corridor was never entered on the way
            # home. Climbing is cheap while stationary and nearly free in time
            # (the legs here are ~5 s each); doing it first costs a few seconds
            # and keeps the corridor. Non-fatal by design: on timeout the flight
            # proceeds from wherever it is rather than stranding an egg.
            if not await _wait_climb(climb_alt, timeout_s=15.0):
                state.record_anomaly(
                    f"flight {flight}: egress climb to {climb_alt:.1f} m not "
                    "confirmed — flying the corridor from below it")
            await _fly_transit(flight, egress=True)
            if not _running():
                break

            # Explicit goto L&R + land + DISARM (resupply crew approaches).
            _phase(MissionPhase.LAND)
            state.command_pointer = len(state.plan.commands) - 1
            cur = _cur_latlon()
            dist = _latlon_dist_m(cur[0], cur[1], home.lat, home.lon) if cur else 300.0
            await commander.goto(home.lat, home.lon, transit_alt)
            home_timeout = 2.0 * dist / max(spec.speed_mps, 0.1) + _WAIT_PAD_S + 10.0
            await _wait_arrival((home.lat, home.lon), timeout_s=home_timeout)
            cur = _cur_latlon()
            d_home = (_latlon_dist_m(cur[0], cur[1], home.lat, home.lon)
                      if cur else float("nan"))
            # Stage the descent down to the hand-over altitude as a position
            # leg (MPC_Z_V_AUTO_DN), then let AUTO.LAND crawl the last few
            # metres onto the L&R pad. See _LAND_STAGE_ALT_M.
            await _set_descent_speed(_LAND_STAGE_MPS)
            stage_alt = min(_LAND_STAGE_ALT_M, transit_alt)
            await commander.goto(home.lat, home.lon, stage_alt)
            drop_m = max(0.0, transit_alt - stage_alt)
            staged = await _wait_descent(
                stage_alt,
                timeout_s=drop_m / _LAND_STAGE_MIN_MPS + _WAIT_PAD_S)
            if not staged:
                logger.warning(
                    f"[mission] flight {flight}: staged descent to "
                    f"{stage_alt:.1f} m timed out at "
                    f"{state.telemetry.relative_alt_m:.1f} m — landing anyway")
            logger.info(f"[mission] flight {flight}: over L&R (d={d_home:.1f} m, "
                        f"alt={state.telemetry.relative_alt_m:.1f} m) "
                        "→ land + disarm for resupply")
            await commander.land(disarm=True)
            # Hand the pad-approach descent back immediately: the next flight
            # localizes onto a 1 m pad with eggs aboard and must fly the descent
            # the release accuracy was validated at, not the staging one.
            await _set_descent_speed(_PAD_DESCENT_MPS)
            # I4 (review 2026-07-24): give the 1 Hz audit sampler a guaranteed
            # chance to record a POST-disarm sample before this END line. With
            # >1 flight the NEXT flight's preflight hold used to supply that
            # for free; at the shipping eggs_aboard=4 default (ONE flight)
            # there is no next hold, and nothing below awaits again before the
            # loop's own `finally: sampler.cancel()` — so without this,
            # tools/verify_flight.py's disarm-after-L&R check (the one that
            # tells the resupply crew it is safe to approach) would have no
            # TELEM evidence at all. commander.land(disarm=True) already
            # blocks until the FC confirms disarm, so this only waits for the
            # SAMPLER, not the aircraft — two full periods so a
            # scheduling-boundary tick can't race it.
            await asyncio.sleep(2 * _TELEM_SAMPLE_S)
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} END "
                f"delivered={n_delivered}/{len(flight_ids)} d_home={d_home:.1f}m "
                f"remaining={state.time_remaining_s():.0f}s")
            state.sortie_time_ok = (
                pol.can_start_known_sortie(state.time_remaining_s())
                if tracker.distinct_confirmed_ids()
                else pol.can_start_sortie(state.time_remaining_s()))
            # Energy accounting: what this flight cost. The delta is taken over
            # the flight itself, so it describes the AIRCRAFT and stays valid
            # across a pack change. A non-positive delta is not a cost — in SITL
            # PX4's battery simulator recharges on disarm, so the exit reading can
            # sit above the entry one.
            _exit_mah, _ = energy_consumed_mah(
                state.telemetry, state.energy_capacity_mah)
            _exit_pct = state.telemetry.battery_percent
            if not math.isnan(_exit_mah) and not math.isnan(_entry_mah):
                _cost = _exit_mah - _entry_mah
                if _cost > 0:
                    state.sortie_energy_mah.append(_cost)
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} ENERGY "
                        f"{_cost:.0f}mAh total={_exit_mah:.0f}mAh")
            # Remember the pack as we left it. A swap happens AFTER this point —
            # during the resupply hold before the next GO — so it can only be
            # detected by comparing these readings with the next flight's entry
            # ones. Checking entry-vs-exit of the SAME flight, as this did until
            # 2026-07-22, spans a window in which the aircraft is armed and
            # airborne: a real swap could never be seen there, and in SITL the
            # simulator's recharge-on-disarm fired it spuriously instead.
            state.energy_exit_mah = _exit_mah
            state.energy_exit_pct = _exit_pct
    finally:
        sampler.cancel()
        await asyncio.gather(sampler, return_exceptions=True)

    if _running():
        state.set_terminal(TerminalState.COMPLETED, MissionPhase.LAND)
        logger.info(f"[mission] complete: {delivered} delivered over "
                    f"{state.sortie_index} flights → completed")
    else:
        logger.warning(f"[mission] ended under watchdog control: {state.terminal.value}")
