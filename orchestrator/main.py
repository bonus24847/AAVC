"""AAVC competition orchestrator — entry point (rules V1.3).

Wires the lightweight flight stack and flies the multi-sortie egg-delivery
mission (pad positions unknown at takeoff — the committee assigns a marker id
per sortie; pads are discovered in flight and remembered across sorties):

    DroneCommander (MAVSDK) ── connect ── PX4 (SITL or real 6X)
        │
        ├── TelemetrySubscriber      → state.telemetry (synchronized snapshot)
        ├── SafetyWatchdog           → battery / GPS / geofence / no-fly /
        │                              ceiling / datalink / time
        ├── VisionWorker → TargetTracker → decode + confirm landing pads
        ├── dashboard (optional)     → live map + cameras + per-sortie GO gate
        └── run_delivery_mission     → per sortie: transit P1→P2→P3 @20 m →
                                        (sweep if pad unknown) → land ON the
                                        assigned pad → release the egg →
                                        egress transit → land at L&R → resupply

Offline + deterministic by design — no LLM, no network. Safe to run headless
(``--no-dashboard`` + ``--assigned-ids``) or with the GCS.

    python -m orchestrator.main --config sitl/aavc_config.yaml
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import yaml
from loguru import logger

from mavlink_adapter.commands import DEFAULT_PX4_TUNING, ConnectionConfig, DroneCommander
from mavlink_adapter.raw_subscriber import RawMavlinkSubscriber
from mavlink_adapter.telemetry import TelemetrySubscriber
from mission_brain.flights import budgeted_flights_for, chunk_flights, remaining_owed
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import load_profile
from mission_brain.schemas import Coordinate, MissionPhase
from mission_brain.search_pattern import build_search_pattern
from tuning.gains_io import load_gains
from vision.detectors.aruco import VALID_MARKER_IDS
from vision.projection import configure_cameras

from . import audit, preflight
from .energy_policy import EnergyPolicy, energy_consumed_mah
from .frame_recorder import FrameRecorder
from .mission import FlightGate, run_delivery_mission
from .safety import SafetyWatchdog
from .state import OrchestratorMode, OrchestratorState, TerminalState
from .tactical_align import AlignParams
from .target_tracker import TargetState, TargetTracker
from .time_policy import TimePolicy
from .vision_worker import DEFAULT_FRAME_MAX_AGE_S, VisionWorker


def _load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        logger.warning(f"[main] config {path} not found — using defaults")
        return {}
    return yaml.safe_load(path.read_text()) or {}


def _parse_assigned_ids(raw: str | list[Any]) -> list[int]:
    """Parse + validate the committee-assigned marker ids (L2).

    Accepts a comma string (``--assigned-ids "3,1,4,6"``) or a list (config
    ``mission.assigned_marker_ids``). Every id must be a competition marker id
    (1..6); an out-of-range or non-integer id raises with a clear message rather
    than being silently accepted and then never decoded (burning window time)."""
    valid = sorted(VALID_MARKER_IDS)
    tokens = ([t.strip() for t in raw.split(",")] if isinstance(raw, str)
              else [str(x).strip() for x in raw])
    ids: list[int] = []
    for tok in tokens:
        if not tok:
            continue
        try:
            value = int(tok)
        except (TypeError, ValueError):
            raise ValueError(
                f"assigned marker id {tok!r} is not an integer; valid ids are {valid}"
            ) from None
        if value not in VALID_MARKER_IDS:
            raise ValueError(f"assigned marker id {value} outside the valid set {valid}")
        ids.append(value)
    return ids


def _build_time_policy(sc: dict[str, Any], profile: Any) -> TimePolicy:
    """Build the sortie time-budget policy from the `search:` config block (L1).

    Only keys actually present override the TimePolicy dataclass defaults — so
    the single source of fallback truth is the dataclass, not stale inline
    literals (the old inline 300/200 fallbacks disagreed with both the dataclass
    350/210 and the shipped config)."""
    keys = ("serve_cost_s", "sortie_cost_s", "known_sortie_cost_s", "rth_reserve_s")
    overrides = {k: float(sc[k]) for k in keys if k in sc}
    return TimePolicy(watchdog_floor_s=profile.min_time_remaining_s, **overrides)


def _build_energy_policy(cfg: dict[str, Any], reserve_frac: float,
                         eggs_aboard: int = 1) -> EnergyPolicy:
    """Build the FLIGHT energy-budget policy (the twin of the time policy).

    The reserve fraction comes from `failsafes.bat_low_thr`, not a key of its
    own: below that threshold the FC flies its own low-battery RTL, so a second
    number here could only ever drift out of agreement with it.

    ``eggs_aboard`` comes from the resolved mission config for the same reason:
    the seed cost of a flight scales with the deliveries it carries, and a seed
    frozen at one delivery would let the pre-flight gate approve a four-egg
    flight the pack cannot finish.
    """
    bat = cfg.get("battery") or {}
    keys = ("capacity_mah", "seed_sortie_mah", "seed_delivery_mah", "margin_mah")
    overrides = {k: float(bat[k]) for k in keys if k in bat}
    return EnergyPolicy(reserve_frac=reserve_frac,
                        eggs_aboard=max(1, int(eggs_aboard)), **overrides)


def _build_connection(cc: dict[str, Any], connect_override: str | None) -> ConnectionConfig:
    """Build ConnectionConfig from the optional ``connection:`` config block.

    ``--connect`` (connect_override) wins for the MAVLink endpoint. Everything else —
    the drop servo channel / PWM band / fallback endpoint — can be tuned at the G5
    bench from config without editing source. Omitted keys keep the dataclass defaults.
    """
    kw: dict[str, Any] = {}
    if connect_override:
        kw["system_address"] = connect_override
    elif cc.get("system_address"):
        kw["system_address"] = str(cc["system_address"])
    for key in ("drop_servo_channel", "drop_servo_pwm_release",
                "drop_servo_pwm_hold", "drop_payload_count"):
        if key in cc:
            kw[key] = int(cc[key])
    if "drop_fallback_endpoint" in cc:
        kw["drop_fallback_endpoint"] = str(cc["drop_fallback_endpoint"])
    for key in ("connect_timeout_s", "arming_timeout_s"):
        if key in cc:
            kw[key] = float(cc[key])
    return ConnectionConfig(**kw)


def _is_sitl_endpoint(system_address: str) -> bool:
    """True when the MAVLink endpoint is a simulator, not a wired flight board.

    SITL speaks UDP (`udpin://0.0.0.0:14540`); the CM4 talks to the 6X over a
    serial port (`serial:///dev/ttyAMA0`). Anything sim-only — PX4's battery
    simulator today — is gated on this rather than on a config block being
    present, because the real bird flies the SAME config file
    (cm4/launch_flight.sh defaults CONFIG=sitl/aavc_config.yaml).
    """
    return system_address.strip().lower().startswith(("udp", "tcp"))


async def _wait_for_gps(state: OrchestratorState, timeout_s: float = 30.0) -> bool:
    """Block until a usable GPS fix arrives (lat/lon non-NaN, ≥2D fix)."""
    t0 = asyncio.get_running_loop().time()
    while asyncio.get_running_loop().time() - t0 < timeout_s:
        t = state.telemetry
        if not math.isnan(t.lat) and not math.isnan(t.lon) and t.gps_fix_type >= 2:
            return True
        await asyncio.sleep(0.5)
    return False


def _evaluate_energy(state: OrchestratorState, energy_policy: EnergyPolicy) -> None:
    """Refresh the energy-budget hint the pre-flight card and the GO gate read.

    Kept beside the time-budget evaluation so the two stay symmetric: neither is
    a flight action, both only refuse to START work.
    """
    consumed, tier = energy_consumed_mah(state.telemetry, energy_policy.capacity_mah)
    rel = (consumed - state.energy_baseline_mah
           if not math.isnan(consumed) else consumed)
    state.energy_tier = tier
    state.sortie_energy_ok, state.energy_detail = energy_policy.can_start_sortie(
        rel, state.sortie_energy_mah)
    state.energy_sorties_left = energy_policy.sorties_remaining(
        rel, state.sortie_energy_mah)


def _sortie_gate_factory(
    state: OrchestratorState,
    *,
    dash: Any,
    home: Coordinate,
    geofence: list[tuple[float, ...]],
    cfg: dict[str, Any],
    profile: Any,
    policy: Any,
    tracker: TargetTracker,
    energy_policy: Any = None,
    skip_preflight: bool = False,
    headless_timeout_s: float = 120.0,
) -> FlightGate:
    """Build the per-FLIGHT gate (rules V1.3): before EVERY flight the mission
    holds in ``MissionPhase.PREFLIGHT`` until the operator GOs.

    The gate owns the CHUNKING: it hands the mission loop flight i's list of at
    most ``state.eggs_aboard`` committee-assigned ids —
    ``chunk_flights(state.assigned_id_queue, state.eggs_aboard)[i-1]`` — or None
    to end the mission. The loop just serves the list it is handed.

    Flight indices PAST those positional chunks are RECOVERY flights (I5,
    review 2026-07-24): ``_chunk_for`` then carries whatever the queue still
    OWES — ``state.assigned_id_queue`` minus ``state.delivered_marker_ids``
    (``mission_brain.flights.remaining_owed``) — so a flight that comes home
    with undelivered eggs (a pad never found, a per-delivery budget abort, a
    release-channel shortage) is automatically followed by one that carries
    exactly those, no operator re-queue needed. ``state.max_sorties`` is
    seeded (main's ``run()``) via ``budgeted_flights_for`` precisely so at
    least one such flight is always budgeted; a fully-served queue makes the
    owed set empty and ``_chunk_for`` returns None, ending the mission
    immediately rather than flying a pointless extra flight.

    With a dashboard: re-evaluate + push the readiness board at ~1 Hz and wait
    for ``/api/cmd/preflight/go`` — the endpoint resolves an assigned marker id
    (a manual pick, or the queue's slot for this hold) into
    ``state.assigned_marker_id`` (and enforces the time/energy policy unless
    forced) before setting the event. That single id is the documented
    per-flight MANUAL OVERRIDE and applies at ``eggs_aboard == 1`` (where a
    flight IS one delivery) or whenever this flight has no chunk to fly
    (``not chunk``) — the queue is empty, or already exhausted by an earlier
    flight; a multi-egg flight WITH a chunk always flies that chunk.
    Headless: take the same chunk from the SAME ``state.assigned_id_queue``
    (seeded from --assigned-ids) and auto-proceed when the criticals pass. The
    queue is re-read from state on every call so a mid-mission
    /api/cmd/mission_ids update applies at the next hold.
    """
    pf = cfg.get("preflight", {}) or {}
    kwargs: dict[str, Any] = dict(
        geofence=[list(v) for v in geofence],
        home_lat=home.lat,
        home_lon=home.lon,
        # The critical floor is the FC's own low-battery threshold — below it a
        # launch flies straight into the failsafe. Default to that rather than to
        # a second, independently-drifting number; the charge ABOVE it is the
        # energy budget's call (advisory row, forceable refusal).
        min_battery_pct=float(pf.get(
            "battery_pct",
            100.0 * (energy_policy.reserve_frac if energy_policy is not None
                     else 0.25))),
        min_gps_sats=int(pf.get("gps_sats", 6)),
        min_time_remaining_s=profile.min_time_remaining_s,
        camera_max_age_s=float(pf.get("camera_max_age_s", 5.0)),
    )
    broadcaster = getattr(dash, "broadcaster", None)
    can_push = broadcaster is not None and hasattr(broadcaster, "push_preflight")

    def _evaluate() -> bool:
        report = preflight.run_preflight(state, **kwargs)
        state.preflight_can_go = report.all_critical_pass
        state.preflight_report = report.to_dict()
        # The window clock only bites once started (first GO); before that a
        # sortie can always start. Once the registry holds confirmed pads the
        # likely next sortie is a direct (known-pad) one — hint with the
        # known-sortie rule so a fittable sortie isn't discouraged.
        if energy_policy is not None:
            _evaluate_energy(state, energy_policy)
        if not state.window_started:
            state.sortie_time_ok = True
        elif tracker.distinct_confirmed_ids():
            state.sortie_time_ok = policy.can_start_known_sortie(
                state.time_remaining_s())
        else:
            state.sortie_time_ok = policy.can_start_sortie(
                state.time_remaining_s())
        if broadcaster is not None and hasattr(broadcaster, "push_preflight"):
            try:
                broadcaster.push_preflight(state.preflight_report)
            except Exception:
                logger.exception("[preflight] push failed")
        return report.all_critical_pass

    def _chunk_for(flight: int) -> list[int] | None:
        """THIS flight's ≤eggs_aboard slice of the committee queue. Re-read
        from state on every call — the GCS may set/extend the queue
        mid-mission, and GO resolves at click time.

        Flight indices WITHIN the queue's positional chunks fly them exactly
        as chunked. A flight index PAST them is a RECOVERY flight (I5): it
        carries whatever the queue still OWES — the positional queue minus
        what ``state.delivered_marker_ids`` says has actually been delivered
        — chunked the same size. None once nothing remains owed (including
        when nothing was ever queued), so the mission ends on this flight
        rather than holding for a pointless GO.
        """
        flights = chunk_flights(state.assigned_id_queue, state.eggs_aboard)
        if 1 <= flight <= len(flights):
            return flights[flight - 1]
        owed = remaining_owed(state.assigned_id_queue, state.delivered_marker_ids)
        if not owed:
            return None
        owed_chunks = chunk_flights(owed, state.eggs_aboard)
        idx = flight - len(flights) - 1
        return owed_chunks[idx] if 0 <= idx < len(owed_chunks) else None

    async def _gate(sortie: int) -> list[int] | None:
        if state.terminal != TerminalState.RUNNING:
            return None
        chunk = _chunk_for(sortie)
        # I1 (review 2026-07-24): _chunk_for already returns None once every
        # queued id is actually delivered — but that alone did not stop the
        # INTERACTIVE hold below from running anyway (a green PREFLIGHT board
        # after a completed mission), and /api/cmd/preflight/go resolves its
        # id POSITIONALLY (assigned_id_queue[sortie_index]) with no notion of
        # state.delivered_marker_ids — so the documented one-click GO handed
        # back an ALREADY-DELIVERED id, which the manual-override branch below
        # then turned into a bogus extra flight with no egg aboard. End the
        # mission here instead, for every branch (interactive/headless/skip),
        # the moment nothing remains owed. Gated on a NON-EMPTY queue: with no
        # queue at all, `chunk` is also None on flight 1 (nothing to chunk
        # yet), and that is the documented "no queue → the GO's manual pick IS
        # the flight" path — the only legitimate `not chunk` case, preserved
        # by this check never firing when the queue is empty.
        if (chunk is None and state.assigned_id_queue
                and not remaining_owed(state.assigned_id_queue,
                                       state.delivered_marker_ids)):
            return None
        if skip_preflight:
            logger.warning(f"[preflight] SKIPPED (--skip-preflight) — flight "
                           f"{sortie} assigned={chunk}")
            return chunk

        if can_push:
            # Interactive: hold for the operator's GO (which carries the id).
            state.assigned_marker_id = None
            state.awaiting_preflight_go = True
            state.phase = MissionPhase.PREFLIGHT
            state.preflight_resume_event.clear()
            logger.info(f"[preflight] flight {sortie}: holding for operator GO "
                        "(enter the committee-assigned pad id)…")
            while state.terminal == TerminalState.RUNNING:
                _evaluate()
                try:
                    await asyncio.wait_for(
                        state.preflight_resume_event.wait(), timeout=1.0)
                    break
                except asyncio.TimeoutError:
                    continue
            state.awaiting_preflight_go = False
            if (state.terminal != TerminalState.RUNNING
                    or not state.preflight_resume_event.is_set()):
                return None
            # Resolve AFTER the GO: the operator may have (re)set the queue
            # during the hold, and the endpoint resolves the id at click time.
            chunk = _chunk_for(sortie)
            manual = state.assigned_marker_id
            if manual is not None and (state.eggs_aboard == 1 or not chunk):
                # One egg per flight → a flight IS one delivery, so the GO
                # body's id (a manual pick, else the queue's slot) is the whole
                # flight: the documented per-flight manual override. With more
                # eggs aboard the queue's chunk wins — EXCEPT when there is no
                # queue at all, where the GO's id is the only assignment there
                # is and returning None would end the mission on a green GO.
                chunk = [manual]
            state.flight_ids = list(chunk or [])
            logger.info(f"[preflight] flight {sortie}: GO — assigned pads "
                        f"{state.flight_ids}")
            return chunk

        # Headless: the queued assignments ARE the committee (SITL / bench runs).
        if chunk is None:
            return None
        # A registry-known pad flies a much shorter delivery AND the watchdog's
        # time floor exempts its egress/landing — gate on the matching rule, and
        # only when EVERY pad this flight serves is already registered.
        known = all(tracker.confirmed_by_marker(a) is not None for a in chunk)
        ok = (policy.can_start_known_sortie(state.time_remaining_s()) if known
              else policy.can_start_sortie(state.time_remaining_s()))
        if state.window_started and not ok:
            logger.warning(f"[preflight] sortie {sortie}: window too short "
                           f"({state.time_remaining_s():.0f}s left for a "
                           f"{'known' if known else 'full-sweep'} sortie) — ending")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(time reserve)")
            return None
        # The energy twin of the same refusal. The dashboard GO endpoint has
        # honoured this since the budget shipped; headless did not, so a bench or
        # SITL run would launch on a pack the policy had already refused and let
        # the FC's low-battery failsafe end the sortie with the egg aboard.
        # Headless has no FORCE button — `--skip-preflight` is its escape hatch.
        if energy_policy is not None and not state.sortie_energy_ok:
            logger.warning(f"[preflight] sortie {sortie}: {state.energy_detail} "
                           "— ending (swap the pack, or --skip-preflight)")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(energy reserve)")
            return None
        if not state.param_pins_ok:
            logger.error(f"[preflight] sortie {sortie}: {state.param_pins_detail} "
                         "— refusing to launch outside the validated envelope")
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s sortie {sortie} refused "
                "(envelope params)")
            return None
        state.phase = MissionPhase.PREFLIGHT
        deadline = asyncio.get_running_loop().time() + headless_timeout_s
        logger.info(f"[preflight] sortie {sortie}: headless — auto-proceed when "
                    "all critical checks pass…")
        while (asyncio.get_running_loop().time() < deadline
               and state.terminal == TerminalState.RUNNING):
            if _evaluate():
                logger.info(f"[preflight] flight {sortie}: criticals pass — "
                            f"assigned pads {chunk}")
                return chunk
            await asyncio.sleep(1.0)
        if state.terminal != TerminalState.RUNNING:
            return None
        fails = ", ".join(
            i["id"] for i in (state.preflight_report or {}).get("items", [])
            if i.get("critical") and i.get("status") != "pass"
        )
        logger.error(f"[preflight] sortie {sortie}: headless timeout — unmet "
                     f"critical checks: {fails or 'unknown'}")
        state.record_anomaly("preflight_timeout")
        return None

    return _gate


async def run(args: argparse.Namespace) -> int:
    cfg = _load_config(Path(args.config))
    profile = load_profile(args.profile)
    logger.info(f"[main] profile={profile.name} ceiling={profile.altitude_ceiling_m} m "
                f"transit={profile.transit_alt_m} m floor={profile.search_floor_m} m "
                f"deliveries≤{profile.max_sorties} eggs={profile.eggs_aboard}")

    # Real-lens camera calibration (FOV + mount angles) from config, applied in
    # place to the shared NADIR model. Defaults match SITL; set the REAL
    # values in the config `cameras:` block at G5/G6 before free flight.
    cams = cfg.get("cameras") or {}
    if cams:
        configure_cameras(nadir=cams.get("nadir"))
        logger.info("[main] applied camera calibration from config (cameras:)")

    geofence = [tuple(v) for v in cfg.get("controlled_airspace", [])]
    search_area = [tuple(v) for v in cfg.get("search_area", [])]
    transit_route = [Coordinate(lat=float(v[0]), lon=float(v[1]))
                     for v in cfg.get("transit_route", [])]

    # ── connect ──
    # The fraction below which the FC flies its own low-battery RTL. Resolved
    # ONCE: it is simultaneously the failsafe pinned on the FC, the reserve the
    # energy budget refuses to plan into, and the pre-flight card's hard floor.
    # Read separately by each, they could drift apart and the planner would be
    # planning through a failsafe.
    bat_low_thr = float((cfg.get("failsafes") or {}).get("bat_low_thr", 0.25))
    conn = _build_connection(cfg.get("connection") or {}, args.connect)
    commander = DroneCommander(conn)

    # Bare placeholder telemetry until the subscriber starts (state needs one).
    telem = TelemetrySubscriber(commander.system)
    # Optional raw pymavlink listener: augments telemetry with per-servo PWM,
    # per-ESC current/RPM, and consumed-mAh (fields MAVSDK doesn't expose) for the
    # dashboard. Reads a dedicated UDP endpoint (SITL: PX4's 14550 GCS broadcast;
    # real CM4: a mavlink-router [UdpEndpoint raw]). 0/absent disables it, and any
    # failure degrades gracefully — the safety watchdog never reads these fields.
    raw_telem = RawMavlinkSubscriber(
        telem.state,
        udp_port=int((cfg.get("connection") or {}).get("raw_telemetry_port", 0)),
    )
    # Build a minimal initial plan so state is valid; rebuilt once GPS is known.
    placeholder_home = Coordinate(
        lat=float(cfg.get("site", {}).get("center_lat", 0.0)),
        lon=float(cfg.get("site", {}).get("center_lon", 0.0)),
    )
    # Blind search inside the SEARCH AREA (the pads live there); the controlled
    # airspace is the geofence. Both polygons are mission-critical (V1.3).
    if len(geofence) < 3:
        logger.error("[main] no controlled_airspace polygon — the geofence is mandatory")
        return 2
    if len(search_area) < 3:
        logger.error("[main] no search_area polygon — the sweep needs the rules' "
                     "search area (config search_area:)")
        return 2
    if not transit_route:
        logger.error("[main] no transit_route — the rules mandate the P1→P2→P3 "
                     "corridor (config transit_route:)")
        return 2
    if len(transit_route) != 3:
        logger.warning(f"[main] transit_route has {len(transit_route)} points "
                       "(rules publish 3) — flying it as configured")
    sc = cfg.get("search", {}) or {}
    mc = cfg.get("mission", {}) or {}
    spec = _build_spec(search_area, placeholder_home, sc, profile.altitude_ceiling_m)
    tracker = _build_tracker(sc)
    plan = render_live_plan(placeholder_home, spec, discovered=[], profile=profile,
                            transit_route=transit_route)

    # Headless/SITL committee stand-in: --assigned-ids beats the config list.
    # Both are validated against the competition id set (1..6) — fail fast.
    if args.assigned_ids:
        assigned_ids = _parse_assigned_ids(str(args.assigned_ids))
    else:
        assigned_ids = _parse_assigned_ids(mc.get("assigned_marker_ids", []))

    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem.state,
        operation_window_s=profile.operation_window_s,
        max_sorties=int(mc.get("max_sorties", profile.max_sorties)),
    )
    # ONE queue for both operator flows: --assigned-ids/config seeds it here;
    # the GCS may (re)set it via POST /api/cmd/mission_ids. The per-flight gate
    # and the dashboard GO both consume state.assigned_id_queue.
    state.assigned_id_queue = list(assigned_ids)
    # FLIGHTS vs DELIVERIES. The queue lists DELIVERIES (one pad each); one
    # FLIGHT (arm→disarm) carries up to eggs_aboard of them, so max_sorties —
    # the loop's flight budget — is the number of flights those deliveries need.
    # Config `mission.max_sorties` has always meant "≤4 pads", i.e. deliveries,
    # so it seeds max_deliveries.
    state.eggs_aboard = int(mc.get("eggs_aboard", profile.eggs_aboard))
    state.max_deliveries = int(mc.get("max_deliveries", state.max_sorties))
    # Budget the maximum LEGAL number of flights, not the count the CURRENT
    # queue implies: /api/cmd/mission_ids may extend the queue mid-mission (the
    # queue is re-read at every hold), and a ceiling frozen at the initial
    # length would accept that write and then never fly it. budgeted_flights_for
    # (I5, review 2026-07-24) adds at least one RECOVERY flight beyond that
    # best-case count — at the shipping eggs_aboard=4 default max_deliveries
    # alone budgets exactly ONE flight, so a flight that came home with
    # undelivered eggs previously had no way to get them flown; see
    # _chunk_for's docstring for the gate-side half of this fix. The outer
    # max(1, ...) keeps this line's own floor >= 1 even though mission.py's
    # loop clamps the same way — budgeted_flights_for(0, ...) is 0 (nothing to
    # ever recover), and this is what gets logged just below.
    state.max_sorties = max(1, budgeted_flights_for(state.max_deliveries,
                                                     state.eggs_aboard))
    logger.info(f"[main] {state.max_deliveries} deliveries max, "
                f"{state.eggs_aboard} egg(s) aboard → ≤{state.max_sorties} "
                f"flight(s); queue={state.assigned_id_queue or 'unset'}")
    # Crash-safe audit trail.
    run_dir = audit.setup_run_logging(plan.mission_id)
    audit_log = audit.AuditLog(run_dir / "audit.jsonl")
    state.audit_sink = audit_log.record

    # ── optional GCS dashboard (decoupled seam) ──
    dash = None
    if not args.no_dashboard:
        try:
            from dashboard.integration import start_dashboard
            dash = await start_dashboard(
                state, commander, host=args.host, port=args.port, app_mode=args.mode
            )
            logger.info(f"[main] dashboard up at http://{args.host}:{args.port}")
        except Exception as e:
            logger.warning(f"[main] dashboard unavailable ({e}); running headless")

    on_drop_prediction = getattr(dash, "record_drop", None)

    try:
        await commander.connect()
        state.link_connected = True
        await telem.start()
        await raw_telem.start()
        logger.info("[main] waiting for GPS fix…")
        if not await _wait_for_gps(state):
            logger.error("[main] no GPS fix — aborting")
            state.set_terminal(TerminalState.FAILED, MissionPhase.ABORT)
            return 3

        # Real launch point = current fix (works with coarse, no-RTK GPS). Rebuild
        # the search pattern about the true home now that GPS is known. The sweep
        # covers the SEARCH AREA polygon (the geofence is the airspace boundary).
        home = Coordinate(lat=state.telemetry.lat, lon=state.telemetry.lon)
        spec = _build_spec(search_area, home, sc, profile.altitude_ceiling_m)
        state.plan = render_live_plan(home, spec, discovered=[], profile=profile,
                                      transit_route=transit_route)
        logger.info(f"[main] home=({home.lat:.7f},{home.lon:.7f}); search area "
                    f"{spec.leg_count} legs / {len(spec.waypoints)} waypoints @ "
                    f"{spec.sweep_alt_m:.0f} m, transit {len(transit_route)} pts @ "
                    f"{profile.transit_alt_m:.0f} m")

        # ── FC-level failsafes (independent of the companion) ──
        # ONLY for the scored mission. A System-ID/Autotune sweep (tuning mode) is
        # a GCS-less OFFBOARD excitation flight: the datalink-loss RTL (PX4 sees no
        # QGC heartbeat → Returns), RC-loss RTL, and geofence-breach RTL would each
        # yank the drone out of OFFBOARD mid-chirp and force-land it (verified in
        # SITL: the sweep climbed to 15 m, then a failsafe force-landed it and the
        # chirp aborted at the 6 m floor). Disable them for tuning — the
        # SafetyWatchdog still covers battery / datalink / GPS / telemetry-stale.
        # Set 0 EXPLICITLY (not just "skip"): a prior mission run may have left
        # these enabled on the FC, and SITL params persist until reboot.
        if args.mode != "tuning":
            if geofence and len(geofence) >= 3:
                try:
                    await commander.upload_geofence(geofence)
                    await commander.set_geofence_action_rtl()
                except Exception as e:
                    state.record_anomaly(f"geofence setup failed: {e}")
            try:
                await commander.set_datalink_loss_rtl(profile.datalink_loss_threshold_s)
            except Exception as e:
                state.record_anomaly(f"datalink-loss RTL setup failed: {e}")
            # RC-loss RTL (S4) — never pinned before; the real bird rode on the
            # FC/QGC default. COM_RCL_EXCEPT=4 (inside set_rc_loss_rtl) exempts
            # Offboard/Mission so a no-RC SITL/autonomous run isn't RTL'd.
            try:
                await commander.set_rc_loss_rtl()
            except Exception as e:
                state.record_anomaly(f"RC-loss RTL setup failed: {e}")
            # FC-level battery failsafe (S4) — the authoritative backstop below
            # the companion watchdog's 30%/20% RTH/LAND.
            fs = cfg.get("failsafes", {}) or {}
            try:
                await commander.set_battery_failsafe(
                    low=bat_low_thr,
                    crit=float(fs.get("bat_crit_thr", 0.15)),
                    emergen=float(fs.get("bat_emergen_thr", 0.07)),
                    action=int(fs.get("low_bat_act", 3)),
                )
            except Exception as e:
                state.record_anomaly(f"battery failsafe setup failed: {e}")
            # Stabilized-nadir camera gimbal (PX4 mount driver) — best-effort:
            # SITL's PX4 lacks the module (params warn + skip); the real 6X is
            # configured here so the servo holds the camera straight down.
            gc = cfg.get("gimbal", {}) or {}
            if gc.get("enabled", False) and gc.get("params"):
                try:
                    await commander.set_gimbal_mount(gc["params"])
                except Exception as e:
                    state.record_anomaly(f"gimbal mount setup failed: {e}")
        else:
            for _fs in ("NAV_DLL_ACT", "NAV_RCL_ACT", "GF_ACTION"):
                try:
                    await commander.set_param_int(_fs, 0)
                except Exception as e:
                    state.record_anomaly(f"tuning: disable {_fs} failed: {e}")

        # ── flight tuning (applied once before takeoff) ──
        # (1) Inner-loop rate/attitude gains from the pre-flight System-ID +
        # Autotune module (runs/sysid/<airframe>_gains.json), if a tuning session
        # saved any — so the mission flies with the tuned gains automatically.
        tuned = load_gains()
        if tuned:
            try:
                n = await commander.apply_param_overrides(tuned)
                logger.info(f"[main] applied {n} tuned gains from the System-ID/Autotune module")
                if n < len(tuned):
                    state.record_anomaly(
                        f"tuned gains: only {n}/{len(tuned)} applied — the "
                        "aircraft is flying a MIX of tuned and stock inner-loop "
                        "gains (see the log for which)")
            except Exception as e:
                state.record_anomaly(f"tuned gains apply failed: {e}")
        # (2) Outer-loop limits (anti-flip tilt, yaw-rate, speed unlock, tighter
        # waypoint acceptance). Config `px4_tuning:`, else the reviewed defaults.
        energy_policy = _build_energy_policy(cfg, bat_low_thr, state.eggs_aboard)
        state.energy_capacity_mah = energy_policy.capacity_mah
        px4_tuning = cfg.get("px4_tuning") or DEFAULT_PX4_TUNING
        try:
            n_pins = await commander.apply_param_overrides(px4_tuning)
            if n_pins < len(px4_tuning):
                state.record_anomaly(
                    f"px4 tuning: only {n_pins}/{len(px4_tuning)} params applied")
            # Applying is best-effort; the ENVELOPE is not. Read the pins back —
            # "applied 0/24" (stale mavsdk_server on the ports) would otherwise
            # fly the mission on PX4 defaults: RTL at 60 m and 1.5 m/s onto the pad.
            bad_pins = await commander.verify_envelope_pins(px4_tuning)
        except Exception as e:
            state.record_anomaly(f"px4 tuning apply failed: {e}")
            bad_pins = ["the whole tuning block failed to apply"]
        state.param_pins_ok = not bad_pins
        state.param_pins_detail = (
            "envelope pins held by the FC" if not bad_pins
            else "FC is NOT holding: " + "; ".join(bad_pins))
        if bad_pins:
            state.record_anomaly(
                "flight-envelope params are at their PX4 defaults — "
                + state.param_pins_detail
                + ". A failsafe RTL would climb through the 20 m ceiling and the "
                "pad descent is ~4x the validated speed. Fix the link (a stale "
                "mavsdk_server holding :14540/:50051 fails every param RPC) and "
                "restart, or launch with FORCE at your own risk.")
        # SITL only — PX4's battery simulator, so the energy budget is
        # exercisable before hardware. These params do NOT exist on the real 6X
        # (the module is not in the fmu-v6x build), and cm4/launch_flight.sh
        # ships the same config file, so gate on the SITL endpoint rather than on
        # the block's presence: on hardware each write would only burn a param
        # timeout on the pad, inside the scored window.
        sim_bat = cfg.get("sim_battery") or {}
        if sim_bat and _is_sitl_endpoint(conn.system_address):
            try:
                await commander.apply_param_overrides(
                    {k: float(v) for k, v in sim_bat.items()})
            except Exception as e:  # noqa: BLE001
                logger.debug(f"[main] sim battery params not applied: {e}")
        elif sim_bat:
            logger.info("[main] sim_battery block skipped — not a SITL endpoint "
                        f"({conn.system_address})")
        # A pack swapped without updating the config and an uncalibrated power
        # module look identical from here, and both make every mAh number
        # fiction. Say so once, loudly, rather than silently trusting it.
        try:
            fc_capacity = float(await commander.get_param_float("BAT1_CAPACITY"))
        except Exception:  # noqa: BLE001 — never block a launch on a param read
            fc_capacity = -1.0
        if fc_capacity <= 0:
            state.record_anomaly(
                "BAT1_CAPACITY unset on the FC — battery percentage and the "
                "energy budget are estimates until the power module is calibrated")
        elif abs(fc_capacity - energy_policy.capacity_mah) > 100.0:
            state.record_anomaly(
                f"battery capacity mismatch: FC {fc_capacity:.0f} mAh vs config "
                f"{energy_policy.capacity_mah:.0f} mAh")

        # ── safety watchdog ──
        watchdog = SafetyWatchdog(
            state, commander, controlled_airspace=cfg.get("controlled_airspace", []),
            rth_battery_pct=profile.rth_battery_pct,
            land_battery_pct=profile.land_battery_pct,
            min_time_remaining_for_continue_s=profile.min_time_remaining_s,
            datalink_loss_threshold_s=profile.datalink_loss_threshold_s,
            gps_loss_threshold_s=profile.gps_loss_threshold_s,
            telemetry_stale_threshold_s=profile.telemetry_stale_threshold_s,
            geofence_margin_m=profile.geofence_margin_m,
            no_fly_zones=cfg.get("no_fly_zones", []),
            altitude_ceiling_m=profile.altitude_ceiling_m,
            search_floor_m=profile.search_floor_m,
            home_lat=home.lat, home_lon=home.lon,
            on_rth=lambda: state.record_audit("watchdog: RTH triggered"),
            on_abort=lambda: state.record_audit("watchdog: ABORT triggered"),
            # In TUNING mode the tool flies the drone itself (chirp sweeps /
            # autotune) with no mission window, so the geofence + time-budget RTH
            # would abort every sweep. Battery / datalink / GPS / telemetry-stale
            # safety stays active either way.
            enforce_mission_limits=(args.mode != "tuning"),
        )
        await watchdog.start()

        # ── TUNING mode: System-ID + Autotune ONLY — a separate program from the
        # flight mission. No vision, no pre-flight gate, no mission ever runs; the
        # drone stays idle until the operator drives a chirp sweep / autotune from
        # the Tuning UI. The watchdog stays up for core safety (battery / datalink /
        # GPS / telemetry-stale) but with mission limits OFF (no geofence/time RTH). ──
        if args.mode == "tuning":
            logger.info("[main] TUNING mode — serving System-ID/Autotune; the mission will NOT run")
            try:
                await asyncio.Event().wait()      # idle until interrupted (launcher teardown)
            finally:
                for label, stop in (("watchdog", watchdog.stop),
                                    ("raw_telem", raw_telem.stop), ("telem", telem.stop)):
                    try:
                        await stop()
                    except Exception:
                        logger.exception(f"[main] {label}.stop() failed")
            return 0

        # ── vision worker → target tracker (+ dashboard feed) ──
        vc = cfg.get("vision", {}) or {}
        frame_max_age_s = float(vc.get("frame_max_age_s", DEFAULT_FRAME_MAX_AGE_S))
        vision = VisionWorker(state, target_description=profile.default_target,
                              frame_max_age_s=frame_max_age_s)
        vision.on_fix(tracker.ingest)     # discovery: confirm targets from fixes
        if dash is not None and getattr(dash, "broadcaster", None) is not None:
            b = dash.broadcaster
            if hasattr(b, "record_vision"):
                vision.on_observation(b.record_vision)
            if hasattr(b, "record_detected_objects"):
                # Live map pins = tracker CONFIRMATIONS, pushed the moment each
                # target is confirmed mid-sweep (one event per target) — NOT the
                # raw per-frame detections, which flickered all over the map.
                # Registered AFTER tracker.ingest so the fix is clustered first.
                vision.on_fix(_confirm_pusher(tracker, b))
        await vision.start()

        # ── mission frame recorder (R2: the RECORD half of "record and transmit") ──
        rc = cfg.get("recording", {}) or {}
        frames = FrameRecorder(
            state, run_dir / "frames",
            nadir=vision.nadir_frame,
            hz=float(rc.get("hz", 1.0)),
            jpeg_quality=int(rc.get("jpeg_quality", 80)),
            enabled=bool(rc.get("enabled", True)),
        )
        await frames.start()

        # ── fly (the per-sortie gate holds in PREFLIGHT before every launch) ──
        align = _align_for(profile, frame_max_age_s)
        policy = _build_time_policy(sc, profile)
        sortie_gate = _sortie_gate_factory(
            state, dash=dash, home=home, geofence=geofence, cfg=cfg,
            profile=profile, policy=policy, energy_policy=energy_policy,
            tracker=tracker,
            skip_preflight=args.skip_preflight,
        )
        on_plan_update = _plan_pusher(dash)
        try:
            await run_delivery_mission(
                commander, state, tracker, spec,
                home=home, transit_route=transit_route, sortie_gate=sortie_gate,
                profile=profile, align=align, policy=policy,
                max_pads=int(sc.get("max_pads", 6)),
                decode_dwell_s=float(sc.get("decode_dwell_s", 4.0)),
                on_drop_prediction=on_drop_prediction,
                on_plan_update=on_plan_update,
                refresh_energy=lambda: _evaluate_energy(state, energy_policy),
            )
        except Exception:
            # An unhandled error mid-mission must NOT just kill the process and
            # leave the aircraft airborne with the safety watchdog cancelled.
            # Command an orderly RTH (LAND as a last resort) while the watchdog
            # is still up, then fall through to the cleanup below.
            logger.exception("[main] mission loop crashed — commanding emergency RTH")
            state.record_anomaly("mission_loop_exception")
            state.set_terminal(TerminalState.FAILED, MissionPhase.RTH)
            try:
                await commander.rth()
            except Exception:
                logger.exception("[main] emergency RTH failed — last-resort LAND")
                try:
                    await commander.land()
                except Exception:
                    logger.exception(
                        "[main] LAND failed too — FC failsafes are the only net left"
                    )
        finally:
            # Tear down supervision + telemetry on EVERY path (success or the
            # emergency RTH above) — previously these only ran on the happy path.
            for label, stop in (
                ("frames", frames.stop),
                ("vision", vision.stop),
                ("watchdog", watchdog.stop),
                ("raw_telem", raw_telem.stop),
                ("telem", telem.stop),
            ):
                try:
                    await stop()
                except Exception:
                    logger.exception(f"[main] {label}.stop() failed")

        # ── SITL-only audit: discovered vs ground truth (NEVER used for planning) ──
        if args.truth_json:
            truth = audit.read_truth_targets(args.truth_json)
            if truth:
                comp = audit.compare_with_truth(tracker.snapshot(), truth)
                for line in comp.lines:
                    logger.info(f"[audit] {line}")
                    state.record_audit(f"AUDIT {line}")

        logger.info(f"[main] mission terminal={state.terminal.value}")
        return 0 if state.terminal in (
            TerminalState.COMPLETED, TerminalState.LANDED_RTH) else 1
    finally:
        if dash is not None:
            try:
                # Bound the teardown so a wedged dashboard shutdown can never hang
                # the process after a completed mission (post-mission hang, 2026-06).
                await asyncio.wait_for(dash.stop(), timeout=10.0)
            except Exception:
                pass
        # Kill the embedded mavsdk_server: its non-daemon stdout-reader thread
        # otherwise blocks interpreter shutdown, hanging the process after a
        # completed mission (post-mission hang root cause, see commander.close()).
        try:
            commander.close()
        except Exception:
            pass


def _rungs_for(profile: Any) -> tuple[float, ...]:
    """Descend rungs clamped under the ceiling, ending near the ground (the
    final 2 m rung tightens the land-ON drift for the 1 m pad).

    Low-ceiling profiles (KMUTNB sky-field, ceiling 5 m) get their own short
    ladder: the legacy filter would emit ``(5, 5, 3, 2, 1.5)`` — a duplicated
    top rung AND mis-indexed per-rung tolerances/speeds (AlignParams defaults
    are positional, sized for the 6-rung KMITL ladder)."""
    ceil = profile.altitude_ceiling_m
    if ceil <= 6.0:
        return (min(4.0, ceil - 1.0), 3.0, 2.0, 1.5)
    top = min(12.0, ceil)
    rungs = tuple(r for r in (top, 8.0, 5.0, 3.0, 2.0, 1.5) if r <= ceil)
    return rungs or (min(5.0, ceil),)


def _align_for(profile: Any, frame_max_age_s: float) -> AlignParams:
    """AlignParams matched to the profile's altitude band.

    KMUTNB (ceiling <= 6 m): 4-rung ladder with positionally-matched
    tolerances/descent speeds, and accept_radius_m tightened 15 -> 5 m — the
    default assumes >= 25 m pad separation, but the sky-field baseline packs
    pads 14.5 m apart, so 15 m would accept a NEIGHBOURING pad as the target.
    High ceilings keep the AlignParams defaults (the validated KMITL set)."""
    rungs = _rungs_for(profile)
    if profile.altitude_ceiling_m <= 6.0:
        return AlignParams(
            rungs=rungs,
            rung_tol_m=(1.0, 0.6, 0.35, 0.2),
            rung_descent_mps=(1.5, 0.8, 0.5, 0.4),
            accept_radius_m=5.0,
            frame_max_age_s=frame_max_age_s,
        )
    return AlignParams(rungs=rungs, frame_max_age_s=frame_max_age_s)


def _build_spec(area: list[tuple[float, ...]], home: Coordinate,
                sc: dict[str, Any], ceiling_m: float) -> Any:
    """Boustrophedon sweep of the SEARCH AREA polygon + the `search:` config."""
    axis = sc.get("sweep_axis_deg")
    return build_search_pattern(
        area, home,
        sweep_alt_m=float(sc.get("sweep_alt_m", 12.0)),
        overlap_frac=float(sc.get("overlap_frac", 0.4)),
        margin_m=float(sc.get("margin_m", 5.0)),
        speed_mps=float(sc.get("speed_mps", 10.0)),
        ceiling_m=ceiling_m,
        axis_deg=float(axis) if axis is not None else None,
    )


def _build_tracker(sc: dict[str, Any]) -> TargetTracker:
    """Pad registry from the `search:` config (confirm votes, gates, clusters)."""
    band = sc.get("radius_band", [0.4, 2.5])
    return TargetTracker(
        cluster_radius_m=float(sc.get("cluster_radius_m", 8.0)),
        confirm_votes=int(sc.get("confirm_votes", 3)),
        min_confidence=float(sc.get("min_confidence", 0.5)),
        min_span_s=float(sc.get("min_span_s", 0.6)),
        target_radius_m=float(sc.get("target_radius_m", 0.2)),
        radius_band=(float(band[0]), float(band[1])),
        max_fix_ground_dist_m=float(sc.get("max_fix_ground_dist_m", 50.0)),
        serve_dedupe_m=float(sc.get("serve_dedupe_m", 12.0)),
    )


def _confirm_pusher(tracker: TargetTracker, broadcaster: Any) -> Any:
    """Closure that pushes each newly CONFIRMED target to the dashboard map the
    moment the tracker confirms it during the sweep (display-only — WHEN to
    serve stays the mission loop's decision). Runs on the VisionWorker thread;
    ``record_detected_objects`` is the broadcaster's thread-safe entry for that.
    """
    from dashboard.payloads import DetectedObjectEvent  # dashboard already up

    pushed: set[int] = set()
    show = (TargetState.CONFIRMED, TargetState.SERVING, TargetState.SERVED)

    def _push(_fix: Any) -> None:
        events = []
        for t in tracker.snapshot():
            if t.target_id in pushed or t.state not in show:
                continue
            pushed.add(t.target_id)
            events.append(DetectedObjectEvent(
                t_monotonic=time.monotonic(),
                label=(f"aruco pad {t.marker_id}" if t.marker_id is not None
                       else f"pad? #{t.target_id}"),
                clothing_color="unknown",
                member_count=t.votes_nadir,
                pose="confirmed",
                confidence=t.best_confidence,
                lat=t.lat, lon=t.lon,
                is_designated_match=t.marker_id is not None,
            ))
        if events:
            try:
                broadcaster.record_detected_objects(events)
            except Exception:
                logger.exception("[main] confirmed-target push failed (non-fatal)")

    return _push


def _plan_pusher(dash: Any) -> Any:
    """A closure that pushes a rebuilt live plan to the dashboard, or None when
    headless / the broadcaster can't take it."""
    b = getattr(dash, "broadcaster", None) if dash is not None else None
    if b is None or not hasattr(b, "push_plan"):
        return None

    def _push(plan: Any, pointer: int) -> None:
        try:
            b.push_plan(json.loads(plan.model_dump_json()), pointer)
        except Exception:
            logger.exception("[main] plan push failed")

    return _push


def main() -> None:
    p = argparse.ArgumentParser(
        description="AAVC 2026 V1.3 multi-sortie egg-delivery orchestrator")
    p.add_argument("--config", default="sitl/aavc_config.yaml",
                   help="field config (geofence, search area, transit route, marker)")
    p.add_argument("--truth-json", default=None,
                   help="OPTIONAL ground-truth pads JSON (SITL spawn_targets) — used "
                        "ONLY for a post-flight discovered-vs-truth audit, never for planning")
    p.add_argument("--connect", default=None,
                   help="MAVLink endpoint (default udpin://0.0.0.0:14540)")
    p.add_argument("--assigned-ids", default=None,
                   help="seed the ordered mission-id queue: comma-separated marker "
                        "ids, one per sortie (e.g. '3,1,4,6'); overrides config "
                        "mission.assigned_marker_ids. Headless runs consume the "
                        "queue directly; with a dashboard the operator can set it "
                        "(POST /api/cmd/mission_ids) or override any sortie with a "
                        "manual pick at GO")
    # KMUTNB repo default: the sky-field practice profile (5 m ceiling). The
    # KMITL-rules envelope is one flag away: --profile competition.
    p.add_argument("--profile",
                   default=os.environ.get("AAVC_PROFILE", "kmutnb_skyfield"))
    p.add_argument("--no-dashboard", action="store_true")
    p.add_argument("--mode", choices=("mission", "tuning"), default="mission",
                   help="mission = fly the delivery sorties (default); "
                        "tuning = System-ID/Autotune only, no mission (separate program)")
    p.add_argument("--skip-preflight", action="store_true",
                   help="bypass the pre-flight readiness gate (bench/debug only)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    try:
        code = asyncio.run(run(args))
    except KeyboardInterrupt:
        # Operator Ctrl-C: asyncio.run has already cancelled the mission task and
        # run the teardown finally (telemetry/vision stop → dash.stop →
        # commander.close kills mavsdk_server) before re-raising. Swallow the
        # traceback and exit cleanly. (Foreground only — a backgrounded run gets
        # SIGINT=SIG_IGN from shell job control, so stop it with SIGTERM/SIGKILL.)
        logger.warning("[main] interrupted by operator (Ctrl-C) — exited")
        code = 130
    raise SystemExit(code)


if __name__ == "__main__":
    main()
