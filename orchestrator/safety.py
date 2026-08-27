"""Safety watchdog — runs as a background task, checks invariants every tick,
and triggers RTH / ABORT when a threshold is breached.

Safety triggers (in order of severity):
  1. No GPS 3D fix sustained > gps_loss_threshold_s in-flight → LAND in place
  2. Battery < 30% → RTH
  3. Battery < 20% → LAND in place (insufficient for RTH)
  4. Geofence proximity < 5 m → warn
  5. Geofence breach → RTH back inside
  6. No-fly-zone entry → RTH (V1.3: strictly prohibited)
  7. Altitude ceiling: > ceiling+warn → anomaly; > ceiling+breach sustained → RTH
     (RTL_RETURN_ALT is pinned to the ceiling so the RTH itself stays legal)
  8. Search-floor advisory: < floor while in SEARCH/TRANSIT (V1.3: below 10 m
     only over the pad) → anomaly, never a flight action
  9. Datalink RSSI critically low for > 5 s → emergency egress
 10. Time remaining < 3 min and not yet at egress → RTH unconditionally
 11. Telemetry age > 2 s → suspect link issue

Above all of these sit the takeover detectors (checked FIRST, armed or not,
fire-once): a sustained RC pilot mode, or a DISARM in a phase that is armed
by design → the orchestrator stands down COMPLETELY (terminal set directly +
DroneCommander.stand_down(); no companion command may fight the pilot).

(A twelfth check — one rotor drawing no current while the others are loaded —
lived here on 2026-08-16/17. It read per-motor ESC current, which this airframe
does not have: the ESCs are PWM-only with no telemetry lead, so the check could
only ever fail open. Removed rather than left as machinery that cannot fire.)
"""

from __future__ import annotations

import asyncio
import math
from collections.abc import Awaitable, Callable
from typing import Any

from loguru import logger

from mavlink_adapter.commands import DroneCommander
from mission_brain.schemas import MissionPhase

from .constants import CEILING_BREACH_M, CEILING_WARN_M
from .state import OrchestratorState, TerminalState

# The terrain-clearance (3D-geofence) hook is opt-in and dormant in this
# competition build (flat field, no DEM). ``DemGrid`` was part of the dropped
# ``mapping`` package — alias it to ``Any`` so the dem=None code path keeps
# type-checking without depending on a module we removed.
DemGrid = Any

_R_EARTH_M = 6_378_137.0

# RC modes that mean the SAFETY PILOT has taken the aircraft (RC-GO conops,
# 2026-08-12): the moment one is sustained the orchestrator must stand down —
# a re-sent GOTO would yank the vehicle straight back into AUTO mid-rescue.
# AUTO modes (HOLD/RTL/LAND/…) are absent ON PURPOSE: those can be the
# watchdog's or the mission's own doing and must not read as a takeover.
_PILOT_MODES = frozenset(
    {"MANUAL", "ALTCTL", "POSCTL", "ACRO", "STABILIZED", "RATTITUDE"})

# AUTO modes the FC enters BY ITSELF when one of its failsafes fires (geofence,
# link loss, low battery). Legitimate when the mission or this watchdog asked
# for them — `DroneCommander.expected_mode` / `_terminal_action` say so — and
# an FC failsafe otherwise. 2026-08-26 (ULog 09_24_10): PX4's vertical fence
# RTL'd the aircraft, the mission kept flying its sweep through the RTL, and
# the next goto went out 0.5 m before touchdown — PX4 obeyed and climbed back.
_FC_FAILSAFE_MODES = frozenset({"RETURN_TO_LAUNCH", "LAND"})

# Phases in which the vehicle is ARMED BY DESIGN for their whole duration —
# the multi-flight conops keeps it armed on a pad between deliveries
# (COM_DISARM_LAND=-1; re-arming over the field is forbidden). A DISARM seen
# in one of these means the pilot (or an FC failsafe) killed the flight under
# the mission loop. Deliberately absent: TAKEOFF (the loop arms DURING that
# phase, and state.phase BOOTS as TAKEOFF before any gate sets PREFLIGHT),
# PREFLIGHT/LAND (disarm there is the multi-flight design working), and
# RTH/ABORT (the watchdog's own terminal actions end in a disarm).
_ARMED_PHASES = frozenset({
    MissionPhase.TRANSIT_INGRESS, MissionPhase.SEARCH, MissionPhase.LOCALIZE,
    MissionPhase.DROP, MissionPhase.TRACK, MissionPhase.TRANSIT_EGRESS,
})


def _dem_lat_lon_to_en(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Flat-earth ENU offset (m) in the DEM's frame — mirrors mapping.lidar3d so DEM
    lookups co-register with the cloud/map (NOT the 111_320 constant used elsewhere)."""
    north_m = math.radians(lat - lat0) * _R_EARTH_M
    east_m = math.radians(lon - lon0) * _R_EARTH_M * math.cos(math.radians(lat0))
    return east_m, north_m


def _point_in_polygon(lat: float, lon: float, polygon: list[list[float]]) -> bool:
    """Ray-cast point-in-polygon test for [(lat, lon), ...] vertices."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if (yi > lat) != (yj > lat):
            x_intersect = (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi
            if lon < x_intersect:
                inside = not inside
        j = i
    return inside


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres (good enough for sub-km AAVC field)."""
    r = 6_378_137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _distance_to_polygon_edge(lat: float, lon: float, polygon: list[list[float]]) -> float:
    """Approximate minimum distance (metres) from (lat, lon) to the nearest
    polygon edge. Flat-earth ENU projection around the query point + 2D
    point-to-segment distance — good enough for the sub-km AAVC field. Used for
    the geofence proximity warning (vehicle inside but close to the boundary)."""
    if len(polygon) < 2:
        return float("inf")
    mlat = 111_320.0                       # metres per degree latitude
    mlon = 111_320.0 * math.cos(math.radians(lat))   # metres per degree longitude
    # Project each vertex to local metres relative to (lat, lon) at the origin.
    pts = [((v[1] - lon) * mlon, (v[0] - lat) * mlat) for v in polygon]
    best = float("inf")
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 == 0.0:
            d = math.hypot(ax, ay)
        else:
            tparam = max(0.0, min(1.0, -(ax * abx + ay * aby) / ab2))
            cx, cy = ax + tparam * abx, ay + tparam * aby
            d = math.hypot(cx, cy)
        best = min(best, d)
    return best


class SafetyWatchdog:
    """Periodically checks state and triggers safety actions via the commander."""

    def __init__(
        self,
        state: OrchestratorState,
        commander: DroneCommander,
        controlled_airspace: list[list[float]],
        check_interval_s: float = 0.5,
        rth_battery_pct: float = 30.0,
        land_battery_pct: float = 20.0,
        min_time_remaining_for_continue_s: float = 180.0,
        datalink_loss_threshold_s: float = 5.0,
        gps_loss_threshold_s: float = 5.0,
        battery_nan_threshold_s: float = 10.0,
        battery_sustain_s: float = 5.0,
        telemetry_stale_threshold_s: float = 10.0,
        pilot_takeover_threshold_s: float = 1.0,
        geofence_margin_m: float = 5.0,
        no_fly_zones: list[list[list[float]]] | None = None,
        altitude_ceiling_m: float = 20.0,
        ceiling_warn_m: float = CEILING_WARN_M,
        ceiling_breach_m: float = CEILING_BREACH_M,
        ceiling_breach_threshold_s: float = 3.0,
        search_floor_m: float = 10.0,
        dem: DemGrid | None = None,
        home_lat: float | None = None,
        home_lon: float | None = None,
        terrain_min_clearance_m: float = 5.0,
        terrain_warn_clearance_m: float = 10.0,
        terrain_breach_threshold_s: float = 2.0,
        on_rth: Callable[[], None] | None = None,
        on_abort: Callable[[], None] | None = None,
    ) -> None:
        self.state = state
        self.commander = commander
        self.controlled_airspace = controlled_airspace
        self.check_interval_s = check_interval_s
        self.rth_battery_pct = rth_battery_pct
        self.land_battery_pct = land_battery_pct
        self.min_time_remaining_s = min_time_remaining_for_continue_s
        self.datalink_loss_threshold_s = datalink_loss_threshold_s
        self.gps_loss_threshold_s = gps_loss_threshold_s
        self.battery_nan_threshold_s = battery_nan_threshold_s
        self.battery_sustain_s = battery_sustain_s
        self.telemetry_stale_threshold_s = telemetry_stale_threshold_s
        self.pilot_takeover_threshold_s = pilot_takeover_threshold_s
        self.geofence_margin_m = geofence_margin_m
        # V1.3 airspace rules: no-fly polygons (entry prohibited), the 20 m
        # ceiling (transit flies AT it, so warn has headroom above), and the
        # 10 m search floor (advisory — the delivery descent is the only legal
        # sub-floor flight and runs in LOCALIZE/DROP/LAND phases).
        self.no_fly_zones = [list(z) for z in (no_fly_zones or [])]
        self.altitude_ceiling_m = altitude_ceiling_m
        self.ceiling_warn_m = ceiling_warn_m
        self.ceiling_breach_m = ceiling_breach_m
        self.ceiling_breach_threshold_s = ceiling_breach_threshold_s
        self.search_floor_m = search_floor_m
        self._ceiling_breach_since: float | None = None
        # (An enforce_mission_limits switch used to be able to turn the geofence,
        # no-fly, ceiling and time checks OFF for the System-ID/Autotune tool.
        # That tool was deleted 2026-08-15 and nothing set the flag afterwards,
        # so the whole disable path went with it on 2026-08-17. These checks are
        # not optional any more, which is the state they should always have been
        # in for a mission build.)
        # Terrain-clearance (3D geofence) — opt-in: active ONLY when a DEM is loaded
        # (AAVC_TERRAIN_MAP). dem=None → the check below is skipped → behavior unchanged.
        self.dem = dem
        self.terrain_min_clearance_m = terrain_min_clearance_m
        self.terrain_warn_clearance_m = terrain_warn_clearance_m
        self.terrain_breach_threshold_s = terrain_breach_threshold_s
        self._terrain_breach_since: float | None = None
        self._terrain_home_h = 0.0
        if dem is not None:
            he, hn = _dem_lat_lon_to_en(
                home_lat if home_lat is not None else dem.lat0,
                home_lon if home_lon is not None else dem.lon0,
                dem.lat0, dem.lon0,
            )
            hh = dem.elevation_at(he, hn)
            self._terrain_home_h = 0.0 if math.isnan(hh) else float(hh)
        self.on_rth = on_rth
        self.on_abort = on_abort
        self._task: asyncio.Task[None] | None = None
        self._datalink_lost_since: float | None = None
        self._gps_lost_since: float | None = None
        self._battery_nan_since: float | None = None
        self._battery_land_since: float | None = None
        self._battery_rth_since: float | None = None
        self._telemetry_stale_since: float | None = None
        self._manual_mode_since: float | None = None
        # Fire-once latch for the takeover/disarm detectors: while a prior
        # terminal action is still settling, _run keeps ticking — without the
        # latch the fire would repeat every 0.5 s (record_audit does NOT
        # dedupe) and re-overwrite the terminal during the pilot's landing.
        self._takeover_fired = False
        # Terminal action in progress: None | "rth" | "abort". The watchdog
        # keeps checking DURING an in-progress RTH (run as a background task,
        # not awaited inline) so a worsening condition can escalate to LAND.
        self._terminal_action: str | None = None
        self._action_tasks: set[asyncio.Task[None]] = set()

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run())

    async def stop(self, action_timeout_s: float = 200.0) -> None:
        """Stop the tick loop, then WAIT for any terminal action still flying.

        ⚠ The wait is the whole point (2026-08-21 review). ``_trigger_rth`` /
        ``_trigger_abort`` dispatch ``commander.rth()`` / ``land()`` as
        background tasks, and those coroutines are what finally send the
        explicit ``disarm()`` — PX4 will not auto-disarm on its own because
        ``COM_DISARM_LAND=-1`` is pinned for the mid-flight pad landings. The
        mission loop exits within ~2 s of the terminal flipping, so without
        this wait ``main``'s teardown reached ``commander.close()`` (which
        kills mavsdk_server) while the RTL was still descending: PX4 completed
        the landing, nothing ever disarmed, and the aircraft sat on the ground
        ARMED with the companion dead and the console already showing DONE.

        The timeout is a backstop above ``rth()``'s own 180 s landing wait —
        this must never be the thing that hangs a shutdown.
        """
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        pending = [t for t in self._action_tasks if not t.done()]
        if pending:
            logger.info(
                f"[safety] waiting for {len(pending)} terminal action(s) to "
                f"finish before teardown (the disarm rides on them)")
            try:
                await asyncio.wait_for(
                    asyncio.gather(*pending, return_exceptions=True),
                    timeout=action_timeout_s)
            except asyncio.TimeoutError:
                logger.error(
                    f"[safety] terminal action still running after "
                    f"{action_timeout_s:.0f}s — tearing down anyway; CHECK THE "
                    "AIRCRAFT IS DISARMED")
                for t in pending:
                    t.cancel()
                await asyncio.gather(*pending, return_exceptions=True)

    async def _run(self) -> None:
        while True:
            try:
                await self._check_once()
            except Exception as e:
                logger.exception(f"[safety] check failed: {e}")
            # Exit once the mission is terminal AND it has settled. A terminal
            # set OUTSIDE the watchdog (mission complete / tactical RTH) settles
            # immediately; a watchdog-triggered RTH/ABORT settles when its
            # background action finishes (vehicle disarmed) — until then keep
            # checking so a worsening condition can escalate (RTH → LAND).
            if self.state.terminal != TerminalState.RUNNING and self._terminal_settled():
                break
            await asyncio.sleep(self.check_interval_s)

    def _terminal_settled(self) -> bool:
        if self._terminal_action is None:
            return True            # terminal set externally — nothing to wait on
        return all(t.done() for t in self._action_tasks)

    async def _check_once(self) -> None:
        st = self.state
        t = st.telemetry

        if self._takeover_fired:
            # Stood down: the pilot owns the aircraft. No further checks and
            # no further triggers — a post-takeover battery/geofence tick
            # would otherwise overwrite the PILOT_TAKEOVER terminal and
            # dispatch a command against a pilot-owned aircraft. _run exits
            # once any in-flight terminal action settles.
            return

        # 1. Pilot takeover + flight-phase disarm (RC-GO conops 2026-08-12;
        # moved above the armed gate 2026-08-21, and above the
        # telemetry-stale check 2026-08-22 — the docstring said these were
        # checked FIRST and they were not: a stale-but-connected armed link
        # escalates with `await self._trigger_rth(); return`, and on every
        # later tick _trigger_rth short-circuits and returns again, so the
        # takeover detectors were UNREACHABLE for as long as the condition
        # held.) The field takeover is
        # "flip POSCTL, then DISARM within ~0.5 s" — faster than the 1.0 s
        # debounce at a 0.5 s tick — and this block used to sit BELOW the
        # armed gate, so the disarm made the watchdog permanently blind the
        # moment it mattered: stand_down() was never called and the mission
        # loop later re-armed the parked aircraft (G7 attempt-1 zombie,
        # ULog-measured 0.46-0.48 s of ARMED+POSCTL in both incidents).
        #
        # D2 — a DISARM in a phase that is armed by design (_ARMED_PHASES):
        # definitive, fires immediately. Checked FIRST: when both conditions
        # hold on the same tick the disarm is the harder evidence.
        if (not t.is_armed) and st.phase in _ARMED_PHASES:
            self._fire_takeover(f"(disarmed in {st.phase.value})",
                                "disarm_in_flight_phase")
            return

        # D1 — a sustained pilot flight mode. While ARMED: any phase except
        # PREFLIGHT (the RC-GO hold has the pilot arming in POSCTL on the
        # ground BEFORE the OFFBOARD flip). While DISARMED: only in
        # _ARMED_PHASES — at boot state.phase still holds its TAKEOFF default
        # while RC transmitters sit in POSCTL, and between flights the pilot
        # legitimately re-stages in POSCTL during the LAND→PREFLIGHT gap;
        # neither is a takeover. After the field's quick disarm the mode
        # REMAINS POSCTL, so the debounce completes ~1 s after the flip even
        # though the vehicle is no longer armed. Debounced so a transient
        # mode blip during PX4's own transitions can't spuriously end the
        # mission.
        if t.is_armed:
            pilot_mode_watched = st.phase != MissionPhase.PREFLIGHT
        else:
            pilot_mode_watched = st.phase in _ARMED_PHASES
        if pilot_mode_watched and t.flight_mode in _PILOT_MODES:
            now = asyncio.get_running_loop().time()
            if self._manual_mode_since is None:
                self._manual_mode_since = now
            elif now - self._manual_mode_since >= self.pilot_takeover_threshold_s:
                self._fire_takeover(f"mode={t.flight_mode}",
                                    f"pilot_takeover_{t.flight_mode.lower()}")
                return
        else:
            self._manual_mode_since = None

        # D3 — the FC entered RTL/LAND on its own (2026-08-26). Only in the
        # armed-by-design phases (the mission's own final LAND has its phase,
        # and land-on-pad / a watchdog RTH announce themselves through
        # expected_mode / _terminal_action). NOT debounced (2026-08-27): PX4
        # never enters RTL/LAND by itself as a transient, and the 1 s wait
        # copied from D1 was exactly the window in which the 14:13 flight's
        # next sweep goto went out 0.12 s before the stand-down — PX4 obeyed
        # it, left the pilot's LAND for HOLD, and the pilot had to take the
        # sticks (POSCTL) before a second LAND held.
        expected = getattr(self.commander, "expected_mode", None)
        if (t.is_armed and st.phase in _ARMED_PHASES
                and t.flight_mode in _FC_FAILSAFE_MODES
                and self._terminal_action is None
                and expected != t.flight_mode):
            self._stand_down(
                f"mode={t.flight_mode}", f"fc_failsafe_{t.flight_mode.lower()}",
                terminal=TerminalState.FC_FAILSAFE, headline="FC FAILSAFE")
            return

        # 2. Telemetry stale — record always; if it stays stale while ARMED,
        # escalate to RTH (companion-side backstop). PX4's NAV_DLL_ACT (set at
        # startup) is the FC-level link-loss failsafe; this catches a
        # degraded-but-connected link. Debounced + armed-gated so a brief
        # telemetry hiccup (or a pre-arm boot gap) never spuriously RTHs.
        if t.age_s() > 2.0 and t.is_connected:
            st.record_anomaly(f"telemetry_stale ({t.age_s():.1f}s)",
                              dedupe_key="telemetry_stale")
            if t.is_armed:
                now = asyncio.get_running_loop().time()
                if self._telemetry_stale_since is None:
                    self._telemetry_stale_since = now
                elif now - self._telemetry_stale_since > self.telemetry_stale_threshold_s:
                    logger.critical(
                        f"[safety] telemetry stale >{self.telemetry_stale_threshold_s:.0f}s "
                        "while armed — triggering RTH"
                    )
                    st.record_anomaly("telemetry_stale_sustained")
                    await self._trigger_rth()
                    return
            else:
                self._telemetry_stale_since = None
        else:
            self._telemetry_stale_since = None

        if not t.is_armed:
            # Pre-takeoff or post-landing — nothing FURTHER to check (the
            # takeover/disarm detectors above already ran; everything below
            # assumes powered flight). ⚠ This gate sat ABOVE the takeover
            # check until 2026-08-21, which is exactly what blinded it.
            return

        # 3. Battery — both thresholds require the reading to STAY down.
        #
        # A single sample used to be enough, which was safe only while the pack
        # had current sensing. It no longer does (2026-08-16: the PM03D failed;
        # the motors moved to a board the FC cannot see — unchanged by the
        # PM02D installed 2026-08-20, which powers the FC alone), so PX4 falls back to a
        # purely voltage-derived gauge — and its load compensation is gated on
        # `current_a > FLT_EPSILON` (lib/battery/battery.cpp
        # `calculateStateOfChargeVoltageBased`), so with no current to correct
        # with, the reading SAGS every time the motors pull hard and springs back
        # after. On one sample that transient is indistinguishable from a flat
        # pack: a climb or a gust would fire LAND NOW in the middle of the field.
        # Requiring the condition to persist keeps a genuinely empty pack fully
        # protected (it never comes back up) while ignoring the sag.
        #
        # This costs at most `battery_sustain_s` of delay, against an FC-side
        # failsafe (BAT_*_THR / COM_LOW_BAT_ACT) that is still armed underneath
        # and has direct sensor access — so the aircraft is not unprotected in
        # the meantime. Credit: the parallel session hit this first.
        if not math.isnan(t.battery_percent):
            self._battery_nan_since = None
            now = asyncio.get_running_loop().time()

            if t.battery_percent < self.land_battery_pct:
                if self._battery_land_since is None:
                    self._battery_land_since = now
                    if self.battery_sustain_s > 0.0:
                        logger.warning(
                            f"[safety] battery {t.battery_percent:.0f}% below the "
                            f"LAND floor — confirming for {self.battery_sustain_s:.0f}s")
                if now - self._battery_land_since >= self.battery_sustain_s:
                    logger.critical(
                        f"[safety] battery {t.battery_percent:.0f}% sustained — LAND NOW")
                    st.record_anomaly(f"battery_critical_{t.battery_percent:.0f}%",
                                      dedupe_key="battery_critical")
                    await self._trigger_abort()
                    return
            else:
                self._battery_land_since = None

            if t.battery_percent < self.rth_battery_pct and st.phase not in (
                MissionPhase.TRANSIT_EGRESS, MissionPhase.LAND, MissionPhase.RTH
            ):
                if self._battery_rth_since is None:
                    self._battery_rth_since = now
                if now - self._battery_rth_since >= self.battery_sustain_s:
                    logger.warning(
                        f"[safety] battery {t.battery_percent:.0f}% sustained — "
                        "triggering RTH")
                    st.record_anomaly(f"battery_low_{t.battery_percent:.0f}%",
                                      dedupe_key="battery_low")
                    await self._trigger_rth()
                    return
            else:
                self._battery_rth_since = None
        else:
            # NaN battery (sensor dropout, or pre-first-frame at boot) silently
            # disables BOTH low-battery checks above. Debounce like GPS/datalink:
            # record it once, and if the NaN stream PERSISTS past the threshold,
            # escalate to a distinct anomaly (+ a loud WARNING) so the operator
            # knows battery protection is blind. Deliberately NOT an auto-RTH: a
            # NaN stream is indistinguishable from a telemetry-plumbing fault, and
            # grounding a healthy vehicle on a display bug is the wrong trade —
            # the FC's own battery failsafe (BAT_*_THR / COM_LOW_BAT_ACT) is the
            # authoritative layer with direct sensor access.
            now = asyncio.get_running_loop().time()
            if self._battery_nan_since is None:
                self._battery_nan_since = now
                st.record_anomaly("battery_telemetry_nan")
            elif now - self._battery_nan_since > self.battery_nan_threshold_s:
                logger.warning(
                    f"[safety] battery telemetry NaN for "
                    f">{self.battery_nan_threshold_s:.0f}s — companion battery "
                    "protection is BLIND; relying on the FC battery failsafe"
                )
                st.record_anomaly("battery_telemetry_nan_sustained")

        # 4. GPS health — a SUSTAINED loss of the 3D fix → LAND IN PLACE, not
        # RTH (operator 2026-08-17: "เพื่อ safe ลง land เลย"). The physics
        # agrees: with no flow module and no GPS the horizontal estimate dies
        # in seconds, and RTL is a command that cannot navigate — it would
        # drift wherever the wind takes it before PX4 degrades to a blind
        # descent anyway. PX4's LAND needs no horizontal position (baro +
        # TFmini still own the vertical), so commanding it immediately puts
        # the aircraft on the ground closest to where it was last known to
        # be, inside the fence. Debounced so a brief glitch — common outdoors
        # — doesn't ground the mission.
        if t.gps_fix_type < 3:
            now = asyncio.get_running_loop().time()
            if self._gps_lost_since is None:
                self._gps_lost_since = now
                st.record_anomaly(f"gps_unhealthy_fix={t.gps_fix_type}",
                                  dedupe_key="gps_unhealthy")
            elif now - self._gps_lost_since > self.gps_loss_threshold_s:
                logger.critical(
                    f"[safety] no GPS 3D fix for >{self.gps_loss_threshold_s:.0f}s "
                    "— LAND in place (RTH cannot navigate without a position)"
                )
                st.record_anomaly("gps_loss_sustained")
                await self._trigger_abort()
                return
        else:
            self._gps_lost_since = None

        # 4. Geofence
        if not math.isnan(t.lat) and not math.isnan(t.lon):
            if not _point_in_polygon(t.lat, t.lon, self.controlled_airspace):
                # FLIGHT-BEHAVIOR CHANGE (re-validate SITL G2/G4): a breach now
                # triggers RTH — return INSIDE the field, matching PX4
                # GF_ACTION=3 (Return) — instead of LAND-in-place, which left
                # the vehicle DOWN outside controlled airspace (a rules
                # violation + recovery hazard). See docs/flight_behavior_changes.md.
                # ⚠ This comment said GF_ACTION=2 until 2026-08-16, and so did
                # the code that set it — 2 is HOLD, not Return, so the FC half
                # of "return inside the field" was never actually in place and
                # this companion check was carrying the rule alone.
                logger.critical(
                    f"[safety] outside controlled airspace at "
                    f"({t.lat:.6f}, {t.lon:.6f}) — RTH back inside"
                )
                st.record_anomaly("geofence_breach")
                await self._trigger_rth()
                return
            # Proximity warning: inside, but within the margin of the boundary
            # (implements the docstring's "geofence proximity < margin → warn").
            edge_m = _distance_to_polygon_edge(t.lat, t.lon, self.controlled_airspace)
            if edge_m < self.geofence_margin_m:
                logger.warning(
                    f"[safety] {edge_m:.1f} m from geofence edge — proximity warning"
                )
                st.record_anomaly("geofence_proximity")

        # 4a. No-fly zones (mission-only) — entering one mid-mission is a
        # rules-violation-grade event: RTH out immediately (RTL_RETURN_ALT=20
        # keeps the return at the ceiling; the L&R side of the field is clear
        # of the published SE zone).
        if not math.isnan(t.lat) and not math.isnan(t.lon):
            for zi, zone in enumerate(self.no_fly_zones):
                if len(zone) >= 3 and _point_in_polygon(t.lat, t.lon, zone):
                    logger.critical(
                        f"[safety] INSIDE no-fly zone {zi} at "
                        f"({t.lat:.6f}, {t.lon:.6f}) — RTH out"
                    )
                    st.record_anomaly(f"no_fly_zone_breach_{zi}")
                    await self._trigger_rth()
                    return

        # 4c. Altitude band (mission-only). The ceiling is a hard rule (20 m);
        # transit flies AT it, and the tamed climb overshoot (~0.3-0.4 m at
        # MPC_Z_VEL_MAX_UP=2) stays under the warn line. A sustained breach
        # past ceiling+breach forces RTH. The search FLOOR is advisory-only:
        # the descent over the pad (LOCALIZE/DROP/LAND) is the rules' one
        # legal sub-floor flight, and TAKEOFF passes through it by definition.
        if not math.isnan(t.relative_alt_m):
            alt = t.relative_alt_m
            if alt > self.altitude_ceiling_m + self.ceiling_breach_m:
                now = asyncio.get_running_loop().time()
                if self._ceiling_breach_since is None:
                    self._ceiling_breach_since = now
                    st.record_anomaly(f"altitude_ceiling_breach_{alt:.1f}m",
                                      dedupe_key="altitude_ceiling_breach")
                elif now - self._ceiling_breach_since > self.ceiling_breach_threshold_s:
                    logger.critical(
                        f"[safety] {alt:.1f} m AGL > ceiling "
                        f"{self.altitude_ceiling_m:.0f} m sustained — RTH"
                    )
                    st.record_anomaly("altitude_ceiling_breach_sustained")
                    await self._trigger_rth()
                    return
            else:
                self._ceiling_breach_since = None
                if alt > self.altitude_ceiling_m + self.ceiling_warn_m:
                    st.record_anomaly(f"altitude_ceiling_warn_{alt:.1f}m",
                                      dedupe_key="altitude_ceiling_warn")
            # 0.5 m tolerance for the EKF/home altitude-frame drift (the same
            # headroom the mission commands + the verifier allows) — a hover
            # commanded just above the floor must not trip a phantom advisory.
            if (alt < self.search_floor_m - 0.5
                    and st.phase in (MissionPhase.SEARCH,
                                     MissionPhase.TRANSIT_INGRESS,
                                     MissionPhase.TRANSIT_EGRESS)):
                st.record_anomaly("below_search_floor")

        # 4b. Terrain clearance (3D geofence) — opt-in, active only when a DEM is loaded.
        # PERMISSIVE ON UNKNOWN: an unmapped (NaN) cell NEVER triggers RTH (the opposite of
        # the planner, which AVOIDS unknown) — grounding on every map-edge waypoint would be
        # worse than no check. Debounced like GPS/datalink so a single bad cell or a 1-tick
        # undershoot can't ground us. dem=None → skipped entirely (behavior unchanged).
        if self.dem is not None and not (
            math.isnan(t.lat) or math.isnan(t.lon) or math.isnan(t.relative_alt_m)
        ):
            te, tn = _dem_lat_lon_to_en(t.lat, t.lon, self.dem.lat0, self.dem.lon0)
            terrain_h = self.dem.elevation_at(te, tn)
            if math.isnan(terrain_h):
                st.record_anomaly("terrain_unknown")
                self._terrain_breach_since = None
            else:
                clearance = t.relative_alt_m - (terrain_h - self._terrain_home_h)
                if clearance < self.terrain_min_clearance_m:
                    now = asyncio.get_running_loop().time()
                    if self._terrain_breach_since is None:
                        self._terrain_breach_since = now
                    elif now - self._terrain_breach_since > self.terrain_breach_threshold_s:
                        logger.critical(
                            f"[safety] terrain clearance {clearance:.1f} m < "
                            f"{self.terrain_min_clearance_m:.0f} m sustained — triggering RTH"
                        )
                        st.record_anomaly(
                            f"terrain_clearance_breach_{clearance:.0f}m",
                            dedupe_key="terrain_clearance_breach")
                        await self._trigger_rth()
                        return
                else:
                    self._terrain_breach_since = None
                    if clearance < self.terrain_warn_clearance_m:
                        st.record_anomaly("terrain_proximity")

        # 5. Datalink / RC-link strength.
        # Field is populated from MAVSDK rc_status() — see telemetry._sub_rc_status.
        # < 50% sustained signal is treated as "datalink weakening".
        if t.datalink_rssi >= 0 and t.datalink_rssi < 50:
            now = asyncio.get_running_loop().time()
            if self._datalink_lost_since is None:
                self._datalink_lost_since = now
            elif now - self._datalink_lost_since > self.datalink_loss_threshold_s:
                logger.warning("[safety] sustained datalink loss — emergency egress")
                st.record_anomaly("datalink_loss_sustained")
                await self._trigger_rth()
                return
        else:
            self._datalink_lost_since = None

        # 6. Time budget
        #
        # DROP is exempt alongside TRANSIT_EGRESS/LAND/RTH (M1, review
        # 2026-07-24): tactical_align enters MissionPhase.DROP only AFTER the
        # landing is confirmed — the vehicle is on the ground with an egg
        # mid-release (or just released), and mission.py's post-delivery
        # climb-out + pre-egress goto also run under that same leftover DROP
        # phase before the next _phase(...) call retags it. RTH is never the
        # right response to "we are on the ground" — it would try to fly an
        # already-landed, possibly still-armed-with-an-open-hold aircraft.
        if (
            st.time_remaining_s() < self.min_time_remaining_s
            and st.phase not in (MissionPhase.TRANSIT_EGRESS, MissionPhase.LAND,
                                 MissionPhase.DROP, MissionPhase.RTH)
        ):
            logger.warning(
                f"[safety] only {st.time_remaining_s():.0f}s left — forcing RTH"
            )
            st.record_anomaly("time_budget_exhausted")
            await self._trigger_rth()

    def _fire_takeover(self, detail: str, anomaly_kind: str) -> None:
        """Stand the orchestrator down — ONCE. The audit line keeps the literal
        ``PILOT TAKEOVER`` (the GCS home-reason chip matches that needle). The
        terminal is set DIRECTLY (never via _trigger_rth/_trigger_abort): any
        companion command after this point would fight the pilot for the
        aircraft. The first fire may overwrite an in-progress LANDED_RTH —
        that is a pilot rescuing a watchdog RTL, and the aircraft is theirs;
        the ``_takeover_fired`` latch stops every later tick from re-firing
        (and from re-writing the audit trail, which does not dedupe)."""
        self._stand_down(detail, anomaly_kind,
                         terminal=TerminalState.PILOT_TAKEOVER,
                         headline="PILOT TAKEOVER")

    def _stand_down(self, detail: str, anomaly_kind: str, *,
                    terminal: TerminalState, headline: str) -> None:
        """The common stand-down: someone else (pilot or FC) owns the aircraft.
        ``headline`` is the literal the audit line and the GCS chip key on."""
        st = self.state
        self._takeover_fired = True
        logger.critical(
            f"[safety] {headline} {detail} — orchestrator standing down, "
            "no further commands")
        st.record_anomaly(anomaly_kind)
        st.record_audit(
            f"t={st.time_elapsed_s():.1f}s {headline} {detail} "
            "— orchestrator standing down")
        st.set_terminal(terminal)
        # Latch the command OWNER too. Setting the terminal stops the mission
        # loop between awaits, but a command already queued would still reach
        # the FC — stand_down() makes DroneCommander refuse every movement
        # command from here on (RESUME 2026-08-19 §2.2).
        self.commander.stand_down()

    def _spawn_action(self, coro: Awaitable[None], cb: Callable[[], None] | None) -> None:
        """Run a terminal action (rth/land) as a tracked background task so the
        watchdog loop keeps checking + can escalate during it. Strong-ref'd so
        the task can't be GC'd mid-flight; the callback fires on success."""
        async def _runner() -> None:
            try:
                await coro
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.exception(f"[safety] terminal action failed: {e}")
            else:
                if cb is not None:
                    cb()
        task = asyncio.create_task(_runner())
        self._action_tasks.add(task)
        task.add_done_callback(self._action_tasks.discard)

    async def _trigger_rth(self) -> None:
        # Idempotent: don't re-issue while a terminal action is already running.
        # Set terminal BEFORE dispatching so any in-flight _wait_near_waypoint
        # exits early (observed during G2 attempt 6 egress geofence breach).
        # Dispatched as a BACKGROUND task (not awaited inline) so the watchdog
        # keeps checking during the RTL and can escalate to LAND — see _run.
        if self._terminal_action is not None:
            return
        self.state.set_terminal(TerminalState.LANDED_RTH, MissionPhase.RTH)
        self._terminal_action = "rth"
        self._spawn_action(self.commander.rth(), self.on_rth)

    async def _trigger_abort(self) -> None:
        # Land in place NOW — orderly PX4 LAND, not motor-kill (commander.abort()
        # is the motor-kill, reserved for imminent crash). This ESCALATES over an
        # in-progress RTH: PX4 LAND overrides RTL, so we just command land and
        # let the prior RTH task settle on the same disarm.
        # FLIGHT-BEHAVIOR CHANGE (re-validate SITL G4): RTH can now escalate to
        # LAND (e.g. battery going critical mid-RTL); previously the watchdog
        # stopped checking the instant RTH was triggered.
        if self._terminal_action == "abort":
            return
        self.state.set_terminal(TerminalState.ABORTED, MissionPhase.ABORT)
        self._terminal_action = "abort"
        self._spawn_action(self.commander.land(), self.on_abort)
