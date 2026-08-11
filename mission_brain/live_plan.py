"""Live per-sortie mission plan for the V1.3 delivery mission.

Each sortie's plan is rebuilt as the flight progresses:

    Gate release:  TAKEOFF + transit P1..P3 (TRANSIT_INGRESS)
                   + the boustrophedon search GOTOs (only when the assigned pad
                     is not yet in the cross-sortie registry)
                   + transit P3..P1 (TRANSIT_EGRESS) + LAND at L&R
    Each serve:    the same, with a [GOTO(LOCALIZE) + DROP_PAYLOAD(DROP)] pair
                   INSERTED before the egress for every pad committed so far
                   (the ledger spans sorties, so the map shows every delivery).

Keeping the search legs and an append-only serve ledger (rather than
interleaving chronologically) gives the dashboard a stable history to paint and
a command pointer that always resolves. ``orchestrator.mission`` swaps
``state.plan`` after each gate release and after each serve, and pushes it to
the GCS.

The plan ends with an explicit LAND at the Launch & Recovery point — NOT an
RTL command: PX4 re-captures home at every arming, and the vehicle re-arms at
L&R between sorties, so the mission flies an explicit goto-home + land.

Pure: no telemetry, no I/O. The numbers it needs (sweep time, serve cost) come
in via the spec + profile so it stays unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass

from .profile import MissionProfile
from .schemas import (
    CommandKind,
    Coordinate,
    MissionCommand,
    MissionPhase,
    MissionPlan,
    active_airframe,
)
from .search_pattern import SearchPlanSpec

# Per-served-pad wall-clock estimate (s) used only for the plan's duration
# field (the live time budget lives in orchestrator.time_policy).
_SERVE_EST_S = 110.0


@dataclass(frozen=True)
class ServedStop:
    """A pad the mission has committed a delivery to (serving or served). The
    ledger of these — spanning every flight — renders the GOTO+DROP section of
    the live plan."""

    stop_index: int
    lat: float
    lon: float
    name: str = ""
    # Per-FLIGHT release-mechanism index (0..eggs_aboard-1) → servo channel
    # drop_servo_channel + payload_id. Distinct from stop_index, which is the
    # mission-global ledger key. Defaults to 0 (single-egg flights).
    payload_id: int = 0


def render_live_plan(
    home: Coordinate,
    spec: SearchPlanSpec,
    *,
    discovered: list[ServedStop],
    profile: MissionProfile,
    transit_route: list[Coordinate] | None = None,
    include_search: bool = True,
    sortie: int | None = None,
    mission_id: str = "aavc_delivery_mission",
) -> MissionPlan:
    """Render the current sortie + serve-ledger into a :class:`MissionPlan`.

    ``transit_route`` is the mandatory P1→P2→P3 corridor (flown at
    ``profile.transit_alt_m`` both ways; scored per point). ``include_search``
    is False for a registry-known sortie — the plan then goes straight from
    the ingress to the serve. The minimum shape (TAKEOFF + LAND + ≥2 more
    commands) always validates.
    """
    ceiling = profile.altitude_ceiling_m
    cruise = min(spec.sweep_alt_m, ceiling)
    transit_alt = min(profile.transit_alt_m, ceiling)
    route = list(transit_route or [])
    cmds: list[MissionCommand] = []
    seq = 0

    def _add(cmd: MissionCommand) -> None:
        nonlocal seq
        cmds.append(cmd)
        seq += 1

    _add(MissionCommand(
        seq=seq, kind=CommandKind.TAKEOFF, phase=MissionPhase.TAKEOFF,
        coord=Coordinate(lat=home.lat, lon=home.lon, alt_m=transit_alt),
        altitude_m=transit_alt,
        notes=f"sortie {sortie or '?'}: arm + takeoff to transit altitude",
    ))

    for n, p in enumerate(route, start=1):
        _add(MissionCommand(
            seq=seq, kind=CommandKind.GOTO, phase=MissionPhase.TRANSIT_INGRESS,
            coord=Coordinate(lat=p.lat, lon=p.lon, alt_m=transit_alt),
            altitude_m=transit_alt, speed_mps=min(spec.speed_mps, 20.0),
            notes=f"transit P{n} (ingress, strictly {transit_alt:.0f} m)",
        ))

    if include_search:
        for i, wp in enumerate(spec.waypoints):
            _add(MissionCommand(
                seq=seq, kind=CommandKind.GOTO, phase=MissionPhase.SEARCH,
                coord=Coordinate(lat=wp.lat, lon=wp.lon, alt_m=cruise),
                altitude_m=cruise, speed_mps=min(spec.speed_mps, 20.0),
                notes=f"search leg waypoint {i}",
            ))

    for d in discovered:
        _add(MissionCommand(
            seq=seq, kind=CommandKind.GOTO, phase=MissionPhase.LOCALIZE,
            coord=Coordinate(lat=d.lat, lon=d.lon, alt_m=cruise),
            altitude_m=cruise, stop_index=d.stop_index,
            notes=f"align over pad {d.name or d.stop_index}",
        ))
        _add(MissionCommand(
            seq=seq, kind=CommandKind.DROP_PAYLOAD, phase=MissionPhase.DROP,
            coord=Coordinate(lat=d.lat, lon=d.lon),
            payload_id=d.payload_id, stop_index=d.stop_index, confirmed=True,
            notes=f"land ON pad {d.name or d.stop_index} + release the egg "
                  "after touchdown",
        ))

    for n, p in zip(range(len(route), 0, -1), reversed(route), strict=True):
        _add(MissionCommand(
            seq=seq, kind=CommandKind.GOTO, phase=MissionPhase.TRANSIT_EGRESS,
            coord=Coordinate(lat=p.lat, lon=p.lon, alt_m=transit_alt),
            altitude_m=transit_alt, speed_mps=min(spec.speed_mps, 20.0),
            notes=f"transit P{n} (egress, strictly {transit_alt:.0f} m)",
        ))

    _add(MissionCommand(
        seq=seq, kind=CommandKind.LAND, phase=MissionPhase.LAND,
        coord=Coordinate(lat=home.lat, lon=home.lon),
        notes="explicit goto Launch & Recovery + land + disarm (NOT RTL — PX4 "
              "re-captures home at every re-arm)",
    ))

    # Duration: takeoff/land overhead + transit both ways (~170 m corridor each
    # way on the KMITL field) + the sweep when flown + one serve per ledger pad.
    transit_est = 2.0 * 170.0 / max(spec.speed_mps, 1.0) if route else 0.0
    est = (30.0 + transit_est + (spec.est_duration_s if include_search else 0.0)
           + _SERVE_EST_S * max(1, len(discovered)) + 40.0)
    est = max(60.0, min(est, 1800.0))

    return MissionPlan(
        mission_id=mission_id,
        airframe=active_airframe(),
        expected_duration_s=est,
        commands=cmds,
        target_group_strategy=(
            "One egg cargo per sortie to the committee-ASSIGNED ArUco pad "
            "(DICT_4X4_50, ids 1-6): mandatory transit corridor both ways at "
            f"{transit_alt:.0f} m; blind boustrophedon sweep of the search area "
            "when the pad is not yet in the cross-sortie registry "
            "(finish-sweep-then-serve, operator 2026-07-03); vision-guided "
            "land-ON-pad with an id-verified LAND gate; release after touchdown."
        ),
        fallback_strategy=(
            "Safety watchdog RTH on low battery / GPS loss / geofence / no-fly / "
            "ceiling / datalink / time budget (RTL_RETURN_ALT=20 keeps the "
            "failsafe legal). The time policy refuses to start a sortie the "
            "window can't cover unless the operator forces it."
        ),
    )


def pointer_for(
    plan: MissionPlan,
    *,
    wp_index: int | None = None,
    stop_index: int | None = None,
    transit_index: int | None = None,
    egress: bool = False,
    kind: str = "goto",
) -> int:
    """Resolve a dashboard command pointer into ``plan.commands``.

    ``wp_index`` → the Nth search GOTO; ``stop_index`` (+ ``kind`` "goto"/
    "drop") → that pad's LOCALIZE GOTO or DROP; ``transit_index`` (1-based, +
    ``egress``) → the Nth transit GOTO in that direction. Falls back to 0
    (TAKEOFF) if nothing matches, so the pointer is always in range.
    """
    cmds = plan.commands
    if transit_index is not None:
        want_phase = (MissionPhase.TRANSIT_EGRESS if egress
                      else MissionPhase.TRANSIT_INGRESS)
        legs = [i for i, c in enumerate(cmds)
                if c.phase is want_phase and c.kind is CommandKind.GOTO]
        # Egress commands are rendered P3→P1 but scored/named P{n} descending;
        # resolve by the ORDER FLOWN in that direction (1-based).
        order = list(reversed(legs)) if egress else legs
        if 1 <= transit_index <= len(order):
            return (order[len(order) - transit_index] if egress
                    else order[transit_index - 1])
        return 0
    if wp_index is not None:
        search = [i for i, c in enumerate(cmds)
                  if c.phase is MissionPhase.SEARCH and c.kind is CommandKind.GOTO]
        if 0 <= wp_index < len(search):
            return search[wp_index]
        return 0
    if stop_index is not None:
        want = CommandKind.DROP_PAYLOAD if kind == "drop" else CommandKind.GOTO
        for i, c in enumerate(cmds):
            if (c.kind is want and c.stop_index == stop_index
                    and c.phase in (MissionPhase.LOCALIZE, MissionPhase.DROP)):
                return i
    return 0
