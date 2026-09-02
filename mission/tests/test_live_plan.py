"""Live per-sortie mission plan (mission_brain.live_plan, V1.3).

The delivery mission rebuilds its plan per sortie: mandatory transit corridor
both ways, search legs only while the assigned pad is unknown, a GOTO+DROP pair
per committed pad, and an explicit LAND at the L&R point (never RTL). These
lock the plan shape, the ledger, and that the dashboard command pointer always
resolves — transit points included.
"""

from __future__ import annotations

from mission_brain.live_plan import ServedStop, pointer_for, render_live_plan
from mission_brain.profile import COMPETITION, load_profile
from mission_brain.schemas import CommandKind, Coordinate, MissionPhase, MissionPlan
from mission_brain.search_pattern import build_search_pattern

SEARCH_AREA = [
    [13.730723, 100.787840],
    [13.730703, 100.789776],
    [13.731359, 100.789916],
    [13.731239, 100.787824],
]
TRANSIT = [Coordinate(lat=13.730322, lon=100.787446),
           Coordinate(lat=13.730397, lon=100.788694),
           Coordinate(lat=13.730712, lon=100.788755)]
HOME = Coordinate(lat=13.730250, lon=100.787300)


def _spec():
    return build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0)


def test_unknown_pad_sortie_shape() -> None:
    plan = render_live_plan(HOME, _spec(), discovered=[], profile=COMPETITION,
                            transit_route=TRANSIT, sortie=1)
    assert isinstance(plan, MissionPlan)
    assert plan.commands[0].kind is CommandKind.TAKEOFF
    assert plan.commands[0].altitude_m == COMPETITION.transit_alt_m
    # Ingress P1..P3 right after takeoff, at the strict transit altitude.
    ingress = [c for c in plan.commands if c.phase is MissionPhase.TRANSIT_INGRESS]
    assert len(ingress) == 3
    assert all(c.altitude_m == COMPETITION.transit_alt_m for c in ingress)
    assert plan.commands[1:4] == ingress
    # Search legs present (pad unknown), then egress reversed, then LAND home.
    assert sum(1 for c in plan.commands if c.phase is MissionPhase.SEARCH) >= 2
    egress = [c for c in plan.commands if c.phase is MissionPhase.TRANSIT_EGRESS]
    assert len(egress) == 3
    assert egress[0].coord is not None and egress[0].coord.lat == TRANSIT[2].lat
    assert plan.commands[-1].kind is CommandKind.LAND
    assert plan.commands[-1].coord is not None
    assert plan.commands[-1].coord.lat == HOME.lat
    assert not any(c.kind is CommandKind.RTH for c in plan.commands)  # never RTL
    assert not any(c.kind is CommandKind.DROP_PAYLOAD for c in plan.commands)
    assert 0 < plan.expected_duration_s <= 1800


def test_known_pad_sortie_skips_search_legs() -> None:
    disc = [ServedStop(0, 13.7309, 100.7882, "PAD3")]
    plan = render_live_plan(HOME, _spec(), discovered=disc, profile=COMPETITION,
                            transit_route=TRANSIT, include_search=False, sortie=2)
    assert sum(1 for c in plan.commands if c.phase is MissionPhase.SEARCH) == 0
    # Ingress → LOCALIZE+DROP → egress → LAND.
    kinds = [c.kind for c in plan.commands]
    assert kinds[:4] == [CommandKind.TAKEOFF] + [CommandKind.GOTO] * 3
    assert CommandKind.DROP_PAYLOAD in kinds
    assert kinds[-1] is CommandKind.LAND
    drop = next(c for c in plan.commands if c.kind is CommandKind.DROP_PAYLOAD)
    # ONE egg servo: the physical release is always payload 0; the ledger index
    # rides in stop_index.
    assert drop.payload_id == 0
    assert drop.stop_index == 0
    assert drop.confirmed


def test_serve_pairs_append_before_egress() -> None:
    disc = [ServedStop(0, 13.7309, 100.7882, "PAD3"),
            ServedStop(1, 13.7311, 100.7893, "PAD5")]
    plan = render_live_plan(HOME, _spec(), discovered=disc, profile=COMPETITION,
                            transit_route=TRANSIT)
    egress_start = next(i for i, c in enumerate(plan.commands)
                        if c.phase is MissionPhase.TRANSIT_EGRESS)
    tail = plan.commands[egress_start - 4:egress_start]
    assert [c.kind for c in tail] == [
        CommandKind.GOTO, CommandKind.DROP_PAYLOAD,
        CommandKind.GOTO, CommandKind.DROP_PAYLOAD,
    ]
    drops = [c for c in plan.commands if c.kind is CommandKind.DROP_PAYLOAD]
    assert [d.stop_index for d in drops] == [0, 1]
    assert all(d.payload_id == 0 for d in drops)   # one egg servo, always id 0
    assert drops[0].coord is not None
    assert drops[0].coord.lat == 13.7309 and drops[0].coord.lon == 100.7882


def test_pointer_resolves_transit_waypoints_and_serves() -> None:
    disc = [ServedStop(0, 13.7309, 100.7882, "PAD3")]
    plan = render_live_plan(HOME, _spec(), discovered=disc, profile=COMPETITION,
                            transit_route=TRANSIT)
    # Transit: ingress P1 is command 1; egress leg 1 (P3, flown first) is the
    # first TRANSIT_EGRESS command.
    assert pointer_for(plan, transit_index=1) == 1
    assert pointer_for(plan, transit_index=3) == 3
    egress_first = next(i for i, c in enumerate(plan.commands)
                        if c.phase is MissionPhase.TRANSIT_EGRESS)
    assert pointer_for(plan, transit_index=1, egress=True) == egress_first
    # wp 0 = first SEARCH GOTO = right after the 3 ingress points.
    assert pointer_for(plan, wp_index=0) == 4
    n_wp = sum(1 for c in plan.commands
               if c.phase is MissionPhase.SEARCH and c.kind is CommandKind.GOTO)
    assert pointer_for(plan, stop_index=0, kind="goto") == 4 + n_wp
    assert pointer_for(plan, stop_index=0, kind="drop") == 5 + n_wp
    # Out-of-range / unknown falls back to TAKEOFF (always in range).
    assert pointer_for(plan, wp_index=999) == 0
    assert pointer_for(plan, stop_index=42, kind="drop") == 0
    assert pointer_for(plan, transit_index=9) == 0


def test_no_transit_route_still_renders_a_valid_plan() -> None:
    # Bench/unit contexts without a corridor: TAKEOFF + search + LAND ≥ 4 cmds.
    plan = render_live_plan(HOME, _spec(), discovered=[], profile=COMPETITION)
    assert len(plan.commands) >= 4
    assert plan.commands[-1].kind is CommandKind.LAND
    assert not any(c.phase is MissionPhase.TRANSIT_INGRESS for c in plan.commands)


def test_served_count_scales_duration_but_stays_bounded() -> None:
    spec = _spec()
    many = [ServedStop(i, 13.7308, 100.788 + 1e-4 * i, f"PAD{i + 1}")
            for i in range(4)]
    plan = render_live_plan(HOME, spec, discovered=many, profile=COMPETITION,
                            transit_route=TRANSIT)
    assert plan.expected_duration_s <= 1800
    drops = [c for c in plan.commands if c.kind is CommandKind.DROP_PAYLOAD]
    assert len(drops) == 4


def test_multi_delivery_flight_renders_distinct_payload_and_stop_ids():
    prof = load_profile("competition")
    discovered = [
        ServedStop(stop_index=0, lat=13.7307, lon=100.7880, name="PAD3", payload_id=0),
        ServedStop(stop_index=1, lat=13.7306, lon=100.7883, name="PAD1", payload_id=1),
        ServedStop(stop_index=2, lat=13.7308, lon=100.7885, name="PAD4", payload_id=2),
        ServedStop(stop_index=3, lat=13.7309, lon=100.7887, name="PAD6", payload_id=3),
    ]
    plan = render_live_plan(HOME, _spec(), discovered=discovered, profile=prof,
                            include_search=False, sortie=1)
    drops = [c for c in plan.commands if c.kind is CommandKind.DROP_PAYLOAD]
    assert [d.payload_id for d in drops] == [0, 1, 2, 3]
    assert [d.stop_index for d in drops] == [0, 1, 2, 3]
