"""Time-budget policy (orchestrator.time_policy).

The reserve rule decides when to refuse a new sortie / the in-sortie descent
commit. These lock the gate boundaries and the no-double-trigger invariant
with the watchdog.
"""

from __future__ import annotations

from mission_brain.profile import COMPETITION
from orchestrator.main import _build_time_policy
from orchestrator.time_policy import TimePolicy


def test_build_time_policy_empty_config_uses_dataclass_defaults() -> None:
    """L1: with no `search:` overrides, main.py's builder must fall back to the
    TimePolicy dataclass defaults — not stale inline literals. The old inline
    fallbacks (300/200) disagreed with both the dataclass (350/210) and config."""
    pol = _build_time_policy({}, COMPETITION)
    assert pol == TimePolicy(watchdog_floor_s=COMPETITION.min_time_remaining_s)
    assert pol.sortie_cost_s == 350.0
    assert pol.known_sortie_cost_s == 210.0
    assert pol.watchdog_floor_s == COMPETITION.min_time_remaining_s == 180.0


def test_build_time_policy_honours_config_overrides() -> None:
    pol = _build_time_policy({"sortie_cost_s": 400.0, "serve_cost_s": 120.0}, COMPETITION)
    assert pol.sortie_cost_s == 400.0
    assert pol.serve_cost_s == 120.0
    assert pol.known_sortie_cost_s == 210.0        # unspecified → dataclass default


def test_can_start_serve_boundary() -> None:
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0,
                   watchdog_floor_s=180.0, margin_s=30.0)
    threshold = p.reserve_s + p.serve_cost_s + p.margin_s   # 180 + 110 + 30 = 320
    assert threshold == 320.0
    assert p.can_start_serve(threshold)
    assert p.can_start_serve(threshold + 1.0)
    assert not p.can_start_serve(threshold - 1.0)


def test_can_start_sortie_needs_a_whole_sortie_more() -> None:
    p = TimePolicy(serve_cost_s=110.0, sortie_cost_s=240.0, rth_reserve_s=60.0,
                   watchdog_floor_s=180.0, margin_s=30.0)
    threshold = p.reserve_s + p.sortie_cost_s + p.margin_s   # 180+240+30 = 450
    assert p.can_start_sortie(threshold)
    assert not p.can_start_sortie(threshold - 1.0)
    # Enough to finish the current serve, not enough for a whole new sortie.
    assert p.can_start_serve(320.0) and not p.can_start_sortie(320.0)
    # A sharper estimate (registry-known pad, no search time) may still fit.
    assert p.can_start_sortie(320.0, est_sortie_s=110.0)


def test_reserve_is_the_larger_of_floor_and_rth() -> None:
    # A tiny watchdog floor must not let RTH be stranded — rth_reserve wins.
    p = TimePolicy(watchdog_floor_s=20.0, rth_reserve_s=60.0)
    assert p.reserve_s == 60.0


def test_serve_finishes_above_the_watchdog_floor() -> None:
    """No-double-trigger: a serve started at the gate threshold ends with margin
    to spare above the watchdog's time floor, so the watchdog can't interrupt it."""
    p = TimePolicy(watchdog_floor_s=COMPETITION.min_time_remaining_s)
    start = p.reserve_s + p.serve_cost_s + p.margin_s     # earliest legal start
    remaining_after = start - p.serve_cost_s              # consumed by the serve
    assert remaining_after >= p.watchdog_floor_s + p.margin_s
    assert remaining_after > p.watchdog_floor_s           # strictly above the floor


def test_constructed_from_profile_floor() -> None:
    p = TimePolicy(watchdog_floor_s=COMPETITION.min_time_remaining_s)
    assert p.watchdog_floor_s == 180.0


def test_known_sortie_gate_uses_the_watchdog_phase_exemption() -> None:
    """The time-floor RTH exempts egress/landing, so a known sortie only needs
    the floor to hold until the egress begins — 403 s remaining (the G4 case
    the stricter rule refused by 17 s) must be a GO."""
    p = TimePolicy(watchdog_floor_s=180.0, known_pre_egress_s=150.0,
                   known_sortie_cost_s=210.0, rth_reserve_s=60.0, margin_s=30.0)
    threshold = max(180.0 + 150.0 + 30.0, 60.0 + 210.0 + 30.0)   # 360
    assert p.can_start_known_sortie(threshold)
    assert p.can_start_known_sortie(403.0)          # the real G4 refusal case
    assert not p.can_start_known_sortie(threshold - 1.0)
    # Full-sweep sorties keep the strict whole-sortie reserve.
    assert not p.can_start_sortie(403.0)


def test_can_start_delivery_honours_both_arms() -> None:
    """The gate takes the LARGER of its two arms.

    Physical arm: egress + one serve + margin = 60+110+30 = 200 s.
    Watchdog arm: the descent must reach a phase the time-floor RTH exempts
    (LAND) before the floor fires = 180+90+30 = 300 s. 300 wins.
    """
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0, margin_s=30.0,
                   watchdog_floor_s=180.0, serve_pre_land_s=90.0)
    assert p.delivery_reserve_s == 300.0
    assert p.can_start_delivery(300.0) is True
    assert p.can_start_delivery(299.0) is False
    # 200 s was the OLD (rth-only) threshold — it must no longer be a GO.
    assert p.can_start_delivery(200.0) is False


def test_delivery_approved_at_threshold_finishes_above_the_watchdog_floor() -> None:
    """No-double-trigger, the DELIVERY twin of the serve invariant above.

    A delivery approved at the gate threshold must still be inside a
    watchdog-EXEMPT phase (LAND/DROP) by the time the time-floor RTH could
    fire — otherwise the watchdog RTHs mid-descent with the egg aboard and,
    because the mission loop exits on a non-RUNNING terminal, forfeits every
    remaining delivery too.
    """
    p = TimePolicy(watchdog_floor_s=COMPETITION.min_time_remaining_s)
    start = p.delivery_reserve_s                       # earliest legal start
    # Worst case: the whole non-exempt part of the serve (hop + acquire + the
    # descend rungs) runs before LAND is commanded.
    at_land_commit = start - p.serve_pre_land_s
    assert at_land_commit >= p.watchdog_floor_s + p.margin_s
    assert at_land_commit > p.watchdog_floor_s         # strictly above the floor
    # …and the physical arm still holds: the serve finishes with the egress in hand.
    assert start - p.serve_cost_s >= p.rth_reserve_s


def test_can_start_discovery_scales_with_the_deliveries_still_owed() -> None:
    """Mid-flight discovery must fund every delivery the flight still owes —
    not just one. One egg aboard reduces to the per-delivery gate; four eggs
    need three more serves on top of it."""
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0, margin_s=30.0,
                   watchdog_floor_s=180.0, serve_pre_land_s=90.0)
    # visit_cost_s (M2, review 2026-07-24) is added on top of the chain —
    # the visit being approved has its own cost, separate from the serves
    # it funds the chain for.
    assert p.can_start_discovery(300.0 + p.visit_cost_s, 1) is True  # visit + delivery_reserve_s
    assert p.can_start_discovery(300.0 + p.visit_cost_s - 1.0, 1) is False
    # 4 owed: 3 further serves (330 s) on top of the last one's own threshold.
    assert p.can_start_discovery(630.0 + p.visit_cost_s, 4) is True
    assert p.can_start_discovery(630.0 + p.visit_cost_s - 1.0, 4) is False
    # The old single-serve reserve (can_start_serve, 320 s) approved discovery
    # that left deliveries 2-4 unfundable — that is the regression this closes.
    assert p.can_start_serve(320.0) and not p.can_start_discovery(320.0, 4)


def test_can_start_discovery_never_drops_below_the_physical_cost() -> None:
    """With a tiny watchdog floor the chain arm shrinks; the plain physical
    cost of n serves + the egress must still hold."""
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0, margin_s=30.0,
                   watchdog_floor_s=0.0, serve_pre_land_s=0.0)
    threshold = p.visit_cost_s + 530.0  # visit + (60 + 4*110 + 30)
    assert p.can_start_discovery(threshold, 4) is True
    assert p.can_start_discovery(threshold - 1.0, 4) is False


def test_can_start_discovery_funds_the_visit_it_is_itself_approving() -> None:
    """M2 (review 2026-07-24): can_start_serve/can_start_sortie both size the
    work they approve into their own reserve; can_start_discovery used to be
    the odd one out — it sized the CHAIN OF SERVES a decode visit must be
    followed by, but not the visit itself (a _wait_arrival(timeout 60) hop +
    decode_dwell_s=4 dwell). Approving a visit right at the old boundary left
    the very next delivery's own gate exactly at ITS threshold with the
    visit's own cost already spent — no margin, immediately refusable by a
    single elapsed second."""
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0, margin_s=30.0,
                   watchdog_floor_s=180.0, serve_pre_land_s=90.0)
    old_threshold = p.delivery_reserve_s               # 300.0 — the pre-fix gate
    assert p.can_start_discovery(old_threshold, 1) is False, (
        "a visit approved here spends visit_cost_s before delivery_reserve_s "
        "for the delivery it exists to enable even starts")
    threshold = p.visit_cost_s + p.delivery_reserve_s
    assert p.can_start_discovery(threshold, 1) is True
    assert p.can_start_discovery(threshold - 1.0, 1) is False
