"""Time-budget policy for the multi-sortie delivery window (CLAUDE.md §9, E1).

The 20-minute window is finite; a land-ON-and-release is expensive (~110 s) and
a whole sortie (transit out + search + serve + egress + land) more so (~240 s
unknown-pad, less when the registry already knows the pad). An orienteering-
style reserve rule keeps the mission from greedily starting work it can't
finish and still get home: it only *refuses to start new work* — it is never a
flight action. The rules add a per-minute penalty past the window, so the
per-sortie gate 409s a late launch unless the operator explicitly forces it
(their call, not the software's). The safety watchdog remains the single
hard-stop authority (battery / GPS / geofence / its own time floor → RTH).

The key invariant (see test): work approved at the gate threshold finishes with
``remaining >= watchdog_floor + margin`` — strictly above the watchdog's time
floor — so a policy-approved serve can never be interrupted by the watchdog's
time-budget RTH, and the policy never blocks the watchdog. Construct with
``watchdog_floor_s = profile.min_time_remaining_s`` so the two agree. Where a
gate approves work that only becomes watchdog-EXEMPT part-way through (a known
sortie, a delivery), the invariant is measured to that hand-over point —
``known_pre_egress_s`` / ``serve_pre_land_s`` — not to touchdown.

A FLIGHT now carries up to ``eggs_aboard`` deliveries (2026-07-24 briefing), so
every mid-flight reserve is sized against the deliveries STILL OWED, not against
one: ``can_start_discovery(remaining, n)``.

Pure arithmetic — no telemetry, no clock — so it is trivially unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TimePolicy:
    # Wall-clock to land-ON one pad + release + climb back out
    # (acquire + 4 descend rungs + LAND + settle/release + takeoff). Calibrate in G4.
    serve_cost_s: float = 110.0
    # Wall-clock for one FULL delivery sortie: transit both ways + expected
    # search + the serve + the L&R landing. Calibrated in SITL G4 2026-07-03/04:
    # full-sweep sorties ran 337-353 s on the KMITL field.
    sortie_cost_s: float = 350.0
    # A registry-known sortie (no search: transit → direct serve → transit →
    # land) — SITL G4 measured 210-231 s typical (145 s best).
    known_sortie_cost_s: float = 210.0
    # Time from launch to ENTERING the egress transit for a known sortie
    # (ingress + direct serve; G4 ≈ 130-150 s). The watchdog's time-floor RTH
    # explicitly EXEMPTS the TRANSIT_EGRESS/LAND/RTH phases, so a known sortie
    # only needs the floor to hold until egress begins — not until touchdown.
    known_pre_egress_s: float = 150.0
    # Time from a DELIVERY starting until the aircraft enters a phase the
    # watchdog's time-floor RTH EXEMPTS. A serve runs SEARCH (the hop to the
    # pad) → LOCALIZE (acquire + the descend rungs) → LAND → DROP, and
    # safety.py exempts only TRANSIT_EGRESS / LAND / RTH — so the exemption
    # starts at the LAND commit, NOT at the delivery start (the same reasoning
    # as known_pre_egress_s above, applied one level down). Budgeted at the
    # pessimistic end of the measured SEARCH+LOCALIZE span (~70-90 s): a ≤20 s
    # pad hop, the 12 s acquire_timeout_s, then the six descend rungs
    # (12→1.5 m at the pinned MPC_Z_V_AUTO_DN=0.4 m/s plus per-rung centring).
    # Anything shorter lets can_start_delivery approve a descent the watchdog
    # aborts mid-rung with the egg aboard — and that ends the whole MISSION
    # (mission.py's loop exits on a non-RUNNING terminal), forfeiting every
    # delivery still to come, not just this one.
    serve_pre_land_s: float = 90.0
    # Time to fly the egress transit + land at L&R from the far side.
    rth_reserve_s: float = 60.0
    # The safety watchdog's own time-floor RTH trigger (= profile.min_time_remaining_s).
    watchdog_floor_s: float = 180.0
    # Slack so the gate and the watchdog never race at the boundary.
    margin_s: float = 30.0
    # Wall-clock cost of ONE mid-flight discovery visit (a hop to a candidate
    # + dwell): mission.py's _decode_visits is a goto + _wait_arrival(timeout
    # 60) + decode_dwell_s=4 of polling. Budgeted well under the 60 s worst
    # case (visits normally arrive in a few seconds; can_start_discovery is
    # re-checked before EACH candidate in the loop, not just once for the
    # whole pass, so a single over-budget visit is bounded, not compounding).
    visit_cost_s: float = 45.0

    @property
    def reserve_s(self) -> float:
        """End-of-mission reserve that must survive any new work — the larger of
        the watchdog floor and the physical RTH cost (so a small floor can't
        strand the aircraft)."""
        return max(self.watchdog_floor_s, self.rth_reserve_s)

    def can_start_serve(self, remaining_s: float) -> bool:
        """True if there is time to land-ON one pad AND keep the reserve —
        gates the in-sortie descent commit + the single retry."""
        return remaining_s >= self.reserve_s + self.serve_cost_s + self.margin_s

    @property
    def delivery_reserve_s(self) -> float:
        """Remaining window a single mid-flight delivery must have to START.

        TWO arms, both mandatory — the gate takes the larger:

        * the WATCHDOG arm, ``watchdog_floor_s + serve_pre_land_s + margin_s``:
          the descent has to reach a phase the time-floor RTH exempts (LAND)
          before that floor fires. This is the module invariant at the top of
          the file, and it is not politeness — a watchdog RTH mid-approach
          leaves ``state.terminal`` non-RUNNING, which ends the mission loop and
          forfeits every remaining delivery, not just this one.
        * the PHYSICAL arm, ``rth_reserve_s + serve_cost_s + margin_s``: it must
          also finish the serve and still fly the egress + L&R landing. The
          aircraft is already committed to THIS flight, so this arm reserves the
          egress cost, not the full end-of-mission ``reserve_s``.

        (Until the 2026-07-24 review this was the physical arm ALONE — 200 s at
        the shipped numbers — so a 110 s serve approved at 200 s ended at 90 s,
        below the 180 s floor, and the watchdog aborted a descent the policy had
        just approved. The numerator changed from "a flight is one delivery"
        without re-deriving the reserve.)"""
        return max(self.watchdog_floor_s + self.serve_pre_land_s + self.margin_s,
                   self.rth_reserve_s + self.serve_cost_s + self.margin_s)

    def can_start_delivery(self, remaining_s: float) -> bool:
        """Mid-flight gate before each delivery in a multi-egg flight — see
        ``delivery_reserve_s``. The per-delivery battery guard lives alongside
        this in the mission loop."""
        return remaining_s >= self.delivery_reserve_s

    def can_start_discovery(self, remaining_s: float,
                            deliveries_owed: int) -> bool:
        """Gate for OPTIONAL mid-flight discovery (decode visits, registry
        completion) on a flight that still owes ``deliveries_owed`` deliveries.

        Discovery is not free: every second it spends is a second the deliveries
        it exists to enable no longer have. Sized so the flight can still start
        AND finish everything it owes afterwards — the LAST of which must itself
        clear ``delivery_reserve_s``::

            (n - 1) * serve_cost_s + delivery_reserve_s

        …and never below the plain physical cost of n serves plus the egress
        (``rth_reserve_s + n*serve_cost_s + margin_s``), which dominates when the
        watchdog floor is small.

        ``n = 1`` reduces to ``can_start_delivery`` PLUS ``visit_cost_s`` (see
        below) — this replaces the old ``can_start_serve`` reserve at the
        multi-egg discovery call sites: that one funds exactly ONE following
        serve, so at four eggs aboard discovery could legitimately burn the
        window down until deliveries 3-4 were refused — half the score, no
        rule broken, no anomaly that reads as a problem.

        Unlike ``can_start_serve``/``can_start_sortie`` (each sizes the work
        IT approves into its own reserve), this used to size only the chain of
        serves a visit must be followed by, not the visit itself (M2, review
        2026-07-24) — so a visit approved right at the boundary spent
        ``visit_cost_s`` before the very next delivery's own gate even started
        counting, leaving it exactly AT its threshold with zero margin.
        ``visit_cost_s`` is added on top for the same no-double-trigger reason
        the module docstring gives for every other gate here."""
        n = max(1, int(deliveries_owed))
        need_chain = (n - 1) * self.serve_cost_s + self.delivery_reserve_s
        need_physical = (self.rth_reserve_s + n * self.serve_cost_s
                         + self.margin_s)
        return remaining_s >= self.visit_cost_s + max(need_chain, need_physical)

    def can_start_sortie(self, remaining_s: float,
                         est_sortie_s: float | None = None) -> bool:
        """True if there is time for a whole further sortie AND the reserve.
        Recomputed each gate tick; the GO endpoint refuses without ``force``
        when False. ``est_sortie_s`` lets the caller pass a sharper estimate
        (e.g. a registry-known pad needs no search time)."""
        est = self.sortie_cost_s if est_sortie_s is None else est_sortie_s
        return remaining_s >= self.reserve_s + est + self.margin_s

    def can_start_known_sortie(self, remaining_s: float) -> bool:
        """Gate for a registry-known sortie. The watchdog's time-floor RTH
        exempts the egress/landing phases, so the floor only has to survive
        until the egress BEGINS (``known_pre_egress_s``), not until touchdown —
        plus the physical minimum of flying the whole sortie + RTH reserve.
        G4-measured: launching at ~400 s remaining lands with ~175 s left,
        entirely inside the exempt phases."""
        need_floor = self.watchdog_floor_s + self.known_pre_egress_s + self.margin_s
        need_physical = self.rth_reserve_s + self.known_sortie_cost_s + self.margin_s
        return remaining_s >= max(need_floor, need_physical)
