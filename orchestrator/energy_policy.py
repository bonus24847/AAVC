"""Energy-budget policy for the multi-sortie delivery window.

The twin of ``time_policy.py``: that one refuses to start a sortie the clock
cannot finish, this one refuses to start a sortie the PACK cannot finish.

The numbers say this matters. ONE 6S 7,500 mAh pack yields 5,625 mAh once the
flight controller's own low-battery reserve is set aside, and a four-delivery
flight seeds at 1,700 + 3 x 900 = 4,400 mAh at the ~29 A hover the EFT E5 bench
table gives for a 7.17 kg X6100 — inside the pack, but with only ~1.2 Ah of
slack for wind, a longer sweep, a go-around, or any second flight. Since
2026-07-25 the aircraft therefore carries TWO of those packs in PARALLEL:
15,000 mAh at 6S, 11,250 mAh usable. The second pack is bought back in weight,
not for free — +1.05 kg takes hover to ~35.6 A, so capacity +100 % buys
endurance +64 % (11.6 -> 19.0 min), and the seeds below moved with the current,
not just the capacity.

That is enough for the briefing-default single four-egg flight with room to
spare, which is the point: at ``eggs_aboard=4`` the whole mission is ONE
arm→disarm cycle, so there is no landing at which the crew could swap packs
even though the rules allow it (they approach for resupply between flights).
This module still refuses work the pack cannot finish — a second flight, a
``eggs_aboard=1`` rollback's four sorties, or any flight starting on a pack
that practice already drained.

Two caveats this module cannot see. The flight controller reports the SUM of
the parallel pair, so a pack dropping out mid-flight halves the real capacity
while every number here still assumes 15,000 mAh. And no figure below has ever
been MEASURED: they are bench-table arithmetic (SITL's battery simulator
recharges on disarm), so confirm them against a watt-meter at G5.

Like ``time_policy`` this only refuses to START new work; it is never a flight
action. The safety watchdog and the FC's own battery failsafe remain the hard
stops.

Pure arithmetic — no telemetry, no clock — so it is trivially unit-testable.
"""

from __future__ import annotations

import math
import statistics
import time
from collections.abc import Sequence
from dataclasses import dataclass

#: A pack swap shows up as the charge going UP. Normal flight never does that,
#: so this only has to clear sensor noise, not a real discharge.
SWAP_PERCENT_JUMP = 15.0

#: How stale the flight controller's coulomb count may be before it stops
#: counting as a measurement. The count arrives on the OPTIONAL raw-MAVLink
#: listener, so it can stop updating while the rest of telemetry keeps flowing —
#: and a frozen number that still claims to be "measured" is worse than the
#: coarse percentage estimate, because it shadows it.
CONSUMED_MAX_AGE_S = 10.0


@dataclass(frozen=True)
class EnergyPolicy:
    # Pack label capacity. Declared in config and cross-checked against the FC's
    # BAT1_CAPACITY, because a pack swapped without updating the config and an
    # uncalibrated power module look identical from here.
    capacity_mah: float = 15000.0
    # Charge the companion may not plan against. Construct from
    # failsafes.bat_low_thr — NOT a second, independently-drifting number.
    # ⚠ CORRECTED 2026-08-16: the old wording here ("the FC flies its own
    # low-battery RTL below this fraction") was wrong. At BAT_LOW_THR the FC
    # only raises a WARNING; with COM_LOW_BAT_ACT=3 it returns at BAT_CRIT_THR
    # (0.15) and lands at BAT_EMERGEN_THR (0.07). This fraction is a planning
    # floor chosen to sit above all of that, not the lip of a failsafe.
    reserve_frac: float = 0.25
    # These defaults describe the CURRENT aircraft: two DXF 6S 7500 mAh packs in
    # parallel (2026-07-25), 15,000 mAh at 8.22 kg AUW. The hover current they
    # are derived from moved with the mass, not just the capacity — hover power
    # scales ~W^1.5, so (8.22/7.17)^1.5 = 1.23x turns the EFT E5 bench table's
    # 29 A into ~35.6 A. Config (sitl/aavc_config.yaml `battery`) overrides all
    # four; these are the headless fallback and must describe the same aircraft.
    #
    # First-flight estimate for a ONE-delivery flight: 35.6 A hover x 3.5 min
    # (was 1700 at 29 A / 7.17 kg). Used ONLY until one flight has been measured.
    seed_sortie_mah: float = 2100.0
    # Marginal seed cost of each EXTRA delivery carried in the SAME flight: the
    # ~110 s TimePolicy.serve_cost_s at that same 35.6 A hover
    # = 35.6 A x 110/3600 h = 1.09 Ah, rounded to 1100 mAh (was 900 at 29 A).
    #
    # A FLIGHT stopped being one delivery at the 2026-07-24 briefing
    # (eggs_aboard=4 → one ~10-minute flight), but the seed did not move with
    # it: the pre-flight gate then compared the pack against ~1/3 of what the
    # flight actually costs and showed a falsely green card for a flight the
    # pack cannot finish. Scaling the SEED (not the measurements — those are
    # already whole flights) is what keeps the gate honest on flight 1.
    seed_delivery_mah: float = 1100.0
    # Deliveries carried per FLIGHT — 1 = the original one-egg-per-flight model.
    # Constructed from mission.eggs_aboard so the two can't drift apart.
    eggs_aboard: int = 1
    # Slack so a sortie approved at the boundary still lands with charge in hand.
    # Scaled with the cost it guards (150 x 1.23, rounded up) — it protects
    # against the flight-cost ESTIMATE being wrong, and the estimate grew.
    margin_mah: float = 200.0

    def usable_mah(self) -> float:
        """Charge available for planning, i.e. above the FC's own reserve."""
        return self.capacity_mah * (1.0 - self.reserve_frac)

    def seed_flight_mah(self) -> float:
        """Seed cost of ONE whole flight at the configured ``eggs_aboard``:
        the one-delivery seed plus the marginal cost of each extra delivery
        the same flight now carries."""
        extra = max(0, int(self.eggs_aboard) - 1)
        return self.seed_sortie_mah + extra * self.seed_delivery_mah

    def sortie_cost_mah(self, history: Sequence[float]) -> float:
        """Expected cost of the next FLIGHT: the median of what flights have
        actually cost, or the seed estimate before any have flown. Median rather
        than mean so one sweep-heavy first flight does not distort the rest.
        Measurements are whole-flight deltas (mission.py takes them across the
        arm→disarm cycle), so they already include every delivery — only the
        SEED has to be scaled by eggs_aboard."""
        measured = [v for v in history if v > 0 and not math.isnan(v)]
        return statistics.median(measured) if measured else self.seed_flight_mah()

    def sorties_remaining(self, consumed_mah: float,
                          history: Sequence[float]) -> float:
        """How many further sorties the remaining charge covers — the headline
        number on the GCS. NaN when consumption is unknown."""
        if math.isnan(consumed_mah):
            return math.nan
        cost = self.sortie_cost_mah(history)
        return max(0.0, (self.usable_mah() - consumed_mah) / cost) if cost > 0 else math.nan

    def can_start_sortie(self, consumed_mah: float,
                         history: Sequence[float]) -> tuple[bool, str]:
        """Whether the pack can cover another sortie, and why.

        The reason is written for the operator and is shown verbatim on the
        pre-flight card. An unknown consumption (NaN — no calibrated power
        module yet) ALLOWS the sortie: refusing to fly because we cannot measure
        would ground a serviceable aircraft, and the FC's own failsafe still
        protects it.
        """
        cost = self.sortie_cost_mah(history)
        if math.isnan(consumed_mah):
            return True, (f"energy unknown (no calibrated power module) — "
                          f"next sortie needs ~{cost:.0f} mAh")
        left = self.usable_mah() - consumed_mah
        if left >= cost + self.margin_mah:
            return True, (f"{left:.0f} mAh usable left, next sortie "
                          f"needs ~{cost:.0f}")
        return False, (f"{left:.0f} mAh usable left, next sortie needs "
                       f"~{cost:.0f} — swap the battery")


def energy_consumed_mah(telemetry: object, capacity_mah: float, *,
                        now_monotonic: float | None = None,
                        max_age_s: float = CONSUMED_MAX_AGE_S) -> tuple[float, str]:
    """Charge drawn from the pack so far, and which tier the number came from.

    Tier ``A`` is the flight controller's coulomb count — trustworthy, but only
    present once the power module is calibrated, and only while the raw-MAVLink
    listener that carries it is alive. A count that has stopped arriving is NOT
    tier A: it would freeze at its last value and shadow the tier-B estimate that
    is still updating, so the budget would plan against a pack that appears to
    have stopped draining. Tier ``B`` derives consumption from the reported
    percentage, which PX4 itself estimates; it is much coarser, and the GCS shows
    which tier is live because an estimate presented as a measurement is worse
    than no number at all. A negative coulomb count is PX4's "not available"
    sentinel (SITL sends -383).
    """
    consumed = float(getattr(telemetry, "battery_consumed_mah", math.nan))
    stamped = float(getattr(telemetry, "battery_consumed_monotonic", math.nan))
    fresh = True
    if max_age_s > 0 and not math.isnan(consumed):
        if math.isnan(stamped):
            fresh = False            # a value nobody timestamped is unverifiable
        else:
            now = time.monotonic() if now_monotonic is None else now_monotonic
            fresh = (now - stamped) <= max_age_s
    if not math.isnan(consumed) and consumed >= 0.0 and fresh:
        return consumed, "A"
    percent = float(getattr(telemetry, "battery_percent", math.nan))
    if not math.isnan(percent) and 0.0 <= percent <= 100.0:
        return capacity_mah * (1.0 - percent / 100.0), "B"
    return math.nan, "none"


def baseline_for_pack(now_mah: float, now_pct: float, capacity_mah: float) -> float:
    """The consumption baseline to adopt for a freshly-fitted pack.

    Subtracting the baseline from the raw reading is what makes "charge used"
    mean "used from THIS pack". The obvious baseline — whatever the meter reads
    the moment the new pack is fitted — quietly assumes every spare arrives full,
    and a spare at 60 % would then be budgeted as if it held the whole 7,500 mAh:
    the gate would approve a sortie the pack cannot finish and the FC's own
    low-battery failsafe would end it mid-flight with the egg aboard.

    So the pack's own reported charge sets the starting point: a 60 % spare
    starts life already 40 % consumed. Percentage is coarse, but it is the only
    signal that distinguishes a full spare from a part-used one, and erring here
    costs a needless swap rather than a dead pack in the air.
    """
    if math.isnan(now_mah):
        return 0.0
    if math.isnan(now_pct) or not (0.0 <= now_pct <= 100.0):
        return now_mah                      # no charge signal: assume it is full
    already_used = capacity_mah * (1.0 - now_pct / 100.0)
    return now_mah - already_used


def detect_battery_swap(prev_mah: float, now_mah: float,
                        prev_pct: float, now_pct: float) -> bool:
    """Whether the pack was changed between two readings.

    The readings must SPAN the moment the crew can touch the aircraft — the
    resupply hold between sorties, when it is landed and disarmed at L&R. Either
    signal is enough: the coulomb count restarting near zero (the flight
    controller powers from the pack, so a swap reboots it), or the charge jumping
    up by more than noise.
    """
    if (not math.isnan(prev_mah) and not math.isnan(now_mah)
            and prev_mah > 100.0 and now_mah < prev_mah * 0.5):
        return True
    if (not math.isnan(prev_pct) and not math.isnan(now_pct)
            and now_pct - prev_pct > SWAP_PERCENT_JUMP):
        return True
    return False
