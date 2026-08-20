"""Energy-budget policy for the multi-sortie delivery window.

The twin of ``time_policy.py``: that one refuses to start a sortie the clock
cannot finish, this one refuses to start a sortie the PACK cannot finish.

The numbers say this matters. ONE 6S 17,000 mAh semi-solid pack — on the
aircraft since 2026-08-19 — yields 12,750 mAh once the flight controller's own
low-battery reserve (0.25) is set aside; a four-delivery flight seeds at
1,750 + 3 x 900 = 4,450 mAh and the GO gate wants that plus the 250 mAh
margin: 4,700 against 12,750. That is comfortable on paper — but the hover
figure behind the seeds is a CALCULATED ~43 A at this pack's AUW, and the
gauge that reports consumption is voltage-only (PM02D powers the FC alone;
motors run from a board the FC cannot sense), which sags ~30-35 percentage
points under flight load (measured 2026-08-20: 28 % under thrust at ~65-70 %
resting SoC). The % the gates see in flight is load-truth, not state of
charge — conservative, and by design.

History worth keeping, because it is the argument for the next pack decision:
1 x 7,500 to 2026-07-25, then 2 x 7,500 in PARALLEL (15,000 mAh, 11,250
usable — the 8.22 kg configuration every validated G4/G4' run used), briefly
back to 1 x 7,500 (2026-08-17), then the 17,000 semi-solid. Capacity is never
free — the second pack's +1.05 kg took hover to ~35.6 A, so +100 % capacity
bought +64 % endurance (11.6 -> 19.0 min). The seeds move with the CURRENT,
not just the capacity.

That is enough for the briefing-default single four-egg flight with room to
spare, which is the point: at ``eggs_aboard=4`` the whole mission is ONE
arm→disarm cycle, so there is no landing at which the crew could swap packs
even though the rules allow it (they approach for resupply between flights).
This module still refuses work the pack cannot finish — a second flight, a
``eggs_aboard=1`` rollback's four sorties, or any flight starting on a pack
that practice already drained.

One caveat this module cannot see: no figure below has ever been MEASURED.
They are bench-table arithmetic (SITL's battery simulator recharges on disarm),
and on the single pack they additionally rest on an ESTIMATED pack mass, so
confirm them against a watt-meter at G5. The ground-truth plan (2026-08-20):
the 1 Hz TELEM audit line now carries batt=/vbat= (the first battery series
this project has ever recorded), and the field procedure logs the charger's
returned-mAh after every session plus rest voltage around each flight — the
seeds get re-derived from THOSE, not from more arithmetic
(.claude/skills/PX4MASTER/references/power-battery.md). (A second caveat retired with the
parallel pair: the FC used to report the SUM of two packs, so one dropping out
mid-flight halved the real capacity invisibly. One pack cannot do that.)

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
    capacity_mah: float = 7500.0
    # Charge the companion may not plan against. Construct from
    # failsafes.bat_low_thr — NOT a second, independently-drifting number.
    # ⚠ CORRECTED 2026-08-16: the old wording here ("the FC flies its own
    # low-battery RTL below this fraction") was wrong. At BAT_LOW_THR the FC
    # only raises a WARNING; with COM_LOW_BAT_ACT=3 it returns at BAT_CRIT_THR
    # (0.15) and lands at BAT_EMERGEN_THR (0.07). This fraction is a planning
    # floor chosen to sit above all of that, not the lip of a failsafe.
    reserve_frac: float = 0.25
    # ⚠ HEADLESS FALLBACK ONLY. sitl/aavc_config.yaml `battery` overrides all
    # four of these, and the real aircraft flies ONE 6S 17000 mAh semi-solid
    # pack (capacity 17000 in config; pack/AUW measured 2026-08-19). These
    # fallback numbers rest on a smaller, heavier-hover basis on PURPOSE: if the
    # config battery block is ever absent, the GO gate must err STRICT (assume
    # too little charge), never optimistic. Do NOT read them as a description of
    # the current pack — that lives in config, not here.
    #
    # First-flight seed for a ONE-delivery flight (fallback hover basis). Used
    # ONLY until one real flight has been measured; the measurement replaces it.
    seed_sortie_mah: float = 1750.0
    # Marginal seed cost of each EXTRA delivery carried in the SAME flight
    # (~110 s TimePolicy.serve_cost_s at hover). Kept at 900 as the conservative
    # fallback — the config value is what a real flight actually uses.
    #
    # A FLIGHT stopped being one delivery at the 2026-07-24 briefing
    # (eggs_aboard=4 → one ~10-minute flight): seed_flight_mah() scales the SEED
    # by eggs_aboard so the pre-flight gate weighs the pack against the WHOLE
    # flight, not ~1/3 of it. Measurements are already whole-flight deltas.
    seed_delivery_mah: float = 900.0
    # Deliveries carried per FLIGHT — 1 = the original one-egg-per-flight model.
    # Constructed from mission.eggs_aboard so the two can't drift apart.
    eggs_aboard: int = 1
    # Slack so a sortie approved at the boundary still lands with charge in hand.
    # Scaled with the cost it guards (150 x 1.23, rounded up) — it protects
    # against the flight-cost ESTIMATE being wrong, and the estimate grew.
    margin_mah: float = 250.0

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
