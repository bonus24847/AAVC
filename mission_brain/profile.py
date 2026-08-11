"""Mission profile — one source of truth for operational limits, selectable
between a locked-down ``competition`` profile (AAVC: fixed time window, geofence
margin, battery/loss thresholds) and a configurable ``production`` profile for
real-world operations.

Selected by the ``AAVC_PROFILE`` env var (default ``competition``), mirroring
the ``AAVC_AIRFRAME`` startup-pick pattern. The competition profile reproduces
the values that were previously hard-coded in ``SafetyWatchdog`` /
``OrchestratorState``, so the AAVC quad's behaviour is byte-for-byte unchanged —
``tests/test_config.py`` regression-locks this.
"""

from __future__ import annotations

import os

from pydantic import BaseModel, ConfigDict, Field


class MissionProfile(BaseModel):
    """Operational envelope for a mission run. Fed into SafetyWatchdog +
    OrchestratorState (and, in later phases, the altitude clamp + drop count)."""

    model_config = ConfigDict(extra="forbid")

    name: str
    # ── time budget (s) ──
    operation_window_s: float = Field(..., gt=0)
    min_time_remaining_s: float = Field(..., ge=0)
    # ── battery (%) — RTH first, then LAND-in-place ──
    rth_battery_pct: float = Field(..., ge=0, le=100)
    land_battery_pct: float = Field(..., ge=0, le=100)
    # ── link / sensor loss debounce (s) before the watchdog escalates to RTH ──
    datalink_loss_threshold_s: float = Field(..., ge=0)
    gps_loss_threshold_s: float = Field(..., ge=0)
    telemetry_stale_threshold_s: float = Field(..., ge=0)
    # ── geofence proximity-warning margin (m) ──
    geofence_margin_m: float = Field(..., ge=0)
    # ── altitude band (m AGL), the operator/airframe envelope (wired in B2) ──
    altitude_floor_m: float = Field(..., ge=0)
    altitude_ceiling_m: float = Field(..., gt=0)
    # ── AAVC 2026 V1.3 flight-rule band: transit legs fly AT transit_alt_m; the
    # search phase must stay at/above search_floor_m — only the delivery descent
    # over the pad (LOCALIZE/DROP/LAND) may go below it. ──
    transit_alt_m: float = Field(..., gt=0)
    search_floor_m: float = Field(..., ge=0)
    # ── payload: cargo released per landing (V1.3: ONE egg, payload_id 0) ──
    drop_count_max: int = Field(..., ge=0)
    # ── delivery sorties per operation window (V1.3: ≤4 pads, 1 egg each) ──
    max_sorties: int = Field(..., ge=1)
    # ── V1.3+briefing: eggs carried per FLIGHT (arm→disarm). 1 = the original
    # one-egg-per-sortie model; 4 = the briefing's carry-all-in-one-flight. ──
    eggs_aboard: int = Field(1, ge=1)
    # ── A4: when False, a PLANNED drop is suppressed unless a confident target
    # was confirmed (production: never drop on an unverified location). When
    # True (competition), a centroid drop still fires — an attempt beats no drop.
    drop_without_confirmation: bool
    # ── default vision target (B3). Production = empty: the operator must name
    # the target; there is no AAVC stock mannequin in real-world ops. ──
    default_target: str


# Competition profile = the exact values that were hard-coded before B1 (so the
# AAVC quad is unchanged). DO NOT drift these without re-checking the watchdog.
COMPETITION = MissionProfile(
    name="competition",
    operation_window_s=1200.0,        # AAVC 20-min window
    min_time_remaining_s=180.0,
    rth_battery_pct=30.0,
    land_battery_pct=20.0,
    datalink_loss_threshold_s=5.0,
    gps_loss_threshold_s=5.0,
    telemetry_stale_threshold_s=10.0,
    geofence_margin_m=5.0,
    # ── AAVC 2026 V1.3: hard 20 m ceiling; transit strictly AT 20 m; search
    # band 10-20 m; below 10 m only for the delivery descent over the pad ──
    altitude_floor_m=3.0,             # lowest commanded hover (LAND descends below)
    altitude_ceiling_m=20.0,          # competition rule: never exceed 20 m AGL
    transit_alt_m=20.0,               # rules: transit legs strictly at 20 m AGL
    search_floor_m=10.0,              # rules: search phase ≥ 10 m AGL
    # M9 (review 2026-07-24): drop_count_max is DEAD config — nothing reads it
    # except its own pinning test (tests/test_config.py); the payload_id
    # release channel actually always addresses ONE egg (0..eggs_aboard-1),
    # per delivery, not per sortie. Left at 1 rather than deleted (no reader
    # to break either way) — do not read this field for anything live.
    drop_count_max=1,
    # ≤4 landing pads = the DELIVERY ceiling (seeds state.max_deliveries'
    # default in orchestrator/main.py). NOT the flight count: with
    # eggs_aboard eggs carried per flight, the actual flight ceiling is
    # state.max_sorties, computed by mission_brain.flights.max_flights_for —
    # 1 flight at eggs_aboard=4, matching the briefing below, not "4 flights
    # of one delivery each" this comment used to (wrongly) say.
    max_sorties=4,
    eggs_aboard=4,                    # briefing: carry all 4 in one flight
    # V1.3 reverses "an attempt beats no drop": landing on the WRONG pad (or off
    # the pad) wastes the assigned egg — the align layer defers instead of
    # landing without a decoded assigned-ID confirmation (require_id_votes).
    drop_without_confirmation=False,
    default_target="aruco landing pad",
)

# Production profile = wider envelope for real operations. Operator-tunable
# (a YAML override loader is a planned extension); these are sane defaults.
PRODUCTION = MissionProfile(
    name="production",
    operation_window_s=3600.0,        # longer real-world missions
    min_time_remaining_s=180.0,
    rth_battery_pct=30.0,
    land_battery_pct=20.0,
    datalink_loss_threshold_s=5.0,
    gps_loss_threshold_s=5.0,
    telemetry_stale_threshold_s=10.0,
    geofence_margin_m=10.0,           # more standoff from the boundary
    altitude_floor_m=2.0,             # precision landing
    altitude_ceiling_m=120.0,         # mapping / survey
    transit_alt_m=30.0,               # generic cruise for real-world ops
    search_floor_m=2.0,               # no competition floor outside AAVC
    drop_count_max=1,
    max_sorties=1,
    eggs_aboard=1,                    # one egg per flight in production
    drop_without_confirmation=False,  # never drop on an unverified target
    default_target="",                # operator must specify the target
)

# KMUTNB sky-field practice profile (2026-08-11): identical safety envelope to
# COMPETITION except the altitude band — the user-briefed hard 5 m AGL ceiling
# on the rooftop pitch (transit 4 m, sweep 4 m, search floor 2.5 m, hover floor
# 1 m). Everything else (20-min window, battery/link thresholds, 4 deliveries,
# eggs_aboard 4, no unverified drops) carries over unchanged. COMPETITION itself
# must stay byte-identical — tests/test_config.py pins it.
KMUTNB_SKYFIELD = COMPETITION.model_copy(update={
    "name": "kmutnb_skyfield",
    "altitude_floor_m": 1.0,
    "altitude_ceiling_m": 5.0,
    "transit_alt_m": 4.0,
    "search_floor_m": 2.5,
})

_PROFILES: dict[str, MissionProfile] = {
    "competition": COMPETITION,
    "production": PRODUCTION,
    "kmutnb_skyfield": KMUTNB_SKYFIELD,
}


def load_profile(name: str | None = None) -> MissionProfile:
    """Return the active MissionProfile. Precedence: explicit ``name`` arg →
    ``AAVC_PROFILE`` env → ``competition`` (the safe default). An unknown name
    falls back to competition."""
    key = (name or os.environ.get("AAVC_PROFILE", "")).strip().lower() or "competition"
    return _PROFILES.get(key, COMPETITION)
