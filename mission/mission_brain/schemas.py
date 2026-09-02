"""Pydantic schemas for AAVC mission planning + vision.

Typed, validated shapes for the deterministic flight core — the plan the
mission executes, the per-frame vision analysis, and the airframe/command
enums. No LLM, no wizard: those schema families were part of the dropped
GCS-agent/wizard stack (§4).
"""

from __future__ import annotations

import os
from enum import Enum
from typing import Literal

from loguru import logger
from pydantic import BaseModel, ConfigDict, Field


class Coordinate(BaseModel):
    model_config = ConfigDict(extra="forbid")

    lat: float = Field(..., description="WGS84 latitude in decimal degrees")
    lon: float = Field(..., description="WGS84 longitude in decimal degrees")
    alt_m: float | None = Field(
        None,
        description="Altitude in metres AGL. None = use mission default.",
    )


class MissionPhase(str, Enum):
    PREFLIGHT = "preflight"    # readiness gate, before arm/takeoff (holds for operator GO)
    TAKEOFF = "takeoff"
    TRANSIT_INGRESS = "transit_ingress"
    SEARCH = "search"
    LOCALIZE = "localize"
    DROP = "drop"
    TRACK = "track"            # object-tracking mission: follow a detected target
    TRANSIT_EGRESS = "transit_egress"
    LAND = "land"
    RTH = "rth"
    ABORT = "abort"


class CommandKind(str, Enum):
    TAKEOFF = "takeoff"
    GOTO = "goto"
    HOVER = "hover"
    SEARCH_PATTERN = "search_pattern"
    DROP_PAYLOAD = "drop_payload"
    LAND = "land"
    RTH = "rth"
    SET_MODE = "set_mode"


class Airframe(str, Enum):
    """Vehicle classes the lab can fly. The operator picks one at startup
    (AAVC_AIRFRAME, see active_airframe) — the schema lists them all so future
    fleet expansion does not require a schema change."""

    QUADCOPTER = "quadcopter"
    HEXACOPTER = "hexacopter"                     # AAVC 2026: EFT X6100, PX4 6001
    VTOL = "vtol"                                # quad-plane / hybrid
    FIXED_WING = "fixed_wing"
    TILT_VTOL = "tilt_vtol"
    TAILSITTER = "tailsitter"
    TWINBOOM = "twinboom"                         # CFD-derived twin-boom A-tail VTOL


#: The aircraft AAVC actually flies. Changed 2026-07-22 from QUADCOPTER when the
#: airframe became the EFT X6100 hexacopter (PX4 airframe 6001 on the real 6X,
#: 22000_gz_eft_x6100 in SITL). Everything vehicle-shaped — plan stamping, the
#: the SITL model name — resolves through active_airframe()
#: rather than hardcoding a member, so a future swap is one env var or one edit.
DEFAULT_AIRFRAME = Airframe.HEXACOPTER


def active_airframe() -> Airframe:
    """Airframe this process is flying: ``AAVC_AIRFRAME`` or DEFAULT_AIRFRAME.

    An unknown value falls back to the default rather than raising — a typo in
    an env var must not take down a mission that is otherwise ready to fly — but
    it is LOUD about it: the value picks the vehicle every plan is stamped
    with, so a silently ignored override is a silently wrong airframe.
    """
    raw = os.getenv("AAVC_AIRFRAME", "").strip().lower()
    if not raw:
        return DEFAULT_AIRFRAME
    try:
        return Airframe(raw)
    except ValueError:
        logger.warning(
            f"[schemas] AAVC_AIRFRAME={raw!r} is not a known airframe "
            f"({', '.join(a.value for a in Airframe)}) — flying as "
            f"{DEFAULT_AIRFRAME.value}. Gains, plant model and plan stamp all "
            "follow that, so fix the value if it was not what you meant.")
        return DEFAULT_AIRFRAME


class MissionCommand(BaseModel):
    model_config = ConfigDict(extra="forbid")

    seq: int = Field(..., ge=0)
    kind: CommandKind
    phase: MissionPhase
    coord: Coordinate | None = None
    # Physical airframe envelope. The tighter *operational* band (competition
    # 10-80 m, production 2-120 m) is enforced as a profile-driven clamp at
    # plan-build time, not here — widened from 10-80 in B2 so production mapping
    # (120 m) / precision-landing (2 m) plans validate while cached competition
    # plans (10-80) stay valid.
    altitude_m: float | None = Field(None, ge=2, le=120)
    speed_mps: float | None = Field(None, ge=0, le=20)
    duration_s: float | None = Field(None, ge=0)
    payload_id: int | None = Field(None, ge=0)
    # Per-target drop tracking for multi-tree delivery: each DROP_PAYLOAD carries a
    # distinct stop_index so it fires exactly once (vs the legacy single-payload
    # SAR drop, which uses stop 0). See orchestrator.state.dropped_stops.
    stop_index: int = Field(0, ge=0)
    # An operator-CONFIRMED drop (reviewed on the GCS) bypasses the
    # production-profile "suppress unconfirmed drop" guard in the mission loop.
    confirmed: bool = False
    notes: str = ""


class MissionPlan(BaseModel):
    """Pre-flight mission plan generated before the operation window."""

    model_config = ConfigDict(extra="forbid")

    mission_id: str
    airframe: Airframe = Field(
        default_factory=active_airframe,
        description="Vehicle class this plan targets. The executor builds "
        "airframe-appropriate PX4 mission items (VTOL transitions, FW takeoff "
        "direction). A plan deserialised without the field is assumed to be for "
        "the aircraft we currently fly — resolved per instance, since a static "
        "default would stamp the compile-time airframe onto a process that was "
        "told to fly a different one.",
    )
    expected_duration_s: float = Field(
        ..., gt=0, le=1800,
        description="Estimated total mission time (s). The AAVC 20-min window "
        "(1200 s) is a soft policy enforced at plan generation, NOT a hard "
        "schema bound — widened lab/research plans (e.g. large-area mapping) "
        "may legitimately run longer, up to this 1800 s ceiling.",
    )
    commands: list[MissionCommand] = Field(..., min_length=4)
    target_group_strategy: str = Field(
        ..., description="How the agent will discriminate the 'right' recipient group"
    )
    fallback_strategy: str = Field(
        ..., description="What to do if any phase fails (e.g. RTH on data-link loss)"
    )


# ---------------- Vision schemas ----------------


class DetectedTarget(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # clothing_color/pose predate the V1.3 ArUco-pad target (they described the
    # old human/mannequin mission); kept only so the flat dashboard
    # DetectedObjectEvent wire shape (§5) is unchanged — the pad path sets both
    # to "unknown".
    clothing_color: Literal["green camo", "blue camo", "other", "unknown"]
    pose: Literal["standing", "sitting", "prone", "unknown"]
    member_count: int = Field(..., ge=0, description="detections clustered within ~3 m")
    centroid_pixel_xy: tuple[int, int] | None = Field(
        None, description="Image-frame centroid (x, y) in pixels; None if too small to localise"
    )
    confidence: float = Field(..., ge=0.0, le=1.0)


class VisionAnalysis(BaseModel):
    """Single frame analysis result."""

    model_config = ConfigDict(extra="forbid")

    targets_detected: list[DetectedTarget]
    matches_designated_description: bool = Field(
        ..., description="True if any detected target matches the mission's target description"
    )
    matched_target_index: int | None = Field(
        None, description="Index into targets_detected of the best match, if any"
    )
    rationale: str
    confidence: float = Field(..., ge=0.0, le=1.0)
