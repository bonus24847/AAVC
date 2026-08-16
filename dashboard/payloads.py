"""Wire-format Pydantic models for /ws/realtime + /api endpoints.

These are intentionally separate from `mission_brain.schemas` —
those are the validation schemas the mission core produces; these are
the flat UI-facing snapshots the browser consumes. Keeping them
separate lets either evolve independently.

Trimmed to the lightweight competition GCS set: telemetry, command
trace, command session/result, anomalies, drop predictions, detected
objects, plus a minimal vision event the realtime fan-out needs.
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict


class TelemetryFrame(BaseModel):
    """5 Hz unified telemetry snapshot (everything the gauges/instruments need)."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float                              # seconds since process start
    lat: float | None = None
    lon: float | None = None
    alt_msl_m: float | None = None
    alt_agl_m: float | None = None
    ground_speed_mps: float | None = None
    heading_deg: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    roll_rate_dps: float | None = None
    pitch_rate_dps: float | None = None
    yaw_rate_dps: float | None = None
    battery_percent: float | None = None
    battery_voltage_v: float | None = None
    battery_consumed_mah: float | None = None
    battery_current_a: float | None = None
    battery_capacity_mah: float | None = None
    # Which tier the energy number came from: "A" = the power module's coulomb
    # count, "B" = derived from percentage (much coarser), "none" = no signal.
    # Shown on the GCS: an estimate presented as a measurement is worse than no
    # number at all.
    energy_tier: str = "none"
    energy_sorties_left: float | None = None
    sortie_energy_ok: bool = True
    gps_fix_type: int = 0
    gps_satellites: int = 0
    datalink_rssi: int = -1
    is_armed: bool = False
    flight_mode: str = "UNKNOWN"
    phase: str = "takeoff"
    command_pointer: int = 0
    elapsed_s: float = 0.0
    remaining_s: float = 0.0
    terminal: str = "running"
    airframe: str = "hexacopter"           # running vehicle config
    link_connected: bool = False           # MAVSDK heartbeat present
    # ── V1.3 multi-flight delivery (a FLIGHT is one arm→disarm cycle carrying
    # eggs_aboard eggs; a DELIVERY is one pad served within it) ──
    sortie_index: int = 0                  # 1-based current FLIGHT (0 = pre-mission)
    max_sorties: int = 0                   # flights the window plans for
    delivery_index: int = 0                # 1-based DELIVERY across the mission
    max_deliveries: int = 0                # pads to serve in the window
    eggs_aboard: int = 1                   # deliveries carried per flight
    flight_ids: list[int] = []             # THIS flight's assigned pad ids, in order
    assigned_marker_id: int | None = None  # the delivery's committee-assigned pad id
    assigned_id_queue: list[int] = []      # ordered 4-of-6 mission queue (W3)
    sortie_time_ok: bool = True            # window covers another sortie (gate hint)
    servo_pwm_us: list[int] = []
    esc_current_a: list[float] = []
    esc_rpm: list[int] = []


class VisionEvent(BaseModel):
    """One observation from the vision worker (realtime fan-out only)."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    phase: str
    matches_designated_description: bool
    confidence: float
    rationale: str
    target_lat: float | None = None
    target_lon: float | None = None


# Educational map: each high-level commander verb → the MAVLink message(s) it
# actually puts on the wire to the flight controller. Surfaced in the GCS
# MAVLink panel so an operator can study how a command translates to MAVLink
# and correlate it with the resulting telemetry. Unknown verb → "" (hidden).
MAVLINK_FOR_COMMAND: dict[str, str] = {
    "arm_and_takeoff":         "MAV_CMD_COMPONENT_ARM_DISARM → MAV_CMD_NAV_TAKEOFF",
    "goto":                    "MAV_CMD_DO_REPOSITION  (COMMAND_INT)",
    "hover":                   "MAV_CMD_DO_REPOSITION  (loiter)",
    "land":                    "MAV_CMD_NAV_LAND",
    "rth":                     "MAV_CMD_NAV_RETURN_TO_LAUNCH",
    "drop_payload":            "MAV_CMD_DO_SET_ACTUATOR  (→ DO_SET_SERVO fallback)",
    "abort":                   "MAV_CMD_COMPONENT_ARM_DISARM  (force-disarm / kill)",
    "connect":                 "HEARTBEAT  (link established)",
    "upload_geofence":         "MISSION_COUNT / MISSION_ITEM_INT  (MAV_MISSION_TYPE_FENCE)",
    "set_geofence_action_rtl": "PARAM_SET  GF_ACTION=3 (Return)",
    "set_datalink_loss_rtl":   "PARAM_SET  NAV_DLL_ACT=2 (RTL)",
    "run_mission":             "MISSION_COUNT / MISSION_ITEM_INT → MAV_CMD_MISSION_START",
    # Manual verbs from the dashboard command bar (recorded as dashboard_<verb>).
    "dashboard_arm":           "MAV_CMD_COMPONENT_ARM_DISARM  (arm)",
    "dashboard_disarm":        "MAV_CMD_COMPONENT_ARM_DISARM  (disarm)",
    "dashboard_takeoff":       "MAV_CMD_NAV_TAKEOFF",
    "dashboard_land":          "MAV_CMD_NAV_LAND",
    "dashboard_rtl":           "MAV_CMD_NAV_RETURN_TO_LAUNCH",
    "dashboard_hold":          "MAV_CMD_DO_REPOSITION  (hold)",
    "dashboard_pause":         "MAV_CMD_DO_PAUSE_CONTINUE  (pause)",
    "dashboard_resume":        "MAV_CMD_DO_PAUSE_CONTINUE  (resume)",
    "dashboard_drop":          "MAV_CMD_DO_SET_ACTUATOR",
    "dashboard_abort":         "MAV_CMD_COMPONENT_ARM_DISARM  (kill)",
    "dashboard_kill":          "MAV_CMD_COMPONENT_ARM_DISARM  (force-kill motors)",
    "dashboard_vehicle_arm":   "MAV_CMD_COMPONENT_ARM_DISARM  (arm)",
    "dashboard_vehicle_disarm": "MAV_CMD_COMPONENT_ARM_DISARM  (disarm)",
}


class CommandEvent(BaseModel):
    """One MAVLink command dispatched via the LoggedCommander wrapper."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    method: str            # arm_and_takeoff / goto / hover / drop_payload / land / rth
    args: dict[str, Any]   # method kwargs (flat-serialised)
    mavlink: str = ""      # the MAVLink message(s) this verb puts on the wire


class AnomalyEvent(BaseModel):
    """One anomaly recorded on state.anomalies."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    message: str


class DropPredictionEvent(BaseModel):
    """Output of orchestrator.drop_trajectory.predict, pushed when DROP_PAYLOAD fires."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    impact_lat: float
    impact_lon: float
    impact_t_s: float
    horizontal_drift_m: float
    trajectory: list[tuple[float, float, float, float]]   # (t_s, lat, lon, alt_agl_m)


class DetectedObjectEvent(BaseModel):
    """Per-target snapshot carrying the world-frame coordinate so the map can
    drop a labelled pin. `label` is the human-readable class.

    TWO producers feed this same event type, deliberately not unified (§5 keeps
    the wire shape stable):
      * orchestrator.main._confirm_pusher — one event per tracker CONFIRMED pad
        (pose="confirmed"), fired once when a pad is confirmed mid-sweep. Drives
        the map pins + the "Confirmed pads" readout.
      * orchestrator.vision_worker._detected_object_events — the raw per-frame
        detections (pose="unknown"), higher rate. `is_designated_match` =
        decoded id is one of THIS FLIGHT's assigned ids (state.flight_ids —
        eggs_aboard can be > 1; falls back to the single state.assigned_marker_id
        pre-flight / when flight_ids is empty).
    """

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    label: str                                    # "aruco pad <id>" / "landing pad"
    clothing_color: str                           # raw class for filter coloring
    member_count: int
    pose: str
    confidence: float
    lat: float                                    # WGS84
    lon: float
    is_designated_match: bool = False             # the target the mission cares about


class CommandSessionEvent(BaseModel):
    """Arm/disarm state change for the dashboard command channel.

    The session is GUI-only — it gates whether the FastAPI endpoints will
    accept commands. PX4 has its own arming state. Two independent guards:
    the operator must explicitly arm the dashboard *and* the aircraft must
    be in a state where the command makes sense.
    """

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    armed: bool
    operator_note: str = ""           # what the operator typed when arming


class CommandResultEvent(BaseModel):
    """Outcome of a dashboard-originated command (Hold/Resume/RTL/Land/Drop/…)."""

    model_config = ConfigDict(extra="forbid")

    t_monotonic: float
    command: str                      # "hold" | "resume" | "rtl" | "land" | "drop" | …
    ok: bool
    detail: str = ""                  # error text if ok=False, ack text otherwise
    operator_note: str = ""


class WsEnvelope(BaseModel):
    """Single message over the websocket."""

    model_config = ConfigDict(extra="forbid")

    kind: Literal[
        "telemetry", "vision", "command", "anomaly",
        "drop_prediction", "hello",
        "detected_object", "command_session", "command_result",
        "preflight", "plan_update",
        "ping",
    ]
    payload: dict[str, Any]
