// Mirror of dashboard/payloads.py — trimmed to the competition GCS.
// Only the event shapes the dashboard actually renders are kept:
// telemetry, vision, command, anomaly, drop-prediction, detected-object,
// command session/result, and the static plan/config payloads. Wizard,
// tuner, SDF-params, autotune/ESC-cal and agent-advisor types were dropped
// along with their widgets.

export interface TelemetryFrame {
  t_monotonic: number;
  lat: number | null;
  lon: number | null;
  alt_msl_m: number | null;
  alt_agl_m: number | null;
  ground_speed_mps: number | null;
  heading_deg: number | null;
  roll_deg: number | null;
  pitch_deg: number | null;
  roll_rate_dps: number | null;
  pitch_rate_dps: number | null;
  yaw_rate_dps: number | null;
  battery_percent: number | null;
  battery_voltage_v: number | null;
  battery_consumed_mah: number | null;
  battery_current_a: number | null;
  battery_capacity_mah: number | null;
  energy_tier: string;              // "A" measured | "B" estimated | "none"
  energy_sorties_left: number | null;
  sortie_energy_ok: boolean;
  gps_fix_type: number;
  gps_satellites: number;
  datalink_rssi: number;
  is_armed: boolean;
  flight_mode: string;
  phase: string;
  command_pointer: number;
  elapsed_s: number;
  remaining_s: number;
  terminal: string;
  airframe: string;            // running vehicle config (HEXACOPTER for AAVC)
  link_connected: boolean;     // MAVSDK heartbeat present
  // V1.3 multi-flight delivery: a FLIGHT is one arm->disarm cycle carrying
  // eggs_aboard eggs; a DELIVERY is one pad served within that flight.
  sortie_index?: number;               // 1-based current FLIGHT (0 = pre-mission)
  max_sorties?: number;                // flights the window plans for
  delivery_index?: number;             // 1-based DELIVERY across the mission
  max_deliveries?: number;             // pads to serve in the window
  eggs_aboard?: number;                // deliveries carried per flight
  flight_ids?: number[];               // THIS flight's assigned pad ids, in order
  assigned_marker_id?: number | null;  // the delivery's committee-assigned pad id
  assigned_id_queue?: number[];        // ordered 4-of-6 mission queue (W3)
  sortie_time_ok?: boolean;            // window covers another sortie (gate hint)
  servo_pwm_us: number[];
  esc_current_a: number[];
  esc_rpm: number[];
}

export interface VisionEvent {
  t_monotonic: number;
  phase: string;
  matches_designated_description: boolean;
  confidence: number;
  rationale: string;
  target_lat: number | null;
  target_lon: number | null;
}

export interface CommandEvent {
  t_monotonic: number;
  method: string;
  args: Record<string, unknown>;
  mavlink: string;
}

export interface AnomalyEvent {
  t_monotonic: number;
  message: string;
}

export interface DropPredictionEvent {
  t_monotonic: number;
  impact_lat: number;
  impact_lon: number;
  impact_t_s: number;
  horizontal_drift_m: number;
  trajectory: [number, number, number, number][];   // (t_s, lat, lon, alt_agl_m)
}

export interface DetectedObjectEvent {
  t_monotonic: number;
  label: string;
  clothing_color: string;       // raw class string — reused as the marker-colour key
  member_count: number;
  pose: string;
  confidence: number;
  lat: number;
  lon: number;
  is_designated_match: boolean;
}

export interface CommandSessionEvent {
  t_monotonic: number;
  armed: boolean;
  operator_note: string;
}

export interface CommandResultEvent {
  t_monotonic: number;
  command: string;
  ok: boolean;
  detail: string;
  operator_note: string;
}

// Pre-flight readiness board (orchestrator/preflight.py). Pushed at ~1 Hz while
// the mission holds in the PREFLIGHT phase awaiting the operator's GO.
export type PreflightStatus = 'pass' | 'warn' | 'fail' | 'pending';

export interface PreflightItem {
  id: string;
  label: string;
  status: PreflightStatus;
  critical: boolean;
  detail: string;
}

export interface PreflightReport {
  t_monotonic: number;
  all_critical_pass: boolean;
  items: PreflightItem[];
}

// ── System-ID + Autotune (pre-flight tuning module) ──
export interface SysIdStatus { state: string; axis: string; detail: string; }

export interface FrfDict {
  axis: string;
  f_hz: number[];
  mag: number[];
  phase_deg: number[];
  coherence: number[];
  note: string;
}

export interface PlantFitDict {
  axis: string;
  b: number | null;
  tau_eff_s: number | null;
  omega_n_rad_s: number | null;
  fit_kind: string;
  r2: number | null;
  coherence_med: number | null;
  f_band_hz: [number, number] | null;
  note: string;
}

export interface SysIdResult {
  t_monotonic: number;
  fits: { frf: FrfDict; fit: PlantFitDict }[];
  calibration: { airframe: string; b_measured: Record<string, number>; source: string };
}

export interface ComputedGainDict {
  param: string;
  value: number;
  loop: string;
  axis: string;
  formula: string;
  assumptions: string[];
}

export interface TunerDesign {
  airframe: string;
  gains: ComputedGainDict[];
  warnings: string[];
  plant_summary: Record<string, number>;
  calibration_source: string | null;
  measured_vs_model: Record<string, unknown>[];
}

export interface TunerParam { param: string; value: number | null; loop: string; axis: string; }
export interface TunerParamsSnapshot { t_monotonic: number; params: TunerParam[]; }

export interface TunerApplyResult {
  t_monotonic: number;
  ok: boolean;
  applied: { param: string; ok: boolean; value?: number; detail?: string }[];
  saved_to?: string | null;          // gains persisted → mission auto-applies
}

export interface AutotuneStatus { state: string; axis: string; progress_pct: number; detail: string; }

export interface HelloPayload {
  telemetry: TelemetryFrame;
  recent_vision: VisionEvent[];
  recent_commands: CommandEvent[];
  recent_anomalies: AnomalyEvent[];
  recent_drops: DropPredictionEvent[];
  recent_objects: DetectedObjectEvent[];
}

// WebSocket envelope kinds the dashboard subscribes to. The backend may
// broadcast additional kinds (wizard/tuner/etc.) for other tooling — the
// client simply ignores any kind it has no handler for.
export type WsKind =
  | 'hello' | 'telemetry' | 'vision'
  | 'command' | 'anomaly' | 'drop_prediction' | 'ping'
  | 'detected_object' | 'command_session' | 'command_result'
  | 'preflight' | 'plan_update'
  | 'sysid_status' | 'sysid_result' | 'tuner_design' | 'tuner_apply' | 'autotune_status';

// Blind search rebuilds the plan in flight (a serve pair per discovered target);
// the backend pushes the new plan + live command pointer so the map repaints.
export interface PlanUpdate {
  plan: MissionPlan;
  command_pointer: number;
}

export interface WsEnvelope {
  kind: WsKind;
  payload: unknown;
}

// Static site/airspace config (GET /api/config).
export interface AavcConfig {
  site: { center_lat?: number; center_lon?: number; ground_alt_m?: number; name?: string };
  controlled_airspace: [number, number][];
  search_area: [number, number][];
  emergency_egress: [number, number][];
  // V1.3 mandatory corridor P1→P2→P3 (plain [lat, lon] pairs from config).
  primary_transit_route: [number, number][];
  no_fly_zones: [number, number][][];
  marker?: Record<string, unknown>;
  cameras?: { nadir?: { fov_deg?: number; width_px?: number; height_px?: number } };
  ground_operation: { gcs_position?: [number, number]; launch_recovery_zone_radius_m?: number };
  mission: Record<string, unknown>;
}

export interface MissionCommand {
  seq: number;
  kind: string;          // TAKEOFF | GOTO | DROP_PAYLOAD | RTH
  phase: string;
  coord: { lat: number; lon: number; alt_m: number | null } | null;
  altitude_m: number | null;
  speed_mps: number | null;
  duration_s: number | null;
  payload_id: number | null;
  notes: string;
}

export interface MissionPlan {
  mission_id: string;
  airframe?: string;
  expected_duration_s: number;
  commands: MissionCommand[];
  target_group_strategy: string;
  fallback_strategy: string;
}
