// Mirror of dashboard/payloads.py — trimmed to the competition GCS.
// Only the event shapes the dashboard actually renders are kept:
// telemetry, vision, command, anomaly, drop-prediction, detected-object,
// command session/result, and the static plan/config payloads. Wizard,
// tuner, SDF-params, autotune/ESC-cal and agent-advisor types were dropped
// (the System-ID block went with the tuning module, 2026-08-15)
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

