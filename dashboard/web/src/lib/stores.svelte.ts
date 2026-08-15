// Svelte 5 reactive stores — module-scoped runes feeding every widget.
//
// Trimmed to the competition GCS: telemetry, the mission plan, static
// site config, the command/anomaly/vision/drop/detected-object logs, and
// the dashboard command session. Wizard / SDF-params /
// agent-advisor state was removed with their widgets.

import type {
  AavcConfig, AnomalyEvent, CommandEvent, CommandResultEvent,
  CommandSessionEvent, DetectedObjectEvent, DropPredictionEvent,
  HelloPayload, MissionPlan, PlanUpdate, PreflightReport,
  TelemetryFrame, VisionEvent,
} from './types';

import { CAM_H, CAM_W, NADIR_FOV_RAD, latLonToEnu } from './field';

export type ActiveView = 'flight' | 'tuning';

const SCAN_GRID_M = 5;            // coverage grid resolution
const SCAN_CELL_CAP = 4000;      // bound the coverage set (field ≈ 360×160 m)

const MAX_LOG = 200;
const MAX_TRACK_POINTS = 1500;
const OBJECT_DEDUPE_M = 4.0;          // cluster detections within ~4 m

function metersBetween(a: [number, number], b: [number, number]): number {
  // Equirectangular approx — adequate for sub-km clustering.
  const R = 6_378_137;
  const dLat = ((b[0] - a[0]) * Math.PI) / 180;
  const dLon = ((b[1] - a[1]) * Math.PI) / 180;
  const lat = ((a[0] + b[0]) / 2) * (Math.PI / 180);
  const x = dLon * Math.cos(lat);
  const y = dLat;
  return Math.sqrt(x * x + y * y) * R;
}

class MissionStore {
  telemetry = $state<TelemetryFrame | null>(null);
  plan = $state<MissionPlan | null>(null);
  config = $state<AavcConfig | null>(null);
  visionLog = $state<VisionEvent[]>([]);
  commandLog = $state<CommandEvent[]>([]);
  anomalyLog = $state<AnomalyEvent[]>([]);
  dropPredictions = $state<DropPredictionEvent[]>([]);
  actualTrack = $state<[number, number][]>([]);
  detectedObjects = $state<DetectedObjectEvent[]>([]);
  // Scan coverage — the nadir footprint accumulated into a 5 m ENU grid. The Set
  // is plain (not a rune); `scanVersion` bumps when it grows so MapView repaints
  // without copying the Set every 5 Hz telemetry frame.
  scannedCells = new Set<string>();
  scanVersion = $state(0);
  commandSession = $state<CommandSessionEvent>({
    t_monotonic: 0,
    armed: false,
    operator_note: '',
  });
  commandResults = $state<CommandResultEvent[]>([]);
  // Pre-flight readiness board — non-null while the mission holds for GO.
  preflight = $state<PreflightReport | null>(null);

  // Single view since the tuning module was removed (2026-08-15); kept so the
  // widgets that read it need no change.
  activeView = $state<ActiveView>('flight');

  // Geofence — separate store from config so the operator can read (and a
  // future API could edit) it without writing back to aavc_config.yaml.
  // Seeded from controlled_airspace on /api/config load.
  geofence = $state<{ vertices: [number, number][]; edited: boolean }>({
    vertices: [],
    edited: false,
  });

  applyHello(p: HelloPayload): void {
    this.telemetry = p.telemetry;
    this.visionLog = p.recent_vision.slice(-MAX_LOG);
    this.commandLog = p.recent_commands.slice(-MAX_LOG);
    this.anomalyLog = p.recent_anomalies.slice(-MAX_LOG);
    this.dropPredictions = p.recent_drops ?? [];
    this.detectedObjects = p.recent_objects ?? [];
    if (p.telemetry.lat !== null && p.telemetry.lon !== null) {
      this.actualTrack = [[p.telemetry.lat, p.telemetry.lon]];
    }
  }

  applyTelemetry(t: TelemetryFrame): void {
    this.telemetry = t;
    if (t.lat !== null && t.lon !== null) {
      const last = this.actualTrack[this.actualTrack.length - 1];
      const lat = t.lat, lon = t.lon;
      if (!last || Math.abs(last[0] - lat) > 1e-7 || Math.abs(last[1] - lon) > 1e-7) {
        const track = this.actualTrack.slice();
        track.push([lat, lon]);
        if (track.length > MAX_TRACK_POINTS) track.shift();
        this.actualTrack = track;
      }
    }
    this._accumulateCoverage(t);
  }

  // Mark the nadir camera footprint as scanned: a rectangle of half-width
  // alt·tan(FOV/2) (height scaled by the sensor aspect) quantised to the grid.
  // Intrinsics come from /api/config cameras.nadir (the measured lens) with the
  // field.ts constants as the pre-config fallback.
  private _accumulateCoverage(t: TelemetryFrame): void {
    const site = this.config?.site;
    if (!site || site.center_lat == null || site.center_lon == null) return;
    if (t.lat === null || t.lon === null || t.alt_agl_m === null || t.alt_agl_m < 1.0) return;
    if (this.scannedCells.size >= SCAN_CELL_CAP) return;
    const nadirCfg = this.config?.cameras?.nadir;
    const fovRad = nadirCfg?.fov_deg != null
      ? (nadirCfg.fov_deg * Math.PI) / 180 : NADIR_FOV_RAD;
    const camW = nadirCfg?.width_px ?? CAM_W;
    const camH = nadirCfg?.height_px ?? CAM_H;
    const halfW = t.alt_agl_m * Math.tan(fovRad / 2);
    const halfH = halfW * (camH / camW);
    const [e, n] = latLonToEnu(t.lat, t.lon, site.center_lat, site.center_lon);
    let added = false;
    for (let de = -halfW; de <= halfW; de += SCAN_GRID_M) {
      for (let dn = -halfH; dn <= halfH; dn += SCAN_GRID_M) {
        const key = `${Math.round((e + de) / SCAN_GRID_M)},${Math.round((n + dn) / SCAN_GRID_M)}`;
        if (!this.scannedCells.has(key)) { this.scannedCells.add(key); added = true; }
      }
    }
    if (added) this.scanVersion++;
  }

  appendVision(ev: VisionEvent): void {
    this.visionLog = [...this.visionLog, ev].slice(-MAX_LOG);
  }

  appendCommand(ev: CommandEvent): void {
    this.commandLog = [...this.commandLog, ev].slice(-MAX_LOG);
  }

  appendAnomaly(ev: AnomalyEvent): void {
    this.anomalyLog = [...this.anomalyLog, ev].slice(-MAX_LOG);
  }

  appendDropPrediction(ev: DropPredictionEvent): void {
    // Bound — predictions fire on every DROP_PAYLOAD. MapView rebuilds all
    // trajectory features each render, so keeping this small bounds paint cost.
    this.dropPredictions = [...this.dropPredictions, ev].slice(-MAX_LOG);
  }

  appendDetectedObject(ev: DetectedObjectEvent): void {
    // Merge close-by detections of the same class into one map pin — the
    // vision worker fires per-frame, so multiple frames over the same target
    // would otherwise stack dozens of identical markers.
    const next = this.detectedObjects.slice();
    let merged = false;
    for (let i = 0; i < next.length; i++) {
      const cur = next[i];
      if (cur.clothing_color !== ev.clothing_color) continue;
      const d = metersBetween([cur.lat, cur.lon], [ev.lat, ev.lon]);
      if (d <= OBJECT_DEDUPE_M) {
        const keep = ev.confidence >= cur.confidence ? ev : cur;
        next[i] = {
          ...keep,
          t_monotonic: ev.t_monotonic,
          is_designated_match: cur.is_designated_match || ev.is_designated_match,
        };
        merged = true;
        break;
      }
    }
    if (!merged) next.push(ev);
    this.detectedObjects = next.slice(-MAX_LOG);
  }

  applyCommandSession(ev: CommandSessionEvent): void {
    this.commandSession = ev;
  }

  appendCommandResult(ev: CommandResultEvent): void {
    this.commandResults = [...this.commandResults, ev].slice(-MAX_LOG);
  }

  applyPreflight(p: PreflightReport): void {
    this.preflight = p;
  }

  // Blind search rebuilds the plan in flight; swap it in so MapView repaints the
  // sweep + the serve pins. (command_pointer also rides telemetry frames.)
  applyPlanUpdate(p: PlanUpdate): void {
    this.plan = p.plan;
  }

  setActiveView(v: ActiveView): void { this.activeView = v; }

  setPlan(p: MissionPlan): void { this.plan = p; }

  setConfig(c: AavcConfig): void {
    this.config = c;
    if (Array.isArray(c.controlled_airspace) && this.geofence.vertices.length === 0) {
      this.setGeofenceVertices(c.controlled_airspace);
    }
  }

  setGeofenceVertices(vs: [number, number][]): void {
    this.geofence = {
      vertices: vs.map((v) => [v[0], v[1]] as [number, number]),
      edited: false,
    };
  }
}

export const mission = new MissionStore();

// One attempt; true once the config (the map's georeference) has been applied.
// /api/plan may legitimately 404 before the first GO — the plan also arrives
// live via the plan_update WS event, so it never blocks the retry loop.
async function fetchStatic(): Promise<boolean> {
  try {
    const [planRes, cfgRes] = await Promise.all([
      fetch('/api/plan'),
      fetch('/api/config'),
    ]);
    if (planRes.ok) mission.setPlan(await planRes.json());
    if (cfgRes.ok) mission.setConfig(await cfgRes.json());
    return cfgRes.ok;
  } catch (e) {
    console.warn('[stores] static fetch failed:', e);
    return false;
  }
}

let staticInflight = false;

// Retry until the config arrives. The page is routinely opened while the
// stack is still booting (the launcher takes ~1-2 min) or refreshed across an
// orchestrator restart — the old one-shot fetch left the map georeferenced to
// the DEFAULT site forever while the WS quietly reconnected (2026-07-17).
export async function loadStatic(): Promise<void> {
  if (staticInflight) return;
  staticInflight = true;
  try {
    while (!(await fetchStatic())) {
      await new Promise((r) => setTimeout(r, 2000));
    }
  } finally {
    staticInflight = false;
  }
}

// A WS (re)connect means the server may have restarted — or this tab missed
// plan_update events while disconnected. Refetch both statics once.
export function refreshStatic(): void { void loadStatic(); }
