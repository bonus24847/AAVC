// Wrapper around fetch() for the POST /api/cmd/* endpoints. Always
// adds the X-AAVC-CMD header (server rejects without it as a CSRF
// mitigation — see dashboard/commands.py).

const CMD_HEADER = 'X-AAVC-CMD';
const CMD_VALUE = '1';

export interface CmdResponse {
  ok: boolean;
  detail?: string;
  status: number;
}

async function post(path: string, body: object): Promise<CmdResponse> {
  try {
    const resp = await fetch(path, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        [CMD_HEADER]: CMD_VALUE,
      },
      body: JSON.stringify(body),
    });
    let payload: { detail?: string; ok?: boolean } = {};
    try { payload = await resp.json(); } catch { /* empty / non-JSON */ }
    return {
      ok: resp.ok && (payload.ok ?? true),
      detail: payload.detail ?? (resp.ok ? '' : `HTTP ${resp.status}`),
      status: resp.status,
    };
  } catch (e: unknown) {
    const msg = e instanceof Error ? e.message : String(e);
    return { ok: false, detail: msg, status: 0 };
  }
}

// Session arm/disarm — gates whether the FastAPI endpoints accept commands.
export const cmdArm = (operator_note: string) =>
  post('/api/cmd/arm', { operator_note });
export const cmdDisarm = (operator_note: string = '') =>
  post('/api/cmd/disarm', { operator_note });

// Vehicle arming (PX4 motors) — distinct from the session arm above.
export const cmdVehicleArm = (operator_note: string = '') =>
  post('/api/cmd/vehicle_arm', { operator_note });
export const cmdVehicleDisarm = (operator_note: string = '') =>
  post('/api/cmd/vehicle_disarm', { operator_note });

// Mission verbs
export const cmdTakeoff = (operator_note: string = '') =>
  post('/api/cmd/takeoff', { operator_note });
export const cmdHold = (operator_note: string = '') =>
  post('/api/cmd/hold', { operator_note });
export const cmdResume = (operator_note: string = '') =>
  post('/api/cmd/resume', { operator_note });
export const cmdRTL = (operator_note: string = '') =>
  post('/api/cmd/rtl', { operator_note });
export const cmdLand = (operator_note: string = '') =>
  post('/api/cmd/land', { operator_note });
export const cmdAbort = (operator_note: string = '') =>
  post('/api/cmd/abort', { operator_note });
// Manual egg release at the current target. Backend dispatches
// commander.drop_payload() — touchdown-gated (refused airborne unless forced).
export const cmdDrop = (operator_note: string = '') =>
  post('/api/cmd/drop', { operator_note });
// Kill switch — cut motors instantly (action.kill). The vehicle drops.
export const cmdKill = (operator_note: string = '') =>
  post('/api/cmd/kill', { operator_note });

// Ordered 4-of-6 mission queue — which pads the drone serves, in sortie
// order. ids=[] clears. Editable mid-mission: applies at the next PREFLIGHT
// hold (or the current one — GO resolves at click time). The per-sortie GO
// click remains the egg-loaded / crew-clear acknowledgment.
export const cmdMissionIds = (ids: number[], operator_note: string = '') =>
  post('/api/cmd/mission_ids', { ids, operator_note });

// Per-sortie launch authorization — releases the mission's PREFLIGHT hold.
// The backend fires only on a green board (all critical checks pass);
// `payload_confirmed` acknowledges the egg-loaded advisory and
// `assigned_marker_id` is THIS sortie's pad id (0-6) — null resolves it
// from the mission queue (a non-null id overrides the queue for this
// sortie only). `force` overrides the window-too-short refusal.
export const cmdPreflightGo = (
  payload_confirmed: boolean, assigned_marker_id: number | null,
  force: boolean = false, operator_note: string = '',
) =>
  post('/api/cmd/preflight/go',
       { payload_confirmed, assigned_marker_id, force, operator_note });




// Live FC gains snapshot (GET).
