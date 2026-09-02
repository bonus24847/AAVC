// Number formatters used by every gauge / panel.

export function fmtNum(v: number | null | undefined, digits = 1, suffix = ''): string {
  if (v === null || v === undefined || Number.isNaN(v)) return '—';
  return v.toFixed(digits) + suffix;
}

export function fmtInt(v: number | null | undefined, suffix = ''): string {
  if (v === null || v === undefined || Number.isNaN(v)) return '—';
  return Math.round(v).toString() + suffix;
}

export function fmtLatLon(v: number | null | undefined): string {
  if (v === null || v === undefined || Number.isNaN(v)) return '—';
  return v.toFixed(6);
}

export function fmtTime(secs: number): string {
  const s = Math.max(0, Math.floor(secs));
  const m = Math.floor(s / 60);
  const sec = s % 60;
  return `${m.toString().padStart(2, '0')}:${sec.toString().padStart(2, '0')}`;
}

export function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}

// Phase colour palette — pinned to the severity-led operational theme.
// Used by waypoint chips on the map. Same colour values are referenced
// from CSS variables in global.css so the map and the rest of the UI
// agree on what each phase looks like.
export function phaseColor(phase: string): string {
  switch (phase) {
    case 'preflight':       return '#a78bfa';   // violet — pre-arm readiness hold
    case 'takeoff':         return '#f59e0b';   // warning amber
    case 'transit_ingress': return '#22d3ee';   // info cyan
    case 'search':          return '#22c55e';   // nominal green
    case 'localize':        return '#facc15';   // caution yellow
    case 'drop':            return '#ef4444';   // critical red
    case 'transit_egress':  return '#22d3ee';
    case 'land':            return '#f59e0b';
    case 'rth':             return '#ef4444';
    case 'abort':           return '#7f1d1d';
    default:                return '#94a3b8';
  }
}
