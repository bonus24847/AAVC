// Synthetic AAVC field geometry for the offline map — the competition site bans
// internet, so the map is drawn from these constants rather than satellite tiles.
//
// Positions are local ENU metres, mirrored from sitl/worlds/aavc_field.sdf; keep
// them in sync with that world file. Georeferenced at runtime to the config site
// centre via enuToLatLon. Camera constants mirror vision/projection.py (used by
// the scan-coverage footprint in stores.svelte.ts).

const R_EARTH = 6_378_137;
const DEG = Math.PI / 180;

export function enuToLatLon(eastM: number, northM: number, lat0: number, lon0: number): [number, number] {
  const lat = lat0 + (northM / R_EARTH) / DEG;
  const lon = lon0 + (eastM / (R_EARTH * Math.cos(lat0 * DEG))) / DEG;
  return [lat, lon];
}

export function latLonToEnu(lat: number, lon: number, lat0: number, lon0: number): [number, number] {
  const east = (lon - lon0) * DEG * R_EARTH * Math.cos(lat0 * DEG);
  const north = (lat - lat0) * DEG * R_EARTH;
  return [east, north];
}

// Ground plane (sitl/worlds/aavc_field.sdf: model ground_plane @ pose 115 0 0,
// 360×160). The 230×60 m competition field sits inside it.
export const GROUND_PLANE = { cx: 115, cy: 0, w: 360, h: 160 };
export const FIELD = { x0: 0, x1: 230, y0: -30, y1: 30 };
export const LAUNCH_PAD = { x: 0, y: 0, size: 10 };

export type SceneryKind = 'tree' | 'house' | 'car';
export const SCENERY: { kind: SceneryKind; x: number; y: number }[] = [
  // Trees — north edge
  { kind: 'tree', x: 5, y: 39 }, { kind: 'tree', x: 35, y: 42 }, { kind: 'tree', x: 65, y: 38 },
  { kind: 'tree', x: 95, y: 43 }, { kind: 'tree', x: 125, y: 39 }, { kind: 'tree', x: 155, y: 44 },
  { kind: 'tree', x: 185, y: 38 }, { kind: 'tree', x: 215, y: 42 },
  // Trees — south edge
  { kind: 'tree', x: 20, y: -40 }, { kind: 'tree', x: 55, y: -43 }, { kind: 'tree', x: 90, y: -38 },
  { kind: 'tree', x: 125, y: -42 }, { kind: 'tree', x: 160, y: -39 }, { kind: 'tree', x: 200, y: -41 },
  // Houses — corners + one north of centre
  { kind: 'house', x: -12, y: 42 }, { kind: 'house', x: -12, y: -42 },
  { kind: 'house', x: 244, y: 40 }, { kind: 'house', x: 244, y: -40 }, { kind: 'house', x: 130, y: 52 },
  // Cars — launch zone + south "road"
  { kind: 'car', x: -9, y: 9 }, { kind: 'car', x: -9, y: -9 }, { kind: 'car', x: -13, y: 0 },
  { kind: 'car', x: 40, y: -34 }, { kind: 'car', x: 110, y: -34 }, { kind: 'car', x: 180, y: -34 },
];

// Nadir camera (mirror vision/projection.py CAMERA_FOV_RAD / WIDTH / HEIGHT).
export const NADIR_FOV_RAD = 1.74;
export const CAM_W = 1280;
export const CAM_H = 720;

// Build a closed [lon,lat][] ring for an axis-aligned ENU rectangle (centre+size).
export function enuRect(
  cx: number, cy: number, w: number, h: number, lat0: number, lon0: number,
): [number, number][] {
  const c: [number, number][] = [
    [cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
    [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2],
  ].map(([e, n]) => {
    const [lat, lon] = enuToLatLon(e, n, lat0, lon0);
    return [lon, lat] as [number, number];
  });
  return [...c, c[0]];
}
