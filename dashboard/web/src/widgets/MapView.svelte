<script lang="ts">
  // 2D tactical map for the AAVC field. Fully OFFLINE — the competition site
  // bans internet, so the basemap is a SYNTHETIC field drawn from the world-file
  // layout (lib/field.ts), georeferenced to the config site centre. Paints:
  //   - the synthetic field: ground plane, 230×60 m competition field, launch
  //     pad, scenery (trees/houses/cars), and a scan-coverage shade (grey =
  //     ground the nadir camera has already swept)
  //   - the geofence (controlled airspace) + AAVC search sub-region
  //   - the planned flight path + waypoint chips (TAKEOFF / search GOTO / RTH)
  //   - the live drone position + heading reticle and its flown track
  //   - TARGETS / payload drop points: an amber ring appears the moment the
  //     target tracker CONFIRMS a body during the sweep, and turns solid blue
  //     once a payload is released there
  // Raw per-frame vision detections are deliberately NOT drawn — they flicker
  // and clutter the map; only tracker-confirmed targets appear.

  import { onMount, onDestroy } from 'svelte';
  import maplibregl from 'maplibre-gl';
  import { mission } from '../lib/stores.svelte';
  import { phaseColor } from '../lib/format';
  import {
    FIELD, GROUND_PLANE, LAUNCH_PAD, SCENERY, enuRect, enuToLatLon,
  } from '../lib/field';

  const SITE_DEFAULT: [number, number] = [14.6525, 101.1875];   // [lat, lon] fallback
  function siteCenter(): [number, number] {
    const s = mission.config?.site;
    return (s && s.center_lat != null && s.center_lon != null)
      ? [s.center_lat, s.center_lon] : SITE_DEFAULT;
  }
  // Site key ("lat,lon") the synthetic field was last built at. The config can
  // arrive AFTER the map loads (slow fetch, server still booting) — the field
  // must relocate from the SITE_DEFAULT fallback to the real site when it does,
  // so this is a rebuild-on-change key, not a one-shot flag (2026-07-17).
  let builtSiteKey: string | null = null;

  let container: HTMLDivElement;
  let map: maplibregl.Map | null = null;
  let mapReady = $state(false);
  let droneMarker: maplibregl.Marker | null = null;
  // Plan waypoint chips (TAKEOFF / GOTO target / RTH) keyed by seq.
  let waypointMarkers: Map<number, { marker: maplibregl.Marker; el: HTMLDivElement }> = new Map();
  // Target / payload drop markers — amber ring from the moment a target is
  // confirmed (detected_object events), solid blue once a payload is released
  // there. Rebuilt from confirmed targets ∪ the live plan's DROP_PAYLOAD points.
  let dropMarkers: maplibregl.Marker[] = [];

  // Offline synthetic basemap — just a dark field-surround background colour; the
  // field itself is drawn as GeoJSON layers (buildSyntheticField). No tiles, so
  // the map is never black even with zero network.
  const STYLE = {
    version: 8 as const,
    sources: {},
    layers: [
      { id: 'bg', type: 'background' as const, paint: { 'background-color': '#0a140f' } },
    ],
  };

  onMount(() => {
    const [lat0, lon0] = siteCenter();
    map = new maplibregl.Map({
      container,
      style: STYLE,
      center: [lon0, lat0],
      zoom: 16,
      attributionControl: { compact: true },
    });
    map.addControl(new maplibregl.NavigationControl(), 'top-right');
    map.on('load', () => {
      mapReady = true;
      setupSources();
      droneMarker = makeDroneMarker();
    });
  });

  onDestroy(() => {
    map?.remove();
  });

  function setupSources() {
    if (!map) return;
    for (const id of [
      'field-ground', 'field-comp', 'field-grid', 'launch-pad', 'scenery', 'scanned-coverage',
      'plan-path', 'actual-track', 'controlled-airspace', 'search-area',
      'no-fly-zones', 'transit-route',
    ]) {
      map.addSource(id, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
    }
    // ── synthetic field (drawn first, so everything else sits on top) ──
    map.addLayer({
      id: 'field-ground-fill', source: 'field-ground', type: 'fill',
      paint: { 'fill-color': '#12251a' },                                   // dark surround
    });
    map.addLayer({
      id: 'field-comp-fill', source: 'field-comp', type: 'fill',
      paint: { 'fill-color': '#1c3d28', 'fill-opacity': 0.9 },              // mown competition field
    });
    map.addLayer({
      id: 'field-comp-line', source: 'field-comp', type: 'line',
      paint: { 'line-color': '#3f6b4d', 'line-width': 1.2 },
    });
    map.addLayer({
      id: 'field-grid-line', source: 'field-grid', type: 'line',
      paint: { 'line-color': '#2a4633', 'line-width': 0.6 },                // 50 m scale grid
    });
    map.addLayer({
      id: 'scanned-coverage-fill', source: 'scanned-coverage', type: 'fill',
      paint: { 'fill-color': '#94a3b8', 'fill-opacity': 0.22 },             // grey = scanned ground
    });
    map.addLayer({
      id: 'launch-pad-fill', source: 'launch-pad', type: 'fill',
      paint: { 'fill-color': '#475569', 'fill-opacity': 0.7 },
    });
    map.addLayer({
      id: 'scenery-pt', source: 'scenery', type: 'circle',
      paint: {
        'circle-radius': ['match', ['get', 'kind'], 'house', 5, 'car', 3, 4],
        'circle-color': ['match', ['get', 'kind'], 'house', '#64748b', 'car', '#f59e0b', '#15803d'],
        'circle-opacity': 0.85,
        'circle-stroke-width': 0.5, 'circle-stroke-color': '#0a140f',
      },
    });
    map.addLayer({
      id: 'controlled-airspace-fill', source: 'controlled-airspace', type: 'fill',
      paint: { 'fill-color': '#f59e0b', 'fill-opacity': 0.05 },
    });
    map.addLayer({
      id: 'controlled-airspace-line', source: 'controlled-airspace', type: 'line',
      paint: { 'line-color': '#f59e0b', 'line-width': 1.5, 'line-dasharray': [3, 2] },
    });
    map.addLayer({
      id: 'search-area-fill', source: 'search-area', type: 'fill',
      paint: { 'fill-color': '#22d3ee', 'fill-opacity': 0.07 },
    });
    map.addLayer({
      id: 'search-area-line', source: 'search-area', type: 'line',
      paint: { 'line-color': '#22d3ee', 'line-width': 1.5 },
    });
    // V1.3: no-fly zones (entry prohibited) + the mandatory transit corridor.
    map.addLayer({
      id: 'no-fly-fill', source: 'no-fly-zones', type: 'fill',
      paint: { 'fill-color': '#ef4444', 'fill-opacity': 0.16 },
    });
    map.addLayer({
      id: 'no-fly-line', source: 'no-fly-zones', type: 'line',
      paint: { 'line-color': '#ef4444', 'line-width': 1.5, 'line-dasharray': [2, 2] },
    });
    map.addLayer({
      id: 'transit-route-line', source: 'transit-route', type: 'line',
      paint: { 'line-color': '#818cf8', 'line-width': 2, 'line-dasharray': [4, 2] },
    });
    map.addLayer({
      id: 'transit-route-pt', source: 'transit-route', type: 'circle',
      paint: {
        'circle-radius': 5, 'circle-color': '#818cf8',
        'circle-stroke-width': 1.5, 'circle-stroke-color': '#0a140f',
      },
    });
    map.addLayer({
      id: 'plan-path-line', source: 'plan-path', type: 'line',
      paint: { 'line-color': '#f59e0b', 'line-width': 2, 'line-opacity': 0.6, 'line-dasharray': [2, 1] },
    });
    map.addLayer({
      id: 'actual-track-line', source: 'actual-track', type: 'line',
      paint: { 'line-color': '#22c55e', 'line-width': 3 },
    });
    // Payload drop points are drawn as DOM markers (amber pending / blue dropped)
    // in the drop-points effect below — no GeoJSON layer needed here.
  }

  function makeDroneMarker(): maplibregl.Marker {
    const el = document.createElement('div');
    // Phosphor-amber bracketed reticle. Heading-rotation lives on an inner
    // wrapper so MapLibre's outer translate transform is never compounded.
    el.style.transformOrigin = 'center center';
    el.style.filter = 'drop-shadow(0 0 6px rgba(245, 158, 11, 0.7))';
    const inner = document.createElement('div');
    inner.className = 'aavc-heading-inner';
    inner.style.transformOrigin = 'center center';
    inner.innerHTML = `<svg width="32" height="32" viewBox="-16 -16 32 32">
      <circle r="13" fill="rgba(245,158,11,0.12)" stroke="#f59e0b" stroke-width="1.4"/>
      <line x1="0" y1="-13" x2="0" y2="-5" stroke="#f59e0b" stroke-width="2.2"/>
      <line x1="-13" y1="0" x2="-10" y2="0" stroke="#f59e0b" stroke-width="1.2"/>
      <line x1="13" y1="0" x2="10" y2="0" stroke="#f59e0b" stroke-width="1.2"/>
      <line x1="0" y1="13" x2="0" y2="10" stroke="#f59e0b" stroke-width="1.2"/>
      <circle r="2.5" fill="#f59e0b"/>
    </svg>`;
    el.appendChild(inner);
    return new maplibregl.Marker({ element: el }).setLngLat([101.19148, 14.65141]).addTo(map!);
  }

  function setSourceData(id: string, data: GeoJSON.FeatureCollection) {
    if (!map || !mapReady) return;
    (map.getSource(id) as maplibregl.GeoJSONSource | undefined)?.setData(data as any);
  }

  function polygonFeature(pts: [number, number][]): GeoJSON.FeatureCollection {
    return {
      type: 'FeatureCollection',
      features: [{
        type: 'Feature',
        geometry: {
          type: 'Polygon',
          // close the ring + swap to [lon, lat]
          coordinates: [[...pts.map(([la, lo]) => [lo, la]), [pts[0][1], pts[0][0]]]],
        },
        properties: {},
      }],
    };
  }

  function rectFeature(ring: [number, number][]): GeoJSON.Feature {
    return { type: 'Feature', geometry: { type: 'Polygon', coordinates: [ring] }, properties: {} };
  }

  // Build the static synthetic field (ground plane, competition field, 50 m grid,
  // launch pad, scenery) once the site centre is known, then frame the view.
  function buildSyntheticField() {
    if (!map || !mapReady) return;
    const [lat0, lon0] = siteCenter();
    const siteKey = `${lat0},${lon0}`;
    if (siteKey === builtSiteKey) return;
    setSourceData('field-ground', {
      type: 'FeatureCollection',
      features: [rectFeature(enuRect(GROUND_PLANE.cx, GROUND_PLANE.cy, GROUND_PLANE.w, GROUND_PLANE.h, lat0, lon0))],
    });
    const fcx = (FIELD.x0 + FIELD.x1) / 2, fcy = (FIELD.y0 + FIELD.y1) / 2;
    setSourceData('field-comp', {
      type: 'FeatureCollection',
      features: [rectFeature(enuRect(fcx, fcy, FIELD.x1 - FIELD.x0, FIELD.y1 - FIELD.y0, lat0, lon0))],
    });
    // 50 m scale grid across the competition field.
    const grid: GeoJSON.Feature[] = [];
    for (let x = FIELD.x0; x <= FIELD.x1; x += 50) {
      grid.push({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates:
        [enuToLatLon(x, FIELD.y0, lat0, lon0), enuToLatLon(x, FIELD.y1, lat0, lon0)].map(([la, lo]) => [lo, la]) } });
    }
    for (let y = FIELD.y0; y <= FIELD.y1; y += 30) {
      grid.push({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates:
        [enuToLatLon(FIELD.x0, y, lat0, lon0), enuToLatLon(FIELD.x1, y, lat0, lon0)].map(([la, lo]) => [lo, la]) } });
    }
    setSourceData('field-grid', { type: 'FeatureCollection', features: grid });
    setSourceData('launch-pad', {
      type: 'FeatureCollection',
      features: [rectFeature(enuRect(LAUNCH_PAD.x, LAUNCH_PAD.y, LAUNCH_PAD.size, LAUNCH_PAD.size, lat0, lon0))],
    });
    setSourceData('scenery', {
      type: 'FeatureCollection',
      features: SCENERY.map((s) => {
        const [la, lo] = enuToLatLon(s.x, s.y, lat0, lon0);
        return { type: 'Feature', properties: { kind: s.kind }, geometry: { type: 'Point', coordinates: [lo, la] } };
      }),
    });
    // Frame the whole ground plane (field + scenery) with a little padding.
    const sw = enuToLatLon(GROUND_PLANE.cx - GROUND_PLANE.w / 2, GROUND_PLANE.cy - GROUND_PLANE.h / 2, lat0, lon0);
    const ne = enuToLatLon(GROUND_PLANE.cx + GROUND_PLANE.w / 2, GROUND_PLANE.cy + GROUND_PLANE.h / 2, lat0, lon0);
    map.fitBounds([[sw[1], sw[0]], [ne[1], ne[0]]], { padding: 24, duration: 0 });
    builtSiteKey = siteKey;
  }

  // ---- reactive updates ----

  // Build the synthetic field as soon as the map + config are ready.
  $effect(() => {
    if (!mapReady) return;
    void mission.config;          // re-run when config arrives
    buildSyntheticField();
  });

  // Scan coverage — grey-shade each swept 5 m cell. Rebuilds only when the set
  // grows (scanVersion bumps), not every telemetry frame.
  $effect(() => {
    if (!mapReady) return;
    const v = mission.scanVersion;     // dependency
    void v;
    const [lat0, lon0] = siteCenter();
    const step = 5;
    const features: GeoJSON.Feature[] = [];
    for (const key of mission.scannedCells) {
      const [ix, iy] = key.split(',').map(Number);
      features.push(rectFeature(enuRect(ix * step, iy * step, step, step, lat0, lon0)));
    }
    setSourceData('scanned-coverage', { type: 'FeatureCollection', features });
  });

  // Geofence outline.
  $effect(() => {
    if (!mapReady) return;
    const verts = mission.geofence.vertices;
    if (verts.length >= 3) {
      setSourceData('controlled-airspace', polygonFeature(verts));
    } else {
      setSourceData('controlled-airspace', { type: 'FeatureCollection', features: [] });
    }
  });

  // AAVC search sub-region (from config).
  $effect(() => {
    if (!mapReady) return;
    const cfg = mission.config;
    if (cfg && cfg.search_area?.length >= 3) {
      setSourceData('search-area', polygonFeature(cfg.search_area));
    } else {
      setSourceData('search-area', { type: 'FeatureCollection', features: [] });
    }
  });

  // V1.3 mandatory transit corridor P1→P2→P3 + no-fly zones (from config).
  $effect(() => {
    if (!mapReady) return;
    const cfg = mission.config;
    const route = (cfg?.primary_transit_route ?? []) as [number, number][];
    const feats: GeoJSON.Feature[] = [];
    if (route.length >= 1) {
      const pts = route.map(([lat, lon]) => [lon, lat] as [number, number]);
      if (pts.length >= 2) {
        feats.push({ type: 'Feature', properties: {},
                     geometry: { type: 'LineString', coordinates: pts } });
      }
      pts.forEach((c, i) => feats.push({
        type: 'Feature', properties: { name: `P${i + 1}` },
        geometry: { type: 'Point', coordinates: c },
      }));
    }
    setSourceData('transit-route', { type: 'FeatureCollection', features: feats });

    const zones = (cfg?.no_fly_zones ?? []) as [number, number][][];
    const zoneFeats = zones.filter((z) => z.length >= 3)
      .map((z) => polygonFeature(z).features[0]);
    setSourceData('no-fly-zones', { type: 'FeatureCollection', features: zoneFeats });
  });

  // Plan path + numbered waypoint chips (TAKEOFF / GOTO target / DROP / RTH).
  $effect(() => {
    if (!mapReady) return;
    const plan = mission.plan;
    if (!plan) return;
    const coords: [number, number][] = plan.commands
      .filter(c => c.coord)
      .map(c => [c.coord!.lon, c.coord!.lat]);
    if (coords.length >= 2) {
      setSourceData('plan-path', {
        type: 'FeatureCollection',
        features: [{
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: coords },
          properties: {},
        }],
      });
    }
    for (const { marker } of waypointMarkers.values()) marker.remove();
    waypointMarkers = new Map();
    for (const cmd of plan.commands) {
      if (!cmd.coord) continue;
      // The drop point + its localize-approach waypoint share a spot with the
      // amber/blue drop markers drawn below — skip their chips to declutter.
      if (cmd.kind === 'drop_payload' || cmd.phase === 'localize') continue;
      const el = document.createElement('div');
      el.style.cssText = `
        display: flex; align-items: center; justify-content: center;
        width: 22px; height: 22px;
        background: ${phaseColor(cmd.phase)};
        color: #020617; font-weight: 700; font-size: 11px;
        font-family: "IBM Plex Mono", ui-monospace, monospace;
        border: 1px solid rgba(241, 245, 249, 0.65);
        box-shadow: 0 0 6px rgba(0, 0, 0, 0.7);
        cursor: pointer;
        border-radius: 3px;
      `;
      el.textContent = String(cmd.seq);
      const altStr = cmd.altitude_m !== null ? `${cmd.altitude_m.toFixed(0)}m AGL` : '';
      const speedStr = cmd.speed_mps !== null ? ` @${cmd.speed_mps.toFixed(1)}m/s` : '';
      el.title = `#${cmd.seq} · ${cmd.kind.toUpperCase()} · ${cmd.phase}\n${altStr}${speedStr}\n${cmd.notes}`;
      const m = new maplibregl.Marker({ element: el, anchor: 'center' })
        .setLngLat([cmd.coord.lon, cmd.coord.lat])
        .addTo(map!);
      waypointMarkers.set(cmd.seq, { marker: m, el });
    }
  });

  // Highlight the active command's chip.
  $effect(() => {
    const cur = mission.telemetry?.command_pointer ?? -1;
    for (const [seq, { el }] of waypointMarkers) {
      const active = seq === cur;
      el.style.transform = active ? 'scale(1.35)' : 'scale(1)';
      el.style.borderColor = active ? '#f59e0b' : 'rgba(241, 245, 249, 0.65)';
      el.style.borderWidth = active ? '2px' : '1px';
      el.style.boxShadow = active
        ? '0 0 12px rgba(245, 158, 11, 0.9)'
        : '0 0 6px rgba(0, 0, 0, 0.7)';
      el.style.zIndex = active ? '20' : '5';
    }
  });

  // Flown track.
  $effect(() => {
    if (!mapReady) return;
    const track = mission.actualTrack;
    if (track.length < 2) return;
    setSourceData('actual-track', {
      type: 'FeatureCollection',
      features: [{
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: track.map(([la, lo]) => [lo, la]) },
        properties: {},
      }],
    });
  });

  // Live drone position + heading.
  $effect(() => {
    if (!mapReady) return;
    const t = mission.telemetry;
    if (!t || t.lat === null || t.lon === null || droneMarker === null) return;
    droneMarker.setLngLat([t.lon, t.lat]);
    if (t.heading_deg !== null) {
      // Heading goes on a child wrapper so MapLibre's outer translate is never
      // compounded (which would either drop the rotation or grow the transform
      // list unbounded at 5 Hz over a multi-minute mission).
      const el = droneMarker.getElement();
      let inner = el.querySelector<HTMLDivElement>(':scope > .aavc-heading-inner');
      if (!inner) {
        inner = el.firstElementChild as HTMLDivElement;
        if (inner) inner.classList.add('aavc-heading-inner');
      }
      if (inner) inner.style.transform = `rotate(${t.heading_deg}deg)`;
    }
  });

  // Targets / payload drop points. A marker appears the MOMENT the tracker
  // confirms a target during the sweep (detected_object = one confirmation
  // event per target), merged with the live plan's DROP_PAYLOAD points (the
  // same physical targets once serving starts — deduped by proximity). Amber
  // ring = found, payload still to drop; solid blue = payload released there
  // (a drop event within ~8 m — we land ON the target before releasing, so
  // the impact ≈ the marker).
  function _distM(aLat: number, aLon: number, bLat: number, bLon: number): number {
    const R = 6_378_137;
    const dn = ((bLat - aLat) * Math.PI) / 180;
    const de = ((bLon - aLon) * Math.PI) / 180 * Math.cos((aLat * Math.PI) / 180);
    return Math.hypot(dn, de) * R;
  }
  $effect(() => {
    if (!mapReady) return;
    const plan = mission.plan;
    const drops = mission.dropPredictions;       // dependency — fires on each release
    const found = mission.detectedObjects;       // dependency — confirmed targets
    for (const m of dropMarkers) m.remove();
    dropMarkers = [];
    const pts: { lat: number; lon: number; name: string }[] = [];
    if (plan) {
      for (const cmd of plan.commands) {
        if (cmd.kind !== 'drop_payload' || !cmd.coord) continue;
        pts.push({ lat: cmd.coord.lat, lon: cmd.coord.lon,
                   name: `payload ${cmd.payload_id ?? '?'}` });
      }
    }
    for (const o of found) {
      // Skip confirmations that already became a plan serve point (same target).
      if (pts.some((p) => _distM(p.lat, p.lon, o.lat, o.lon) <= 8)) continue;
      pts.push({ lat: o.lat, lon: o.lon, name: o.label });
    }
    for (const p of pts) {
      const done = drops.some((d) => _distM(p.lat, p.lon, d.impact_lat, d.impact_lon) <= 8);
      const el = document.createElement('div');
      el.title = done ? `${p.name} · DROPPED` : `${p.name} · found — payload pending`;
      el.style.cssText = `
        width: 16px; height: 16px; border-radius: 50%;
        background: ${done ? '#3b82f6' : 'transparent'};
        border: 2.5px solid ${done ? '#93c5fd' : '#f59e0b'};
        box-shadow: 0 0 8px ${done ? 'rgba(59,130,246,0.95)' : 'rgba(245,158,11,0.75)'};
      `;
      const m = new maplibregl.Marker({ element: el, anchor: 'center' })
        .setLngLat([p.lon, p.lat]).addTo(map!);
      dropMarkers.push(m);
    }
  });
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Map</span>
    <span class="flex items-center gap-3 ml-3 flex-wrap text-[10px] normal-case font-normal"
          style="color: var(--color-aavc-ink-mute); letter-spacing: 0.02em;">
      <span class="flex items-center gap-1" title="Planned flight path (waypoint chain from MissionPlan)">
        <span class="w-3 h-px" style="background: var(--color-aavc-warning);"></span>plan</span>
      <span class="flex items-center gap-1" title="Actual flown track from vehicle telemetry">
        <span class="w-3 h-px" style="background: var(--color-aavc-nominal);"></span>actual</span>
      <span class="flex items-center gap-1" title="Geofence — controlled airspace / legal flight envelope">
        <span class="w-3 h-px" style="background: var(--color-aavc-warning); border-top: 1px dashed var(--color-aavc-warning);"></span>geofence</span>
      <span class="flex items-center gap-1" title="AAVC search sub-region from config">
        <span class="w-3 h-px" style="background: var(--color-aavc-info);"></span>search</span>
      <span class="flex items-center gap-1" title="Confirmed target — appears the moment it is found; payload still to drop">
        <span class="w-2.5 h-2.5 inline-block rounded-full" style="border: 2px solid #f59e0b;"></span>target found</span>
      <span class="flex items-center gap-1" title="Payload released here">
        <span class="w-2.5 h-2.5 inline-block rounded-full" style="background: #3b82f6; border: 1px solid #93c5fd;"></span>dropped</span>
      <span class="flex items-center gap-1" title="Ground the nadir camera has already swept">
        <span class="w-2.5 h-2.5 inline-block rounded-sm" style="background: rgba(148,163,184,0.5);"></span>scanned</span>
    </span>
  </div>
  <div bind:this={container} class="flex-1 min-h-0 overflow-hidden"
       style="background: var(--color-aavc-body);"></div>
</div>
