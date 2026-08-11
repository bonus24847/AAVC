<script lang="ts">
  // Left-rail telemetry. One panel, dense numerics, severity-coloured.
  // Replaces the previous floating gauge widgets so everything an
  // operator needs to read at a glance lives in one column.

  import { mission } from '../lib/stores.svelte';
  import { fmtNum, fmtInt, fmtTime, clamp } from '../lib/format';
  import ArucoGlyph from './ArucoGlyph.svelte';

  const t = $derived(mission.telemetry);

  // Confirmed landing pads (V1.3 scoring: the pad's COORDINATE must be
  // obtained + shown on the GCS). Dedupe map events by pad label, keep the
  // freshest fix per id.
  const pads = $derived.by(() => {
    const seen = new Map<string, { label: string; lat: number; lon: number }>();
    for (const d of mission.detectedObjects) {
      if (d.label.startsWith('aruco pad')) seen.set(d.label, d);
    }
    // Carry the decoded id so the readout can show the marker itself — the
    // label ("aruco pad 3") is the only place the id survives the event.
    return [...seen.values()]
      .map((p) => ({ ...p, id: Number(p.label.slice('aruco pad '.length)) }))
      .sort((a, b) => a.id - b.id);
  });
  const assignedLabel = $derived(
    t?.assigned_marker_id ? `aruco pad ${t.assigned_marker_id}` : '');

  // -------- severity helpers --------

  function batteryTier(pct: number | null): string {
    if (pct === null) return '';
    if (pct < 22) return 'tier-critical';
    if (pct < 32) return 'tier-warning';
    if (pct < 50) return 'tier-caution';
    return 'tier-nominal';
  }

  function gpsTier(fix: number, sats: number): string {
    if (fix < 3) return 'tier-critical';
    if (sats < 8) return 'tier-warning';
    return 'tier-nominal';
  }

  function linkTier(rssi: number, connected: boolean): string {
    if (!connected) return 'tier-critical';
    if (rssi < 50) return 'tier-warning';
    return 'tier-nominal';
  }

  function timeTier(remaining_s: number): string {
    if (remaining_s < 60) return 'tier-critical';
    if (remaining_s < 180) return 'tier-warning';
    return 'tier-info';
  }

  // -------- compact motors --------
  const motors = $derived((t?.servo_pwm_us ?? []).slice(0, 6));
  function motorFrac(pwm: number): number {
    // Auto-detect PWM scale (1000-2000 µs for ESCs).
    if (pwm <= 1.5) return clamp(pwm, 0, 1);
    if (pwm >= 800 && pwm <= 2050) return clamp((pwm - 1000) / 1000, 0, 1);
    return clamp(pwm / 1999, 0, 1);
  }
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Telemetry</span>
    <span class="ml-auto">
      {#if t?.link_connected}
        <span class="aavc-chip aavc-chip-nominal">● link</span>
      {:else}
        <span class="aavc-chip aavc-chip-critical">✕ no link</span>
      {/if}
    </span>
  </div>

  <div class="aavc-panel-body aavc-scroll overflow-y-auto" style="padding: 14px;">

    <!-- Hero readouts: AGL + battery — the two operators reach for first -->
    <div class="grid grid-cols-2 gap-3 pb-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">altitude agl</span>
        <span class="aavc-readout-value size-lg">
          {fmtNum(t?.alt_agl_m ?? null, 1)}<span class="aavc-readout-unit">m</span>
        </span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">battery</span>
        <span class="aavc-readout-value size-lg {batteryTier(t?.battery_percent ?? null)}">
          {fmtInt(t?.battery_percent ?? null)}<span class="aavc-readout-unit">%</span>
        </span>
      </div>
    </div>

    <!-- V1.3 scoring: confirmed pad ids + their obtained coordinates -->
    <div class="py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div style="color: var(--color-aavc-ink-mute); font-size: 10px; letter-spacing: 0.06em;">
        CONFIRMED PADS (id · lat · lon)
      </div>
      {#if pads.length === 0}
        <div class="text-xs" style="color: var(--color-aavc-ink-mute); padding-top: 4px;">
          none decoded yet
        </div>
      {:else}
        {#each pads as p (p.label)}
          <div class="font-mono text-[12px] flex items-center gap-2" style="padding-top: 3px;
               color: {p.label === assignedLabel ? 'var(--color-aavc-warning)' : 'var(--color-aavc-ink)'};">
            <ArucoGlyph id={p.id} size={15} />
            <span>PAD {p.id}</span>
            <span style="color: var(--color-aavc-ink-dim);">
              {p.lat.toFixed(6)}, {p.lon.toFixed(6)}
            </span>
            {#if p.label === assignedLabel}<span class="aavc-chip aavc-chip-caution">assigned</span>{/if}
          </div>
        {/each}
      {/if}
    </div>

    <!-- Speed + Heading -->
    <div class="grid grid-cols-2 gap-3 py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">gnd speed</span>
        <span class="aavc-readout-value">
          {fmtNum(t?.ground_speed_mps ?? null, 1)}<span class="aavc-readout-unit">m/s</span>
        </span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">heading</span>
        <span class="aavc-readout-value">
          {fmtInt(t?.heading_deg ?? null)}<span class="aavc-readout-unit">°</span>
        </span>
      </div>
    </div>

    <!-- Battery sub-readouts + MSL -->
    <div class="grid grid-cols-3 gap-3 py-3 text-xs border-b"
         style="color: var(--color-aavc-ink-dim); border-color: var(--color-aavc-border);">
      <div>
        <div style="color: var(--color-aavc-ink-mute); font-size: 10px; letter-spacing: 0.06em;">VOLT</div>
        <div class="font-mono" style="color: var(--color-aavc-ink); font-size: 13px;">
          {fmtNum(t?.battery_voltage_v ?? null, 1)} V
        </div>
      </div>
      <div>
        <div style="color: var(--color-aavc-ink-mute); font-size: 10px; letter-spacing: 0.06em;">USED</div>
        <div class="font-mono" style="color: var(--color-aavc-ink); font-size: 13px;">
          {fmtInt(t?.battery_consumed_mah ?? null)} mAh
        </div>
      </div>
      <div>
        <div style="color: var(--color-aavc-ink-mute); font-size: 10px; letter-spacing: 0.06em;">MSL</div>
        <div class="font-mono" style="color: var(--color-aavc-ink); font-size: 13px;">
          {fmtNum(t?.alt_msl_m ?? null, 0)} m
        </div>
      </div>
    </div>

    <!-- GPS + Link -->
    <div class="grid grid-cols-2 gap-3 py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">gps</span>
        <span class="aavc-readout-value size-sm {gpsTier(t?.gps_fix_type ?? 0, t?.gps_satellites ?? 0)}">
          fix {t?.gps_fix_type ?? 0} · {t?.gps_satellites ?? 0} sat
        </span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">datalink</span>
        <span class="aavc-readout-value size-sm {linkTier(t?.datalink_rssi ?? -1, t?.link_connected ?? false)}">
          {t?.link_connected ? (t.datalink_rssi >= 0 ? `${t.datalink_rssi}` : '—') : 'down'}
        </span>
      </div>
    </div>

    <!-- Time budget -->
    <div class="grid grid-cols-2 gap-3 py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">elapsed</span>
        <span class="aavc-readout-value size-sm">{fmtTime(t?.elapsed_s ?? 0)}</span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">remaining</span>
        <span class="aavc-readout-value size-sm {timeTier(t?.remaining_s ?? 9999)}">
          {fmtTime(t?.remaining_s ?? 0)}
        </span>
      </div>
    </div>

    <!-- Energy budget: capacity, draw, and the number the operator acts on -->
    <div class="py-3" style="border-top: 1px solid var(--color-aavc-border);">
      <div class="flex items-center justify-between mb-1">
        <span class="aavc-readout-label">energy</span>
        <span class="font-mono text-[10px]"
              style="color: var(--color-aavc-ink-mute);"
              title={t?.energy_tier === 'A'
                ? 'measured by the power module'
                : t?.energy_tier === 'B'
                  ? 'derived from battery percentage — coarse'
                  : 'no battery signal'}>
          {t?.energy_tier === 'A' ? '● measured'
            : t?.energy_tier === 'B' ? '○ estimated' : '— no data'}
        </span>
      </div>
      {#if t?.battery_capacity_mah}
        <div class="font-mono text-xs" style="color: var(--color-aavc-ink-2);">
          {fmtInt(t.battery_consumed_mah ?? 0)} / {fmtInt(t.battery_capacity_mah)} mAh
        </div>
      {/if}
      {#if t?.battery_current_a != null && t?.battery_voltage_v != null}
        <div class="font-mono text-xs" style="color: var(--color-aavc-ink-2);">
          {fmtInt(t.battery_current_a * t.battery_voltage_v)} W
          · {fmtNum(t.battery_current_a, 1)} A
        </div>
      {/if}
      {#if t?.energy_sorties_left != null}
        <div class="font-mono text-xs mt-1"
             style="color: {t.energy_sorties_left < 1
               ? 'var(--color-aavc-critical)' : 'var(--color-aavc-ink)'};">
          {t.energy_sorties_left < 1
            ? '⚡ SWAP BATTERY BEFORE NEXT GO'
            : `energy for ${fmtNum(t.energy_sorties_left, 1)} more sorties`}
        </div>
      {/if}
    </div>

    <!-- Motors (compact bars) -->
    <div class="py-3">
      <div class="flex items-center justify-between mb-1.5">
        <span class="aavc-readout-label">motors · m1-m6</span>
      </div>
      {#if motors.length === 0}
        <div class="text-xs" style="color: var(--color-aavc-ink-mute);">no actuator data</div>
      {:else}
        <div class="grid grid-cols-6 gap-2">
          {#each motors as pwm, i}
            <div class="flex flex-col items-center gap-1">
              <div class="relative h-16 w-2.5 overflow-hidden rounded-sm"
                   style="background: #0b1220; border: 1px solid var(--color-aavc-border);">
                <div class="absolute bottom-0 inset-x-0 transition-all duration-150"
                     style="height: {motorFrac(pwm) * 100}%;
                            background: var(--color-aavc-nominal);"></div>
              </div>
              <span class="font-mono text-[10px]"
                    style="color: var(--color-aavc-ink-dim);">M{i + 1}</span>
              <span class="font-mono text-[10px]"
                    style="color: var(--color-aavc-ink-2);">{fmtInt(pwm)}</span>
            </div>
          {/each}
        </div>
      {/if}
    </div>
  </div>
</div>
