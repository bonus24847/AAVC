<script lang="ts">
  // Compact flight monitor for the Tuning right rail. Reads the same 5 Hz
  // telemetry the flight GCS does (mission.telemetry ← /ws/realtime), but trimmed
  // to what matters while running System-ID / Autotune:
  //   armed + flight mode  → is the vehicle in OFFBOARD and actually flying?
  //   alt AGL + battery     → holding the sweep altitude, enough charge to finish
  //   attitude + body rates → the rates ARE the sys-ID excitation signal (they
  //                           should wiggle through the chirp); attitude shows
  //                           the drone re-levelling between axes
  //   motor PWM             → saturation watch (a saturated motor corrupts the
  //                           identified plant — large chirp amplitudes can clip)
  // It deliberately drops the mission-only readouts (GPS sats, time budget,
  // ground speed) that the full TelemetrySidebar carries.

  import { mission } from '../lib/stores.svelte';
  import { fmtNum, fmtInt, clamp } from '../lib/format';

  const t = $derived(mission.telemetry);

  function batteryTier(pct: number | null): string {
    if (pct === null) return '';
    if (pct < 22) return 'tier-critical';
    if (pct < 32) return 'tier-warning';
    if (pct < 50) return 'tier-caution';
    return 'tier-nominal';
  }

  const motors = $derived((t?.servo_pwm_us ?? []).slice(0, 6));
  function motorFrac(pwm: number): number {
    // Auto-detect PWM scale (1000-2000 µs for ESCs; 0-1 for normalised).
    if (pwm <= 1.5) return clamp(pwm, 0, 1);
    if (pwm >= 800 && pwm <= 2050) return clamp((pwm - 1000) / 1000, 0, 1);
    return clamp(pwm / 1999, 0, 1);
  }
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Flight Monitor</span>
    <span class="ml-auto flex items-center gap-1.5">
      {#if t?.is_armed}
        <span class="aavc-chip aavc-chip-armed">▲ ARMED</span>
      {:else}
        <span class="aavc-chip aavc-chip-nominal">○ disarmed</span>
      {/if}
      <span class="aavc-chip aavc-chip-info">{t?.flight_mode || '—'}</span>
    </span>
  </div>

  <div class="aavc-panel-body aavc-scroll overflow-y-auto" style="padding: 12px;">

    <!-- Hero: altitude AGL + battery -->
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

    <!-- Voltage + armed/link state -->
    <div class="grid grid-cols-2 gap-3 py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">voltage</span>
        <span class="aavc-readout-value">
          {fmtNum(t?.battery_voltage_v ?? null, 1)}<span class="aavc-readout-unit">V</span>
        </span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">datalink</span>
        <span class="aavc-readout-value size-sm {t?.link_connected ? 'tier-nominal' : 'tier-critical'}">
          {t?.link_connected ? 'connected' : 'down'}
        </span>
      </div>
    </div>

    <!-- Attitude (roll / pitch) -->
    <div class="grid grid-cols-2 gap-3 py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout">
        <span class="aavc-readout-label">roll</span>
        <span class="aavc-readout-value">
          {fmtNum(t?.roll_deg ?? null, 1)}<span class="aavc-readout-unit">°</span>
        </span>
      </div>
      <div class="aavc-readout">
        <span class="aavc-readout-label">pitch</span>
        <span class="aavc-readout-value">
          {fmtNum(t?.pitch_deg ?? null, 1)}<span class="aavc-readout-unit">°</span>
        </span>
      </div>
    </div>

    <!-- Body rates — the sys-ID excitation signal -->
    <div class="py-3 border-b" style="border-color: var(--color-aavc-border);">
      <div class="aavc-readout-label mb-1.5">body rates · °/s</div>
      <div class="grid grid-cols-3 gap-2 text-center">
        {#each [['roll', t?.roll_rate_dps], ['pitch', t?.pitch_rate_dps], ['yaw', t?.yaw_rate_dps]] as [name, val] (name)}
          <div>
            <div style="color: var(--color-aavc-ink-mute); font-size: 10px; letter-spacing: 0.06em;">{name}</div>
            <div class="font-mono" style="color: var(--color-aavc-ink); font-size: 14px;">
              {fmtNum(val as number | null, 0)}
            </div>
          </div>
        {/each}
      </div>
    </div>

    <!-- Motors (compact bars) — saturation watch -->
    <div class="pt-3">
      <div class="aavc-readout-label mb-1.5">motors · m1-m6</div>
      {#if motors.length === 0}
        <div class="text-xs" style="color: var(--color-aavc-ink-mute);">no actuator data</div>
      {:else}
        <div class="grid grid-cols-6 gap-2">
          {#each motors as pwm, i (i)}
            <div class="flex flex-col items-center gap-1">
              <div class="relative h-14 w-2.5 overflow-hidden rounded-sm"
                   style="background: #0b1220; border: 1px solid var(--color-aavc-border);">
                <div class="absolute bottom-0 inset-x-0 transition-all duration-150"
                     style="height: {motorFrac(pwm) * 100}%;
                            background: var(--color-aavc-nominal);"></div>
              </div>
              <span class="font-mono text-[10px]" style="color: var(--color-aavc-ink-dim);">M{i + 1}</span>
              <span class="font-mono text-[10px]" style="color: var(--color-aavc-ink-2);">{fmtInt(pwm)}</span>
            </div>
          {/each}
        </div>
      {/if}
    </div>
  </div>
</div>
