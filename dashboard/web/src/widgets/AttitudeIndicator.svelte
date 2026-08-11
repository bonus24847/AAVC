<script lang="ts">
  // Artificial horizon (roll/pitch) + a heading tape (compass) along the bottom,
  // so the operator scans attitude AND heading in one glance next to the camera.
  import { mission } from '../lib/stores.svelte';
  import { fmtNum, clamp } from '../lib/format';

  const t = $derived(mission.telemetry);
  const roll = $derived(t?.roll_deg ?? 0);
  const pitch = $derived(t?.pitch_deg ?? 0);
  const heading = $derived(t?.heading_deg ?? 0);
  const pitchPx = $derived(clamp(pitch * 2.2, -120, 120));

  function cardinal(deg: number): string {
    const d = ((deg % 360) + 360) % 360;
    return { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[d] ?? '';
  }

  // Heading tape: a 120°-wide window centred on the current heading. Ticks every
  // 10°, labels (cardinal or degrees) every 30°. viewBox 300 wide → 2.5 px/°.
  const PXDEG = 2.5;
  const headingTicks = $derived.by(() => {
    const h = heading;
    const out: { x: number; label: string; major: boolean }[] = [];
    const start = Math.ceil((h - 60) / 10) * 10;
    for (let d = start; d <= h + 60; d += 10) {
      const norm = ((d % 360) + 360) % 360;
      const major = norm % 30 === 0;
      out.push({ x: 150 + (d - h) * PXDEG, label: cardinal(norm) || (major ? String(norm) : ''), major });
    }
    return out;
  });
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Attitude</span>
    <span class="ml-auto font-mono text-[11px]"
          style="color: var(--color-aavc-ink-dim);">
      R {fmtNum(roll, 1)}° · P {fmtNum(pitch, 1)}° · HDG {fmtNum(heading, 0)}°
    </span>
  </div>

  <div class="aavc-panel-body flex-1 min-h-0 p-3 flex flex-col gap-2">
    <div class="relative flex-1 min-h-0 w-full overflow-hidden rounded"
         style="background: #0b1220; border: 1px solid var(--color-aavc-border);">
      <!-- rotating sky/ground layer -->
      <div class="absolute inset-0 origin-center"
           style="transform: rotate({-roll}deg) translateY({pitchPx}px);">
        <div class="absolute inset-x-[-50%] top-[-50%] h-[100%]"
             style="background: linear-gradient(180deg, #1e3a5f, #0f1f3a);"></div>
        <div class="absolute inset-x-[-50%] top-[50%] h-[100%]"
             style="background: linear-gradient(180deg, #5d3215, #2c1808);"></div>
        <div class="absolute inset-x-[-50%] top-1/2 h-px"
             style="background: var(--color-aavc-ink-2);"></div>
        <!-- pitch ladder, every 10° (1.2 px/° matches original scale) -->
        {#each [-30, -20, -10, 10, 20, 30] as p}
          <div class="absolute left-1/2 -translate-x-1/2 flex items-center gap-2 font-mono text-[10px]"
               style="top: calc(50% + {-p * 2.2}px);
                      color: var(--color-aavc-ink-2);">
            <span class="inline-block w-6 h-px"
                  style="background: var(--color-aavc-ink-2);"></span>
            <span>{p}</span>
            <span class="inline-block w-6 h-px"
                  style="background: var(--color-aavc-ink-2);"></span>
          </div>
        {/each}
      </div>

      <!-- fixed aircraft mark -->
      <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
        <svg viewBox="-50 -50 100 100" class="w-1/2 h-1/2"
             style="color: var(--color-aavc-warning);">
          <line x1="-30" y1="0" x2="-10" y2="0"
                stroke="currentColor" stroke-width="3"/>
          <line x1="10"  y1="0" x2="30" y2="0"
                stroke="currentColor" stroke-width="3"/>
          <circle cx="0" cy="0" r="2.5" fill="currentColor"/>
        </svg>
      </div>

      <!-- roll scale ticks at the top -->
      <div class="absolute inset-x-0 top-0 flex justify-center pointer-events-none">
        <div class="relative h-3 w-3/4">
          {#each [-60, -30, 0, 30, 60] as r}
            <div class="absolute top-0 -translate-x-1/2 font-mono text-[9px]"
                 style="left: calc(50% + {r * 0.6}%);
                        color: var(--color-aavc-ink-dim);">
              {r === 0 ? '▾' : Math.abs(r)}
            </div>
          {/each}
        </div>
      </div>
    </div>

    <!-- heading tape (compass) -->
    <svg viewBox="0 0 300 34" class="w-full" style="height: 34px; flex-shrink: 0;"
         preserveAspectRatio="none">
      <rect x="0" y="0" width="300" height="34" fill="#0b1220"
            stroke="var(--color-aavc-border)" stroke-width="1" rx="3" />
      {#each headingTicks as tick (tick.label + tick.x)}
        {#if tick.x >= 4 && tick.x <= 296}
          <line x1={tick.x} y1="0" x2={tick.x} y2={tick.major ? 9 : 5}
                stroke="var(--color-aavc-ink-2)" stroke-width={tick.major ? 1.2 : 0.8} />
          {#if tick.label}
            <text x={tick.x} y="22" text-anchor="middle" font-size="10"
                  font-family="IBM Plex Mono, monospace"
                  fill={/[NESW]/.test(tick.label) ? 'var(--color-aavc-warning)' : 'var(--color-aavc-ink-dim)'}>
              {tick.label}
            </text>
          {/if}
        {/if}
      {/each}
      <!-- centre index -->
      <polygon points="150,2 145,11 155,11" fill="var(--color-aavc-warning)" />
      <line x1="150" y1="0" x2="150" y2="34" stroke="var(--color-aavc-warning)" stroke-width="0.8" opacity="0.5" />
      <text x="150" y="32" text-anchor="middle" font-size="10" font-weight="700"
            font-family="IBM Plex Mono, monospace" fill="var(--color-aavc-warning)">
        {fmtNum(heading, 0)}°
      </text>
    </svg>
  </div>
</div>
