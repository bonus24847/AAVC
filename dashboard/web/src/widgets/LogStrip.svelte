<script lang="ts">
  // Right-rail tabbed log: MAVLink command trace + anomaly log. Stable
  // height even when one log bursts.

  import { mission } from '../lib/stores.svelte';
  import { fmtTime } from '../lib/format';

  type Tab = 'mavlink' | 'anomalies';
  let tab = $state<Tab>('mavlink');

  const mavlinkCount = $derived(mission.commandLog.length);
  const anomalyCount = $derived(mission.anomalyLog.length);

  const mavlinkRows = $derived(mission.commandLog.slice(-80).reverse());
  const anomalyRows = $derived(mission.anomalyLog.slice(-80).reverse());
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header" style="padding: 0;">
    <div class="flex w-full">
      <button class="px-4 py-2.5 text-[11px] font-semibold uppercase tracking-[0.10em]
                     border-r transition-colors duration-100"
              style="border-color: var(--color-aavc-border);
                     color: {tab === 'mavlink' ? 'var(--color-aavc-ink)' : 'var(--color-aavc-ink-dim)'};
                     background: {tab === 'mavlink' ? 'var(--color-aavc-panel-2)' : 'transparent'};"
              onclick={() => tab = 'mavlink'}>
        MAVLink
        <span class="ml-2 font-mono text-[10px]" style="color: var(--color-aavc-ink-mute);">{mavlinkCount}</span>
      </button>
      <button class="px-4 py-2.5 text-[11px] font-semibold uppercase tracking-[0.10em]
                     transition-colors duration-100"
              style="color: {tab === 'anomalies' ? 'var(--color-aavc-ink)' : 'var(--color-aavc-ink-dim)'};
                     background: {tab === 'anomalies' ? 'var(--color-aavc-panel-2)' : 'transparent'};"
              onclick={() => tab = 'anomalies'}>
        Anomalies
        <span class="ml-2 font-mono text-[10px]"
              style="color: {anomalyCount > 0 ? 'var(--color-aavc-critical)' : 'var(--color-aavc-ink-mute)'};">
          {anomalyCount}
        </span>
      </button>
    </div>
  </div>

  <div class="flex-1 min-h-0 aavc-scroll overflow-y-auto px-3 py-2">
    {#if tab === 'mavlink'}
      {#if mavlinkRows.length === 0}
        <div class="text-xs py-2" style="color: var(--color-aavc-ink-mute);">no MAVLink commands dispatched yet</div>
      {:else}
        <table class="w-full text-[12px] font-mono">
          <tbody>
            {#each mavlinkRows as ev}
              <tr class="border-b align-top" style="border-color: var(--color-aavc-border);">
                <td class="py-1.5 pr-3 whitespace-nowrap"
                    style="color: var(--color-aavc-ink-mute); width: 56px;">
                  t+{fmtTime(ev.t_monotonic)}
                </td>
                <td class="py-1.5">
                  <div class="flex items-baseline gap-2 flex-wrap">
                    <span class="font-semibold" style="color: var(--color-aavc-info);">{ev.method}</span>
                    <span style="color: var(--color-aavc-ink-dim);">
                      {Object.entries(ev.args).map(([k, v]) => `${k}=${typeof v === 'number' ? v.toFixed(3) : v}`).join(' ')}
                    </span>
                  </div>
                  {#if ev.mavlink}
                    <div class="mt-0.5 text-[11px]" style="color: var(--color-aavc-warning);">
                      ⇢ {ev.mavlink}
                    </div>
                  {/if}
                </td>
              </tr>
            {/each}
          </tbody>
        </table>
      {/if}
    {:else}
      {#if anomalyRows.length === 0}
        <div class="text-xs py-2" style="color: var(--color-aavc-nominal);">no anomalies — nominal</div>
      {:else}
        <ul class="space-y-0.5 text-[12px] font-mono">
          {#each anomalyRows as ev}
            <li class="py-1 border-b" style="border-color: var(--color-aavc-border);">
              <span style="color: var(--color-aavc-ink-mute);">t+{fmtTime(ev.t_monotonic)}</span>
              <span class="ml-3" style="color: var(--color-aavc-critical);">⚠</span>
              <span class="ml-2" style="color: var(--color-aavc-ink-2);">{ev.message}</span>
            </li>
          {/each}
        </ul>
      {/if}
    {/if}
  </div>
</div>
