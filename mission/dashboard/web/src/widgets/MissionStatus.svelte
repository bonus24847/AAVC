<script lang="ts">
  import { mission } from '../lib/stores.svelte';
  import { fmtTime } from '../lib/format';
  import ArucoGlyph from './ArucoGlyph.svelte';

  const t = $derived(mission.telemetry);
  const flightIds = $derived(t?.flight_ids ?? []);
  const padId = $derived(t?.assigned_marker_id ?? null);

  function phaseTier(phase: string): string {
    switch (phase) {
      case 'drop':           return 'aavc-chip-warning';
      case 'rth':
      case 'abort':          return 'aavc-chip-critical';
      case 'localize':       return 'aavc-chip-caution';
      case 'search':         return 'aavc-chip-nominal';
      case 'transit_ingress':
      case 'transit_egress': return 'aavc-chip-info';
      default:               return 'aavc-chip';
    }
  }

  function timeTier(remaining_s: number): string {
    if (remaining_s < 60) return 'tier-critical';
    if (remaining_s < 180) return 'tier-warning';
    return '';
  }
</script>

<div class="flex items-center justify-between gap-4 px-4 py-2.5 rounded-lg"
     style="background: var(--color-aavc-panel); border: 1px solid var(--color-aavc-border);">
  <div class="flex items-center gap-3 flex-wrap">
    <span class="aavc-chip {phaseTier(t?.phase ?? '')} font-mono">
      {(t?.phase ?? 'idle').toUpperCase()}
    </span>
    {#if (t?.sortie_index ?? 0) > 0}
      <span class="aavc-chip aavc-chip-info font-mono"
            title="flight (one arm→disarm cycle) / planned flights · delivery (one pad served) / pads to serve">
        FLIGHT {t?.sortie_index}/{t?.max_sorties || '?'}
        {#if (t?.delivery_index ?? 0) > 0}
          · DELIVERY {t?.delivery_index}/{t?.max_deliveries || '?'}
        {/if}
      </span>
    {/if}
    {#if flightIds.length > 1}
      <span class="aavc-chip aavc-chip-caution font-mono"
            title="this flight's assigned ArUco landing-pad ids, in order">
        {#each flightIds as id}<ArucoGlyph {id} size={16} />{/each}PAD {flightIds.join(',')}
      </span>
    {:else if padId}
      <span class="aavc-chip aavc-chip-caution font-mono"
            title="committee-assigned ArUco landing-pad id for this sortie">
        <ArucoGlyph id={padId} size={16} />PAD {padId}
      </span>
    {/if}
    <span class="font-mono text-[12px]" style="color: var(--color-aavc-ink-dim);">
      CMD <span style="color: var(--color-aavc-ink);">#{t?.command_pointer ?? 0}</span>
    </span>
    <span class="text-[12px]" style="color: var(--color-aavc-ink-dim);">
      MODE <span class="font-mono" style="color: var(--color-aavc-ink);">{t?.flight_mode ?? '—'}</span>
    </span>
    {#if t?.is_armed}
      <span class="aavc-chip aavc-chip-armed">▲ VEHICLE ARMED</span>
    {:else}
      <span class="aavc-chip">vehicle disarmed</span>
    {/if}
    {#if t && !t.link_connected}
      <span class="aavc-chip aavc-chip-critical">⊘ NO LINK · start `make sitl`</span>
    {/if}
  </div>

  <div class="flex items-baseline gap-5 font-mono text-[13px]">
    <div>
      <span class="text-[10px] uppercase tracking-[0.10em]"
            style="color: var(--color-aavc-ink-mute);">elapsed</span>
      <span class="ml-1.5 font-semibold" style="color: var(--color-aavc-ink);">{fmtTime(t?.elapsed_s ?? 0)}</span>
    </div>
    <div>
      <span class="text-[10px] uppercase tracking-[0.10em]"
            style="color: var(--color-aavc-ink-mute);">remain</span>
      <span class="ml-1.5 font-semibold {timeTier(t?.remaining_s ?? 9999)}"
            style="color: var(--color-aavc-ink);">{fmtTime(t?.remaining_s ?? 0)}</span>
    </div>
    <div class="text-[11px]" style="color: var(--color-aavc-ink-mute);">{t?.terminal ?? '—'}</div>
  </div>
</div>
