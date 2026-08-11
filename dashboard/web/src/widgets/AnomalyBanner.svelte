<script lang="ts">
  import { mission } from '../lib/stores.svelte';
  const latest = $derived(mission.anomalyLog.at(-1));
</script>

{#if latest}
  <div class="px-4 py-2 text-[12px] font-mono rounded"
       style="background: color-mix(in srgb, var(--color-aavc-critical) 14%, transparent);
              border: 1px solid color-mix(in srgb, var(--color-aavc-critical) 55%, transparent);
              color: var(--color-aavc-critical);">
    <span class="font-semibold uppercase tracking-[0.10em]">⚠ Anomaly</span>
    <span class="mx-2" style="color: var(--color-aavc-ink-mute);">·</span>
    <span style="color: var(--color-aavc-ink-2);">{latest.message}</span>
  </div>
{/if}
