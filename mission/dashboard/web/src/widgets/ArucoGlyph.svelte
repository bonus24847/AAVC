<script lang="ts">
  // The committee assigns a pad by handing over an ArUco MARKER, so every place
  // the GCS names a pad shows the marker itself — the operator matches a picture
  // to a picture instead of translating it into a number under time pressure.
  // Bits are baked from the flight detector's own dictionary
  // (tools/gen_aruco_glyphs.py, held there by tests/test_aruco_glyphs.py), so
  // the chip can never drift from what the aircraft decodes. Inline SVG keeps it
  // offline and crisp at any size — no asset, no endpoint (AAVC bans network).
  import { ARUCO_GLYPH_ROWS, ARUCO_GLYPH_CELLS } from '../lib/aruco-glyphs';

  let { id, size = 24, dim = false }: {
    id: number; size?: number; dim?: boolean;
  } = $props();

  const rows = $derived(ARUCO_GLYPH_ROWS[id] ?? []);
</script>

{#if rows.length}
  <svg class="aavc-glyph" class:dim viewBox="0 0 {ARUCO_GLYPH_CELLS} {ARUCO_GLYPH_CELLS}"
       width={size} height={size} shape-rendering="crispEdges"
       role="img" aria-label="ArUco marker id {id}">
    <rect width={ARUCO_GLYPH_CELLS} height={ARUCO_GLYPH_CELLS} fill="#000" />
    {#each rows as row, y}
      {#each row.split('') as cell, x}
        {#if cell === '1'}
          <rect x={x} y={y} width="1" height="1" fill="#fff" />
        {/if}
      {/each}
    {/each}
  </svg>
{:else}
  <span class="aavc-glyph-missing" title="id {id} is outside the ArUco pad set">?</span>
{/if}

<style>
  .aavc-glyph {
    display: block;
    border-radius: 2px;
    /* A white marker on a dark GCS needs a seam or it bleeds into the chip. */
    outline: 1px solid color-mix(in srgb, var(--color-aavc-border) 80%, transparent);
  }
  .aavc-glyph.dim { opacity: 0.55; }
  .aavc-glyph-missing {
    display: inline-block;
    font-family: var(--font-mono);
    color: var(--color-aavc-critical);
  }
</style>
