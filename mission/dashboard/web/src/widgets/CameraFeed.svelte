<script lang="ts">
  // Single camera feed: NADIR (down-looking, gimbal-stabilized — alignment /
  // descent / landing ON the assigned pad). Polled single-frame snapshots
  // rather than the MJPEG multipart stream — Firefox 41+ no longer renders
  // multipart/x-mixed-replace via <img>, so polling at ~5 Hz gives the same
  // effective rate with universal browser support.
  //
  //   NADIR → /api/camera/frame.png (mirror of /tmp/aavc_nadir.png)
  //
  // The bridge updates the underlying file at ~5 Hz, and the endpoint
  // re-reads it per request.

  import { onMount, onDestroy } from 'svelte';

  const POLL_MS = 200;          // 5 Hz to match the bridge
  const FPS_WINDOW = 8;
  const STALE_MS = 3000;

  interface Feed {
    label: string;
    path: string;
    src: string;
    load_ts: number[];
    errored: boolean;
  }

  let nadir = $state<Feed>({
    label: 'Nadir', path: '/api/camera/frame.png', src: '', load_ts: [], errored: false,
  });
  let now = $state(Date.now());

  function onLoad(feed: Feed) {
    feed.errored = false;
    feed.load_ts = [...feed.load_ts, Date.now()].slice(-FPS_WINDOW);
  }
  function onError(feed: Feed) {
    feed.errored = true;
  }

  let pollTimer: number | undefined;
  onMount(() => {
    tick();
    pollTimer = window.setInterval(tick, POLL_MS);
  });
  onDestroy(() => {
    if (pollTimer !== undefined) clearInterval(pollTimer);
  });
  function tick() {
    const t = Date.now();
    nadir.src = `${nadir.path}?t=${t}`;
    now = t;
  }

  function fps(feed: Feed): number {
    const ts = feed.load_ts;
    if (ts.length < 2) return 0;
    const span = (ts[ts.length - 1] - ts[0]) / 1000;
    return span > 0 ? (ts.length - 1) / span : 0;
  }
  function ageMs(feed: Feed): number {
    return feed.load_ts.length > 0 ? now - feed.load_ts[feed.load_ts.length - 1] : Infinity;
  }
  function noFrames(feed: Feed): boolean {
    return feed.load_ts.length === 0;
  }
  function ageText(ms: number): string {
    if (!isFinite(ms)) return '—';
    if (ms < 1500) return `${ms} ms ago`;
    return `${(ms / 1000).toFixed(1)} s ago`;
  }
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Cameras</span>
    <span class="ml-auto text-[10px] normal-case font-normal"
          style="color: var(--color-aavc-ink-mute);">nadir</span>
  </div>

  <div class="flex-1 min-h-0 grid gap-1.5 p-1.5" style="grid-template-columns: 1fr;">
    {#each [nadir] as feed (feed.label)}
      {@const stale = ageMs(feed) > STALE_MS}
      {@const dead = feed.errored || noFrames(feed)}
      <div class="flex flex-col min-w-0 overflow-hidden rounded"
           style="border: 1px solid var(--color-aavc-border); background: #000;">
        <div class="flex items-center justify-between px-2 py-1 shrink-0"
             style="background: var(--color-aavc-panel-2); border-bottom: 1px solid var(--color-aavc-border);">
          <span class="text-[10px] uppercase tracking-[0.10em] font-semibold"
                style="color: var(--color-aavc-ink-dim);">{feed.label}</span>
          {#if dead}
            <span class="aavc-chip aavc-chip-critical">no signal</span>
          {:else if stale}
            <span class="aavc-chip aavc-chip-warning">stale · {ageText(ageMs(feed))}</span>
          {:else}
            <span class="aavc-chip aavc-chip-nominal">{fps(feed).toFixed(1)} fps</span>
          {/if}
        </div>

        <div class="relative flex-1 min-h-0">
          <img
            src={feed.src}
            alt="{feed.label} camera feed"
            class="w-full h-full object-contain"
            style="opacity: {dead ? 0.05 : 1};"
            onload={() => onLoad(feed)}
            onerror={() => onError(feed)}
          />

          {#if dead}
            <div class="absolute inset-0 flex flex-col items-center justify-center gap-1 px-2 text-center pointer-events-none">
              <div class="font-semibold uppercase tracking-[0.10em]"
                   style="color: var(--color-aavc-critical); font-size: 11px;">
                No frame
              </div>
              <div class="font-mono text-[9px]" style="color: var(--color-aavc-ink-mute);">
                {feed.path}
              </div>
            </div>
          {:else}
            <!-- HUD crosshair overlay (alignment reference for the landing pad) -->
            <div class="absolute inset-0 pointer-events-none"
                 style="background:
                   linear-gradient(transparent calc(50% - 0.5px), rgba(34, 197, 94, 0.22) 50%, transparent calc(50% + 0.5px)),
                   linear-gradient(90deg, transparent calc(50% - 0.5px), rgba(34, 197, 94, 0.22) 50%, transparent calc(50% + 0.5px));"></div>
          {/if}
        </div>
      </div>
    {/each}
  </div>
</div>
