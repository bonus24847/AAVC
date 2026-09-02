<script lang="ts">
  // Third-person SPECTATOR view of the drone in Gazebo, for the Tuning right
  // rail. A fixed ground camera (sitl/models/spectator_cam) watches the launch
  // pad / sys-ID hover column, so the operator can WATCH the drone take off,
  // fly the mission from outside the aircraft — the onboard
  // the nadir cam only ever shows the ground.
  //
  //   SPECTATOR → /api/camera/spectator.png (mirror of /tmp/aavc_spectator.png)
  //
  // Same single-frame polling as CameraFeed (Firefox 41+ won't render
  // multipart/x-mixed-replace via <img>, so we poll at ~5 Hz with a cache-bust
  // query). No HUD crosshair — this is a situational view, not an alignment aid.

  import { onMount, onDestroy } from 'svelte';

  const POLL_MS = 200;          // 5 Hz to match the bridge
  const FPS_WINDOW = 8;
  const STALE_MS = 3000;

  const path = '/api/camera/spectator.png';
  let src = $state('');
  let loadTs = $state<number[]>([]);
  let errored = $state(false);
  let now = $state(Date.now());

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
    src = `${path}?t=${t}`;
    now = t;
  }

  function onLoad() {
    errored = false;
    loadTs = [...loadTs, Date.now()].slice(-FPS_WINDOW);
  }
  function onError() {
    errored = true;
  }

  const fps = $derived.by(() => {
    if (loadTs.length < 2) return 0;
    const span = (loadTs[loadTs.length - 1] - loadTs[0]) / 1000;
    return span > 0 ? (loadTs.length - 1) / span : 0;
  });
  const ageMs = $derived(loadTs.length > 0 ? now - loadTs[loadTs.length - 1] : Infinity);
  const noFrames = $derived(loadTs.length === 0);
  const stale = $derived(ageMs > STALE_MS);
  const dead = $derived(errored || noFrames);

  function ageText(ms: number): string {
    if (!isFinite(ms)) return '—';
    if (ms < 1500) return `${ms} ms ago`;
    return `${(ms / 1000).toFixed(1)} s ago`;
  }
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header">
    <span>Gazebo · Spectator</span>
    <span class="ml-auto">
      {#if dead}
        <span class="aavc-chip aavc-chip-critical">no signal</span>
      {:else if stale}
        <span class="aavc-chip aavc-chip-warning">stale · {ageText(ageMs)}</span>
      {:else}
        <span class="aavc-chip aavc-chip-nominal">{fps.toFixed(1)} fps</span>
      {/if}
    </span>
  </div>

  <div class="relative flex-1 min-h-0" style="background: #000;">
    <img
      {src}
      alt="Gazebo spectator camera feed"
      class="w-full h-full object-contain"
      style="opacity: {dead ? 0.05 : 1};"
      onload={onLoad}
      onerror={onError}
    />

    {#if dead}
      <div class="absolute inset-0 flex flex-col items-center justify-center gap-1 px-2 text-center pointer-events-none">
        <div class="font-semibold uppercase tracking-[0.10em]"
             style="color: var(--color-aavc-critical); font-size: 11px;">
          No spectator frame
        </div>
        <div class="font-mono text-[9px]" style="color: var(--color-aavc-ink-mute);">
          start the camera bridge (make camera-bridge)
        </div>
      </div>
    {/if}
  </div>
</div>
