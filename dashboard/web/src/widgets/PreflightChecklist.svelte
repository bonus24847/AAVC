<script lang="ts">
  // Pre-flight readiness + mission start. The mission HOLDS in the PREFLIGHT
  // phase (orchestrator/preflight.py) until the operator deliberately starts it
  // — it never auto-launches. This panel is NON-blocking (the operator can switch
  // to the Tuning tab and run System-ID first, with the drone idle) and starts the
  // mission only via a SLIDE-to-confirm, so a stray click can't launch.
  import { mission } from '../lib/stores.svelte';
  import { cmdArm, cmdMissionIds, cmdPreflightGo } from '../lib/cmd-client';
  import type { PreflightItem, PreflightStatus } from '../lib/types';
  import ArucoGlyph from './ArucoGlyph.svelte';

  const report = $derived(mission.preflight);
  const phase = $derived(mission.telemetry?.phase ?? '');
  const view = $derived(mission.activeView);
  const sessionArmed = $derived(mission.commandSession.armed);
  // Active only while the orchestrator is actually holding for a start command.
  const holding = $derived(phase === 'preflight');

  const critical = $derived(report?.items.filter((i) => i.critical) ?? []);
  const advisory = $derived(report?.items.filter((i) => !i.critical) ?? []);
  const greenBoard = $derived(report?.all_critical_pass ?? false);

  let payloadConfirmed = $state(false);
  let forceLaunch = $state(false);
  let goPending = $state(false);
  let goError = $state('');
  let collapsed = $state(false);
  const sortieNo = $derived((mission.telemetry?.sortie_index ?? 0) + 1);
  const maxSorties = $derived(mission.telemetry?.max_sorties || 4);
  // The queue lists DELIVERIES (one pad each); a FLIGHT carries eggs_aboard of
  // them. Everything that counts queue SLOTS must count deliveries — capping
  // the editor at max_sorties would allow one id when all 4 fly in one flight.
  const maxDeliveries = $derived(mission.telemetry?.max_deliveries || 4);
  const eggsAboard = $derived(mission.telemetry?.eggs_aboard || 1);
  const sortieTimeOk = $derived(mission.telemetry?.sortie_time_ok ?? true);
  // Each resupply hold is a NEW sortie: clear the per-sortie inputs. The
  // mission queue is deliberately NOT reset — it spans sorties (server state).
  $effect(() => {
    if (holding) { payloadConfirmed = false; forceLaunch = false; dragX = 0; }
  });

  // ── 4-of-6 ordered mission queue (server-authoritative, echoed via telemetry).
  // The 6-chip grid EDITS the queue: click = append in sortie order / remove;
  // slots already flown (before this sortie) are locked. Each change POSTs
  // /api/cmd/mission_ids; GO then sends assigned_marker_id=null and the backend
  // resolves queue[sortie-1] — one mechanism end-to-end.
  const serverQueue = $derived(mission.telemetry?.assigned_id_queue ?? []);
  let localQueue = $state<number[]>([]);
  let lastServerJson = $state('[]');
  let queuePending = $state(false);
  let queueError = $state('');
  // Adopt the server echo whenever it CHANGES (optimistic local edits survive
  // until the echo confirms or another writer replaces the queue).
  $effect(() => {
    const json = JSON.stringify(serverQueue);
    if (json !== lastServerJson) {
      lastServerJson = json;
      localQueue = [...serverQueue];
    }
  });
  // Queue slots already flown = DELIVERIES done (at one egg per flight this is
  // sortieNo - 1, exactly as before). delivery_index is only bumped when a
  // delivery starts, so at a between-flights hold it IS the flown count.
  const servedCount = $derived(
    Math.max(sortieNo - 1, mission.telemetry?.delivery_index ?? 0));
  // What THIS hold's flight will serve: the next <= eggs_aboard unflown slots.
  const holdIds = $derived(localQueue.slice(servedCount, servedCount + eggsAboard));
  const queuedId = $derived(holdIds.length > 0 ? holdIds[0] : null);
  const queueFull = $derived(localQueue.length >= Math.min(4, maxDeliveries));

  async function toggleQueue(mid: number) {
    if (queuePending) return;
    const pos = localQueue.indexOf(mid);
    if (pos >= 0 && pos < servedCount) return;       // flown slot — locked
    if (pos < 0 && queueFull) return;
    const prev = localQueue;
    const next = pos >= 0 ? localQueue.filter((v) => v !== mid) : [...localQueue, mid];
    localQueue = next;                               // optimistic
    queueError = '';
    queuePending = true;
    try {
      if (!sessionArmed) {
        const ar = await cmdArm('mission queue edit');
        if (!ar.ok) {
          localQueue = prev;
          queueError = ar.detail || 'could not arm command session';
          return;
        }
      }
      const res = await cmdMissionIds(next, `flight ${sortieNo} hold`);
      if (!res.ok) {
        localQueue = prev;
        queueError = res.detail || 'queue update rejected';
      }
    } finally {
      queuePending = false;
    }
  }

  // Show the full card only in the Flight view (so the Tuning tab is never
  // covered); elsewhere just a small pill remains.
  const showCard = $derived(holding && view === 'flight' && !collapsed);
  const showPill = $derived(holding && !showCard);

  const canGo = $derived(greenBoard && payloadConfirmed && queuedId !== null
    && (sortieTimeOk || forceLaunch) && !goPending);

  const STATUS_META: Record<PreflightStatus, { sym: string; color: string }> = {
    pass:    { sym: '✓', color: 'var(--color-aavc-nominal)' },
    fail:    { sym: '✕', color: 'var(--color-aavc-critical)' },
    warn:    { sym: '!', color: 'var(--color-aavc-warning)' },
    pending: { sym: '○', color: 'var(--color-aavc-ink-mute)' },
  };

  async function startMission() {
    goError = '';
    goPending = true;
    try {
      if (!sessionArmed) {
        const ar = await cmdArm('mission start');
        if (!ar.ok) { goError = ar.detail || 'could not arm command session'; return; }
      }
      // null id → the backend resolves this sortie's pad from the queue.
      const res = await cmdPreflightGo(payloadConfirmed, null, forceLaunch,
                                       `flight ${sortieNo} start`);
      if (!res.ok) goError = res.detail || 'start rejected';
    } finally {
      goPending = false;
    }
  }

  function openInFlight() {
    mission.setActiveView('flight');
    collapsed = false;
  }

  // ── slide-to-confirm ──
  let trackEl = $state<HTMLDivElement | null>(null);
  let trackW = $state(0);
  let dragX = $state(0);
  let dragging = $state(false);
  const HANDLE = 54;
  const maxX = $derived(Math.max(0, trackW - HANDLE));
  const progress = $derived(maxX > 0 ? dragX / maxX : 0);

  function onDown(e: PointerEvent) {
    if (!canGo) return;
    dragging = true;
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
  }
  function onMove(e: PointerEvent) {
    if (!dragging || !trackEl) return;
    const rect = trackEl.getBoundingClientRect();
    dragX = Math.max(0, Math.min(maxX, e.clientX - rect.left - HANDLE / 2));
  }
  function onUp() {
    if (!dragging) return;
    dragging = false;
    if (progress >= 0.9) { dragX = maxX; startMission(); }
    else dragX = 0;
  }
</script>

{#if showCard}
  <div class="aavc-preflight-float aavc-panel">
    <div class="aavc-panel-header" style="justify-content: space-between;">
      <span>Flight {sortieNo}/{maxSorties} · pre-flight check</span>
      <span class="flex items-center gap-2">
        <span class="aavc-chip {greenBoard ? 'aavc-chip-nominal' : 'aavc-chip-critical'}">
          {greenBoard ? '● ready' : '○ not ready'}
        </span>
        <button class="aavc-pf-min" title="ย่อ — ทำ Tuning / อย่างอื่นก่อน" onclick={() => (collapsed = true)}>—</button>
      </span>
    </div>

    <div class="aavc-scroll" style="overflow-y: auto; padding: 12px 16px; flex: 1; min-height: 0;">
      {#if !report}
        <div style="color: var(--color-aavc-ink-mute); padding: 16px 0;">evaluating readiness…</div>
      {:else}
        {@render group('Critical — must pass to launch', critical)}
        {@render group('Advisory', advisory)}
      {/if}
    </div>

    <div style="border-top: 1px solid var(--color-aavc-border); padding: 12px 16px;">
      <!-- ordered mission queue (committee-assigned pad ids, in sortie order) -->
      <div class="aavc-payload-confirm" style="display: block;">
        <span style="display: block; margin-bottom: 6px;">Mission queue — click the
          <strong>committee-assigned</strong> pad ids (ArUco) in sortie order,
          click again to remove:</span>
        <div style="display: grid; grid-template-columns: repeat(6, 1fr); gap: 6px;">
          {#each [1, 2, 3, 4, 5, 6] as mid}
            {@const pos = localQueue.indexOf(mid)}
            {@const inQueue = pos >= 0}
            {@const served = inQueue && pos < servedCount}
            <button class="aavc-chip aavc-qchip {inQueue ? 'aavc-chip-caution' : ''}"
                    class:served
                    style="border: 1px solid {inQueue ? 'var(--color-aavc-warning)' : 'var(--color-aavc-border)'};"
                    disabled={served || (!inQueue && queueFull) || queuePending}
                    title={served ? `pad ${mid}: delivery ${pos + 1} flown` : `ArUco pad ${mid}`}
                    onclick={() => toggleQueue(mid)}>
              <ArucoGlyph id={mid} size={30} dim={served} />
              <span class="aavc-qcap">{mid}</span>
              {#if inQueue}<span class="aavc-qpos">{served ? '✓' : pos + 1}</span>{/if}
            </button>
          {/each}
        </div>
        <div class="aavc-qstrip">
          {#each Array.from({ length: maxDeliveries }, (_, i) => i + 1) as s}
            <span class="aavc-qslot"
                  class:now={s > servedCount && s <= servedCount + eggsAboard}
                  class:done={s <= servedCount}>
              {s}→{#if localQueue[s - 1]}<ArucoGlyph id={localQueue[s - 1]} size={15}
                dim={s <= servedCount} /><span class="aavc-qcap">{localQueue[s - 1]}</span>{:else}·{/if}
            </span>
          {/each}
        </div>
        {#if queueError}
          <div class="aavc-preflight-hint" style="color: var(--color-aavc-critical);">{queueError}</div>
        {/if}
      </div>

      <label class="aavc-payload-confirm">
        <input type="checkbox" bind:checked={payloadConfirmed} />
        <span><strong>Egg cargo (×{eggsAboard})</strong> loaded &amp; secured</span>
      </label>

      {#if !sortieTimeOk}
        <label class="aavc-payload-confirm" style="color: var(--color-aavc-critical);">
          <input type="checkbox" bind:checked={forceLaunch} />
          <span>window can't cover another sortie — <strong>FORCE</strong>
            late launch (overtime penalty)</span>
        </label>
      {/if}

      {#if goError}
        <div class="aavc-preflight-hint" style="color: var(--color-aavc-critical);">{goError}</div>
      {/if}

      <!-- slide to start -->
      <div class="aavc-slide-track" class:disabled={!canGo} bind:this={trackEl} bind:clientWidth={trackW}>
        <div class="aavc-slide-fill" style="width: {dragX + HANDLE}px;"></div>
        <span class="aavc-slide-label">
          {goPending ? 'starting…'
            : !greenBoard ? 'critical checks not passed'
            : queuedId === null ? 'queue a pad id for this flight ↑'
            : !payloadConfirmed ? 'confirm the egg cargo ↑'
            : !sortieTimeOk && !forceLaunch ? 'window too short — tick FORCE ↑'
            : `slide to launch flight ${sortieNo} → pad${holdIds.length > 1 ? 's' : ''} ${holdIds.join(', ')} (queue)  ▸▸▸`}
        </span>
        <div class="aavc-slide-handle" style="transform: translateX({dragX}px); transition: {dragging ? 'none' : 'transform 0.18s'};"
             onpointerdown={onDown} onpointermove={onMove} onpointerup={onUp}>▸</div>
      </div>

    </div>
  </div>
{/if}

{#if showPill}
  <button class="aavc-preflight-pill {greenBoard ? 'ready' : ''}" onclick={openInFlight}>
    {greenBoard ? '✓ Ready' : '○ Pre-flight'} · ▸ start mission
  </button>
{/if}

{#snippet group(title: string, items: PreflightItem[])}
  <div style="margin-bottom: 12px;">
    <div class="aavc-preflight-grouphdr">{title}</div>
    {#each items as it (it.id)}
      <div class="aavc-preflight-row">
        <span class="aavc-preflight-sym" style="color: {STATUS_META[it.status].color};">
          {STATUS_META[it.status].sym}
        </span>
        <span class="aavc-preflight-label">{it.label}</span>
        <span class="aavc-preflight-detail">{it.detail}</span>
      </div>
    {/each}
  </div>
{/snippet}

<style>
  /* Non-blocking floating card — sits below the header so the Flight|Tuning
     toggle + command bar stay usable; no dark backdrop. */
  .aavc-preflight-float {
    position: fixed;
    top: 68px;
    left: 50%;
    transform: translateX(-50%);
    width: min(620px, 94vw);
    max-height: 72vh;
    z-index: 40;
    display: flex;
    flex-direction: column;
    box-shadow: 0 24px 64px rgba(0, 0, 0, 0.7);
  }
  .aavc-pf-min {
    width: 22px; height: 22px; line-height: 1;
    border: 1px solid var(--color-aavc-border);
    border-radius: 4px; background: var(--color-aavc-panel-2);
    color: var(--color-aavc-ink-dim); cursor: pointer; font-size: 14px;
  }
  .aavc-pf-min:hover { filter: brightness(1.2); }
  .aavc-preflight-pill {
    position: fixed;
    bottom: 110px;
    left: 50%;
    transform: translateX(-50%);
    z-index: 40;
    padding: 7px 16px;
    border-radius: 999px;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.02em;
    border: 1px solid var(--color-aavc-warning);
    background: color-mix(in srgb, var(--color-aavc-warning) 18%, var(--color-aavc-panel));
    color: var(--color-aavc-ink);
    cursor: pointer;
    box-shadow: 0 6px 20px rgba(0, 0, 0, 0.5);
  }
  .aavc-preflight-pill.ready {
    border-color: var(--color-aavc-nominal);
    background: color-mix(in srgb, var(--color-aavc-nominal) 18%, var(--color-aavc-panel));
  }
  .aavc-preflight-pill:hover { filter: brightness(1.1); }
  .aavc-preflight-grouphdr {
    font-size: 10px; text-transform: uppercase; letter-spacing: 0.12em;
    color: var(--color-aavc-ink-mute); margin: 0 0 6px;
  }
  .aavc-preflight-row {
    display: grid;
    grid-template-columns: 18px minmax(150px, auto) 1fr;
    align-items: baseline; gap: 10px; padding: 4px 0;
    border-bottom: 1px solid color-mix(in srgb, var(--color-aavc-border) 60%, transparent);
  }
  .aavc-preflight-sym { font-weight: 700; text-align: center; }
  .aavc-preflight-label { color: var(--color-aavc-ink); font-size: 13px; }
  .aavc-preflight-detail {
    color: var(--color-aavc-ink-mute);
    font-family: 'IBM Plex Mono', ui-monospace, monospace;
    font-size: 11px; text-align: right;
  }
  .aavc-payload-confirm {
    display: flex; align-items: center; gap: 8px;
    color: var(--color-aavc-ink-dim); font-size: 13px; cursor: pointer; margin-bottom: 10px;
  }
  .aavc-payload-confirm input { width: 16px; height: 16px; accent-color: var(--color-aavc-warning); }

  /* mission-queue editor chips + plan strip */
  .aavc-qchip {
    position: relative; cursor: pointer;
    font-family: var(--font-mono);
    /* glyph over a small numeric caption — the picture is what the operator
       matches against the committee's card; the id stays for radio calls. */
    display: flex; flex-direction: column; align-items: center; gap: 3px;
    padding: 6px 4px;
  }
  .aavc-qcap {
    font-family: var(--font-mono); font-size: 10px; line-height: 1;
    color: var(--color-aavc-ink-mute); letter-spacing: 0.04em;
  }
  .aavc-qchip:disabled { cursor: not-allowed; opacity: 0.55; }
  .aavc-qchip.served { opacity: 0.45; }
  .aavc-qpos {
    position: absolute; top: -6px; right: -4px;
    min-width: 14px; height: 14px; padding: 0 2px;
    border-radius: 999px; font-size: 9px; line-height: 14px; font-weight: 800;
    background: var(--color-aavc-warning); color: #1a1200; text-align: center;
  }
  .aavc-qchip.served .aavc-qpos { background: var(--color-aavc-nominal); color: #04140a; }
  .aavc-qstrip {
    display: flex; gap: 10px; margin-top: 8px;
    font-family: var(--font-mono); font-size: 11px;
    color: var(--color-aavc-ink-mute);
  }
  .aavc-qslot { display: inline-flex; align-items: center; gap: 3px; }
  .aavc-qslot.now { color: var(--color-aavc-warning); font-weight: 700; }
  .aavc-qslot.done { text-decoration: line-through; opacity: 0.6; }
  .aavc-preflight-hint { font-size: 11px; color: var(--color-aavc-ink-mute); margin-bottom: 8px; }

  /* slide-to-confirm */
  .aavc-slide-track {
    position: relative;
    height: 48px;
    border-radius: 8px;
    background: var(--color-aavc-panel-2);
    border: 1px solid var(--color-aavc-border);
    overflow: hidden;
    user-select: none;
    touch-action: none;
  }
  .aavc-slide-track.disabled { opacity: 0.5; }
  .aavc-slide-fill {
    position: absolute; inset: 0 auto 0 0;
    background: color-mix(in srgb, var(--color-aavc-nominal) 45%, transparent);
    pointer-events: none;
  }
  .aavc-slide-label {
    position: absolute; inset: 0;
    display: flex; align-items: center; justify-content: center;
    font-size: 13px; font-weight: 600; letter-spacing: 0.04em;
    color: var(--color-aavc-ink-dim); pointer-events: none;
  }
  .aavc-slide-handle {
    position: absolute; top: 3px; left: 3px;
    width: 48px; height: 42px;
    display: flex; align-items: center; justify-content: center;
    border-radius: 6px;
    background: var(--color-aavc-nominal);
    color: #04140a; font-size: 18px; font-weight: 800;
    cursor: grab; touch-action: none;
  }
  .aavc-slide-track.disabled .aavc-slide-handle { cursor: not-allowed; background: var(--color-aavc-ink-mute); }
</style>
