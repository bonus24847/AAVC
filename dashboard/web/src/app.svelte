<script lang="ts">
  import { onMount } from 'svelte';
  import { mission, loadStatic, refreshStatic } from './lib/stores.svelte';
  import { RealtimeClient } from './lib/ws.svelte';
  import type {
    AnomalyEvent, AutotuneStatus, CommandEvent, CommandResultEvent, CommandSessionEvent,
    DetectedObjectEvent, DropPredictionEvent, HelloPayload,
    PlanUpdate, PreflightReport, SysIdResult, SysIdStatus, TelemetryFrame,
    TunerApplyResult, TunerDesign, VisionEvent,
  } from './lib/types';

  import MissionStatus from './widgets/MissionStatus.svelte';
  import AnomalyBanner from './widgets/AnomalyBanner.svelte';
  import TelemetrySidebar from './widgets/TelemetrySidebar.svelte';
  import MapView from './widgets/MapView.svelte';
  import CameraFeed from './widgets/CameraFeed.svelte';
  import AttitudeIndicator from './widgets/AttitudeIndicator.svelte';
  import CommandBar from './widgets/CommandBar.svelte';
  import LogStrip from './widgets/LogStrip.svelte';
  import KillSwitch from './widgets/KillSwitch.svelte';
  import PreflightChecklist from './widgets/PreflightChecklist.svelte';
  import SystemIdTunerPanel from './widgets/SystemIdTunerPanel.svelte';
  import GazeboViewPanel from './widgets/GazeboViewPanel.svelte';
  import TuningMonitorPanel from './widgets/TuningMonitorPanel.svelte';

  const wsUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/realtime';
  const client = new RealtimeClient(wsUrl);

  // Two SEPARATE programs share one bundle. The launcher's URL picks the view
  //   ?mode=tuning  → System-ID + Autotune tool only (no mission)
  //   ?mode=flight  → Flight Mission GCS only
  // If the query is MISSING (e.g. a reused browser tab dropped it), fall back to
  // the backend's own reported program (/api/health → app_mode) so the page can
  // never land in the wrong view. `null` = still resolving (brief, localhost).
  const urlMode = new URLSearchParams(location.search).get('mode');
  let APP_MODE = $state<'flight' | 'tuning' | null>(
    urlMode === 'tuning' ? 'tuning' : urlMode === 'flight' ? 'flight' : null,
  );

  // Mirror the resolved mode into the store (PreflightChecklist etc. read it).
  $effect(() => { if (APP_MODE) mission.setActiveView(APP_MODE); });

  onMount(() => {
    // No explicit ?mode= in the URL → ask the backend which program it is.
    if (APP_MODE === null) {
      fetch('/api/health')
        .then((r) => (r.ok ? r.json() : null))
        .then((h) => { APP_MODE = h && h.app_mode === 'tuning' ? 'tuning' : 'flight'; })
        .catch(() => { APP_MODE = 'flight'; });
    }
    loadStatic();
    client.on('hello', (p) => { mission.applyHello(p as HelloPayload); refreshStatic(); });
    client.on('telemetry', (p) => mission.applyTelemetry(p as TelemetryFrame));
    client.on('vision', (p) => mission.appendVision(p as VisionEvent));
    client.on('command', (p) => mission.appendCommand(p as CommandEvent));
    client.on('anomaly', (p) => mission.appendAnomaly(p as AnomalyEvent));
    client.on('drop_prediction', (p) => mission.appendDropPrediction(p as DropPredictionEvent));
    client.on('detected_object', (p) => mission.appendDetectedObject(p as DetectedObjectEvent));
    client.on('command_session', (p) => mission.applyCommandSession(p as CommandSessionEvent));
    client.on('command_result', (p) => mission.appendCommandResult(p as CommandResultEvent));
    client.on('preflight', (p) => mission.applyPreflight(p as PreflightReport));
    client.on('plan_update', (p) => mission.applyPlanUpdate(p as PlanUpdate));
    client.on('sysid_status', (p) => mission.applySysIdStatus(p as SysIdStatus));
    client.on('sysid_result', (p) => mission.applySysIdResult(p as SysIdResult));
    client.on('tuner_design', (p) => mission.applyTunerDesign(p as TunerDesign));
    client.on('tuner_apply', (p) => mission.applyTunerApply(p as TunerApplyResult));
    client.on('autotune_status', (p) => mission.applyAutotuneStatus(p as AutotuneStatus));
    client.start();
  });
</script>

<!--
  Competition GCS layout (single Flight view):

    ┌──────────────────────────────────────────────────────────┐
    │ header  (uplink · ARMED · AnomalyBanner · KILL)          │  shrink-0
    ├──────────────────────────────────────────────────────────┤
    │ MissionStatus                                            │  shrink-0
    ├──────────┬──────────────────────────────┬────────────────┤
    │          │  MAP                          │                │  flex-1
    │ Telem    ├───────────────┬──────────────┤   LogStrip     │  min-h-0
    │ Sidebar  │  CAMERAS x2   │  ATTITUDE    │  (MAVLink /    │
    │          │ (nadir+obliq) │              │   anomalies)   │
    ├──────────┴───────────────┴──────────────┴────────────────┤
    │ CommandBar                                               │  shrink-0
    └──────────────────────────────────────────────────────────┘
-->

{#if APP_MODE === null}
  <div class="h-full flex items-center justify-center text-sm" style="color: var(--color-aavc-ink-mute);">
    connecting to backend…
  </div>
{:else}
<div class="h-full flex flex-col gap-2.5 p-2.5">

  <!-- ============ HEADER ============ -->
  <header class="flex items-center justify-between gap-4 px-4 py-2.5 rounded-[10px]"
          style="background: var(--color-aavc-panel);
                 border: 1px solid var(--color-aavc-border);
                 box-shadow: 0 1px 0 0 color-mix(in srgb, #fff 4%, transparent) inset;">
    <div class="flex items-center gap-4 flex-wrap">
      <div class="flex items-center gap-2.5">
        <span style="display:inline-block;width:4px;height:20px;border-radius:2px;
                     background:{APP_MODE === 'tuning' ? 'var(--color-aavc-info)' : 'var(--color-aavc-accent)'};
                     box-shadow:0 0 10px color-mix(in srgb, {APP_MODE === 'tuning' ? 'var(--color-aavc-info)' : 'var(--color-aavc-accent)'} 60%, transparent);"></span>
        <div class="text-[19px] font-bold" style="letter-spacing:0.04em;color: var(--color-aavc-ink);">
          AAVC<span style="color: var(--color-aavc-ink-dim);font-weight:500;">·{APP_MODE === 'tuning' ? 'TUNING' : 'GCS'}</span>
        </div>
        <span class="aavc-chip" style="border-color: {APP_MODE === 'tuning' ? 'var(--color-aavc-info)' : 'var(--color-aavc-accent)'};
                     color: {APP_MODE === 'tuning' ? 'var(--color-aavc-info)' : 'var(--color-aavc-accent)'};">
          {APP_MODE === 'tuning' ? 'System-ID + Autotune (no mission)' : 'Flight Mission'}
        </span>
      </div>
      <span class="aavc-chip {client.connected ? 'aavc-chip-nominal' : 'aavc-chip-critical'}">
        {client.connected ? '● uplink' : '○ uplink down'}
      </span>
      {#if mission.commandSession.armed}
        <span class="aavc-chip aavc-chip-armed">▲ CMD ARMED</span>
      {/if}
    </div>
    <div class="flex items-center gap-3">
      <AnomalyBanner />
      <KillSwitch />
    </div>
  </header>

  {#if APP_MODE === 'flight'}
    <!-- ============ MISSION STATUS STRIP ============ -->
    <MissionStatus />

    <!-- ============ MAIN ROW ============
         280 telemetry | 1fr center (map + camera/attitude) | 360 log strip -->
    <div class="flex-1 min-h-0 grid gap-2"
         style="grid-template-columns: 280px 1fr 360px;">

      <!-- LEFT: telemetry sidebar -->
      <div class="min-h-0">
        <TelemetrySidebar />
      </div>

      <!-- CENTER: map on top (60%), cameras + attitude below (40%) -->
      <div class="min-h-0 grid gap-2" style="grid-template-rows: 3fr 2fr;">
        <div class="min-h-0">
          <MapView />
        </div>
        <div class="min-h-0 grid gap-2" style="grid-template-columns: 2fr 1fr;">
          <div class="min-h-0">
            <CameraFeed />
          </div>
          <div class="min-h-0">
            <AttitudeIndicator />
          </div>
        </div>
      </div>

      <!-- RIGHT: MAVLink + anomaly log -->
      <div class="min-h-0">
        <LogStrip />
      </div>
    </div>
  {:else}
    <!-- ============ TUNING PROGRAM (System-ID + Autotune) ============
         5-step wizard on the left; a persistent RIGHT RAIL (Gazebo spectator
         view over a compact flight monitor) so the operator can WATCH the drone
         take off, chirp-sweep and land across all five steps — the wizard panel
         owns the step state, the rail lives outside it so it never unmounts. -->
    <div class="flex-1 min-h-0 grid gap-2" style="grid-template-columns: minmax(0,1fr) 400px;">
      <div class="min-h-0">
        <SystemIdTunerPanel />
      </div>
      <div class="min-h-0 grid gap-2" style="grid-template-rows: minmax(0,3fr) minmax(0,2fr);">
        <div class="min-h-0"><GazeboViewPanel /></div>
        <div class="min-h-0"><TuningMonitorPanel /></div>
      </div>
    </div>
  {/if}

  <!-- ============ COMMAND BAR (both modes — arm session + RTL/LAND/KILL safety) ============ -->
  <CommandBar />
</div>

{#if APP_MODE === 'flight'}
  <!-- Pre-flight readiness + slide-to-start — flight mission only. -->
  <PreflightChecklist />
{/if}
{/if}
