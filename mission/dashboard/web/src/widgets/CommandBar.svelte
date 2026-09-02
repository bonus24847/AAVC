<script lang="ts">
  // Horizontal command bar under the centre column. Big tap targets so the
  // operator can drive it standing / hot-seated at competition.
  //
  // Layout: ARM-to-command toggle | vehicle arm + flight verbs (Takeoff /
  // Hold / Resume / Drop) | destructive verbs (RTL / Land / Abort, red).
  //
  // Destructive verbs go through ConfirmModal (single-click confirm, no
  // type-the-verb so an emergency RTL/LAND/ABORT isn't slowed down). Takeoff
  // also confirms (props-clear check). Everything is sent via cmd-client.ts,
  // which stamps the X-AAVC-CMD header.

  import { mission } from '../lib/stores.svelte';
  import {
    cmdArm, cmdDisarm,
    cmdVehicleArm, cmdVehicleDisarm,
    cmdTakeoff, cmdHold, cmdResume, cmdRTL, cmdLand, cmdAbort, cmdDrop,
  } from '../lib/cmd-client';
  import ConfirmModal from './ConfirmModal.svelte';

  type PendingVerb = null | 'takeoff' | 'rtl' | 'land' | 'abort';
  let pending = $state<PendingVerb>(null);
  let busy = $state(false);
  let lastError = $state('');

  const armed = $derived(mission.commandSession.armed);
  const vehicleArmed = $derived(mission.telemetry?.is_armed ?? false);
  const linkUp = $derived(mission.telemetry?.link_connected ?? false);
  const planLoaded = $derived(!!mission.plan);
  const lastResult = $derived(
    mission.commandResults.length > 0
      ? mission.commandResults[mission.commandResults.length - 1]
      : null,
  );

  async function toggleArm() {
    busy = true;
    lastError = '';
    const r = armed ? await cmdDisarm('arm toggle') : await cmdArm('arm toggle');
    if (!r.ok) lastError = r.detail ?? `failed (HTTP ${r.status})`;
    busy = false;
  }

  // Vehicle arm/disarm (PX4 motors) — required before TAKEOFF.
  async function toggleVehicleArm() {
    busy = true;
    lastError = '';
    if (!(await ensureArmedFor('vehicle arm'))) { busy = false; return; }
    const r = vehicleArmed
      ? await cmdVehicleDisarm('operator disarm')
      : await cmdVehicleArm('operator arm');
    if (!r.ok) lastError = r.detail ?? `failed (HTTP ${r.status})`;
    busy = false;
  }

  type SafeVerb = 'hold' | 'resume' | 'drop';
  async function sendSafe(verb: SafeVerb) {
    busy = true;
    lastError = '';
    if (!(await ensureArmedFor(verb))) { busy = false; return; }
    const fns: Record<SafeVerb, typeof cmdResume> = {
      hold: cmdHold, resume: cmdResume, drop: cmdDrop,
    };
    const r = await fns[verb]('');
    if (!r.ok) lastError = r.detail ?? `failed (HTTP ${r.status})`;
    busy = false;
  }

  /** Some verbs are gated server-side on an armed command session. Rather than
   * make the operator hunt for the ARM toggle mid-task, auto-arm with an
   * explicit note (captured in the audit log). Returns true once the session
   * is armed, false if /arm failed (caller aborts). */
  async function ensureArmedFor(verb: string): Promise<boolean> {
    if (armed) return true;
    const r = await cmdArm(`auto-arm: ${verb}`);
    if (!r.ok) {
      lastError = r.detail ?? `auto-arm failed (HTTP ${r.status})`;
      return false;
    }
    return true;
  }

  async function confirmDestructive(note: string) {
    if (!pending) return;
    const verb = pending;
    pending = null;
    busy = true;
    lastError = '';
    if (!(await ensureArmedFor(verb))) { busy = false; return; }
    let r;
    switch (verb) {
      case 'takeoff': r = await cmdTakeoff(note); break;
      case 'rtl':     r = await cmdRTL(note); break;
      case 'land':    r = await cmdLand(note); break;
      case 'abort':   r = await cmdAbort(note); break;
    }
    if (r && !r.ok) lastError = r.detail ?? `failed (HTTP ${r.status})`;
    busy = false;
  }
</script>

<div class="aavc-panel" style="height: 96px;">
  <div class="flex items-center gap-4 px-4 h-full">

    <!-- ARM-to-command toggle -->
    <div class="flex flex-col items-start gap-1 pr-4 border-r"
         style="border-color: var(--color-aavc-border); min-width: 168px;">
      <span class="text-[10px] uppercase tracking-[0.10em] font-semibold"
            style="color: var(--color-aavc-ink-mute);">Arm to command</span>
      <label class="aavc-toggle">
        <input type="checkbox" checked={armed} disabled={busy} onchange={toggleArm} />
        <span class="aavc-toggle-track"></span>
        <span class="font-mono text-[12px] font-semibold"
              style="color: {armed ? 'var(--color-aavc-critical)' : 'var(--color-aavc-ink-mute)'};">
          {armed ? 'ARMED' : 'DISARMED'}
        </span>
      </label>
    </div>

    <!-- Vehicle arm + flight verbs -->
    <div class="flex items-center gap-2">
      <button class="aavc-btn aavc-btn-cmd {vehicleArmed ? 'aavc-btn-warning' : ''}"
              disabled={busy || !linkUp}
              onclick={toggleVehicleArm}
              title="Arm / disarm PX4 vehicle motors. Required before TAKEOFF (takeoff does not auto-arm).">
        {vehicleArmed ? 'DISARM ✈' : 'ARM ✈'}
      </button>
      <button class="aavc-btn aavc-btn-cmd aavc-btn-primary"
              disabled={busy || !linkUp}
              onclick={() => (pending = 'takeoff')}
              title="Climb to takeoff altitude. Vehicle must be armed first (ARM ✈).">TAKEOFF</button>
      <button class="aavc-btn aavc-btn-cmd"
              disabled={busy || !linkUp || !planLoaded}
              onclick={() => sendSafe('resume')}
              title="AUTO.MISSION — start / resume the uploaded plan">RESUME</button>
      <button class="aavc-btn aavc-btn-cmd"
              disabled={busy || !linkUp}
              onclick={() => sendSafe('hold')}
              title="Hold position — pause the active mission">HOLD</button>
      <button class="aavc-btn aavc-btn-cmd aavc-btn-warning"
              disabled={busy || !linkUp}
              onclick={() => sendSafe('drop')}
              title="Release the egg (touchdown-gated: refused while airborne unless forced)">DROP</button>
    </div>

    <!-- Destructive verbs. Auto-arm via the confirm-modal handler so an
         emergency RTL / LAND / ABORT is never blocked on session state. -->
    <div class="flex items-center gap-2 pl-3 ml-auto border-l"
         style="border-color: var(--color-aavc-border);">
      <button class="aavc-btn aavc-btn-cmd aavc-btn-danger"
              disabled={busy || !linkUp}
              onclick={() => (pending = 'rtl')}
              title="Return to launch (auto-arms session if needed)">RTL</button>
      <button class="aavc-btn aavc-btn-cmd aavc-btn-danger"
              disabled={busy || !linkUp}
              onclick={() => (pending = 'land')}
              title="Land in place (auto-arms session if needed)">LAND</button>
      <button class="aavc-btn aavc-btn-cmd aavc-btn-danger"
              disabled={busy || !linkUp}
              onclick={() => (pending = 'abort')}
              title="Hard abort + land (auto-arms session if needed)">ABORT</button>
    </div>
  </div>

  <!-- Result strip -->
  {#if lastError || lastResult}
    <div class="px-4 py-1.5 border-t font-mono text-[11px] tabular-nums"
         style="border-color: var(--color-aavc-border);">
      {#if lastError}
        <span style="color: var(--color-aavc-critical);">✗ {lastError}</span>
      {:else if lastResult}
        <span style="color: {lastResult.ok ? 'var(--color-aavc-nominal)' : 'var(--color-aavc-critical)'};">
          {lastResult.ok ? '✓' : '✗'} {lastResult.command.toUpperCase()} · {lastResult.detail}
        </span>
      {/if}
    </div>
  {/if}
</div>

<ConfirmModal
  open={pending === 'takeoff'}
  title="confirm takeoff"
  verb="TAKEOFF"
  body={
    'Vehicle will climb to the configured takeoff altitude (within the profile ceiling). ' +
    'Ensure props are clear and you have visual contact.' +
    (armed ? '' : '\n\nNote: the command session is not armed — confirming will auto-arm with note "auto-arm: takeoff".')
  }
  strict={false}
  danger={false}
  onConfirm={confirmDestructive}
  onCancel={() => (pending = null)}
/>

<ConfirmModal
  open={pending === 'rtl'}
  title="confirm return-to-launch"
  verb="RTL"
  body={
    'Aircraft will abort the current task and fly home. Session auto-disarms after RTL completes.' +
    (armed ? '' : '\n\nNote: the command session is not armed — confirming will auto-arm with note "auto-arm: rtl".')
  }
  strict={false}
  danger={true}
  onConfirm={confirmDestructive}
  onCancel={() => (pending = null)}
/>

<ConfirmModal
  open={pending === 'land'}
  title="confirm land in place"
  verb="LAND"
  body={
    'Aircraft will descend to the ground at the current position. Session auto-disarms after LAND completes.' +
    (armed ? '' : '\n\nNote: the command session is not armed — confirming will auto-arm with note "auto-arm: land".')
  }
  strict={false}
  danger={true}
  onConfirm={confirmDestructive}
  onCancel={() => (pending = null)}
/>

<ConfirmModal
  open={pending === 'abort'}
  title="confirm abort"
  verb="ABORT"
  body={
    'Marks the mission terminal and lands. Use only when the mission is not salvageable. Session auto-disarms after ABORT completes.' +
    (armed ? '' : '\n\nNote: the command session is not armed — confirming will auto-arm with note "auto-arm: abort".')
  }
  strict={false}
  danger={true}
  onConfirm={confirmDestructive}
  onCancel={() => (pending = null)}
/>
