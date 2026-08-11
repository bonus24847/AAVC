<script lang="ts">
  // Kill switch — top-right header. Cuts motors INSTANTLY via /api/cmd/kill
  // (commander.abort -> action.kill). The vehicle drops out of the sky — this
  // is a last-resort emergency control, mirroring the safety pilot's RC kill
  // switch. Unlike RTL/Land/Abort it does NOT land gracefully.
  //
  // Single-click opens a one-click confirm (no type-the-verb) so it stays fast
  // in an emergency while still guarding against a stray misclick. Auto-arms
  // the command session if the operator hasn't armed it.

  import { mission } from '../lib/stores.svelte';
  import { cmdArm, cmdKill } from '../lib/cmd-client';
  import ConfirmModal from './ConfirmModal.svelte';

  let pending = $state(false);
  let busy = $state(false);
  let lastError = $state('');

  const armed = $derived(mission.commandSession.armed);

  async function confirmKill(note: string) {
    pending = false;
    busy = true;
    lastError = '';
    if (!armed) {
      const ra = await cmdArm('auto-arm: kill switch');
      if (!ra.ok) {
        lastError = ra.detail ?? `auto-arm failed (HTTP ${ra.status})`;
        busy = false;
        return;
      }
    }
    const r = await cmdKill(note);
    if (!r.ok) lastError = r.detail ?? `kill failed (HTTP ${r.status})`;
    busy = false;
  }
</script>

<button
  class="aavc-btn aavc-btn-cmd aavc-btn-danger"
  style="padding: 0.25rem 0.9rem; font-size: 11px; font-weight: 700; letter-spacing: 0.12em;"
  disabled={busy}
  onclick={() => (pending = true)}
  title="KILL SWITCH — cut motors instantly (the vehicle drops). Emergency last resort.">
  🛑 KILL
</button>

{#if lastError}
  <span class="font-mono text-[11px] ml-1" style="color: var(--color-aavc-critical);">✗ {lastError}</span>
{/if}

<ConfirmModal
  open={pending}
  title="confirm KILL SWITCH"
  verb="KILL"
  body={
    '⚠ EMERGENCY MOTOR KILL.\n\n' +
    'Cuts ALL motors INSTANTLY (MAVLink force-disarm / action.kill). The vehicle ' +
    'will DROP — there is no landing and no recovery.\n\n' +
    'Use only as a last resort (fly-away, imminent crash) when a graceful ' +
    'RTL / Land is not safe.' +
    (armed ? '' : '\n\nNote: command session not armed — confirming will auto-arm, then kill.')
  }
  strict={false}
  danger={true}
  onConfirm={confirmKill}
  onCancel={() => (pending = false)}
/>
