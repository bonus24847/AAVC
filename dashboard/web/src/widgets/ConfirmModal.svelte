<script lang="ts">
  // Reusable confirm modal for destructive command paths.
  // - strict mode requires the operator to type the verb to enable confirm
  // - Escape cancels, Enter confirms (when canConfirm true) from any
  //   focused input via window-level keydown listener

  interface Props {
    open: boolean;
    title: string;
    verb: string;
    body: string;
    strict?: boolean;
    danger?: boolean;
    onConfirm: (note: string) => void;
    onCancel: () => void;
  }

  let {
    open, title, verb, body,
    strict = false, danger = false,
    onConfirm, onCancel,
  }: Props = $props();

  let typed = $state('');
  let note = $state('');

  $effect(() => {
    if (open) {
      typed = '';
      note = '';
      requestAnimationFrame(() => {
        const target = strict ? 'aavc-modal-typed' : 'aavc-modal-note';
        document.getElementById(target)?.focus();
      });
    }
  });

  const canConfirm = $derived(!strict || typed.trim().toUpperCase() === verb.toUpperCase());

  $effect(() => {
    if (!open) return;
    function global(e: KeyboardEvent) {
      if (e.key === 'Escape') {
        e.preventDefault();
        onCancel();
      } else if (e.key === 'Enter' && canConfirm) {
        e.preventDefault();
        onConfirm(note);
      }
    }
    window.addEventListener('keydown', global);
    return () => window.removeEventListener('keydown', global);
  });
</script>

{#if open}
  <div class="aavc-modal-overlay" role="presentation" tabindex="-1">
    <div class="aavc-modal" role="dialog" aria-modal="true">
      {#if danger}
        <div class="aavc-modal-strip"></div>
      {/if}

      <div class="px-5 pt-4 pb-3 flex items-center justify-between">
        <h2 class="text-[15px] font-semibold uppercase tracking-[0.10em]"
            style="color: {danger ? 'var(--color-aavc-critical)' : 'var(--color-aavc-info)'};">
          {title}
        </h2>
        <span class="aavc-chip font-mono"
              style="color: {danger ? 'var(--color-aavc-critical)' : 'var(--color-aavc-info)'};
                     border-color: {danger ? 'var(--color-aavc-critical)' : 'var(--color-aavc-info)'};">
          {verb}
        </span>
      </div>

      <div class="px-5 pb-4 space-y-3">
        <p class="text-[13px] leading-relaxed" style="color: var(--color-aavc-ink-2);">
          {body}
        </p>

        {#if strict}
          <label for="aavc-modal-typed"
                 class="block text-[11px] uppercase tracking-[0.10em] font-semibold"
                 style="color: var(--color-aavc-ink-mute);">
            Type <span class="font-mono"
                       style="color: var(--color-aavc-critical); font-weight: 600;">{verb}</span> to confirm
          </label>
          <input
            id="aavc-modal-typed"
            class="aavc-input font-mono"
            type="text"
            bind:value={typed}
            spellcheck="false"
            autocomplete="off"
            placeholder={verb}
          />
        {/if}

        <label for="aavc-modal-note"
               class="block text-[11px] uppercase tracking-[0.10em] font-semibold mt-2"
               style="color: var(--color-aavc-ink-mute);">
          Operator note (logged)
        </label>
        <input
          id="aavc-modal-note"
          class="aavc-input"
          type="text"
          bind:value={note}
          spellcheck="false"
          maxlength={200}
          placeholder="e.g. low battery, wind gusting"
        />
      </div>

      <div class="px-5 pb-5 pt-1 flex justify-end gap-3">
        <button class="aavc-btn aavc-btn-ghost" onclick={onCancel}>Cancel</button>
        <button
          class="aavc-btn {danger ? 'aavc-btn-danger' : 'aavc-btn-primary'}"
          disabled={!canConfirm}
          onclick={() => onConfirm(note)}
        >
          Confirm · {verb}
        </button>
      </div>
    </div>
  </div>
{/if}
