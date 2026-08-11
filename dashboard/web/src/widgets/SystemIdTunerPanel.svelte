<script lang="ts">
  // System-ID + Autotune — the combined pre-flight tuning module (NOT the scored
  // sortie). Driven as an explicit 5-step wizard: a horizontal step rail (each
  // node turns GREEN when complete, the active one is highlighted) over a single
  // focused working pane. Flow: ① Arm session → ② Vehicle params → ③ Sys-ID chirp
  // sweep → ④ Design gains (model-based or PX4 autotune) → ⑤ Apply + save (the
  // flight mission auto-loads the saved gains). All flight actions need the
  // command session armed.
  import { onMount } from 'svelte';
  import { mission } from '../lib/stores.svelte';
  import {
    autotuneAbort, autotuneStart, cmdArm, sysidRun, tunerApply, tunerDesign, tunerParams,
  } from '../lib/cmd-client';
  import { fmtNum } from '../lib/format';
  import type { FrfDict, PlantFitDict } from '../lib/types';

  // EFT X6100 + EFT E5 power system, from Power-System-Guide-1.pdf — these must
  // stay in step with sitl/models/eft_x6100*. Operator-editable; re-check the
  // mass against the real weigh-in at G5.
  let physical = $state({
    mass_kg: 7.17, ixx: 0.34, iyy: 0.34, izz: 0.64,
    arm_length_m: 0.50, n_motors: 6, max_thrust_per_motor_n: 37.65,
    prop_torque_coeff: 0.02, motor_time_constant_s: 0.03,
  });
  let spec = $state({
    rate_bandwidth_hz: 6.0, rate_damping: 0.7,
    velocity_bandwidth_hz: 0.5, time_scale_factor: 4.0,
  });

  // Friendly label + unit + hover hint for every input, so the tab reads without
  // a control-systems background (raw keys like `ixx` mean nothing to most users).
  const META: Record<string, { label: string; unit: string; hint: string }> = {
    mass_kg: { label: 'Mass', unit: 'kg', hint: 'มวลรวมทั้งลำ (รวม payload)' },
    ixx: { label: 'Roll inertia Ixx', unit: 'kg·m²', hint: 'ความเฉื่อยรอบแกน roll' },
    iyy: { label: 'Pitch inertia Iyy', unit: 'kg·m²', hint: 'ความเฉื่อยรอบแกน pitch' },
    izz: { label: 'Yaw inertia Izz', unit: 'kg·m²', hint: 'ความเฉื่อยรอบแกน yaw (มักมากกว่า roll/pitch)' },
    arm_length_m: { label: 'Arm length', unit: 'm', hint: 'ระยะจากกลางลำถึงมอเตอร์' },
    n_motors: { label: 'Motors', unit: '', hint: 'จำนวนมอเตอร์ (hexa = 6)' },
    max_thrust_per_motor_n: { label: 'Max thrust / motor', unit: 'N', hint: 'แรงดันสูงสุดต่อมอเตอร์' },
    prop_torque_coeff: { label: 'Prop torque ratio', unit: 'k_M/k_F', hint: 'อัตราแรงบิด yaw ต่อแรงยก' },
    motor_time_constant_s: { label: 'Motor lag τ', unit: 's', hint: 'หน่วงมอเตอร์+ESC (term D)' },
    rate_bandwidth_hz: { label: 'Rate bandwidth', unit: 'Hz', hint: 'ยิ่งสูงยิ่งตอบสนองไว (เริ่ม ~6)' },
    rate_damping: { label: 'Rate damping ζ', unit: '', hint: 'ความหน่วง (0.7 = นุ่มกำลังดี)' },
    velocity_bandwidth_hz: { label: 'Velocity bandwidth', unit: 'Hz', hint: 'ความไวลูป velocity (หยาบ)' },
    time_scale_factor: { label: 'Cascade separation', unit: '×', hint: 'ลูปนอกช้ากว่าลูปใน X เท่า' },
  };
  const fieldUnit = (k: string) => (META[k]?.unit ? ` (${META[k].unit})` : '');

  const sessionArmed = $derived(mission.commandSession.armed);
  const design = $derived(mission.tunerDesign);
  const sysidResult = $derived(mission.sysidResult);
  const sysidStatus = $derived(mission.sysidStatus);
  const autotune = $derived(mission.autotune);
  const applyResult = $derived(mission.tunerApply);
  const vehicleArmed = $derived(mission.telemetry?.is_armed ?? false);

  let fcParams = $state<Record<string, number | null>>({});
  let applySel = $state<Set<string>>(new Set());
  let busy = $state('');
  let err = $state('');

  // ── wizard step model ──
  const STEPS = [
    { title: 'Arm session', hint: 'กด Arm command session (ปลดล็อกการสั่งโดรน)' },
    { title: 'Vehicle params', hint: 'ตรวจค่าตัวโดรน แล้วกดยืนยัน' },
    { title: 'Sys-ID sweep', hint: 'กด Run — โดรนบินส่าย chirp วัด transfer function' },
    { title: 'Design gains', hint: 'กด Compute model gains หรือใช้ PX4 autotune' },
    { title: 'Apply + save', hint: 'เลือกเกน แล้วกด Apply (ต้อง disarm ก่อน)' },
  ];
  const AXES = ['roll', 'pitch', 'yaw'];

  let step = $state(1);              // 1..5 — the pane currently shown
  let paramsConfirmed = $state(false);

  // Per-step completion → drives the green rail + "what's next".
  const doneArr = $derived([
    sessionArmed,
    paramsConfirmed,
    !!sysidResult,
    !!design || autotune.state === 'success',
    !!applyResult?.ok,
  ]);
  // 1-based index of the first incomplete step; 6 once everything is done.
  const firstIncomplete = $derived((doneArr.findIndex((d) => !d) + 1) || 6);
  const reachable = (n: number) => n <= firstIncomplete && n <= 5;

  // Auto-advance the visible pane forward as each step completes (async signals
  // like sys-ID/autotune/apply arrive over the websocket). Never moves backward,
  // so the operator can still click back to revisit a finished step.
  let lastFI = 1;
  $effect(() => {
    const fi = firstIncomplete;
    if (fi > lastFI) step = Math.min(fi, 5);
    lastFI = fi;
  });

  const sweeping = $derived(['sweeping', 'fitting'].includes(sysidStatus?.state ?? ''));

  // Live per-axis phase: results are pushed only at the end, so mid-sweep we infer
  // earlier axes are done from the fixed roll→pitch→yaw order + the current status.
  function axisPhase(axis: string): 'fit' | 'failed' | 'run' | 'pending' | 'idle' {
    if (sysidResult) {
      const f = sysidResult.fits.find((x) => x.fit.axis === axis);
      if (!f) return 'failed';
      return f.fit.b != null && f.fit.b > 0 ? 'fit' : 'failed';
    }
    const cur = sysidStatus?.axis ?? '';
    if (!cur && !sweeping) return 'idle';
    const ci = AXES.indexOf(cur);
    const ai = AXES.indexOf(axis);
    if (ci < 0) return 'pending';
    if (ai < ci) return 'fit';
    if (ai === ci) {
      const s = sysidStatus?.state ?? '';
      if (s === 'fit') return 'fit';
      if (s === 'failed') return 'failed';
      return 'run';
    }
    return 'pending';
  }
  const liveFitCount = $derived(AXES.filter((a) => axisPhase(a) === 'fit').length);

  function railSub(i: number): string {
    switch (i) {
      case 0: return sessionArmed ? 'armed ✓' : 'not armed';
      case 1: return paramsConfirmed ? 'confirmed ✓' : `${physical.mass_kg}kg · ${physical.n_motors} mot`;
      case 2: return sysidResult ? `${liveFitCount}/3 axes ✓`
        : sweeping ? `${sysidStatus?.axis ?? ''} ${sysidStatus?.state ?? ''}`.trim() : 'not run';
      case 3: return design ? 'model gains ✓'
        : autotune.state === 'success' ? 'autotune ✓'
        : autotune.state !== 'idle' ? autotune.state : 'not designed';
      case 4: return applyResult?.ok ? 'applied ✓' : 'pending';
      default: return '';
    }
  }

  function goto(n: number) { if (reachable(n)) step = n; }

  async function refreshParams() {
    const snap = await tunerParams();
    if (snap) {
      const m: Record<string, number | null> = {};
      for (const p of snap.params) m[p.param] = p.value;
      fcParams = m;
    }
  }
  onMount(refreshParams);

  async function onArm() {
    err = ''; busy = 'arm';
    const r = await cmdArm('tuning session');
    if (!r.ok) err = r.detail || 'arm failed';
    busy = '';
  }

  function confirmParams() { paramsConfirmed = true; step = 3; }

  async function onRunSysid() {
    err = ''; busy = 'sysid';
    const res = await sysidRun(['roll', 'pitch', 'yaw'], 'attitude');
    if (!res.ok) err = res.detail || 'sys-ID start failed';
    busy = '';
  }

  async function onComputeGains() {
    err = ''; busy = 'design';
    const res = await tunerDesign(physical, spec, true);
    if (!res.ok) err = res.detail || 'design failed';
    // Default-select only the MC rate/attitude gains shown in the table (not the
    // coarse MPC velocity/position estimates) — apply matches what's visible.
    else applySel = new Set((mission.tunerDesign?.gains ?? [])
      .filter((g) => g.param.startsWith('MC_')).map((g) => g.param));
    busy = '';
  }

  async function onAutotune() {
    err = ''; busy = 'autotune';
    const res = await autotuneStart();
    if (!res.ok) err = res.detail || 'autotune start failed';
    busy = '';
  }
  const onAbortAutotune = () => autotuneAbort();

  async function onApply() {
    err = ''; busy = 'apply';
    const gains = (design?.gains ?? [])
      .filter((g) => applySel.has(g.param))
      .map((g) => ({ param: g.param, value: g.value }));
    const res = await tunerApply(gains, 'tuner apply');
    if (!res.ok) err = res.detail || 'apply failed';
    await refreshParams();
    busy = '';
  }

  function toggleSel(param: string) {
    const next = new Set(applySel);
    if (next.has(param)) next.delete(param); else next.add(param);
    applySel = next;
  }

  // ── SVG Bode helpers ──
  function poly(f: number[], y: number[], logy: boolean, w: number, h: number, pad: number,
                ylo?: number, yhi?: number): string {
    if (f.length < 2) return '';
    const fx = f.map((v) => Math.log10(Math.max(v, 1e-3)));
    const yy = logy ? y.map((v) => Math.log10(Math.max(v, 1e-9))) : y;
    const xmin = Math.min(...fx), xmax = Math.max(...fx);
    const ymin = ylo ?? Math.min(...yy), ymax = yhi ?? Math.max(...yy);
    const sx = (v: number) => pad + (w - 2 * pad) * (v - xmin) / ((xmax - xmin) || 1);
    const sy = (v: number) => (h - pad) - (h - 2 * pad) * (v - ymin) / ((ymax - ymin) || 1);
    return fx.map((x, i) => `${sx(x).toFixed(1)},${sy(yy[i]).toFixed(1)}`).join(' ');
  }
  const W = 240, H = 110, PAD = 8;
  const fitFor = (axis: string): PlantFitDict | undefined =>
    sysidResult?.fits.find((f) => f.fit.axis === axis)?.fit;
  const frfFor = (axis: string): FrfDict | undefined =>
    sysidResult?.fits.find((f) => f.frf.axis === axis)?.frf;

  const axisLabel = (p: string) =>
    p === 'fit' ? '✓ fit' : p === 'failed' ? '✕ fail' : p === 'run' ? '… running' : p === 'pending' ? 'queued' : '—';
</script>

<div class="aavc-panel h-full flex flex-col overflow-hidden">
  <div class="aavc-panel-header" style="justify-content: space-between;">
    <span>System&nbsp;ID&nbsp;+&nbsp;Autotune</span>
    <span class="text-[10px] normal-case font-normal" style="color: var(--color-aavc-ink-mute);">
      pre-flight tuning aid · not the scored sortie
    </span>
  </div>

  <!-- ============ STEP RAIL (the long row — green when complete) ============ -->
  <div class="aavc-rail">
    {#each STEPS as s, i (s.title)}
      {@const n = i + 1}
      {@const done = doneArr[i]}
      <button class="aavc-railnode" class:done class:active={step === n} class:locked={!reachable(n)}
              disabled={!reachable(n)} onclick={() => goto(n)} title={s.hint}>
        <span class="aavc-railbadge">{done ? '✓' : n}</span>
        <span class="aavc-railtitle">{s.title}</span>
        <span class="aavc-railsub">{railSub(i)}</span>
      </button>
      {#if i < STEPS.length - 1}
        <div class="aavc-railseg" class:done></div>
      {/if}
    {/each}
  </div>

  <!-- what to press next -->
  <div class="aavc-next">
    {#if firstIncomplete > 5}
      <span class="aavc-next-ok">✓ ครบทุกขั้น — เกนถูกบันทึกแล้ว มิชชันจะโหลดไปใช้ให้อัตโนมัติ</span>
    {:else}
      <span class="aavc-next-badge">ขั้นที่ {firstIncomplete}/5</span>
      <span class="aavc-next-hint">{STEPS[firstIncomplete - 1].hint}</span>
    {/if}
  </div>

  {#if err}
    <div class="px-4 py-1.5 text-[11px]" style="color: var(--color-aavc-critical);">{err}</div>
  {/if}

  <!-- ============ ACTIVE STEP PANE ============ -->
  <div class="flex-1 min-h-0 aavc-scroll overflow-y-auto p-3">

    {#if step === 1}
      <!-- ① ARM -->
      <div class="aavc-tuner-card aavc-pane-narrow">
        <div class="aavc-tuner-cardhdr">Step 1 · Arm command session</div>
        <div class="aavc-tuner-desc">
          ทุกคำสั่งที่สั่งโดรน (sweep / autotune / apply) ต้องอาร์ม command session ก่อน — กันสั่งพลาดโดยไม่ตั้งใจ
        </div>
        <div class="flex items-center gap-3 mt-1">
          <button class="aavc-tuner-btn aavc-btn-primary" disabled={sessionArmed || busy === 'arm'} onclick={onArm}>
            {sessionArmed ? '✓ session armed' : busy === 'arm' ? 'arming…' : 'Arm command session'}
          </button>
          <span class="aavc-chip {sessionArmed ? 'aavc-chip-nominal' : 'aavc-chip-critical'}">
            {sessionArmed ? '● armed' : '○ not armed'}
          </span>
        </div>
        {#if sessionArmed}
          <div class="aavc-pane-ok">อาร์มแล้ว — ระบบเลื่อนไปขั้นถัดไปให้อัตโนมัติ ✓</div>
        {/if}
      </div>

    {:else if step === 2}
      <!-- ② VEHICLE PARAMS -->
      <div class="grid gap-3" style="grid-template-columns: 1fr 1fr;">
        <div class="aavc-tuner-card">
          <div class="aavc-tuner-cardhdr">Step 2 · Physical params (hexa)</div>
          <div class="aavc-tuner-desc">ค่าทางกายภาพของโดรน (ใส่ default EFT X6100 ให้แล้ว — แก้ได้) · เลื่อนชี้ ⓘ ดูคำอธิบาย</div>
          <div class="grid grid-cols-2 gap-x-3 gap-y-1.5">
            {#each Object.keys(physical) as k (k)}
              <label class="aavc-tuner-field">
                <span class="aavc-tuner-fieldlabel" title={META[k]?.hint ?? ''}
                      >{META[k]?.label ?? k}{fieldUnit(k)}<span class="aavc-help">ⓘ</span></span>
                <input type="number" step="0.001" bind:value={physical[k as keyof typeof physical]} />
              </label>
            {/each}
          </div>
        </div>
        <div class="aavc-tuner-card">
          <div class="aavc-tuner-cardhdr">Step 2 · Performance spec</div>
          <div class="aavc-tuner-desc">อยากให้ตอบสนองไว/นุ่มแค่ไหน (เริ่มจากค่า default ได้)</div>
          <div class="grid grid-cols-2 gap-x-3 gap-y-1.5">
            {#each Object.keys(spec) as k (k)}
              <label class="aavc-tuner-field">
                <span class="aavc-tuner-fieldlabel" title={META[k]?.hint ?? ''}
                      >{META[k]?.label ?? k}{fieldUnit(k)}<span class="aavc-help">ⓘ</span></span>
                <input type="number" step="0.1" bind:value={spec[k as keyof typeof spec]} />
              </label>
            {/each}
          </div>
        </div>
      </div>
      <div class="flex items-center gap-3 mt-3">
        <button class="aavc-tuner-btn aavc-btn-primary" onclick={confirmParams}>
          {paramsConfirmed ? '✓ params confirmed — แก้แล้วกดซ้ำได้' : 'ยืนยันค่า → ไปขั้น Sys-ID'}
        </button>
        <span class="text-[11px]" style="color: var(--color-aavc-ink-mute);">
          ค่าพวกนี้ใช้ตอน “Compute model gains” (ขั้น 4) — Sys-ID วัด plant จริงไม่ต้องพึ่งค่านี้
        </span>
      </div>

    {:else if step === 3}
      <!-- ③ SYS-ID SWEEP -->
      <div class="aavc-tuner-card">
        <div class="aavc-tuner-cardhdr" style="display:flex; justify-content:space-between;">
          <span>Step 3 · Sys-ID chirp sweep</span>
          {#if sysidStatus}<span style="color: var(--color-aavc-ink-mute);">{sysidStatus.state}: {sysidStatus.axis} {sysidStatus.detail}</span>{/if}
        </div>
        <div class="aavc-tuner-desc">
          โดรนจะบินขึ้นแล้วส่าย chirp ทีละแกน (roll → pitch → yaw) เพื่อวัด transfer function H(jω)=ω/u และหา b/τ ของ plant จริง
        </div>

        <div class="flex items-center gap-3 mb-3">
          <button class="aavc-tuner-btn aavc-btn-primary"
                  disabled={!sessionArmed || busy === 'sysid' || sweeping} onclick={onRunSysid}
                  title="บินส่าย chirp ทีละแกนเพื่อวัด b/τ ของโดรน">
            {sweeping ? 'sweeping…' : busy === 'sysid' ? 'starting…' : sysidResult ? 'Re-run sweep ×3' : 'Run Sys-ID (chirp ×3)'}
          </button>
          {#if !sessionArmed}
            <span class="text-[11px]" style="color: var(--color-aavc-warning);">arm command session ก่อน (ขั้น 1)</span>
          {/if}
        </div>

        <!-- live per-axis tracker -->
        <div class="aavc-axes">
          {#each AXES as ax (ax)}
            {@const p = axisPhase(ax)}
            <div class="aavc-axis aavc-axis-{p}">
              <span class="aavc-axis-name">{ax}</span>
              <span class="aavc-axis-state">{axisLabel(p)}</span>
            </div>
          {/each}
        </div>
        <div class="aavc-axisbar"><div class="aavc-axisbar-fill" style="width: {(liveFitCount / 3) * 100}%;"></div></div>
        <div class="text-[10px] mt-1" style="color: var(--color-aavc-ink-mute); min-height: 13px;">
          {sysidResult ? `identified ${liveFitCount}/3 axes · source: ${sysidResult.calibration.source || '—'}`
            : sysidStatus?.detail || 'ยังไม่ได้รัน — กด Run เพื่อเริ่มวัด plant'}
        </div>

        {#if sysidResult}
          <div class="grid gap-3 mt-3" style="grid-template-columns: repeat(3, 1fr);">
            {#each AXES as axis (axis)}
              {@const fit = fitFor(axis)}
              {@const frf = frfFor(axis)}
              <div>
                <div class="text-[11px] font-semibold mb-1" style="color: var(--color-aavc-ink);">{axis}</div>
                <svg viewBox="0 0 {W} {H}" class="w-full" style="background:#0b1220; border:1px solid var(--color-aavc-border); border-radius:4px;">
                  {#if frf && frf.f_hz.length > 1}
                    <polyline points={poly(frf.f_hz, frf.coherence, false, W, H, PAD, 0, 1)}
                              fill="none" stroke="#334155" stroke-width="1" />
                    <polyline points={poly(frf.f_hz, frf.mag, true, W, H, PAD)}
                              fill="none" stroke="#22d3ee" stroke-width="1.6" />
                  {:else}
                    <text x={W / 2} y={H / 2} fill="#64748b" font-size="9" text-anchor="middle">no FRF</text>
                  {/if}
                </svg>
                {#if fit}
                  <div class="text-[10px] mt-1 font-mono" style="color: var(--color-aavc-ink-dim);">
                    b={fmtNum(fit.b, 1)} · τ={fit.tau_eff_s != null ? fmtNum(fit.tau_eff_s, 3) + 's' : '—'}<br />
                    R²={fmtNum(fit.r2, 2)} · coh={fmtNum(fit.coherence_med, 2)}<br />
                    <span style="color: var(--color-aavc-ink-mute);">{fit.fit_kind}</span>
                  </div>
                {/if}
              </div>
            {/each}
          </div>
          <div class="text-[10px] mt-2" style="color: var(--color-aavc-ink-mute);">
            Bode = แต่ละแกนตอบสนองยังไงเทียบความถี่ · <b>b</b> = gain ของ plant · coherence สูง = น่าเชื่อ ·
            cyan = |H| (log) · grey = coherence · x = log f
          </div>
        {/if}
      </div>

    {:else if step === 4}
      <!-- ④ DESIGN GAINS -->
      <div class="grid gap-3" style="grid-template-columns: 1fr 1fr;">
        <div class="aavc-tuner-card">
          <div class="aavc-tuner-cardhdr">Step 4A · Model-based gains</div>
          <div class="aavc-tuner-desc">
            คำนวณ PID จาก plant ที่วัดได้ (pole-placement) — ทำหลัง Sys-ID เพื่อใช้ค่า b ที่วัดจริง
          </div>
          <button class="aavc-tuner-btn aavc-btn-primary w-full" disabled={busy === 'design'} onclick={onComputeGains}>
            {busy === 'design' ? 'computing…' : 'Compute model gains'}
          </button>
          <div class="text-[10px] mt-2" style="color: var(--color-aavc-ink-mute);">
            {#if design?.calibration_source}
              <span style="color: var(--color-aavc-nominal);">✓ ใช้ค่า b ที่วัดจาก Sys-ID</span>
            {:else if !sysidResult}
              ยังไม่มี Sys-ID — จะคำนวณจากค่ากายภาพแทน (แม่นน้อยกว่า)
            {:else}
              พร้อมคำนวณจากค่าที่วัดได้
            {/if}
          </div>
          {#if design}
            <button class="aavc-tuner-btn w-full mt-2" onclick={() => (step = 5)}>
              ✓ {design.gains.filter((g) => g.param.startsWith('MC_')).length} gains พร้อม → ไป Apply
            </button>
          {/if}
        </div>

        <div class="aavc-tuner-card">
          <div class="aavc-tuner-cardhdr">Step 4B · PX4 built-in autotune</div>
          <div class="aavc-tuner-desc">อีกทาง: ให้ PX4 บินจูนเอง (แทน/เทียบกับ model-based)</div>
          <div class="flex items-center gap-2 mb-2">
            <span class="aavc-chip {autotune.state === 'success' ? 'aavc-chip-nominal'
              : autotune.state === 'failed' || autotune.state === 'aborted' ? 'aavc-chip-critical'
              : 'aavc-chip-armed'}">{autotune.state}</span>
            {#if autotune.axis}<span class="text-[11px]" style="color: var(--color-aavc-ink-dim);">{autotune.axis}</span>{/if}
          </div>
          <div class="h-1.5 rounded-full overflow-hidden mb-2" style="background: var(--color-aavc-panel-2);">
            <div class="h-full" style="width: {autotune.progress_pct}%; background: var(--color-aavc-info); transition: width 0.3s;"></div>
          </div>
          <div class="text-[10px] mb-2" style="color: var(--color-aavc-ink-mute); min-height: 13px;">{autotune.detail}</div>
          <div class="flex gap-2">
            <button class="aavc-tuner-btn flex-1" disabled={!sessionArmed} onclick={onAutotune}
                    title="PX4 บินขึ้น จูนเอง (MC_AT) แล้วบันทึก gain ลง FC">Run autotune</button>
            <button class="aavc-tuner-btn flex-1" onclick={onAbortAutotune} title="หยุด autotune">Abort</button>
          </div>
        </div>
      </div>

    {:else if step === 5}
      <!-- ⑤ APPLY + SAVE -->
      <div class="aavc-tuner-card">
        <div class="aavc-tuner-cardhdr" style="display:flex; justify-content:space-between;">
          <span>Step 5 · Apply gains → FC (model vs flight controller)</span>
          {#if design?.calibration_source}<span style="color: var(--color-aavc-nominal);">using measured b</span>{/if}
        </div>
        <div class="aavc-tuner-desc">ติ๊กเลือก gain ที่จะใช้ (เทียบ model ↔ ค่าใน FC ตอนนี้) แล้วกด Apply · จะ disarm ก่อน</div>
        {#if !design}
          <div class="text-[11px] py-3" style="color: var(--color-aavc-ink-mute);">
            ยังไม่มีเกน — กลับไปขั้น 4 เพื่อ Compute model gains (หรือใช้ PX4 autotune) ก่อน
            <button class="aavc-tuner-btn mt-2" onclick={() => (step = 4)}>← ไปขั้น Design</button>
          </div>
        {:else}
          <table class="w-full text-[11px] font-mono">
            <thead>
              <tr style="color: var(--color-aavc-ink-mute);">
                <th class="text-left font-normal py-1">apply</th>
                <th class="text-left font-normal">param</th>
                <th class="text-right font-normal">FC now</th>
                <th class="text-right font-normal">model</th>
              </tr>
            </thead>
            <tbody>
              {#each design.gains.filter((g) => g.param.startsWith('MC_')) as g (g.param)}
                <tr class="border-t" style="border-color: var(--color-aavc-border);">
                  <td class="py-1"><input type="checkbox" checked={applySel.has(g.param)} onchange={() => toggleSel(g.param)} /></td>
                  <td title={g.formula} style="color: var(--color-aavc-ink-2);">{g.param}</td>
                  <td class="text-right" style="color: var(--color-aavc-ink-mute);">{fmtNum(fcParams[g.param], 3)}</td>
                  <td class="text-right" style="color: var(--color-aavc-info);">{fmtNum(g.value, 4)}</td>
                </tr>
              {/each}
            </tbody>
          </table>
          <button class="aavc-tuner-btn aavc-btn-primary mt-3 w-full"
                  disabled={!sessionArmed || vehicleArmed || applySel.size === 0 || busy === 'apply'}
                  onclick={onApply}
                  title="เขียน gain ที่ติ๊กลง FC + save ไฟล์ → mission รอบหน้าโหลดใช้เอง (ต้อง disarm ก่อน)">
            {vehicleArmed ? 'disarm vehicle to apply'
              : busy === 'apply' ? 'applying…'
              : `Apply ${applySel.size} gains → FC + save`}
          </button>
          {#if applyResult}
            <div class="text-[11px] mt-2" style="color: {applyResult.ok ? 'var(--color-aavc-nominal)' : 'var(--color-aavc-critical)'};">
              {applyResult.ok ? '✓' : '✕'} applied {applyResult.applied.filter((a) => a.ok).length}/{applyResult.applied.length}{applyResult.saved_to ? ' · saved → the flight mission auto-loads these' : ''}
            </div>
          {/if}
          {#if design.warnings.length}
            <ul class="text-[10px] mt-2 space-y-0.5" style="color: var(--color-aavc-warning);">
              {#each design.warnings as w (w)}<li>⚠ {w}</li>{/each}
            </ul>
          {/if}
        {/if}
      </div>
    {/if}
  </div>
</div>

<style>
  /* ── step rail (the long horizontal row) ── */
  .aavc-rail {
    display: flex;
    align-items: flex-start;
    padding: 12px 18px 10px;
    border-bottom: 1px solid var(--color-aavc-border);
    background: color-mix(in srgb, var(--color-aavc-accent) 5%, transparent);
  }
  .aavc-railnode {
    flex: 0 0 auto;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 3px;
    min-width: 96px;
    max-width: 150px;
    padding: 0 4px;
    background: none;
    border: none;
    cursor: pointer;
    text-align: center;
  }
  .aavc-railnode.locked { cursor: not-allowed; opacity: 0.45; }
  .aavc-railbadge {
    width: 30px; height: 30px;
    display: flex; align-items: center; justify-content: center;
    border-radius: 50%;
    border: 2px solid var(--color-aavc-border);
    background: var(--color-aavc-panel-2);
    color: var(--color-aavc-ink-mute);
    font-weight: 800; font-size: 14px;
    transition: all 0.18s;
  }
  .aavc-railtitle {
    font-size: 11px; font-weight: 600;
    color: var(--color-aavc-ink-dim);
    line-height: 1.15;
  }
  .aavc-railsub {
    font-size: 9px;
    font-family: 'IBM Plex Mono', ui-monospace, monospace;
    color: var(--color-aavc-ink-mute);
    line-height: 1.1;
    min-height: 11px;
  }
  /* active step — the one to act on now */
  .aavc-railnode.active .aavc-railbadge {
    border-color: var(--color-aavc-accent);
    color: var(--color-aavc-accent);
    box-shadow: 0 0 0 4px color-mix(in srgb, var(--color-aavc-accent) 22%, transparent);
  }
  .aavc-railnode.active .aavc-railtitle { color: var(--color-aavc-ink); }
  /* completed step — clearly GREEN */
  .aavc-railnode.done .aavc-railbadge {
    border-color: var(--color-aavc-nominal);
    background: var(--color-aavc-nominal);
    color: #04140a;
    box-shadow: 0 0 10px color-mix(in srgb, var(--color-aavc-nominal) 55%, transparent);
  }
  .aavc-railnode.done .aavc-railtitle { color: var(--color-aavc-nominal); }
  .aavc-railnode.done .aavc-railsub { color: color-mix(in srgb, var(--color-aavc-nominal) 75%, var(--color-aavc-ink-mute)); }
  /* connector between badges */
  .aavc-railseg {
    flex: 1 1 auto;
    height: 3px;
    margin: 14px 4px 0;
    border-radius: 2px;
    background: var(--color-aavc-border);
    transition: background 0.25s;
  }
  .aavc-railseg.done { background: var(--color-aavc-nominal); }

  /* ── "what's next" banner ── */
  .aavc-next {
    display: flex; align-items: center; gap: 10px;
    padding: 7px 18px;
    font-size: 12px;
    border-bottom: 1px solid var(--color-aavc-border);
  }
  .aavc-next-badge {
    font-weight: 700; font-size: 10px; letter-spacing: 0.06em;
    padding: 2px 8px; border-radius: 999px;
    background: color-mix(in srgb, var(--color-aavc-accent) 18%, transparent);
    color: var(--color-aavc-accent);
    border: 1px solid color-mix(in srgb, var(--color-aavc-accent) 45%, transparent);
    white-space: nowrap;
  }
  .aavc-next-hint { color: var(--color-aavc-ink-dim); }
  .aavc-next-ok { color: var(--color-aavc-nominal); font-weight: 600; }

  /* ── per-axis Sys-ID tracker ── */
  .aavc-axes { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
  .aavc-axis {
    display: flex; flex-direction: column; align-items: center; gap: 2px;
    padding: 7px 4px; border-radius: 5px;
    border: 1px solid var(--color-aavc-border);
    background: var(--color-aavc-body);
  }
  .aavc-axis-name { font-size: 12px; font-weight: 700; text-transform: capitalize; color: var(--color-aavc-ink); }
  .aavc-axis-state { font-size: 10px; font-family: 'IBM Plex Mono', monospace; color: var(--color-aavc-ink-mute); }
  .aavc-axis-fit { border-color: var(--color-aavc-nominal); background: color-mix(in srgb, var(--color-aavc-nominal) 12%, transparent); }
  .aavc-axis-fit .aavc-axis-state { color: var(--color-aavc-nominal); }
  .aavc-axis-run { border-color: var(--color-aavc-accent); background: color-mix(in srgb, var(--color-aavc-accent) 12%, transparent); }
  .aavc-axis-run .aavc-axis-state { color: var(--color-aavc-accent); }
  .aavc-axis-failed { border-color: var(--color-aavc-critical); }
  .aavc-axis-failed .aavc-axis-state { color: var(--color-aavc-critical); }
  .aavc-axisbar { height: 5px; border-radius: 3px; overflow: hidden; background: var(--color-aavc-panel-2); margin-top: 8px; }
  .aavc-axisbar-fill { height: 100%; background: var(--color-aavc-nominal); transition: width 0.3s; }

  /* ── cards / fields / buttons ── */
  .aavc-pane-narrow { max-width: 560px; }
  .aavc-pane-ok { margin-top: 8px; font-size: 11px; color: var(--color-aavc-nominal); }
  .aavc-tuner-card {
    background: var(--color-aavc-panel-2);
    border: 1px solid var(--color-aavc-border);
    border-radius: 6px;
    padding: 10px 12px;
  }
  .aavc-tuner-cardhdr {
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    color: var(--color-aavc-ink-mute);
    margin-bottom: 8px;
  }
  .aavc-tuner-field { display: flex; flex-direction: column; gap: 2px; }
  .aavc-tuner-field input {
    background: var(--color-aavc-body);
    border: 1px solid var(--color-aavc-border);
    border-radius: 3px;
    padding: 3px 6px;
    font-size: 12px;
    font-family: 'IBM Plex Mono', monospace;
    color: var(--color-aavc-ink);
    width: 100%;
  }
  .aavc-tuner-btn {
    padding: 7px 12px;
    font-size: 12px;
    font-weight: 600;
    border-radius: 5px;
    border: 1px solid var(--color-aavc-border);
    background: var(--color-aavc-panel);
    color: var(--color-aavc-ink);
    cursor: pointer;
    transition: filter 0.12s;
  }
  .aavc-tuner-btn:hover:not(:disabled) { filter: brightness(1.12); }
  .aavc-tuner-btn:disabled { opacity: 0.4; cursor: not-allowed; }
  .aavc-btn-primary {
    background: var(--color-aavc-accent);
    border-color: var(--color-aavc-accent);
    color: #04140a;
  }

  .aavc-tuner-desc {
    font-size: 10px;
    color: var(--color-aavc-ink-mute);
    margin: -3px 0 8px;
    line-height: 1.4;
  }
  .aavc-tuner-field .aavc-tuner-fieldlabel {
    font-size: 10px;
    font-family: 'IBM Plex Sans', sans-serif;
    color: var(--color-aavc-ink-dim);
    cursor: help;
  }
  .aavc-tuner-field .aavc-help {
    font-size: 9px;
    color: var(--color-aavc-ink-mute);
    opacity: 0.55;
    margin-left: 2px;
  }
</style>
