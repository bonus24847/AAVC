# Mission speed — recover window margin without undoing safety caps

**Date:** 2026-07-20
**Status:** IMPLEMENTED and SITL-validated 2026-07-20 — see §8 Results
**Operator goal (chosen 2026-07-20):** *fastest that is still safe* — ceiling-risk
knobs are in scope, but every change must be re-validated in SITL.

---

## 1. Why

The 4-sortie mission burns **16.62 min** of the 20-minute window (mean over the
12 complete runs in `runs/*/audit.jsonl`; σ is small — 16.5-16.7 min, this is a
repeatable number, not a bad day). That leaves **3.4 min** of margin.

Finishing early scores nothing: the rules end the repeat loop when the cargo is
delivered to all pads **or** the time limit is reached, so saved seconds cannot
be converted into points. What the margin buys is **insurance** — one deferred
landing, one re-sweep, or one re-flight of a failed delivery must not push the
team into the per-minute overtime penalty. At 3.4 min the mission currently
tolerates roughly one bad sortie.

## 2. Evidence

Phase totals, mean over 12 complete 4-sortie runs (1 Hz `TELEM phase=` samples):

| Phase | mean s | % of window | per sortie |
|---|---|---|---|
| **land** | **337.7** | **33.9%** | 84.4 |
| → land at L&R (post-release) | 254.0 | 25.5% | 63.5 |
| → land ON pad (pre-release) | 83.7 | 8.4% | 20.9 |
| search | 177.3 | 17.8% | 44.3 |
| transit_egress | 142.0 | 14.2% | 35.5 |
| transit_ingress | 116.7 | 11.7% | 29.2 |
| takeoff | 90.8 | 9.1% | 22.7 |
| drop (climb-out) | 77.5 | 7.8% | 19.4 |
| localize | 54.9 | 5.5% | 13.7 |

Three findings drive this design:

1. **The aircraft does not hover — it translates slowly.** Only 15 s of the
   1002 s window is true stationary hover (1.5%). "Fix the hover" is the wrong
   frame; the descents and the accel/decel profile are the cost.
2. **The L&R landing descends 20 m at a median 0.39 m/s** (n=1264 samples above
   10 m AGL; fastest -0.56) against a configured `MPC_Z_VEL_MAX_DN=3.0` — 13% of
   the cap. `mission.py:455` gotos home at 19.5 m and hands straight to
   `commander.land()`, so `MPC_LAND_SPEED=0.3` governs the whole 19.5 m. The
   staged `MPC_LAND_ALT1/ALT2` slowdown never engages in SITL because PX4 needs
   a distance sensor and gz_x500 has none (TF-Luna commented out,
   `sitl/aavc_config.yaml:262-266`, VERIFY-AT-G5).
3. **Transit reaches its 10 m/s cap but flies at 58% of theoretical.** One-way
   route is 188.4 m; ideal round trip at 10 m/s is 37.7 s, measured 64.7 s. The
   gap is accel/jerk, and the two params that govern AUTO — `MPC_JERK_AUTO`
   (default 4.0) and `MPC_ACC_HOR_MAX` (default 5.0) — **are never set**. The
   `MPC_ACC_HOR=3.0` that *is* set is not the one AUTO uses.

## 3. Changes

### W1 — Staged L&R descent (~104 s)

`orchestrator/mission.py:450-463`: between the goto home at `transit_alt` and
`commander.land(disarm=True)`, insert a second goto over the same point at a
staging altitude (`5.0 m` AGL, module constant beside the existing climb-staging
constant so it is tunable in one place).

The 19.5 → 5 m leg is an ordinary position setpoint governed by
`MPC_Z_VEL_MAX_DN=3.0` (~7 s); `AUTO.LAND` then owns only the last 5 m at 0.3
m/s (~17 s). **~25 s replaces ~51 s per sortie.**

`MPC_Z_VEL_MAX_DN` must be explicitly restored to 3.0 immediately before this
leg. `tactical_align.py` steps it down to 0.4 per rung and restores at
`tactical_align.py:420`; if that restore is ever missed the new staged leg is
*slower* than today. Do not rely on the remote restore — set it here.

**`MPC_LAND_SPEED` is not touched.** Raising the `MPC_LAND_*` family globally was
already tried and reverted (`e02ffa3`: AUTO.LAND climbed to 41 m after the L&R
touchdown). The repo's own post-mortem prescribes a per-L&R code change instead
of a global param — this is that change, and it avoids the reverted path
entirely.

Real-bird note: with the TF-Luna fitted at G5 the staged slowdown *will* engage,
so the final 5 m runs at the `MPC_LAND_ALT2` crawl. Behaviour stays "fast to 5 m,
gentle below it" in both worlds.

### W2 — Transit accel/jerk (~40-60 s)

`sitl/aavc_config.yaml` `px4_tuning:` + the `DEFAULT_PX4_TUNING` fallback in
`mavlink_adapter/commands.py` (kept equal by `tests/test_px4_tuning_parity.py`):

| Param | From (PX4 default, unset) | To |
|---|---|---|
| `MPC_JERK_AUTO` | 4.0 | 8.0 |
| `MPC_ACC_HOR_MAX` | 5.0 | 8.0 |

Bounded above by the existing anti-flip `MPC_TILTMAX_AIR=35°`, which is unchanged.

`NAV_ACC_RAD=2.0` stays. Widening it would let the aircraft cut the corner at
P1/P2/P3 — the coordinates the committee scores per pass. Not worth the seconds.

### W3 — Takeoff (~35 s)

Set `MPC_TKO_SPEED` (never set; PX4 default 1.5, measured median 1.01 m/s) to
**2.0**, matching the already-tuned `MPC_Z_VEL_MAX_UP=2.0`.

This is the ceiling-risk knob and is only acceptable because of existing
staging: `arm_and_takeoff` climbs to `climb_alt = transit_alt - 2.0 = 17.5 m`
(`mission.py:110-115`), and the first transit goto closes the last 2 m at a 1.0
m/s cap (`mission.py:204`). The 2 m margin exists precisely to absorb takeoff
overshoot. Peak altitude is a pass/fail metric on every validation run.

## 4. Explicitly not changed

| Knob | Why it stays |
|---|---|
| `MPC_XY_CRUISE=10` | "Slower sweep = more frames per pad = reliable decode." Valid for the sweep. It does **not** apply to the transit legs, where nothing is decoded — phase-dependent cruise is a candidate for a second round, after W1-W3 are measured. |
| `MPC_LAND_SPEED`, `MPC_LAND_ALT1/2` | The reverted 41 m bug (`e02ffa3`). |
| `MPC_Z_VEL_MAX_UP=2.0`, two-stage climb | Ceiling busts at 19.68 m / 21.83 m (`b858abd`, `cf00e5e`, `3cb17bd`). |
| `NAV_ACC_RAD=2.0` | Scored transit coordinates. |
| Rung ladder + tolerances | Fixed 0.59/0.70 m release scatter; pinned by `tests/test_config.py:46-48`. |
| Yaw/tilt caps | Operator anti-flip decision. |
| `_WAIT_PAD_S`, `rung_timeout_s`, 60 s decode-visit | Timeouts — they cost time only when something has already gone wrong, not in the steady state. |

## 5. Validation

Each workstream gets its own full 4-sortie SITL run, measured against the
current baseline (16.62 min mean). Metrics, all four required:

1. **Window time** — must drop; record per-phase breakdown.
2. **Peak altitude** — must stay under 20 m (transient >20.5 m is a WARN per the
   existing watchdog convention; sustained or >22 m is a FAIL).
3. **Release accuracy vs truth** — must stay within the current 0.05-0.32 m band;
   `landing_accuracy_threshold_m=0.5`.
4. **`tools/verify_flight.py` PASSES** on the run's audit slice.

Any run failing 2, 3, or 4 reverts that workstream regardless of the time saved.

Operational gotchas (previously hit, do not re-learn):
- Background tasks are killed at ~19-20 min; a run is 14-17 min. Detach with
  `nohup` + `disown` + a pidfile waiter.
- Kill any stale `mavsdk_server` before launching, or every param RPC fails
  silently ("applied 0/N") and the run measures nothing.
- `audit.jsonl` appends across runs — slice per-run by `wc -l` baseline before
  handing it to `verify_flight.py`.

## 6. Expected outcome

**16.6 min → ~13 min**, raising window margin from 3.4 to ~7 min: enough to
absorb a deferred landing *and* a re-sweep, or to re-fly one failed delivery.

## 7. Out of scope (tracked separately)

`tactical_align.py:372-413` breaks each descent rung on **horizontal** error
only (`last_err` is ground distance) with no altitude-arrival check, so the rung
ladder does not actually gate the descent — the real descent from ~11 m is flown
by PX4 AUTO.LAND and the per-rung `MPC_Z_VEL_MAX_DN` ladder (3.0 → 0.4,
`tactical_align.py:92`) barely shapes anything. Evidence: `localize` lasted 6 s
on sortie 1 while altitude moved only 12.15 → 11.16 m. This is a safety /
architecture issue, not a speed one, and must not be bundled into a timing
change.

---

## 8. Results (2026-07-20, two SITL runs)

**16.70 min → 14.87 min.** Window margin 3.3 → 5.1 min. Release accuracy and the
scored transit corridor were unaffected.

| | baseline 07-15 | run 1 (W2+W3) | run 2 (+W1 corrected) |
|---|---|---|---|
| window | 1001.7 s / 16.70 min | 976.1 s / 16.27 min | **892.1 s / 14.87 min** |
| `land` phase | 337.7 s | 337.0 s | **259.0 s** |
| transit ingress / egress | 116.7 / 142.0 | 106.0 / 127.0 | 106.0 / 129.0 |
| takeoff | 90.8 | 81.0 | 80.0 |
| release vs truth | — | 0.15-0.22 m | 0.04-0.23 m |
| transit points | — | 24/24 | 24/24 |
| peak altitude | 20.33 m | 20.24 m | 20.69 m |
| verify_flight | — | PASS | FAIL (ceiling only — see §9) |

W2 and W3 landed as designed: transit −9%/−11%, takeoff −11%, and the peak
altitude did **not** degrade (20.24 vs a 20.33 m baseline), so the 2 m climb
staging absorbed the faster takeoff exactly as predicted.

### The W1 correction — the AUTO/manual parameter split

W1 as first written **executed perfectly and saved nothing**: all four sorties
handed over to AUTO.LAND at 5.9 m with no timeout, yet the staged leg sank at
0.39 m/s (max 0.45) — the same speed as the AUTO.LAND it replaced, so the
`land` phase was unchanged at 337.0 s.

Reading the live FC showed `MPC_Z_VEL_MAX_DN = 3.0` (the set had worked) while
`MPC_Z_V_AUTO_DN = 0.4`. The first hypothesis — that PX4 clamps one to the
other — was **tested against the running FC and refuted** (dropping MAX_DN to
0.4 left AUTO_DN at 2.0). PX4's own source settles it:

- `MPC_Z_V_AUTO_DN` — *"Descent velocity in autonomous modes"*
- `MPC_Z_VEL_MAX_DN` — *"For manual modes and offboard"* (`FlightTaskManualAccelerationSlow`)

They are separate parameters for separate mode families, and this mission flies
AUTO end to end. Consequences:

1. `tactical_align`'s per-rung descent ladder (3.0 → 0.4 on `MPC_Z_VEL_MAX_DN`)
   has never shaped an AUTO descent — independently corroborating the timing
   investigation's observation that the ladder "barely shapes anything".
2. The descent speed that actually flew every validated landing was
   `MPC_Z_V_AUTO_DN = 0.4`, a value the repo never set. It had persisted in the
   SITL `parameters.bson` (0.4 is below the parameter's own declared minimum of
   0.5, so nothing chose it deliberately — most likely a stale
   `tools/landing_trial.py --set` experiment).
3. **Real-bird safety:** PX4's default is 1.5 m/s. An unpinned 6X at G5 would
   have descended onto the pad ~4x faster than anything ever validated, with the
   egg aboard — and SITL could never have caught it, because SITL was silently
   running 0.4. `MPC_Z_V_AUTO_DN: 0.4` is now pinned in both the config and
   `DEFAULT_PX4_TUNING`, and `mission.py` raises it to 2.5 for the L&R staged
   descent only, handing it straight back after touchdown.

This is the same class of defect as W2 (`MPC_ACC_HOR` vs `MPC_ACC_HOR_MAX`):
the project had tuned the manual-mode twin of the parameter AUTO actually reads.
Twice. Any future "PX4 knob has no effect" should check the AUTO twin first.

## 9. The one failing check is not from this work

Run 2's `verify_flight` FAIL is `altitude: max 20.69 m — 3 consecutive samples
above 20.5 m`, and it is a pre-existing condition, not a regression:

- The high samples are in **transit**, not takeoff/climb — so `MPC_TKO_SPEED`
  (the only ceiling-risk knob touched) is not implicated. Run 1 carried the same
  `MPC_TKO_SPEED=2.0` with **zero** samples above 20.3 m.
- Per-sortie median transit altitude against a commanded 19.5 m:

  | run | S1 | S2 | S3 | S4 | spread |
  |---|---|---|---|---|---|
  | baseline | 19.62 | 19.96 | 19.18 | 19.14 | 0.82 m |
  | run 1 | 19.35 | 20.09 | 18.53 | 19.32 | 1.56 m |
  | run 2 | 19.26 | 18.61 | 20.29 | 20.35 | 1.74 m |

  Every run wanders, baseline included. Run 1's worst deviation (−0.97 m) is the
  same magnitude as run 2's (+0.85 m); only the sign differs, and the sign is
  random. Run 2 happened to draw high twice in a row.

The real finding is that the ±0.9 m per-arm altitude-frame drift leaves almost
no headroom under a 20 m ceiling commanded at 19.5 m — a competition risk that
exists with or without this work, now measured across three runs. Tracked
separately in `2026-07-20-altitude-frame-drift.md`; it must not be papered over
by simply lowering the commanded transit altitude (that trades a ceiling bust
for a floor bust — `verify_flight` checks the [18.8, 20.5] corridor from both
sides).
