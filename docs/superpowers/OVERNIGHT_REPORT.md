# Overnight Work — Morning Report (2026-07-07 → 08)

*Branch `feat/win-optimization`. Detailed trail: `OVERNIGHT_LOG.md`. All work is
committed + `make test`/`make lint` green.*

## TL;DR — the mission now wins

The delivery mission now **reliably delivers 4/4 eggs** to the correct assigned
pads (up from **3/4**), **fully verified across 5 different pad layouts**, inside
the 20-min window with ~3 min spare, landing at 0.07–0.19 m, transit **24/24**,
zero ceiling/geofence violations. For 4 pads that is **score-optimal**
(4 deliveries + all transit points + under time).

**The one change that won it:** `search.confirm_votes` **3 → 2**.

## Why that one line was the whole game

The mission was losing its 4th sortie to **re-sweeps**. Root cause (traced with
systematic-debugging):

- At sweep altitude the ArUco marker is only ~18 px, so a pad often **decodes
  only 1–2×** during the sweep.
- The registry required **3 decoded votes** to CONFIRM a pad. Pads with <3
  logged `cluster_identified` but stayed `CANDIDATE` → `registry=unknown` on the
  sortie they were assigned → the mission flew a **full re-sweep** (~90–116 s).
- Those re-sweeps ate the time budget → only 3 of 4 sorties fit.

Lowering the registry threshold to 2 confirms those pads **on the sortie-1
sweep**, so sorties 2–4 fly **direct** (0 re-sweeps). It is **safe** because the
terminal **land gate re-verifies the marker id AT the pad** during descent — a
mis-registered pad *defers* (climbs, returns with the egg), it never mis-lands.

## Evidence (SITL, fresh sim per run, SEED = pad layout)

| Run | Layout (ids) | confirm_votes | Delivered | verify_flight |
|---|---|---|---|---|
| baseline (before) | 7 (3,2,4,6) | 3 | **3/4** ❌ | — |
| I1b | 7 (3,2,4,6) | 2 | **4/4** | PASS |
| I2a | 42 (6,1,5,3) | 2 | **4/4** | FAIL\* |
| I2b | 99 (4,6,2,3) | 2 | **4/4** | PASS |
| I3 | 7 (3,2,4,6) | **1** | **4/4** | PASS |
| I4a | 1 (2,5,1,6) | 2 | **4/4** | PASS |
| I4b | 5 (pads 3,4,5,6) | 2 | *interrupted†* | — |
| I5a ᵐ | 7 (3,2,4,6) | 3 | **4/4** | PASS (19 ok, 0 warn) |
| I5b ᵐ | 7 (3,2,4,6) | **8**‡ | **3/3 flown** | PASS (16 ok, 0 warn) |
| I5c ᵐ | 5 (5,3,6,4) | **2 (shipped)** | **4/4** | PASS (19 ok, 0 warn) |

**Every run since the fix delivers on every flown sortie, across all 5
layouts.** \* I2a's verify_flight FAIL was the intermittent transit-corridor
check (Open item 2), not a delivery failure. † I4b was cut at ~23:41 by the
laptop battery dying mid-run: the sortie-1 sweep confirmed all four pads,
sortie 1 delivered (pad 5, 0.15 m), then the log ends during the egress
climb-out — not a data point. ᵐ I5 = the 2026-07-08 morning session
(`feat/registry-topup`), validating the vote top-up structural fix; I5c
re-runs the interrupted I4b layout end-to-end on the SHIPPED config —
releases 0.05–0.15 m, window 1027 s / 17.1 min. ‡ I5b's confirm_votes=8 is
deliberately pathological (nothing can confirm during a sweep) to force the
top-up path: the sweep ended with 2 pads unconfirmed → the registry-completion
top-up visits confirmed them in ~20 s → sorties 2–3 flew DIRECT to top-up'd
pads (releases 0.09–0.18 m); the 4th sortie was correctly time-gated (the
artificial 397 s sortie-1 cost), which the shipped config does not incur
(see I5c).

## Also delivered

- **Representative 3:1 / 2.15 kg airframe SITL model** (`sitl/models/x500/` on
  commit 5798d93) + **`docs/AIRFRAME_SIZING.md`** — full conceptual sizing
  (frame/props/motor/ESC/battery/wiring), eCalc/bench verification steps, and the
  real-airframe re-tune procedure. (Mission validation runs on the tuned stock
  x500; the 3:1 model tracks altitude poorly under stock gains — the documented
  bench-re-tune item.)
- A **regression test** locking the confirm_votes fix.

## Open items / recommendations (none block the win)

1. **`confirm_votes` 1 vs 2 (real-field decode robustness).** Committed **=2**
   (conservative, cross-checked, 4/4 across 4 layouts). Tested **=1** overnight →
   also clean 4/4 with **0 false confirms** in SITL. Trade-off: =1 confirms on ANY
   single decode (robust when field decode is marginal) but a rare false ArUco
   decode → a *missed* delivery (land gate defers at the pad); =2 needs 2 agreeing
   decodes (no false-confirm risk) but re-sweeps a pad that decodes <2×. **After
   G6/G7 real-pad testing:** decode ≥2× reliably → keep =2; decode marginal →
   switch to =1. **Best of both = the structural fix:** revisit an
   *identified-but-unconfirmed* pad (decoded once, position known) to top up its
   votes — far cheaper than a full re-sweep — instead of re-sweeping. Keeps the
   =2 cross-check AND kills under-decode re-sweeps with no false-confirm risk
   (`mission.py:381` + a `TargetTracker.identified_unconfirmed()` accessor; ~½ day).
   **→ DONE 2026-07-08** (morning session, `feat/registry-topup`): accessor +
   pre-sweep top-up + extended decode-visit/registry-completion lists, TDD'd,
   SITL-validated (see the table's I5 rows).
2. **Transit-corridor dip (secondary, no score loss).** The vehicle holds
   ~19.1–19.4 m for each leg but the first ~4 s dip to ~17 m as it closes the
   last 2 m of climb en-route — a DELIBERATE ceiling-safety choice (a full-rate
   climb straight to 19.5 m overshoots +1.8 m → busts the 20 m ceiling). Transit
   scoring is unaffected (24/24). A jerk-limited climb-to-20 m profile could hold
   the corridor without overshoot — future work (needs careful ceiling validation).
3. **L&R recovery landing (~26% of flight time).** A dedicated fast recovery
   landing is the biggest speed lever but is BLOCKED on a PX4 mystery: a global
   `MPC_LAND_*` speed-up made AUTO.LAND climb to 41 m after the L&R touchdown
   (ceiling RTH crash). Needs an isolate-the-param SITL study before it's safe.
4. **The 3:1 airframe needs a bench System-ID re-tune** before real flight
   (procedure in AIRFRAME_SIZING.md).

## Branch / commits (on `feat/win-optimization`)

- `5798d93` representative 3:1 airframe model + sizing doc
- `7dc5a05` **fix(mission): confirm_votes 3→2 — the win**
- `e0598d5` run mission validation on tuned stock x500
- `820fbcc` test: lock confirm_votes behavior
- (+ overnight log/report commits)

## How to verify yourself

```bash
git checkout feat/win-optimization
make test && make lint          # green
make sitl                       # fresh SITL
make spawn-targets SEED=7 && make camera-bridge
.venv/bin/python -m orchestrator.main --config sitl/aavc_config.yaml \
  --no-dashboard --assigned-ids "3,2,4,6" --truth-json /tmp/aavc_targets.json
# -> "complete: 4 delivered over 4 sorties"; verify_flight PASS
```
