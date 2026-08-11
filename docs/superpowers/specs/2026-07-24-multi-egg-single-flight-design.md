# Multi-egg single flight: FLIGHT ⊃ DELIVERY

**Date:** 2026-07-24
**Status:** DESIGN — approved, not yet implemented
**Supersedes:** the "one egg per sortie, ≤4 sorties" locked decision
(CLAUDE.md §2), on the authority of the event briefing (below)

---

## Why

The operator attended an event briefing that changed three facts the whole
mission model was built on. The briefing overrides the PDF — every page of
`AAVC2026_RulesAndRegulation_V1.3_140769.pdf` carries a "subject to be changed"
watermark, and the digest already warns that figure-derived values are
provisional.

| | PDF V1.3 | Briefing (authoritative) |
|---|---|---|
| Pads on the field | "Up to four (4) landing pads will be placed across the search area" (p. 5) | **6 pads** — one per marker id 1–6 |
| Assignment | one payload–pad pair per flight, at resupply | **4 of the 6**, queued up front |
| Cargo | one no.0 egg per flight, resupply between flights | **carry all 4**, one flight |
| Scoring | "Each **flight sortie** will be scored using all the criteria above" (p. 12) | **per delivery** |

The PDF already permits multi-cargo hardware — *"If the air vehicle can carry
multiple cargo, each must have its own release mechanism for independent cargo
release control. There is no restriction regarding the number of payload modules
installed on a single aerial vehicle as long as its MTOW does not exceed the
specified limit"* (p. 8) — and forbids only *"to release multiple cargo at the
same time"* (p. 13), which our land-then-release sequence satisfies by
construction.

**With per-delivery scoring, one flight strictly dominates four.** Same points,
and the flight-time model says roughly a third less air time: today's four
sorties cost `sortie_cost_s 350 + 3 × known_sortie_cost_s 210 ≈ 980 s`, while a
single flight pays takeoff / ingress / egress / L&R landing once instead of four
times (≈ 625 s). That also removes the mid-mission battery swap the current
model needs — `orchestrator/energy_policy.py:6-8` records that a 7,500 mAh 6S
pack yields 5,625 mAh usable while the four-sortie mission wants ~6,800 mAh.

## What is NOT changing

The flight-critical machinery is untouched. This is a change to how deliveries
are **grouped into flights**, not to how a delivery is flown.

- id-verified LAND gate (`require_id_votes`) — wrong-id pads still never steer a
  descent, and the aircraft still climbs-and-defers rather than land unverified
- touchdown-gated release — an airborne release is still SKIPPED and audited
- `gps_fallback=False`
- finish-sweep-then-serve, the cross-sortie pad registry, vote top-up visits
- transit corridor at 20 m, ceiling / no-fly / search-floor watchdogs
- explicit goto-home + LAND (never RTL); `COM_DISARM_LAND=-1`
- classical CV only; deterministic; offline

---

## Design

### 1. Vocabulary: FLIGHT ⊃ DELIVERY

"Sortie" currently means two things at once — *one arm→disarm cycle* and *one
delivery*. The briefing splits them, so the design splits the word.

| Term | Definition | Owns |
|---|---|---|
| **FLIGHT** | takeoff at L&R → transit in → work → transit out → land + disarm at L&R | GO gate, transit audit, L&R landing check, battery-swap detection |
| **DELIVERY** | one pad: align → land ON → release one egg | scoring, idempotence ledger, truth audit |

### 2. `eggs_aboard` is the only knob

`eggs_aboard: N` chunks `state.assigned_id_queue` into flights of ≤ N
deliveries:

```
eggs_aboard: 1  →  [[3],[1],[4],[6]]   today's behaviour, exactly
eggs_aboard: 4  →  [[3,1,4,6]]         the briefing model
eggs_aboard: 2  →  [[3,1],[4,6]]       falls out free
```

**This is an interface, not a mode switch.** There is ONE code path: the mission
loop is `for flight: for delivery in flight:`, and `eggs_aboard=1` collapses the
inner loop to a single iteration. No `if multi_egg:` branch exists, so there is
no untested path to rot, and rollback if the committee reverses is a one-integer
config edit rather than a revert.

`eggs_aboard: 2` matters beyond rollback: it is the fallback if the G5 bench
finds four boxes too heavy or too bulky (§8).

### 3. Two indices, two jobs

| Index | Scope | Purpose |
|---|---|---|
| `payload_id` | 0..N-1 **within its flight** | servo channel = `drop_servo_channel + payload_id` (AUX 9/10/11/12). Resets each flight — every flight is reloaded from empty. |
| `stop_index` | 0-based across the **whole mission** | keys the idempotence ledger and the plan pointer. Globally unique, so a release fires exactly once. |

`mission_brain/schemas.py:116-119` already carries `stop_index` per
`DROP_PAYLOAD` "so it fires exactly once (vs the legacy single-payload …)" — the
mechanism survives from the retired 5-payload design and needs no change.

### 4. Mission loop

```python
flights = chunk(state.assigned_id_queue, eggs_aboard)   # eggs_aboard=1 → today
for f, flight_ids in enumerate(flights, 1):
    await flight_gate(f, flight_ids)      # ONE GO per flight: "N eggs loaded, crew clear"
    state.start_window()                  # first GO starts the 20-min clock
    TAKEOFF → transit ingress

    # Find before serving — extends finish-sweep-then-serve to the whole flight.
    if any(id not in registry for id in flight_ids):
        top-up visits → full sweep → decode visits

    for d, assigned in enumerate(flight_ids):
        if not _delivery_budget_ok():     # §5 — NEW
            break
        await _serve(stop_index=queue_pos(assigned), assigned=assigned, payload_id=d)

    climb out → transit egress → goto L&R → LAND → DISARM
```

`stop_index` is the id's position in the **full mission queue**, not a running
counter over successful serves — so it stays unique and stable whether or not a
delivery is skipped, and the ledger entry for a pad is the same value no matter
what happened to the deliveries before it.

The flight gate's GO body keeps its shape; `payload_confirmed` just means "all N
eggs loaded" instead of "the egg is loaded".

`_fly_transit`, `_sweep_for`, `_serve` and `_decode_visits` are reused verbatim.
The work is the outer structure and the indices handed in.

**Serve order is queue order** — the sequence the operator set in the GCS queue
editor. Geographic (nearest-first) ordering would save flight time and cannot
cost points under per-delivery scoring, but the operator asked for queue order
and it is what the committee can most easily follow on the radio. Revisit only
if G4′ shows the window is tight.

**A pad that is not found is skipped, not fatal.** Serve the ids the sweep
confirmed, keep the undelivered eggs, continue to the next id, and land with
whatever is left aboard. `DELIVERY k END delivered=False reason=not_found`.

A skipped id is **not retried later in the same flight**, even if a subsequent
delivery's approach happens to decode it. Retrying would mean flying back across
the search area on a pad the sweep and its decode visits already failed to
confirm, at the cost of the deliveries still ahead. The id stays unserved and is
reported as such; if the window allows, the operator can queue it into a
follow-up flight.

### 5. NEW: per-delivery abort gate ⚠

This is the one genuinely new safety requirement, and the reason the change is
not purely structural.

Today, flight = delivery, so every go/no-go decision happens on the ground at a
GO gate. With four deliveries inside one flight **there is no gate between them**
— if time or charge runs out during delivery 3, the only thing left holding the
aircraft is the FC's own low-battery failsafe. That is not good enough when the
single-flight energy margin is thin.

Before each delivery:

```python
reserve = egress_transit_cost + lr_landing_cost + margin
ok = (time_left   > reserve + serve_cost) and (energy_left > reserve + serve_cost)
```

On failure: skip the remaining deliveries, climb out, fly the egress transit,
land at L&R with the undelivered eggs. Audited, not silent. This runs
automatically — there is no operator to ask mid-air, which is exactly why the
existing GO-gate checks cannot cover it.

`orchestrator/time_policy.py` and `energy_policy.py` gain per-flight cost models
(a flight now costs takeoff + ingress + sweep + N × serve + egress + landing) and
expose this mid-flight predicate.

### 6. Audit format — a deliberate break

CLAUDE.md §5 says to keep the audit formats stable. This change breaks them on
purpose: `sortie=` no longer identifies anything, because one arm→disarm cycle
now contains several deliveries.

```
FLIGHT 1 START eggs=4 ids=3,1,4,6 remaining=1200s
TRANSIT_PASS P1 ingress flight=1 d=2.4m
DELIVERY 1 START pad=3 payload=0 stop_index=0
DELIVERY 1 RELEASE pad=3 payload=0 lat=… lon=…
DELIVERY 1 END delivered=True
DELIVERY 2 START pad=1 payload=1 stop_index=1
…
FLIGHT 1 END delivered=4/4 d_home=1.8m
```

`tools/verify_flight.py` changes in lockstep, and **drops its
`stop_index == sortie - 1` inference** (`verify_flight.py:349`) in favour of
reading the `stop_index` now logged explicitly. Every fail-closed behaviour it
has is preserved: pre-GO TELEM still dropped, NaN samples still warned, a release
with no truth still FAILs, the L&R fix still cross-checked against config.

Existing `docs/evidence/G4_*.txt` are rendered reports, not `audit.jsonl`, so
they stay readable as historical records.

### 7. SITL

**Field: 6 pads, 4 assigned.** `sitl/spawn_targets.py` `N_PADS = 4` becomes
config-driven (`n_pads: 6`); the assigned queue takes 4 of the 6 spawned ids.
This closes a real gap: today SITL spawns exactly the pads that get assigned, so
**a pad that is never in the queue has never been flown against**. Two permanent
distractors exercise the wrong-id rejection over a whole mission, not just
within one sortie.

**Payload dummies, Tier 1 — geometry + mass.** `sitl/models/cargo_box/model.sdf`
already carries the V1.3 dimensions (`0.16 0.07 0.18` m), but the four instances
sit on the ground at L&R (`aavc_field.sdf:175-191`) and the aircraft model
carries nothing but `camera_link`. Add four cargo-box links to `eft_x6100` in
the belly bay, with mass and inertia.

The geometry closes an open worry: `eft_x6100_base` puts the landing gear at
(±0.20, ±0.32) reaching to z = −0.283, so the belly bay is ≈ 0.40 × 0.64 m by
0.28 m tall. Four boxes laid along Y need 0.28 m of the 0.64 m and 0.18 m of the
0.28 m height. **They fit, with room to spare** — volume is not the binding
constraint after all. Mass: 4 × ~90 g (no.0 egg + 300-gsm card) ≈ 0.36 kg on a
6.831 kg `base_link`, i.e. +5.3%. Tuned gains and the energy estimate then face
a loaded aircraft instead of an empty one.

**Payload dummies, Tier 2 — release actually drops the box.** Each box becomes a
gz `DetachableJoint` (native in Harmonic). When its servo fires the box falls
onto the pad. Three payoffs:

- **mass sheds ~90 g per delivery** — Tier 1 alone would model a pessimistically
  heavy aircraft for the whole flight, which matters when the single-flight
  margin is the thing being measured
- the release is visually verifiable against the correct pad
- the fallen box is a second truth signal for the post-flight audit

Wiring stays out of the flight core: a SITL-only bridge watches the actuator
output PX4 already publishes into gz and fires the detach topic — the same
signal the real servo responds to. The orchestrator never learns it is in SITL.

### 8. Hardware — new G5/G6 items

1. **Four independent release mechanisms.** The rules require one per cargo. The
   airframe has a single servo on AUX 9 today; this needs AUX 9/10/11/12.
   `commands.py:692-700` already indexes `drop_servo_channel + payload_id` and
   bounds-checks against `drop_payload_count`, so the software side is a config
   change.
2. **MTOW and CG.** +0.36 kg of cargo plus three more release mechanisms. The
   7.17 kg AUW figure from `Power-System-Guide-1.pdf` needs re-checking against
   whether it already counted payload.
3. **Energy.** The single-flight margin is the whole question. Measure at G6/G7;
   the §5 abort gate is what protects it in the meantime.
4. If 2 or 3 fails, `eggs_aboard: 2` — two flights, no code change.

---

## Files

| File | Change |
|---|---|
| `orchestrator/mission.py` | loop restructure (the bulk) + per-delivery abort |
| `orchestrator/tactical_align.py` | `_drop_once` / `acquire_and_land_drop` take `payload_id`; retire the "release is ALWAYS payload_id=0" invariant (`:526`, `:548`) |
| `orchestrator/state.py` | `sortie_index` → `flight_index` + `delivery_index` + `flight_ids` |
| `orchestrator/time_policy.py`, `energy_policy.py` | per-flight cost model + the mid-flight predicate |
| `orchestrator/audit.py`, `tools/verify_flight.py` | new grammar, changed together |
| `mission_brain/live_plan.py` | `payload_id` from `ServedStop` instead of the `payload_id=0` literal (`:124`); render N GOTO+DROP pairs per flight |
| `mission_brain/profile.py` | `eggs_aboard`; `drop_count_max` → N; `max_sorties` → `max_flights` |
| `dashboard/` | GO per flight; chips read "flight 1/1 · delivery 2/4"; queue editor unchanged |
| `sitl/spawn_targets.py` | `N_PADS` → config `n_pads: 6` |
| `sitl/models/eft_x6100/model.sdf` | four cargo-box links + detachable joints |
| `sitl/` (new) | payload detach bridge |
| `sitl/aavc_config.yaml` | `eggs_aboard: 4`, `drop_payload_count: 4`, `n_pads: 6` |
| `docs/RULES_AAVC2026.md`, `CLAUDE.md` | record the briefing override |

## Testing

- **Regression pin:** `eggs_aboard=1` must produce a byte-identical plan and
  audit trail to today. This is the proof that rollback works and that the single
  code path really does subsume the old behaviour.
- `test_live_plan.py` — N GOTO+DROP pairs, `payload_id` 0..N-1 distinct,
  `stop_index` globally distinct
- `test_target_tracker.py` — 6 pads / 4 assigned: both distractors enter the
  registry and neither is ever served
- `test_delivery_mission.py` — multi-delivery flight; partial find (serve what
  was confirmed, keep the rest); **mid-flight abort on time and on charge**
- `test_tactical_align.py` — `payload_id` reaches `drop_payload`; airborne
  release still refused
- `verify_flight` tests for the new grammar
- **G4′ SITL:** `n_pads=6`, `eggs_aboard=4` → one flight, four deliveries,
  `tools/verify_flight.py` PASSES

## Open questions

- Does the briefing's per-delivery scoring also re-score transit per delivery?
  Design assumes not (one ingress, one egress per flight — the operator's call,
  and the reading the rules text supports). Confirm at the tech exchange.
- Whether the 7.17 kg AUW already includes cargo (§8.2).
