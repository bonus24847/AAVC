# Battery capacity + in-flight energy budget

**Date:** 2026-07-22 · **Status:** IMPLEMENTED (see `docs/superpowers/plans/2026-07-22-battery-energy-budget.md`)
**Aircraft:** EFT X6100 hexacopter, DXF 6S 7500 mAh 140C through a Holybro PM03D

## Why

The GCS shows battery percentage, voltage and consumed mAh, but nothing that
answers the one question the operator actually has at each per-sortie GO:
**will this pack cover the next sortie, or does it need swapping first?**

That question matters more on this aircraft than it did on the retired quad, and
the numbers are now first-party rather than guessed. Per the team's own power
system document (`Power-System-Guide-1.pdf`, EFT E5 5008/335KV on 18x6.5" props,
manufacturer bench table at 24 V), a 7.2 kg AUW hexacopter hovers at **~29 A**
(~56 % throttle, 1,200 g per arm).

Against the sortie durations actually measured in today's G4 run:

| sortie | airborne | energy | cumulative |
|---|---|---|---|
| 1 (includes the search sweep) | 4.75 min | 2,296 mAh | 2,296 |
| 2 | 2.87 min | 1,387 mAh | 3,683 |
| 3 | 3.15 min | 1,522 mAh | 5,206 |
| 4 | 3.30 min | 1,595 mAh | **6,800** |

One pack yields 5,625 usable mAh, i.e. **11.6 minutes airborne — about 3.3
sorties**. The full four-sortie mission needs 6,800 mAh, so it **cannot be flown
on a single pack**: at least one swap is required, and on the numbers above the
pack runs out during sortie 4. The rules already have the crew approach the
aircraft between sorties (it lands at L&R and disarms for resupply), so swapping
is allowed — but only if the operator knows to do it BEFORE burning window time
on a sortie that cannot finish. The power-system document independently reaches
the same conclusion and specifies buying at least three packs to rotate.

The mission already reasons this way about *time*: `orchestrator/time_policy.py`
refuses to start work that cannot finish inside the 20-minute window. This is the
same idea for *energy*, and is deliberately built to mirror that module.

## What we measured first (2026-07-22, live SITL)

Before designing anything, we checked whether the input data exists:

| signal | SITL | notes |
|---|---|---|
| `BATTERY_STATUS.current_consumed` | **-383** | negative = not provided |
| `BATTERY_STATUS.current_battery` | **-100** | negative = not provided |
| `battery_remaining` | 100 %, flat for 36 s of flight | no drain at all |
| voltage | 16.20 V, flat | also not a usable proxy |

**SITL provides no battery signal whatsoever.** PX4 ships a battery simulator
(`SIM_BAT_ENABLE` default 1, `SIM_BAT_DRAIN` seconds, `SIM_BAT_MIN_PCT` default
50 %) but it is not active in our gz configuration. Any energy feature written
without addressing this is untestable until G6 on real hardware — which is far
too late to discover it is wrong. Enabling the simulator is therefore part of
this work, not an optional extra.

The real aircraft will provide `current_consumed` **only after the PM03D is
calibrated at G5** (`BAT1_*` are all unset today). So the feature must degrade
sensibly when the good signal is absent, and must say which signal it is using.

## Design

### 1. `orchestrator/energy_policy.py` — new, pure logic

Mirrors `time_policy.py`: a frozen dataclass of costs plus predicates, no I/O, so
it is fully unit-testable without flying.

```
usable_mah()                 = capacity_mah × (1 − reserve_frac)
sortie_cost_mah(history)     = median(history) if history else seed_sortie_mah
can_start_sortie(consumed_mah, history) -> (ok: bool, reason: str)
```

`reserve_frac` is **the existing `failsafes.bat_low_thr` (0.25)**, not a new
number. Below that the flight controller triggers its own low-battery RTL, so
energy the companion is not allowed to plan against is exactly the energy the FC
has already reserved. With a 7500 mAh pack that leaves **5,625 mAh usable**.

`reason` is written for the operator, not the log: e.g. *"1,100 mAh usable left,
next sortie needs ~1,700 — swap the battery"*.

### 2. Two-tier measurement, and always say which tier

| tier | source | when | quality |
|---|---|---|---|
| A | `BATTERY_STATUS.current_consumed` (already read by `raw_subscriber`) | real aircraft after the G5 PM03D calibration | coulomb-counted, trustworthy |
| B | `capacity_mah × (1 − remaining_percent/100)` | anything else, incl. SITL | coarse — PX4's percent is itself estimated |

Tier A is used whenever the FC reports a non-negative `current_consumed`;
otherwise tier B. **The GCS displays the active tier.** These two numbers do not
deserve equal confidence and the person looking at the screen has to know which
one they are being shown — an estimate presented as a measurement is worse than
no number at all.

Instantaneous power comes from MAVSDK's `battery.current_battery_a × voltage_v`
(already subscribed in `telemetry.py`, currently discarded) and is display-only.

### 3. Capacity: config declares it, the FC cross-checks it

New `battery:` block in `sitl/aavc_config.yaml`:

```yaml
battery:
  capacity_mah: 7500        # DXF 6S 7500 mAh 140C
  cells: 6
  seed_sortie_mah: 1700     # 29 A hover x 3.5 min (manufacturer bench table);
                            # only used for sortie 1, then measurement replaces it
```

At startup, read `BAT1_CAPACITY` from the FC and compare. Mismatch or unset
raises an anomaly. That single check catches both live failure modes: *"the pack
was swapped for a different one and nobody updated the config"* and *"the PM03D
was never calibrated so every mAh number is fiction"*.

### 4. Battery-swap detection

While landed at L&R and disarmed (the only moment the crew touches the aircraft),
treat either of these as a swap:

- `current_consumed` drops toward zero, or
- `remaining_percent` rises by more than 15 points

On a swap: reset the consumed baseline, keep the learned per-sortie cost history
(it describes the aircraft, not the pack), and write `BATTERY SWAP sortie=n` to
the audit trail so a post-flight review can explain a discontinuity in the energy
trace instead of treating it as corrupt data.

### 5. Preflight gate — blocking, with a working FORCE

**Corrected during implementation.** This section originally called for a
**critical** `energy` row. Reading the endpoint showed that would not work:
`dashboard/commands.py::preflight_go` checks `state.preflight_can_go`
(= `all_critical_pass`) with **no force escape**, so a critical row makes GO
impossible even with `force: true` — the exact dead path the "time" row hit
(fixed 2026-07-15, a138aaf; see the comment at `orchestrator/preflight.py`).

What was built instead mirrors the proven pattern:
- the `energy` row is **advisory** (`critical=False`) so it shows on the card
  without freezing the board, and
- the refusal lives in `state.sortie_energy_ok`, which the GO endpoint checks as
  `if not state.sortie_energy_ok and not req.force` — directly beside the
  identical `sortie_time_ok` check.

Behaviour is what was asked for — GO is blocked, FORCE overrides — without
recreating a known bug. `tests/test_energy_policy.py` asserts the row stays
advisory AND that the board stays green while it is failing, so a future change
back to `critical=True` fails the suite instead of silently killing FORCE.

### 6. GCS battery panel

Extends the existing battery row rather than adding a new widget:

```
BATT   62%   22.4V   ●tier A
       4,650 / 7,500 mAh used · 975 mAh usable left
       PWR 650 W   (29 A hover reference)
       ⚡ energy for 0 more sorties — SWAP BEFORE NEXT GO
```

The "energy for N more sorties" line is the headline; the swap prompt is styled
like the existing critical chips so it reads at a glance from across a table.

### 7. SITL battery simulation (test infrastructure)

A **SITL-only** param block, applied on the same path as `px4_tuning` but kept
separate and clearly labelled, enabling `SIM_BAT_ENABLE` and setting
`SIM_BAT_DRAIN` / `SIM_BAT_MIN_PCT` so the pack visibly drains across a 4-sortie
run. These parameters mean nothing on the real board and must never be merged
into the flight tuning block.

Draining the simulated pack over roughly one mission window makes the whole chain
observable in SITL: percent falls, tier B accounting tracks it, the policy starts
refusing sorties, and the swap prompt appears.

## Non-goals

- **No remaining-flight-time prediction in seconds.** That needs a power model
  per flight mode (hover / cruise / climb draw very differently) and we have no
  real data to fit one. **Deferred until a real test flight produces a log** —
  the operator will bring the ULog back and we fit the model from measurements
  rather than guessing now.
- **No changes to the FC's own battery failsafes.** They are configured
  correctly and are the last line of defence; this feature plans *above* them.
- **No persistence across runs.** Per-sortie costs are learned within a mission.

## Testing

- Unit tests for `energy_policy` — reserve maths, seed-then-measured cost,
  the block/allow boundary, and the FORCE override releasing a blocked gate.
- Config parity test extended so `battery:` keys stay in step with their
  consumer, in the spirit of `test_px4_tuning_parity.py`.
- SITL: a 4-sortie run with the battery simulator enabled must show the pack
  draining, the sortie-count estimate falling, and the gate blocking once the
  remaining charge drops below one sortie plus reserve.
- Tier-B path is what SITL exercises; tier A is verified at G5 with the
  calibrated PM03D, against a known-charge pack.

## Risks

| risk | mitigation |
|---|---|
| Tier-B numbers are coarse and could block a sortie the aircraft could actually fly | FORCE override; the tier is shown on screen |
| `seed_sortie_mah` derives from a bench table, not this airframe in flight | it is only used for sortie 1; measurement replaces it immediately, and the source is cited |
| SITL sim params leaking onto the real board | kept in a separate, labelled block, never merged into `px4_tuning` |
| A swap misdetected mid-mission would reset the baseline wrongly | only evaluated while landed AND disarmed at L&R, and audited |
