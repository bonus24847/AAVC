# Power & battery — gauge truth, PM02D, and THE energy answer

## Wiring truth (2026-08-20)
Pack → **PM02D → FC/avionics ONLY**; motors run from a SEPARATE board the FC
cannot sense. So: FC voltage = real; FC current = **avionics only (~0.6-0.8 A)**,
never the ~30-43 A of flight.

**THE TRAP:** the PM02D's current output makes coulomb counting *look*
possible. It is not: an avionics-only counter reads a pack that never empties
— fails OPTIMISTIC and silently. Three pins keep that door shut, all
deliberate, all checked:
- `BAT1_CAPACITY = -1` (any ≤0 is fine) — selects PX4's voltage-only SoC
  branch. BOARD check in `make preflight`.
- `connection.raw_telemetry_port: 0` — the only path that ever fills
  `battery_consumed_mah` stays off.
- energy accounting stays tier B (percent-derived) in
  `orchestrator/energy_policy.py`.

`BAT1_I_CHANNEL=-1` is a NO-OP (documented board default) — not a switch.

## The gauge is interpolate(cell_v, V_EMPTY, V_CHARGED) — nothing else
Endpoints on the board (17000 semi-solid, since 2026-08-19):
`BAT1_V_CHARGED=4.18`, `BAT1_V_EMPTY = 3.40` (25.1 V full / ~22.6 V empty).
BOARD-checked every preflight. Two claims, verify separately: a correct
VOLTAGE (divider) and a correct PERCENTAGE (endpoints).
✅ `BAT1_V_DIV` RE-CLOSED ON THE PM02D 2026-08-23. Its earlier closure was on
the old converter wiring and the divider belongs to the module, so it was
re-verified the same way: multimeter **24.9 V** at the connector vs the FC's
own **24.89 V** read over MAVSDK — 0.01 V, 0.04%. Board still carries
`BAT1_V_DIV = -1` (the 6X default divider), i.e. nothing to change.
Both claims were checked in that one reading, which is the point of splitting
them: the VOLTAGE agrees with a meter, and the PERCENTAGE agrees with its own
arithmetic — 4.148 V/cell through `interpolate(3.77, 4.18)` is 92% against the
FC's 93-94%. Re-verify again after any change to the power module.

## Measured sag per flight (ULog ground truth, 2026-08-20 session)

| flight | % start → min under load | V/cell min under load |
|---|---|---|
| 1 (42 s) | 78 → 61 | 3.864 |
| 2 (100 s) | 75 → 40 | 3.761 |
| 3 (121 s) | 61 → 18 | 3.684 |

Flight 3's floor-RTH fired at a read of 28% sustained while resting SoC was
~60% — the 30-40 point sag is now MEASURED, not modeled
(docs/evidence/ulog_review_2026-08-21.md).

## Measured sag — read this before "fixing" a battery RTH
No current sensing → no load compensation → the % SAGS under thrust and
rebounds. **Measured 2026-08-20 (flight 3): 28% under load at ~65-70% resting
SoC — a ~30-35 point sag at ~30-40 A.** The watchdog floor (30% + 5 s
sustain) therefore fires EARLY under load: conservative, by design. The
operational answer is a CHARGED pack, never a lower floor or thinner margin.
Also: pack rests above V_CHARGED right after charging, so the gauge pins at
100% for the first minute — harmless.

## THE energy answer (2026-08-20 — best available; all hover currents are CALCULATED)
Full 4-egg practice/comp mission, one flight:

| Item | Value | Basis |
|---|---|---|
| Hover current | ~43 A | scaled bench table (√-law), NEVER measured |
| Full mission airtime | ~8-10 min | transit+sweep+4 deliveries (SITL: 457-880 s) |
| → Mission energy | **~5.7-7.2 Ah** | 43 A × time |
| Usable (reserve 0.25) | 12.75 Ah of 17.0 | config |
| GO-gate budget | 4.70 Ah (1750+3×900+250) | seeds, deliberately lean |
| Verdict | flies with margin **IF the pack starts charged** | gauge sag is the real limiter, not mAh |
| Parasitics | 0.6-0.76 A parked (≈3.8 Ah per 5 h plugged) | measured — UNPLUG between sessions |

## Ground-truth collection (replaces arithmetic — started 2026-08-20)
1. **TELEM now records batt=/vbat= at 1 Hz** (first battery series ever);
   `make verify` prints per-flight first→last/min figures.
2. **Charger log — every charge**: date · pack · mAh returned · flights/airtime
   since last charge. The charger's returned-mAh is the only true consumption
   measure this wiring allows.
3. **Rest voltage** before/after each flight (GCS ⬆ MSL bar shows vbat via
   the battery chip; 1 min settle).
4. When enough rows exist: re-derive `seed_sortie_mah`/`seed_delivery_mah`
   (sitl/aavc_config.yaml battery block) and `_DELIVERY_BATT_MARGIN_PCT`
   (orchestrator/mission.py) from DATA; also pull the CM4's real Aug-17/18
   `FLIGHT n ENERGY` audit lines when it is reachable.

## Cross-file consistency (manual check until a tool exists)
The endpoints/capacity appear in: `tools/preflight_params.py` (board truth),
both site configs' battery blocks + power narratives, CLAUDE.md §2+G5,
docs/FLIGHT.md. 2026-08-20 sweep left them consistent at 4.18/3.77/17000 —
when the NEXT pack arrives, change ALL of them in one commit.

## Two packs in parallel for the 30-Aug flight (operator decision 2026-08-29)
17000 semi-solid ∥ 15000 6S = 32 Ah nominal, +1.3 kg → AUW ≈ 8.5 kg (T/W 2.7).
Measured basis: hover 38–39 A / mission mean 36 A at 7.2 kg; 29-Aug 3563 mAh in
349 s. Scaled (P ∝ m^1.5): ≈ 46 A mean, 3-egg mission ≈ 10–11 min ≈ 8 Ah of 32
(24 usable at the 25 % reserve) — energy margin ≈ 2.9×. The voltage gauge's
sag halves with the current split, so the loaded reading should end ≈ 55–60 %.
Rules: connect only with both packs full and within 0.1 V of each other (a
mismatch cross-charges at connection); path rated ≥ 100 A peak; XT90 to the
aircraft last; mount the 15000 to keep the CG centred; endpoints stay
3.77/4.18 (conservative for the LiPo half); `battery.capacity_mah 32000` in
both configs; `MPC_THR_HOVER` seed 0.65 (BOARD).

## The floors ladder (since 2026-08-29)
| floor | who | action |
|---|---|---|
| 30 % | mission (`egress_battery_pct`) | planned corridor egress, land, swap |
| 20 % | companion watchdog (`rth_battery_pct`, 5 s sustain) | ROUTED RTH via the gateways (`expected_mode` keeps D3 quiet) — ends the process |
| 15 % | PX4 `BAT_CRIT_THR`, `COM_LOW_BAT_ACT 3` | straight-line RTL at 25 m; D3 → FC FAILSAFE stand-down |
| 10 % | companion (`land_battery_pct`) | LAND in place — ends the process |
| 7 % | PX4 `BAT_EMERGEN_THR` | land |
A tie between the companion and PX4 (15/15 until 2026-08-29) always went to
PX4 (no sustain). Only the 30 % egress keeps the orchestrator alive for a
next gate — and on the real bird that gate refuses at once (CLAUDE.md §0f,
deferred item).


## 2026-08-29 night (Bang Bo, parallel pack): endpoints + internal resistance re-fitted

Three ULogs (`15_54_13`, `15_58_06`, `16_03_38`), 17000 ∥ 15000 aboard: flight current
**53–55 A mean** (max 80 A), hover motor mean **0.69–0.72** (→ `MPC_THR_HOVER 0.70`),
5.6 Ah drawn in total while the voltage gauge fell 83 → 52 % resting / **33 % under
load**. Sag was 1.3–1.5 V at 54 A on every flight = **R = 0.0038–0.0043 Ω/cell**.
Operator-approved: **`BAT1_R_INTERNAL = 0.004`** (PX4 adds I·R back to the cell voltage,
`battery.cpp` l.226, so the loaded % reads like the resting %) and **`BAT1_V_EMPTY = 3.65`**
(3.77 called the pack empty with ~30 % real charge left; at 3.65 the floors sit at real
reserves of ~35 % egress-30 / ~28 % RTH-20 / ~22 % PX4-crit-15 / ~15 % land-10).
`BAT1_CAPACITY` stays −1: 1.17's fusion is `min(voltage-based, coulomb)` and can only read
lower, so the coulomb counter cannot fix a pessimistic voltage gauge.


## 2026-08-30 03:20 — V_EMPTY 3.65 -> 3.40, measured in flight on the parallel pack

The 3.65 + `BAT1_R_INTERNAL 0.004` gauge was itself measured in flight (Bang Bo
`17_32_04`, `17_41_39`): it falls **7.7 gauge points per Ah** (the old 3.77 with
no compensation fell 14.4 — the change halved the error but did not remove it).
The 3-egg competition mission draws **≈ 9.9 Ah at 54 A over 11 min**, so at 3.65
it would end at **gauge 24 %** and cross the 30 % planned-egress floor at minute
10 — during the third delivery — with the pack under load still at 21.6 V.
Resting-voltage deltas between flights put the real usable capacity of the two
used packs at ~17-20 Ah, i.e. the mission uses about half of it.

At **`BAT1_V_EMPTY = 3.40`** the same mission ends at ~48 %, the 30 % floor moves
to minute 15, and each floor lands where the pack really is: 30 % = 20.5 V under
load, 20 % = 20.0 V, PX4 `BAT_CRIT_THR` 15 % = 19.8 V, `BAT_EMERGEN_THR` 7 % =
19.5 V. No floor was changed. ⚠ The first write did NOT survive a battery
unplug (read back 3.65) — this board's param storage has a history of failed
imports, so ALWAYS re-read after any power cycle; the second write was verified
across a deliberate `action.reboot()`.

Pilot's rule when the gauge is in doubt — the raw pack voltage under load:
**< 21.0 V head home, < 20.4 V land now.**
