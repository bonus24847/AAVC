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
`BAT1_V_CHARGED=4.18`, `BAT1_V_EMPTY=3.77` (25.1 V full / ~22.6 V empty).
BOARD-checked every preflight. Two claims, verify separately: a correct
VOLTAGE (divider) and a correct PERCENTAGE (endpoints).
⚠ `BAT1_V_DIV` REOPENED 2026-08-20: its multimeter closure was on the old
converter wiring; one multimeter-vs-GCS check ON THE PM02D re-closes it
(three real flights flew it with plausible voltages — re-verify, not fault).

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
