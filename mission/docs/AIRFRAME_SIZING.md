# AAVC 2026 — Conceptual Airframe Sizing (propulsion + frame)

> **⚠ SUPERSEDED 2026-07-22 — the aircraft is now an EFT X6100 HEXACOPTER.**
> Everything below sizes the retired 700 mm X-quad (4 motors, ~2 kg AUW, 15"
> props) and is kept only as the reasoning trail for that decision. Current
> aircraft: EFT X6100 hexa-X, wheelbase 1.000 m, 6 motors, 18" props on 6S,
> AUW 7.17 kg (Power-System-Guide-1.pdf — weigh at G5), DXF 6S 7500 mAh 140C through a
> Holybro PM03D, and NO optical flow (a Benewake TFmini-S is the only height
> aid). See CLAUDE.md §2 and `sitl/models/eft_x6100_base/model.sdf` for the
> authoritative numbers.


**Date:** 2026-07-07 · **Status:** conceptual, class-level. Verify on eCalc +
manufacturer thrust tables + a bench thrust-stand **before procurement**.

> **⚠ 2026-07-08 — the eCalc pass is DONE: `docs/ECALC_PASS.md` supersedes
> this doc's class numbers.** Key deltas: motor class → **~700 Kv** for 10"@6S
> (the 880–960 Kv row below is the 7–8"-prop class — do not buy it); honest
> AUW with real part weights ≈ **2.5–2.65 kg** (not 2.15) — T/W ~3:1 still
> holds with the selected SunnySky V2814 KV700; peak current ≈ **~110 A
> full-stick** (not 150–160 A) so the battery C-requirement relaxes to real
> ≥25 C, weight-first; mission pack = the team's **DXF 6S 7500 120C** (health
> gates in ECALC_PASS.md §7).

## Requirements

- **Mission:** PX4 **quad-X**, one egg/sortie, land-ON a 1×1 m pad to ~0.1–0.5 m,
  ≤4 sorties in a 20-min window, **20 m AGL hard ceiling**, IAAI KMITL. Precise +
  egg-gentle + reliable.
- **Fixed avionics** (~250–300 g): Pixhawk 6X, Raspberry Pi CM4, single
  OV9281 nadir camera (1280×720 mono GS) + gimbal pitch servo (2026-07-15;
  mass ≈ the old two-cam pair), ARK Flow + TF-Luna rangefinder, ELRS Rx,
  egg-release servo.
- **Payload:** one chicken egg + protective cradle ~80–120 g.
- **Operator design calls (2026-07-07):**
  1. **Single battery for all 4 sorties** (motors off during resupply; the pack
     covers ~16–18 min of actual flight + reserve).
  2. **Thrust-to-weight ~3:1** (speed/agility prioritised over lightness).

## Sizing result — converged AUW ≈ 2.0 kg

| Mass group | Estimate |
|---|---|
| Fixed avionics + egg + cradle | ~400 g |
| Battery (6S Li-ion, ~130 Wh) | ~650 g |
| 4 motors + props + ESC + frame + wiring | ~950 g |
| **AUW** | **~2.0 kg** → 3:1 = **~6 kg thrust (~1.5 kg/motor)** |

## Conceptual BOM (class-level)

| Item | Spec | Rationale |
|---|---|---|
| **Frame** | quad-X, **~500–560 mm** wheelbase, **tall landing gear** | swing 10" props; ground clearance for the downward nadir cam / flow / rangefinder + the egg cradle & release servo |
| **Props** | **10"** (10×4.5–10×5), 2–3 blade | large disk = efficient (endurance) *and* high static thrust (the 3:1) |
| **Motors** | ~~4× ~2810–2814, ~880–960 Kv @ 6S~~ → **SunnySky V2814 KV700 ×4** (ECALC_PASS.md: ~700 Kv is the 10"@6S class), ~1.85–2.0 kg max each | reaches 3:1 at the honest 2.6 kg AUW; hover ≈33% thrust → cool + efficient = reliable |
| **ESC** | **6S 4-in-1, ~55–60 A/ch**, BLHeli_32 / AM32 + telemetry | peak ~30–35 A/motor + margin; FC-readable current/RPM |
| **Battery** | **6S LiPo, ~6000–6500 mAh (~133–144 Wh), 75–100 C** (~800 g) — *revised from Li-ion; see Propulsion verification below* | the 3:1 burst (~150 A) exceeds what a ~130 Wh 6S Li-ion can source (~45–90 A); LiPo delivers it and still gives ~20 min |
| **Voltage** | **6S (22.2 V)** | same power at lower current than 4S → thinner wire, cooler ESC/motors → reliable + efficient |

## Reliability notes

- 6S + low hover throttle keeps motors/ESC cool (the dominant reliability factor
  over a 20-min single-charge mission).
- 10" props give gentle prop-wash for the precise land-ON; the tuned slow final
  crawl (`MPC_LAND_SPEED`) + tall gear absorb the (higher, at 2 kg) landing energy
  and protect the egg.
- High-drain Li-ion sag under the 3:1 burst must stay above PX4's low-voltage
  failsafe — confirm the loaded voltage curve on the bench.

## Verification path (before buying / flying)

1. **eCalc + thrust tables** for the exact motor+prop+ESC+battery combo: confirm
   ~1.5 kg/motor, hover throttle ~20–25 %, motor/ESC temps, loaded pack voltage.
2. **Bench thrust-stand (G5):** thrust curve, current, temperature at hover + WOT.
3. **Endurance test:** confirm ≥ 18 min flight + 20 % reserve on the chosen pack
   at mission-representative power (not pure hover).

## Integration with the flight stack — IMPORTANT

- **Re-tune for the real airframe.** Every gain/limit validated in SITL this
  session (cruise `MPC_XY_CRUISE=10`, the landing params, and the
  `MPC_Z_VEL_MAX_UP=2` climb cap) is calibrated for the SITL `gz_x500` model. The
  real ~2 kg bird needs the System-ID + Autotune module (`tuning/`) run on the
  bench / first flights to re-derive PID gains, and the climb/landing params
  re-checked — the ceiling-overshoot ∝ v² relationship depends on the real
  thrust-to-inertia.
- **Endurance gates mission success.** The mission is time-marginal (barely fits
  4 sorties in 20 min). Propulsion efficiency directly decides whether all 4
  fit — validate real endurance early, and note the 20 m ceiling caps climb speed
  regardless of the 3:1 headroom (so agility helps transit/wind, not climb-out).

## Propulsion verification (Item 1) — 2026-07-07

Physics check (momentum theory + figure of merit ~0.65, drivetrain η ~85 %) —
confirm on eCalc + the motor's published thrust table before buying:

- **Hover:** P_ideal = W^1.5 / √(2ρA) ≈ 124 W (2 kg, 4×10", ρ 1.20); ÷FoM ÷η ≈
  **~250–320 W** → **~12–16 A @ 6S**. Hover throttle at 3:1 ≈ **~35–45 %**
  (healthy headroom; motors loaf → cool + efficient).
- **Endurance:** ~130 Wh × 0.8 ÷ ~290 W mission-avg ≈ **~21 min flight** → covers
  ~16–18 min mission + reserve. ✅
- **Peak current:** all 4 at max (the 3:1) ≈ **~150–160 A**; realistic aggressive
  bursts ~100–130 A; hover ~12–16 A.

**⚠ Battery revision — Li-ion → LiPo.** The ~150 A burst is the binding
constraint. A 6S Li-ion pack at ~130 Wh is only ~1–2 P (P42A/P45B ≈ 45 A/cell) →
sources ~45–90 A, **not 150 A**; reaching 150 A needs 6S**4P** (~370 Wh, ~1.7 kg
— absurd). So to honour 3:1, use a **6S LiPo ~6000–6500 mAh, 75–100 C (~800 g)**.
(If you'd drop to ~2.2:1, a 6S2P Li-ion (~186 Wh) gives *better* endurance — a
real fork.) LiPo nudges AUW to **~2.15 kg** (T/W ~2.8–3:1).

## Power wiring (gauge + connectors)

Sized to the ~150 A brief peak with ~15 A continuous at hover:

| Run | Gauge | Note |
|---|---|---|
| **Main battery leads** (pack → ESC) | **10 AWG** (~5.3 mm²) silicone, high-strand | ~150 A short bursts on 10–20 cm leads; 8 AWG if long / for margin |
| **Battery connector** | **XT90** (90 A cont / 150 A+ burst) — **not XT60** | AS150 + anti-spark for more margin |
| **ESC power input** (4-in-1) | **10 AWG**, short; add a low-ESR input cap | matches leads; tames spikes |
| **Motor phase wires** | **16 AWG** (~40 A/motor) | motors usually pre-wired; extend in 16 AWG |
| **Power module / current sensor** | **Hall-effect, ≥150–200 A** (Holybro PM02D/PM03 class) | shunt modules (~60–120 A) clip at the 3:1 burst |
| **5 V avionics** (CM4 + FC + cams) | **20–22 AWG** from a **UBEC 5 V ≥6 A** | ~2–4 A; keep off the noisy main bus |

## Re-tuning the real airframe (Item 2)

⚠ Needs the **physical bird on a bench** — cannot be done remotely; here is the
procedure + what to expect:

1. **Bench System-ID** (`tuning/sysid.py`; props off → tethered): chirp each axis
   (roll/pitch/yaw), measure the FRF. The 2 kg / 500 mm airframe has higher
   inertia + a lower natural frequency than the SITL `gz_x500`, so expect **lower
   rate-loop P gains** and different phase margins.
2. **Synthesise + apply** (`tuning/synthesis.py`) → verify stability (hover, step
   response) tethered → free flight (G6/G7).
3. **Re-check the mission params today's SITL A/B set — they are `gz_x500`-specific:**
   - **Climb** `MPC_Z_VEL_MAX_UP` — ceiling overshoot ∝ v²/(2·a_decel); a_decel
     depends on the real thrust margin. Re-measure the overshoot on the real bird
     and re-set the cap for the 20 m ceiling.
   - **Landing** `MPC_LAND_SPEED` + descent rungs — re-validate touchdown
     gentleness + precision at ~2.15 kg (more landing energy).
   - **XY** (`MPC_XY_P` …) — the wind-rejection set was A/B'd on `gz_x500`; re-tune.
   - **Cruise** `MPC_XY_CRUISE=10` (as shipped; a 13 m/s probe was never
     committed) — safe to keep; confirm it holds within tilt limits.
4. **Optional dry-run:** a representative SITL model (mass/inertia/motor-thrust ≈
   the real airframe) lets the mission timing + tuning workflow be rehearsed
   before the bird flies — the *actual* gains still come from step 1.

## Open items

- Exact part numbers (post-eCalc).
- Frame selection: 500–560 mm quad-X with tall gear + a payload bay for the egg
  cradle and downward-sensor clearance.
- CG placement + FC/camera vibration isolation at 2 kg.
