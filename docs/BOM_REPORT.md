# AAVC 2026 — BOM & Procurement Report

**Date:** 2026-07-08 · **Status:** procurement-ready draft — quantities and specs
final, exact part numbers pending the eCalc pass (§7); prices are ESTIMATE BANDS
(THB, July 2026) to confirm at order time.
**Basis:** `docs/AIRFRAME_SIZING.md` (2026-07-07 conceptual sizing, LiPo revision),
rules V1.3 mission requirements, and the validated SITL geometry (nadir decode of
the 400 mm marker at the 12 m sweep). Software is competition-ready on `main`
(4/4 deliveries, 5 layouts, verify PASS) — **this hardware path is the remaining
critical path to winning.** Event ≈ late August → ~7 weeks.

---

## 1. Executive summary

- **Aircraft:** quad-X, 500–560 mm, 10" props, 6S, AUW ≈ **2.15 kg**, T/W ≈ 3:1
  (~1.5 kg thrust/motor). One 6S LiPo pack flies all 4 sorties (motors off during
  resupply).
- **Already owned (§3):** Pixhawk 6X (PX4 1.17.0, cal done), RPi CM4 + baseboard,
  full ELRS RC chain (TX16S + Nomad + DBR4), GCS laptop.
- **To buy (§4):** complete propulsion + power chain, both mission cameras, optical
  flow + rangefinder, egg-release servo, spares. Exact part numbers are now
  selected — see **`docs/ECALC_PASS.md`** (2026-07-08).
  Rough total: **~28,000–48,000 THB** (mission pack = the owned DXF 6S 7500;
  one new backup pack included).
- **Undecided → decided by this report (§5):** nadir camera. Recommendation:
  **Raspberry Pi Camera Module 3 Wide** as primary (matches the validated SITL
  sweep geometry with ~1.7× the decode margin), **Pi Global Shutter (IMX296) + 4 mm
  lens as the motion-blur hedge — buy both** (~5k THB combined, cheap insurance).
- **Longest lead time:** ARK Flow (import) — **order first, this week.** Every week
  of procurement slip is a week less of G6/G7 practice.

## 2. Requirements recap (what the BOM must deliver)

| Requirement | Source | BOM consequence |
|---|---|---|
| Land-ON a 1×1 m pad, release after touchdown | rules V1.3 | egg-release servo (AUX 9, PWM 1900/1100 per config), gentle `MPC_LAND_SPEED=0.3` → tall gear + cradle absorb touchdown |
| Decode 400 mm ArUco at 12 m sweep | vision/detectors | nadir camera ≥1280 px wide at the SITL FOV class (§5 math) |
| 4 sorties ≤ 20 min on ONE battery | operator call 2026-07-07 | ~130–144 Wh 6S pack; ~21 min endurance at mission-avg power (sizing doc §Propulsion) |
| T/W ~3:1 at 2.15 kg → burst ~150 A | operator call 2026-07-07 | **LiPo 75–100C (not Li-ion)**, XT90, 10 AWG, hall-effect current sensing |
| No-RTK final-metre precision | locked decision | ARK Flow (CAN) + TF-Luna down (EKF2_OF_CTRL at G5) |
| No internet/4G in flight (DQ) | rules | field network = LOCAL only (CM4 AP / travel router), no cloud parts |

## 3. Already owned — verify, don't re-buy

| Item | Status | Verify at G5 |
|---|---|---|
| Pixhawk 6X (PX4 1.17.0, Quad-X params + calibration done) | **HAVE** | which GPS/compass + power module shipped in the kit (M9N/M10 + PM02D typical) — the PM02D is fine for logging but §4A still wants a ≥150 A hall sensor for the 3:1 burst |
| Raspberry Pi CM4 + baseboard | **HAVE** | a free USB port for the OV9281 (UVC — CSI count no longer matters, single-camera 2026-07-15) |
| ELRS RC chain: TX16S + Nomad TX + DBR4 RX | **HAVE** (HITL-verified link) | CRSF wiring to the 6X; failsafe/kill switch mapping (FLIGHT.md G5 sheet) |
| GCS laptop + SITL dev machine | **HAVE** | field power/shade; local-only network to the CM4 |
| **DXF 6S 7500 mAh 120C LiPo** (added 2026-07-08 — condition unverified) | **HAVE** | ECALC_PASS.md §5b/§7: confirm 22.2 V variant + connector→XT90; capacity test ≥6,300 mAh out + cell-IR balance → then it IS the mission pack (~822–890 g class, ~18–21 min endurance at 2.6 kg) |

## 4. BOM — to buy

Statuses: **P0** = order this week (blocks G5/G6 or long lead) · **P1** = order
with the frame build · **P2** = spares/nice-to-have.
Prices = THB estimate bands, confirm at order.

### A. Propulsion & power (from AIRFRAME_SIZING.md, LiPo revision)

| # | Item | Spec | Qty | Est. mass | Est. THB | Prio |
|---|---|---|---|---|---|---|
| A1 | Frame | quad-X **500–560 mm**, tall landing gear, payload bay for cradle + downward sensors | 1 | (in 950 g group) | 2,500–4,500 | P0 |
| A2 | Motors | **SunnySky V2814 KV700** (selected, ECALC_PASS.md §3/§6 — the earlier "880–960 Kv" was the 7–8"-prop class; 10"@6S wants ~700 Kv), ~1.85–2.0 kg max each | 4 (+1 spare P2) | 480 g | 4,500–6,500 | P0 |
| A3 | Props | **10×4.5–10×5**, 2–3 blade | 4 sets (12+ props incl. spares) | — | 600–1,200 | P0 |
| A4 | ESC | **6S 4-in-1, 55–60 A/ch**, BLHeli_32/AM32, current+RPM telemetry | 1 | — | 1,800–3,500 | P0 |
| A5 | Battery | **Mission pack = owned DXF 6S 7500 120C** (§3, gated by ECALC_PASS.md §7 health checks) + **buy 1×** light 6S 6000–6500 ≤950 g real ≥25C (Tattu 35C class) as backup/practice | own + **1** | 850–950 g flying | 3,500–6,500 | **P0** |
| A6 | Charger + safety | **VERIFY-own first** (the DXF implies one exists): 6S balance + storage mode, ≥10 A; else buy (dual-channel eases resupply practice) + LiPo-safe bags ×2 + cell checker | 1 set | ground | 0–4,000 | **P0** |
| A7 | Connectors + wire | **XT90** ×3 pairs (anti-spark on pack side), **10 AWG** main leads ~2 m, 16 AWG motor extensions, heatshrink | set | — | 500–900 | P1 |
| A8 | Power module | **Hall-effect ≥150–200 A** (PM03/Mauch class) — shunt modules clip at the 3:1 burst | 1 | — | 1,500–3,000 | P1 |
| A9 | UBEC | **5 V ≥6 A** dedicated avionics rail (CM4 + cams off the noisy main bus) | 1 | — | 300–600 | P1 |

### B. Mission sensors & payload systems

| # | Item | Spec | Qty | Est. THB | Prio |
|---|---|---|---|---|---|
| B1 | Optical flow | **ARK Flow (CAN)** — import, longest lead time in the BOM → order FIRST. (Fallback if unobtainable in time: Matek 3901-L0X, UART — different integration, decide only if forced) | 1 | 4,500–6,500 | **P0** |
| B2 | Rangefinder | **TF-Luna** (downward, flow height + `SENS_FLOW_MAXHGT=4`) | 1 | 900–1,500 | P1 |
| B3 | Egg-release servo | Metal-gear micro/standard (MG90S–DS3225 class by hold force), AUX ch 9, PWM hold 1100 / release 1900 (config `connection:`) + linkage/horn | 1 (+1 spare P2) | 300–800 | **P0** (G5 bench gate) |
| B4 | Egg cradle | Foam/TPU print, egg + cradle budget **80–120 g**; gravity-open hold actuated by B3. Bay MUST fit the rules-V1.3 organiser box (heart, **~16×7×18 cm, 300-gsm, handled**) — not just a bare egg | 1 (+spares) | 300–500 (materials) | P1 |
| B5 | Practice eggs | For G7/G8 + resupply drill | ~2 dozen | 150–300 | P2 |

### C. Camera (single-camera decision 2026-07-15 — supersedes §5)

| # | Item | Spec | Qty | Est. THB | Prio |
|---|---|---|---|---|---|
| C1 | Nadir camera | **Meige OV9281** USB UVC — **mono, global shutter, 1280×720, ≤120 fps** (operator-selected 2026-07-15; lens HFOV unstated on the listing → MEASURE at G6, config ships a 99.7° UNMEASURED placeholder) | 1 | (operator-sourced — price TBD) | **P0** |
| C2 | Gimbal pitch servo + mount | Single-axis stabilized-nadir mount (PX4 mount driver, `gimbal:` config block); metal-gear servo on an AUX channel + vibration-isolated camera cage. **VERIFY at G5:** travel/direction (nose-down ⇒ servo compensates), PWM band, no collision with the egg-release servo (AUX 9) | 1 | 300–900 (servo + print) | **P0** (G5 bench gate) |

(C3 oblique cue cam + C4 dual-mounts: **retired** — the oblique role is gone;
the white-pad cue lives in the nadir frame. The ±5°-mount-error ≈ 0.9 m per
10 m AGL projection note now applies to the gimbal's residual pitch trim.)

### D. Integration & field kit

| # | Item | Spec | Est. THB | Prio |
|---|---|---|---|---|
| D1 | Field network | Local-only link laptop↔CM4 (CM4 as AP, or travel router) — **no internet (DQ)** | 0–1,200 | P1 |
| D2 | Mounting/misc | Standoffs, battery straps ×2, Velcro, zip ties, vibration pads (FC + cams), CG ballast | 500–1,000 | P1 |
| D3 | Spares kit | Prop nuts, XT90 spares, 10/16 AWG offcuts, servo horn spares, SD cards ×2 | 400–800 | P2 |

**Rough total: ~28,000–50,000 THB** (with 2 packs; camera operator-sourced —
gimbal servo + mount is the only new C-section spend).

## 5. Nadir camera — the open decision, resolved — **SUPERSEDED 2026-07-15**

> The operator selected the **Meige OV9281** (mono global-shutter UVC,
> 1280×720) on a stabilized-nadir gimbal servo — replacing the CM3-Wide/GS
> pair below AND the oblique cue cam. Width stays 1280 px → the 18 px decode
> floor and the 4-leg sweep are unchanged; global shutter removes the motion
> blur/skew concern that motivated the C2 hedge; mono is detector-safe (the
> decode is grayscale; the pad cue's brightness/shape/contrast gates carry
> the discrimination). The lens HFOV is unstated on the listing — the bench
> gate below still applies at G6: measure the real HFOV + decode rate on a
> printed pad. The analysis is kept for the method + fallback candidates.

**Requirement:** decode the 400 mm marker at the 12 m sweep. The detector floor is
~15–18 px across the marker (with the ROI ×4 booster) — SITL validates exactly
18 px at 1280 px/99.7° HFOV. px-on-marker = width_px × 0.4 / (2·12·tan(HFOV/2)).

| Candidate | Res (used) | HFOV | Marker @12 m | Swath @12 m | Sweep legs* | Notes |
|---|---|---|---|---|---|---|
| SITL reference | 1280×960 | 99.7° | **18 px** (floor) | 28.4 m | 4 | what the pre-2026-07-15 evidence was flown on |
| **Meige OV9281 (flying)** | 1280×720 | unmeasured (placeholder 99.7°) | **18 px** at the placeholder (width-only math) | 28.4 m at the placeholder | 4 | mono **global shutter** (no skew/blur at 10 m/s), ≤120 fps UVC; height 960→720 only trims the along-track footprint (~5 frames/pad @3.3 Hz still ≥ confirm_votes 2) |
| CM3 Wide (old pick) | 2304×1296 binned | ~102° | ~31 px | ~29.7 m | 4 | rolling shutter; picamera2 path — fallback if the OV9281 lens proves too narrow |
| Pi GS IMX296 + 4 mm | 1456×1088 | ~75° | ~32 px | ~18.4 m | ~6 | the old GS hedge; sweep ~1.5× longer |
| Generic UVC 1080p | 1920×1080 | ~78° | ~40 px | ~19.4 m | ~6 | v4l2 backend ready; fps/latency vary wildly by model |

\* legs over the 57–74 m search-area width at overlap 0.4; more legs = longer
sortie-1 sweep (only sortie 1 sweeps — the registry + top-up fix serve the rest).

**Bench gate (G6, printed 1×1 m pad), now for the OV9281:** MEASURE the real
lens HFOV (px-on-marker = width_px × 0.4 / (2·12·tan(HFOV/2)) — at 1280 px the
marker holds ≥ the 18 px floor for any HFOV ≤ ~100°) + decode rate at 12 m
equivalent distance, static and on a moving rig; trim the gimbal's residual
pitch via `depression_deg`. If the lens is narrower, the sweep gets MORE
margin (narrower swath → recompute legs via `search.overlap/spacing`). The
projection is calibrated, not assumed.

## 6. Mass & power sanity (from AIRFRAME_SIZING.md)

| Group | Budget |
|---|---|
| Fixed avionics + egg + cradle | ~400 g (OV9281 + gimbal servo ≈ 30–50 g are inside this) |
| Battery 6S LiPo | ~800 g |
| Motors + props + ESC + frame + wiring | ~950 g |
| **AUW** | **≈ 2.15 kg** → 3:1 needs ~6.4 kg thrust (~1.6 kg/motor — top of the A2 class, confirm on thrust tables) |

Hover ~250–320 W (~12–16 A @ 6S); mission-avg ~290 W → **~21 min** on 130 Wh ✓
covers the 16–18 min mission + reserve. Peak (all-motor 3:1) ~150–160 A → the
75–100 C pack, XT90, 10 AWG, hall sensor line up with that burst.

## 7. Verification gates (unchanged from AIRFRAME_SIZING.md — do not skip)

1. **Before ordering motors/ESC/pack:** eCalc + manufacturer thrust tables for the
   exact combo — confirm ~1.5–1.6 kg/motor, hover throttle, temps, loaded pack
   voltage above the PX4 low-voltage failsafe under the 150 A burst.
2. **G5 bench (props off):** motor map, egg-servo release→hold on AUX 9, camera
   frames fresh, ARK Flow CAN enumeration + `SENS_FLOW_ROT` check, printed-pad
   decode.
3. **Endurance test:** ≥18 min at mission-representative power + 20% reserve on the
   chosen pack **before** committing the single-battery plan.
4. **System-ID re-tune at 2.15 kg** (`tuning/`, procedure in AIRFRAME_SIZING.md §Re-tuning)
   — SITL gains do NOT transfer; also re-measure climb overshoot for the 20 m ceiling.

## 8. Procurement timeline vs the ~7 weeks left

| Week | Action |
|---|---|
| **W1 (now)** | Order all **P0** (ARK Flow first — import lead time), run the eCalc pass, pick exact part numbers |
| W2 | Order P1 with the frame; print cradle + mounts; bench the two nadir cams on a printed pad |
| W2–3 | Build + **G5 bench** (props off) |
| W4 | **G6 tethered**: hover, calibration, flow A/B, bench release, decode lock |
| W5–6 | **G7 free flight**: full multi-sortie practice at the field; real decode stats → confirm `confirm_votes=2` |
| W7 | **G8 dress rehearsal**: live eggs, 20-min window, resupply drill |

## 9. Open items after this report

- ~~Exact part numbers (post-eCalc, W1)~~ **DONE 2026-07-08** → `docs/ECALC_PASS.md`
  (V2814 KV700 ×4, X500 V2 frame, Tekko32 65 A, APC 10×4.5MR, owned DXF 7500 as
  mission pack + 1 backup; honest AUW ≈2.6 kg, T/W ~3:1, endurance ~18 min derated).
- ~~CM4 carrier CSI count (decides C3 CSI-vs-USB)~~ **MOOT 2026-07-15** — single
  OV9281 over USB UVC; C3 retired.
- OV9281 lens HFOV + fps/FOURCC defaults — measure/bench-pick at G5/G6
  (`--fourcc GREY --fps 50` shipped as CLI knobs).
- Gimbal servo model + AUX channel + PWM band; MNT_* values — VERIFY-AT-G5.
- What shipped with the 6X kit (GPS/compass model, PM02D) — adjust A8 if a suitable
  ≥150 A hall sensor is already in hand.
- Cradle geometry vs landing-gear height (egg clearance at `MPC_LAND_SPEED=0.3` touchdown).
