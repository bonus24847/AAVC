# AAVC 2026 — Propulsion eCalc Pass & Exact Part Selection

> **⚠ SUPERSEDED 2026-07-22 — the aircraft is now an EFT X6100 HEXACOPTER.**
> Everything below sizes the retired 700 mm X-quad (4 motors, ~2 kg AUW, 15"
> props) and is kept only as the reasoning trail for that decision. Current
> aircraft: EFT X6100 hexa-X, wheelbase 1.000 m, 6 motors, 18" props on 6S,
> AUW 7.17 kg (Power-System-Guide-1.pdf — weigh at G5), DXF 6S 7500 mAh 140C through a
> Holybro PM03D, and NO optical flow (a Benewake TFmini-S is the only height
> aid). See CLAUDE.md §2 and `sitl/models/eft_x6100_base/model.sdf` for the
> authoritative numbers.


**Date:** 2026-07-08 · **Status:** verification of `AIRFRAME_SIZING.md` (§7 gate 1)
done — exact part numbers selected below. Numbers marked **[web]** were checked
against manufacturer/retailer pages on 2026-07-08; numbers marked **[est]** are
momentum-theory / manufacturer-class estimates (±10%) to be confirmed on the
in-box datasheet + the G5 thrust stand. Prices ≈ USD (×~36 THB), confirm at order.

---

## 1. What this pass CHANGES vs the 2026-07-07 sizing (read first)

1. **Motor class correction: ~700 Kv, not 880–960 Kv.** At 6S (22.2 V) a
   880–960 Kv motor spins a 10" prop toward ~20k rpm unloaded — that Kv belongs
   to 7–8" props. For 10" @ 6S the right class is **≈550–780 Kv**. The candidates
   below deliver the ~1.6–2.0 kg/motor the 3:1 needs at sane rpm and current.
2. **Honest AUW is ≈2.5–2.65 kg, not 2.15 kg.** Real part weights (X500 V2 frame
   610 g [web], V2814 motors 120 g each [web], a real 6S-6000 pack 880–1,075 g
   [web]) push the rollup ~400–500 g over the concept. **The design still
   closes**: T/W ≈ 2.9–3.1 with the selected motors, endurance ≈ 17 min vs
   ~14.6 min of actual motors-on flight time (§5) — but the margin is thinner
   than the concept promised, so the G5 endurance test (§7) is non-negotiable.
3. **Peak current correction: ~90–110 A worst-case, not 150–160 A.** The 3:1
   thrust point costs ~55–65 A total (momentum theory, §4); full-stick on the
   selected motors ~90–110 A. Keep XT90 + 10 AWG (healthy margin); the battery
   C-requirement relaxes from "75–100 C label" to **any quality pack with real
   ≥20–25 C continuous** — which unlocks lighter packs (Tattu 35C class) that
   the mass budget needs anyway.

## 2. Requirements (unchanged)

Hover thrust/motor = AUW/4; max thrust/motor ≥ 3× that (operator call, T/W ~3:1);
prop 10" (efficiency + 500 mm frame clearance: adjacent motors 354 mm apart →
100 mm tip clearance ✓); 6S; one pack flies all 4 sorties.

## 3. Candidates (10" @ 6S, published-data motors only)

| Motor | Kv | Weight | Max thrust (6S, 10–11") | Max A/motor | 4× thrust | Verdict |
|---|---|---|---|---|---|---|
| **SunnySky V2814 KV700** [web: $34.95, 120 g] | 700 | 120 g | ~1.85–2.0 kg (1045) [est] | ~20–24 A [est] | **7.4–8.0 kg** | **SELECTED** — right thrust class, best price, V-series = multirotor line |
| T-Motor MN3508 KV700 [web: "1.9 kg thrust"] | 700 | ~104 g | ~1.9 kg | ~20 A | 7.6 kg | Premium alt: −60 g total, EZO bearings, ~2× price — buy if budget allows |
| T-Motor MN3110 KV780 [web: max 1.2 kg, 481 W/26 A] | 780 | 80 g | **1.2 kg** | 26 A | 4.8 kg | **REJECTED** — T/W only ~1.9–2.2 at real AUW (my earlier BOM-class guess was wrong; the web datasheet kills it) |
| SunnySky X2814 (any Kv) | — | — | — | — | — | **REJECTED** — X-series is the FIXED-WING line (13" props); the sizing doc's "2810–2814 880–960 Kv" row pointed here by accident |
| Generic 2814 900 Kv clones | 900 | ~95 g | no published tables | — | — | REJECTED — no data = no reliability case |

## 4. Physics check at the honest AUW (2.6 kg, 4×10" disks)

Momentum theory, ρ=1.20, disk area A = 4·π·(0.127 m)² = 0.2027 m², FoM 0.65,
drivetrain η 0.85 (same method as AIRFRAME_SIZING.md):

| Point | Thrust | P_ideal | P_electrical | Current @22.2 V |
|---|---|---|---|---|
| Hover (2.6 kg) | 25.5 N | 185 W | **~335 W** (+~15 W avionics ≈ 350 W) | ~16 A |
| 3:1 burst (7.8 kg) | 76.5 N | 962 W | **~1,740 W** | ~78 A |
| Full stick (V2814 ×4) | ~8 kg | — | ~2,000–2,400 W [est] | **~90–110 A** |

- Hover thrust fraction ≈ 33% of max → expect `MPC_THR_HOVER` ≈ 0.35–0.45
  (trim on the real bird; the SITL note in the 3:1 model comment said the same).
- Wiring/connector design point = full stick ~110 A → XT90 (90 A cont/150 A
  burst) + 10 AWG hold with margin. Battery real C needed: 110 A / 6 Ah ≈ 18 C.

## 5. Mass rollup (real parts) + endurance

| Group | g |
|---|---|
| Frame: X500 V2 kit (incl. tall gear, rails, straps) [web] | 610 |
| Motors: 4 × V2814 [web 120 g] | 480 |
| Props: 4 × APC 10×4.5MR class | 55 |
| ESC 4-in-1 60 A + PDB wiring + XT90 + 10 AWG | ~120 |
| Power module (hall) + UBEC 5 V | ~60 |
| Pixhawk 6X + GPS mast + cabling | ~110 |
| CM4 + carrier + OV9281 nadir cam + gimbal servo + ARK Flow + TF-Luna + ELRS RX + egg servo | ~180 |
| Egg + cradle (budget) | ~100 |
| Battery 6S 6000 mAh (Tattu 35C class ~880 g [est]; CNHL 70C is 1,075 g [web] — heavier, avoid unless only option) | 880–1,075 |
| **AUW** | **≈ 2,595–2,790** → design to **≤2.65 kg** (pick the light pack; single-camera 2026-07-15: the OV9281 + gimbal servo ≈ the old two-cam pair — mass wash) |

**Endurance @2.6 kg:** hover ≈350 W; mission-average (climbs + 10 m/s cruise +
descents) ≈ **~360–380 W** → 6000 mAh (133 Wh) × 0.8 usable ≈ 106 Wh →
**≈17 min**. Motors-on flight time across the validated 4-sortie mission ≈
window 1,027 s − 3 disarmed resupply holds (~150 s) ≈ **14.6 min** → reserve
≈ 15%. **Verdict: closes, but thin** on a new 6000 pack — resolved by the
owned pack below.

### 5b. Owned pack: DXF 6S 7500 mAh 120C (operator, 2026-07-08) — the mission pack

The team already owns a DXF 6S 7500 120C (condition: "อาจจะไม่เต็มที่" — assume
aged). DXF's 6S soft-case bricks in this class weigh **~822–890 g incl. leads**
[web: 7500-GTR 822 g @135×43×55 mm; 8400/120C 890 g] — i.e. **lighter per Wh
than a new Tattu 6000** and it drops straight into the §5 rollup at the same
~2.6 kg design point.

| | Nominal | Health-derated (×0.85) |
|---|---|---|
| Energy | 166.5 Wh | ~142 Wh |
| Usable (×0.8) | 133 Wh | **~113 Wh** |
| Endurance @ ~370 W | ~21.5 min | **~18.3 min** |
| Reserve vs 14.6 min flight | ~32% | **~20%** ✓ |
| Burst need (110 A full-stick) | 14.7 C | trivial even aged (sag gate §7 still applies) |

**Plan:** the DXF flies the mission (pending the §7 health gates); buy **one**
new light 6S 6000–6500 (≤950 g) as backup/practice instead of 2–3 new packs
(saves ~3–4.5k THB). If the DXF fails the health gate it demotes to
bench/practice and the new pack takes the mission. Checks specific to this
pack: confirm it is the **22.2 V (3.7 V/cell) variant** — DXF also sells a
22.8 V HV flavor of the 7500 [web]; confirm the connector (DXF ships EC5 *or*
XT90 [web]) and re-terminate to **XT90** if needed; measured-capacity and
cell-IR gates in §7.

## 6. Selected parts (the "เบอร์จริง")

| # | Part | Exact selection | Qty | ≈ USD | Note |
|---|---|---|---|---|---|
| A1 | Frame | **Holybro X500 V2 frame kit** [web $122.99] | 1 | 123 | 610 g, RPi-ready platform board, dual rails, straps included |
| A2 | Motors | **SunnySky V2814 KV700** [web $34.95 ea] | 4 + 1 spare | 175 | SunnySky USA shows Back Order [web] — order via TH/CN stock (Shopee/Banggood/AliExpress); premium alt: T-Motor MN3508 KV700 |
| A3 | Props | **APC 10×4.5MR** (MR = multirotor) sets | 4 sets (8 CW+8 CCW) | 40–60 | CF (T-Motor P10) optional later; plastic is fine + crash-cheap for practice |
| A4 | ESC | **Holybro Tekko32 F4 Metal 65 A 4-in-1** (BLHeli_32, current sense, DShot RPM) | 1 | 90–110 | budget alt: SpeedyBee BLS 60 A (~$55) |
| A5 | Battery | **Mission pack = owned DXF 6S 7500 120C** (§5b, gated by §7 health checks) + buy **1×** light 6S 6000–6500 ≤950 g (Tattu 35C class) as backup/practice | own + 1 | 90–130 | CNHL G+Plus 70C = 1,075 g [web] → fallback only; **weight beats C-label here** |
| A6 | Charger | **VERIFY-own first** (the DXF implies a 6S charger exists) — must do 6S balance + storage mode, ≥10 A; else ToolkitRC M6D / ISDT class + 2× LiPo bags + cell checker | 1 set | 0–120 | dual-channel = practice-day turnaround |
| A7 | Connectors/wire | XT90 ×3 pairs (1 anti-spark), 10 AWG 2 m, 16 AWG 1 m, heatshrink | set | 15–25 | |
| A8 | Power module | Hall-effect ≥120 A class (Mauch PL/Holybro digital for FMUv6X — **verify what shipped with the 6X first**, BOM_REPORT §3) | 1 | 40–90 | shunt PM02D ok for logging, hall preferred for the burst |
| A9 | UBEC | 5 V / 6–10 A switching BEC (CM4 + cams rail) | 1 | 10–18 | |
| B3 | Servo | **EMAX ES08MA II / MG90S metal-gear** + horn/linkage | 1 + 1 spare | 8–15 | hold force for the egg latch is tiny; metal gear for reliability |

Camera/flow/rangefinder per `BOM_REPORT.md` §4B–C (single **Meige OV9281**
mono GS on a gimbal pitch servo — 2026-07-15 supersedes the CM3 Wide + GS
pair; ARK Flow, TF-Luna unchanged).

**Propulsion+power subtotal ≈ $700–950 (≈25,000–34,000 THB) with 2 packs** —
inside the BOM report's band.

## 7. Confirmation gates (before/at G5 — this pass does not replace them)

1. **In-box datasheet check** (V2814 ships with its thrust table): 6S + 1045 row
   must show ≥1.8 kg max and ≤~25 A — else swap to MN3508-700 before building.
2. **Thrust stand (G5)**: hover point (~650 g) current + temp after 5 min ≤60 °C
   class; full-throttle current ×4 vs the A5 pack's real C.
3. **Endurance bench**: ≥18 min at 360–380 W average + 20% remaining — the
   single-pack-4-sorties call lives or dies here (fallback: swap packs at the
   2nd resupply — the mission already disarms at L&R, zero code change).
4. **Loaded-voltage sag** under a 110 A burst ≥ PX4 low-battery failsafe line.
4b. **DXF pack health gate** (§5b, before it earns the mission): confirm
   22.2 V variant + connector; full charge→discharge capacity test at ~15 A —
   must deliver **≥6,300 mAh**; cell-IR spread balanced (no cell ≫ others);
   storage/charge balance holds overnight. Fail any → demote to practice pack.
5. Re-run the **System-ID re-tune** at the real ~2.6 kg (AIRFRAME_SIZING.md §Re-tuning)
   — and re-measure climb overshoot for the 20 m ceiling (overshoot ∝ v², heavier
   bird = more inertia).

## 8. Sizing-doc deltas recorded

`AIRFRAME_SIZING.md` rows superseded by this pass: motor class (→700 Kv), AUW
(→2.5–2.65 kg realistic), peak current (→~110 A full-stick / ~78 A at 3:1),
battery ("75–100C" → quality ≥25C real, weight-first). Marked in-place there.
