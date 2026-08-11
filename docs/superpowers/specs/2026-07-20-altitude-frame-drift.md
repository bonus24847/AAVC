# Per-arm altitude-frame drift eats the ceiling margin

**Date:** 2026-07-20
**Status:** OPEN — measured, not yet fixed
**Severity:** competition risk (a >20 m sample is a rules ceiling breach)

---

## The problem

Transit is commanded at **19.5 m** (`transit_alt = 20 − _ALT_BIAS_MARGIN_M`), but
the altitude the aircraft actually holds wanders **±0.9 m between sorties** — and
the wander is per-sortie, not per-sample: each sortie holds its own offset for
the whole flight.

Per-sortie median transit altitude, three independent 4-sortie SITL runs:

| run | S1 | S2 | S3 | S4 | spread |
|---|---|---|---|---|---|
| baseline (07-15, unmodified code) | 19.62 | 19.96 | 19.18 | 19.14 | 0.82 m |
| 07-20 run 1 | 19.35 | 20.09 | 18.53 | 19.32 | 1.56 m |
| 07-20 run 2 | 19.26 | 18.61 | 20.29 | 20.35 | 1.74 m |

The sign is random. Run 1 drew −0.97 m (harmless); run 2 drew +0.85 m twice in a
row and produced `verify_flight`'s
`altitude: max 20.69 m — 3 consecutive samples above 20.5 m`. An earlier G4 run
on unmodified code reached 20.87 m. **Nothing in the 2026-07-20 speed work causes
this** — the high samples are in transit, not in takeoff or climb-out.

## Mechanism (hypothesis, not yet confirmed)

PX4 re-captures HOME at every arming, and the vehicle arms once per sortie at
L&R. `DroneCommander._refresh_home_alt` re-caches the home MSL per arm so the
goto AGL→MSL conversion follows it. If the EKF's altitude estimate at the moment
of that capture is offset — a settling touchdown, baro drift over the 15-minute
window, no RTK — the whole sortie inherits that offset, which matches the
observed "constant offset per sortie, random between sorties" signature.

**Not yet verified.** Confirm before fixing: log the captured home MSL and the
EKF altitude at each arm, and correlate the per-sortie offset against them.

## Why the obvious fix is wrong

Lowering the commanded transit altitude (e.g. 19.5 → 19.0) trades a ceiling bust
for a floor bust: `verify_flight` checks the corridor from **both** sides
([18.8, 20.5] m), and run 1 already produced an 18.53 m sortie median at the
current command. With a ±0.9 m drift and a 1.7 m-wide safe band there is no
commanded altitude that is safe at both ends — the drift itself has to shrink.

## Candidate directions

1. **Re-reference altitude per sortie in flight** — after the transit altitude is
   captured, compare commanded vs observed AGL and correct the offset, instead of
   trusting the arm-time home capture for the whole sortie.
2. **Use the rangefinder** (TF-Luna, fitted at G5) as the AGL reference near the
   ground so the home capture is not baro-only. SITL cannot validate this — local
   PX4 is v1.15 and gz has no distance sensor on `gz_x500`.
3. **Watchdog-side mitigation** — the in-flight ceiling watchdog already warns
   >20.5 m and RTHs >22 m. It could instead *correct* the commanded altitude down
   when it sees a sustained positive offset, turning a rules breach into a
   self-correcting trim.

Direction 1 is the only one testable in SITL today, so start there.

## Definition of done

Four consecutive 4-sortie SITL runs where every per-sortie median transit
altitude sits within ±0.4 m of the command and no run produces a sample above
20.5 m — plus the same check on the real bird at G7, where the drift may differ.
