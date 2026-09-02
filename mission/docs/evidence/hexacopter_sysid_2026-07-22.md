# EFT X6100 — System-ID sweep + gain design (2026-07-22)

Offline pre-flight tuning aid, run in SITL against `gz_eft_x6100`. Not part of a
scored sortie.

## Measured plant (`runs/sysid/hexacopter_latest.json`, ULog `07_07_09.ulg`)

Attitude chirp 0.6 → 13 Hz, 25 s per axis, held at 15 m.

| axis | b measured (rad/s² per unit cmd) | b from the model | meas/model |
|---|---|---|---|
| roll | 101.73 | 99.43 | 1.02 |
| pitch | 98.20 | 86.11 | 1.14 |
| yaw | 4.98 | 6.00 | 0.83 |

The roll prediction is within 2 %, which validates both the X6100 mass/inertia
estimates and the new hexa-X couple factors in `tuning/plant.py`. Pitch measured
14 % *higher* than the geometric model — PX4 normalises control allocation per
axis, so the 2.0 : √3 roll/pitch authority ratio the geometry implies is largely
flattened by the time it reaches the rate loop. Worth remembering before trusting
the pitch factor for anything load-bearing.

## Why the designed gains were not applied verbatim

The model-based design at the default 6 Hz rate bandwidth returned
`MC_YAWRATE_P = 10.6` and `MC_YAW_P = 9.42`. Those follow correctly from
`b_yaw = 4.98` — a weak axis needs a big gain to hit a given bandwidth — but they
are not flyable: at that gain the normalised yaw torque saturates on a rate error
of about 5 °/s.

Calibration against the **validated quad set** (`quadcopter_gains.json`, which
flew G4 repeatedly) makes the trade explicit:

| axis | quad P | quad b | quad closed-loop f_c | hexa b | P for the same f_c |
|---|---|---|---|---|---|
| roll | 0.447 | 118.0 | 8.4 Hz | 101.7 | 0.519 |
| pitch | 0.311 | 169.9 | 8.4 Hz | 98.2 | 0.537 |
| yaw | 0.528 | 13.8 | 1.16 Hz | 4.98 | 1.466 |

Roll and pitch: the 6 Hz design lands on 0.519 / 0.537 — i.e. it reproduces the
closed-loop speed the quad already proved. Applied as designed.

Yaw: matching the quad's 1.16 Hz would need P = 1.47, which saturates at a 39 °/s
rate error against an `MC_YAWRATE_MAX` of 45 °/s — no margin at all (the quad had
2.2×). **P was set by hand to 0.80** (saturates at 72 °/s, 1.6× margin) and
`MC_YAWRATE_I` capped at 0.40, mirroring the hand-softening the quad's yaw got.
`MC_YAW_P` kept at 3.5 rather than the designed 9.42.

Only `MC_*` gains are persisted. The design also proposes `MPC_XY_*` / `MPC_Z_*`
values, but those would fight the wind-rejection set that `px4_tuning` pins for
landing precision, so they are deliberately excluded — the same choice the quad's
gains file made.

## Applied set (`runs/sysid/hexacopter_gains.json`)

```
MC_ROLLRATE_P  0.5188   MC_PITCHRATE_P  0.5375   MC_YAWRATE_P  0.80
MC_ROLLRATE_I  0.7783   MC_PITCHRATE_I  0.8062   MC_YAWRATE_I  0.40
MC_ROLLRATE_D  0.0156   MC_PITCHRATE_D  0.0161   MC_YAWRATE_D  0.0
MC_ROLL_P      9.4248   MC_PITCH_P      9.4248   MC_YAW_P      3.5
```

`orchestrator/main.py` loads this at the start of every mission, so a SITL boot
(which resets FC params) still flies tuned.

## Validation: tuned vs untuned, same 4-sortie G4 mission

| metric | baseline (PX4 defaults) | tuned | delta |
|---|---|---|---|
| release vs truth, mean | 0.145 m | 0.170 m | +0.025 |
| release vs truth, max | 0.23 m | 0.24 m | +0.01 |
| align final error, mean | 0.147 m | 0.160 m | +0.013 |
| window | 885 s | 891 s | +6 s |
| max altitude | 19.75 m | 19.80 m | +0.05 |
| `verify_flight` | 19 ok / 0 warn | 19 ok / 0 warn | — |

**The tuned gains are neutral in SITL.** Every difference is smaller than the
run-to-run scatter of four samples (the baseline's 0.02 m outlier alone moves its
mean by 0.03), and both runs pass every check. That is the expected result rather
than a disappointment: delivery accuracy here is set by the vision/align loop and
PX4's AUTO position controller, not by the inner rate loop. The rate loop earns
its keep in disturbance rejection, and SITL's steady 3 m/s wind barely exercises
that.

**Kept anyway, with a condition.** The tuned set is derived from this aircraft's
measured plant, where PX4's defaults are generic small-quad numbers, so it is the
better starting point for the real bird. But it is 3.5x the default rate P — if
the real airframe's authority is lower than the sim's (likely: the model's rotor
lag is idealised), those gains can oscillate. **Before any autonomous flight on
hardware, re-run the sweep on the REAL aircraft (or PX4's built-in autotune) at
G6 on the tether and re-derive.** Do not carry the SITL numbers into a free
flight unvalidated.

Evidence: `G4prime_hexacopter_2026-07-22.txt` (baseline) and
`G4prime_hexacopter_tuned_2026-07-22.txt` (tuned).


---

# RE-RUN on the corrected model (same day, later)

The sweep above was flown against a model built from guesses. After
`Power-System-Guide-1.pdf` supplied the real geometry (arm 0.500 m, AUW 7.17 kg,
18" props, 37.65 N per motor) the model was rebuilt and the sweep repeated. The
first gain set was deleted rather than kept — gains identified on the wrong
aircraft are worse than no gains, because `orchestrator/main.py` applies them
automatically at every mission start.

## Measured plant, corrected model (ULog `10_31_18.ulg`)

| axis | b measured | b from the model | meas/model | previous (wrong model) |
|---|---|---|---|---|
| roll | 118.01 | 110.74 | 1.07 | 101.73 |
| pitch | 98.02 | 95.90 | 1.02 | 98.20 |
| yaw | 11.38 | 7.06 | 1.61 | 4.98 |

Roll and pitch now agree with the analytic model to within 7 % and 2 % — the
geometry, the mass budget and the hexa-X couple factors all check out against a
measurement none of them was fitted to.

**Yaw authority more than doubled** (4.98 → 11.38). That is the single most
consequential change: on the old model, matching the quad's validated yaw
response needed `MC_YAWRATE_P` 1.47, which saturated the normalised yaw torque
at a 39 °/s error against a 45 °/s limit — no margin at all. At b_yaw 11.38 the
same response costs P 0.64 with a 2.0× margin. The earlier hand de-rating was
compensating for a modelling error, not a real property of the aircraft.

## Applied set (`runs/sysid/hexacopter_gains.json`)

```
MC_ROLLRATE_P  0.4472   MC_PITCHRATE_P  0.5384   MC_YAWRATE_P  0.77
MC_ROLLRATE_I  0.6709   MC_PITCHRATE_I  0.8077   MC_YAWRATE_I  0.40
MC_ROLLRATE_D  0.0134   MC_PITCHRATE_D  0.0162   MC_YAWRATE_D  0.0
MC_ROLL_P      9.4248   MC_PITCH_P      9.4248   MC_YAW_P      3.5
```

Roll and pitch are the design output unmodified. `MC_ROLLRATE_P` landing on
0.4472 is worth noting: the validated quad set used 0.4475 on a completely
different airframe, which is what you would expect if both designs are tracking
the same closed-loop bandwidth rather than the same hardware.

Yaw was backed off from the design's 0.928 (1.2 Hz) to **0.77**, giving a 1.65×
saturation margin against `MC_YAWRATE_MAX` 45 °/s, with I capped at 0.40 and
`MC_YAW_P` kept at the quad's validated 3.5. `MPC_*` gains are still excluded so
they cannot overwrite the pinned wind-rejection set.

**Still to do on hardware:** re-derive on the real aircraft at G6. The plant here
is a simulation whose rotor lag is idealised; nothing above substitutes for a
sweep on the real airframe.
