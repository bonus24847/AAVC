# ULog + frame review — the 2026-08-20 field session's black boxes

First run of the PX4MASTER discovery loop on REAL data. Source: the old FC
microSD, read PHYSICALLY in the laptop (per the 1.17 MAVFTP-untrusted rule),
archived whole at `~/aavc_sdcard_archive_2026-08-21/` (283 MB, 39 log
sessions, 2025-04-09 → 2026-08-20). Small evidence files copied to
`docs/evidence/sdcard_old_2026-08-21/`.

**Clock correction first:** ULog names carry GPS (true) time — the session
ran 12:56–15:13 **local afternoon** of 2026-08-20. Every clock we watched
live (CM4, laptop terminal) was stale; the "night session" in earlier notes
is an artifact of that. Frames are daylight.

The three flights (GPS-timed, matched by armed-duration/profile):

| ULog | armed | = flight | ended by |
|---|---|---|---|
| 07_21_36.ulg | 42 s | 1 — GPS-frame sink into ground | pilot takeover |
| 07_46_55.ulg | 100 s | 2 — phantom ceiling | watchdog RTH |
| 08_11_09.ulg | 121 s | 3 — transit 3/3 on baro ref | battery-floor RTH @68 s |

## Findings

### 1. CAMERA IS THE NEW #1 BLOCKER — real frames cannot decode ArUco
402 real daylight frames from the CM4 (`~/aavc_cm4_runs/.../frames`):
**0/402 decode** — and a REAL printed paper marker WAS on the field (visible
in nadir_000070, ~55 px across, ~3 m AGL). Raw cv2.aruco across five
dictionaries: zero decodes anywhere; CLAHE/normalize/sharpen/Otsu on a 6×
upscaled crop: still zero — **the cell information is destroyed in the
image, not filtered out by our valid-ids gate.** The marker prints as washed
gray-on-white mush; high-altitude grass frames are uniformly soft.
Prime suspect: **M12 lens focus left at near field** — the fov calibration
(2026-08-17) focused on a target at 0.495 m and nothing since verifies far
focus. Secondary: sunlight washout on glossy paper, and the 60 Hz airframe
vibration on the hard-mounted camera (see #3; linear amplitude computes to
~7 µm — small, so focus stays prime).
**Bench discriminator (do BEFORE any mission flight):** printed marker at
3–8 m, aircraft parked, motors OFF → if soft, refocus the lens to far field
and lock it; repeat with motors idling to expose any vibration blur;
`make replay` on the captured frames = pass/fail. Without this fix the
mission flies perfectly and delivers zero eggs.

### 2. Battery sag — now measured per-flight (voltage-only gauge, PM02D era)
| flight | % start → min (under load) | V/cell min under load |
|---|---|---|
| 1 | 78 → 61 | 3.864 |
| 2 | 75 → 40 | 3.761 |
| 3 | 61 → 18 | 3.684 |
Flight 3 read **18 %** while resting SoC was ~60 %: the sag is 30–40 points
at this AUW, exactly as modeled — the 30 % floor RTH at t=68 s was the gauge
doing its (conservative) job on a part-charged pack. Operational rule stands:
start missions charged; do not lower floors.

### 3. Vibration: ~60 Hz spike, control band clean
`accel_vibration_metric` mean 17–18 m/s² in flights 2–3 (p95 ~27, max ~30) —
high in absolute terms. FFT (flight 3, accel Z): dominant peak **59.5–61.6 Hz
(~1 m/s² per bin)** = per-rev motor/prop frequency → prop balance / motor
bell / mount. Control band (<40 Hz) is clean (≤0.016 m/s²) and IMU cutoffs
(gyro 40 / accel 30 Hz) filter it — which matches the excellent transit
tracking (1.4–2.0 m passes). Action: prop balance check when convenient;
verify camera mount stiffness (interacts with #1); NOT a flight blocker.

### 4. MPC_THR_HOVER is at default 0.5; true hover measured ≈ 0.60
Motors mean while armed: 0.37 / 0.56 / 0.60 across the three flights
(p95 0.71–0.77; flight 3 touched 1.00 momentarily, flight 2 0.95 —
saturation HEADROOM IS THIN at this AUW; keep speed/jerk caps as tuned).
Board holds `MPC_THR_HOVER=0.5` (default) with `MPC_USE_HTE=1`, and the HTE
topic logged all-NaN (did not converge/publish in these short flights) — so
the seed matters. **Proposal for operator approval: `MPC_THR_HOVER` 0.5 →
0.58** (just under measured mean; improves takeoff ramp, land-detector
margins, and post-reset first-flight behaviour). Not written — operator
decision.

### 5. Height frames: the story confirmed in the raw data
- **Zero EKF z/xy resets in all three flights** — the flight-2 altitude
  "steps" were the fused estimate slewing after GPS, not discrete resets.
- baro-vs-GPS divergence (detrended p2p): flight 1 **2.5 m**, flight 2
  **10.8 m** (GPS height ref era — the phantom-ceiling flight), flight 3
  **5.8 m** (baro ref: fused height stayed with baro while GPS wandered —
  by design). The baro-ref decision is validated by the boxes.
- TFmini `dist_bottom` returned plausible ranges up to **10 m over daytime
  grass** — better than the community's 6–7 m expectation; aiding stays
  gated at 7 m regardless.

### 6. Old SD card retired
7.4 GB no-name-class card (not the recommended class to begin with), carries
the param_import_fail history + an `APM/` dir from a previous life. Archived
whole; final state preserved. Replacement: SanDisk Extreme U3 32 GB
(purchased 2026-08-21), to be FAT32'd + `sd_bench`'d before install.

## Actions handed to the bench (today)
1. Camera focus test + refocus (finding 1) — gate for ANY ArUco flight.
2. `MPC_THR_HOVER` 0.5→0.58 — awaiting operator approval.
3. New-card install per plan (f3 → FAT32 → FC → preflight → sd_bench →
   param export → fence-probe).
4. Prop balance + camera-mount stiffness check (finding 3, non-blocking).

---

## ADDENDUM (same day, bench walk test) — finding 1 RESOLVED and RE-SCOPED

Static decode gate run 2026-08-21 with a REAL 38 cm printed marker (id 2),
aircraft on its side, a person walking the marker out (live loop:
docs/evidence/walk_test_decode_2026-08-21.log):

| range | result |
|---|---|
| 1.9 m (174 px) → ~14 m (23 px) | **continuous decode, every frame** |
| ~14 → ~20+ m (15-18 px) | intermittent decode (ROI booster working) |
| walk back in | seamless re-acquire at every step |

Sharpness sat at 680-780 the whole test **with no lens adjustment at all** —
the M12 focus was never wrong for 2-20 m. The mission's sweep band (8-12 m)
carries ~2× decode margin with the competition-size marker.

**Therefore the field-frame mush is NOT static focus.** Revised diagnosis:
**in-flight blur** — the ~60 Hz per-rev vibration on the hard-mounted camera
and/or exposure during translation. The camera gate for the next flight is
re-scoped: hover the aircraft over/near the printed marker and confirm live
decode DURING flight (the orchestrator's own vision worker + dashboard chip,
or a hover with frame_recorder and `make replay` after). Mitigations queued:
prop balance (the 60 Hz peak), camera mount stiffness check, and if needed a
forced short exposure on the OV9281.
