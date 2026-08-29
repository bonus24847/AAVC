# FC microSD recheck + flight-log discovery loop — 2026-08-29 evening

Card: SanDisk Extreme 32 GB (installed 2026-08-21), mounted read-only on the laptop at 17:45.
Everything below was produced with `pyulog` from the card itself; nothing on the card was modified.

## 1. Card state
- `fault_*.log`: none · `param_import_fail.txt`: none · `dataman` 128528 B (12:56) · `parameters_backup.bson` 3262 B (13:07)
- 26 ULogs, 149 MB, 7 date folders (08-21 … 08-29). Newest: `2026-08-29/05_56_59.ulg` (the scored flight),
  `06_07_12.ulg`, `06_07_36.ulg` (the two re-arms). **Nothing flew after 13:07.**
- md5 card == `captures/ulog_2026-08-29/`: `05_56_59` caeabf9d843c… · `06_07_12` b63ed6ea71d1… ·
  `06_07_36` c7a57e38d2c4… · `parameters_backup.bson` 7ee43fa872e4… — all MATCH.
- The two 28-Aug ULogs were not on the laptop anywhere; now `captures/ulog_2026-08-28/` (`MD5SUMS`
  written from the copies, both MATCH the card).

## 2. Board parameters as flown on 29 Aug (initial params of `05_56_59`)
`BAT_LOW_THR .25 · BAT_CRIT_THR .15 · BAT_EMERGEN_THR .07 · COM_LOW_BAT_ACT 3` · `BAT1_CAPACITY -1 ·
V_EMPTY 3.77 · V_CHARGED 4.18 · N_CELLS 6` · `RTL_RETURN_ALT 25 · RTL_DESCEND_ALT 10 · GF_ACTION 3 ·
GF_MAX_VER_DIST 50` · `EKF2_HGT_REF 0 · EKF2_GPS_CTRL 7 · EKF2_RNG_CTRL 1 · EKF2_RNG_A_HMAX 7` ·
`MAV_0_CONFIG 101 (rate 1200) · MAV_1_CONFIG 102 (rate 0, mode 2) · MAV_1_FORWARD 1` · `MPC_XY_CRUISE 5 ·
MPC_LAND_SPEED .3 · MPC_LAND_ALT1/2 4/2 · MPC_Z_V_AUTO_DN .4 · MPC_THR_HOVER .58 · MPC_THR_MIN .12 ·
MPC_XY_VEL_I_ACC .8 · MPC_TILTMAX_AIR 30` · `LNDMC_ALT_GND 2 · LNDMC_Z_VEL_MAX .25 · LNDMC_XY_VEL_MAX 1.5 ·
LNDMC_ROT_MAX 20 · LNDMC_TRIG_TIME 1` · `COM_DISARM_LAND -1 · COM_DISARM_PRFLT 10 · COM_OBL_RC_ACT 0 ·
COM_OF_LOSS_T 1 · COM_RCL_EXCEPT 4 · NAV_RCL_ACT 2 · NAV_DLL_ACT 2 · COM_DL_LOSS_T 5 · COM_KILL_DISARM 5 ·
COM_MOT_TEST_EN 1` · `SYS_AUTOSTART 6001 · PWM_MAIN_FUNC1 101 · PWM_AUX_FUNC1/4 301/304 · SENS_TFMINI_CFG 103`.

## 3. The scored flight (349 s) — battery / failsafe / offboard
- Battery 100 → 31 % (min), 25.10 → 22.80 V min, 3563 mAh counted, 35.8 A mean, 64 A peak; < 40 % at t=290.
  No battery warning level ever set (LOW is 25 %). The planned 30 % egress floor was ~1 min away at the kill.
- `offboard_control_mode` published t=0.0–4.2 s only (the RC-GO prime); nav: POSCTL → OFFBOARD (t=1.4) →
  AUTO_TAKEOFF (3.3) → HOLD (23.7) → LAND (108.4) → TAKEOFF (121.3) → HOLD (138.8 … kill).
- Re-arm logs (13:07:12, 13:07:36): armed by RC switch, `Switching to Offboard is currently not available`
  at t=1.1 s, NO `offboard_control_mode` topic at all, `Disarmed by auto preflight disarming` at 11 s.
  Beacon carried `stale=266/289`, `why=pilot`. `MPC_XY_CRUISE` read 3.5 on both (the sweep's live pin
  never restored because the process died at the kill; `apply_param_overrides` re-pins 5.0 at start).

## 4. The tip-over, from the land-detector flags (t=255–345, 1 Hz)
| t | lidar | thrust sp | pitch sp / act | motors | xy err | traj vz | in_descend | has_low_thr | ground_contact |
|---|---|---|---|---|---|---|---|---|---|
| 259–263 | 1.70 → 0.00 | 0.56–0.66 | ≈ −3° | 1520/1770 | 0.07–0.19 | +0.2…+0.5 | 1 | 0 | 0 |
| 264–276 | 0.02 (on the grass) | 0.12 | +2…+5° / −2.6° | **1100/1292** | 0.15–0.26 | 0.0–0.06 | **0** | **1** | **0** |
| 277–285 | climb-out | 0.47–0.72 | −10.5° / +15.6° (jolt) | 1483/1830 | 0.30 | −1.0 | 0 | 0 | 0 |
| 306–318 | 1.9 (hover) | 0.62–0.65 | ≈ −4° | 1510/1710 | ≤ 0.12 | 0 | 0 | 0 | 0 |
| 319–323 | 1.61 → 0.00 | 0.59–0.64 | | | | +0.2…+0.5 | 1 | 0 | 0 |
| 324–337 | 0.00 (on the grass) | 0.53 → 0.12 | **+4 → +21.7°** / +1° | 1100/1292 | **0.10 → 1.38** | ≈ 0 | **0** | **1** | **0** |
| 338–340 | 0.00 | 0.30 → 0.87 → 1.00 | +11…+15° / **+18 → +53.3°** | 1100/1900 | 1.30 | −1.0 | 0 | 0 | 0 |
| 342.9 | | | kill engaged | | | | | | |
`maybe_landed`/`landed` were 0 throughout; `vertical/horizontal/rotational_movement` 0 on the ground;
`close_to_ground_or_skipped_check` 1. Allocator unallocated torque reached 0.41 roll / 0.53 pitch, thrust 0.50.
Reading: PX4's multicopter land detector in a climb-rate-controlled mode accepts "hit ground" only while the
trajectory commands a descent (`in_descend`); the trajectory setpoint had reached its (under-ground) target
and its vz was 0, so the detector could not latch in HOLD. The lateral setpoint that fed the integrator came
from the ladder's lost-pad retry gotos aimed at the pad's estimated xy (`best_latlon`), 1.4 m from the aircraft.

## 5. Per-flight health, the nine real flights since 26 Aug
| flight | armed s | GPS alt walk / baro walk (m) | home.alt rewrites (worst) | frame bias vs TFmini (median, 1–9 m) | lidar valid % / 0.00 readings | accel-z std (m/s²) / p2p p95 | motors M1..M6 mean (spread) | xy p95 / z p95 (m) | batt min % | flags |
|---|---|---|---|---|---|---|---|---|---|---|
| 08-26 08_21_26 | 99 | −3.5 / +0.4 | 3 (+0.07) | +0.15 | 40 / 46 | 1.55 / 7.8 | .558 .584 .605 .537 .558 .584 (.069) | 0.18 / 0.12 | 61 | local_position_invalid at kill |
| 08-26 09_24_10 | 165 | **−12.1** / −0.9 | 11 (**−13.1**) | −0.57 | 30 / 103 | 1.30 / 6.5 | .571 .614 .642 .543 .549 .635 (.099) | 0.20 / 0.15 | 39 | geofence_breached t=93 |
| 08-26 10_25_05 | 98 | −3.1 / −0.2 | 1 (−0.91) | −0.08 | 39 / 46 | 1.15 / 5.7 | .613 .587 .632 .568 .565 .635 (.070) | 0.20 / 0.13 | 21 | battery_warning t=75 |
| 08-27 05_44_48 | 355 | +1.2 / +1.6 | 1 (+0.88) | +1.58 (late +1.73, max 3.03) | 28 / 241 | 1.70 / 7.9 | .616 .627 .640 .603 .583 .659 (.077) | 0.17 / 0.16 | 23 | battery_warning t=351 |
| 08-27 07_13_36 | 157 | −0.9 / +0.8 | 1 (+0.62) | +0.12 | 23 / 105 | 1.45 / 7.2 | .601 .597 .631 .567 .581 .617 (.064) | 0.18 / 0.10 | 60 | — |
| 08-27 07_51_21 | 170 | −4.3 / +0.1 | 2 (−0.62) | −0.44 | 32 / 58 | 1.38 / 6.9 | .575 .619 .646 .548 .589 .605 (.098) | 0.15 / 0.11 | 41 | — |
| 08-27 08_25_52 | 252 | **−9.3** / 0.0 | 5 (−1.80) | −1.57 (late −1.58) | 47 / 86 | 1.54 / 7.6 | .616 .601 .595 .622 .554 .663 (.109) | 0.19 / 0.18 | 25 | — |
| 08-28 08_05_29 | 373 | **+8.9** / −0.6 | 8 (+0.47) | −0.59 (late −0.74) | 28 / 249 | 1.43 / 6.3 | .574 .647 .646 .574 .551 .670 (.119) | 0.18 / 0.14 | 16 | battery_warning t=325 |
| 08-28 10_28_22 | 311 | +0.2 / −1.8 | 6 (−1.23) | −0.73 | 29 / 186 | 1.37 / 7.1 | .566 .641 .637 .569 .565 .642 (.077) | 0.20 / 0.14 | 34 | — |
| 08-29 05_56_59 | 348 | **+9.2** / +0.3 | 3 (+3.54) | **+2.44** (late +2.57, max 8.99) | 40 / 180 | 1.62 / 7.4 | .570 .619 .628 .560 .533 .655 (.122) | 0.39 / 0.83 (ground episodes) | 31 | — |
EKF: `filter_fault_flags` 0 and no `estimator_event_flags` resets on every flight; innovation test ratios
hgt ≤ 0.30, pos ≤ 0.15, vel ≤ 0.29. GPS 13–16 sats, eph ≤ 1.95, epv ≤ 3.9, jamming indicator ≤ 49.
`accel_vibration_metric` in flight: mean 12–15, p95 15–18, max 16–23 (worst 24-Aug 27.8). CPU 40–44 % mean,
5 V rail ≥ 4.88 V. `local_position_invalid` appears only in the last 1–2 samples of kill-ended flights.

## 6. MAVLink commands (vehicle_command / vehicle_command_ack)
- Every companion (sysid 245 / comp 190) command ACCEPTED on every flight — 29 Aug: DO_REPOSITION ×117,
  NAV_TAKEOFF ×2, NAV_LAND ×1, DO_SET_MODE ×2, DO_SET_ACTUATOR ×1 (p4 +0.8 at t=120.4).
- `ACTUATOR_TEST` (310) ×8 on 27-Aug 05_44_48 and 28-Aug 08_05_29: from sysid 255 (the console), at
  t = −225…−223 s and −155…−153 s = BEFORE arming (latch check, functions 1301–1304, 0.8 then 0.0);
  AUX1–4 (`actuator_outputs` instance 1) min = max = 1000 µs over both whole flights → no in-flight release.
- `SET_MESSAGE_INTERVAL` acks with result 4 (FAILED), one per burst every ~30 s, all addressed to
  255/0 = the console's radio `_request_streams` (12 silence requests + 10 rate requests per burst);
  the request is not in `vehicle_command` (handled inside the MAVLink module), the failed one is a
  message id with no stream class (PING/4 by burst position). Companion (245/190) acks: all 0.
- Commands 1000/1001 from sysid 1/1 (the FC itself) at each land/takeoff: MAV_CMD_DO_GIMBAL_MANAGER_*
  chatter with no gimbal (`MNT_MODE_IN -1`); harmless.

## 7. Deploy state
`cm4/deploy.sh drone@10.42.0.1 --check --repo ~/Desktop/aavc-comp` → MD5 MATCH (aavc-comp 1334bfb =
practice fc87e30) at 17:50; every §0e file byte-identical on the CM4; `~/mission/.aavc_site` = competition.
CM4 clock read 25 Aug (4.6 days slow; no RTC, NTP unsynced) — audit `ts` fields are wrong, `t=` is right.
FC unpowered at 19:20 (`preflight_params.py` could not find a vehicle) — the bench items wait for the pack.
