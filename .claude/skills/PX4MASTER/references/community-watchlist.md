# Community watchlist — known problems we likely carry (mined 2026-08-20)

Three research sweeps (PX4 GitHub/releases · discuss.px4.io · hardware-specific),
every finding mapped to THIS aircraft. These are problems OTHER people already
hit with our configuration — found before they found us. Merged top actions
first; per-domain digests with links after.

## Ranked actions (bench/field, before the competition)

1. **RC-loss drill + ELRS failsafe mode** (2 min, catastrophic if wrong,
   never verified): props off, armed on the bench, ELRS TX OFF → PX4 must
   announce manual-control lost within ~1 s (`COM_RC_LOSS_T`) and run
   `NAV_RCL_ACT`. Verify the RX failsafe is **No Pulses** (a "Last Pos" RX
   feeds fake sticks forever and PX4's RC-loss NEVER fires). Also record
   which path the RX actually uses on the flight fw — SBUS-into-RCIN or CRSF
   (stock fmu-v6x has no crsf_rc driver; this is undocumented in-repo).
   Re-run after any ELRS firmware/bind/model change; a model-match mismatch
   = "bound + telemetry but dead sticks".
2. **Replace the FC's microSD** — the 2026-08-20 dataman wedge is a known
   SD/FAT-stall class, AND all of PX4 1.17 carries an unfixed **silent SDMMC
   READ-corruption bug on STM32H7** (NuttX#389/PX4#28071 — our exact chip;
   our two historical BSON param-import failures are its textbook output).
   Buy 2× **SanDisk Extreme U3 32 GB** (PX4's named card; ≤32 GB, fresh
   FAT32), `sd_bench -r 50` old vs new over nsh, archive the old card as
   evidence (do NOT rehabilitate), regenerate `parameters_backup.bson` on
   the new card and verify it holds the full param set.
   **Until v1.17.1: distrust MAVFTP file reads** — pull the card physically
   for ULogs, or download twice + hash-compare; restore params only from the
   laptop-side backup copy.
3. **ESC low-voltage cutoff vs the semi-solid pack**: Li-ion-class chemistry
   sits far below LiPo voltages near empty; an ESC with a LiPo-style LVC
   **cuts motors in the air with charge remaining**, and with no current
   sensing we get zero warning. Check/disable every ESC's LVC (NiMH setting
   if it can't be disabled) BEFORE the 17 Ah pack flies missions.
4. **CM4-WiFi-vs-GPS EMI A/B**: a documented field case fixed GPS jumps
   (9→17 sats) with shielding — our CM4 is a 2.4 GHz AP *transmitting
   continuously in flight* centimetres from the GPS, plus the NOMAD UHF.
   Bench with sky view: log `gps_status.jamming_indicator`/`noise_per_ms` +
   sat count, AP off vs on (NOMAD keyed too). The 24 m vertical walk may be
   partly self-inflicted.
5. **Measure the ArUco decode floor**: at 74.2° FOV the 400 mm marker fills
   the frame near ~0.35 m — the last metre is open-loop by design. Bench:
   camera over a printed pad on a tape measure, log decode hit/miss vs
   height; then confirm the mission's last id-verified fix sits above that
   floor and quantify drift from there to touchdown. That number IS the 1 m
   pad margin.
6. **Powerbank idle-cut test (the CM4 "spontaneous reboot")**: most banks cut
   the port under ~50-100 mA for 15-60 s — an idle CM4 dips below, the bank
   sleeps, the CM4 cold-boots "by itself" (matches 2026-08-20). Test: CM4
   idle 45 min on the exact competition bank. Fix: KeepAlive dongle /
   trickle-mode bank / BEC (power-architecture change = operator decision).
   Check `vcgencmd get_throttled` after each session.
7. **Re-seed `MPC_THR_HOVER` from the first 17 Ah-pack hover ULog** — it is
   the HTE seed at every arm and the land-detector's reference; too-low =
   false ground-contact during descents + the post-param-wipe ceiling climb
   we already logged.
8. **One instrumented discharge before trusting the pack**: timed hover to
   the 30 % (sagged) floor → land → charger charge-back mAh = how much truly
   remains at the floor; place `BAT1_V_EMPTY` from OUR curve, not LiPo
   folklore. (No sag-compensation param exists on 1.16+ without current —
   `BAT1_V_LOAD_DROP` was removed; stop looking.)
9. **Session hygiene on v1.17.0** (all fixed after our build): geofence
   uploads ON THE GROUND only (navigator SKIPS fence violation checks while
   an upload is in progress); **no USB plug-ins / new MAVLink instances once
   a session is live** (Mavlink::start() re-inits the shared command-ACK
   semaphore → corrupted command tracking); if command ACKs go weird after
   any link change → reboot FC before GO. Upgrade when v1.17.1 exists (and
   confirm it picked up NuttX#389 first).
10. **Next-field-day ULog checklist** (adds to the discovery loop):
    baro-vs-GPS overlay during transit acceleration (prop-wash/airflow
    offset); baro dip in the last metre of each landing → set
    `EKF2_GND_EFF_DZ` = dip+10 % (`EKF2_GND_MAX_HGT` ≥ the TFmini-blind
    0.4 m); FFT for the 18"-prop-pass peak vs the 15-40 Hz control band
    (hard-mount beat soft-mount in the reference case); ONE clean
    baro→range height-source handover per descent (no flapping); yaw
    setpoint-vs-actual through P1→P3 heading changes; hover actuator
    outputs < ~60 %.

## Shopping list
2× SanDisk Extreme U3 32 GB (FC) · powerbank KeepAlive dongle or trickle-mode
bank · official CM4 antenna kit (+`dtparam=ant2`, verify by measured signal,
not config) · (optional) High-Endurance card for the CM4 boot disk.

## Field-day sheet additions
- GO-gate glance: relative alt reads ~0 (±0.5 m) on the pad; `eph`/`epv`
  sane (own thresholds, e.g. wait above eph 3 m / epv 5 m) — PX4's GPS
  checks only guard the FIRST fix, in-air trust is ours (relaxed post-fix
  checks are deliberate upstream).
- After any radio model-profile change: stick-wiggle seen in the GCS RC view
  before the pack goes on.
- Memorize: PM02D reading exactly **40.96 V = driver/variant mismatch**, not
  a battery problem. Bench once: `ina228 status` to record which chip our
  unit carries; leave dividers at default (±5 % spec is why the multimeter
  cross-check stays mandatory).
- TFmini truth over grass: usable range outdoors is ~6-7 m, not 12 (datasheet
  is at 90 % reflectivity). The delivery descent ends over the WHITE pad =
  best case. Open decision: `EKF2_RNG_A_HMAX` 7→6 (7 was chosen to clear the
  5 m rung; 6 still clears it — operator's call, revalidate in SITL first).

## Validated NON-actions (community confirms our locked decisions)
- Do NOT migrate to PX4's built-in precision landing (a directly-compared
  rig drifted away on PX4 while landing fine on ArduPilot; unresolved).
- Keep `MC_YAWRATE_MAX=45` — a 30"-prop case independently landed on 40°/s.
- Keep the per-arm AGL→MSL conversion (`_refresh_home_alt`): DO_REPOSITION
  altitude is ALWAYS treated as AMSL upstream (#10246, wontfix-old).
- Keep `EKF2_HGT_REF` where it is per site; never flip mid-campaign — with
  baro ref + range aid, altitude stays BARO-ANCHORED (the lidar aligns to
  the baro frame; #26226) — the vision gates own the final metres.
- A single-sample altitude jump must keep being ridden out by the watchdog's
  sustain window: upstream has an unresolved baro-spike→40 m-EKF-reset
  report on exactly our config. Any in-flight height RESET in a ULog is a
  no-GO until understood.
- `SENS_BAR_AUTOCAL` (new in 1.17, default on) is inert at baro ref but goes
  LIVE if anyone flips to GPS height ref — decide its value deliberately at
  that moment (tracked in preflight's informational list).
- Geofence breach checking is PREDICTIVE (trajectory-based) and cannot be
  disabled: keep corridor-to-fence margin generous at the event-briefing
  re-measure; after any jerk/accel retune, fly one SITL pass along the
  fence edge.
- PX4 autotune on v1.17.0 cannot re-trigger without a reboot (bench
  annoyance; reboot between autotune sessions).

Full source links live in the three research digests (session 2026-08-20);
key ones: PX4 releases + v1.17 notes, PX4 #28071/#27593/#27343/#27124/#26226/
#24436/#17605/#10246, NuttX #389, discuss.px4.io threads 7773/24096/48562/
24555/8908/45940/46207/47362/47333/44947/35897, ExpressLRS #2912 + PWM-RX
docs, PX4 SD/logging + dataman + land-detector + battery docs, Holybro PM02D
docs, ArduPilot TFmini + Li-ion-LVC threads.
