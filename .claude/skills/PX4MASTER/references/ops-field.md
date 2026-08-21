# Field operations — procedures, verifier discipline, open items

The step-by-step field-day sequence lives in SKILL.md. This file holds the
supporting procedures and the open ledger.

## Post-flight discovery loop (how the UNKNOWN bugs get found)
The 2026-08-20 lesson: GPS-height jitter was invisible until real flights —
sim, bench and review all missed it. The loop that surfaces the next one:
1. `make verify RUN=…` on the CM4's audit copy (behaviour vs rules).
2. **Open the black box**: pull the flight's ULog from the FC SD and run PX4
   Flight Review (logs.px4.io, or `pip install flight_review` locally) —
   it checks things nobody thought to ask: vibration (>2-3 m/s² peak-to-peak
   is bad), actuator saturation (watch this at the 17000-pack AUW), thrust
   margin, EKF innovations/resets, setpoint-vs-estimate tracking, FFT peaks.
   `tools/verify_flight.py --ulog` gives the scripted subset (needs pyulog).
3. Search the symptom on discuss.px4.io + the PX4 GitHub issues BEFORE
   debugging from scratch — most of what this aircraft will hit was hit by
   someone in 2019.
4. 15 minutes, whole crew: "what surprised us today?" → every surprise
   becomes a references/ entry or an open item HERE (the new-bug rule).

## RC-GO conops (proven live, incl. two real takeovers 2026-08-20)
Web/console = STAGE ONLY. The pilot releases: ARM (in POSCTL, throttle low)
→ flip OFFBOARD within a few seconds (ground auto-disarm timer) → aircraft
launches itself. Once airborne: throttle stick to CENTRE so a takeover
doesn't step the altitude. **POSCTL flip = pilot owns the aircraft, any
time**; since 2026-08-21 the takeover is enforced at BOTH layers — the
watchdog detectors (sustained pilot mode, armed or not; disarm in a flying
phase) set the terminal + `stand_down()`, and `DroneCommander` then refuses
every FC-state-changing call. Field rule stays: after ANY takeover, stop the
mission (⏹) BEFORE approaching the aircraft. Between attempts: re-open the
launcher icon (clear_state runs first, by design).

## Battery ops
- UNPLUG the pack between sessions (0.6-0.76 A parked; 5 h ≈ 3.8 Ah).
- Start missions CHARGED — the under-load sag (power-battery.md) is what ends
  flights on a part-charged pack, not the mAh budget.
- Log every charge + rest voltages per the ground-truth table
  (power-battery.md) — the seeds get re-derived from that data.

## SD crash-dump procedure
fence-probe flags `fault_*.log` → pull it over MAVFTP, READ it, archive to
docs/evidence/ with date + firmware, THEN delete and reboot, then
`make preflight` again (a reboot after SD work is when param-import surprises
appear — fc-params.md).

## ArUco real-world proof (the missing evidence, in order)
1. **Bench**: print a 1×1 m pad (or the 400 mm marker at minimum,
   `tools/gen_pads.py` renders the exact artwork), real camera on the parked
   aircraft held/propped >0.4 m from it, `make camera-real`, then
   `make replay DIR=<single frame>` — proves sensor+optics+detector.
2. **Daytime flight over the printed pad** at the practice field — the real
   decode-at-altitude proof (folded into G7; the OV9281 is mono — night
   frames are unusable, as 2026-08-20 confirmed).
3. Every real flight already records frames on the CM4
   (`runs/<id>/frames/`); pull with rsync when in WiFi range and run
   `make replay DIR=…`. SITL pipeline baseline: 127/670 frames, ids 1-6.

## Post-flight verifier discipline (tools/verify_flight.py)
- It fails CLOSED (drops pre-GO TELEM, FAILs a release with no truth,
  cross-checks L&R vs config) — never "simplify" those into silent skips.
- Audit grammar changes move emitter + parser + docstring + golden tests in
  ONE commit (batt/vbat, 2026-08-20, is the template — optional group kept
  old archives verifiable).
- A regex that matches nothing turns the tool into a rubber stamp: every
  grammar regex needs a golden-line test against a REAL emitted line.
- Transient ceiling pokes are WARN; sustained or >ceiling+2 FAIL — matches
  the in-flight watchdog; don't tighten one without the other.

## Two-repo reality (practice ↔ aavc-comp)
Same aircraft, two repos. `sync_core.sh` does NOT cover `tools/` — the BOARD
param truth can drift (a live `BAT1_V_CHARGED` 4.15-vs-4.05 conflict already
happened; resolved to 4.18 on-board). Whoever runs preflight last overwrites:
before a shared field day, diff `tools/preflight_params.py::BOARD` across the
repos. Approvals do not cross sessions — send data+reasoning+proposal, the
other repo's operator decides.

## Open items / follow-ups (updated 2026-08-21 — G7 attempt #1 debrief)
- [x] 🔴 **Zombie mission RE-ARMED the aircraft after pilot takeover —
      FIXED 2026-08-21** (commit on `fix/safety-review-2026-08-19`).
      Symptom: no `PILOT TAKEOVER` audit line in either incident; ~8 min
      after takeover+disarm the loop called arm_and_takeoff on the parked
      aircraft. Mechanism (ULog-proven): the pilot flips POSCTL then DISARMS
      in **0.46-0.48 s** — under the detector's 1.0 s debounce at a 0.5 s
      tick — and `safety.py`'s `if not t.is_armed: return` sat ABOVE the
      takeover check, so the disarm blinded the watchdog permanently;
      `stand_down()` never ran; `mission.py`'s egress climb-out read
      "not armed" as "needs arming". Fix: takeover checks moved ABOVE the
      armed gate (disarmed evaluation gated to flying phases so boot/
      between-flight POSCTL postures don't false-fire), a NEW
      disarm-in-flight-phase detector (immediate, no debounce), a fire-once
      latch, and `_guard_pilot` extended to `run_mission`/`_arm_with_retry`
      (per attempt)/`arm_and_takeoff`'s tail/`_drop_via_set_actuator`/every
      param+failsafe setter. Caught-by: `tests/test_safety.py` (disarm/boot/
      between-flight/fire-once cases) + `tests/test_commands.py`
      (stand_down coverage table). **HARDWARE DRILL PASSED 2026-08-21**
      (props off, nothing armed): D1 fired at the 1.0 s debounce on the LIVE
      FC mode string — MAVSDK 3.17.2 reports "POSCTL" verbatim, killing
      hypothesis (a) with hardware — D2 fired on the first tick in both
      SEARCH and TRANSIT_INGRESS, and the stood-down commander refused
      goto/param/drop/rth/arm/run_mission while READS still worked (a
      selective refusal, not a dead link). Log:
      `docs/evidence/G7_takeover_drill_2026-08-21.txt`. Still owed: the same
      takeover DURING flight — blocked on the RC re-acquire item below.
      Field rule stays: **after ANY takeover, stop the mission (⏹) BEFORE
      approaching the aircraft.**
- [ ] Dashboard raw dispatches BYPASS the pilot guard (follow-up from the
      2026-08-21 review, deliberately not folded into the fix): web
      `/api/cmd/takeoff`, `/vehicle_arm`, `/resume`, `/hold` call
      `commander.system.action.*` directly (dashboard/commands.py:299-390),
      so a post-takeover operator click can still arm/fly a pilot-owned
      aircraft. `/vehicle_disarm` + `/kill` must STAY unguarded (legitimate
      post-takeover safing).
- [x] **GCS shows pads AS THEY ARE FOUND — DONE 2026-08-21** (mission repo
      d3f01cd + aavc-gcs 9e34b4f). Symptom: operator pulled flight 1 down
      while ids 4,5 were being identified live behind a blank console.
      Mechanism: in-flight WiFi death killed the mission_status sync AND the
      beacon carried CONFIRMED pads only. Fix: `pads_identified` {id:[e,n]}
      lane in mission_status.json (tracker.identified_unconfirmed, write-on-
      change), beacon line `AAVC seen=4,5`, GCS orange `.padbox.ident` +
      revived 4-state per-pad ladder (renderPadLive was dead code). Caught-by:
      `tests/test_gcs_status.py` promotion/no-rewrite pins +
      `tests/test_status_beacon.py` seen-line goldens. MAV_1_FORWARD verified
      ACTIVE live this session (radio_mission/cam/stale/prog fresh on the
      console API over the CP2102 radio). Console restart applies it.
- [x] **Plan path + stop order on the console map — DONE 2026-08-21**
      (mission repo + aavc-gcs 525853c; no prior open item — the G7 takeover
      root: operator could not see "where is it going NEXT"). Fix:
      `_plan_pusher` now also writes `plan` [[lat,lon,kind,seq],…] +
      `plan_ptr` into mission_status.json at every rebuild (first write at
      gate release, WiFi still up); console draws a purple dashed polyline +
      numbered stops, kept on screen when the feed goes stale. WiFi-only by
      design (too big for the radio). Caught-by:
      `tests/test_gcs_status.py::test_plan_pusher_writes_the_console_map_path`.
- [ ] G7 attempt #1 analysis pending: why sweep legs stalled from wp7
      (wind? speed?), why only 2/6 markers decoded (coverage vs camera) —
      ULog from the FC card + frames from the CM4.
- [ ] 🔴 **RC does not RE-ACQUIRE after a TX power-cycle** (2026-08-21 bench,
      undiagnosed — operator parked it): link was available=True 100%, TX
      off→on, FC stayed available=False. Candidates: TX boot warning screen
      holding RF off, ELRS model-match, RX in bind mode after today's rapid
      battery cycles. **MUST be resolved + the RC-loss drill completed before
      ANY flight** — the safety pilot is the last net. Drill tooling works
      (rc_status watcher via CM4 router); pick it back up at "turn TX on,
      wait for ACQUIRED".
- [x] **FC microSD swap DONE 2026-08-21**: new SanDisk Extreme U3 32 GB —
      full-surface verified in the laptop (29 GiB write+read, zero
      mismatches), factory FAT32 kept; old card archived whole
      (~/aavc_sdcard_archive_2026-08-21) and RETIRED. Post-install: BOARD
      params 100% from FRAM, fence download answers (dataman alive — the
      no-card state reproduces the wedge signature, good to know),
      parameters_backup.bson auto-regenerated (3,387 B, same size as the
      FRAM-current set). Note: `sd_bench` is NOT in this firmware build —
      on-board latency benchmarking waits for a fw that includes it.
- [ ] Pack at 22.17 V resting = gauge 0% (below V_EMPTY) after the field
      session — CHARGE before any flight; log the charger's returned mAh
      (first row of the ground-truth table).
- [ ] **CAMERA in-flight blur gate** (the new #1 — see SKILL.md shortlist and
      docs/evidence/ulog_review_2026-08-21.md finding 1 + addendum: static
      focus is FINE 1.9-14 m; flight frames scored 41-76 vs 680-780 static).
      **Code lever LANDED 2026-08-21**: `camera_grabber.py --exposure-100us N`
      (+ optional `--gain`) forces `auto_exposure=1` +
      `exposure_time_absolute=N` via v4l2-ctl (fail-soft; unit 100 µs), wired
      as `CAM_EXPOSURE` env with a REAL-side default of 20 (= 2 ms, was auto
      16.6 ms) in `run_mission.sh` ensure_infra + `cm4/launch_flight.sh`;
      bench-verified on the CM4 (readback: Manual Mode, 20). Caught-by:
      `tests/test_cameras.py` exposure trio; `cm4/deploy.sh --check` now
      hashes the two executed sitl/ files so a stale grabber shows as DRIFT.
      ⚠ the OV9281 has NO auto-gain — if outdoor frames come out dark, raise
      CAM_GAIN, don't lengthen the exposure first. GATE STILL OPEN: outdoor
      sharpness A/B (auto vs 10/20/30) then hover-decode over the printed
      marker DURING flight.
      ⚠⚠ **INDOORS THE 2 ms DEFAULT MAKES THE CAMERA LOOK DEAD** — measured
      on the bench 2026-08-21 (same room, same frame, mean grey / Laplacian):
      auto 129/361 · 2 ms gain64 **7/10** · 5 ms 10/14 · 10 ms 50/106 ·
      2 ms gain128 24/24. So a bench session (and the console's own
      auto-`start_infra` camera chip) must run `CAM_EXPOSURE=0` or every
      frame is black mush; the 20 default is for DAYLIGHT only. Corollary
      still unmeasured: full sun may want a value well BELOW 20 — pick it
      from the outdoor A/B, do not assume 20 is right just because it is the
      default. Also learned: while `auto_exposure=3` the
      `exposure_time_absolute` readback is `flags=inactive` and shows its
      DEFAULT (166), NOT what the driver chose — the G7 flight's true
      exposure is therefore unknown, and "16.6 ms in flight" was an
      artifact of reading an inactive control.
- [x] `MPC_THR_HOVER` 0.5 → **0.58 WRITTEN + read-back on the board
      2026-08-21** (operator-approved). Measured true hover ≈0.60 (motors
      mean, flight 3) while the board held PX4's 0.5 default and the
      estimator (`MPC_USE_HTE=1`) logged all-NaN — it never converged in
      flights that short, so the SEED is what the takeoff ramp, the land
      detector and every post-reset first flight actually fly on. Now in
      `tools/preflight_params.py::BOARD`, so a board that loses it shows up
      as a STOP at the next field day. Re-check the ramp on the next flight.
- [ ] Prop balance / camera-mount stiffness — 60 Hz per-rev vibration peak
      (~1 m/s²) measured; control band clean, so non-blocking.
- [x] CM4 archives + frames pulled (2026-08-21, ~/aavc_cm4_runs) · ULog
      discovery loop ran on all three flights · old SD archived whole
      (~/aavc_sdcard_archive_2026-08-21) + evidence files in
      docs/evidence/sdcard_old_2026-08-21.
- [ ] Work the ranked bench/field actions in `community-watchlist.md`
      (RC-loss drill · SD card swap · ESC LVC · CM4-AP-vs-GPS EMI A/B ·
      ArUco decode floor · powerbank idle-cut · MPC_THR_HOVER re-seed ·
      pack discharge curve · v1.17.0 session-hygiene rules).
- [ ] `BAT1_V_DIV` re-verify multimeter-vs-GCS on the PM02D wiring.
- [ ] Charger-mAh + rest-voltage log: start at the next charge cycle.
- [ ] CM4 online: rsync real audit archives + frames; extract real
      `FLIGHT n ENERGY` lines; first real-frame `make replay`.
- [ ] Printed pad: fabricate + bench decode + daytime flight (G7 evidence).
- [ ] KMITL comp config: decide its own height-reference stance
      (`EKF2_HGT_REF=1` there today; 20 m ceiling has 2.5 m watchdog margin
      vs the +2.9 m step measured at KMUTNB — the comp operator's call).
- [x] PILOT-takeover guard at the command layer — DONE 2026-08-21 as part of
      the zombie-re-arm fix above: `_guard_pilot` now covers every
      FC-state-changing method (motion, arming, mission upload, raw drop
      side-channel, param/failsafe/geofence/gimbal setters); `abort()` alone
      stays unguarded (emergency motor kill). The 2026-08-20 zombie
      (gotos to a disarmed aircraft for 3.5 min) was the same mechanism.
- [ ] Watchdog ceiling cross-check against TFmini below 12 m (would have
      voted down flight 2's phantom breach) — safety-code change, own review.
- [ ] `kmitl_config.yaml` battery-block comment mirror (only power narrative
      was synced 2026-08-20).
- [ ] Automated cross-file battery-endpoint consistency check (manual rule in
      power-battery.md until then).
- [ ] docs/REAL_FLIGHT_GCS.md + docs/FLIGHT.md still describe the pre-🚀
      stack-start; status_beacon.main() age logic untested;
      test_status_beacon positional indexing brittle (code-review backlog).
- [x] ✅ **Latch servos dead 2026-08-21 — FIXED THE SAME DAY: the BEC was
      dead, replaced with a new one; all four latches cycled open→hold and
      the operator confirmed every corner moves.** Root cause was purely
      electrical: the wire feeding the AUX servo rail was broken and the BEC
      itself had failed. Evidence chain that ruled everything else out
      (worth keeping — it is how a "the software is broken" report was
      turned into a 5-minute electrical fix):
      GCS press → radio → FC all working (`mavlink status`: TELEM1 GCS
      heartbeat valid, rx 3.2 KB/s; `pwm_out status` DURING the press showed
      func 301-304 driven at 1900/1990 — the GCS ACTUATOR_TEST supervisor's
      own values — for ~20 min straight); CM4-side ACTUATOR_TEST ACK=0 on
      all four; zero physical motion. The 6X does NOT power the AUX rail
      itself; the wire most likely broke in the PM03D→converter→PM02D
      rewires (no real egg release since 2026-08-18). The ORIGINAL "8 s
      late release" report = the same fault while intermittent (failing
      contact), NOT radio queueing. REMAINING: repair/re-solder the BEC
      feed → verify rail ~5 V → press-to-move re-test (expect < 1 s; the
      command path is already proven) and re-latch check on all four.
      ⚠ Field rule: press เก็บ (re-latch) on the console BEFORE restoring
      rail power — the supervisor holds all four latches commanded OPEN and
      they snap open the instant power returns. Lessons: (a)
      `SERVO_OUTPUT_RAW`/`ACTUATOR_OUTPUT_STATUS` here carry MAIN only —
      telemetry is BLIND to the AUX bank; the only software view of AUX
      values is nsh `pwm_out status`; (b) a console restart silently wipes
      the ACTUATOR_TEST hold set; (c) **`system_power.servo_valid` on this
      6X is USELESS as a rail indicator** — it read `1` with the BEC wire
      BROKEN and again `1` after the repair attempt, alongside a rock-steady
      `voltage5v_v` 5.03 (that is the FC's own 5 V from the PM02D, not the
      servo rail) and `voltage_payload_v` 0.0. There is no software view of
      the servo rail on this board: the multimeter is the only instrument.
      Trace it in three points — BEC INPUT (expect pack ~22-25 V) → BEC
      OUTPUT (expect ~5-6 V) → the pin at an FMU PWM OUT connector — plus a
      check that the servo plugs are not reversed on the 3-row header
      (SIG/+5/GND), since a mirrored plug is silent in exactly the same way.
      **Spares rule from this incident: carry a spare BEC to every field
      day.** It is the single component whose failure silently costs every
      point on the scoreboard while the aircraft flies a perfect mission.
- [ ] **ArUco throughput raised 3.4x on 2026-08-21** (operator: "ดันให้
      ประมวลผลได้มากที่สุด"). Measured on the CM4 first, which moved the
      target: per analysed frame the pipeline spent **123 ms writing two PNGs
      + 26 ms reading one back = 78% of all CPU on the FILE FORMAT**, against
      41 ms of actual detection (of which detectMarkers is 19 ms; a blank
      frame costs the same, so the ROI booster is not the cost). Changes:
      grabber writes 10 Hz instead of 5 (the sensor's own YUYV rate at
      1280x720) with the dashboard mirror OFF (`--no-mirror`), one-deep V4L2
      queue, and the vision worker now decodes on a NEW-FRAME guard (mtime)
      while polling at 20 Hz — so the decode rate is the camera's rate and a
      fresh frame waits ≤50 ms instead of ≤300 ms (at 6 m/s that alone was
      1.8 m of pad-position error riding into every fix). Result: 2.9 → ~10
      analysed frames/s, grabber 56% → 74% of ONE core, whole SoC ~35%.
      **FOLLOW-UP THE SAME DAY — the transport went JPEG and the frame files
      are now `/tmp/aavc_*.jpg`** (writer picks the codec from the path
      suffix; every consumer moved in one commit — worker, align loop via the
      shared constant, preflight, dashboard, beacon, status_sync, HITL/gz
      bridges, launchers, clear_state). CM4-measured: encode 48 → 12 ms,
      decode 33 → 15, file 280 → 62 KB (which also lightens the WiFi frame
      sync). Delivered: 10 Hz frames for **39%** of a core instead of 74%, and
      a worker pass of 57 ms instead of 73.
      **And the sensor's MJPG mode turns out to be free** — `CAP_PROP_
      CONVERT_RGB=0` hands back the camera's OWN JPEG bytes, so
      `--mjpeg-passthrough` writes them with NO decode and NO re-encode:
      measured **20.1 Hz for 7.2% of a core** (121 fps available), 33 KB
      frames, and nothing is compressed twice. **It became the REAL default
      the same day**, once its one risk was closed: the CAMERA picks the JPEG
      quality, so a test rendered the actual pad artwork at the pixel sizes
      the mission flies (17-42 px = the 8-16 m band) and pushed it through
      q95/85/80/70/60 — **every size decoded at every quality**, while the
      camera's 33 KB frames sit at ~q80-85. Exposure/gain still apply in MJPG
      mode (auto_exposure=1, exposure_time_absolute=20 read back while
      streaming) and the format negotiates 1280x720. `CAM_PASSTHROUGH=0`
      reverts to YUYV + re-encode. Still owed: the same frames through a
      PRINTED marker in daylight — folded into the exposure A/B already
      required. Past ~18 Hz the decode becomes
      the limit, so `vision.decode_workers` (default 1) can run frames
      concurrently, emitting IN FRAME ORDER — out-of-order fixes would make
      the tracker's confirm span (last_t - first_t) negative and no cluster
      would ever confirm. Also fixed on the way: the pose is now snapshotted
      BEFORE the decode, not after — every fix used to be geolocated with the
      attitude the aircraft had ~55 ms later.
- [ ] Landing loop rate doubled 2026-08-21 (operator: land as close to pad
      centre as possible): `AlignParams.cycle_hz` 5 → 10 with `lock_cycles`
      3 → 6 and `max_lost_cycles` 8 → 16, so the WALL-CLOCK constants the
      validated runs flew are unchanged (lock 0.6 s, lost-before-climb 1.6 s)
      while corrections happen twice as often and the median filter's own lag
      halves (0.4 → 0.2 s — that lag rides straight into the commanded
      setpoint, so it IS landing error). Now config-driven under `align:` so
      `tools/landing_trial.py` can A/B it without touching the flight core.
      ⚠ the three numbers are ONE setting: two are counted in CYCLES, so a
      rate change alone silently makes the loop twitchy exactly when the
      camera is struggling. A test pins the wall-clock constants.
      REMAINING: measure land-ON precision before/after in SITL
      (`tools/landing_trial.py --pad-index 0 --n 8`) — the change is
      argued from first principles and unit-pinned, not yet flown.
- [ ] aavc-gcs launcher `start_infra` sshes as the LOCAL username
      (`bonus-linux@10.42.0.1` → Permission denied, seen 2026-08-21 on
      console restart) instead of `drone@` — harmless while the CM4 stack is
      already up, but the auto-start path is broken; fix the user default in
      the launcher.
