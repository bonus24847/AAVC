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

## Bench/field rules added 2026-08-22 (from the full-system review)

- **A spare BEC goes to every field day.** The one feeding the AUX servo rail
  died on 2026-08-21 and took every egg release with it, while telemetry showed
  nothing wrong — `system_power.servo_valid` reads 1 on this 6X regardless.
- **`COM_MOT_TEST_EN=1` (PX4's own default) + `CBRK_IO_SAFETY=22027` means that
  while DISARMED at the resupply hold, any MAVLink source can drive a latch or
  a motor.** In flight it is inert (Commander denies ACTUATOR_TEST while armed),
  so this is a crew-safety and egg-on-the-ground risk between flights, not an
  airborne one. `docs/SERVO_AUX_MAPPING.md` says to set it back to 0 before
  flying and nothing enforces it — keep hands and eggs clear of the aircraft
  while the console is up.
- **`/tmp` on the CM4 is tmpfs (RAM), 3.9 GB free of 7.8.** Frame files cost no
  SD wear at any rate we can produce.
- **The two repos share EVERYTHING except `.aavc_site`** since 2026-08-22 —
  including both field configs and `tools/`. Run `bash sync_core.sh <other>`
  after any core change and commit the target too; the drift this closed had
  left the comp repo without a type audit at all.

## 2026-08-23 — the camera was bolted upside down, and nobody had looked

Bench session with the CM4 + FC live. Three findings, in the order they bit.

- **`mount_yaw_deg` was an ASSUMPTION (0.0) and the truth is 180.** Symptom:
  the sweep sees pads and never re-acquires them (G7 #1 decoded 2 of 6).
  Mechanism: `vision/projection.py` un-rotates each ray by this angle before
  reading body forward/right, so a 180 error point-mirrors EVERY pad fix about
  the aircraft — at the 12 m sweep a frame-edge pad ~9 m out is reported ~18 m
  from where it is, the aircraft flies to empty grass, and the tracker's
  cluster never confirms. Nothing else misbehaves, which is why it survived.
  Fix: measured on the airframe (`tools/measure_mount_yaw.py`), aircraft raised
  ~0.75 m on a crate and levelled, camera looking down at the floor, ONE object
  walked clockwise round it — nose / right / tail / left, one nadir frame each.
  The four frames solved 180.3 / 187.4 / 183.4 / 186.1 (mean 184.3), so the
  bolted angle is 180 and the old 0.0 was wrong by ~175 in EVERY direction.
  Caught by: `tests/test_measure_mount_yaw.py::_BENCH_2026_08_23` keeps the
  four pixels as data and `test_the_bench_measurement_is_what_the_config_ships`
  fails if either field config drifts off the measurement.
  ⚠ **Re-measure after ANY camera re-mount** — SITL cannot check it, because
  the gz camera is posed with the same assumption the code makes.
  Side effect, handled: the sweep heading is derived as
  `leg_bearing - mount_yaw`, so 180 alone would have flown every sweep leg
  BACKWARDS. `search_pattern.py` now takes whichever of the two valid
  perpendiculars flies the leg nose-first (the footprint is a rectangle, so
  coverage is identical). `overlap_frac` deliberately stays at **0.44**
  (operator call): the measurement licenses 0.30 and ~158 s back at KMITL, but
  not while the in-flight decode problem is open.

- **`measure_mount_yaw.py` had a sign error that only an off-nose placement
  could expose.** It computed `psi = ang - bearing`; `projection.py` wants
  `psi = ang + bearing`. At the documented `bearing = 0` (object at the nose)
  the two are identical — and at 180 they agree too — so the tool was correct
  at exactly the placements its own runbook and its own tests used. The
  right-wing confirmation frame read **7 deg** where the aircraft's projection
  says **187**. Fix: the plus, plus tests that close the loop THROUGH
  `project_pixel` at bearings 0/90/180/270/37/-125 instead of re-checking the
  tool's arithmetic against itself. Lesson worth keeping: a self-consistent
  test of one formula proves nothing about the convention it must match — the
  round trip is the test.

- **The USB camera re-enumerates and the grabber keeps running on a dead fd.**
  Symptom: `/tmp/aavc_nadir.jpg` stops changing (same mtime, same md5) while
  `pgrep camera_grabber` still shows the process. `dmesg`: `usb 1-1.1: USB
  disconnect, device number 3` then a re-enumerate as device 4, moving the
  camera **video0 -> video1**. The `by-id` path in `run_mission.sh` fixes the
  STARTING case only; a mid-run disconnect leaves the already-open descriptor
  dead. Trigger here was moving the aircraft by hand — which means a bump in
  flight can kill the camera for the rest of a sortie with the aircraft still
  flying. Detection EXISTS and works: `cm4/status_beacon.py` sends
  `AAVC cam=DEAD <n>s stale` once the frame is >6 s old, over the radio.
  ✅ **FIXED the same day**: `sitl/camera_grabber.py` now runs a frame-liveness
  watchdog — no frame WRITTEN for `--reopen-after-s` (default 3 s) and it
  releases the handle and re-opens the `by-id` path, which resolves to whatever
  node the camera came back on, re-forcing the exposure (a re-enumerated node
  returns to the driver's defaults). Failed re-opens back off 1→15 s instead of
  storming, and a camera that comes back at a DIFFERENT resolution is refused,
  not flown — the projection derives fx and the principal point from the
  configured size. The process never exits over this: in flight nothing would
  restart it, and a frame file that stops ageing is already the honest signal.
  Manual fallback if it ever needs one: kill by PID from
  `pgrep -f 'camera_grabber.p[y]'` and let `start_infra.sh` restart it.
  **VERIFIED LIVE on the aircraft the same day**, without unplugging anything
  (`sudo` on the CM4 needs a password, so the USB `authorized` toggle was out):
  a second grabber was pointed at a symlink, started on a node that opens and
  delivers nothing (`/dev/video14`), and the symlink was then repointed at the
  camera's `by-id` path. The log shows the whole cycle — `no frame for 3.0s —
  reopening` → 37 re-opens while the camera stayed dead → `reopen failed …
  retrying in 15s` as the backoff capped → `reopened (#38)` → the frame file
  appearing and then ageing normally. Saved as
  `docs/evidence/camera_selfheal_2026-08-23.log`.
  ⚠ **It re-enumerated AGAIN, unprompted, during that very test** — `by-id`
  moved from video1 back to video0 at 14:16 with nobody touching the aircraft.
  Twice in one afternoon: treat this as a frequent event, not an anomaly, and
  never hard-code a `/dev/videoN` anywhere. Every consumer must go through
  `/dev/v4l/by-id/…-video-index0`.
  ⚠ Two limits of the fix, both known: a `read()` that BLOCKS forever is not
  covered (that needs a second thread), and on the dead node each failing read
  took ~10 s to return, so the watchdog fires a few seconds later than
  `--reopen-after-s` suggests.
  ⚠ Reading the frame file WITHOUT checking its age is how this fooled a whole
  measurement session: three "no marker found" results in a row were a
  14-minute-old picture of the floor. Check mtime before trusting any frame —
  `tools/measure_mount_yaw.py` now REFUSES a frame older than 10 s
  (`--max-age-s 0` to measure from an archived one on purpose), and the beacon
  was saying `cam=DEAD` over the radio the whole time nobody was looking.

- **`pgrep -f` bites from the other side too.** `run_mission.sh` reported
  "mavlink-router already up" when no router existed, because the ssh command
  line that launched it contained the literal string `mavlink-routerd` (in an
  unrelated `pgrep` at the end of the same line) and `ensure_infra`'s own
  `pgrep -f 'mavlink-route[r]d'` matched THAT shell. The bracket idiom protects
  the pattern from itself, not from other processes quoting it. If staging ever
  fails with `unmet critical checks: link` while the log says the router is up,
  suspect this first.

## Hover decode test — the procedure (built 2026-08-23, NOT YET FLOWN)

The open question is narrow: the camera is sharp on the bench (Laplacian
680-780, decoding the 38 cm marker 1.9-14 m) and scored 41-76 in flight with
0 of 402 frames decoding. Three fixes have landed since — held sweep heading,
forced 2 ms exposure, 20 Hz frames — and none has been tested in the air.

1. Put the printed marker on the ground. Start the normal stack
   (`cm4/start_infra.sh`), then beside it:
   `.venv/bin/python tools/hover_decode.py`
2. Hand-fly a hover over the marker and HOLD each height ~20 s, stepping
   3 → 5 → 8 → 12 m. Twenty seconds is a few hundred frames, enough for the
   rolling verdict to mean something.
3. Read the console while flying — the beacon carries `AAVC cam=<WORD>
   dec=n/N sh=x` over the radio and the console spells it out:
   GOOD (remember this height) · WEAK (come down a bit) · BLUR (lower/slower;
   if it is still blurred while stationary the mount is the problem) ·
   HIGH (not blur — too high, or the pad is not under the camera) ·
   DARK (2 ms exposure with no auto-gain — raise `CAM_GAIN`).
4. After landing, `/tmp/aavc_hover_decode.jsonl` has every frame with its AGL:
   that gives decode rate PER HEIGHT, which is the number that decides the
   sweep altitude.

⚠ Expected marker sizes at fx 847: 400 mm is 42 px at 8 m, 28 px at 12 m,
21 px at 16 m — the detector's design range. Do NOT run this test with the
marker filling the frame: at ~33 px per module the default
`adaptiveThreshWinSize` (max 23) cannot threshold inside a module and the
decode fails for a reason that has nothing to do with flight. That is exactly
what a bench frame from 0.75 m reads as, and the tool correctly calls it HIGH.

⚠ The tool itself has been run end-to-end on real frames but NEVER in flight.

## Open items / follow-ups (updated 2026-08-22 — post-review)
- [x] 🟠 **The failures that HIDE — 14 fixed 2026-08-22** (practice ac4a155 /
      comp 8dc3d27 / console ca7ab79). One family, found by the full-system
      review: the system kept working and stopped telling the truth.
      **Flight core** — `drop_payload` had no exception boundary up the whole
      stack (anything but a clean ActionError ended the mission ARMED on a
      pad); `land()`/`rth()` guarded only at entry and then ran a 30 s/180 s
      wait into an unconditional `disarm()`, so a pilot rescue mid-descent was
      answered by disarming a flying aircraft; the ACQUIRE "expanding box"
      never expanded (`(ring % 4) // 2 or 1` is always 1); all 14 telemetry
      subscribers shared one `_touch()` so ONE frozen stream was masked by the
      other thirteen — and a frozen `is_armed` blinds the new disarm detector
      permanently; the per-rung ladder wrote `MPC_Z_VEL_MAX_DN`, the MANUAL
      twin (inert for AUTO, while leaving 3.0 = 2x the pin in the param the
      safety pilot's POSCTL rescue DOES read); the takeover detectors were not
      actually checked first; `record_anomaly` deduped by kind while
      `altitude_ceiling_warn_{alt:.1f}m` minted a new kind every 0.1 m (~120
      lines a minute onto the SD card). **Verifier/console** — a takeover
      flight PASSED `verify_flight.py` with 0/0 deliveries; the progress label
      said "delivering pad N" during an abort landing (and the radio scrapes
      `cur=` from that label); `window.PLAN_KEEP` was never cleared so the
      previous flight's route stayed drawn through the next mission's
      preflight; the radio display gates were 15 s against a 5 s beacon (two
      lost packets blanked the readout, with nothing on screen saying the radio
      was quiet); the beacon burst a whole tick into a 1200 B/s link.
      **Tooling** — `px4_type_audit.py` could not see the 8 inline-named param
      writes, which are the FAILSAFE chain and fail exactly the way
      `EKF2_HGT_REF` did (a TIMEOUT that reads like a link hiccup).
      Caught-by: `tests/test_telemetry_streams.py` (new), the land/rth
      mid-command takeover pins in `test_commands.py`, the ceiling-warn dedupe
      + identity-numbered pins in `test_safety.py`, the hardcoded-setter
      inventory in `test_px4_type_audit.py`, the pacing pins in
      `test_status_beacon.py`, and the takeover/unfinished-flight checks in
      `test_verify_flight.py`. 592 green.
- [x] ⏸→**CLOSED 2026-08-22** (see the dated section at the end of this file):
      **Fix 3 (mission-plan polyline) could not reach a REAL flight** —
      the plan is written on the CM4 and nothing copies it to the laptop:
      `status_sync.sh` was deliberately removed from the real launchers on
      2026-08-18 (operator: "เอาไหนที่ใช้วิทยุไม่ได้ เอาออกเลย") and the beacon
      does not carry `plan` (too big for a 50-char STATUSTEXT). Two options,
      BOTH the operator's call because both change that locked decision: a
      one-shot WiFi pull of the few-KB plan at staging (WiFi is up at L&R
      then), or drop the polyline from the real console. Everything else about
      the feature now works — written, run-scoped, staleness-styled, and live
      on the SITL/WiFi console.
- [x] ⏸→**RESOLVED 2026-08-22 by instrumenting it** (dated section at the end):
      **The x4 ROI decode booster added ZERO decodes across 32 synthetic
      conditions** while costing a detector construction + resize + a full
      `detectMarkers` per blob, uncapped. Left IN deliberately: the synthetic
      set is not the real-marker daylight case it was written for, and
      removing a decode path two days before a flight on synthetic evidence is
      the wrong direction of risk. Decide it with real hover frames.
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
- [x] **Dashboard raw dispatches no longer bypass the pilot guard — DONE
      2026-08-23.** They called `commander.system.action.*` directly, so they
      never passed `_guard_pilot` and a console click could arm or fly a
      pilot-owned aircraft. Closed at `_dispatch`, the single choke point every
      verb goes through, so no handler can be added later that forgets:
      `_POST_TAKEOVER_ALLOWED = {"kill", "vehicle_disarm"}` and everything else
      gets a 409. Those two stay open deliberately — the pilot may have taken
      over BECAUSE something is wrong, and taking the kill switch away from
      them is the wrong failure direction. `DroneCommander.pilot_in_control` is
      now a public property so callers outside the class can refuse BEFORE
      acting instead of learning it from an exception. Caught-by:
      `tests/test_dashboard_commands.py` — 6 flying verbs refused, both safing
      verbs still dispatched, and the ordinary no-takeover path unchanged.
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
- [x] **G7 attempt #1 analysis — DONE 2026-08-23, and the question was wrong.**
      The item read "why sweep legs stalled from wp7". They did not stall from
      wp7: `audit_20260819T154517Z.jsonl` shows **wp0 through wp7 timing out in
      turn, at Δ≈25.0 s each** — exactly `_ProgressGuard`'s "no 1 m of closure
      for 25 s" window — while all three ingress transit points were MISSED by
      40.0 / 43.7 / 47.3 m. The TELEM track says why: `armed=0` and
      `alt≈0.0` from t≈41 s, max altitude 5.7 m, max distance from the start
      fix 15.4 m. **The aircraft was parked and disarmed while the mission flew
      on until t=369 s.** That is the ZOMBIE MISSION, root-caused and fixed
      2026-08-21 (`4fe3935`) — the pilot took over and disarmed inside the
      0.46 s the old detector could not see. Matches ULog `07_21_36` (42 s
      armed, ended by takeover) exactly. The other half of the item, "only 2/6
      markers decoded", is the yaw spin + blur root-caused 2026-08-22. **Both
      halves are closed by fixes that already shipped; neither needed a new
      one.** Lesson: the anomaly names described the SYMPTOM's location, and
      reading them as the fault sent the question to the wrong subsystem for
      two days. `armed=` was in every TELEM line the whole time.
- [x] **The other two flights of that session now read cleanly too**, which is
      what made the above legible: flight 2 (`audit_20260819T155948Z`) flew
      transit P1 1.7 m / P2 1.8 m then `altitude_ceiling_warn_10.6m` →
      `breach_12.0m` → `sustained` at t=30.7 s → RTH, P3 missed at 9.6 m
      because it was already returning. Flight 3 (`audit.jsonl`) flew
      **transit 3/3 at 1.5 / 2.0 / 1.4 m** and was RTH'd by `battery_low_28%`
      at t=68.2 s on a part-charged pack. So the navigation is good: three
      flights, three unrelated causes, and the aircraft tracked its route to
      ~1.5 m whenever it was allowed to fly.
- [x] 🔴 **"RC does not RE-ACQUIRE after a TX power-cycle" — ROOT CAUSE FOUND
      2026-08-23, and it was never the radio.** The KILL SWITCH was engaged.
      `RC_MAP_KILL_SW=8` and ch8 sat at 2011 (ON); a TX power-cycle re-reads the
      physical switch positions, which is exactly why the symptom appeared at
      that moment and never cleared. Mechanism, verified in the v1.17 source:
      `manualControlCheck.cpp` raises `armingCheckFailure(...,
      health_component_t::remote_control, "Kill switch engaged")`, and
      `SYS_STATUS.hpp::fillOutComponent` sets the RC_RECEIVER health bit ONLY
      when that component has no arming-check error — so an engaged switch
      CLEARS the RC health bit while the link is perfect. Every RC indicator
      this project owns (MAVSDK `rc_status`, QGC's RC bar, the drill script)
      reads that bit, so a switch position and a dead transmitter produce the
      identical "RC unavailable".
      Evidence at the bench, in order: RF fine (TX16S full bars, RX solid
      green); FC receiving (`RC_CHANNELS chancount=16 rssi=100`); frames LIVE
      (ch1-ch4 moved 624-930 us while the sticks were worked — which killed the
      "RX is sending a frozen failsafe frame" hypothesis); PX4 naming it
      (`[sev 2] Preflight Fail: Kill switch engaged`); and the confirming flip —
      switch off → `healthy=True`, MAVSDK `available=True signal=100%`.
      **Caught from now on by `tools/rc_check.py`**, which reports frames,
      movement, the raw present/enabled/health triplet, the kill/arm channel
      values and the FC's own STATUSTEXT, and returns 2 for "no frames — this
      IS the link" vs 1 for "frames arrive, a SWITCH cleared the health bit".
      ⚠ The OTHER half of this blocker is still OPEN: the **RC-loss drill** has
      not been run, and the ELRS failsafe mode still needs to be set to **No
      Pulses**. Do not read this tick as "RC is signed off".
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
- [x] `BAT1_V_DIV` re-verified on the PM02D 2026-08-23: multimeter 24.9 V vs
      FC 24.89 V (0.04%). Stays -1. Endpoints checked in the same reading.
- [ ] Charger-mAh + rest-voltage log: start at the next charge cycle.
- [ ] CM4 online: rsync real audit archives + frames; extract real
      `FLIGHT n ENERGY` lines; first real-frame `make replay`.
- [ ] Printed pad: fabricate + bench decode + daytime flight (G7 evidence).
- [x] KMITL comp config height-reference stance — **DECIDED 2026-08-23: baro,
      same as practice** (see the dated entry at the end of this file). This
      item's own arithmetic was what settled it, once the number it compared
      against was the right one: it cited a "+2.9 m step", but the ULog review
      measured **10.8 m peak-to-peak** of baro-vs-GPS divergence on the
      phantom-ceiling flight. 2.5 m of margin never covered that.
- [x] PILOT-takeover guard at the command layer — DONE 2026-08-21 as part of
      the zombie-re-arm fix above: `_guard_pilot` now covers every
      FC-state-changing method (motion, arming, mission upload, raw drop
      side-channel, param/failsafe/geofence/gimbal setters); `abort()` alone
      stays unguarded (emergency motor kill). The 2026-08-20 zombie
      (gotos to a disarmed aircraft for 3.5 min) was the same mechanism.
- [ ] Watchdog ceiling cross-check against TFmini below 12 m — **re-scoped
      2026-08-23, and NOT the fix for what happened.** Flight 2's "phantom
      ceiling" was a height-REFERENCE artifact: it flew on `EKF2_HGT_REF=1`
      (GPS) with 10.8 m of baro-vs-GPS divergence p2p, so the fused altitude
      wandered into a 12.0 m reading against a 10 m ceiling. Flight 3 the same
      afternoon on `=0` (baro) had no altitude event at all. Fixing the cause
      beats teaching the watchdog to disbelieve its own input, and a veto layer
      would have MASKED a 10.8 m estimator error rather than surfaced it. The
      cross-check survives only as defence-in-depth, and with a bound worth
      writing down: the TFmini returns plausible ranges to ~10 m over daytime
      grass, so it can veto a false breach reported below that and nothing
      above it. Own review, still.
- [x] **`EKF2_HGT_REF` = 0 (BARO) at BOTH fields — operator decision
      2026-08-23** ("เรื่องความสูงเป็นสิ่งที่สำคัญมาก แนะนำให้ใช้ baro กับ
      lidar ตามเดิม"). `kmitl_config.yaml` had carried PX4's default 1 (GPS)
      for three days after the practice field moved off it, on the reasoning
      that a 20 m ceiling has more watchdog margin. It has **2.5 m** (transit
      commanded 19.5 m, watchdog RTH at 22 m) against a **measured 10.8 m** of
      baro-vs-GPS divergence — the same breach, on a scored flight, inside a
      20-minute window that does not survive an RTH. Everything else about the
      height stance was already identical at both fields and stays: lidar fused
      (`EKF2_RNG_CTRL=1`) with conditional aiding below 7 m pinning the final
      metres, flow off, and NOT `=2` (range as the reference) — that makes the
      local origin ride ground level, so a shed cargo box or a person under the
      beam would move "down". `DEFAULT_PX4_TUNING` has been baro since
      2026-08-20, so the config was the last place the old value lived.
      Caught-by: `test_px4_tuning_parity.py::test_every_field_flies_baro_height_with_lidar_aiding`
      walks EVERY `sitl/*config.yaml`, so a field added later cannot ship PX4's
      default quietly, plus the `BOOT_LATCHED` check above.
      ⚠ **reboot_required**: pushing it at mission start stores the value and
      changes nothing until the FC reboots. On the field day, run
      `tools/preflight_params.py` — if the board still reads 1, reboot after the
      push and re-check.
- [x] **`EKF2_HGT_REF` is now CHECKED before a field day — DONE 2026-08-23.**
      It was in neither `preflight_params.py` list, and it could not simply
      join PINNED: the EKF latches its height reference the first time any
      source fuses (`EKF/height_control.cpp:61` returns early once
      `_height_sensor_ref` is set), so `apply_param_overrides` writing it at
      mission start changes the NEXT flight, not the one about to happen. Only
      the value the board BOOTED with counts. It is therefore a third class —
      `BOOT_LATCHED` — checked as a BOARD-grade STOP, with the expected value
      read from the field config in force (`.aavc_site`, or `--config`) rather
      than hard-coded, because the two fields disagree today and a check that
      is wrong at one of them gets ignored at both. An unreadable config skips
      the check instead of inventing a value to compare against.
- [ ] `kmitl_config.yaml` battery-block comment mirror (only power narrative
      was synced 2026-08-20).
- [x] **Cross-file battery consistency is now a test — 2026-08-23.**
      `tests/test_battery_consistency.py` executes the rule that lived in
      power-battery.md as "keep these in step by hand": both field configs must
      describe the SAME pack, `battery.cells` must equal the `BAT1_N_CELLS` the
      preflight STOPs on, `power-battery.md`'s quoted `BAT1_V_CHARGED` /
      `BAT1_V_EMPTY` must equal what `BOARD` enforces (the doc is what gets
      believed at 07:30; `BOARD` is what stops the day), and the two pins that
      keep coulomb counting shut — `BAT1_CAPACITY <= 0` and
      `raw_telemetry_port: 0` — must hold in every config. Plus an arithmetic
      sanity check on the endpoints against the operator's 25.1 V / ~22.6 V
      spec, which catches a decimal slip that every equality above would agree
      on. These numbers have already disagreed once across the two repos
      (`BAT1_V_CHARGED` 4.15-vs-4.05) and a wrong one never LOOKS wrong: the
      whole gauge is `interpolate(cell_v, V_EMPTY, V_CHARGED)`.
- [x] **Runbooks corrected 2026-08-23.** `REAL_FLIGHT_GCS.md` still told the
      operator the launcher starts `status_sync` alongside the console — removed
      from the real launchers on 2026-08-18 — so it now states what the real
      console actually reads (the NOMAD beacon, everything), which three things
      still ride ssh/WiFi and that all three work only at the launch point
      (🚀 stage, auto camera+beacon bring-up, the one-shot plan pull), and to
      read the **📻 ขาด N วิ** badge before believing a quiet screen.
      `FLIGHT.md` carried three: "props off until G6" (G6 was dropped
      2026-08-16 — props off through G5, first props-on flight is G7), the
      retired `.png` frame paths, and — the one that mattered — it documented
      `HEADLESS=1` **auto-GO** without saying it bypasses the RC gate. There are
      two real-flight entry points with different launch semantics: the
      console's 🚀 runs `run_mission.sh REAL=1`, which defaults `RC_GO=1` and
      only STAGES, while `cm4/launch_flight.sh HEADLESS=1` launches the
      aircraft itself once preflight passes. The runbook now puts them side by
      side, because the dangerous one was the command written down first.
- [ ] status_beacon.main() age logic untested; test_status_beacon positional
      indexing brittle (code-review backlog).
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
- [ ] **Landing loop retuned 2026-08-21** (operator: land as close to pad
      centre as possible) — and a REAL BUG found on the way: the loop slept a
      fixed `1/cycle_hz` AFTER its work, so the configured rate was never the
      achieved one. With ~60 ms of detect on the CM4, "5 Hz" ran at **3.7**,
      and `lock_cycles`/`max_lost_cycles` — counted in cycles — silently
      tracked whatever the detector cost, so a faster camera would have
      shortened the very timeouts that keep the descent honest. Fixed with
      deadline pacing (`_Pacer`: sleep only the remainder; an overrunning
      cycle returns immediately instead of stacking debt). Now
      `cycle_hz=12` truly means 12, with `lock_cycles=9` / `max_lost_cycles=24`
      holding the wall-clock band every validated run flew (lock ~0.75 s,
      lost ~2.0 s). The win is LAG, not raw rate: the median filter's own
      delay drops ~0.8 s → **0.25 s**, and at even 1 m/s of drift that delay
      alone was a quarter-metre of landing error. Config-driven under
      `align:`; a test pins the wall-clock band so the trio cannot drift
      apart. ⚠ ~60 ms of work per cycle puts the honest ceiling near 16 Hz —
      12 keeps ~38% headroom for CPU contention with the vision worker.
      REMAINING: measure land-ON precision (`tools/landing_trial.py
      --pad-index 0 --n 8`, or the real flight) — argued from measurement and
      unit-pinned, not yet flown.
- [x] `/tmp` on the CM4 is **tmpfs (RAM)**, 3.9 GB free of 7.8 GB total —
      checked 2026-08-21 before raising the frame rate. Frame files therefore
      cost no SD wear and no SD write latency at any rate we can produce,
      which is what makes 25 Hz frames a free choice rather than a trade.
- [x] aavc-gcs launcher `start_infra` sshed as the LOCAL username
      (`bonus-linux@10.42.0.1` → Permission denied, seen 2026-08-21 on
      console restart) instead of `drone@` — harmless while the CM4 stack was
      already up, which is exactly why it went unnoticed. FIXED 2026-08-21
      (aavc-gcs `efbab6e`): `_maybe_start_infra` now takes the full
      `user@host` AND the `-i` identity out of the GO command
      (`_mission_cmd_ssh_target` / `_mission_cmd_ssh_identity`) instead of the
      bare hostname the TCP probe works with. The CM4 key is not `id_rsa`, so
      the identity mattered as much as the user.

### The two items Tier 4 left open — both closed 2026-08-22 (operator: "จัดเลย")

- [x] **The mission route could not reach a real flight** (review 3.9). The
      polyline shipped 2026-08-21 drew in SITL and was invisible where it was
      needed: the plan is written on the CM4, a STATUSTEXT packet is 50 chars,
      and the laptop kept no copy. Closed with the NARROW option —
      `_maybe_pull_plan()` ssh-`cat`s `mission_status.json` at most every 8 s
      and only while the ssh probe says the CM4 answers; `_plan_from_status()`
      keeps the plan fields and nothing else. That is not a rollback of the
      2026-08-18 radio-only rule: phase, pads, camera health and home-reason
      stay on the beacon precisely BECAUSE a WiFi copy of those freezes on
      link loss and then contradicts the radio. A route cannot fail that way —
      it is what the aircraft was TOLD to fly, it first exists at gate release
      with the aircraft still at L&R in WiFi range, and once the link drops the
      map keeps it faded and labelled "ค่าล่าสุด — ลิงก์ขาด". A failed read
      returns `None`, never an empty plan, so out-of-range never wipes the map.
      ⚠ Found on the way, and the reason the pointer was never drawn:
      `plan_ptr` was a COMMAND index handed to a map that indexes DRAWN
      waypoints. Every skipped command (DROP_PAYLOAD, anything without a
      coordinate) slides the two apart, so it could only ever have highlighted
      a stop the aircraft had already left. `_plan_pusher` now translates it,
      and the leg being flown draws large and orange. **Lesson worth keeping:
      an index is only meaningful next to the list it indexes — shipping one
      across a process boundary without its list is how a feature ends up
      "working" and never drawing.**
- [x] **×4 ROI decode booster: KEPT, and now measurable.** 32 synthetic
      conditions produced zero decodes only it could make, which is an argument
      to delete a decode path — on evidence that cannot cover the case it was
      written for (real optics: motion blur, a marker printed on cloth, sun
      glare in the quiet zone). Deleting it two days before a flight on that
      basis is the wrong direction of risk, so the question was made
      answerable instead of re-argued: `vision/detectors/aruco.py::DECODE_STATS`
      counts `frames/direct/boosted/cue_only`, and `vision_worker.stop()` logs
      `[vision] decode provenance: …` at the end of EVERY flight. `boosted=0`
      after a real hover over a printed marker deletes it; anything else keeps
      it. **The rule this encodes: when synthetic evidence disagrees with a
      component's stated purpose, instrument it — do not let the cheaper
      evidence win by default.**

### 2026-08-22 — the competition config was still standing on the practice field

- [x] **`sitl/kmitl_config.yaml` carried the PRACTICE field's whole `site:`
      block** — name, `center_lat/lon`, and the rooftop `ground_alt_m` — over
      the competition field's own L&R. **31.5 km apart**, with the comment on
      `center_lat` reading `# = ground_operation.launch_recovery (ENU/world
      origin)`: the file asserted the equality it broke. Found while rendering
      the mission route for the operator, not by any check.
      **Why nothing noticed:** the aircraft never reads it. PX4 captures home
      where the vehicle arms, and the plan takes its home from GPS, so the
      flight is correct either way. What reads `site.center` is everything that
      turns a pad's lat/lon into METRES — `orchestrator/gcs_status.py`'s
      `_enu`, and therefore the console feed, the `AAVC pads` beacon lines, the
      Svelte dashboard and `sitl/spawn_targets.py`. The console then re-anchors
      those metres at the VEHICLE's origin, so at KMITL every pad marker would
      have been drawn **31.5 km off the map** while each individual number
      looked entirely reasonable — the pad readout is a V1.3 scoring line.
      **Fix, in two halves that do different jobs:**
      `orchestrator/main.py::_resolve_site_origin` reads
      `ground_operation.launch_recovery` FIRST (the point the aircraft itself
      uses), falls back to `site.center`, returns `None` rather than (0, 0)
      when a config names no field at all, and **logs an ERROR naming both
      values and the distance** when the two disagree — because the SITL
      spawner and the dashboard still read `site.center` directly, so
      resolving it quietly here would have fixed one consumer of four. The
      config itself was then corrected, which fixes the other three at once.
      **The check that would have caught it:**
      `tests/test_geometry_invariant.py::test_every_field_config_calls_its_L_and_R_one_thing`
      walks EVERY `sitl/*config.yaml` and fails when `site.center` and
      `launch_recovery` are more than 1 m apart. Deliberately `gen_geo`-free so
      it covers the competition field and any field added later — verified to
      fail on the old file with `site.center is 31,518 m from …`.
      **Lesson, and it is the same one this file was born with:** the leg that
      drifts is the leg nothing reads out loud. A comment asserting an
      invariant is not the invariant; only a test that fails is.
- [x] Same file, same sweep — corrections that cost nothing to make and
      mislead a human at exactly the wrong moment: the header described the
      flight as `transit at 4 m` / `search 2.5-5 m band` (a KMUTNB-era
      profile) and quoted the practice pitch's `143.2 deg` axis while
      `search.sweep_axis_deg` correctly holds KMITL's `87.0`; the `mission:`
      block documented `10 / 9.0 / 2.5` inside a file flown with the
      **competition** profile (20 / 20 / 10) and named `kmutnb_skyfield` as
      the profile in force; the servo comments pointed at
      `gcs/kmutnb_field.yaml` when this field's console reads
      `aavc-gcs/aavc_field.yaml`. None are read by code. All of them are read
      by whoever writes the briefing.
### 2026-08-23 — two things the archived flight logs were saying all along

- [x] **PX4's battery SIMULATOR was being written to the real aircraft, on the
      pad, inside the scored window.** Every real mission start logged three
      `SIM_BAT_* failed: TIMEOUT` warnings and `applied 0/3` — **~9 s** of the
      20-minute window spent proving a module that is not in the fmu-v6x build
      is not there. The gate was `_is_sitl_endpoint`, and its OWN docstring
      says why that is wrong here: `cm4/launch_flight.sh` runs the real
      aircraft through a mavlink-router at `udpin://0.0.0.0:14540`, so the real
      bird reads as SITL. `_detect_simulator` — which asks for `SIM_GZ_EN`, a
      param that exists only in px4_sitl builds — **was written for exactly
      this and then never called by anything.** Now it is, via
      `_should_push_sim_battery`, with the endpoint heuristic kept only for
      "the link could not answer at all" (a wrong guess about a battery
      simulator costs a timeout, never safety — which is why a fallback is
      acceptable here and would not be for a safety pin).
      The seconds are the smaller half. The bigger one: `applied 0/3` at every
      launch teaches the operator that `applied 0/N` is normal, and
      **`applied 0/24` is the signature this project relies on** to catch a
      dead param link (the stale-mavsdk_server story). A warning that always
      fires is a warning that stops being read.
- [x] **The geofence upload retries now (3 attempts, 1 s apart).** It was one
      shot, and on 2026-08-19 that cost **two staged flights in one session**:
      `clear_geofence: TIMEOUT` → upload error → "FC geofence NOT verified …
      refusing to fly", twice, twenty seconds apart, on a link that was
      otherwise healthy. The refusal is CORRECT and stays — PX4 answers a
      missing fence with "accept all points", so an unverified fence is no
      fence — but the fix for a flaky RPC is to stop losing the upload, not to
      stop checking it. Retrying weakens nothing: `_verify_geofence` still has
      to pass on the attempt that lands, and a fence that reads back as another
      field's is refused on every attempt. Competition day gives 5 minutes of
      setup before the clock starts; a re-stage does not fit in it.
- [x] **`EKF2_HGT_REF` and `MAV_1_FORWARD` TIMEOUTs in those same logs are
      ALREADY FIXED** — checked, not assumed. Both are INT32 and were being
      written with `set_param_float`, which PX4 answers with silence that
      MAVSDK reports as `TIMEOUT`; both are in `_INT_PARAMS` today, and the
      last run of that session already shows `28/29` instead of `27/29`. Worth
      recording because the two failure lines look identical to the SIM_BAT
      ones and it would be easy to "fix" them twice.

### 2026-08-22 — the camera was spinning because nobody ever chose a heading

- [x] 🔴 **ROOT CAUSE FOUND for the in-flight blur: `MPC_YAW_MODE` sat at PX4's
      factory 0 = "towards waypoint" on all three 2026-08-20 flights**, and the
      repo never pushed that param until 2026-08-21. Operator saw it from the
      field first — "โดรนหมุนตัวครบ 360 องศาขณะกำลังไปข้างหน้าเพื่อ scan หา pad".
      **Confirmed in the logs, from the SETPOINT side, which is what makes it a
      command and not a drift:** ULog `08_11_09.ulg` — the commanded heading
      walks 144.8 → 119 → 94 → 69 → 44 → 18 → 353 → 327 → 302 → 276 → 249 →
      223 …, a full circle at ~25 °/s (= `MPC_YAWRAUTO_MAX`), **867° of yaw
      travel in a 122 s flight** (2.4 turns); flight 2 459°; flight 1 147°.
      Alternatives falsified in the same read: `WV_EN=0` (no weathervane), and
      the *setpoint* moving rules out an EKF/mag yaw drift the controller was
      merely chasing.
      **Mechanism:** the mission flies entirely on `goto_location`
      (DO_REPOSITION) with `yaw_deg` left at its `NaN` default, and PX4's
      `FlightTaskAuto` only reaches `_set_heading_from_mode()` — i.e.
      `MPC_YAW_MODE` — when the triplet yaw is NOT finite
      (`FlightTaskAuto.cpp:490-501`). NaN yaw + factory param = "point the nose
      at the next waypoint", and a boustrophedon sweep alternates leg direction
      by 180°, so the aircraft turned at every leg end. The nadir camera is
      bolted to the body: that is the rotational half of the blur, and it is
      why only **1 of 457** recorded frames decoded.
      **Fixed three ways, deliberately overlapping:** (a) `MPC_YAW_MODE: 5`
      (yaw fixed) has been in the config since 2026-08-21 but **has never
      flown**; (b) the search phase now passes an explicit
      `yaw_deg=spec.sweep_yaw_deg` on all four of its gotos, which **beats the
      param outright** because PX4 takes the triplet yaw first — so this works
      even if 5 never lands; (c) `MPC_YAW_MODE` joins
      `tools/preflight_params.py::PINNED` so a staged aircraft can be asked
      whether 5 actually stuck. ⚠ That last one is not paranoia: PX4's own
      metadata for the param declares `@max 4` while its enum defines
      `@value 5 yaw fixed` and the switch handles it — a validating layer would
      refuse exactly the value the mission needs.
      **Lesson:** a NaN is a decision to let someone else decide. Every
      `goto` in this repo passed one for a year, and the someone else was a
      factory default nobody had read.
- [x] **The sweep heading is now DERIVED, not inherited.**
      `SearchPlanSpec.sweep_yaw_deg = leg_bearing − CameraModel.mount_yaw_rad`
      places the camera's WIDE (1280 px) image axis across track — the
      footprint `swath_m` has always assumed and `search_pattern.py`'s own
      docstring has always flagged as unvalidated. Before this the orientation
      was whatever heading the aircraft happened to hold, i.e. **how the crew
      set it down on the pad**: wrong way round the real swath is the 720 px
      axis (0.5625×), 10.2 m instead of 18.2 m at 12 m, against a planned
      12.7 m spacing — a **2.50 m gap per strip**, and a 1 m pad fits inside
      with 1.5 m to spare.
- [x] `overlap_frac` **0.30 → 0.44** in BOTH field configs — belt-and-braces
      under the heading fix: it makes spacing ≤ the NARROW swath, so coverage
      is complete at any mount/heading combination. 0.4375 is the 16:9 break-
      even (720/1280); the 640×480 module default is 4:3 and would have hidden
      the whole problem, which is why the test builds an explicit 1280×720
      camera. Cost +1 leg at KMITL. Drop back to 0.30 only once
      `mount_yaw_deg` is MEASURED.
- [x] **`cameras.nadir.mount_yaw_deg` MEASURED 2026-08-23 = 180** — the camera
      is bolted UPSIDE DOWN; the shipped 0.0 was an assumption and was wrong by
      ~175 deg in every direction. Four placements of one floor object round
      the raised, levelled airframe (nose/right/tail/left) solved 180.3 /
      187.4 / 183.4 / 186.1. Both field configs carry 180.0; the four pixels
      are data in `tests/test_measure_mount_yaw.py::_BENCH_2026_08_23`, so a
      config drift or a re-mount breaks the suite. The confirmation placement
      also caught a SIGN ERROR in the tool itself — `psi = ang - bearing` where
      `projection.py` wants `psi = ang + bearing`, identical at the runbook's
      own nose placement and wrong by 2x the bearing anywhere else. The tests
      now round-trip through `project_pixel` at 0/90/180/270/37/-125 rather
      than checking the tool's arithmetic against itself.
      Re-measure after ANY camera re-mount — SITL cannot see this.
- [x] **`sync_core.sh` now CHECKS its own hand-maintained tools list.** Writing
      that tool exposed the trap the script's header already described: `tests/`
      is copied wholesale, `tools/` is an allowlist, so the new test crossed to
      aavc-comp and the tool it imports did not — the comp suite stopped at
      "1 error during collection", 630 tests refusing to run over one missing
      import. Same shape as the 11 failures recorded there from 2026-08-21.
      The sync now scans the copied tests for `tools/` imports and FAILS naming
      the file to add. Verified by withholding the tool and watching it fire.
      It sets both the projection bearing (a 90° error reports a frame-edge pad
      ~13 m from where it is at the 12 m sweep — the aircraft flies to empty
      grass and the cluster never confirms) and the heading the sweep holds.
      ⚠ **SITL cannot catch an error here, and cannot catch the blur either:**
      the gz camera shares `CameraModel`'s mounting assumption, and it renders
      instantaneously — a simulated shutter has no exposure time, so rotational
      and translational smear do not exist in sim by construction. That is the
      whole gap between "G4′ 4/4 in SITL" and "2/6 on the field".
- [ ] **There is no KMITL SITL world, and the config now says so.**
      `sitl/launch_sitl.sh` spawns on `kmutnb_skyfield.sdf` at the practice
      L&R, and `tools/gen_geo.py` holds the KMUTNB pitch geometry only — so a
      SITL run against `kmitl_config.yaml` flies a route 31.5 km from where the
      vehicle sits. Treat that config as REAL-FLIGHT only; do SITL work in the
      practice repo. If the 28-Aug survey is worth simulating, the world's
      `<spherical_coordinates>`, `PX4_HOME_*` and `site.center` move together
      or the same class of bug comes back wearing a different hat.
