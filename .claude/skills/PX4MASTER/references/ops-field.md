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
time**; the mission does not fight a takeover (guard at the exception layer —
see open items). Between attempts: re-open the launcher icon (clear_state
runs first, by design).

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
- [ ] 🔴 **Zombie mission RE-ARMED the aircraft after pilot takeover**
      (2026-08-21 flight: ~8 min after takeover+disarm the stuck mission
      loop called arm_and_takeoff on the parked aircraft — every command
      timed out only because the FC was already powered down). TOP code fix:
      the takeover/disarm must TERMINATE the mission loop at the
      DroneCommander layer (the peer repo's `_guard()` pattern). Field rule
      effective immediately: **after ANY takeover, stop the mission (⏹)
      BEFORE approaching the aircraft.**
- [ ] **GCS must show pads AS THEY ARE FOUND (operator request 2026-08-21)**:
      in-flight WiFi death kills the mission_status.json sync path, and the
      radio beacon only broadcasts CONFIRMED pads — so the operator saw
      nothing while markers 4,5 were identified live. Fix chain: beacon
      broadcasts IDENTIFIED ids too (e.g. `AAVC ids=…` line) + GCS padbox
      lights an intermediate state (identified=orange, confirmed=green) from
      `_parse_beacon`. Verify MAV_1_FORWARD is ACTIVE (post-reboot) on the
      same bench pass.
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
- [ ] **CAMERA FOCUS bench gate** (the new #1 — see SKILL.md shortlist and
      docs/evidence/ulog_review_2026-08-21.md finding 1).
- [ ] `MPC_THR_HOVER` 0.5 → 0.58 — measured true hover ≈0.60 (motors mean,
      flight 3) while the board holds the 0.5 default and HTE logged NaN;
      AWAITING OPERATOR APPROVAL, then write + re-verify.
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
- [ ] PILOT-takeover guard: move from the exception catcher to the command
      layer (`DroneCommander` raising from every motion method — flag on the
      object that owns the sending). Peer repo has the pattern. The 2026-08-20
      zombie orchestrator (kept sending gotos to a disarmed aircraft for
      3.5 min) is the motivating incident.
- [ ] Watchdog ceiling cross-check against TFmini below 12 m (would have
      voted down flight 2's phantom breach) — safety-code change, own review.
- [ ] `kmitl_config.yaml` battery-block comment mirror (only power narrative
      was synced 2026-08-20).
- [ ] Automated cross-file battery-endpoint consistency check (manual rule in
      power-battery.md until then).
- [ ] docs/REAL_FLIGHT_GCS.md + docs/FLIGHT.md still describe the pre-🚀
      stack-start; status_beacon.main() age logic untested;
      test_status_beacon positional indexing brittle (code-review backlog).
