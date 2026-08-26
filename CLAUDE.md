# CLAUDE.md — AAVC 2026 (lightweight build)

Project-local context for Claude Code sessions in this repo. This is the
**lightweight rebuild** of the AAVC competition system — deterministic, classical
CV, no LLM. The heavier reference repo lives at `../aavc-2026`; read it for
anything not yet copied here, but do **not** pull its heavy stack back in.

---

## 0. THE DATE (rules V1.3 pp. 9-10 — read it off the PDF, not off memory)

**The competition is 28-30 August 2026 at IAAI KMITL (Terminal 1).**

| Date | What happens |
|---|---|
| **Fri 28 Aug** | 08:00-09:00 registration · 09:00 briefing · **safety inspection** · **13:00-18:00 flight trial** (15 min slots, first-come-first-served, repeat if time allows) · 18:30 opening dinner |
| **Sat 29 Aug** | **Scored flight day 1** — assemble at Terminal 1 before 07:30 |
| **Sun 30 Aug** | **Scored flight day 2** · 18:30 awards + closing |

Three things follow from this and they shape every plan in this file:

1. **28 Aug is not a practice day, it is the LAST day to fix anything.** The
   aircraft has to arrive already flight-ready; the trial slots are for
   learning the real field, not for debugging.
2. **A failed safety inspection eats our own flying time** — the committee
   feeds back issues and the fix has to be re-checked *during* the trial
   window.
3. There are **two more scored components besides flying**: the safety
   inspection and a **15-minute presentation + 5-minute Q&A** on 28 Aug (slots
   announced on arrival). The technical-report score also feeds the overall
   score and already decided our time slot.

This section exists because the date was in the PDF in this repo's root and
nowhere in this file, so a session that needed it could not find it.

---

## 0b. Full-system review, 2026-08-23 — the KMITL window does not fit

Reviewed end to end against the rules PDF: board params, both mission repos,
the path manager, the CM4 device code, and both GCS field maps. The system is
not broken — geometry matches the PDF exactly, the two repos have not drifted,
energy has 3x margin, 718 tests green. **What is wrong is TIME**, and only at
KMITL: the competition field is twice the practice field and several numbers in
`kmitl_config.yaml` were still calibrated on the small one.

**The number that matters is 21 seconds.** A full-sweep four-egg KMITL sortie,
built from measured parts (G4' takeoff/landing, real transit distances, the
sweep from `search_pattern.py` on the shipped config):

| part | 0.44 (ships) | 0.30 |
|---|---|---|
| takeoff / ingress 171 m / **sweep** / 4 serves / egress / land | 17 / 57 / **565** / 320 / 57 / 35 | 17 / 57 / **407** / 320 / 57 / 35 |
| **total of the 1200 s window** | **1051 s** | **893 s** |
| remaining when egg 4's gate runs (`can_start_delivery` refuses below 300) | **321 → 21 s of slack** | 479 → 179 s |

Both deliver 4/4 when nothing goes wrong. The difference is what one bad gust,
one re-align or one decode visit costs: at 0.44 anything 22 s over plan drops an
egg.

- **F-01 `overlap_frac` 0.44 -> 0.30 — TESTED AND CHANGED 2026-08-24.** The
  operator held 0.44 out of doubt the camera would find the pads at the real
  field, and asked for it to be settled in Gazebo with numbers first. Flown head
  to head at KMITL OPTICS (competition envelope, 12 m sweep so the marker is
  28.2 px, same pad seed 1301, only overlap different):

  | | 0.30 | 0.44 |
  |---|---|---|
  | legs / pads found / ids / eggs | 5 · 6/6 · 6/6 · 4/4 | 6 · 6/6 · 6/6 · 4/4 |
  | release error | 0.09-0.22 m | 0.15-0.20 m |
  | mission time | **536 s** | 647 s |

  Identical coverage, identical delivery success, **111 s faster** on that field
  and a computed **158 s** on KMITL's larger polygon. Decode was separately
  measured at EVERY across-track offset out to 94% of the half-swath, so wider
  spacing never puts a pad where the detector fails. Evidence:
  `docs/evidence/sweep_overlap_2026-08-24.txt` + both audit trails.
- **F-02 `sortie_cost_s` — CORRECTED 2026-08-24.** KMITL 420 -> **900**,
  practice 240 -> **690**. Both were wrong the same two ways: the arithmetic
  predated the overlap change, and both sized ONE serve while `eggs_aboard=4`
  flies four. Rebuilt from parts (KMITL: 17 takeoff + 57 ingress + 407 sweep +
  4x80 serves + 57 egress + 35 land = 893, rounded up), cross-checked against
  the 536 s SITL run. The KMITL gate now needs 1110 s of 1200 and still fits,
  and the last egg's slack over the 300 s floor went **21 s -> 179 s**.
- **F-09 the gz camera was still at yaw 0 — FOUND BY THAT TEST, FIXED.**
  2026-08-23 set `mount_yaw_deg: 180` in both configs (correct for the real
  aircraft) while `sitl/models/eft_x6100/model.sdf` still mounted the SITL
  camera at yaw 0, so every SITL projection came out point-mirrored. The first
  0.30 run delivered **0 of 4 with all six pads decoded and every id correct**,
  positions 3.7-20.2 m off, the aircraft flying to empty grass at each pad —
  the G7-attempt-1 signature, reproduced somewhere for the first time. The
  camera pose is now `0 0 0.10 0 1.5708 3.14159`. ⚠ This retires a standing
  warning in this file: SITL and `CameraModel` **no longer share** the mounting
  assumption, so a future mount error shows up in sim instead of at the field.
- **F-03 KMITL asks the decoder for a SMALLER marker than the flight that already
  failed.** KMUTNB sweeps at 8 m = 42 px = 7.1 px/module and decoded 2 of 6.
  KMITL must sweep at 12 m = 28 px = **4.7 px/module**. Flying lower does not
  rescue it: the rules floor the search at 10 m AGL, which is still only
  5.6 px/module. The bench decodes to 14 m statically, so this is the in-flight
  BLUR problem and nothing else — `tools/hover_decode.py` is the instrument for
  closing it.
- **F-04 CLOSED by the operator 2026-08-24 — sortie count is NOT scored.**
  The judges care about three things only: **is the egg delivered accurately, is
  it delivered inside the time window, and is the operation safe** (and what the
  safety provisions actually are). One flight or two changes nothing on the
  scoresheet, so `eggs_aboard=4` stands and the "Repeating Delivery" line is not
  a reason to split the mission. Recorded here because the rules text alone
  reads the other way.
- **F-05 standby-area telemetry — ACCEPTED 2026-08-24, procedural.** Rules p.13
  forbids "activating radio control equipment and other telemetry while in the
  standby area", and the CM4 raises its own WiFi AP at boot with the console
  ssh-starting the beacon as soon as it answers. **Do not connect the pack until
  the launch point**, and keep the console shut while waiting. Operator agreed
  independently. Confirm the interpretation at the 28-Aug briefing.
- **F-06 the RC-loss drill and ELRS "No Pulses" are still not done.** 2026-08-23
  closed the DIAGNOSIS (a held kill switch, not the link) — the net itself has
  never been tested.
- **F-07/F-08 stale text, FIXED 2026-08-23.** PX4MASTER's summary claimed the comp
  config "deliberately keeps `EKF2_HGT_REF=1`" — both configs ship 0, and 1 is the
  setting that RTH'd a flight on 2026-08-20. And `kmitl_config.yaml` carried the
  PRACTICE field's altitude reasoning ("ceiling now 10 m", "legal band
  [search_floor_m 2.5, ceiling 10]"); at KMITL the band is [10, 20]. Same class as
  the 2026-08-22 `site.center` incident: practice-field text inside the
  competition file, every number plausible, describing another field.

Verified CORRECT and needing nothing: transit P1-P3, controlled airspace and
search area all match the PDF coordinate-for-coordinate; the GCS competition map
matches the flight config; the two repos' flight cores are identical; KMITL needs
4275 mAh of 12750 usable.

---

## 1. Mission (locked — official Rules & Regulations V1.3, July 2026)

An autonomous **PX4 hexacopter** (EFT X6100 frame; Pixhawk 6X + Raspberry Pi CM4
companion, **no RTK** so GPS is coarse) delivers fragile egg cargo to
**landing pads matching the committee-ASSIGNED ArUco marker ids** at the real
**IAAI KMITL** field. **Six** pads are physically placed (2026-07-24
event-briefing override of the PDF's "up to four" — `docs/RULES_AAVC2026.md`
"Event-briefing override"): **1×1 m white**, black **circle ⌀750 mm**,
central **400×400 mm ArUco marker** (`DICT_4X4_50`, **ids 1–6**); the
committee **assigns FOUR** per team, the other 2 staying permanent
distractors. A **FLIGHT** (one arm→disarm cycle — `state.sortie_index` is
the flight counter; "sortie" and "flight" are used interchangeably in this
repo) carries up to `eggs_aboard` eggs — **4 by briefing default**, i.e. all
four assigned eggs in ONE flight (`eggs_aboard=1` is the pre-briefing
one-egg-per-flight model, still a fully supported one-integer rollback).
Each assigned pad is its own **DELIVERY**, scored independently. Per
flight: takeoff at the Launch & Recovery site → **mandatory transit
P1→P2→P3 at 20 m** (scored per point, both directions) → search the **search
area** (10–20 m band; the assigned ids are entered by the operator at each
GO) → for each assigned id in turn: **land ON the pad** → **release that
delivery's egg after touchdown** (`payload_id` 0..3 → latch servo on
**AUX 4/1/2/3** as wired) → egress transit → land at L&R → **disarm** → resupply → next
flight (≤4 flights inside the **20-minute operation window** — the briefing
default needs only ONE; per-minute penalty after). Ceiling **20 m AGL**;
below **10 m only** for the delivery descent over the pad.
Fast-but-safe. Full rules digest: `docs/RULES_AAVC2026.md` (source:
`AAVC2026_RulesAndRegulation_V1.3_140769.pdf` in the repo root — V1.3 is
editorial-only over V1.1: no flight-rule value changed; it adds DMS coordinates,
the cargo-box spec (heart box ~16×7×18 cm, 300-gsm), an explicit "record AND
transmit" imaging line, and a "subject to be changed" watermark — which is
why the 2026-07-24 event-briefing override governs pad count / assignment /
eggs-per-flight over the PDF's own printed numbers).

(Superseded history: the 2026-06-15 interim decision — 5 supine red human
dummies, land-BESIDE — was replaced 2026-07-03 by the official V1.1 rules
above. Anything below dated earlier that says "human"/"beside"/"5 payloads,
single sortie" is history, not current design.)

## 2. Locked Decisions — do not relitigate

- **Land-ON-pad, touchdown-gated release** (rules V1.3): the aircraft descends
  in altitude rungs over the assigned pad, lands ON it (final rung tolerance
  0.2 m — a 1 m pad; 0.35 m is the 3 m rung), and opens the egg hold only after
  touchdown confirms
  (`settle_after_land_s=2.0`). Scoring pays for "landed on the pad BEFORE
  releasing" and an intact egg, so the egg is **never released airborne**: if
  telemetry reads clearly airborne the release is SKIPPED (audited). This
  **reverses the old "an attempt beats no drop" doctrine** —
  `gps_fallback=False` and the **id-verified LAND gate** (`require_id_votes`)
  refuse to land unless the assigned id was actually decoded during the
  approach (climb + defer instead). Wrong-id pads NEVER steer the descent.
- **FLIGHT ⊃ DELIVERY: `eggs_aboard` eggs per flight** (rules V1.3 + the
  2026-07-24 event-briefing override): a **FLIGHT** is one arm→disarm cycle
  (`state.sortie_index` is the flight counter — kept as one field rather than
  renamed/churned across every caller, since "sortie" already meant exactly
  this; the AUDIT TRAIL spells it out as FLIGHT/DELIVERY explicitly, §5). A
  **DELIVERY** is one pad served within a flight (`state.delivery_index`,
  1-based across the WHOLE mission, NOT reset per flight). Each flight
  carries up to `mission.eggs_aboard` eggs (briefing default **4**: all four
  assigned pads in ONE flight; `eggs_aboard=1` is the original
  one-egg-per-flight rollback, still regression-tested). The physical
  release is now `payload_id` 0..eggs_aboard-1 (no longer always 0) →
  `MAV_CMD_DO_SET_ACTUATOR` actuator set = AUX pin (`PWM_AUX_FUNCn = 300+n`;
  PX4 has NO `DO_SET_SERVO` handler — the old "AUX 9/10/11/12" plan addressed
  a command that was never implemented). The rack is **wired AUX 4/1/2/3** for
  front-left / rear-right / front-right / rear-left (as-wired 2026-08-15), and
  the mission releases in that diagonal order, so
  `connection.drop_servo_channels: [4, 1, 2, 3]` maps `payload_id` → channel
  (`ConnectionConfig.actuator_index`; empty list = the old
  `drop_servo_channel + payload_id` progression). `drop_payload_count=4`;
  a single latch was the `eggs_aboard=1` case. Table + QGC bench sheet:
  `docs/SERVO_AUX_MAPPING.md`. The mission-global `stop_index` (`= delivery_index - 1`) — **not**
  the flight/sortie index — keys the release idempotence ledger
  (`state.dropped_stops`), so two deliveries inside the same flight can
  never collide. A **per-delivery abort gate** (time + battery —
  `TimePolicy.can_start_delivery`, plus a battery margin above
  `rth_battery_pct`) runs before EVERY delivery inside a flight, not just at
  the flight gate: failing it skips the remaining eggs in that flight
  (audited `DELIVERY abort: flight n skipping remaining ids=…`) and heads
  home with them still aboard, rather than starting a descent the budget
  can't finish. Between FLIGHTS the vehicle lands at L&R and **disarms**
  (resupply crew approaches); `COM_DISARM_LAND=-1` is retained so a
  **mid-flight pad landing (between deliveries) stays ARMED** (no re-arm
  over the field — which also pins PX4 home to the launch point while
  airborne).
- **Mandatory transit corridor** (rules Table 1): every sortie flies
  P1→P2→P3 at **strictly 20 m** outbound and P3→P2→P1 back — scored per
  coordinate passed, so each pass/miss is audited (`TRANSIT_PASS`/`_MISS`).
  The final approach home is an **explicit goto + land, NOT RTL** (kept from
  2026-06-11: PX4 re-captures home at every re-arm). `RTL_RETURN_ALT=20` is
  pinned so any FAILSAFE RTL (geofence/datalink/watchdog) stays at the ceiling
  — PX4's default 60 m would bust it.
- **Blind visual search + cross-flight pad registry** (2026-06-11, reshaped
  2026-07-03, chunked per flight 2026-07-24): pad coordinates are **unknown
  at takeoff** — the committee only assigns marker ids, up to `eggs_aboard`
  of them per FLIGHT (the ordered **mission-id queue**
  `state.assigned_id_queue`, sliced into flight-sized chunks by
  `mission_brain/flights.py::chunk_flights`; set from the GCS queue editor or
  seeded by headless `--assigned-ids`; a manual id in the GO request overrides
  the WHOLE flight only at `eggs_aboard=1` or when the queue has no chunk for
  it — otherwise the queue's chunk wins). A flight with ANY assigned id not
  yet registered flies a
  **boustrophedon sweep** (`mission_brain/search_pattern.py`) over the **search
  area polygon** (not the whole airspace), decoding EVERY pad it sees into the
  registry (`orchestrator/target_tracker.py`, clusters keyed by decoded id,
  k-decoded-vote confirmation). **Finish-sweep-then-serve** (operator
  2026-07-03): the sweep runs to completion — early-stop only once `max_pads`
  distinct ids are confirmed — so a later flight whose WHOLE chunk is already
  registered flies **direct** (no sweep). Undecoded white-pad candidates are revisited at the 10 m
  floor to read their ids; a pad decoded but still short of `confirm_votes`
  (identified-but-unconfirmed) gets a cheap **vote top-up visit** instead of a
  re-sweep (2026-07-08, `TargetTracker.identified_unconfirmed`) — falling
  through to the sweep if the visit doesn't confirm. The single **nadir**
  camera (1280 px wide — the decode needs the pixels) is the SOLE control
  authority (the oblique cue camera was retired 2026-07-15 with the
  single-OV9281 hardware decision). The plan is rebuilt live per sortie
  (`mission_brain/live_plan.py`).
  ⚠ **The search phase HOLDS ONE HEADING** (`SearchPlanSpec.sweep_yaw_deg`,
  passed as `yaw_deg` on all four search-phase `goto`s, 2026-08-22). Every
  `goto` used to send a NaN yaw, so PX4 fell through to `MPC_YAW_MODE` — the
  factory **0 = "towards waypoint"** on all three 2026-08-20 flights — and
  turned the nose at each sweep waypoint: the commanded heading walked a full
  circle at the 25 °/s cap, **867° of yaw in 122 s** (ULog `08_11_09`), and the
  body-fixed camera spun with it (1 of 457 frames decoded). A finite yaw in the
  goto **beats the param** — PX4 reads the triplet yaw before ever consulting
  `MPC_YAW_MODE` (`FlightTaskAuto.cpp:490-501`) — so the fix holds even if
  `MPC_YAW_MODE=5` never lands (its PX4 metadata says `@max 4` while the enum
  defines 5; it is in `preflight_params.py::PINNED` for exactly that reason).
  The heading is DERIVED: `leg_bearing − cameras.nadir.mount_yaw_deg`, then
  flipped 180° if that would fly the leg BACKWARDS — both headings put the
  camera's WIDE (1280 px) axis across track (the footprint is a rectangle, so
  `swath_m` is identical either way) and the plan takes the nose-first one.
  Without that flip the measured 180° mount would send the aircraft down every
  sweep leg in reverse. ✅ **`mount_yaw_deg` MEASURED 2026-08-23 = 180** — the
  camera is bolted **UPSIDE DOWN**, image row 0 looks at the TAIL. It had
  shipped as `0.0`, an ASSUMPTION, and that was wrong by **~175° in EVERY
  direction**: every pixel→lat/lon fix came out point-mirrored about the
  aircraft, which is precisely the shape of a sweep that SEES pads and never
  re-acquires them (G7 #1: 2 of 6 decoded). Four placements of one floor object
  round the raised, levelled airframe (nose / right / tail / left) each solved
  it alone — 180.3 / 187.4 / 183.4 / 186.1, mean 184.3, snapped to the bolted
  180; the four pixels are kept as data in
  `tests/test_measure_mount_yaw.py::_BENCH_2026_08_23`. Found on the way, same
  day: `tools/measure_mount_yaw.py` computed `psi = ang − bearing` where
  `projection.py` wants `psi = ang + bearing` — invisible at the documented
  nose (`bearing=0`) placement and wrong by `2·bearing` anywhere else, so ONLY
  a confirmation placement off the nose could expose it (the right-wing frame
  read 7° where the aircraft's own projection says 187°). ✅ The tail-first side effect is CLOSED
  (`search_pattern.py` now takes the nose-first perpendicular, above).
  ✅ `overlap_frac` is back to **0.30** (2026-08-24) — the operator held it at
  the belt-and-braces 0.44 while unsure the camera would find the pads at the
  real field, and asked for the question flown in Gazebo before it moved. It
  was: same pad seed, KMITL optics (12 m sweep, marker 28.2 px), only overlap
  differing — **6/6 pads, 6/6 ids, 4/4 eggs on BOTH**, releases 0.09-0.22 m vs
  0.15-0.20 m, and 536 s against 647 s. Coverage identical, 111 s faster there
  and ~158 s at KMITL. Decode was measured separately at every across-track
  offset out to 94% of the half-swath, so the wider spacing never puts a pad
  where the detector fails — which was the mechanism the doubt was about.
  0.44 only ever insured against an UNMEASURED mount, and that cannot occur
  now. Evidence `docs/evidence/sweep_overlap_2026-08-24.txt`. ⚠ SITL used to be blind to all of this because the gz camera shared the
  mounting assumption — **no longer**: `sitl/models/eft_x6100/model.sdf` was
  rotated to `0 0 0.10 0 1.5708 3.14159` on 2026-08-24 so sim carries the same
  180 the airframe does, and a mount error now fails in SITL. (SITL still
  renders with no exposure time, so smear does not exist in sim — the blur
  problem stays a field-only question.)
  SITL ground truth (`/tmp/aavc_targets.json`, now with `marker_id`) is used
  ONLY for the post-flight audit + `tools/verify_flight.py`, **never planning**.
- **Single long-running orchestrator across the window** (operator 2026-07-03):
  one process flies all flights; the **per-flight preflight gate** holds in
  PREFLIGHT before every launch. With the 4-of-6 queue set the GO is ONE click
  — it confirms the eggs + crew clear; the backend then resolves THIS
  flight's chunk of up to `eggs_aboard` ids from the queue
  (`chunk_flights`) — a manual id in the GO request only overrides the WHOLE
  flight at `eggs_aboard=1` or when the queue has no chunk for it, otherwise
  the queue's chunk wins (same rule as the search-registry bullet above);
  `/api/cmd/preflight/go` 409s when no assignment exists or the window can't
  cover another flight unless FORCEd — the overtime penalty is the operator's
  call. The window clock starts at the FIRST GO (`state.start_window()`).
- **Classical CV only.** Landing pad = **ArUco decode** (`cv2.aruco`,
  `DICT_4X4_50`, tuned params + an ROI ×4 upscale booster — the 400 mm marker
  is ~18 px at the 12 m sweep) fused with a **white-pad blob cue** (low-S/high-V
  square with a dark marker centre) for search-altitude acquisition
  (`vision/detectors/aruco.py`, `find_landing_pads` → `PadHit` with
  **marker-equivalent `radius_px`**, one 0.2 m size prior end-to-end).
  `render_pad_bgr` is the single pad renderer shared by the SITL textures
  (`tools/gen_pads.py`), the HITL synthetic camera, and the tests. NO torch /
  ultralytics / llama-cpp / moondream / open3d / SLAM / mapping / planning /
  wizard / 3D-map / ESC-cal. If you drop code, **delete it** — no dead imports.
  (Tuning/autotune are NOT in flight — offline pre-flight module below.)
- **V1.3 airspace watchdogs** (`orchestrator/safety.py`): geofence breach → RTH
  (unchanged); **no-fly-zone entry → RTH**; **ceiling** warn >20.5 m / RTH >22 m
  sustained; **search floor** <10 m advisory outside the delivery-descent
  phases. The no-fly polygon + L&R coordinates are APPROXIMATE (figure-only in
  the rules) — config-tunable, re-measure at the event briefing.
  ⚠ **`GF_ACTION` FIXED 2026-08-16 — it was Hold, not Return, for its whole
  life.** `set_geofence_action_rtl()` wrote **2** while its own name, value
  comment, error text and this file all said RTL. PX4's enum
  (`navigator/geofence_params.c`) is `0 None · 1 Warning · 2 Hold · 3 Return ·
  4 Terminate · 5 Land`, so the FC-level breach response was "stop and loiter"
  **at the breach point, outside the fence** — the same outcome the design had
  already rejected once when it moved off LAND-in-place ("left the vehicle DOWN
  outside controlled airspace"). The companion check carried the rule alone;
  nothing was unprotected, but the layer sold as working *without* the CM4 did
  not. Now **3 (Return)**, verified live (fresh SITL rootfs read the factory
  default 2 = Hold, then held 3). Any future value must stay in {3 Return,
  5 Land} — never 2 — because PX4 answers our own `goto_location`
  (DO_REPOSITION) with AUTO_LOITER = MAVLink **HOLD**, the mode the mission
  flies in end to end, so a Hold failsafe is indistinguishable from normal
  flight and cannot be detected mode-side. Lesson worth keeping: the readback
  gate **passed** the whole time — it proves a value was stored, never that the
  value means what the caller thinks.
- **One motor out — REMOVED 2026-08-17, not deferred.** This airframe's ESCs
  are **PWM-only with no telemetry lead** (operator, at the bench: "ESC ผมไม่มี
  สาย UART หรือ TELEM ครับ มีแต่ PWM"), measured the same day with the motor
  map restored and all six rotors verified spinning: `DSHOT_TEL_CFG=0` and
  **zero `ESC_STATUS`** during a motor test even after a
  `SET_MESSAGE_INTERVAL(ESC_STATUS)` the FC ACCEPTED — the FC has nothing to
  send, rather than the link failing to carry it. Per-motor current is the
  ONLY signal either detector can use (PX4's `FD_ACT_EN` watches
  current-per-throttle; the companion read the same over ESC_STATUS), so both
  layers were machinery that could never fire. Deleted rather than left dead:
  `DroneCommander.set_motor_failure_failsafe`, `safety.py::_check_motor_health`,
  the `failsafes.ca_failure_mode` / `act_fail_act` and `motor_health:` config
  blocks, and their tests. `esc_current_a` survives as a dashboard display
  field only — no flight path reads it. **Restoring this starts with buying
  telemetry-capable ESCs, not with a parameter.** What the aircraft has
  instead is the safety pilot, and that is enough to fly: PX4's sequential
  desaturation mixes roll/pitch/thrust before `mixYaw()`
  (`ControlAllocationSequentialDesaturation.cpp`; `CA_METHOD` defaults to
  Automatic, which selects it for every multirotor), so a hexa on five rotors
  holds attitude and thrust and gives up only YAW — measured in SITL as
  `unallocated_torque` yaw climbing to 0.25 with the allocator still solving
  for six (2026-08-16, sihsim_hex `failure motor off -i 3`). It keeps flying
  and starts spinning, which is human-recoverable on POSCTL but ends the
  mission either way: with yaw uncontrolled the body-fixed nadir camera spins,
  and both the ArUco decode and `vision/projection.py`'s attitude-composed
  pixel->lat/lon fall apart.
- **System-ID + Autotune: GONE.** It was the one deliberately re-added
  exception (`tuning/`, `dashboard/tuner.py`, `orchestrator/sysid_sweep.py` — an
  offline numpy-FRF tuning aid). The sweep, the dashboard tool and the tests went
  on 2026-08-15 (PX4's own autotune replaces them); the leftovers went on
  2026-08-17 — the empty `tuning/` package, its `pyproject` entry,
  `sitl/launch_tuning.sh` (it launched a `--mode tuning` the orchestrator no
  longer has), and `SafetyWatchdog(enforce_mission_limits=…)`. That last one is
  the reason to care: it was a switch that turned the geofence, no-fly, ceiling
  and mission-clock checks OFF, kept alive for a tool that no longer existed and
  called by nothing. **Those checks are not optional any more.** `pyulog` stays
  an optional import in `tools/verify_flight.py` (`pip install pyulog`) — there
  is no `[tuning]` extra to install.
- **No LLM, no cloud, no network in flight.** The mission is deterministic
  (`orchestrator/mission.py::run_delivery_mission`). AAVC bans internet/4G (= DQ).
- **EFT X6100 hexacopter + Pixhawk 6X + Raspberry Pi CM4** (airframe swapped
  2026-07-22 from the 700 mm X-quad; differs from the reference repo's
  6C-Mini/Jetson). Board state read live 2026-07-22: PX4 **1.17.0**,
  `SYS_AUTOSTART=6001` (Generic Hexarotor X), `CA_ROTOR_COUNT=6`, sensors +
  radio + flight modes calibrated. ✅ **`PWM_MAIN_FUNC1..6` RESTORED to 101..106
  on 2026-08-17** (written over the NOMAD radio link, readback-confirmed, then
  `ACTUATOR_TEST` M1..M6 one at a time with props off — operator watched all six
  arms spin in order and in the correct direction). They had read **0** on
  2026-08-16 (parallel session, cross-checked against an older ULog) = motors
  unassigned to outputs, aircraft unflyable. The cause was never found, so treat
  it as a REGRESSION that can recur, not a closed item: **re-read these params
  before every field day rather than trusting this line.** One suspect worth
  ruling out — selecting an airframe in QGC's Airframe tab reloads that frame's
  defaults over the Actuators assignment, so a "model" re-pick can clear the
  motor map; the values survived a QGC session on 2026-08-17, but that is one
  data point, not a cleared suspect. Same session found `PWM_AUX_FUNC1..4` sitting at
  402/405/409/410 (**RC passthrough** — two egg latches were wired to the roll
  and yaw sticks, i.e. the eggs would have released while the aircraft banked);
  now 301..304, verified across a reboot. Power (**REWIRED 2026-08-16** — the Holybro PM03D failed and is
  OUT; since **2026-08-20 a Holybro PM02D feeds the Pixhawk — FC/avionics
  ONLY**, replacing the interim converter): the
  **motors run from a SEPARATE board the FC cannot sense**. So any current the
  FC reads (~0.7 A via the PM02D) is avionics draw, not the ~35-43 A of
  flight, and a current-fused gauge on that wiring fails OPTIMISTIC and
  silently — the PM02D's current output must NEVER feed coulomb counting. The switch to the voltage
  branch is **`BAT1_CAPACITY=-1`** (`lib/battery/battery.cpp`
  `estimateStateOfCharge` takes the voltage-only `else` only when capacity
  <= 0) — **NOT** `BAT1_I_CHANNEL=-1`, which is a no-op: -1 means "board
  default" and the default already IS -1. `BAT1_V_DIV`, `BAT1_V_EMPTY` and
  `BAT1_V_CHARGED` then carry the WHOLE measurement. `V_DIV` is **CLOSED**
  (operator 2026-08-16: multimeter vs the live GCS reading, taken AFTER the
  converter swap, so it belongs to this wiring); `V_EMPTY`/`V_CHARGED` are
  **not**, and a correct
  voltage still yields a wrong percentage if they are, because the whole gauge
  is `interpolate(cell_v, v_empty, v_charged)` and every threshold is a % of it.
  Because
  `calculateStateOfChargeVoltageBased` only load-compensates when current > 0,
  the gauge sags under thrust and rebounds, so `safety.py` requires both
  battery thresholds to hold for `battery_sustain_s` (5 s) before acting. **Absolute
  height is BARO at BOTH fields** (`EKF2_HGT_REF=0`, operator 2026-08-23;
  practice moved 2026-08-20, competition followed): on PX4's default `=1` (GPS)
  the baro-vs-GPS divergence measured **10.8 m peak-to-peak** inside one flight,
  the reported AGL inflated to 12.0 m while the aircraft physically held ~8.5 m,
  and the ceiling watchdog — correct on its inputs — RTH'd a flight that was
  tracking transit to 1.7 m. NOT `=2` (range): that makes the local origin ride
  ground level, so a shed cargo box or a person under the beam would move
  "down". ⚠ `reboot_required` — PX4 stores a new value immediately while the
  estimator keeps running on the OLD reference, so a read-back says "held" and
  proves nothing; it is deliberately NOT in `verify_envelope_pins`, and
  `tools/preflight_params.py` checks it as a **`BOOT_LATCHED`** STOP against the
  field config in force. Height
  aiding for the real bird is a **Benewake TFmini-S** downward lidar
  (`EKF2_RNG_CTRL=1` pinned, conditional aiding below `EKF2_RNG_A_HMAX=7.0` m
  pinning the final metres; its `SENS_TFMINI_CFG` port is chosen at the bench).
  **Optical flow was CUT 2026-07-22** — no flow module in the kit, so
  `EKF2_OF_CTRL` is pinned to 0. The single nadir camera (Meige OV9281 UVC, mono
  global-shutter) is **HARD-MOUNTED looking down — no gimbal** (operator
  2026-08-16; `gimbal.enabled=false`). It therefore pitches with the body, which
  promotes the roll/pitch composition in `vision/projection.py` from a
  refinement to load-bearing (a translating multirotor holds 10-15°;
  uncompensated that is ~alt·tan(tilt) ≈ 2 m at 12 m). Both flight call sites
  pass attitude and a test pins that they keep doing so.
  SITL mirrors this: PX4 **v1.17 worktree** `~/PX4-Autopilot-v1.17` (branch
  `aavc/sitl-v1.17`) + airframe `22000_gz_eft_x6100` + model
  `sitl/models/eft_x6100`, whose rotor table mirrors the board's own `CA_ROTOR*`.
  ⚠ NEVER export `PX4_SIM_SPEED_FACTOR` at 1x — PX4 1.17 answers it with a
  `set_physics` call that leaves the gz world at ZERO GRAVITY.
- **Trimmed Svelte GCS** (`dashboard/`): live map (transit corridor + no-fly
  zones drawn), camera, command bar, **per-sortie GO + the 4-of-6 mission-queue
  editor** (click ids in sortie order; flown slots lock; each edit POSTs
  `/api/cmd/mission_ids`), sortie/pad chips, and the **Confirmed-pads readout**
  — every pad id is shown as the **ArUco marker glyph itself** (inline SVG,
  `dashboard/web/src/lib/aruco-glyphs.ts`, baked from the detector's own
  `DICT_4X4_50` by `tools/gen_aruco_glyphs.py`; the numeric id stays as a
  caption for radio calls) so the operator matches the committee's card
  picture-to-picture instead of translating it into a number
  (id + obtained lat/lon — a V1.3 scoring line). Two views: **Flight** +
  pre-flight **Tuning**.
- **MAVLink URL:** `udpin://0.0.0.0:14540`. Bare `udpin://:port` is rejected.
- **Python 3.12**, type hints, async throughout orchestrator + mavlink_adapter
  (MAVSDK is async-native). `loguru` for logging; no `print()` in flight paths.

## 3. Module Map

```
mission_brain/   live_plan.py (per-flight plan: transit + search + serve + LAND),
                 profile.py (transit_alt/search_floor/max_sorties/eggs_aboard),
                 flights.py (chunk_flights/max_flights_for — the queue-to-
                 flights split), schemas.py,
                 search_pattern.py (boustrophedon sweep of the search area)
orchestrator/    main.py (per-sortie gate factory, --assigned-ids), mission.py
                 (run_delivery_mission: gate → transit → sweep/registry → serve →
                 egress → land+disarm; 1 Hz TELEM audit sampler), tactical_align.py
                 (land-ON descend gate + id-verified LAND gate + touchdown-gated
                 release), vision_worker.py (multi-pad fixes + frame-age gate),
                 frame_recorder.py (record-half JPEG trail), drop_trajectory.py,
                 safety.py (+ceiling/no-fly/floor + battery-NaN escalation),
                 constants.py (shared envelope thresholds), state.py (sortie fields,
                 start_window), audit.py (id-scored truth compare), preflight.py,
                 target_tracker.py (marker-id-keyed pad registry),
                 time_policy.py (sortie + serve reserves),
                 energy_policy.py (pack budget: usable mAh, per-sortie cost,
                 GO refusal + swap detection)
mavlink_adapter/ commands.py (DroneCommander/MAVSDK; drop_payload_count
                 defaults to 1, config drives it to 4 -> AUX 4/1/2/3 via
                 drop_servo_channels/actuator_index,
                 RTL_RETURN_ALT=20, RC-loss/battery failsafe pins), telemetry.py,
                 raw_subscriber.py (ESC/servo/consumed-mAh for the dashboard)
vision/          detectors/aruco.py (find_landing_pads + PadHit + render_pad_bgr)
                 + base.py, projection.py (pixel -> lat/lon)
dashboard/       FastAPI server (server/routes/commands/tuner/payloads/realtime/
                 command_proxy.py) + Svelte web/ (integration.py = the seam, §5)
sitl/            launch_sitl.sh (PX4_HOME = KMITL L&R), spawn_targets.py (6 pads,
                 seeded ids+positions in the search polygon; WGS84 ENU↔lat/lon),
                 gz_camera_bridge.py, hitl_synthetic_camera.py (pad faces),
                 aavc_config.yaml, models/landing_pad_id_{1..6}/ (baked by
                 tools/gen_pads.py), models/cargo_box/ (4 payload dummies on
                 detachable joints — Tier 1 belly mass + Tier 2 visible drop),
                 payload_detach_bridge.py (audit-tail -> gz detach on RELEASE,
                 make payload-bridge, optional/gz-transport-only);
                 HITL: launch_hitl.sh (jMAVSim), hitl_router.conf,
                 hitl_param_config.py (nsh param setter, `make hitl-params`)
tools/           gen_pads.py (pad models, decode self-checked),
                 gen_aruco_glyphs.py (GCS marker glyphs, decode self-checked),
                 gen_grass.py,
                 verify_flight.py (post-flight drone-response verifier — fails CLOSED),
                 landing_trial.py (SITL land-ON precision bench, `--set` A/B knob)
build_bom.py     repo-root Thai/English hardware-BOM builder
cm4/             launch_flight.sh (real camera) + launch_hitl.sh (synthetic cam,
                 CM4-in-the-loop) + aavc-flight/aavc-hitl.service (optional systemd)
docs/            RULES_AAVC2026.md, FLIGHT.md (G5+ real bird), HITL.md (runbook) +
                 HITL_CHECKLIST.md (bench sheet)
tests/           test_pad_detector.py, test_delivery_mission.py, test_live_plan.py,
                 test_tactical_align.py (land-ON + id gate), test_target_tracker.py,
                 test_preflight.py, test_safety.py, test_time_policy.py,
                 test_truth_audit.py, test_commands.py, test_hardware_gates.py, …
```

## 4. What was DROPPED vs the reference (`../aavc-2026`) and why

| Dropped | Why |
|---|---|
| Claude / Gemini / local-LLM planners | Mission is deterministic; offline-only |
| YOLO / ultralytics / torch / ONNX | Targets are ArUco pads → cv2.aruco decode |
| Moondream / open-vocab VLM advisor | No open-vocabulary search in this mission |
| mapping / orthomosaic / lidar3d / SLAM / kiss-icp / open3d | Not a survey mission |
| planning / ESC-cal / customparam | Out of scope; QGC covers cal |
| aircraft_bridge / twinboom / multi-airframe wizard | Single locked X-hexa airframe |
| httpx / rich / pillow / lxml / pre-commit / hypothesis | Unused by the lean core |
| supine-human detector + land-beside geometry (2026-07-03) | V1.1 target is the ArUco pad; land-ON |

**Re-added 2026-06-06, then dropped again 2026-08-15/17:** `tuning/` +
`dashboard/tuner.py` + `orchestrator/sysid_sweep.py`, the pre-flight System-ID +
Autotune module — PX4's own autotune replaces it (§2). `pyulog` is an optional
import in `tools/verify_flight.py`, never a flight-core dep.

Build files reflect this: `pyproject.toml` deps = mavsdk, pymavlink,
opencv-python-headless, numpy, pydantic, loguru, pyyaml, fastapi, uvicorn only;
dev = pytest, ruff, mypy.

## 5. Contracts (honour these — the flight core is written, do NOT modify it)

> The 2026-07-03 rules change (V1.1) deliberately rewrote the detector
> (`vision/detectors/aruco.py`), the terminal controller (land-ON + id gate),
> and the mission loop (multi-sortie + transit). Those were authorised by the
> locked-decision change above; the contracts below are the current ones.

- **Camera frames:** NADIR = `/tmp/aavc_nadir.jpg` (the single camera; 1280 px
  wide in SITL). The nadir frame is mirrored to `/tmp/aavc_frame.jpg` (the
  dashboard camera endpoint) unless the writer runs `--no-mirror`, which the
  REAL launchers do. **JPEG q95 since 2026-08-21** — the writers pick the codec
  from the path suffix; CM4-measured 48→12 ms to encode and 33→15 to decode,
  which is what pays for decoding every frame instead of every other one.
  The REAL launchers additionally default to `--mjpeg-passthrough` (2026-08-21):
  the camera already emits JPEG, so its bytes are written verbatim — no decode,
  no re-encode, nothing compressed twice — measured **20 Hz for 7% of one CM4
  core** (121 fps available) with 33 KB frames. Decode survival was tested
  across JPEG q95→q60 at 17-42 px markers (the 8-16 m band): all decode, and
  the camera sits at ~q80-85. `CAM_PASSTHROUGH=0` reverts to YUYV + re-encode.
  ⚠ **EXPOSURE IS AUTO, and the forced 2 ms that used to be the default was the
  whole in-flight decode failure** (measured on the aircraft 2026-08-24). It was
  added 2026-08-21 on the theory that `auto_exposure=3` sat at 16.6 ms and
  smeared the frames. Read back OUTDOORS, **auto picks 1 ms — SHORTER than the
  2 ms forced on it**, so auto was never the blur. Forcing 2 ms is what broke
  the decode: same scene, same marker at the KMITL pixel size (29-32 px),
  auto → brightness 124 / sharpness 929 / **DECODES**, forced 2 ms →
  brightness 190 / sharpness 934 / **nothing**. Sharpness was never the problem;
  at ~190 mean the marker's white quiet zone blows out and the black/white edge
  ArUco needs is gone. On auto, a marker carried at RUNNING speed across 8-10 m
  decoded **145/222 then 125/224 frames (65% / 56%)** at 25-35 px — the sweep's
  own pixel size, at twice the sweep's angular rate. `CAM_EXPOSURE` therefore
  defaults to **0 = auto**; a positive value pins a manual exposure in 100 µs
  units. ⚠ `0` now actively RESTORES `auto_exposure=3` + the default gain: it
  used to mean "don't touch", which let a previous forced run leave the camera
  in manual with a stale exposure and nothing said so — that is exactly how the
  decode stayed broken from 2026-08-21 to 2026-08-24 while every launcher
  believed it was on auto. Frame rate is unaffected and was never the issue:
  measured 25.0 fps written, the sensor offering 120.
- **Dashboard DetectedObjectEvent** fields (unchanged set): `t_monotonic, label,
  clothing_color, member_count, pose, confidence, lat, lon,
  is_designated_match`. Values now: `label="aruco pad <id>"|"landing pad"`,
  `pose/clothing_color="unknown"`, `is_designated_match` = decoded id ==
  `state.assigned_marker_id` (`orchestrator/vision_worker.py:_detected_object_events`).
  ⚠ KNOWN GAP (multi-egg, 2026-07-24): `assigned_marker_id` is only
  `flight_ids[0]`, the FIRST id of a multi-egg flight's chunk — so a live
  detection of the flight's 2nd..Nth pad shows as a plain "landing pad", not
  a starred match, on the dashboard. Cosmetic only: the actual id-verified
  LAND gate re-checks each delivery's OWN assigned id per pad
  (`_serve`'s `assigned_marker_id=assigned`, `orchestrator/mission.py`), so
  every delivery still lands on (and only on) its correct pad regardless of
  this display quirk.
- **Drop predictor** (`orchestrator/drop_trajectory.py`): `predict(...) ->
  DropPrediction(points=[TrajectoryPoint(t_s, lat, lon, alt_agl_m)], impact_lat,
  impact_lon, impact_t_s, horizontal_drift_m)` — overlay/audit only (release is
  on the ground, drift ≈ 0).
- **Mission plan** (`mission_brain/live_plan.py`): per-flight + live.
  `render_live_plan(home, spec, *, discovered, profile, transit_route,
  include_search, sortie)` renders TAKEOFF@transit_alt → P1..P3 GOTOs
  (TRANSIT_INGRESS) → SEARCH legs (omitted when every assigned pad is
  registered) → per-`ServedStop` `[GOTO(LOCALIZE), DROP_PAYLOAD(payload_id=
  0..eggs_aboard-1, stop_index=i)]` (one GOTO+DROP pair per delivery,
  `ServedStop.payload_id` — no longer always 0) → P3..P1 (TRANSIT_EGRESS) →
  final **LAND** at L&R (never an RTH command). The mission loop swaps
  `state.plan` at each gate release and each serve. `airframe = HEXACOPTER`
  (via `schemas.active_airframe()`, env `AAVC_AIRFRAME`).
  `pointer_for(..., transit_index=, egress=)` resolves transit points.
- **Dashboard plan feed**: `GET /api/plan` serves the LIVE `state.plan`; the
  broadcaster emits `plan_update` (`{plan, command_pointer}`) on every rebuild.
- **Profile** (`mission_brain/profile.py`): competition,
  `altitude_ceiling_m=20`, `transit_alt_m=20`, `search_floor_m=10`,
  `drop_count_max=1` (defined but currently unread by the mission loop —
  NOT the eggs-per-flight count; do not read it as "one egg aboard" any
  more), `max_sorties=4` (the profile's legacy count field — seeds
  `state.max_deliveries` unless config sets `mission.max_deliveries`; the
  FLIGHT budget `state.max_sorties` is then recomputed as
  `max_deliveries ÷ eggs_aboard`, `orchestrator/main.py`), `eggs_aboard=4`
  (briefing default: all four assigned eggs in one flight; `=1` in the
  `production` profile), `drop_without_confirmation=False`.
- **Per-flight gate**: `POST /api/cmd/preflight/go` body
  `{payload_confirmed, assigned_marker_id (1-6 or null → resolve from the
  queue), force, operator_note}` — writes the resolved SINGLE id to
  `state.assigned_marker_id` BEFORE setting `preflight_resume_event`; 409 when
  neither a manual id nor a queued one exists. The gate loop (`orchestrator/
  main.py`) then derives THIS flight's full chunk of up to `eggs_aboard` ids
  from the queue (`chunk_flights`) into `state.flight_ids` — the manual id
  only becomes the whole flight's assignment at `eggs_aboard=1` or when the
  queue has no chunk for it. `POST /api/cmd/mission_ids` body `{ids (≤4,
  distinct, 1-6; [] clears), operator_note}` sets `state.assigned_id_queue`
  (the ONE queue both dashboard GO and headless `--assigned-ids` consume,
  re-read live each hold; validated against `state.max_deliveries`, not
  `max_sorties` — a full 4-id queue is legal even when `eggs_aboard=4` makes
  `max_sorties==1`). `GET /api/cmd/preflight` +
  `sortie_index, max_sorties (flights), delivery_index, max_deliveries,
  eggs_aboard, sortie_time_ok, assigned_id_queue, queued_id`; TelemetryFrame
  carries `assigned_id_queue` AND `flight_ids`.
- **Audit trail** (`runs/<mission_id>/audit.jsonl`): the grammar broke
  2026-07-24 from `SORTIE n …`/`sortie=` to **FLIGHT/DELIVERY**, in lockstep
  with `tools/verify_flight.py` (keep them in lockstep on any future change).
  1 Hz `TELEM phase=… flight=n lat=… lon=… alt=… armed=… batt=… vbat=…
  mode=…` samples (batt/vbat since 2026-08-20, mode since 2026-08-21 — all
  three OPTIONAL in the verifier so older archives still parse) +
  `TRANSIT_PASS|MISS Pn ingress|egress flight=n d=…m` (once per flight, each
  direction — NOT once per delivery) + `FLIGHT n START eggs=… ids=…
  remaining=…s` / `FLIGHT n END delivered=x/y d_home=…m remaining=…s` +
  `DELIVERY k START pad=… payload=… stop_index=…` / `DELIVERY k RELEASE
  pad=… payload=… lat=… lon=…` / `DELIVERY k END delivered=True|False
  pad=… …` (`k` = `state.delivery_index`, 1-based across the WHOLE mission,
  NOT reset per flight — this is what `orchestrator/tactical_align.py`'s
  `_drop_once` and `orchestrator/mission.py`'s `_serve` key every line by).
  Also: `DELIVERY abort: flight n skipping remaining ids=… …` (the
  per-delivery time/battery gate), `FLIGHT n CONFIG WARN …` (eggs assigned
  exceed the configured `drop_payload_count`), `FLIGHT n ENERGY …mAh`,
  `BATTERY SWAP before flight n …`. `tools/verify_flight.py` parses all of
  these by regex — keep the formats stable; its own docstring is the
  up-to-date transcription of the exact grammar.
- **Dashboard seam** (`orchestrator/main.py` imports it optionally):
  `from dashboard.integration import start_dashboard;
  start_dashboard(state, commander, *, host, port) -> handle`. `handle` has
  `.broadcaster` (with `record_vision(VisionAnalysis)` + `record_drop(DropPrediction)`),
  an optional `.record_drop` attr, and `async def stop(self)`. If anything is
  missing, `main` falls back to headless — keep it import-clean and robust.

## 6. Common Commands

```bash
make install        # .venv + pip install -e .[dev]
make sitl           # PX4 SITL + Gazebo with the IAAI KMITL field
make spawn-targets  # spawn the 6 ArUco pads, ids 1-6 (SEED=n re-rolls ids + positions)
make camera-bridge  # gz camera -> /tmp/aavc_nadir.jpg (+frame mirror; system python3)
make run            # orchestrator (TRUTH=path → truth audit). Headless sorties:
                    #   .venv/bin/python -m orchestrator.main --assigned-ids "3,1,4,6" …
# fly the SITL world on the COMPETITION envelope (transit 20 / ceiling 20 /
# floor 10) instead of the practice profile — this is what makes a SITL run
# represent KMITL. The config's own `mission:` block canNOT do it: those keys
# are DOCUMENTATION; the flown values come from mission_brain/profile.py.
#   AAVC_PROFILE=competition .venv/bin/python -m orchestrator.main \
#       --config sitl/aavc_config.yaml --truth-json /tmp/aavc_targets.json \
#       --no-dashboard --assigned-ids "6,3,5,1"
make web-build      # build the Svelte dashboard
make test           # pytest
make lint           # ruff + mypy
# post-flight drone-response check (operator requirement):
#   .venv/bin/python tools/verify_flight.py runs/<mission_id>/audit.jsonl --truth …
# hand-flown hover decode test — does the camera READ in the air? (run on the
# CM4 beside the grabber; the beacon turns its verdict into one radio line and
# the console spells that out in Thai):
#   .venv/bin/python tools/hover_decode.py            # -> /tmp/aavc_decode.json
#   .venv/bin/python tools/hover_decode.py --no-mavlink   # bench, no vehicle
# is the RC link down, or is PX4 refusing an RC it can hear? (a switch in the
# wrong position clears the RC health bit that every indicator reads):
#   .venv/bin/python tools/rc_check.py --endpoint udpout:127.0.0.1:14550
# read the camera's mount rotation off the airframe (see the tool's docstring
# for the bench procedure; 0.0 in config is an ASSUMPTION until this is run):
#   .venv/bin/python tools/measure_mount_yaw.py --grid /tmp/g.jpg
# regenerate the pad models after a geometry change:
#   .venv/bin/python tools/gen_pads.py
# land-ON precision bench (SITL tuning aid, NOT the scored mission):
#   .venv/bin/python tools/landing_trial.py --pad-index 0 --n 8 [--set PARAM=VAL]
# HITL (real 6X + CM4 + ELRS RC) — full runbook docs/HITL.md, bench sheet HITL_CHECKLIST.md:
#   make hitl-params SERIAL=/dev/ttyACM0   # one-time FC config via nsh (SYS_HITL, airframe, RC)
#   make hitl                              # jMAVSim HIL link (laptop)
#   SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh   # CM4-in-the-loop mission stack
```

## 7. Validation Gates (do not skip)

| Gate | Definition of done |
|---|---|
| G0 import-clean | `make lint` + `make test` pass; no dead imports |
| G1 SITL bare | PX4 + Gazebo load the KMITL field, manual takeoff/land |
| G2 pads spawn | `make spawn-targets` places 6 ArUco pads (ids 1-6, varied yaw — 4 get committee-assigned per team, 2 stay permanent distractors); camera bridge feeds frames |
| G3 detect | `find_landing_pads` decodes a SITL pad id at sweep altitude; projections sane |
| G4 SITL mission | `make run --assigned-ids …`: FLIGHT(s) ⊃ DELIVERIES — transit both ways per flight, land-ON each assigned pad, release after touchdown (`payload_id` 0..eggs_aboard-1 → AUX 4/1/2/3), land+disarm at L&R, window < 20 min; `tools/verify_flight.py` PASSES. All evidence below **predates the 2026-07-24 briefing** and validated the then-current one-egg-per-flight model (`eggs_aboard=1` — behaviourally the SAME loop today, per the regression pin `test_delivery_mission.py::test_eggs_aboard_1_is_one_delivery_per_flight`): **PASSED on the hexacopter 2026-07-22** on the model rebuilt from `Power-System-Guide-1.pdf` (1.000 m wheelbase, 7.17 kg, 18" props, 37.65 N/motor): 4/4 delivered id-correct, release 0.13-0.25 m from truth, transit 8/8 in order, 14.8 min, 19 checks / 0 warnings — `docs/evidence/G4_hexacopter_corrected-model_2026-07-22.txt`. Re-run with the sysid gains: 19/0, releases 0.11-0.27 m, 882 s (`G4_hexacopter_tuned-gains_2026-07-22.txt`). Three runs — guessed geometry, corrected geometry, corrected+tuned — all pass and agree within scatter. ✅ **G4′ CLOSED 2026-07-25**: the briefing default (`eggs_aboard=4` — ONE flight, four deliveries, 6-pad field) **PASSED 14 checks / 0 warnings** on the 2-pack aircraft (AUW 8.22 kg): 4/4 delivered id-correct, releases **0.15–0.21 m** from truth, transit 6/6 in order, max altitude 19.69 m, landed 2.6 m from L&R, **457 s of the 1200 s window** — `docs/evidence/G4prime_multiegg_2packs_2026-07-25.txt`. That run was also the first on the flight clock and the progress-based leg guard (§8). ⚠ It ran at host RTF **0.95** (headless), so it confirms no regression but does NOT re-create the ~0.20 RTF that caused the 2026-07-25 `sweep_leg_timeout_wp0` — that condition is covered by unit tests, not by this flight. ✅ **G4″ 2026-08-24 — the first SITL run on the COMPETITION envelope**, flown to settle `overlap_frac` with numbers rather than argument: `AAVC_PROFILE=competition` (transit 20 m, ceiling 20, search floor 10) with the sweep at **12 m**, so the 400 mm marker is **28.2 px** — the KMITL condition, not the practice field's easier 8 m / 42 px. Same pad seed 1301 both times, only overlap differing: **0.30 → 5 legs, 6/6 pads, 6/6 ids, 4/4 eggs, releases 0.09-0.22 m, transit 6/6, landed 2.0 m from L&R, 536 s**; 0.44 → 6 legs, the same 6/6 · 6/6 · 4/4, releases 0.15-0.20 m, **647 s**. Identical coverage and delivery; 0.30 is 111 s faster here and a computed 158 s at KMITL. `verify_flight` 13 ok / 1 on both (the 1 is a transit segment shorter than the 1 Hz TELEM cadence — its own line reads "100% of samples within the corridor"). Evidence: `docs/evidence/sweep_overlap_2026-08-24.txt` + both audit trails. ⚠ The FIRST attempt of that pair delivered **0 of 4 while decoding all six pads with every id correct** — the gz camera was still at yaw 0 against a config that had just been set to the measured 180, so every SITL fix came out point-mirrored (positions 3.7-20.2 m out). Fixed in `sitl/models/eft_x6100/model.sdf` and now asserted by `test_world_assets.py::test_the_sim_camera_carries_the_measured_airframe_mount`. |
| G5 HW bench | 6X + CM4 bench (props off): ✅ **6-motor map DONE 2026-08-17** (`PWM_MAIN_FUNC1..6` = 101..106 restored + readback, `ACTUATOR_TEST` M1..M6 individually — all six spin, correct order and direction, operator-observed; `SYS_AUTOSTART=6001` / `CA_ROTOR_COUNT=6` untouched). battery calibration: `BAT1_V_DIV` was closed 2026-08-16 on the post-PM03D CONVERTER wiring (multimeter vs GCS; board carries -1 = the 6X default divider) — ✅ **RE-CLOSED ON THE PM02D 2026-08-23**: the module replaced the interim converter on 2026-08-20 and the divider belongs to the module, so it was re-verified the way it was closed the first time — operator's multimeter **24.9 V** at the connector against the FC's own **24.89 V** over MAVSDK, a 0.01 V (0.04%) agreement, board still carrying `BAT1_V_DIV = -1` (the 6X default). The gauge chain was re-checked in the same reading: 4.148 V/cell against `interpolate(3.77, 4.18)` = 92% vs the FC's 93-94%, so `V_EMPTY`/`V_CHARGED` are consistent too. (Motors remain unsensed — this measures avionics voltage only, which is the whole gauge, §2.) ✅ **`BAT1_CAPACITY=-1` + `BAT1_N_CELLS=6` verified on-board 2026-08-17** (USB readback — voltage-only gauge branch active); ✅ **TFmini-S alive 2026-08-17**: `SENS_TFMINI_CFG=103` (TELEM3), `DISTANCE_SENSOR` streamed at the requested rate over USB, steady 0.50 m on the bench, 0.4–12 m limits; ✅ **`BAT1_V_EMPTY`/`V_CHARGED` gauge chain verified 2026-08-17 with the pack on**: FC gauge 42% vs interpolate() 41% at 22.70 V = 3.784 V/cell — consistent with the pack's story (unused since last charge + 5 h of the 0.76 A avionics draw the FC also reported); 3.6/4.05 were the 7500-LiPo era's endpoints (full-charge glance 2026-08-18: 24.84 V = 4.139 V/cell → gauge 100%); the **17000 mAh semi-solid went aboard 2026-08-19 with `BAT1_V_CHARGED=4.18` / `BAT1_V_EMPTY=3.77` on the board** (operator spec 25.1 V full / ~22.6 V empty; BOARD-verified by preflight_params 2026-08-20). ⚠ Measured the same night at KMUTNB: with no current sensing there is no load compensation, so the gauge **sags ~30-35 percentage points under flight load** (28% under thrust at ~65-70% resting SoC — flight 3 was RTH'd by the 30% floor at 68 s on a part-charged pack). The %-floors fire EARLY under load, which is the safe direction; the operational answer is a charged pack, not a lower floor. ⚠ Bench habit: UNPLUG the pack between sessions — 5 h plugged in cost ~3.8 Ah, half of this pack. ⚠ Bench readbacks of `COM_DISARM_LAND` / `RTL_RETURN_ALT` / `EKF2_OF_CTRL` showing non-mission values are NOT regressions — the commander pins those at MISSION START (`orchestrator/main.py` calls `apply_param_overrides`; `connect()` itself writes NO params — believing otherwise is what hides an "applied 0/24" failure); the parked board holds bench state, not flight state. **`tools/preflight_params.py` encodes exactly that split** — run it before every field day (`.venv/bin/python tools/preflight_params.py`, via the router or `--serial /dev/ttyAMA0`): the BOARD block is a STOP if anything is off, the PINNED block is informational. 2026-08-18 on the pack: **BOARD 15/15 correct**, PINNED sitting at defaults (`GF_MAX_VER_DIST` 30, `EKF2_OF_CTRL` 1, `MPC_Z_V_AUTO_DN` 1.5, `COM_DISARM_LAND` 2, `RTL_RETURN_ALT` 6) as expected. ✅ **camera `fov_deg` MEASURED 2026-08-17**: real WSD-9781-v12 lens = **74.2°** (±1°) — 50 mm on-screen marker flat on the floor at a tape-measured 0.495 m, 85.5 px avg over 3 sharp frames → fx 847 px (replaces the 99.7° unmeasured placeholder; config + SITL gz camera + projection default all updated together — sweeps grow ~1.5× more legs since the swath is 0.66× the placeholder's, spacing is fov-derived so no coverage gaps). ⚠ Learned on the way: the parked aircraft's camera sits **3.5 cm** off the ground — ground-level targets are out of focus AND below the TFmini's 0.4 m minimum, so **parked TFmini readings (the steady "0.50 m") are below-min-range garbage** — trust it only in flight. Egg-release servo verified (2026-08-15). **G5 is CLOSED.** ✅ **ESC telemetry: CLOSED NEGATIVE 2026-08-17 — do not re-run this check.** It was run in the required order (motor map restored, all six rotors verified spinning, so an empty reading finally meant something) and came back empty: `DSHOT_TEL_CFG=0`, zero `ESC_STATUS` even after a `SET_MESSAGE_INTERVAL` the FC ACCEPTED. Cause is physical — the ESCs are PWM-only with no telemetry lead — so the one-motor-out layers were deleted rather than left inert (§2). Nothing on this bench can change that; new ESCs can. ✅ **egg-release servos DONE 2026-08-15** (`ACTUATOR_TEST` per pin, all four corners; one latch needs `PWM_AUX_MAX=2100`, `docs/SERVO_AUX_MAPPING.md`). ⚠ **"Preflight fail: Crash dump present on SD" — seen and CLEARED 2026-08-18.** PX4 blocks arming while a hardfault log sits on the card. **Read it before deleting it**: the fault was a Hard Fault (`arm_hardfault.c:173`, `cfsr=0x100` = instruction-bus error escalated by `hfsr=0x40000000`) in the **Idle Task**, 57 min into a boot with no GPS time (i.e. a bench session, never in flight), on **this** firmware (build May 13 2026). One occurrence, kept at `docs/evidence/hardfault_6X_2026-08-18_recovered.log`. Cleared by deleting `/fs/microsd/fault_*.log` over MAVFTP then rebooting; the fence and all 15 BOARD params survived the reboot, re-verified. ⚠ The card also carries `param_import_fail.txt` (kept as evidence): this board has TWICE failed to import `/fs/mtd_params` ("BSON document size doesn't match bytes decoded") and fallen back to `/fs/microsd/parameters_backup.bson`. Both records are against OLDER firmware (1.14.3, 1.15.4) so they read as upgrade artifacts rather than live corruption — but they show **the SD backup is the only net under the motor map**, and the recovery restores ONLY what that file holds. It was checked 2026-08-18 and is CURRENT (129 params: `SYS_AUTOSTART=6001`, `PWM_MAIN_FUNC1..6`, `PWM_AUX_FUNC1..4`, `SENS_TFMINI_CFG`) — the older recovery in the log restored a 54-param set carrying `SYS_AUTOSTART=4001`, the QUAD airframe, which is what "recovered but wrong aircraft" looks like. ⚠ Tooling note: raw pymavlink is unreliable against this board — pymavlink 2.4.49 raises `TypeError` from its instanced-message bookkeeping on PX4 1.17 messages and silently drops whatever arrived with it, which is why param reads and MAVFTP transfers die mid-way. Every helper here guards `recv_match`; `tools/preflight_params.py` sidesteps it entirely by going through MAVSDK. |
| ~~G6 HW tethered~~ | **DROPPED 2026-08-16** (operator): the aircraft has already flown 3-4 real flights, so a tether proves nothing new. ⚠ What was NOT dropped are the two G6 items that are measurements, not ceremony — **camera `fov_deg` calibration** (moved to G5, ground procedure) and the **first land-ON + release over a printed pad** (folded into G7's first flight) |
| G7 HW mission | Full mission flown **at the KMUTNB sky field** — the practice field standing in for KMITL. This IS the "fly the mission for real" step, not a hurdle before it |
| G8 Dress rehearsal | Full mission + live egg deliveries within the 20-min window |

**HITL** (optional, between G4 and G5 — not a locked gate): real 6X (custom
`px4_fmu-v6x_hil` fw, `SYS_HITL=1`) + real CM4 + real ELRS RC vs jMAVSim, **no
motors**. Validates the CM4↔FC link, param push, the full V1.3 sequence, and the
safety-pilot RC/failsafes. Runbook `docs/HITL.md`; bench sheet
`docs/HITL_CHECKLIST.md`. ⚠ the HIL build has NO real actuator output — reflash a
flight fw before G7.

## 8. Working Notes

- Tests import the **real** modules (`mission_brain.live_plan`,
  `vision.projection`, `vision.detectors.aruco`, `orchestrator.mission`) — no
  mocks of the unit under test. Keep `make test` green when touching those.
- `make test` / `make lint` run under `env -u PYTHONPATH` so a sourced ROS env
  cannot leak `launch_testing` / lark plugins into the venv (it will otherwise
  fail at collection). Mirror that if you invoke pytest by hand.
- The orchestrator must stay runnable **headless** (`--no-dashboard
  --assigned-ids …`) and degrade gracefully if the dashboard seam is absent.
- SITL geometry invariant: config `site.center` == config
  `ground_operation.launch_recovery` == world `<spherical_coordinates>` ==
  `launch_sitl.sh` `PX4_HOME_*` == the L&R point. ENU offsets in config
  comments/spawner/world are all relative to it. ⚠ **The first equality is now
  TESTED across every `sitl/*config.yaml`**
  (`test_geometry_invariant.py::test_every_field_config_calls_its_L_and_R_one_thing`)
  because it silently broke: `kmitl_config.yaml` shipped the practice field's
  whole `site:` block — 31.5 km from the competition L&R — under a comment
  asserting the two were equal (found 2026-08-22). The aircraft never noticed
  (home comes from GPS); what breaks is everything that converts a pad lat/lon
  into METRES — `gcs_status._enu`, so the console feed, the `AAVC pads` beacon
  lines, the Svelte dashboard and `spawn_targets.py` — and the console
  re-anchors those metres at the VEHICLE's origin, so every pad marker lands
  31.5 km off the map with every individual number looking sane.
  `orchestrator/main.py::_resolve_site_origin` now takes the origin from
  `launch_recovery` (the point the aircraft uses) and logs an ERROR naming both
  values when they disagree — the other three consumers still read
  `site.center`, which is why the CONFIG has to be right, not just main.py.
  There is no KMITL SITL world: `launch_sitl.sh` spawns on
  `kmutnb_skyfield.sdf` and `tools/gen_geo.py` holds the KMUTNB frame only, so
  `kmitl_config.yaml` is a REAL-FLIGHT config — SITL work belongs in the
  practice repo.
- The rules' no-fly zone + L&R coordinates are APPROXIMATE (figure-only) —
  update `sitl/aavc_config.yaml` after the event briefing.
- **Mission time is FLIGHT time, not host time (2026-07-25):** every mission
  deadline — the 20-minute window, the TimePolicy reserves, every leg and rung
  timeout — is read from `state.now()` (`orchestrator/flight_clock.py`), which
  tracks the aircraft's own clock (`telemetry.vehicle_time_s`, from MAVSDK
  `raw_gps().timestamp_us` = PX4's `hrt_absolute_time()`). On hardware that IS
  wall time; under PX4 lockstep it is SIMULATED time, and a loaded host runs
  the sim slower than the wall — 443 s of sim across 776 s of wall, dipping to
  **0.20x** on one leg (ULog `04_51_49`, 2026-07-25). Consequences before the
  fix: the window was consumed ~1.8x too fast, a "12.9-minute" run was really
  7.4 minutes of flying, and a distance-derived leg timeout abandoned a
  waypoint the aircraft was still closing on at 8 m/s
  (`sweep_leg_timeout_wp0`). Host-side liveness (camera frame age, telemetry
  staleness) deliberately still uses `time.monotonic()` — those ask "is this
  process being fed?", a question about the host. The clock falls back to the
  wall whenever the vehicle stops reporting, so losing the stream degrades to
  the old behaviour rather than freezing every timeout.
  Paired change: leg abandonment is now **progress-based**
  (`mission.py::_ProgressGuard`) — a leg is dropped when it stops CLOSING
  (no 1 m of closure for 25 s), with the old `2 x dist/speed + 20 s` kept only
  as an 8x hard ceiling. The real bird hits the same failure without any clock
  trickery: a headwind or a re-planned longer leg stretches wall time past 2x
  nominal while the aircraft is flying perfectly well.
  ⚠ RTF is much better headless: the failing run had the dashboard up (0.57
  average), the passing one did not (0.95).
- **The mission clock ran at 1/20 speed on the real aircraft (2026-08-26,
  same flight, CM4 audit).** `t=` advanced 0.3 → 0.6 s across ~50 s of flying
  and `t=1.0s` held for 13-20 wall seconds in preflight, while the 08-18 bench
  session (no GPS fix → no vehicle time → wall fallback) ticked 1:1. Cause in
  `orchestrator/flight_clock.py`: `state.now()` feeds the clock on EVERY read
  (~10 Hz), the hardware `raw_gps` timestamp changes only every so often, and a
  re-feed of the SAME value reset the wall anchor — so when the value finally
  moved, `min(veh_step, wall_step)` saw ~0.1 s of wall and credited that.
  Reproduced exactly: a timestamp changing every 1 / 5 / 20 s read at 10 Hz
  credited 9.9 / 1.9 / 0.4 s per 100 s. Every deadline — the 20-min window,
  `can_start_delivery`, the 25 s progress guard, touchdown and rung timeouts —
  stretched 20× on the real bird and none of them could have fired at KMITL.
  Fixed: a re-feed moves nothing; within an EPOCH flight time is
  `base + min(cum_vehicle, cum_wall)` (cumulative, so arrival jitter cancels
  instead of being lost per step); a reboot / backwards stamp / forward jump
  > `JUMP_S` / stale gap starts a new epoch, netting off what the stale
  fallback already credited so nothing is counted twice. Tests: the three
  2026-08-26 cases at the end of `tests/test_flight_clock.py`. ⚠ Still open:
  WHY the hardware timestamp only changes every ~13-20 s (MAVSDK `raw_gps`
  over TELEM2 at the router's rate?) — the clock now tolerates it, but a
  vehicle-time stream that sparse is worth a look on the bench.
- **`relative_alt_m` is `alt_m − home LATCHED AT ARMING`, never PX4's own
  `relative_alt` (2026-08-26, ULog `2026-08-26/08_21_26.ulg`).** PX4 1.17
  (upstream `6604c52c98`, #25003, `HomePosition::update`) rewrites `home.alt`
  for 120 s after takeoff whenever baro and the integrated GPS vertical
  velocity agree with each other but disagree with GPS altitude by > 1 m. That
  assumes a GPS-referenced EKF; ours is baro-referenced (`EKF2_HGT_REF=0`), so
  the EKF's altitude does not follow the GPS drift and the shift moves
  `gpos.alt − home.alt` — MAVSDK's `relative_altitude_m` — while the aircraft
  stays put. On 2026-08-26 the aircraft held **8.5 m by lidar, EKF and
  setpoint** while `relative_alt` walked 8.5 → 9.6 → **11.7 m**; the ceiling
  watchdog (`safety.py`, correct on its input) RTH'd a practice-envelope sweep
  at t=55 s. **Every real flight since 08-21 carried it**: +3.29 / −4.65 /
  +0.91 / +1.17 / −3.18 m, both signs — a negative shift also makes the
  touchdown fallback read "airborne" on the pad (egg kept), a positive one
  lets `near_ground` pass 3 m up. The only PX4 switch is `COM_HOME_EN=0`
  (no home at all) — not an option. Fix: `TelemetrySubscriber._sub_home`
  latches `home_alt_msl` while disarmed / still on the ground after arming and
  **refuses** home updates once airborne (landed_state leaves ON_GROUND *or*
  PX4's relative alt passes 1 m, so a dead landed_state stream still freezes);
  a pad landing mid-flight (armed, ON_GROUND) stays frozen. `px4_relative_alt_m`
  keeps the raw value and a WARN logs the first > 1 m gap. `goto()` prefers the
  same latch (`commander.home_alt_source`, wired in `main.py`) — its
  connect-time cache was only refreshed when the COMMANDER armed, and the RC-GO
  conops arms from the transmitter. ⚠ PX4's OWN home-relative nets
  (`RTL_RETURN_ALT`, `GF_MAX_VER_DIST`) still ride the rewritten home: a +4.65 m
  shift at KMITL puts a failsafe RTL at ~24.6 m real. Tests:
  `tests/test_home_alt_latch.py`.
- **The `mission:` block in a config does NOT set the envelope (re-learnt
  2026-08-24).** `altitude_ceiling_m`, `transit_alt_m` and `search_floor_m`
  there are documentation; the flown values come from
  `mission_brain/profile.py` via `AAVC_PROFILE`, and the sweep altitude is
  clamped to the PROFILE's ceiling with `min()`. A SITL run started without
  `AAVC_PROFILE=competition` silently flies the practice envelope (transit
  9 m, ceiling 10) whatever the config says — a test of KMITL behaviour then
  measures the wrong aircraft, and the run looks completely normal while doing
  it. Check the startup line: it prints `transit N pts @ X m, sweep N legs @ Y m`.
- **Killing an orchestrator mid-flight strands the SITL aircraft in the air
  (2026-08-24).** The next launch dies at `unmet critical checks: on_ground`
  and the truth audit reads 0/6 with `frames=0` — which reads exactly like a
  dead camera and is not. Restart the stack; do not retry the run.
- **Restart SITL before a scored run if anything else flew first (2026-07-22):**
  a sys-ID sweep lands wherever the chirps drifted it, and the next mission then
  takes off from THERE. PX4 re-captures home at that arm, so the mission's own
  `d_home` reads a healthy 2.9 m while the aircraft is actually 112 m from the
  configured L&R — `tools/verify_flight.py` caught it precisely because it
  cross-checks the final fix against config rather than trusting `d_home`. Do
  not "fix" that check; restart SITL so the vehicle respawns at the L&R origin.
- **Wiping SITL `parameters.bson` costs you the next flight (2026-07-22):**
  clearing `build/px4_sitl_default/rootfs/parameters*.bson` is sometimes needed to
  make new `param set-default` values in an airframe actually apply — PX4 will not
  override a value it has already saved. But the first flight afterwards runs on
  un-converged estimators (hover thrust via `MPC_USE_HTE`, EKF biases): a mission
  launched straight after a wipe climbed steadily through the ceiling and the
  watchdog RTH'd it, while the identical mission on the next boot passed 19/0.
  Fly one throwaway flight, or restart SITL once, before trusting a run.
- **Altitude-frame gotcha (G4 2026-07-04):** PX4 re-captures HOME at every
  arming, so the goto AGL→MSL conversion re-caches the home MSL per arm
  (`DroneCommander._refresh_home_alt`); the frame still wanders ±~0.7 m, so
  transit is commanded 0.5 m under the strict altitude and the touchdown
  threshold is 1.5 m. Don't "fix" these back to exact values. A lone 1 Hz
  sample can still poke ~20.5 m during egress — `tools/verify_flight.py` treats
  a TRANSIENT ceiling excursion as a WARN (only a sustained hold or >ceiling+2
  fails), matching the in-flight watchdog (warn >20.5, RTH >22).
- **AUTO reads DIFFERENT params than manual (2026-07-20, bit us twice):** PX4
  splits several limits by mode family, and this mission flies AUTO end to end —
  so the AUTO twin is the one that moves the aircraft:
  `MPC_Z_V_AUTO_DN` (autonomous descent) vs `MPC_Z_VEL_MAX_DN` (manual/offboard,
  `FlightTaskManualAccelerationSlow`); `MPC_JERK_AUTO` (AUTO) vs `MPC_JERK_MAX`
  (manual). A staged L&R descent commanding `MPC_Z_VEL_MAX_DN=3.0` still sank at
  0.39 m/s until it was pointed at `MPC_Z_V_AUTO_DN`. **If a PX4 knob "has no
  effect", check the AUTO twin first.**
  ⚠ **CORRECTION (2026-07-22, code review vs the v1.17 source):** this note used
  to name `MPC_ACC_HOR_MAX` as the AUTO horizontal-accel knob and `MPC_ACC_HOR`
  as its manual twin. That pair is the other way round —
  `FlightTaskAuto.hpp` declares `MPC_ACC_HOR` (defined in
  `multicopter_autonomous_params.c`), while `MPC_ACC_HOR_MAX` is
  `FlightTaskManualPosition`'s. The mission's live accel cap is therefore the
  `MPC_ACC_HOR=3.0` that was labelled "manual", and the 5.0 pinned into
  `MPC_ACC_HOR_MAX` shapes only the safety pilot's manual mode. Values were left
  alone (3.0 is what every validated run flew); only the comments were fixed. The
  2026-07-20 speed-up therefore came from `MPC_JERK_AUTO` + `MPC_TKO_SPEED` + the
  staged descent, not from the accel cap. ⚠ `tactical_align`'s per-rung descent ladder still steps
  `MPC_Z_VEL_MAX_DN` — i.e. it does NOT shape the AUTO descent it was written to
  shape (OPEN; the effective pad-approach descent is the pinned
  `MPC_Z_V_AUTO_DN=0.4`, which is what every validated landing actually flew —
  do not "unpin" it: PX4's default is 1.5, ~4× faster onto the pad than anything
  tested, and SITL cannot catch that because SITL had 0.4 persisted in
  `parameters.bson`).
- **OPEN RISK — per-arm altitude drift (2026-07-20, much smaller on the hexa):**
  on the QUAD the held transit altitude wandered **±0.9 m between sorties**
  (constant within a sortie, random sign across them) against a 19.5 m command —
  three 4-sortie runs including an unmodified-code baseline — which is most of
  the 20 m ceiling margin and produced `verify_flight` ceiling FAILs at 20.69 m
  and 20.87 m. Lowering the command is NOT the fix; the corridor is checked from
  both sides. See `docs/superpowers/specs/2026-07-20-altitude-frame-drift.md`.
  **The hexacopter's first G4 run (2026-07-22) held 19.31 / 19.41 / 19.29 /
  19.23 m across its four sorties — a 0.18 m spread, max 19.75 m, zero ceiling
  warnings** (`docs/evidence/G4prime_hexacopter_2026-07-22.txt`). Encouraging,
  but that is ONE run: re-measure across several before treating it as closed.
- **Truth-coordinate fix (2026-07-04):** `spawn_targets` had used the WGS84
  EQUATORIAL radius for the north axis, writing truth ~0.5 m south of where gz
  renders each pad — this inflated every measured touchdown-vs-truth distance.
  Fixed to the ellipsoidal meridional/prime-vertical scale; the "0.44-0.53 m
  scatter" that drove the earlier tuning was mostly this bias.
- **OPEN G6 item (nearly closed in SITL):** with the truth fix, the tuned
  4-sortie mission releases at **0.09-0.30 m** and the id-truth audit reads
  0.10-0.14 m — align locks 0.14-0.19 m, and PX4 LAND adds little.
  `landing_accuracy_threshold_m` is now **0.5**. A final-descent visual
  re-centre (optical flow on the real bird) is the remaining item for the
  real no-RTK hardware, not SITL.
- **`verify_flight.py` fails CLOSED (review 2026-07-04):** it drops pre-GO TELEM
  (window-clock reset), warns on NaN samples, FAILs a release with no truth, and
  cross-checks the L&R fix vs config — don't "simplify" these back into the
  silent-skip forms. `spawn_targets` parses the gz Boolean reply + retries (the
  CLI exits 0 on a service timeout) and writes truth atomically 0600; HITL frames
  are 0600 too. **`make hitl-params` is untested on hardware** (nsh SERIAL_CONTROL
  shell) — `--dry-run` prints the manual fallback; the **CRSF driver is NOT in the
  stock fmu-v6x build** (add `CONFIG_DRIVERS_RC_CRSF_RC=y`).

## 9. Reference Papers (ScienceDirect) — map to modules

Curated literature for command / control / monitor / plan, mapped to the module each
informs. Links with `/abs/` = abstract only (full text behind paywall — use the KMUTNB
library / EZproxy login); `/pii/` (✅ OA) = open access, full text free. These are
**design references**, not dependencies — no code is pulled from them; the flight core
stays classical + deterministic (§2). Use them to justify/refine an approach, not to
re-architect a locked decision.

**Applied in the 2026-06-11 blind-search build** (these moved from "design reference"
to implemented): **A** — attitude-composed projection + median-filtered, tilt-gated
centring (`vision/projection.py`, `orchestrator/tactical_align.py`); **B** — size-prior
gate + k-frame temporal-consistency confirmation (`orchestrator/target_tracker.py`,
`tactical_align.py`); **C** — multi-frame median geolocation fusion (`target_tracker.py`,
`tactical_align.py`); **E2** — boustrophedon coverage sweep
(`mission_brain/search_pattern.py`); **E1** — time-window reserve policy
(`orchestrator/time_policy.py`). **F** (LSTM anomaly) and **G** (NN / ballistic
wind-drift drop) stay reference-only — deliberately NOT applied (land-ON-and-release
needs no in-flight release model; ML stays out of the flight core, §2).

### A. Vision servo landing → `orchestrator/tactical_align.py`, `orchestrator/vision_worker.py`

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Autonomous ship deck landing of a quadrotor UAV using feed-forward IBVS · *Aerospace Sci. Tech.* | [/abs/…S1270963822005430](https://www.sciencedirect.com/science/article/abs/pii/S1270963822005430) | IBVS centring-error → metric correction; closest analogue to the descend-and-align rung loop that now ends ON the pad (our target is static, so drop the feed-forward velocity term) |
| Autonomous landing of low-cost quadrotor on a moving platform · *Robotics & Auton. Syst.* | [/abs/…S0921889019300508](https://www.sciencedirect.com/science/article/abs/pii/S0921889019300508) | Low-cost / coarse-sensor landing pipeline — matches the no-RTK, CM4 budget |
| Adaptive visual servo, finite-time tracking, virtual-reticle algorithm · *Robotics & Auton. Syst.* | [/abs/…S092188902100049X](https://www.sciencedirect.com/science/article/abs/pii/S092188902100049X) | Reticle/centring formulation for the per-rung re-centre |
| Robust visual servoing for quadrotors landing on a moving target · *J. Franklin Inst.* | [/abs/…S0016003221000223](https://www.sciencedirect.com/science/article/abs/pii/S0016003221000223) | Robustness of the servo loop to detection noise |
| Quadrotor going through a window and landing: IBVS · *Control Eng. Practice* | [/abs/…S0967066121001040](https://www.sciencedirect.com/science/article/abs/pii/S0967066121001040) | IBVS that ends in a precision landing — terminal-phase reference; the V1.3 land-ON-a-1-m-pad is exactly this precision class |

### B. Landing-pad detection (ArUco + white-pad cue) → `vision/detectors/aruco.py`

(Target changed 2026-07-03 to the official ArUco pad; the detector moved from
colour+shape gates to ArUco decode + a white-square/dark-centre cue. The
colour+shape references still inform the cue's joint test; fiducial decode is
standard OpenCV.)

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Efficient & accurate circular object detection in color images · *Comput. & Electr. Eng.* | [/abs/…S0045790614001244](https://www.sciencedirect.com/science/article/abs/pii/S0045790614001244) | Colour + shape jointly — analogue to the white-pad mask → squareness/dark-centre cue |
| A sparse structure for fast circle detection · *Pattern Recognition* | [/abs/…S0031320319303255](https://www.sciencedirect.com/science/article/abs/pii/S0031320319303255) | Real-time CPU contour/shape detection — fits the few-ms-on-CM4 budget |
| Curvature-aided Hough transform for circle detection · *Expert Syst. Appl.* | [/abs/…S0957417415008210](https://www.sciencedirect.com/science/article/abs/pii/S0957417415008210) | The pad's ⌀750 ring is a potential secondary cue if the field print differs from Fig. 6 |

### C. Pixel → (lat, lon) projection & monocular geolocation → `vision/projection.py`

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Joint localization and target tracking with a monocular camera · *Robotics & Auton. Syst.* | [/abs/…S0921889015001268](https://www.sciencedirect.com/science/article/abs/pii/S0921889015001268) | Ground-target localisation from one camera — the ray→ground-intersection model |
| Image-based UAV position & velocity estimation using a monocular camera · *Control Eng. Practice* | [/abs/…S0967066123000291](https://www.sciencedirect.com/science/article/abs/pii/S0967066123000291) | Monocular state estimate — sanity-check for the pixel→ground projection |
| A review of UAV autonomous navigation in GPS-denied environments · *Robotics & Auton. Syst.* | [✅ OA /pii/…S0921889023001720](https://www.sciencedirect.com/science/article/pii/S0921889023001720) | Survey backing the "cameras own the final metre, not GPS" no-RTK rationale |

### D. Control + offline System-ID / Autotune → `mavlink_adapter/` (the `tuning/` module itself was removed 2026-08-15/17 — these stay as design references for the PX4-autotune gains we bake in)

| Paper · Journal | Link | How it applies here |
|---|---|---|
| System identification and H∞-based control of quadrotor attitude · *Mech. Syst. Signal Process.* | [/abs/…S0888327019305795](https://www.sciencedirect.com/science/article/abs/pii/S0888327019305795) | Frequency-response sys-ID of the attitude loop — direct backing for `tuning/sysid.py` (numpy FRF) |
| A two-step method for system identification of a low-cost quadrotor · *Aerospace Sci. Tech.* | [/abs/…S1270963819309368](https://www.sciencedirect.com/science/article/abs/pii/S1270963819309368) | Plant-model identification on cheap hardware → `tuning/plant.py` |
| PID controller auto-tuning (step response + damping-optimum) · *ISA Transactions* | [/abs/…S0019057813001419](https://www.sciencedirect.com/science/article/abs/pii/S0019057813001419) | Model-based gain design → `tuning/synthesis.py` / `engine.py` |
| Autonomous quadrotor: navigation & guidance systems · *Eng. Appl. of AI* | [/abs/…S0957415817301757](https://www.sciencedirect.com/science/article/abs/pii/S0957415817301757) | End-to-end GNC architecture — orchestrator + mavlink_adapter framing |

### E. Mission planning (sortie scheduling, time budget) → `orchestrator/mission.py`, `time_policy.py`

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Online stochastic UAV mission planning with time windows & time-sensitive targets · *Eur. J. Oper. Res.* | [/abs/…S0377221714002288](https://www.sciencedirect.com/science/article/abs/pii/S0377221714002288) | Time-window planning — backs the 20-min window reserve policy + per-sortie gate |
| Dynamic coverage path planning for UAV formations in multi-region tasks · *Aerospace Sci. Tech.* | [/abs/…S1270963825007540](https://www.sciencedirect.com/science/article/abs/pii/S1270963825007540) | Coverage of the search polygon + revisit ordering of undecoded candidates |

### F. Safety / health monitoring → `orchestrator/safety.py`, `audit.py`

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Detecting structural anomalies of quadcopter UAVs via LSTM autoencoder · *Pervasive & Mob. Comput.* | [/abs/…S1574119222001493](https://www.sciencedirect.com/science/article/abs/pii/S1574119222001493) | Health-monitoring **concept only** — ML stays out of the flight core (§2); informs what signals the watchdog could trend |

> Most failsafe/fault-tolerance literature is on IEEE/MDPI, not ScienceDirect — extend
> the search there if §F needs more depth.

### G. Payload drop / ballistic release → `orchestrator/drop_trajectory.py`

| Paper · Journal | Link | How it applies here |
|---|---|---|
| Approach methods for autonomous precision aerial drop from a small UAV · *IFAC-PapersOnLine* | [✅ OA /pii/…S2405896317310030](https://www.sciencedirect.com/science/article/pii/S2405896317310030) | Release-point computation — audit/overlay model only (V1.3 releases ON the ground) |
| NN-based prediction of precise airdrop trajectory planning · *Aerospace Sci. Tech.* | [/abs/…S1270963821008129](https://www.sciencedirect.com/science/article/abs/pii/S1270963821008129) | Wind-drift + impact-point modelling for `predict(...)` horizontal-drift output |

---

*Lightweight rebuild of `../aavc-2026`. Update on locked-decision changes, contract
changes, or gate transitions.*
