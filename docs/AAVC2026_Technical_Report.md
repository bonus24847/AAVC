# AAVC 2026 — Technical Design & Analysis Report

**Autonomous PX4 Quadcopter for Precision Fragile-Cargo Delivery**
Consortium of Aerospace Engineering (CASE) · Autonomous Aerial Vehicle Challenge 2026
Rules & Regulations **V1.3** (July 2026) · IAAI, KMITL Ladkrabang

> Team: **AeroOptix** · Institution: KMUTNB, Faculty of Engineering · Advisor: _[advisor]_
> Document version: 1.1 · Prepared for the 20 July 2026 technical-document deadline.
> _This report is structured to the rules Appendix A (Executive Summary 10 pts · Team
> Organization 10 pts · Concept Development 30 pts · Engineering Design & Analysis 40 pts)
> and cross-references the Appendix B DCOS airworthiness criteria (§7.3)._

---

## Executive Summary (10 pts)

We field a single **autonomous PX4 quadcopter** that delivers one fragile raw-egg cargo
per sortie onto the landing pad matching a committee-assigned ArUco marker, at the IAAI
KMITL field, and repeats this for up to four pads inside a 20-minute operation window —
the AAVC 2026 (V1.3) mission.

The design philosophy is **fast-but-safe determinism**: a classical computer-vision
pipeline (no machine learning in the flight loop, no network — a network use is an
automatic disqualification) drives a Pixhawk 6X flight controller from a Raspberry Pi
CM4 companion. The aircraft flies the mandatory transit corridor P1→P2→P3 at a strict
20 m, sweeps the search area only when the assigned pad's location is still unknown,
descends over the correct pad in verified altitude rungs, **lands ON the pad, and
releases the egg only after touchdown is confirmed** — the egg is never released
airborne, because the scoring pays for an intact egg placed on the pad and a broken egg
scores nothing.

Cargo location is unknown at take-off: the committee only assigns a marker id per
sortie. The system therefore performs **blind visual search with a cross-sortie pad
registry** — every pad seen during the first sweep is decoded and remembered, so later
sorties fly direct to their pad with no sweep, conserving the window.

Key validated results (SITL, full 4-sortie mission on the KMITL field geometry, flown
in the final flight configuration — the single OV9281 nadir camera and the System-ID
gain set):

| Metric | Result (fresh G4 confirmation run, tuned gains, SEED 7) |
|---|---|
| Eggs delivered on the correct assigned pad | **4 / 4** — verifier-confirmed id, released 0.10–0.30 m from the true pad centre, all landed before release |
| Transit coordinates scored (both directions) | **24 / 24** (0 misses) |
| Return-to-L&R landings | 4 / 4, within 2.6–2.7 m of the L&R point |
| Operation window used (4 sorties) | **1001 s ≈ 16.7 min** (< 1200 s) |
| Altitude discipline | max 20.33 m — under the 20.5 m warn line, no ceiling flag |
| Verifier verdict | **PASS — 19 checks ok, 0 warnings** (`tools/verify_flight.py`, fail-closed) |

Prior committed validation: the same layout flown pre-tuning released 0.05–0.26 m; a
dedicated sortie on grayscale frames proved the monochrome pipeline end-to-end
(0.14 m); the V1.3 confirmation run (SEED 42) delivered 4/4 at 0.10–0.32 m with
1,174 JPEG frames recorded; and five earlier seeded layouts each delivered 4/4 with
verifier PASS 0-warn — see § Appendix B.

The software has been hardened against the operational failure modes that matter for a
real drone over a real field: an emergency motor-cut ("kill") path from the ground
station, an in-flight camera-freeze guard that prevents landing on a stale image, a
centred-final-rung LAND gate that refuses an off-centre landing (climb, defer, and
re-approach — never spend the egg on a bad landing), flight-controller-level RC-loss
and battery failsafes pinned and read back, and a persistent imagery record satisfying
the rules' "record **and** transmit" imaging requirement. All 239 automated tests
pass; the flight core is deterministic and offline.

---

## 1. Team Organization (10 pts)

> _[Fill with the organization chart + the roles/functions table + member photos, per
> Appendix A. Suggested structure below — replace with the actual roster.]_

| Role | Member | Function |
|---|---|---|
| Team Leader (contact) | _[name]_ | Overall coordination; single point of contact with AAVC staff |
| Faculty Advisor | _[name]_ | Airworthiness sign-off, safety oversight |
| Airframe / Structures | _[name]_ | Frame, landing gear, payload bay design & build |
| Propulsion / Power | _[name]_ | Motor/ESC/battery selection, thrust & endurance analysis |
| Avionics / Flight Control | _[name]_ | Pixhawk 6X integration, PX4 tuning, failsafes |
| Software / Autonomy | _[name]_ | Mission orchestrator, vision, safety watchdog, GCS |
| Safety Pilot | _[name]_ | RC oversight, manual-takeover, kill authority |
| C2 / GCS Operators (≤4) | _[names]_ | Per-sortie GO, monitoring, resupply coordination |

Crew inside the GCS operating area complies with the rules limit: 1 safety pilot, ≤4
command-and-control operators, ≤3 technical-support personnel. All flight-line crew wear
PPE (protective glasses, reflective vests) per DCOS.

---

## 2. Concept Development (30 pts)

### 2.1 Capability parameters derived from the operational challenge

The operating environment fixes the design envelope. From the rules (V1.3) and the IAAI
KMITL field geometry we derive the required vehicle capability:

| Environment / rule driver | Value | Derived vehicle requirement |
|---|---|---|
| Controlled airspace (geofence) | ~296 × 167 m | Endurance + speed to cover it inside the window |
| Search area | ~210–227 × 57–74 m | Coverage sweep of a ~1.3 ha polygon |
| Transit altitude (mandatory) | **strictly 20 m** | Precise altitude hold at 20 m AGL |
| Search band | 10–20 m AGL | Marker-decodable imagery from 10–20 m |
| Delivery descent | below 10 m only over the pad | Precision terminal descent + land-ON |
| Landing pad | 1 m white square, ⌀750 mm ring, **400 mm ArUco** (DICT_4X4_50, ids 1–6) | Decode a 400 mm marker from the search band |
| Operation window | 20 min, ≤4 pads, per-minute overtime penalty | ≤ ~5 min per full delivery sortie |
| Cargo | one no.0 raw egg in a heart box (~16 × 7 × 18 cm, 300-gsm) | Gentle payload bay + release; ~[egg+cradle mass] |
| MTOW limit | ≤ 25 kg combined | Comfortable margin (single small quad) |
| No RTK GPS | coarse (~0.5–1.5 m) | Vision, not GPS, owns the final metre |

**Decode geometry drives the camera and sweep altitude.** A 400 mm ArUco marker
subtends ≈ 18 px at a 12 m sweep altitude on a 1280-px nadir sensor — enough for a
reliable decode with an ROI up-scale, but only just; this sets the single nadir
camera at **1280 px wide** (Meige OV9281, 1280 × 720 mono global shutter on a
gimbal-stabilized mount — the swath math uses width only) and the sweep altitude
at ~12 m rather than the 20 m ceiling.

**Window budget drives endurance and sweep speed.** Four full sorties inside 20 minutes
means ≈ 5 minutes each (transit both ways + sweep + serve + land + resupply). The
cross-sortie pad registry (see §2.3) removes the sweep from sorties 2–4, so the window
is comfortably met (≈ 17 min measured in SITL).

### 2.2 Selected delivery system — configuration and architecture

**Vehicle configuration: single multirotor (quad-X).** The rules require VTOL because
of the confined urban field; a quad-X is the simplest airframe that (a) hovers precisely
over a 1 m pad, (b) descends vertically for a land-ON, and (c) needs no transition logic.
A single vehicle (rather than a cooperative search+deliver pair) keeps the system
mass, cost, and failure surface small while still meeting the window through the
registry optimisation.

**System architecture (air + ground):**

```
AIR SEGMENT                                    GROUND SEGMENT (GCS)
┌──────────────────────────────┐              ┌───────────────────────────┐
│ Pixhawk 6X (PX4)  ── flight   │  telemetry   │ Primary terminal (laptop) │
│   control, failsafes, geofence│◄────────────►│  + Svelte dashboard       │
│      ▲  MAVLink (UART)        │  ≥500 m LOS  │  (live map, cameras,       │
│      │                        │              │   per-sortie GO, KILL)     │
│ Raspberry Pi CM4 (companion)  │              │ Safety-pilot RC (ELRS)     │
│   mission orchestrator,       │  imagery     │ Comm tower + backup power  │
│   vision, safety watchdog     │─────────────►│ ≥1 external display        │
│   ▲ OV9281 nadir 1280×720     │              └───────────────────────────┘
│     (mono GS, gimbal servo)   │
│   optical flow + rangefinder  │   ← position lock for the no-RTK final metre
│   egg-release servo (AUX9)    │
└──────────────────────────────┘
```

The flight-management stack is a **non-proprietary, open-source architecture** (PX4 on
Pixhawk 6X) integrated by the team — compliant with the rules prohibition on complete
proprietary flight packages. Higher-level autonomy (search, identification,
localisation, land-and-release) runs on the CM4 as deterministic classical code.

**Rationale for the key decisions:**
- *Classical CV, not ML, in the flight loop* — the target is a fiducial (ArUco); a
  standard OpenCV decode is deterministic, CPU-cheap on the CM4, and auditable. No
  torch/YOLO/VLM, no cloud, no network (network = DQ).
- *Land-ON + touchdown-gated release* — the scoring rewards "landed on the pad **before**
  releasing" and an intact egg; releasing airborne risks a broken egg for zero payload
  score, so release is hard-gated behind a confirmed touchdown.
- *Cross-sortie registry* — the committee assigns pads one at a time; remembering every
  pad seen in the first sweep converts sorties 2–4 into direct flights, buying window
  margin.
- *Vision owns the final metre* — with no RTK the GPS wanders ~0.5–1.5 m, larger than the
  1 m pad; the nadir camera + optical-flow/rangefinder close the terminal descent.

### 2.3 Concept of operations (mission profile + tactics)

Each sortie executes the rules Fig. 4 profile as a deterministic state machine
(`orchestrator/mission.py`):

```
PREFLIGHT (per-sortie GO: operator enters the assigned id + confirms egg)
  → TAKEOFF (arm, climb to 20 m at the L&R site)
  → TRANSIT_INGRESS  P1 → P2 → P3  @ 20 m   (scored per coordinate)
  → SEARCH   (only if the assigned pad is NOT yet in the registry:
              boustrophedon sweep of the search polygon @ ~12 m,
              decoding EVERY pad seen into the cross-sortie registry)
  → LOCALIZE (descend over the assigned pad in verified altitude rungs;
              wrong-id pads never steer the descent)
  → LAND     (land ON the pad; final rung tolerance 0.2 m on a 1 m pad)
  → DROP     (release the egg AFTER touchdown is confirmed; skip if airborne)
  → TRANSIT_EGRESS  P3 → P2 → P1  @ 20 m    (scored per coordinate)
  → LAND at L&R  → DISARM (resupply crew approaches) → next sortie
```

**Search-and-identify tactic.** A sortie whose assigned id is not yet registered flies a
**boustrophedon (lawn-mower) sweep** of the search-area polygon, decoding every pad it
sees into a marker-id-keyed registry with a k-vote confirmation (to reject a single
mis-decode). The sweep runs to completion (early-stop only once all pads are confirmed),
so subsequent sorties fly **direct** to their registered pad with no sweep. An
undecoded white-pad candidate is revisited at the 10 m floor to read its id; a pad
decoded but not yet vote-confirmed gets a cheap vote-top-up visit rather than a full
re-sweep.

**Delivery tactic (land-ON, touchdown-gated).** Over the assigned pad the aircraft
descends through altitude rungs (12 → 8 → 5 → 3 → 2 → 1.5 m), re-centring on the decoded
marker at each rung with a tightening tolerance (down to **0.2 m** before the final
LAND — a third of the 1 m pad). The terminal controller refuses to land unless the
**assigned id was actually decoded** during the approach (the id-verified LAND gate);
a wrong-id pad or an undecoded blob can steer the descent but never commits the egg.
The final rung must also actually **lock**: if the re-centre never converges inside
the 0.2 m tolerance (e.g. a biased fix streak at low altitude), the vehicle refuses to
land, climbs back and defers, and the mission re-approaches once if the window allows.
After PX4 confirms touchdown, a 2 s settle precedes the servo release — gentle for the
egg. If telemetry ever reads airborne at the release point, the release is **skipped and
audited**.

**Repeat-delivery tactic.** Between sorties the aircraft lands at the L&R site and
disarms so the resupply crew can approach safely; a new egg (and battery if needed) is
loaded, the operator enters the next assigned id, and the next sortie launches. The
20-minute window clock starts at the first GO; the per-sortie gate refuses a new launch
that cannot finish inside the window unless the operator explicitly accepts the overtime
penalty.

---

## 3. Engineering Design & Analysis (40 pts)

### 3.1 Airframe structure and integrity

- **Frame:** quad-X on a **Holybro X500 V2** 500 mm airframe (deep-drawn carbon arms +
  polymer hubs, ~610 g including tall landing gear, dual battery rails, and the RPi
  platform board). The tall gear gives the egg cradle ground clearance for the land-ON
  touchdown and keeps the downward flow/rangefinder sensors clear of the terrain.
- **Rotor clearance:** adjacent motors are 354 mm apart, giving **100 mm of prop-tip
  clearance** for the 254 mm (10″) props — no rotor overlap, adequate margin against
  arm flex (DCOS clearance-between-moving-parts).
- **Payload bay:** sized to accept the rules-V1.3 organiser cargo box (heart,
  ~16 × 7 × 18 cm, 300-gsm art card, with a handle loop) — not merely a bare egg — with
  the release servo clearing the box on opening (a G5 bench fit-check item).
- **Structural load cases:** flight manoeuvre load (envelope-limited to 35° tilt /
  3 m/s² horizontal accel), the land-ON touchdown impact (absorbed by the tall gear + a
  compliant cradle at `MPC_LAND_SPEED` = 0.3 m/s — the 2.6 kg vehicle carries more
  landing energy than the earlier 2.0 kg concept, so the gear/cradle compliance is sized
  accordingly), ground handling, and transport.
- **Power-path sizing:** 10 AWG silicone main leads + XT90 (90 A continuous, 150 A+
  burst) for the ~16 A hover / ~110 A worst-case peak; 16 AWG motor phase wires (~40 A
  each); a Hall-effect power module rated ≥150 A. Connectors and wiring are secured and
  strain-relieved; critical components are protected from dust/rain (DCOS).
- **Open items (honest status):** the **centre-of-gravity placement** and a **quantified
  structural load-factor / margin analysis** are not yet finalised — both are flagged as
  open in the sizing docs and are G5-bench items (CG trim + a static load check). The
  real airframe will also be re-tuned by bench System-ID (`tuning/`) because its higher
  inertia and lower natural frequency differ from the SITL model; the full identify →
  synthesise → apply → re-validate loop has already been exercised end-to-end in SITL
  (a clean 3-axis identification produced the current gain set, confirmed by the
  Appendix B G4 run).

### 3.2 Propulsion and flight-performance analysis

Propulsion is sized by momentum theory (ρ = 1.20, figure-of-merit 0.65, drivetrain
efficiency 0.85, four 10″ disks, disk area 0.203 m²) and cross-checked against the motor
datasheet. All values are pre-bench estimates to be confirmed on a thrust stand at G5.

| Parameter | Value | Basis |
|---|---|---|
| Airframe | Holybro **X500 V2**, 500 mm quad-X, tall gear | sizing |
| Motor / propeller | **SunnySky V2814 KV700** ×4 / **APC 10×4.5MR** | eCalc |
| ESC | **Holybro Tekko32 F4 65 A** 4-in-1 (BLHeli_32, DShot RPM) | BOM |
| Battery | **6S 7500 mAh LiPo** (22.2 V, 166.5 Wh nominal / 142 Wh health-derated) | eCalc |
| All-up weight (with egg) | **≈ 2,565 g** (design ceiling 2.65 kg) | mass budget |
| Max thrust (4 motors) | **7.4–8.0 kg** | eCalc |
| Thrust-to-weight ratio | **≈ 2.9–3.1 : 1** (target 3:1) | eCalc |
| Hover thrust fraction / throttle | **≈ 33 % of max** (MPC_THR_HOVER ≈ 0.35–0.45) | eCalc |
| Hover current / power | **≈ 16 A / ≈ 350 W** @ 22.2 V | eCalc |
| Peak current (full-stick, worst case) | ≈ 90–110 A → pack C-rate ≈ 15 C (trivial for a 120C pack) | eCalc |
| Estimated endurance (health-derated pack) | **≈ 18.3 min**; motors-on flight time ≈ 14.6 min in the 16–17 min mission → **≈ 20 % reserve** | eCalc |
| Cruise speed (MPC_XY_CRUISE) | 10 m/s | config |
| Max XY velocity (MPC_XY_VEL_MAX) | 14 m/s | config |
| Climb cap (MPC_Z_VEL_MAX_UP) | 2.0 m/s | config (ceiling-safe) |
| Fast descent cap (MPC_Z_VEL_MAX_DN) | 3.0 m/s | config |
| Final touchdown speed (MPC_LAND_SPEED) | 0.3 m/s | config (egg-gentle) |

**Excess-thrust / power margin (DCOS).** At the ≈ 2.6 kg all-up weight, hover uses only
**≈ 33 % of available thrust — a ~67 % throttle headroom** — so the motors "loaf" at hover
for thermal and reliability margin and retain full control authority to reject the
late-August SW-monsoon gusts modelled in SITL (base 4 m/s + 3 m/s gusts). The 3:1
thrust-to-weight holds across the egg / no-egg load cases (the 100 g payload is < 4 % of
AUW). The climb cap is deliberately set to
2.0 m/s: at the PX4-default 3 m/s the climb-out overshot to 19.68 m — 0.3 m under the
20 m ceiling — so the cap roughly halves the overshoot for safe headroom.

**Flight-performance verification vs the §2.1 capability.** The derived requirements
(cover the search polygon, transit at 20 m, ≤5 min/sortie) are met in SITL: full
4-sortie missions complete in ≈ 17 min with the cross-sortie registry, transit is flown
at a commanded 19.5 m (0.5 m under the strict 20 m to absorb the ±0.7 m EKF/home-frame
drift), and releases land 0.05–0.30 m from the pad centre across the committed G4 runs.

### 3.3 Avionics and sensor subsystem

| Subsystem | Component | Function |
|---|---|---|
| Flight controller | **Pixhawk 6X** (PX4 v1.17) | Attitude/position control, geofence, FC-level failsafes |
| Companion computer | **Raspberry Pi CM4** | Mission orchestrator, vision, safety watchdog, GCS seam |
| Nadir camera | **Meige OV9281** — 1280 × 720, mono **global shutter**, USB UVC, lens FOV measured at bench (placeholder ~99.7°) | Sole (and only) camera: ArUco decode + white-pad cue + control authority |
| Camera mount | Single-axis **gimbal pitch servo**, PX4 mount driver (stabilized straight-down) | Keeps the frame nadir through pitch; residual trim calibrated at bench |
| Position lock | **ARK Flow** optical flow + **TF-Luna** downward rangefinder | Velocity/position lock for the no-RTK final descent |
| GNSS | non-RTK GPS | Coarse (~0.5–1.5 m); vision owns the final metre |

The single nadir camera is the **sole control authority** — the decode needs its
pixels, and the white-pad acquisition cue lives in the same frame (monochrome is
detector-safe: the decode is grayscale, and brightness + squareness + dark-centre
contrast carry the pad cue). Its global shutter removes motion blur/skew at the
10 m/s sweep. Pixel → ground-coordinate projection composes the vehicle attitude,
and is tilt-gated and median-filtered to reject the noisiest (banked) frames.

### 3.4 Software system architecture — operating modes and safety features

The autonomy stack is a **deterministic, offline, classical** system on the CM4
(Python 3.12, async). There is no LLM, no cloud, and no network in flight (a network use
is an automatic DQ). Layers:

- **Mission orchestrator** (`orchestrator/mission.py`) — the per-sortie state machine of
  §2.3, driven by a per-sortie preflight GO gate; one long-running process flies all
  sorties across the window.
- **Vision worker** (`orchestrator/vision_worker.py`) — nadir-camera ArUco decode + a
  white-pad blob cue, feeding a marker-id-keyed pad registry with k-vote confirmation.
- **Terminal controller** (`orchestrator/tactical_align.py`) — the rung-descent
  land-ON gate, the id-verified LAND gate, the centred-final-rung gate (an unlocked
  final rung defers instead of landing), and the touchdown-gated release.
- **Safety watchdog** (`orchestrator/safety.py`) — a 2 Hz background monitor (below).
- **GCS** — a trimmed Svelte dashboard: live map (transit corridor + no-fly zones
  drawn), camera feed, the ordered 4-of-6 mission-queue editor + per-sortie GO, a
  confirmed-pads readout (id + obtained coordinate — a scoring line), and an
  emergency KILL.

**Operating modes:** PREFLIGHT (per-sortie hold for GO) · TAKEOFF · TRANSIT_INGRESS ·
SEARCH · LOCALIZE · LAND · DROP · TRANSIT_EGRESS · RTH · ABORT.

**Safety features (fail-safe measures, DCOS "fail-safe operating mode"):**

*Companion-side watchdog (2 Hz, phase-aware):*
- Geofence breach → Return-to-Home; no-fly-zone entry → RTH.
- Altitude ceiling: transient poke > 20.5 m → anomaly; sustained > 22 m → RTH.
- Search-floor advisory: below 10 m outside the delivery-descent phases → anomaly.
- GPS 3D-fix loss (debounced) → RTH; datalink loss (debounced) → emergency egress.
- Battery: < 30 % → RTH, < 20 % → LAND-in-place; a **sustained NaN** battery stream
  (sensor dropout) escalates to a loud operator anomaly rather than silently disabling
  battery protection.
- **In-flight camera-freeze guard** — the camera writer publishes each frame by atomic
  replace, so a dead writer would leave the last image frozen on disk and a naïve reader
  would keep "seeing" a stale pad. Both the vision worker and the terminal controller
  reject a nadir frame older than 2 s, so a camera failure during descent reads as *no
  detection* (triggering the climb-back/defer path) and can never satisfy the LAND gate
  on a frozen image.

*Flight-controller-level failsafes (fire even if the companion dies — pinned and read
back at mission start):*
- `NAV_DLL_ACT` = Return (datalink loss), `GF_ACTION` = RTL (geofence breach),
  `NAV_RCL_ACT` = Return (RC / safety-pilot link loss, with `COM_RCL_EXCEPT` so an
  autonomous no-RC sortie is not spuriously returned), and battery thresholds
  (`BAT_LOW/CRIT/EMERGEN_THR` + `COM_LOW_BAT_ACT`) sitting below the companion's 30/20 %
  layer so the two never race.
- `RTL_RETURN_ALT` = 20 m so any failsafe RTL stays legal at the ceiling (PX4's 60 m
  default would bust it).

*Ground-station interlocks:*
- An emergency **KILL** button that force-cuts the motors, plus vehicle arm/disarm — all
  guarded by a command-channel arm session and a CSRF header, with destructive verbs
  auto-disarming the session so an accidental re-fire cannot occur.
- The manual **DROP** is touchdown-gated exactly like the autonomous path: it is refused
  while telemetry reads airborne (unless the operator explicitly forces it) so the egg
  cannot be released from height by a mis-click.

**Imaging record & transmit (rules V1.3).** The system *transmits* imagery live to the
GCS and *records* it: a low-rate JPEG trail (~1 Hz nadir) is persisted per mission into
`runs/<id>/frames/` off the flight-critical path (any recorder failure is a warning,
never mission-fatal).

**Software integrity evidence.** The flight core has 239 automated tests (unit +
integration) that import the real modules — the mission loop, the terminal controller,
the watchdog, the detector, the projection math — all passing, with static type-checking
and linting clean. Behavioural changes are test-driven; the SITL G4 gate exercises the
whole sequence end-to-end (§ Appendix, SITL validation). Beyond the test suite, the
full operator flow was exercised by driving the real GCS in a browser through a
multi-sortie SITL mission; the two operational defects this surfaced (an unreachable
operator override on a short window, and a rung timeout falling through to an
uncentred landing) were fixed, locked by regression tests, and re-validated in flight.

### 3.5 Payload handling mechanism

A single servo on flight-controller **AUX channel 9** actuates the egg hold: PWM **1900
µs = release**, **1100 µs = hold** (the bipolar mapping straddles centre so the servo
genuinely crosses between positions). One release mechanism → one egg per sortie
(`payload_id` always 0); the rules prohibit simultaneous multi-cargo release and
winching, both respected. The cradle is a compliant foam/TPU print (egg + cradle mass
budget **80–120 g**, 100 g in the mass rollup) that absorbs the touchdown and opens under
gravity when the servo releases. Release is **hard-gated behind a confirmed touchdown** (2 s settle)
and is idempotent per sortie (a ledger prevents a double release across retries).
Per DCOS, the payload is securely retained during flight and the mechanism has no
unsafe behaviour.

### 3.6 Communication system

- **Telemetry / C2:** a ≥ 500 m line-of-sight datalink in a recommended band
  (**920–925 / 2400–2500 / 5725–5850 MHz**). **4G LTE / SIM is not used** (banned).
- **Safety-pilot RC:** ELRS (long-range, in the 2.4 GHz recommended band).
- **Imagery:** the CM4 relays camera frames to the GCS and to ≥ 1 external display for
  evaluation, per the rules.
- **On-board routing:** a mavlink-router fans the flight link to the orchestrator
  (offboard), the dashboard, and an optional ground QGroundControl. The QGC endpoint
  **defaults to loopback** — it is opened to a specific ground host only deliberately, so
  the armed aircraft's MAVLink control plane is not exposed unauthenticated on the field
  network.

### 3.7 Operating procedures (normal and emergency)

**Normal operation:** the operator enters the committee's assignments once as an
ordered mission queue (up to 4 of the 6 ids). Then per sortie: preflight readiness
board green (link, arm-ready, EKF, home, sensors, GPS fix, battery, on-ground,
geofence, camera-fresh) → operator confirms the egg loaded → GO (the pad id resolves
from the queue; a manual pick can override one sortie) → autonomous sortie → land +
disarm at L&R → resupply → repeat. The window clock and the per-sortie time-budget
gate manage the 4-sortie/20-minute schedule; the gate refuses a launch that cannot
finish inside the window unless the operator explicitly forces it, accepting the
overtime penalty (the readiness board shows the short window as an advisory, so the
operator's override stays reachable).

**Emergency operation:** the safety pilot can take manual control at any time; the GCS
KILL cuts motors instantly. Automatic failsafes (companion watchdog + FC-level RTL on
geofence/datalink/RC/battery) recover or land the vehicle without operator action. On
any unhandled mission-loop error the orchestrator commands an emergency RTH→LAND and
falls through to the FC failsafe. The team may return mid-window and restart a sortie
free of penalty as long as the window has not expired.

### 3.8 Airworthiness — DCOS compliance summary (Appendix B)

| DCOS criterion | Compliance |
|---|---|
| Flight-performance margin across all load configs | T/W + hover-throttle margin (§3.2) |
| Envelope protection (min speed, tilt, rates) | PX4 limits: `MPC_TILTMAX_AIR` 35°, `MC_YAWRATE_MAX` 50°/s, speed caps (§3.2) |
| Propulsion excess-thrust margin | §3.2 hover-throttle margin |
| Power reserve (no energy starvation) | Battery sized for 4 sorties + reserve; FC battery failsafe (§3.4) |
| Airframe load (flight/ground/transport) | §3.1 |
| Redundancy (backup power, redundant sensors) | GCS backup power; optical-flow + rangefinder augment GPS; FC + companion split |
| HW compatibility (protocols, power, comms) | PX4/MAVLink throughout; §3.6 band compliance |
| Secure fastening / payload retention | §3.1, §3.5 |
| Clearance between moving parts | §3.1 |
| Secure wiring + environmental protection | 10/16 AWG silicone leads, XT90/soldered joints, strain-relieved; conformal-coat / enclosure for the CM4 + FC against dust/rain (§3.1) |
| Envelope protection + fail-safe modes activatable any time | §3.4 watchdog + FC failsafes + GCS KILL |
| Normal + emergency procedures | §3.7 |
| PPE per role | §1 |

---

## Appendix — Bill of Materials & SITL Validation

### A. Bill of Materials

Canonical design (500 mm / 10″ / ≈2.6 kg). Owned items are verified, not re-bought.
Costs are THB ranges; full detail in `docs/BOM_REPORT.md`.

**Owned (verify only):** Pixhawk 6X (PX4 1.17, calibrated) · Raspberry Pi CM4 + baseboard ·
ELRS RC chain (TX16S + RadioMaster Nomad dual-band TX + DBR4 diversity RX, HITL-verified) ·
GCS laptop + SITL dev machine · DXF 6S 7500 mAh 120C LiPo mission pack.

| Group | Item | Selected part | Qty | Est. THB | Priority |
|---|---|---|---|---|---|
| Airframe | Frame | Holybro X500 V2, 500 mm, tall gear | 1 | 2,500–4,500 | P0 |
| Propulsion | Motors | SunnySky V2814 KV700 (~2 kg thrust ea.) | 4 (+1 spare) | 4,500–6,500 | P0 |
| Propulsion | Props | APC 10×4.5MR | 4 sets | 600–1,200 | P0 |
| Propulsion | ESC | Holybro Tekko32 F4 65 A 4-in-1 (BLHeli_32) | 1 | 1,800–3,500 | P0 |
| Power | Backup battery | 6S 6000–6500 mAh ≤950 g | 1 | 3,500–6,500 | P0 |
| Power | Module + UBEC + wiring | Hall ≥150 A PM, 5 V/6 A UBEC, XT90/10 AWG | set | 2,300–4,500 | P1 |
| Nav | Optical flow | ARK Flow (CAN) — longest lead, order first | 1 | 4,500–6,500 | P0 |
| Nav | Rangefinder | TF-Luna (downward) | 1 | 900–1,500 | P1 |
| Vision | Nadir camera (single) | **Meige OV9281** USB UVC — mono global shutter, 1280×720 (owned; lens FOV measured at bench) | 1 | owned | P0 |
| Vision | Gimbal servo + mount | Single-axis pitch servo (PX4 mount driver, stabilized nadir) + vib-isolated cage | 1 | 300–900 | P0 |
| Payload | Release servo | Metal-gear (MG90S–DS3225 class), AUX 9 | 1 (+1 spare) | 300–800 | P0 |
| Payload | Egg cradle + practice eggs | Foam/TPU print, 80–120 g; ~2 dozen eggs | 1 | 450–800 | P1 |
| Misc | Mounts, spares, field network | vib-isolated mounts, spares kit, local AP | — | 1,200–3,000 | P1–P2 |

**Estimated total (backup pack bought, mission pack + camera owned):**
**≈ 23,000–40,000 THB** — comfortably within a student-competition budget. (The
eCalc propulsion + power scope, which also prices a charger and both packs new,
banded at ≈ 25,000–34,000 THB.)
_(A separate `build_bom.py`/`AAVC_BOM.xlsx` describes an earlier 700 mm/15″/3.2 kg concept
that was superseded by this 500 mm/2.6 kg design and is retained only as history.)_

### B. SITL validation results (G4 gate)

The full 4-sortie mission was flown in PX4 SITL + Gazebo on the IAAI KMITL field geometry
(config coordinates identical to the rules PDF, with a modelled SW-monsoon wind: 4 m/s
base + 3 m/s gusts), then checked by an independent, fail-closed post-flight verifier
(`tools/verify_flight.py`).

**Fresh confirmation run (this report — final flight configuration: single OV9281
nadir camera, System-ID tuned gains; SEED 7, pads/assigned 3/2/4/6):**

| Check | Result |
|---|---|
| Eggs delivered on the correct assigned pad | **4 / 4**, verifier-confirmed id — 0.20 / 0.26 / 0.10 / 0.30 m from the truth centre, all landed-before-release |
| Transit coordinates in order (both ways) | **24 / 24**, 0 misses |
| Return-to-L&R landings | 4 / 4, 2.6–2.7 m from L&R |
| Search floor / geofence / no-fly | all clear (no sub-10 m outside the descent; track inside geofence; clear of the no-fly zone) |
| Altitude | max 20.33 m — below the 20.5 m warn line (no ceiling flag) |
| Operation window | **1001 s ≈ 16.7 min** ≤ 1200 s |
| Camera-freeze / RC-loss anomalies | none (the frame-age gate did not false-fire; `NAV_RCL_ACT`=Return did not spuriously RTL) |
| Verifier verdict | **PASS — 19 checks ok, 0 warnings** |

The same seeded layout flown pre-tuning released 0.05–0.26 m (PASS, one known
two-stage-climb-closure warning), and a dedicated sortie flown on grayscale `--mono`
frames confirmed the monochrome OV9281 pipeline end-to-end (decode, confirm, land-ON
0.14 m) with no detector change.

**Prior committed validation:**

- **V1.3 confirmation run (SEED 42 — pads 6/1/5/3, assigned 3/1/5/6):** 4/4 delivered
  at 0.10–0.32 m, transit 24/24, window 971 s ≈ 16.2 min, **1,174 JPEG frames**
  persisted (the rules' "record" line). One return leg drifted to 20.87 m in the MSL
  altitude frame — the documented **no-RTK altitude-frame drift**: PX4 re-captures its
  home MSL at every arming, so a per-sortie AGL→MSL re-cache can wander ±~0.7 m; the
  vehicle held its commanded 19.5 m AGL setpoint while the MSL frame drifted. On the
  real aircraft the downward rangefinder (TF-Luna) + optical flow provide true AGL and
  remove this drift; the in-flight watchdog treats it as warn-only (RTH only above
  22 m). The two later runs above held ≤ 20.33 m.
- **Five seeded layouts (I1b/I2b/I4a/I5a/I5c, previous session):** 4/4 delivered every
  run, windows 16.4–17.1 min, peak altitude ≤ 20.35 m, verifier **PASS 0-warn** —
  establishing that the delivery, transit-scoring, and window performance are
  repeatable and that the SEED-42 altitude excursion is a run-to-run frame-drift
  boundary case, not a systematic error.
- **End-to-end operator flow (browser-driven):** the real GCS was operated through a
  multi-sortie SITL mission off the 4-of-6 mission queue. The two operational defects
  this surfaced — an unreachable operator override on a short window, and a rung
  timeout falling through to an uncentred landing (2.46 m off-pad) — were fixed,
  locked by regression tests, and re-validated (post-fix mission PASS 0-warn, with the
  next sortie correctly refused by the time-budget gate at 295 s remaining).

---

*Report structured to AAVC 2026 Rules & Regulations V1.3, Appendix A. The flight system
is deterministic, classical-CV, offline (no LLM, no network — network use is a DQ).*
