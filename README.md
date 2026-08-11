# AAVC 2026 — Lightweight Autonomous Egg-Delivery Build

KMUTNB entry for the **Autonomous Aerial Vehicle Challenge (AAVC) 2026** at
IAAI KMITL (official Rules & Regulations **V1.3**, July 2026 — digest in
`docs/RULES_AAVC2026.md`). This is the **lightweight rebuild**: a
deterministic, classical-CV flight core with no LLM, no cloud, and no heavy ML
— everything runs on a Raspberry Pi CM4 CPU beside a Pixhawk 6X.

---

## The Mission (rules V1.3)

An autonomous PX4 hexacopter (EFT X6100) delivers **one fragile egg cargo per sortie** to
the **landing pad matching the committee-assigned ArUco marker id**. Up to
**4 pads** (1×1 m white, black ⌀750 mm ring, central 400 mm ArUco
`DICT_4X4_50`, ids 1–6) sit in a ~227×60 m search area; their positions are
unknown — only the id is handed to the operator at each resupply. Each sortie:

1. **Launch** at the Launch & Recovery site and fly the **mandatory transit
   route P1→P2→P3 at strictly 20 m** (scored per coordinate passed).
2. **Search** the search area (10–20 m band): decode every pad in view into a
   **cross-sortie registry**; sorties whose assigned pad is already registered
   skip the sweep and fly **direct**.
3. **Align + descend** over the assigned pad on the nadir camera; an
   **id-verified LAND gate** refuses to land unless the assigned id was
   actually decoded during the approach.
4. **Land ON the pad** and **release the egg only after touchdown** (scoring:
   landed-before-release + intact cargo), then climb out.
5. Fly the **egress transit P3→P2→P1**, land at L&R, **disarm**, resupply,
   and take the next assignment — up to 4 sorties inside the **20-minute
   operation window** (per-minute penalty after).

GPS is **coarse (no RTK)**, so the cameras — not GPS — own the final metre.

- **Altitudes:** transit strictly 20 m · search ≥10 m · below 10 m only for
  the delivery descent · ceiling 20 m AGL (hard competition limits).
- **Profile:** `competition` — 20-min window, 1 egg/sortie, ≤4 sorties.
- **Posture:** fast-but-safe. An independent watchdog can pre-empt the mission.

---

## Architecture (lean module map)

```
mission_brain/        Deterministic mission planning (no LLM)
  live_plan.py          per-sortie plan: transit + search + serve + LAND
  profile.py            competition / production envelopes (ceiling, floor, sorties)
  schemas.py            Pydantic models (MissionPlan, Coordinate, ...)
  search_pattern.py     boustrophedon sweep of the search-area polygon

orchestrator/         Async mission state machine
  main.py               Entry point: connect -> GPS -> watchdog -> vision -> fly
  mission.py            run_delivery_mission: per-sortie gate -> transit ->
                        sweep/registry -> land-ON serve -> egress -> land+disarm
  tactical_align.py     Visual-servo descend rungs + id-verified LAND gate +
                        touchdown-gated egg release (nadir camera)
  vision_worker.py      Nadir-camera multi-pad detect loop (+ dashboard feed)
  target_tracker.py     Marker-id-keyed pad registry (k-decoded-vote confirm)
  drop_trajectory.py    Ballistic predictor (audit/overlay only — we release landed)
  safety.py             Watchdog: battery / GPS / geofence / NO-FLY / ceiling /
                        search-floor / datalink / time
  state.py, audit.py    Shared state + crash-safe audit trail (1 Hz TELEM samples)

mavlink_adapter/      MAVSDK command facade + telemetry fan-in
vision/               Classical CV: ArUco pad detector + geo-projection
  detectors/aruco.py    DICT_4X4_50 decode + ROI upscale booster + white-pad cue
                        (+ render_pad_bgr — the ONE pad renderer: sim + tests)
  projection.py         pixel -> (lat, lon) ray/ground intersection
dashboard/            FastAPI + Svelte GCS (map with transit/no-fly layers,
                      4-of-6 mission-queue editor + per-sortie GO,
                      confirmed-pads readout)
sitl/                 PX4 SITL + Gazebo KMITL field, pad spawner, camera bridge
tools/                gen_pads.py (pad models), verify_flight.py (post-flight
                      drone-response verifier), gen_grass.py
tests/                pytest: pad detector, delivery mission, plan, align, safety
```

---

## Quickstart

```bash
# 1. Install (creates .venv, installs the lightweight stack + dev tools)
make install

# 2. Launch PX4 SITL + Gazebo with the IAAI KMITL field
make sitl

# 3. Spawn the 4 ArUco landing pads into the running sim (SEED=n re-rolls)
make spawn-targets

# 4. Bridge the Gazebo camera to the frame files (system python3 — uses gz apt pkg)
make camera-bridge

# 5. Fly the mission (orchestrator + dashboard; set the 4-of-6 mission queue in
#    the pre-flight card, then one GO per sortie). Headless committee stand-in:
make run    # then queue + GO per sortie from http://127.0.0.1:8765
#    .venv/bin/python -m orchestrator.main --no-dashboard --assigned-ids "3,1,4,6" \
#        --truth-json /tmp/aavc_targets.json

# 6. Verify the drone actually flew per the rules (post-flight):
.venv/bin/python tools/verify_flight.py runs/<mission_id>/audit.jsonl
```

Run `make test` for the pytest suite and `make lint` for ruff + mypy.

---

## Vision-Guided Land-ON-Pad Algorithm

A single gimbal-stabilized nadir camera (Meige OV9281 — monochrome global
shutter, 1280×720, USB UVC) drives a two-stage approach to each pad:

1. **Search-altitude acquisition (sweep).** At the 12 m sweep the 1 m white
   pad is a ~45 px high-contrast blob in the 1280×720 nadir frame while the
   400 mm marker is only ~18 px — right at the decoder's floor. The detector
   (`find_landing_pads`) fuses a **white-square/dark-centre cue** with ArUco
   decode boosted by a ×4 ROI upscale retry; every decoded id feeds the
   cross-sortie registry, and undecoded candidates are revisited at the 10 m
   floor to read their ids. Both the decode and the cue are mono-safe (the
   saturation gate is trivially true on gray; brightness + squareness + the
   dark-centre contrast carry the discrimination).

2. **Nadir alignment + descent (over the pad).** Overhead, `tactical_align`
   walks the aircraft down altitude rungs (~12 → 8 → 5 → 3 m), re-centring on
   the marker at each rung (median-fused, tilt-gated, size-prior-checked).
   A pad decoded as a **different id never steers the descent**, and the
   **id-verified LAND gate** climbs away and defers if the assigned id was
   never read — the egg is committed the moment the vehicle lands, so it never
   lands blind (`gps_fallback=False`; V1.3 reverses "an attempt beats no
   drop").

3. **Land ON + release.** The final rung tolerance (0.35 m) centres the gear
   on the 1 m pad; PX4 crawls the touchdown (`MPC_LAND_SPEED=0.3`), the
   vehicle settles 2 s, and only then the servo opens — landed-before-release
   is a scoring line and the egg must survive. The aircraft stays ARMED
   (`COM_DISARM_LAND=-1`), climbs out, and flies the egress transit home.

No model weights, no GPU.

---

## Fast-but-Safe Flight + Safety Watchdog

The mission flies briskly (cruise 10 m/s) but an **independent
`SafetyWatchdog`** runs alongside the mission loop and can pre-empt it:

- **Battery:** RTH at 30 %, LAND-in-place at 20 %.
- **GPS / datalink loss:** debounced thresholds (~5 s) escalate to RTH.
- **Geofence:** breach → RTH back inside; proximity margin warns first.
- **No-fly zone (V1.3):** entry → immediate RTH.
- **Altitude:** >20.5 m warns; >22 m sustained → RTH. Sub-10 m outside the
  delivery descent is audited. `RTL_RETURN_ALT=20` keeps every failsafe
  return at the ceiling (PX4's default 60 m would bust it).
- **Time budget:** reserves margin within the 20-min window; the per-sortie
  gate refuses a launch the window can't cover (operator can FORCE — the
  overtime penalty is their call).

FC-level failsafes (geofence RTL, datalink-loss RTL) are uploaded to PX4 so the
aircraft is protected even if the companion computer dies. The safety pilot's RC
override is the ultimate failsafe.

---

## Security & Reproducible Builds

- **Dashboard binds `127.0.0.1` by default.** The command channel (arm / takeoff
  / drop / land / abort) has **no token auth** — its only guards are the
  `X-AAVC-CMD` request header (a CSRF mitigation: no CORS origin is whitelisted,
  so a cross-origin browser POST can't add it) and the "arm to command" session
  toggle. That trust model assumes a private/loopback bind. **Do NOT set
  `AAVC_DASHBOARD_HOST=0.0.0.0` without first putting the dashboard behind auth**
  (a reverse proxy + token / client cert). WebSocket handshakes are accepted only
  from same-origin or loopback origins.
- **No internet / 4G in flight.** The AAVC site bans network access (= DQ) and
  the mission is fully deterministic + offline. Build the venv **once**
  beforehand and pin it: `pip install -e ".[dev,tuning]" -c requirements.lock`
  (regenerate with `make lock`) so the field build is byte-reproducible.
- **Camera frames** are written to `/tmp/aavc_*.png` with `0600` perms; with
  `/tmp`'s sticky bit this keeps another local user from reading or swapping the
  frames the flight loop and dashboard trust.

---

## Hardware

| Layer | Choice |
|---|---|
| Airframe | **EFT X6100 hexacopter** (X layout, 1.000 m wheelbase, 18" props, AUW 7.17 kg) |
| Flight controller | **Pixhawk 6X** |
| Companion compute | **Raspberry Pi CM4** (classical CV, no GPU) |
| Firmware | **PX4 v1.17.0**, airframe *Generic Hexarotor X* (`SYS_AUTOSTART=6001`) |
| Camera | **Meige OV9281** USB UVC — mono global shutter, 1280×720 (decode needs the width), on a **gimbal-stabilized nadir pitch servo** (PX4 mount driver, VERIFY-AT-G5) |
| Sensors | **Benewake TFmini-S** downward lidar (AGL for the descent + touchdown gate; `EKF2_RNG_CTRL=1`). No optical flow — dropped 2026-07-22, so the camera alone owns the final metre |
| Drop | Servo egg-release × 1 (one cargo per sortie; resupply between) |
| Power | **DXF 6S 7500 mAh 140C** via a **Holybro PM03D** (digital power module; `BAT1_*` calibration is a G5 item) |
| RC / telemetry | **RadioMaster TX16S mk2** + **Nomad** ELRS TX + **DBR4** diversity RX (CRSF on TELEM1) |
| GPS | Coarse, **no RTK** — the camera owns final-metre accuracy |
| GCS | Trimmed Svelte dashboard (map, camera, mission queue + per-sortie GO, pad readout) |

**Field:** IAAI KMITL — airspace ~296×167 m, search area ~227×60 m.
**Ceiling:** 20 m AGL. **Targets:** ≤4 ArUco landing pads (ids 1–6).

---

## License

Internal KMUTNB project. Do not redistribute without permission.
