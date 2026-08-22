# Real-Flight Runbook — AAVC 2026 (SITL → real 6X → free flight)

Take the SITL-validated mission (G4) onto the **real Pixhawk 6X + Raspberry Pi CM4**
drone and out to free flight. Bring it up in the **locked gates** (`CLAUDE.md §7`)
— do **not** skip to free flight. Each gate gates the next.

> The aircraft is a real hexacopter with real motors and a real payload. Treat every step
> as flight-test discipline: **props off through G5**, geofence + battery limits set,
> a manual kill (QGC / RC) within reach, eyes on the aircraft.
> (G6 — the tethered gate — was **DROPPED 2026-08-16**, CLAUDE.md §7: the
> aircraft had already flown, so a tether proved nothing new. The first
> propellers-on flight is G7.)

---

## Topology (onboard CM4)

```
  Pixhawk 6X ──serial /dev/ttyACM0 (USB) or /dev/ttyAMA0 (UART) @921600──► mavlink-router
                                                                            ├─► :14540  orchestrator (MAVSDK, --connect udpin://0.0.0.0:14540)
                                                                            └─► :14550  QGC over the telemetry radio / wifi (manual kill)
  OV9281 nadir camera (hard-mounted nadir, NO gimbal) ──► sitl/camera_grabber.py ──► /tmp/aavc_{nadir,frame}.jpg ──► vision_worker
```

One command brings the whole stack up onboard (see `cm4/launch_flight.sh`):

```bash
# bench (G5): dashboard up, operator GO + in-browser kill
SERIAL=/dev/ttyACM0 BACKEND=v4l2 bash cm4/launch_flight.sh
# field, UNATTENDED: headless, AUTO-GO when preflight criticals pass
HEADLESS=1 SERIAL=/dev/ttyACM0 BACKEND=v4l2 bash cm4/launch_flight.sh
```

⚠ **There are TWO real-flight entry points and they LAUNCH DIFFERENTLY. Know
which one you started.**

| | `cm4/launch_flight.sh` | `sitl/run_mission.sh` with `REAL=1` |
|---|---|---|
| Started by | hand, on the CM4 | the console's 🚀 button, over ssh |
| `HEADLESS=1` | **AUTO-GO** — the aircraft launches itself once preflight criticals pass | n/a |
| RC gate | none | **`RC_GO=1` by default** — 🚀 only STAGES; the safety pilot arms on RC and flips OFFBOARD to release |

The console path is the one used at KMUTNB and the one the competition will
fly, and it cannot launch the aircraft: the web button stages, the pilot
releases (`docs/REAL_FLIGHT_GCS.md`, "RC-GO"). `HEADLESS=1` here is the older
unattended path and has **no such interlock** — do not use it with people near
the aircraft, and do not reach for it out of habit at the field because it is
the command written down first.

No internet is used in flight (build the `.venv` once offline from `requirements.lock`).

---

## Phase 0 — Flight firmware + FC calibration (QGroundControl, ONE TIME)

The 6X may be on the **HIL build** (`fmu-v6x_hil`, `SYS_HITL=1`) from HITL work — that
**cannot fly**. Reflash a flight firmware and calibrate:

1. **Flash flight firmware.** Recommended **PX4 1.17.0 `fmu-v6x_default`** (the board runs
   1.17.0; built from the `~/PX4-Autopilot-v1.17` worktree the SITL gains were tuned
   against), or latest stable (then re-validate gains at G5/G7).
   `make px4_fmu-v6x_default upload` in the PX4 tree, or flash via QGC.
2. **Confirm `SYS_HITL=0`** and set the **real airframe** (*Generic Hexarotor X*,
   `SYS_AUTOSTART=6001`) + the **6-motor map** (`PWM_MAIN_FUNC1..6 = 101..106`).
3. **Calibrate:** accelerometer, gyro, level horizon, magnetometer, ESCs, battery,
   and RC + flight modes if an RC link is used.
4. **Geofence + failsafes** in QGC: a generous inclusion fence around the field,
   datalink-loss → RTL, battery → RTH. (The orchestrator also uploads the
   `controlled_airspace` fence + datalink RTL at connect; QGC's are the independent net.)
5. **Bench-arm test (props OFF):** confirm the FC arms on real sensors. (This is where
   the HITL arming-health block disappears — it was firmware-specific.)

`COM_DISARM_LAND=-1`, the `MPC_*`/`MC_*` limits, and the tuned rate gains are applied
**at runtime** by the orchestrator (`apply_param_overrides`) — nothing extra to preload.

---

## G5 — HW bench (props OFF)

Goal: validate the orchestrator ↔ real-FC seam + the actuators, motors **off the air**.

- [ ] **Props OFF.** Battery in (or bench PSU); 6X on USB/UART; cameras connected.
- [ ] Start the stack: `bash cm4/launch_flight.sh` (bench mode). Confirm the orchestrator
      logs `cached home MSL altitude`, `home=(…)` at the field, `applied N/N PX4 tuning
      params`, geofence uploaded, telemetry streaming.
- [ ] **Camera grabber:** frames fresh in `/tmp/aavc_nadir.jpg` (preflight's camera-age
      critical passes). OV9281 knobs: `--fourcc GREY --fps 50` (bench-pick the final
      values; gray frames are replicated to BGR before write). If the camera isn't
      ready, run a synthetic feeder and pass `NO_CAMERA=1`.
- [ ] **Preflight green + arm:** all criticals pass; GO; the FC **arms** (no takeoff with
      props off — expect it to spin up motors logically only).
- [ ] **Motor map:** QGC *Motors* test — spin **all six** one at a time and confirm each
      motor's position and direction match the airframe diagram. Motor N corresponds to
      `CA_ROTOR(N-1)`; the SITL model `sitl/models/eft_x6100_base` documents the same table.
- [ ] **Egg-release servo:** trigger a release (mission, or the dashboard drop command).
      Confirm the **AUX channel** moves the servo release→hold and the egg hold opens
      GENTLY (the cargo is a raw egg). Lock the real **channel + PWM band** into the
      config `connection:` block (`drop_servo_channel`, `drop_servo_pwm_release/hold`;
      `drop_payload_count=1`).
- [ ] **Cargo-box fit (rules V1.3):** the organiser cargo is a heart-shaped box —
      **~16 × 7 × 18 cm (W×D×H), 300-gsm art card, with a handle loop**, one raw
      no.0 egg inside. Verify the egg-hold bay physically accepts THIS box (not just
      a bare egg) and that release clears the handle.
- [ ] **Frame-recorder storage (rules V1.3 "record and transmit"):** the mission
      recorder writes ~1 Hz JPEGs to `runs/<id>/frames/` (~200–300 MB per 20-min
      window). Confirm the CM4 SD card has headroom for a full day of sorties, or
      lower `recording.hz` / `recording.jpeg_quality` (or set `recording.enabled:
      false` to keep transmit-only).
- [x] ~~**Camera gimbal**~~ — **NOT FITTED** (operator 2026-08-16). The OV9281 is
      hard mounted looking down and `gimbal.enabled` is false, so there is nothing
      to configure here. What the gimbal used to buy is now bought by the
      roll/pitch composition in `vision/projection.py` (see CLAUDE.md §2), which
      makes the `fov_deg` measurement below matter more, not less.
- [ ] **FPV camera + VTX (added to the aircraft 2026-08-16) — three checks:**
      - **Frequency plan.** There are now up to four radios on one airframe: the
        ELRS RC link (TX16S + Nomad + DBR4), MAVLink telemetry, the FPV VTX
        (usually 5.8 GHz) and the CM4's WiFi (2.4 or 5 GHz). Write down what band
        each ACTUALLY uses — do not assume — then separate them.
        ⚠ Priority when they collide: **the RC link never gives ground**. It is
        the safety pilot's authority and the one link whose loss is
        unrecoverable. WiFi yields first (since the FPV took over the imaging
        downlink, WiFi is now a debug convenience, not a scored requirement),
        telemetry second. The common trap is moving the CM4 to 2.4 GHz to dodge
        a 5.8 GHz VTX and landing it straight on top of a 2.4 GHz ELRS link.
      - **Weight.** VTX + camera + antenna is roughly 50–150 g and is NOT in the
        8.22 kg AUW every energy/endurance figure was computed from. Weigh the
        aircraft **with the FPV fitted**, not bare, and update `battery:` seeds if
        it moved materially.
      - **Nadir FOV obstruction.** The mission camera looks straight down through
        a wide cone. Confirm no FPV antenna, lens or bracket enters it — a mast in
        frame costs decode area exactly where pads are searched for. Check on a
        real frame, not by eye.
- [x] **Camera stream (OV9281): DONE 2026-08-17** — real grabber (`make camera-real
      BACKEND=v4l2`, default args) wrote fresh 1280×720 mono→BGR frames to
      /tmp on the CM4 continuously through the bench session. Final
      `--fourcc/--fps` values still bench-pickable if needed.
- [x] **Detector: DONE 2026-08-17** — operator held an ArUco marker (phone screen —
      harsher than paper: backlight + refresh) under the real nadir camera indoors
      over noisy terrazzo; `find_landing_pads` on the live frame decoded
      **id 1 at confidence 0.95** — operator confirmed id 1 was the marker
      shown (picture-to-picture, both directions of the evidence) — (radius
      45 px, tilted view, pure-ArUco path —
      `pad_side_px=0` as expected with no white pad in frame). The field-pad
      variant (full 1×1 m print at sweep distance) folds into G7's first flight.
- [ ] **Downward rangefinder (Benewake TFmini-S, 0.1-12 m):** wire it to a free serial
      port, set `SENS_TFMINI_CFG` to that port (the value is deliberately NOT pinned in
      config — picking the wrong port steals one the CM4 or GPS needs), then confirm
      `listener distance_sensor` streams and the reading matches a tape measure at
      0.5 / 2 / 5 m. `EKF2_RNG_CTRL=1` is already pinned by the mission's param block.
      Optical flow was CUT from the project (2026-07-22) — there is no flow module to
      configure, and `EKF2_OF_CTRL` is pinned to 0.
- [ ] **Battery gauge (VOLTAGE ONLY — PM02D feeds the FC alone):** the PM03D is OUT; the
      PM02D installed 2026-08-20 powers only the avionics, so the FC still cannot
      sense motor current and the gauge runs on voltage alone. Set `BAT1_CAPACITY=-1`
      (this SELECTS the voltage-only branch; ANY positive value silently re-arms a
      current-fused estimate that reads optimistic on this wiring — do NOT set 7500),
      `BAT1_N_CELLS=6`, and the CURRENT pack's endpoints per cell — the 17000 mAh
      semi-solid is `BAT1_V_CHARGED=4.18` / `BAT1_V_EMPTY=3.77` (25.1 V full / 22.6 V
      empty ÷ 6). ⚠ Re-verify FC voltage vs a multimeter ON THE PM02D wiring once
      (the 2026-08-16 `BAT1_V_DIV` closure belonged to the old converter).
      Then run `.venv/bin/python tools/preflight_params.py` and confirm the
      BOARD block is all ✔ (it checks every one of these). The failsafe fractions
      (0.25 / 0.15 / 0.07) are fractions of that voltage span — re-confirm against the
      pack's discharge curve.

## G6 — dropped 2026-08-16

Tethered hover was dropped: this airframe has already flown several real flights,
so a tether proves nothing new. Its two real items live on — the camera
`fov_deg` calibration is a **G5 ground procedure** (measured **74.2°** on the real
WSD-9781 lens, replacing the old 99.7° placeholder), and the first
**land-ON + release** over a printed pad is folded into **G7's first flight**.
⚠ This aircraft has **NO optical flow and NO gimbal — only the downward Lidar
(TFmini-S)** — so the old flow-lock (`EKF2_OF_CTRL` A/B) and gimbal-trim
(`depression_deg`) steps are gone with the section; `EKF2_OF_CTRL` is pinned 0.

## G7 — HW free flight (practice field)

Goal: the full multi-sortie delivery mission outdoors with **real cameras** seeing the field.

- [ ] Field set up: ≤4 printed 1×1 m ArUco pads (distinct ids 1–6) in a marked search area;
      transit waypoints + geofence + battery limits verified in QGC; manual kill ready;
      **eyes on**. Mark an L&R spot and mirror the real coordinates into the config
      (`site.center` / `ground_operation` / `transit_route` for the practice field).
- [ ] **Real camera required** — gz/synthetic frames do NOT see the field.
- [ ] Field run: `HEADLESS=1 … bash cm4/launch_flight.sh` with `--assigned-ids` (or set the
      4-of-6 mission queue on the dashboard, then GO per sortie). Confirm per sortie:
      takeoff → transit P1→P2→P3 at
      20 m → sweep/registry → land ON the assigned pad → release after touchdown →
      egress transit → land + disarm at L&R → resupply → next sortie.
- [ ] Post-flight: `--truth-json` audit + `tools/verify_flight.py runs/<id>/audit.jsonl`
      (altitude bands, transit passes, touchdown-before-release, L&R landings, window).

## G8 — Dress rehearsal

- [ ] Full mission + **live egg deliveries** within the 20-minute window, on the real field,
      end-to-end with no operator intervention beyond setting the mission queue once + the
      per-sortie GO (egg confirm) + the kill-switch standby. Practice the resupply drill
      (egg + battery swap) against the clock.

---

## Notes
- **The nadir camera is the sole control authority** for align/descend/land-ON/release —
  its calibration (G6) is flight-critical. There is no second camera: the white-pad blob
  cue lives in the same nadir frame.
- **CM4 dies mid-flight?** The FC-level failsafes (geofence RTL, datalink-loss RTL, battery
  RTH) are the net — `cm4/launch_flight.sh` does NOT auto-restart the mission (re-arming
  mid-air is unsafe).
- **Back to flight from HITL / back to HITL:** reflash the appropriate firmware
  (`fmu-v6x_default` for flight, `fmu-v6x_hil` for HITL) — see `docs/HITL.md`.
