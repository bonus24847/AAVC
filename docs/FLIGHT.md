# Real-Flight Runbook — AAVC 2026 (SITL → real 6X → free flight)

Take the SITL-validated mission (G4) onto the **real Pixhawk 6X + Raspberry Pi CM4**
drone and out to free flight. Bring it up in the **locked gates** (`CLAUDE.md §7`)
— do **not** skip to free flight. Each gate gates the next.

> The aircraft is a real hexacopter with real motors and a real payload. Treat every step
> as flight-test discipline: props off until G6, geofence + battery limits set,
> a manual kill (QGC / RC) within reach, eyes on the aircraft.

---

## Topology (onboard CM4)

```
  Pixhawk 6X ──serial /dev/ttyACM0 (USB) or /dev/ttyAMA0 (UART) @921600──► mavlink-router
                                                                            ├─► :14540  orchestrator (MAVSDK, --connect udpin://0.0.0.0:14540)
                                                                            └─► :14550  QGC over the telemetry radio / wifi (manual kill)
  OV9281 nadir camera (gimbal) ──► sitl/camera_grabber.py ──► /tmp/aavc_{nadir,frame}.png ──► vision_worker
```

One command brings the whole stack up onboard (see `cm4/launch_flight.sh`):

```bash
# bench (G5/G6): dashboard up, operator GO + in-browser kill
SERIAL=/dev/ttyACM0 BACKEND=v4l2 bash cm4/launch_flight.sh
# field  (G7/G8): headless, auto-GO when preflight criticals pass
HEADLESS=1 SERIAL=/dev/ttyACM0 BACKEND=v4l2 bash cm4/launch_flight.sh
```

No internet is used in flight (build the `.venv` once offline from `requirements.lock`).

---

## Phase 0 — Flight firmware + FC calibration (QGroundControl, ONE TIME)

The 6X may be on the **HIL build** (`fmu-v6x_hil`, `SYS_HITL=1`) from HITL work — that
**cannot fly**. Reflash a flight firmware and calibrate:

1. **Flash flight firmware.** Recommended **PX4 1.15.4 `fmu-v6x_default`** (built from the
   local `~/PX4-Autopilot` source → matches the SITL the gains were tuned against), or
   latest stable (then re-validate gains at G5/G7).
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
- [ ] **Camera grabber:** frames fresh in `/tmp/aavc_nadir.png` (preflight's camera-age
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
- [ ] **Power module (Holybro PM03D):** `BAT1_*` is unset out of the box. In QGC *Power*
      set `BAT1_N_CELLS=6` and `BAT1_CAPACITY=7500` for the DXF 6S 7500 mAh pack, then
      verify the reported pack voltage against a meter and that current reads ~0 A at
      idle. The failsafe fractions (0.25 / 0.15 / 0.07) must be re-confirmed against the
      real pack's discharge curve before G7.

## G6 — Tethered

Goal: stable hover + the vision/descent/drop loop on real attitude, on a tether.

- [ ] Tether rigged; props ON; open area; manual kill ready.
- [ ] **Hover:** arm + a low hover (manual or a short auto). Confirm stable attitude/alt
      (the SITL-tuned `MC_*RATE_*` gains; re-tune via the System-ID/Autotune module if it
      oscillates).
- [ ] **Camera calibration:** with the aircraft at a **known AGL** over a printed pad,
      compare the projected fix to the true position. Tune the config `cameras:` block —
      **measure the real OV9281 lens HFOV** (the shipped `fov_deg: 99.7` is an
      UNMEASURED placeholder; OV9281 modules ship many lens options) and trim the
      stabilized mount's **residual pitch error** via `depression_deg` (nominal 90°).
      A ±5° pointing error ≈ 0.9 m drift per 10 m AGL. The camera is 1280 px wide —
      the 400 mm marker must decode at the 12 m sweep (verify on real mono frames).
- [ ] **Flow position lock (hover A/B):** hover at 2 m over grass, EKF2_OF_CTRL off
      then on — horizontal drift over 30 s should tighten visibly with flow (this is
      the touchdown-scatter fix on the no-RTK GPS; SITL could not test it).
- [ ] **Descend + land-ON + release:** run a single-pad serve over a printed pad on the
      tether; confirm the rung descent, the id-verified LAND gate (cover the marker → it
      must REFUSE to land and climb), touchdown ON the pad, and the touchdown-gated
      release fire in sequence.

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
