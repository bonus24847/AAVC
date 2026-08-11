# Bench HITL Checklist — AAVC 2026 (6X + CM4 + TX16S mk2/Nomad/DBR4)

> **⚠ HITL is a quad proxy.** jMAVSim + `SYS_AUTOSTART=1001` model a
> quadcopter; the real aircraft is the EFT X6100 hexa. HITL validates the
> link, params, sequence and RC failsafes — never hexa dynamics.


A print-and-tick bench sheet. Work top to bottom; **each rung gates the next** — do
not skip ahead. Full explanations + wiring + params: **`docs/HITL.md`** (this sheet
is the *do + verify + record* companion). Record every result in §Sign-off.

> **HITL spins NO motors** (the FC's outputs go to jMAVSim, not the ESCs). Even so:
> **PROPS OFF**, battery out or on a current-limited bench PSU, and the **DBR4 kill
> switch (SE) reachable at all times**. If anything is wrong, hit kill / Ctrl-C the
> mission / power the 6X down — in that order of speed.

Session date: ____________  Operator: ____________  6X id: ____________
Firmware build hash (`ver all`): ____________________

---

## A. One-time prep (skip if already done — tick to confirm still true)

- [ ] **HIL firmware flashed.** `make px4_fmu-v6x_hil upload` in the PX4 tree.
      nsh `pwm_out_sim status` responds (NOT "command not found").
- [ ] **CRSF driver in the build** (only if using RC): `hil.px4board` has
      `CONFIG_DRIVERS_RC_CRSF_RC=y`; nsh `crsf_rc status` responds.
- [ ] **CM4 `.venv` built** (`make install`); `env -u PYTHONPATH .venv/bin/python -m pytest -q` → 152 passed.
- [ ] **mavlink-router** installed / built; `ROUTERD=<path>` known if not on `PATH`.
- [ ] **jMAVSim** runs on the laptop (`openjdk` + `ant` present).
- [ ] **ELRS bound**: TX16S + Nomad ↔ DBR4 (matching binding phrase; DBR4 LED solid);
      DBR4 failsafe = **"No Pulses"**; telemetry ON.

## B. Wiring + power (every session)

- [ ] **DBR4 CRSF → 6X TELEM1** (UART7 / `/dev/ttyS6`): DBR4 TX→6X RX, RX→6X TX, 5 V, GND.
- [ ] **CM4 mission link → 6X TELEM2** (UART5 / `/dev/ttyS4`) → CM4 UART (`/dev/ttyAMA0`).
- [ ] **6X USB → laptop** (the jMAVSim HIL link).
- [ ] Power the 6X; DBR4 LED solid (bound); CM4 booted.
- [ ] `ls /dev/ttyACM* /dev/ttyAMA*` — note the 6X-USB dev (laptop) + the CM4 UART dev.

Laptop 6X-USB dev: ____________   CM4 mission dev: ____________

---

## C. FC params (once per firmware flash)

- [ ] **Core HITL params** — `make hitl-params SERIAL=/dev/ttyACM0` (jMAVSim stopped).
      Or `--dry-run` then paste into nsh by hand if the shell I/O misbehaves.
- [ ] Verify in nsh (`param show <NAME>`): `SYS_AUTOSTART` = **1001**, `SYS_HITL` = **1**,
      `COM_RC_IN_MODE` = **0**, `NAV_RCL_ACT` = **1**, `COM_RCL_EXCEPT` = **4**.
- [ ] **CRSF params** (if using RC, via nsh): `RC_CRSF_PRT_CFG` = **TELEM1**,
      `RC_CRSF_TEL_EN` = **1**, `MAV_0_CONFIG` = **0**. Power-cycle. `rc_status` shows RC.

> ⚠ Set these via **nsh `param set`**, never a MAVLink PARAM_SET/QGC (byte-wise bug).

---

## D. jMAVSim HIL link (laptop) — the physics owns the USB

```bash
cd ~/PX4-Autopilot && ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 921600 -r 250
```
- [ ] jMAVSim window shows the quad; PX4 boots (no HIL_SENSOR errors).
- [ ] **Bench-arm test IN jMAVSim** (QGC or an RC arm) — the quad arms + lifts, no
      motor noise from the bench. Disarm. *(This proves the HIL link before the mission.)*

## E. RC bench check (in jMAVSim — the safety-pilot chain) — only if RC wired

- [ ] TX16S telemetry live (RSSI / battery / mode) — CRSF is bidirectional + up.
- [ ] Sticks move the jMAVSim quad in **Position/Manual** (channel map: AETR = 1–4).
- [ ] **Arm switch (SA)** arms/disarms in the sim.
- [ ] **Kill switch (SE)** instantly stops the sim quad. **← confirm this works first.**
- [ ] **Mode switch (SB)** selects Position / Hold / Altitude as expected.

---

## F. Mission targets + onboard stack (CM4)

```bash
make spawn-targets                     # writes /tmp/aavc_targets.json (ArUco pads + ids)
# note the assignable ids printed:  ____________________
SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh    # router + synthetic cam + orchestrator
```
- [ ] `spawn-targets` printed the pad ids (record them above — you enter these at GO).
- [ ] Router log: `mavlink-router: /dev/ttyAMA0@921600 -> :14540 … :14541 …`.
- [ ] Synthetic camera writing frames: `/tmp/aavc_nadir.png` age < 2 s.
- [ ] Dashboard reachable: `http://127.0.0.1:8765` (bench) — map + camera + GO panel.

---

## G. Incremental validation — each rung must PASS before the next

| # | Rung | Do | PASS when | ✓ / ✗ |
|---|---|---|---|---|
| 1 | **Link** | stack up, before GO | orch logs `cached home MSL altitude`; telemetry streams | |
| 2 | **Params push** | at first arm | `applied N/N PX4 tuning params`; `home MSL re-cached after arming` per arm | |
| 3 | **Arm + takeoff/land** | GO sortie 1 | quad lifts in jMAVSim; `COM_DISARM_LAND=-1` keeps it armed on a pad | |
| 4 | **Transit** | outbound | `TRANSIT_PASS P1→P2→P3` in order, ~20 m | |
| 5 | **Sweep + decode** | over a pad | `cluster_identified marker=<id>`; undecoded → floor revisit | |
| 6 | **id gate** | wrong/no decode | LAND **refused** (climb + defer) when the assigned id was never decoded | |
| 7 | **Serve (land-ON)** | on the pad | align rungs → land ON → `drop payload 0` **after** touchdown → climb-out | |
| 8 | **Egress + L&R** | inbound | `TRANSIT_PASS P3→P2→P1`; lands at L&R; **disarms** for resupply | |
| 9 | **Multi-sortie** | GO ×4 | 4 sorties inside the window; per-sortie gate holds + releases | |
| 10 | **RC override** | mid-Offboard | flip SB→Position → pilot takes control; orch logs offboard loss + aborts cleanly | |
| 11 | **Kill** | mid-flight | SE kill → sim quad stops instantly | |
| 12 | **RC-loss** | TX16S off | 6X runs `NAV_RCL_ACT` (Hold); power back on → recovers | |
| 13 | **Geofence RTL** | fly at the fence | FC RTLs back inside at `RTL_RETURN_ALT=20` | |
| 14 | **Datalink-loss RTL** | pull the CM4 link | FC RTLs (companion-independent failsafe) at 20 m | |

## H. Post-run verification

- [ ] `env -u PYTHONPATH .venv/bin/python tools/verify_flight.py runs/<mission_id>/audit.jsonl --truth /tmp/aavc_targets.json`
- [ ] **verify_flight → PASS** (transient-ceiling WARN is OK; no FAIL). exit 0.
- [ ] Delivered = sorties flown; ids correct; window < 20 min; disarm between sorties.

## I. Teardown

- [ ] Ctrl-C `cm4/launch_hitl.sh` (mission + router + synthetic cam stop together).
- [ ] Ctrl-C jMAVSim on the laptop.
- [ ] **Before any powered flight (G7): reflash flight firmware** (`make px4_fmu-v6x_default upload`)
      + recal — the HIL build has NO real actuator output. See `docs/FLIGHT.md`.

---

## Sign-off (record results)

| Rung | Result (✓/✗ + note) | Rung | Result |
|---|---|---|---|
| A prep | | G7 serve/land-ON | |
| B wiring | | G8 egress + L&R | |
| C params | | G9 multi-sortie | |
| D HIL arm | | G10 RC override | |
| E RC bench | | G11 kill | |
| F stack | | G12 RC-loss | |
| G1 link | | G13 geofence RTL | |
| G2 params | | G14 datalink RTL | |
| G3 arm/takeoff | | H verify_flight | |
| G4 transit | | | |
| G5 sweep/decode | | Overall PASS? | |
| G6 id gate | | Blockers for G5: | |

Notes / anomalies / follow-ups:
________________________________________________________________
________________________________________________________________

---

## Quick reference

| Thing | Value |
|---|---|
| DBR4 CRSF port | 6X **TELEM1** (UART7, `/dev/ttyS6`) — `RC_CRSF_PRT_CFG=TELEM1`, `RC_CRSF_TEL_EN=1`, `MAV_0_CONFIG=0` |
| CM4 mission link | 6X **TELEM2** (UART5, `/dev/ttyS4`) → CM4 `/dev/ttyAMA0` @921600 |
| jMAVSim HIL link | 6X **USB** → laptop (`/dev/ttyACM0`) |
| Params | `make hitl-params SERIAL=…` — SYS_HITL=1, 1001, RC block (via nsh) |
| Stack | `SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh` |
| Headless | `HEADLESS=1 ASSIGNED_IDS="3,1,4,6" SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh` |
| Verify | `tools/verify_flight.py runs/<id>/audit.jsonl --truth /tmp/aavc_targets.json` |

## Common bench issues → fix

| Symptom | Likely cause → fix |
|---|---|
| `SYS_AUTOSTART` resets to 0 after reboot | stock fw (no sim module) → flash `px4_fmu-v6x_hil` |
| `SYS_HITL` reads a huge int (1065353216) | set via MAVLink PARAM_SET → use nsh `param set` instead |
| RC not detected / `crsf_rc: command not found` | CRSF driver not in build → add `CONFIG_DRIVERS_RC_CRSF_RC=y`, rebuild |
| `make run-hitl`/mission hangs at "connecting…" | mavlink-router not owning the link, or wrong `--connect` port |
| No pad ever decodes in the sweep | `/tmp/aavc_targets.json` missing/stale → re-run `make spawn-targets` |
| synthetic frames stale | camera not fed telemetry on :14541 → check the router `hitlcam` endpoint |
| quad won't arm | preflight/arming health (EKF/GPS in the sim) — check QGC/dashboard messages |
| verify_flight FAIL "no ground truth" | pass `--truth /tmp/aavc_targets.json` |
