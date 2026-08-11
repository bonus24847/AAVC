# HITL Runbook — AAVC 2026 (real Pixhawk 6X + CM4 + ELRS RC)

Hardware-in-the-Loop: the **real Pixhawk 6X** runs PX4 flying inside a simulator,
the **real CM4 companion** runs the real mission software commanding it, and the
**real ELRS radio** (TX16S + Nomad + DBR4) gives the safety pilot live override —
all with **no motors powered** (HITL spins no motors; it is inherently safe).

It de-risks, on actual flight silicon, the three seams SITL can't touch: the
`orchestrator ↔ real-FC` MAVLink link on the **CM4**, the **real RC link +
safety-pilot switches + failsafes**, and runtime PX4 param application — *before*
the first powered flight (G7).

> **At the bench, work from the print-and-tick checklist: `docs/HITL_CHECKLIST.md`.**
> This runbook is the *why + how it works* reference behind it.

> **Why jMAVSim, not gz.** PX4 HITL is supported **only by jMAVSim and Gazebo
> Classic** — the **gz (Harmonic)** sim this repo uses for SITL **cannot drive a
> real FC** (verified against PX4 docs main + v1.15/1.17). jMAVSim renders **no
> cameras**, so the vision pipeline is fed by `sitl/hitl_synthetic_camera.py` (a
> position-driven ArUco-pad stand-in). Porting the gz world to Gazebo Classic was
> rejected as too costly. Sources:
> [PX4 HITL guide](https://docs.px4.io/main/en/simulation/hitl.html).

## What HITL validates / does NOT validate

| ✅ HITL validates (real 6X + real CM4 + real RC) | ❌ HITL does NOT cover |
|---|---|
| MAVSDK ↔ real-FC link **from the CM4** (arm/takeoff/goto/land/RTL) | Vision **precision** (no real rendering) → gz-SITL + **G6** |
| Runtime PX4 param push (`apply_param_overrides`, `RTL_RETURN_ALT`, geofence) | Real flight **dynamics** (jMAVSim ≠ true inertia/props) → G7 |
| The V1.3 **sequence**: transit → sweep → decode → land-ON → touchdown-release → egress → land+disarm, ×4 sorties | **Physical** egg servo + motors → **G5 bench** |
| The **per-sortie preflight gate** + `--assigned-ids` on the real companion | Inner-loop gain quality on real hardware → G5/G7 |
| **Real RC**: safety-pilot arm / mode-override / kill switch against the sim | Real camera + printed pads on the CM4 → **G6 tethered** |
| Failsafes: geofence RTL, datalink-loss RTL, RC-loss action, `COM_DISARM_LAND=-1` | Optical-flow / rangefinder position lock (no real sensors) → G5/G6 |

HITL sits **between G4 (SITL) and G5 (HW bench)** — an optional, powered-down
de-risk of the software ↔ FC ↔ radio seam. It is not a locked gate.

---

## 0. Prerequisites (one-time)

> **⚠ HITL flies a QUAD; the aircraft is a HEXA (since 2026-07-22).** PX4 ships
> no HIL hexacopter airframe and jMAVSim's model is a quadcopter, so
> `SYS_AUTOSTART` stays **1001** throughout this runbook. That is fine for what
> HITL is for — the CM4↔FC link, the param push, the full V1.3 sequence and the
> RC/failsafe behaviour are all airframe-independent — but HITL tells you
> **nothing** about the EFT X6100's dynamics. Hexa flight behaviour is validated
> in Gazebo SITL (`gz_eft_x6100`) and then on the real aircraft at G6/G7.

### 0a. HITL firmware — the 6X needs a CUSTOM build (stock cannot HITL)

**Stock `fmu-v6x` firmware has NO simulation module**, so `SYS_AUTOSTART=1001`
("HIL Quadcopter X") silently resets to 0 every reboot and there is no HIL
actuator path — verified on the bench 6X (PX4 1.17.0, `pwm_out_sim status` →
*command not found*). The fix is **one Kconfig flag**:

```bash
# In the PX4-Autopilot tree:
cat boards/px4/fmu-v6x/hil.px4board          # = default + the sim module:
#   include px4board of default, then:
#   CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
make px4_fmu-v6x_hil upload                   # build (~98.8% flash) + flash over USB
```

That module provides the HIL actuator driver AND unlocks airframe 1001. The
in-tree `simulator_mavlink` module is NOT needed (it is `PLATFORM_POSIX` = SITL
only); the real FC's own `mavlink` module does the HIL_SENSOR-in /
HIL_ACTUATOR_CONTROLS-out when `SYS_HITL=1`.

> ⚠ **The HIL build must NEVER fly.** It has no real actuator output. Before G7,
> reflash a normal flight firmware (`make px4_fmu-v6x_default upload`) and recal —
> see `docs/FLIGHT.md §"Flash flight firmware"`. HITL is a deliberate one-time
> detour on the flight FC.

### 0b. Host + companion

- **jMAVSim** on the laptop: `~/PX4-Autopilot/Tools/simulation/jmavsim/jmavsim_run.sh`
  (ships with PX4; needs `openjdk` + `ant`). Renders no cameras — that's expected.
- **mavlink-router** on the CM4 (and/or laptop). Not in apt — build from source;
  point the launchers at it with `ROUTERD=<path>` if it's not on `PATH`.
- **The CM4 `.venv`** built (`make install`) — opencv-headless + numpy + pymavlink
  + mavsdk. No internet at the field (rules ban it) → build the venv beforehand.
- **ELRS bound** (TX16S + Nomad + DBR4) and wired to the 6X — see §RC below.
- **arm-none-eabi-gcc** toolchain for the firmware build (10.2.1 verified).

---

## 1. FC params — set via the nsh shell, NOT MAVLink (byte-wise gotcha)

⚠ **This 6X firmware stores `PARAM_SET` byte-wise**: setting float `1.0` with type
INT32 stored `SYS_HITL` as `1065353216` (=`0x3F800000`) — it round-trips on read so
it *looks* fine but is garbage. **Set HITL params through the nsh `param set`**
(parsed by native type), not a MAVLink PARAM_SET / QGC.

`sitl/hitl_param_config.py` does this over the USB serial (`make hitl-params`): it
selects airframe 1001, saves, reboots, then sets the failsafe + RC params and
verifies `SYS_HITL=1` + `HIL` enabled.

| Param | Value | Why |
|---|---|---|
| `SYS_AUTOSTART` | **1001** | Airframe "HIL Quadcopter X" (the jMAVSim HIL frame) — reboot after |
| `SYS_HITL` | **1** | Enable HITL: FC ignores real sensors, takes HIL_SENSOR from the sim |
| `COM_RC_IN_MODE` | **0** | RC Transmitter ON — the safety-pilot DBR4 link is live (see §RC) |
| `NAV_RCL_ACT` | **1** (Hold) | RC-loss action — with a real RC bound you can test it (was 0 pre-RC) |
| `COM_RCL_EXCEPT` | **4** | Offboard bit set: MAVSDK offboard/auto tolerates a momentary RC gap |
| `COM_DISARM_LAND` | **-1** | (orchestrator pushes it at runtime) mid-sortie pad landing stays ARMED |
| `RTL_RETURN_ALT` | **20** | (orchestrator pushes it) any failsafe RTL stays under the 20 m ceiling |

The orchestrator still pushes ALL runtime tuning (`MPC_*`, `MC_*RATE_*`, geofence,
`RTL_RETURN_ALT`, `COM_DISARM_LAND`) over MAVLink exactly as in SITL — nothing to
preload beyond the airframe + HITL toggle + RC block above.

---

## 2. Topologies

### (A) CM4-in-the-loop — RECOMMENDED (validates the real companion)

The mission runs on the **real CM4**; jMAVSim provides physics from the laptop.
This is the point of HITL for *this* aircraft — the flight computer that will fly
G7 is the one under test.

```
   ┌──────── laptop ────────┐              ┌─────────────── on the aircraft ───────────────┐
   │  jMAVSim (physics/HIL)  │              │  Pixhawk 6X  (px4_fmu-v6x_hil, SYS_HITL=1)     │
   │      UDP :4560          │◄──USB HIL────┤    USB  ── HIL_SENSOR / HIL_ACTUATOR ─► laptop │
   │  (optional QGC :14550)  │              │    TELEM2 ── MAVLink ──► CM4 (mission link)    │
   └─────────────────────────┘              │                                                │
                                            │  CM4:  mavlink-router (owns TELEM2)            │
                                            │          → :14540  orchestrator (MAVSDK)       │
                                            │          → :14541  hitl_synthetic_camera       │
                                            │        hitl_synthetic_camera ─► /tmp/aavc_*.png │
                                            │          → vision_worker (unchanged)           │
                                            │  DBR4 (ELRS) ── CRSF ──► 6X TELEM1/RC (see §RC)│
                                            │  TX16S + Nomad ══(2.4+900 RF)══► DBR4          │
                                            └────────────────────────────────────────────────┘
```

- **HIL link** (HIL_SENSOR / HIL_ACTUATOR): 6X **USB → laptop jMAVSim** (jMAVSim
  owns the USB serial). This is the standard, most reliable HIL path.
- **Mission link** (MAVSDK offboard): 6X **TELEM2 → CM4** (`/dev/ttyAMA0` or a USB
  UART). mavlink-router on the CM4 fans it to `:14540` (orchestrator) + `:14541`
  (synthetic camera). PX4 runs the two MAVLink instances (USB HIL + TELEM2 mission)
  independently.
- **RC link**: DBR4 → a 6X UART (§RC). Live in the sim — the safety pilot can arm /
  override / kill against jMAVSim, no motors.

Bring it up with `cm4/launch_hitl.sh` (§4).

### (B) Single-PC bench — fallback (FC-link only, no CM4)

Everything on the laptop; the 6X on USB. Quick to stand up, but does **not** exercise
the CM4 compute. Use it for a first link/arm smoke test.

```
  6X ──USB /dev/ttyACM0──► mavlink-router ──UDP─┬─► :14540 orchestrator (laptop .venv)
  (SYS_HITL=1)                                  ├─► :14541 hitl_synthetic_camera
                                                ├─► :4560  jMAVSim (UDP HIL)
                                                └─► :14550 QGC (optional)
```

Router owns the serial; jMAVSim runs in UDP mode (`-u`, no `-d`). Use
`sitl/hitl_router.conf` and `make run-hitl` on the laptop.

---

## 3. MAVLink routing

Committed template: **`sitl/hitl_router.conf`** (edit `Device`/`Baud` for your link).
The CM4 launcher generates an equivalent config if you don't pass `ROUTER_CONF=`.

```ini
[General]
TcpServerPort = 0

[UartEndpoint fc]        # the 6X link the router OWNS
Device = /dev/ttyAMA0    # CM4 TELEM2 UART  (bench: /dev/ttyACM0 USB)
Baud = 921600

[UdpEndpoint offboard]   # orchestrator  (MAVSDK udpin://0.0.0.0:14540)
Mode = Normal
Address = 127.0.0.1
Port = 14540

[UdpEndpoint hitlcam]    # synthetic-camera telemetry feed
Mode = Normal
Address = 127.0.0.1
Port = 14541

[UdpEndpoint qgc]        # optional ground QGC on the laptop
Mode = Server
Address = 0.0.0.0
Port = 14550
```

In topology (A) jMAVSim owns the **USB** HIL link separately (it is NOT in this
router). In topology (B) add a `[UdpEndpoint sim]` → `127.0.0.1:4560` and run
jMAVSim `-u`.

---

## RC / Radio — TX16S mk2 + RadioMaster Nomad + DBR4 (ExpressLRS dual-band)

The safety pilot's link. **In HITL the RC runs through the real 6X** (DBR4 → 6X →
PX4 → jMAVSim), so you validate the whole radio chain + the arm/mode/kill switches
against the simulator with **no motors** — the single most valuable RC test you can
do before G7.

**The chain:** TX16S (EdgeTX) → **Nomad** dual-band ELRS TX module (module bay) →
RF **2.4 GHz + 900 MHz simultaneously** (Gemini) → **DBR4** dual-band diversity RX
→ **CRSF** → a 6X UART. Dual-band gives the redundancy + the ≥500 m LOS the rules
require; CRSF carries FC telemetry (RSSI, battery, mode, GPS) back to the TX16S.

### Bind + configure (one-time)

1. **Match the ELRS binding phrase** on the Nomad and the DBR4 (ELRS Lua on the
   TX16S, or the ELRS web UI over WiFi). Same phrase = bound; the DBR4 LED goes
   solid. Set a sane **packet rate** (e.g. 150–250 Hz dual-band) and a **Model
   Match** so this airframe only binds its own model.
2. **Telemetry ON** (CRSF is bidirectional) so the TX16S shows FC RSSI/battery/mode.
3. **Failsafe = "No Pulses"** on the DBR4 (so PX4 detects RC loss and runs
   `NAV_RCL_ACT`) — do NOT use "hold last", which hides a real link loss.

### Wire + configure CRSF (verified against PX4 1.15.4 + the Holybro 6X pinout)

**Port choice.** CRSF is a full bidirectional FMU UART (telemetry flows back to the
TX16S) — **NOT the "RC IN" pad** (USART6 → the PX4IO co-processor, which handles
SBUS/PPM/DSM only, no CRSF). Use a TELEM UART:

- **DBR4 CRSF → TELEM1** (UART7, `/dev/ttyS6`; TELEM1 has a separate **1.5 A supply
  rail** — it can power the RX). TELEM2 (UART5, `/dev/ttyS4`) is the CM4 mission link
  in topology (A); TELEM3/GPS2 are alternatives.
- Wire **DBR4 TX → 6X RX**, **DBR4 RX → 6X TX**, **5 V**, **GND**.

**⚠ Firmware — `crsf_rc` is NOT in the stock fmu-v6x build.** Verified in the local
PX4 **1.15.4** tree: `boards/px4/fmu-v6x/default.px4board` enables `rc_input`
(SBUS/PPM) but **not** the CRSF driver, so `RC_CRSF_PRT_CFG` alone does nothing (the
`src/drivers/rc/crsf_rc` driver is present but off). Add it to the **same board
config you customise for HIL** — one line:

```
CONFIG_DRIVERS_RC_CRSF_RC=y      # add to hil.px4board (HITL RC test) AND the G7 flight fw
```

Rebuild + flash, then confirm in nsh: `crsf_rc status` (not "command not found").

**Params (set via nsh — byte-wise gotcha):**

| Param | Value | Why |
|---|---|---|
| `RC_CRSF_PRT_CFG` | **TELEM1** | assign the CRSF receiver to the TELEM1 UART |
| `RC_CRSF_TEL_EN` | **1** | enable CRSF telemetry back to the TX16S |
| `MAV_0_CONFIG` | **0** (Disabled) | un-map TELEM1's default MAVLink so CRSF can own the port |

The baud is **set by the driver** — do NOT touch `SER_TEL1_BAUD`. Power-cycle; the
DBR4 then shows as RC (`rc_status` in nsh, or QGC's radio page).

### Channel map (EdgeTX mixer → CRSF → PX4 `RC_MAP_*`)

| CRSF ch | Stick / switch (TX16S) | PX4 param | Purpose |
|---|---|---|---|
| 1 | Aileron (roll) | `RC_MAP_ROLL=1` | manual roll |
| 2 | Elevator (pitch) | `RC_MAP_PITCH=2` | manual pitch |
| 3 | Throttle | `RC_MAP_THROTTLE=3` | manual throttle |
| 4 | Rudder (yaw) | `RC_MAP_YAW=4` | manual yaw |
| 5 | **SA** 2-pos | `RC_MAP_ARM_SW=5` | arm / disarm |
| 6 | **SB** 3-pos | `RC_MAP_FLTMODE=6` | mode select (see below) |
| 7 | **SE** 2-pos (guarded) | `RC_MAP_KILL_SW=7` | **emergency kill** |
| 8 | SC (opt) | — | spare / RTL trigger |

**Flight-mode slots** on the 3-pos SB (`COM_FLTMODE1/4/6` for low/mid/high):

- **Position** — the safety pilot's manual override (GPS-assisted). Flipping to
  Position at any time **snatches control back from the CM4's Offboard/Auto** — the
  orchestrator sees offboard rejected and aborts cleanly. This is the override you
  test in HITL.
- **Hold** (Loiter) — park in place.
- **Altitude / Stabilized** — degraded manual backup.

The scored mission itself is **Offboard/Auto driven by MAVSDK from the CM4** — the
pilot does not select it; they only ever take control *away*. Keep `COM_RC_IN_MODE=0`
so the RC is live for override, while the CM4 still arms via MAVSDK.

### HITL RC checklist (validate before trusting an unattended run)

- [ ] TX16S shows telemetry (RSSI/battery/mode) — CRSF link is bidirectional + up.
- [ ] Sticks move the jMAVSim quad in Position/Manual mode (channel map correct).
- [ ] **Arm switch** arms/disarms in the sim; **Kill switch** instantly stops it.
- [ ] **Mode override**: with the CM4 mission running (Offboard), flip SB→Position —
      the pilot takes control and the orchestrator logs the offboard loss + aborts.
- [ ] **RC-loss**: power the TX16S off — the 6X runs `NAV_RCL_ACT` (Hold), not a
      "hold last stick" ghost. Power back on → link recovers.

---

## 4. Bring-up

### Topology (A) — CM4-in-the-loop

```bash
# ── laptop ── jMAVSim owns the 6X USB HIL link. Confirm the quad ARMS here first.
cd ~/PX4-Autopilot && ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 921600 -r 250
#   (or, from the repo:  make hitl   — same jMAVSim serial launcher)

# ── CM4 ── one-time param config over the SAME 6X (do this with jMAVSim stopped,
#           or over TELEM2; it drops into nsh). Then start the onboard HITL stack.
make hitl-params SERIAL=/dev/ttyACM0            # airframe 1001 + SYS_HITL=1 + RC block, via nsh

# ── CM4 ── known ArUco pads for the synthetic camera (HITL has no organiser field):
make spawn-targets                              # writes /tmp/aavc_targets.json (positions + ids)

# ── CM4 ── router (owns TELEM2) + synthetic camera + the real mission, one command:
SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh     # dashboard up (bench); GO per sortie
#   headless committee stand-in:
#   HEADLESS=1 ASSIGNED_IDS="3,1,4,6" SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh
```

### Topology (B) — single-PC bench

```bash
mavlink-routerd -c sitl/hitl_router.conf                          # edit Device=/dev/ttyACM0 first
cd ~/PX4-Autopilot && ./Tools/simulation/jmavsim/jmavsim_run.sh -q -u -r 250   # UDP HIL → :4560
make spawn-targets                                                # /tmp/aavc_targets.json
make hitl-camera                                                  # synthetic frames from :14541
make run-hitl TRUTH=/tmp/aavc_targets.json                       # mission vs the real 6X
```

> `/tmp/aavc_targets.json` tells the synthetic camera **where to draw** the pads and
> feeds the post-flight truth audit — **never** the mission planner (blind search is
> preserved; the mission decodes the synthetic frames exactly as it would gz frames).

---

## 5. Incremental validation — bring it up in rungs, each gates the next

1. **Link** — the mission connects, logs `cached home MSL altitude`, telemetry
   streams from the CM4. (Ctrl-C before takeoff.)
2. **Params** — orchestrator log shows `applied N/N PX4 tuning params` +
   `home MSL re-cached after arming` per arm (the per-arm frame refresh).
3. **Arm + takeoff/land** — the quad lifts/lands in jMAVSim; `COM_DISARM_LAND=-1`
   keeps it armed on a pad landing.
4. **Transit** — P1→P2→P3 at 20 m; watch `TRANSIT_PASS` audits in order.
5. **Sweep + decode** — the synthetic camera draws a pad as the quad passes; the
   tracker confirms it by **decoded id** (`cluster_identified marker=…`). Undecoded
   candidates get a floor revisit.
6. **Serve** — align rungs → **land ON the pad** → touchdown-gated `drop payload 0`
   (servo is command-only in HITL) → climb-out. Confirm the id-verified LAND gate
   REFUSES to land if the assigned id was never decoded (climb + defer).
7. **Multi-sortie** — 4 sorties inside the window; egress transit; land + **disarm**
   at L&R between sorties; the per-sortie gate holds + releases.
8. **RC override + failsafes** — the §RC checklist: mode-override, kill, RC-loss,
   geofence RTL, datalink-loss RTL (pull the CM4 link → FC RTLs at `RTL_RETURN_ALT`).
9. **verify_flight** — `tools/verify_flight.py runs/<id>/audit.jsonl` PASSES on the
   HITL audit exactly as on SITL (same behavioural contract).

---

## 6. Safety

- HITL spins **no motors** (FC outputs go to the sim). Keep props **OFF** anyway.
- The **DBR4 RC gives an independent kill** (SE) at all times — the ultimate override
  even in HITL. Verify the kill switch works in the sim before every session.
- The geofence + datalink-loss RTL + `RTL_RETURN_ALT=20` are uploaded by the
  orchestrator; confirm they're active (QGC or the dashboard) before an unattended run.
- **Do not `systemctl enable`** any auto-start unit — HEADLESS auto-GO on boot would
  auto-fly. Start HITL manually, deliberately.

---

## 7. Known limitations (carried into G5 / G6 / G7)

- **Vision is a stand-in.** The synthetic pad is drawn **centred** within the lock
  radius (rotated for decode-invariance, but centred) → it validates the
  *sequence/timing + the id gate*, NOT the lateral-align magnitude or projection
  accuracy. Real-camera vision is the **G6 tethered** gate.
- **Dynamics are jMAVSim's**, not the real airframe — the SITL-tuned gains are a
  starting point; real System-ID/tune is **G5/G7**.
- **The egg release is command-only** in HITL; the physical AUX→servo channel + PWM
  band is a **G5 bench** item (`ConnectionConfig.drop_servo_channel`/`drop_servo_pwm_*`,
  and `drop_fallback_endpoint` if `set_actuator` is rejected on the real link).
- **Optical flow + rangefinder** can't be validated (no real sensors) — that
  position lock is a **G5/G6** bench + hover item on the real bird.

## Code touchpoints

- `docs/HITL.md` — this runbook.
- `docs/HITL_CHECKLIST.md` — the bench print-and-tick sheet (do + verify + record).
- `sitl/hitl_param_config.py` — nsh param setter (airframe 1001 + SYS_HITL + RC), `make hitl-params`.
- `sitl/hitl_router.conf` — committed mavlink-router template.
- `sitl/launch_hitl.sh` — jMAVSim HIL launcher (laptop side).
- `cm4/launch_hitl.sh` — CM4-in-the-loop launcher (router + synthetic camera + mission).
- `sitl/hitl_synthetic_camera.py` — position-driven ArUco-pad nadir-camera stand-in.
- `mavlink_adapter/commands.py` — `ConnectionConfig` (`--connect`, drop servo/fallback).
- `Makefile` — `hitl`, `hitl-params`, `hitl-camera`, `run-hitl`.
