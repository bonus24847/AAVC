# Flight-readiness / commissioning log — EFT X6100 hexacopter

Bring-up record for the AAVC 2026 aircraft (EFT X6100 hex · Pixhawk 6X FMU ·
onboard Raspberry Pi CM4 over internal TELEM2 `/dev/serial0` @921600). This is the
history of *what was fixed to make it fly*, so it can be reproduced. Most of these
changes live on the FMU (params/calibration) or the CM4 (systemd), not in code, so
they are recorded here on purpose.

---

## ✅ Milestone — first clean autonomous OFFBOARD hover (2026-07-30)

**Props ON, real flight, passed cleanly.** RC-triggered autonomous hover mission:
pilot arms on the ground → flips the RC mode switch to OFFBOARD → the CM4
`flight_runner` streams setpoints → the FMU auto-takes-off to 5 m, holds ~60 s, and
auto-lands. Symmetric spin-up, stable hover, clean landing. The pilot kept a finger
on KILL (ch8) and POSCTL as the abort path throughout.

This was gated behind the commissioning fixes below — before them the vertical EKF
drifted, the battery read wrong, and OFFBOARD would cut on the 2nd attempt.

---

## What was fixed to get here

| Fix | Where it lives | Detail |
|-----|----------------|--------|
| **EKF vertical drift** | FMU (accel + level calibration) | Stationary Z estimate was wandering ~1.17 m/14 s → after re-cal **0.021 m/14 s**. Was causing position-OFFBOARD loss / drops. |
| **Battery reader** | Hardware (PM03D power module replaced) | Faulty module read **20.16 V / 0 %**; replacement reads **24.62 V / 4.10 V-cell / 100 %** for a charged 6S. Do **not** calibrate around a bad reader — replace it. |
| **Low-battery failsafe** | FMU param `COM_LOW_BAT_ACT = 3` | Return at ~25 %, Land at ~15 %. Set over MAVLink as INT32 (bit-reinterpret, see gotchas). |
| **Auto-recover the setpoint daemon** | CM4 systemd drop-in `Restart=always` | `flight_runner` exits 0 on pilot override; `Restart=on-failure` did **not** relaunch → OFFBOARD wouldn't re-engage on the 2nd try. Now `Restart=always` + `RestartSec=3`, **permanent** (survives reboot). |
| **Level / attitude** | FMU (Level Horizon on a truly level surface) | roll **+0.02°**, pitch **+0.03°**. An earlier 1.35° pitch residual was just the airframe sitting on a sloped surface — no real bias. |
| **Hover thrust** | `config/real.yaml` `hover_thrust: 0.50` | Measured from SD-card ulog `hover_thrust_estimate` over 3 hand-flown hovers (0.494 / 0.497 / 0.502). Supersedes the old 0.56 TWR-3.2 estimate (~0.06 high). |

### Props-off vs props-on — the motor-asymmetry question (closed)
On the bench with **props off**, flipping to OFFBOARD made the motors spin
*unequally* (looked like it might flip). This is **not** a flip risk — it is the
**position-controller integrator winding up** on the ground: props-off the craft
can't climb, so the controller commands growing corrective tilt. With level now at
0.02°/0.03° there is **no attitude bias**, confirming the asymmetry is a ground
artifact. Props-off can prove the *command chain* but **cannot** validate takeoff
symmetry — only a real (props-on) takeoff can. It did: the maiden flight spun up
symmetric.

---

## Flight config actually flown (`config/real.yaml`, key params)

The Sys_ID flight code is not in this repo; the params that flew are captured here so
this record is self-contained.

```yaml
connection:                     # on the CM4 this is sed-patched to /dev/serial0 @921600
  url: "/dev/ttyACM0"           #   (internal TELEM2 — NOT USB on the onboard Pi)
  sitl_disable_power_check: false   # MUST stay false on real hardware

mission:
  type: hover
  trigger: rc                   # pilot arms + flips OFFBOARD; then fully autonomous
  hover_hold_s: 60.0

flight:
  takeoff_alt_m: 5.0
  hover_thrust: 0.50            # measured (see table)
  setpoint_rate_hz: 100.0

# (chirp / sysid / pid_design sections unchanged — not exercised by the hover mission)
```

Behaviour: `flight_runner` streams a HOLD setpoint and **waits**; on arm + OFFBOARD it
auto-takes-off to 5 m → hovers 60 s → auto-lands. Flipping the switch away or KILL
hands control straight back.

---

## Operating gotchas (learned the hard way — read before the next flight)

1. **Arm order B — arm FIRST, then OFFBOARD.** On this FMU you **cannot** arm via the
   ch7 RC switch while already in OFFBOARD (silently rejected). Working sequence:
   **flip to a manual mode (Stabilized/POSCTL) → ARM → flip to OFFBOARD.**
2. **"OFFBOARD red / won't engage" ≈ `flight_runner` is not streaming.** OFFBOARD
   setpoints come **only** from the CM4 `flight_runner` over TELEM2. A GCS (QGC or the
   web GCS, on any laptop) does **not** stream OFFBOARD — it only views telemetry and
   sends arm/mode. First check: `systemctl is-active sysid-flight` and the journal for
   `heartbeat ok`. The daemon must already be streaming *before* you flip OFFBOARD.
3. **The FMU reboots after a calibration.** Re-check the link is back before testing.
4. **One reader on `/dev/serial0`.** Two processes on the serial port corrupt the
   MAVLink stream (garbled reads can look like spurious "ARMED+OFFBOARD"). Stop the
   `sysid-flight` service fully (past `RestartSec`) before opening any second reader.
5. **`pkill/pgrep` self-match.** A plain `pkill -f flight_runner.py` matches the SSH
   command's own cmdline → false counts / exit 255 killing its own shell. Use the
   bracket trick: `pkill -9 -f "[f]light_runner.py"`, and launch unconditionally after.
6. **`sudo` on the CM4 needs a password** (`sudo -n` fails): `echo 1 | sudo -S <cmd>`.
7. **INT32 params over MAVLink** need bit-reinterpret, not a numeric value.
   SET: `struct.unpack('<f', struct.pack('<i', v))[0]` with type `INT32`.
   READ: `struct.unpack('<i', struct.pack('<f', param_value))[0]`.
8. **`disarm` after every landing** — with `Restart=always` the daemon is always ready.

---

## Status & next

- **Flight-ready** for the autonomous hover mission — proven end-to-end 2026-07-30.
- **Real rate-loop SysID chirp** is still pending a safe field (~10–15 m radius,
  one axis, tiny amplitude, pilot on the RC). `hover_thrust` is already 0.50; the
  chirp uses attitude setpoints. **Never** carry SITL gains to the real drone —
  re-identify on real hardware.
- **TFmini Plus LiDAR (UART)** — optional robustness upgrade for AGL / precise
  competition landing.
