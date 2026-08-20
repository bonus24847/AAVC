# FC parameters — the bugs and their guards

Format: **symptom → mechanism → fix → what catches it now**.

## GF_ACTION was Hold(2), not Return(3), for the project's whole life
`set_geofence_action_rtl()` wrote **2** while its own name/comments/errors said
RTL. PX4 enum: 0 None · 1 Warning · **2 Hold** · **3 Return** · 4 Terminate ·
5 Land — so the FC-level breach response was "loiter at the breach point,
outside the fence". Undetectable mode-side: the mission flies in HOLD
(DO_REPOSITION → AUTO_LOITER), so a Hold failsafe looks like normal flight.
**Fix:** 3, verified live 2026-08-16; allowed set is {3, 5}, NEVER 2.
**Guard:** `tools/preflight_params.py` PINNED (`make preflight`); readback at
`commands.py::set_geofence_action_rtl`. **Meta-lesson:** the readback gate
passed the whole time — readback proves a value was STORED, never that it
means what the caller thinks.

## PWM_MAIN_FUNC1..6 read 0 — motor map wiped, cause never found
2026-08-16: all six = 0 → motors unassigned → unflyable. Restored 101..106 on
2026-08-17. Root cause UNKNOWN → treat as recurrable. Suspect not cleared:
QGC's Airframe tab re-pick reloads frame defaults over the Actuators map.
**Guard:** BOARD block of `make preflight` before EVERY field day (exit 2 =
stop). The SD param backup is the only restore net — see below.

## PWM_AUX_FUNC1..4 at 402/405/409/410 — egg latches on the RC sticks
RC-passthrough values wired two egg latches to roll/yaw sticks (eggs would
release in a bank). Now 301..304. **Guard:** BOARD block; any 4xx = fatal.

## INT32 params pushed as float never reach the board (TWO shipped)
PX4 rejects a float write to an INT32 param outright; MAVSDK reports TIMEOUT,
which reads like a link hiccup, and `param_audit.py` used to file it under
"needs a REBOOT" — the true failure wearing an expected label.
Shipped twice: `EKF2_HGT_REF` (its pin NEVER landed, whole life) and
`MAV_1_FORWARD` (same dict, one entry down — the radio beacon's forwarding
pin; a param reset would have silenced the beacon invisibly). Both fixed
2026-08-20 by adding to `_INT_PARAMS` (`mavlink_adapter/commands.py`).
**Guard:** `make type-audit` (`tools/px4_type_audit.py`) — static, resolves
every pushed param's declared type from the PX4 worktree, both directions.
Run it whenever a pushed param is added or the worktree moves.

## MAVSDK getters are type-strict — healthy board reported "unflyable"
`get_param_float` on an INT32 fails and vice versa; asking one way made
preflight report the whole motor map as (ไม่ตอบ). **Fix:** `_read()` tries
both getters per name (`preflight_params.py`).

## "applied 0/24" still flies — on PX4 defaults
`_apply_params` is best-effort by design (one unknown key must not abort a
sortie). A stale mavsdk_server holding the ports once failed EVERY param RPC:
the mission then flies PX4 defaults — RTL at 60 m (3× the ceiling),
`MPC_Z_V_AUTO_DN` 1.5 m/s onto the pad. **Guard:**
`commands.py::verify_envelope_pins` reads back the envelope five after the
push; `make preflight STRICT=1` after staging exits 4 if PINNED didn't land.

## Stored ≠ in force (`_STORED_NOT_PROVEN`)
`MAV_1_FORWARD` (module start arg) and `EKF2_HGT_REF` (EKF latches its height
ref: `checkHeightSensorRefFallback` bails once set) store + read back
immediately while the OLD value keeps running — readback there is honest
about storage only. The reboot_required metadata flag is NOT the test
(`BAT1_CAPACITY` carries it and applies live); "does the owning module
re-read it" is. **Guard:** the (เก็บค่าแล้ว — ต้องรีบูต) annotation in
preflight; plan an FC reboot after changing either.

## AUTO reads DIFFERENT params than manual — bit twice
`MPC_Z_V_AUTO_DN` (AUTO descent) vs `MPC_Z_VEL_MAX_DN` (manual/offboard);
`MPC_JERK_AUTO` vs `MPC_JERK_MAX`; and the reversed pair `MPC_ACC_HOR` (AUTO)
vs `MPC_ACC_HOR_MAX` (manual). A knob with "no effect" → check the AUTO twin
first. STILL OPEN: `tactical_align`'s rung ladder steps `MPC_Z_VEL_MAX_DN`,
i.e. not the AUTO descent it meant to shape — the effective pad descent is
the pinned `MPC_Z_V_AUTO_DN=0.4`; do NOT unpin it (PX4 default 1.5 is ~4×
faster onto the pad than anything validated, and SITL can't catch it because
SITL has 0.4 persisted).

## SYS_HITL=1 = no real actuator output
The HITL firmware never drives motors; a board left flagged is unflyable
silently. **Guard:** `SYS_HITL=0` joined the BOARD block 2026-08-20.

## Crash dump on SD blocks arming
"Preflight fail: Crash dump present on SD" — PX4 refuses to arm with a
`fault_*.log` on the card (seen+cleared 2026-08-18; that one was an Idle-Task
hardfault on a bench boot, archived in docs/evidence/). **Procedure: READ IT
BEFORE DELETING** (it is the only record), archive to docs/evidence/, then
delete `/fs/microsd/fault_*.log` over MAVFTP and reboot; re-verify BOARD after.
**Guard:** `make fence-probe` flags it.

## /fs/mtd_params BSON corruption — the SD backup is the only net
This board has TWICE failed to import FRAM params and fallen back to
`/fs/microsd/parameters_backup.bson`. The recovery restores ONLY what that
file holds — an old backup once restored `SYS_AUTOSTART=4001` (a QUAD): that
is what "recovered but wrong aircraft" looks like. Checked CURRENT 2026-08-18
(129 params incl. the motor map). **Guard:** `make fence-probe` flags
`param_import_fail.txt`; after ANY recovery event run `make preflight` before
believing the board.

## Raw pymavlink 2.4.49 vs PX4 1.17: silent drops
Its instanced-message bookkeeping raises TypeError and silently drops what
arrived with it — param reads and MAVFTP die mid-way. **Rule:** all FC I/O
through MAVSDK; any pymavlink survivor must guard `recv_match` (the beacon is
send-only STATUSTEXT, acceptable).
