---
name: PX4MASTER
description: Field-proven procedures + bug catalog + check tools for the AAVC EFT X6100 (PX4 6X + CM4). Use when preparing or flying a field day, after any FC reboot or power-cycle, before staging a mission, or when debugging PX4 params, battery gauge, GPS/height, MAVLink link/routing, or SITL quirks on this aircraft. Trigger on /PX4MASTER and proactively whenever those situations appear.
---

# PX4MASTER — the AAVC bird's operating truth

One aircraft, many sharp edges, all of them already cut somebody. This skill
is the record: every documented bug/trap with its detection, and the
procedures that a real field night (2026-08-20, 3 flights, 3 live bugs)
validated end-to-end.

**Aircraft state one-liner (2026-08-20):** EFT X6100 hexa · PX4 1.17 on 6X ·
CM4 companion (WiFi AP `AAVC-DRONE` @10.42.0.1 — the AP FLIES WITH THE
AIRCRAFT) · NOMAD radio = the flight telemetry link · PM02D powers the
FC/avionics ONLY → voltage-only battery gauge (`BAT1_CAPACITY=-1`, never
positive) · 17000 mAh semi-solid pack (endpoints 4.18/3.77) · baro height
reference at the practice site (`EKF2_HGT_REF=0`; the KMITL comp config
deliberately keeps 1) · TFmini lidar aids below 7 m.

## Read the right reference

| Topic | File |
|---|---|
| FC params: motor map, GF_ACTION, INT32 pushes, readback lies, SD/crash dump | `references/fc-params.md` |
| Battery/power: gauge branches, endpoints, sag, PM02D trap, THE energy answer | `references/power-battery.md` |
| Height & GPS: frame walks, baro-vs-GPS ref, per-arm drift, TFmini | `references/height-gps.md` |
| Link & routing: wedges, pymavlink, WiFi-AP-flies-away, port rules, pkill trap | `references/link-routing.md` |
| SITL-only traps | `references/sitl.md` |
| Field operations, procedures, verifier discipline, follow-ups | `references/ops-field.md` |
| Community watchlist: known 1.17/hardware issues we likely carry + ranked bench actions | `references/community-watchlist.md` |

## The field-day sequence (validated live 2026-08-20)

1. **Deploy + prove it**: `bash cm4/deploy.sh drone@10.42.0.1` then
   `bash cm4/deploy.sh drone@10.42.0.1 --check` → must print MD5 MATCH.
2. **`make field-check`** (or on the CM4: `make preflight` → `make fence-probe`
   → `make alt-watch`, each with `CONNECT=…`). Stops at the first failure:
   - `preflight` — BOARD params (motor map 101..106, AUX 301..304,
     BAT1_*, `SYS_HITL=0`…). Exit 2 = DO NOT FLY.
   - `fence-probe` — FC mission/dataman aliveness (the wedge that ate three
     stagings on 2026-08-20) + SD crash-dump/param-import flags.
   - `alt-watch` — altitude frame stability; MANDATORY after any FC
     reboot/power-cycle. ⚠ both probes bind the orchestrator's udpin port —
     they exit by themselves; never leave one running into staging.
3. **Stage** (🚀). Watch the log for `cached home MSL altitude: X` — X must
   match the GCS ⬆ MSL readout within ~1 m. Then wait for `RC-GO — staged`.
   Not staged? `ssh … 'tail -20 ~/mission/runs/aavc_delivery_mission/orchestrator.log'`
   → look for `unmet critical checks: <link|armable|ekf|home>`.
4. **RC**: ARM (POSCTL held) → flip OFFBOARD within a few seconds → the
   aircraft launches itself. Throttle stick to centre once airborne.
   **Takeover = flip POSCTL, any time.** Landing+disarm ends the mission.
5. **Expect the WiFi to drop at takeoff** — the AP is on the airframe. The
   radio console is the flight link; ssh monitors resume after landing.
6. **Post-flight**: `make verify RUN=runs/<id>/audit.jsonl` (on the CM4 copy),
   log the battery figures (see power-battery.md ground-truth table), UNPLUG
   the pack.

## After any FC reboot / battery re-plug

GPS altitude walks for minutes after a cold start (measured: 12.7→36.6 m on
the parked field). Baro ref removed the in-flight symptom, but the sequence
stands: wait for `make alt-watch` to print STABLE before staging, and never
trust a home-MSL cache taken while the frame was still moving.

## Session hygiene on PX4 v1.17.0 (bugs fixed AFTER our build — we carry them)

- **No USB plug-ins / new MAVLink instances once a session is live** — every
  `Mavlink::start()` re-initializes the shared command-ACK semaphore
  (PX4#27593): plugging QGC-over-USB mid-session can corrupt command/ACK
  tracking for the flying link. Weird ACK timeouts after any link change →
  reboot the FC before GO.
- **Geofence uploads on the GROUND only** — the navigator SKIPS fence
  violation checks while a fence upload transaction is in progress.
- **On-board file reads are untrusted** until v1.17.1 (silent SDMMC READ
  corruption on our STM32H7, NuttX#389): pull the SD card physically to copy
  ULogs, or download twice + hash-compare; restore params from the
  laptop-side `parameters_backup.bson` copy, never from an SD read.

## หลักที่สนามสอน (field-learned principles, 2026-08-20)

1. **GPS อย่างเดียวไม่พอสำหรับงาน precision** — jitter/walk ของ GPS (แนวดิ่ง
   เดิน 24 m ในเย็นเดียว) ทำให้ทั้ง position และความสูงเชื่อไม่ได้ใน margin
   ระดับเมตร; การแยกหน้าที่ **baro ถือกรอบความสูง + lidar เก็บเมตรท้าย +
   GPS เอาแค่แนวราบ + vision ตัดสินเมตรสุดท้าย** ให้ผลดีกว่าที่วัดได้จริง
   (transit MISS 40 m → PASS 1.4-2.0 m ในคืนเดียว)
2. **ปัญหาบางชนิดโผล่เฉพาะการบินจริง** — sim ไม่มี GPS jitter จริง, bench
   เครื่องไม่ขยับ, code review เห็นแต่โค้ดที่ "ถูกทุกบรรทัดบนสมมติฐานผิด"
   → ทุกไฟลต์จริงคือเครื่องมือค้นหา unknown: เปิด ULog ทุกครั้ง (discovery
   loop ใน ops-field.md)
3. **Readback พิสูจน์แค่ว่าค่าถูกเก็บ ไม่ใช่ว่าระบบใช้ค่านั้น** — GF_ACTION,
   EKF2_HGT_REF, MAV_1_FORWARD ล้วนเคยหลอกด้วยวิธีเดียวกัน; คำถามที่ถูกคือ
   "โมดูลเจ้าของอ่านค่านี้ใหม่เมื่อไหร่"
4. **Fail-closed ต้องปล่อยให้ทำงาน** — ระบบปฏิเสธบิน (fence ไม่ยืนยัน /
   watchdog RTH) สองครั้งในคืนเดียวคือระบบทำงานถูกบน input ที่มันเห็น
   อย่าสู้กับมัน ให้ไปแก้ input (รีบูต FC, รอ frame นิ่ง, ชาร์จแบต)
5. **ความรู้จาก community มาก่อนบั๊กจะเจอเรา** — ก่อน debug อาการใหม่จาก
   ศูนย์ ให้ค้น discuss.px4.io + GitHub ก่อนเสมอ (watchlist =
   `references/community-watchlist.md`)

## Before the next scored flight (unverified-risk shortlist)

1. **IN-FLIGHT IMAGE BLUR (re-scoped 2026-08-21, still #1)**: the static
   camera is PROVEN GOOD — bench walk test with the real 38 cm marker
   decoded continuously 1.9→14 m and intermittently to ~20 m with NO lens
   adjustment (walk_test_decode_2026-08-21.log). Yet 402 in-FLIGHT frames
   decoded zero — so the blur is dynamic: prime suspects are the measured
   ~60 Hz per-rev vibration on the hard-mounted camera and exposure during
   translation. Gate before the next scored flight: HOVER over the printed
   marker and confirm live decode DURING flight. Mitigations: prop balance,
   camera-mount stiffness, forced short exposure if needed.
   Full data: docs/evidence/ulog_review_2026-08-21.md (finding 1 + addendum).
2. RC-loss drill + ELRS failsafe mode = **No Pulses** (2 minutes, props off).
3. FC microSD replacement (SanDisk Extreme U3 32 GB) — in progress
   2026-08-21 (old card archived + retired; new card pending f3/format/
   install/sd_bench).
4. ESC low-voltage cutoff vs the semi-solid pack (motors can cut in the air
   with charge remaining; no current sensor will warn us).
Backlog: `references/community-watchlist.md` ranked actions.

## New-bug rule

When a new bug is found and fixed: add its entry (symptom → mechanism → fix →
tool that now catches it) to the matching `references/*.md` IN THE SAME
change that fixes it, and if a check can catch the whole class, add the check
(`tools/px4_type_audit.py` is the template: it was written for one bug and
immediately caught its sibling).
