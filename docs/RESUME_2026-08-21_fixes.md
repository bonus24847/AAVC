# Handoff — 4 fixes ที่ operator อนุมัติแล้ว (2026-08-21 หลัง G7 attempt #1)

เขียนตอนปิดเซสชันภาคสนาม 21 ส.ค. — เซสชันใหม่เปิดมาทำ 4 ข้อนี้ได้เลย
บริบทเต็มของวัน: `docs/evidence/ulog_review_2026-08-21.md` (+ addendum) และ
PX4MASTER `references/ops-field.md` open items · ทุก commit ถึง `2c4b7ec`
deploy ขึ้น CM4 แล้ว (MD5 MATCH) · **อนุมัติครบทั้ง 4 ข้อโดย operator แล้ว
("เอาทั้ง 4 ข้อเลย") — รวมข้อ 4 ที่แตะ safety layer**

## สถานะเครื่อง ณ ปิดเซสชัน
- การ์ด SD ใหม่ (SanDisk Extreme 32GB, verified 29GiB) อยู่ใน **แลปท็อป**
  (ถูกดึงมาอ่าน ULog) — **ต้องเสียบกลับเข้า FC ก่อนบินครั้งหน้า!**
- แบตพัก 62% — ต้องชาร์จ (จด mAh ชาร์จกลับ = ground truth แถวแรก)
- Markers 38cm ทั้ง 6 อยู่ที่สนาม (ถ้ายังไม่เก็บ)
- `MPC_YAW_MODE=5` + `recording.hz=5` deploy แล้ว — จะ apply ตอน mission
  start ถัดไป (yaw mode เป็น live-apply ไม่ต้องรีบูต)
- หลักฐานไฟลต์: ULog `~/aavc_sdcard_archive_2026-08-21/g7attempt1/07_54_40.ulg`
  · เฟรม 457 ภาพ `~/aavc_cm4_runs/frames_g7attempt1/` · วิดีโอ mp4 สองไฟล์
  ใน `~/aavc_cm4_runs/` (10x + realtime)

## Fix 1 — บังคับ exposure สั้นที่กล้อง (คานงัดหลักของ in-flight blur)

**ทำไม**: เฟรมบิน sharpness 41-76 vs 680-780 ตอนนิ่ง (blur ~10x, แกว่งเฟรม
ต่อเฟรม = motion/vibration ไม่ใช่โฟกัส) → decode กลางอากาศ 1/457 เฟรม
**ที่ไหน**: `sitl/camera_grabber.py` — V4L2 backend เปิดกล้องที่ :69-81
(cv2.VideoCapture + CAP_PROP_*), argparse :143+
**แนวทาง**: flag `--exposure-100us N` → หลังเปิดกล้อง เรียก `v4l2-ctl -d <dev>
-c auto_exposure=1 -c exposure_time_absolute=N` ผ่าน subprocess (deterministic
กว่า CAP_PROP; ต้อง fail-soft ถ้า control ไม่มี — log แล้วไปต่อ) · ค่าเริ่ม
~10-30 (1-3ms) — ปรับด้วยการทดสอบจริง · เดินสายผ่าน `sitl/run_mission.sh`
ensure_infra (env `CAM_EXPOSURE`, default ตั้งใน script ฝั่ง REAL)
**ทดสอบ**: ลูป sharpness สด (แพตเทิร์นใน session log — ssh + Laplacian บน
/tmp/aavc_nadir.png) เทียบ auto vs manual กลางแจ้ง; ภาพต้องไม่มืดจนเกิน
(gain auto ช่วย); เกตจริง = hover-decode test

## Fix 2 — pad identified ขึ้นจอ GCS ทันทีผ่านวิทยุ

**ทำไม**: ไฟลต์ 1 ถูก operator ดึงลงทั้งที่กำลังไปได้ดี เพราะจอไม่แสดงอะไร
(WiFi ตายกลางอากาศ + beacon ส่งเฉพาะ confirmed ซึ่งยังไม่มี)
**ที่ไหน**:
- `orchestrator/gcs_status.py`: มี `pad_confirmed()` :123 (เขียน pads_mapped)
  ผ่าน `tracker_pusher()` :310 ซึ่งกรองเฉพาะ confirmed — เพิ่มการ push ขั้น
  identified (marker รู้ id แต่ยังไม่ครบโหวต) → field ใหม่ใน
  mission_status.json เช่น `"pads_identified": [4,5]`
- `cm4/status_beacon.py`: `compose_lines()` :76 — มีบรรทัด `AAVC p=... m=...
  ok=...` + `AAVC pads x:e,n` (chunked ≤50 chars) — เพิ่ม identified ids
  (เช่นรวมใน `m=`/บรรทัดใหม่ `AAVC seen=4,5`) ระวังงบ 50 ตัวอักษร/บรรทัด
- `~/Desktop/aavc-gcs/src/aavc_gcs.py`: `_parse_beacon` (มีอยู่แล้ว —
  radio_cam/radio_mission/radio_pads) + `.padbox` (เทา→เขียว+✓ ปัจจุบัน)
  → เพิ่มสถานะกลาง **ส้ม = identified**, เขียว = confirmed
**หมายเหตุ**: `MAV_1_FORWARD=1` ลงบอร์ดจริงแล้ว (INT32 fix) + FC รีบูตหลาย
รอบ → beacon-over-radio ควร active — ยืนยันบน bench ก่อนเชื่อ
**ทดสอบ**: `tests/test_status_beacon.py` มีอยู่ (ระวัง: assert ตามตำแหน่ง
list ที่เปราะ — backlog เดิม); เพิ่ม golden test บรรทัดใหม่; จอทดสอบด้วย
`--dry-run` ของ beacon

## Fix 3 — path + ลำดับจุดบนแผนที่ GCS

**ทำไม**: operator มองไม่ออกว่าเครื่อง "กำลังจะไปไหนต่อ" → takeover ก่อนเวลา
**ที่ไหน**:
- hook มีอยู่แล้ว: `orchestrator/mission.py::_rebuild_plan` :286-296 เรียก
  `on_plan_update(plan, pointer)` — main.py เป็นคน wire (ปัจจุบันต่อเข้า
  dashboard broadcaster) → ต่อเพิ่มเข้า `gcs_feed` (สร้างที่ main.py:717,
  ตัวอย่าง wiring :934) เขียน `"plan": [[lat,lon,kind,seq],...]` ลง
  mission_status.json (เขียนตอน staging = WiFi ยังอยู่ → จอได้แผนก่อนบิน)
- `aavc_gcs.py`: `read_mission_status()` :222 → วาด polyline + หมายเลขลำดับ
  (leaflet divIcon) บนแผนที่; อัปเดตเมื่อ plan เปลี่ยน (เท่าที่ sync ถึง)
**ระวัง**: mission.py คือ flight core — แตะเฉพาะฝั่ง main.py wiring +
gcs_status (ไม่ต้องแก้ mission.py เลยถ้า wire ผ่าน on_plan_update เดิม)

## Fix 4 — takeover ต้องฆ่า mission จริง (คดี zombie re-arm) ⚠ safety layer

**เหตุการณ์**: สองไฟลต์ติดกัน pilot takeover แล้ว mission loop ยังเดินต่อ
(เมื่อวาน: goto ค้าง 3.5 นาที; วันนี้: **พยายาม arm_and_takeoff เครื่องที่จอด
อยู่** ~8 นาทีหลัง takeover — รอดเพราะ FC ถูกดับไปแล้ว)
**ปริศนาที่ต้องไขก่อนแก้** (เบาะแสขุดไว้แล้ว):
- กลไกตรวจ **มีครบ**: `orchestrator/safety.py:285-303` — ตรวจ
  `t.flight_mode in _PILOT_MODES` (:53 มี "POSCTL" ครบ) ค้าง ≥1.0s
  (threshold :136) → `commander.stand_down()` (:303) + terminal
- `mavlink_adapter/commands.py:316-350` — `PilotInControlError` +
  `_pilot_in_control` latch + `stand_down()` มีแล้ว
- แหล่ง mode: `mavlink_adapter/telemetry.py:239-241` `mode.name` จาก MAVSDK
- **แต่ audit ทั้งสองคดีไม่มีบรรทัด "PILOT TAKEOVER" เลย** → ตัวตรวจไม่เคยยิง
- สมมติฐานที่ยังไม่ตัด: (a) MAVSDK `FlightMode.name` คืนค่าไม่ตรงกับชุด
  `_PILOT_MODES` (เช่น "POSITION"? ต้องเช็ค enum จริงของ mavsdk เวอร์ชันบน
  CM4), (b) safety watchdog ไม่ได้รันตอนนั้น, (c) `st.phase` เงื่อนไข,
  (d) stream flight_mode ตาย → ค้าง "UNKNOWN"
- **ขั้นแรกของ fix**: reproduce บน bench — subscribe flight_mode ผ่าน
  MAVSDK บน CM4 แล้วให้ pilot สับ POSCTL ดู string จริง; แล้วถึงแก้
**การแก้ที่ตกลงไว้** (ตาม pattern repo ข้างบ้าน + RESUME 2026-08-19 §2.2):
1. แก้ root cause ของ detector ที่ไม่ยิง (จากการ reproduce)
2. เพิ่ม detector ตัวที่สอง: **disarm ขณะ phase บิน** (SEARCH/TRANSIT/
   LOCALIZE/DROP — ไม่ใช่ LAND/PREFLIGHT ที่ disarm ถูกต้องตามแบบ multi-
   flight!) → stand_down + terminal ทันที
3. `stand_down()` ต้องกันครบทุก motion method รวม `arm_and_takeoff`
   (ตรวจว่า `_guard_pilot` ครอบหรือยัง — คดีวันนี้คือ arm_and_takeoff หลุด)
4. เพิ่ม `mode=` เข้า TELEM audit line ด้วย (จะได้วินิจฉัยย้อนหลังได้ —
   จำ lockstep: emitter + `tools/verify_flight.py` regex + docstring +
   golden tests ต้องไปด้วยกันใน commit เดียว)
**ทดสอบ**: `tests/test_safety.py` มีโครง pilot-takeover อยู่; เพิ่มเคส
disarm-in-flight-phase; RC-loss/takeover drill จริงบน bench (props off)

## กติกาปิดงาน (ทุก fix)
`make test` + `make lint` เขียว · `make type-audit` เขียว · commit บน branch
`fix/safety-review-2026-08-19` แยกต่อ fix · `bash cm4/deploy.sh
drone@10.42.0.1` + `--check` MD5 MATCH · aavc-gcs commit แยก repo (จอต้อง
restart console ถึงเห็นผล) · จดทุก fix ลง PX4MASTER references ตาม new-bug
rule · ⚠ อย่าใช้ `pkill -f` กับ pattern ที่มีชื่อไฟล์ซึ่งโผล่ในบรรทัดคำสั่ง
เดียวกัน (โดนมา 3 รอบใน 2 วัน — bracket ช่วยไม่ได้ถ้าชื่อจริงอยู่ท้ายบรรทัด)
