# สั่งบิน mission จาก AAVC GCS — SIM vs เครื่องจริง

## เลือก template mission จาก dropdown (console เดียว 2 mission)

การ์ด 📋 Mission มี **dropdown เลือก template mission** (operator 2026-08-13 —
แทนที่ field editor แบบวาดมือที่ถูกถอดออก): แต่ละ template ผูก แผนที่สนาม
(field yaml) + โฟลเดอร์ผล + คำสั่งปุ่ม 🚀 ของ mission นั้นครบชุด — สลับแล้ว
ทุกอย่างชี้ตาม ทันที (ล็อกระหว่างบิน):

- **KMUTNB สนามซ้อม (เพดาน 5 m)** — repo นี้
- **AAVC สนามแข่ง (เพดาน 20 m)** — repo `mission_AAVC` (entry:
  `scripts/run_mission.sh --ids {ids}`, RC-GO รองรับเหมือนกัน)

รายการ template อยู่ที่ `~/Desktop/aavc-gcs/missions.yaml`; ขั้นตอนต่อ
เที่ยวบิน**เหมือนกันทุก mission** — repo ที่จะเข้าระบบต้องมี: (1) entry รับ
`{ids}` (2) field yaml (3) ตัวเขียน `mission_status.json` ลง captures

**ปรับพิกัดสนามวันแข่ง (event briefing):** แก้ที่ **field yaml + config ของ
template นั้น** — ไฟล์ template คือแหล่งความจริงเดียวของ geometry ไม่มีการ
วาด/override หน้างาน (ตัดสินใจ operator: deterministic กว่า)

ปุ่มบน GCS ชุดเดียวกันใช้ได้ทั้งสองโลก ต่างกันแค่ **คำสั่งที่ตั้งไว้ตอนเปิด console**
(`--mission-cmd`) และป้ายบนปุ่มจะบอกเองว่า console นี้บินโลกไหน:

| ปุ่ม | SIM | เครื่องจริง |
|---|---|---|
| ✏️ เลือก / แก้ไข pad | ✔ | ✔ (ขั้นตอนเดียวกันเป๊ะ) |
| 🚀 up ขึ้นโดรน **[SIM]** / **[REAL]** | ✔ spawn `sitl/run_mission.sh` บนเครื่องนี้ | ✔ ssh ไปสั่ง orchestrator บน **CM4** |
| 🧹 รีเซ็ตสนาม **[SIM]** | ✔ เกิดใหม่ทั้ง SITL+pad+กล่อง | ✘ **ไม่มี** (ปุ่มซ่อนเอง — สนามจริงรีเซ็ตด้วยมือ) |
| ปุ่มปล่อย servo S1–S4 / ทั้งหมด | ✔ (กล่องหลุดใน gz) | ✔ (servo จริงบน AUX 1–4) — ล็อกอัตโนมัติระหว่าง mission |
| ล็อกแก้ไขขณะบิน (ปุ่มเทา 🔒) | ✔ | ✔ (ทั้งตอน mission รัน และตอนเครื่อง ARMED) |

ส่วนที่เป็น **SIM เท่านั้น** นอกเหนือจากปุ่ม 🧹: `launch_stack.sh`, `spawn_targets`,
`--truth-json` (truth audit), payload-detach bridge — ของจริงไม่มีและไม่ต้องมี
(กล่องจริงหล่นเอง, ไม่มี ground truth ให้เทียบ)

## SIM (ที่ใช้อยู่ตอนนี้)

```bash
bash sitl/launch_stack.sh        # หรือ make stack — ตั้ง --mission-cmd/--reset-cmd ให้เอง
# เปิด http://127.0.0.1:8000 → เลือก pad → 💾 บันทึก → 🚀 up ขึ้นโดรน [SIM]
```

ถ้าสนามยังมีกล่องจากรอบก่อน (บัง marker) ปุ่ม 🚀 จะ**ปฏิเสธพร้อมเหตุผล** —
กด **🧹 รีเซ็ตสนาม [SIM]** (~1 นาที, console ไม่ดับ) แล้วค่อยกด 🚀 ใหม่

อยากเห็นภาพกล้องมองล่างระหว่าง test: `make camera-view` (หน้าต่างภาพสดจาก
`/tmp/aavc_nadir.png` — สิ่งที่ detector เห็นจริง ๆ; SIM-only, จอ GCS ไม่มีแผงกล้อง)

## เครื่องจริง (G5+): ขั้นตอนตั้งระบบ

โครงสร้าง: **orchestrator รันบน CM4** (companion บนโดรน) — โน้ตบุ๊ก GCS เป็นแค่จอ+ปุ่ม
ปุ่ม 🚀 จึงต้อง ssh ไปสั่ง CM4 ไม่ใช่รันบนโน้ตบุ๊ก

1. **บน CM4**: อัพ repo ขึ้นไปด้วย `cm4/deploy.sh aavc@<cm4> --install`
   (rsync + สร้าง venv บน CM4 เอง — รันซ้ำได้ตลอด incremental) + mavlink-router แยกสตรีม FC:
   - `udpin://0.0.0.0:14540` → orchestrator (offboard, บน CM4 เอง)
   - `<ip โน้ตบุ๊ก>:14550` → GCS (telemetry)
2. **ตั้ง ssh key** จากโน้ตบุ๊กเข้า CM4 (กดปุ่มแล้วต้องไม่ถามรหัส): `ssh-copy-id aavc@<cm4>`
3. **เปิด console + status sync บนโน้ตบุ๊ก — คำสั่งเดียว**:
   ```bash
   cm4/launch_gcs_real.sh aavc@<ip-cm4>
   ```
   สคริปต์นี้ตั้งค่า GCS ให้ครบทุกตัว (จะได้ไม่ต้องพิมพ์เองหน้างาน) และเปิด
   `status_sync` คู่กันอัตโนมัติ — Ctrl-C ปิดทั้งคู่ ค่าที่มันตั้งให้คือ:

   | ค่า | ความหมาย |
   |---|---|
   | `--field gcs/kmutnb_field.yaml` | เส้น geofence/search/transit ของสนามบนแผนที่ |
   | `--captures captures` | โฟลเดอร์ที่ status_sync ดึงผลจาก CM4 มาลง (จอ stepper/pad ✓) |
   | `--url udpin:0.0.0.0:14550` | ช่องรับ telemetry — Nomad backpack / mavlink-router ยิงมาที่นี่ |
   | `--mission-cmd "ssh … REAL=1 run_mission.sh {ids}"` | ปุ่ม 🚀 = ssh ไป stage mission บน CM4 (`{ids}` = pad ที่เลือก) |
   | `--mission-label REAL` | ป้าย [REAL] บนปุ่ม |
   | *(ไม่ใส่ `--reset-cmd`)* | ปุ่ม 🧹 (SIM-only) ซ่อนตัวเอง |

   (`REAL=1` ใน run_mission.sh สลับ endpoint + ตัด truth audit + เปิด RC-GO
   ให้เองครบ — อยากสั่งมือดูคำสั่งเต็มได้ในตัวสคริปต์)

## เครื่องจริง: ขั้นตอนสั่งบินหน้างาน (ลำดับจริงตอนซ้อม/แข่ง)

1. เช็คลิสต์ก่อนบิน: servo ต่อตาม `docs/SERVO_AUX_MAPPING.md` (ทดสอบ props-off มาแล้ว),
   `BAT1_*` calibrate แล้ว, แบตเต็ม, กล่อง/ไข่ใส่ครบ, GPS 3D fix, safety pilot ถือ RC
2. เปิด console (คำสั่งข้อ 3 ด้านบน) → เห็น telemetry สด → **✏️ เลือก pad** ตาม id
   ที่กรรมการ/โจทย์กำหนด → **💾 บันทึก**
3. ทุกคนถอยพ้นวง → **🚀 up ขึ้นโดรน [REAL]** = **stage เท่านั้น** (โหลด mission เข้า
   เครื่องรอ — เครื่องยังไม่ขยับ)
4. safety pilot **arm ด้วย RC** (POSCTL) → **สลับเข้า OFFBOARD = เครื่องออก**
   (ลำดับเต็มในหัวข้อ RC-GO ถัดไป)
5. ระหว่างบิน: ปุ่มเว็บล็อกเทาหมด — การแทรกแซงมีทางเดียวคือ RC:
   **สลับเข้า POSCTL = ยึดเครื่องคืน** (orchestrator หยุดสั่งถาวร) หรือ kill
6. ลงจอด + disarm → ปุ่มปลดล็อกเอง → เก็บผล ULog/audit จาก CM4

## RC-GO: การปล่อยเครื่องเป็นของ RC ไม่ใช่ของเว็บ (default ของ REAL)

โหมด REAL ตั้งต้นเป็น **RC-GO** (`run_mission.sh` ใส่ `RC_GO=1` ให้เอง;
`RC_GO=0` ถอยกลับเป็นแบบ 🚀-แล้วบินเลยของ SITL) — ปุ่มเว็บ**ไม่มีสิทธิ์ทำให้
เครื่องขยับ** ทุกการปล่อยเครื่องอยู่บนมือ RC:

1. **กด 🚀 = stage**: orchestrator บน CM4 เช็ค preflight แล้ว**หยุดรอ** พร้อม
   stream setpoint ความเร็วศูนย์ ~5 Hz — stream นี้ไม่สั่งอะไรและไม่เปลี่ยนโหมด
   มันแค่เปิดทางให้สวิตช์ OFFBOARD ของ RC เข้าได้ (PX4 ปฏิเสธ OFFBOARD
   ถ้าไม่มี setpoint สดวิ่งอยู่)
2. **arm ด้วย RC** (POSCTL) — เครื่องนิ่งอยู่กับพื้น
3. **สลับเข้า OFFBOARD = ปล่อยเครื่อง**: orchestrator เห็น armed+OFFBOARD
   แล้วรับช่วงใน ~0.2 วิ (takeoff → transit → …) **นาฬิกา 20 นาทีเริ่มนับที่
   จังหวะสลับสวิตช์** ไม่ใช่ตอนกด 🚀 — รอนานแค่ไหนก็ไม่เสียเวลา window
4. **มีปัญหา → สลับเข้า POSCTL**: PX4 เชื่อ RC ทันที และภายใน ~1 วิ
   orchestrator **หยุดสั่งถาวร** (audit `PILOT TAKEOVER`, ไม่มี goto/land
   ย้อนมาสู้กับนักบิน) — จากนั้นเอาลงเองได้เลย

ลำดับต้อง **stage ก่อน → arm ทีหลัง**: interlock ของเว็บปฏิเสธการกด 🚀 ขณะ
armed (กันสั่งซ้อนกลางอากาศ) และก่อน stage ยังไม่มี setpoint stream —
สวิตช์ OFFBOARD จะเข้าไม่ได้อยู่ดี

ข้อควรระวังของจริง: ถ้า WiFi/ssh ถึง CM4 ไม่ได้ ปุ่ม 🚀 จะขึ้น error ทันที (ssh ล้มเหลว
= ข้อความโผล่บนหน้าเว็บ) — mission ที่**บินไปแล้ว**ไม่พึ่ง ssh/WiFi ต่อ (orchestrator
อยู่บนโดรน) หลุด link แล้ว mission บินต่อจนจบเอง มีแต่จอที่มืด

## ลิงก์วิทยุหน้างาน: WiFi ระยะสั้น + Nomad (ELRS)

WiFi ถึง CM4 ไปได้ไม่ไกล — **ไม่เป็นไร ระบบออกแบบมาแบบนั้น**: WiFi ถูกใช้เฉพาะตอน
โดรน**อยู่บนพื้นที่ L&R ข้างโต๊ะ** (ระยะไม่กี่เมตร) ส่วนลิงก์เดียวที่ต้องถึงตลอด
เที่ยวบินคือ **RC ของ safety pilot (Nomad/ELRS)** ซึ่งระยะไกลกว่าสนามหลายเท่าอยู่แล้ว

| ลิงก์ | ใช้ทำอะไร | ต้องถึงเมื่อไหร่ |
|---|---|---|
| WiFi → CM4 (ssh) | deploy, ปุ่ม 🚀 (stage), `status_sync`, เก็บ ULog/audit | เฉพาะโดรนอยู่พื้นที่ L&R (ก่อนบิน + หลังลง) |
| Nomad ELRS (RC + MAVLink) | arm/OFFBOARD = ปล่อยเครื่อง (RC-GO), POSCTL/kill = ยึดคืน + telemetry ลงจอ 14550 | **ตลอดเที่ยวบิน** — ลิงก์ safety ตัวจริง |
| จอ console กลางเที่ยว | ดูเฉย ๆ ไม่ใช่ safety | ไม่บังคับ — หลุดแล้ว mission บินต่อจนจบเอง |

**status sync (จำเป็นในโหมด REAL):** orchestrator เขียน `captures/mission_status.json`
บน CM4 แต่ console อ่านไฟล์บนโน้ตบุ๊ก — เปิด terminal คู่กับ console:

```bash
cm4/status_sync.sh aavc@<cm4>
```

ดึงทุก 2 วิ ผ่าน ssh; หลุดระยะ = จอ stepper/pad ✓ ค้าง (console มี staleness gate 45 วิ
เทาให้เอง) แล้วเด้งกลับมาสดทันทีที่โดรนกลับเข้าระยะ (เช่น ตอนลงจอดที่ L&R —
ผล ✓ ครบทุก pad จะขึ้นตอนนั้น)

## Nomad ELRS MAVLink mode (ทางที่เลือกใช้ — operator 2026-08-12)

RC + telemetry วิ่งบนลิงก์ ELRS เส้นเดียว: 6X ↔ ELRS RX ↔ อากาศ ↔ Nomad ↔
backpack ของโมดูล → UDP **14550** → console กินตรง ๆ ทั้งเที่ยว (operator เคยบิน
โครงนี้มาก่อนแล้วกับ FC เดิม — แต่ **6X + PX4 1.17 ของเรายังไม่เคย** จึงต้องไล่
bench checklist ให้ครบก่อน ห้ามสลับโหมดครั้งแรกที่หน้างาน เพราะแตะลิงก์ safety):

**Bench checklist (props off, ทำครั้งเดียว):**
1. ELRS **≥3.4 ทั้ง TX (Nomad) + RX** → ตั้ง serial protocol = **MAVLink**
   ทั้งคู่ (Lua script / WebUI)
2. ELRS RX ย้ายไป **UART ว่างของ 6X** (เช่น TELEM1) — ตั้ง `MAV_n_CONFIG`
   ของพอร์ตนั้น + baud ตามค่า ELRS WebUI และลด rate ลง (ลิงก์แคบ ~กิโลไบต์/วิ
   เอาแค่ HEARTBEAT/position/battery/mode พอสำหรับจอ)
3. **RC เข้า**: PX4 ต้องเห็นช่อง RC ที่มากับ MAVLink (`RADIO_RC_CHANNELS`) —
   ดู radio bars ใน QGC / `listener input_rc` แล้ว **arm ด้วย RC ให้สำเร็จ**
   (โหมดนี้ไม่ใช้ CRSF driver — ข้อจำกัด `CONFIG_DRIVERS_RC_CRSF_RC` หายไป)
   ⚠ เช็ก `COM_RC_IN_MODE` ด้วย (บทเรียนจาก repo แข่ง 2026-08-13: ค่า 4 =
   stick disabled ทำให้ PX4 **เงียบเฉยต่อการสลับ POSCTL** — takeover ไม่ทำงาน
   โดยไม่มี error ใด ๆ): ค่าที่ถูกต้องต้องยอมรับ RC/มี fallback ที่ตั้งใจ
   แล้วทดสอบสลับ POSCTL จริงบน bench ให้เห็นโหมดเปลี่ยน
4. **Failsafe**: ปิด Nomad ทั้งเครื่อง → PX4 ต้องประกาศ RC loss ในไม่กี่วินาที
   และ action เป็น RTL ตามที่ pin ไว้ — ข้อนี้ผ่านไม่ได้ = ไม่บินจริง
5. **Telemetry ลงจอ**: โน้ตบุ๊กต่อ WiFi ของ backpack (ระยะแค่โต๊ะ operator ↔
   นักบิน) → console เห็น HEARTBEAT/ตำแหน่งที่ 14550 → เดินถือเครื่องออกไป
   สุดสนาม จอต้องยังสด
6. ปิดท้ายด้วย **RC-GO เต็มลำดับบน bench**: stage → arm → OFFBOARD (มอเตอร์ถอด
   ใบ) → POSCTL → เห็น `PILOT TAKEOVER` ใน audit

ทางสำรอง — วิทยุ telemetry SiK 915 MHz หนึ่งคู่ (~พันบาท) เสียบ TELEM1 → USB
โน้ตบุ๊ก → 14550: จอสดเต็มสนามโดยไม่พ่วงลิงก์ RC (มาตรฐานสุด ถ้า MAVLink mode
ติดขัดข้อไหนให้ถอยมาทางนี้ + Nomad กลับเป็น CRSF ล้วน)

หมายเหตุ: จอสดจาก Nomad คือ telemetry (ตำแหน่ง/โหมด/แบต) — ส่วน stepper/pad ✓
มาจาก `mission_status.json` ผ่าน `status_sync` (WiFi) เหมือนเดิม: กลางเที่ยวอาจ
ค้าง แล้วสรุปผลครบตอนโดรนกลับเข้าระยะ
