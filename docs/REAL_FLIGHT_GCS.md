# สั่งบิน mission จาก AAVC GCS — SIM vs เครื่องจริง

ปุ่มบน GCS ชุดเดียวกันใช้ได้ทั้งสองโลก ต่างกันแค่ **คำสั่งที่ตั้งไว้ตอนเปิด console**
(`--mission-cmd`) และป้ายบนปุ่มจะบอกเองว่า console นี้บินโลกไหน:

| ปุ่ม | SIM | เครื่องจริง |
|---|---|---|
| ✏️ เลือก / แก้ไข pad | ✔ | ✔ (ขั้นตอนเดียวกันเป๊ะ) |
| 🚀 บิน mission **[SIM]** / **[REAL]** | ✔ spawn `sitl/run_mission.sh` บนเครื่องนี้ | ✔ ssh ไปสั่ง orchestrator บน **CM4** |
| 🧹 รีเซ็ตสนาม **[SIM]** | ✔ เกิดใหม่ทั้ง SITL+pad+กล่อง | ✘ **ไม่มี** (ปุ่มซ่อนเอง — สนามจริงรีเซ็ตด้วยมือ) |
| ปุ่มปล่อย servo S1–S4 / ทั้งหมด | ✔ (กล่องหลุดใน gz) | ✔ (servo จริงบน AUX 1–4) — ล็อกอัตโนมัติระหว่าง mission |
| ล็อกแก้ไขขณะบิน (ปุ่มเทา 🔒) | ✔ | ✔ (ทั้งตอน mission รัน และตอนเครื่อง ARMED) |

ส่วนที่เป็น **SIM เท่านั้น** นอกเหนือจากปุ่ม 🧹: `launch_stack.sh`, `spawn_targets`,
`--truth-json` (truth audit), payload-detach bridge — ของจริงไม่มีและไม่ต้องมี
(กล่องจริงหล่นเอง, ไม่มี ground truth ให้เทียบ)

## SIM (ที่ใช้อยู่ตอนนี้)

```bash
bash sitl/launch_stack.sh        # หรือ make stack — ตั้ง --mission-cmd/--reset-cmd ให้เอง
# เปิด http://127.0.0.1:8000 → เลือก pad → 💾 บันทึก → 🚀 บิน mission [SIM]
```

ถ้าสนามยังมีกล่องจากรอบก่อน (บัง marker) ปุ่ม 🚀 จะ**ปฏิเสธพร้อมเหตุผล** —
กด **🧹 รีเซ็ตสนาม [SIM]** (~1 นาที, console ไม่ดับ) แล้วค่อยกด 🚀 ใหม่

## เครื่องจริง (G5+): ขั้นตอนตั้งระบบ

โครงสร้าง: **orchestrator รันบน CM4** (companion บนโดรน) — โน้ตบุ๊ก GCS เป็นแค่จอ+ปุ่ม
ปุ่ม 🚀 จึงต้อง ssh ไปสั่ง CM4 ไม่ใช่รันบนโน้ตบุ๊ก

1. **บน CM4**: อัพ repo ขึ้นไปด้วย `cm4/deploy.sh aavc@<cm4> --install`
   (rsync + สร้าง venv บน CM4 เอง — รันซ้ำได้ตลอด incremental) + mavlink-router แยกสตรีม FC:
   - `udpin://0.0.0.0:14540` → orchestrator (offboard, บน CM4 เอง)
   - `<ip โน้ตบุ๊ก>:14550` → GCS (telemetry)
2. **ตั้ง ssh key** จากโน้ตบุ๊กเข้า CM4 (กดปุ่มแล้วต้องไม่ถามรหัส): `ssh-copy-id aavc@<cm4>`
3. **เปิด console บนโน้ตบุ๊ก**:
   ```bash
   /usr/bin/python3 ~/Desktop/aavc-gcs/src/aavc_gcs.py \
     --field gcs/kmutnb_field.yaml --captures captures \
     --url udpin:0.0.0.0:14550 \
     --mission-cmd "ssh aavc@<cm4> 'REAL=1 ~/mission/sitl/run_mission.sh {ids}'" \
     --mission-label REAL
   ```
   (ไม่ใส่ `--reset-cmd` → ปุ่ม 🧹 หายไปเอง; `REAL=1` ใน run_mission.sh
   สลับไป `--connect udpin://0.0.0.0:14540` และตัด truth audit ให้แล้ว)
4. **เปิด status sync** อีก terminal (จอ stepper/pad ✓ ต้องพึ่งตัวนี้ — ดู
   "ลิงก์วิทยุหน้างาน" ด้านล่าง): `cm4/status_sync.sh aavc@<cm4>`

## เครื่องจริง: ขั้นตอนสั่งบินหน้างาน (ลำดับจริงตอนซ้อม/แข่ง)

1. เช็คลิสต์ก่อนบิน: servo ต่อตาม `docs/SERVO_AUX_MAPPING.md` (ทดสอบ props-off มาแล้ว),
   `BAT1_*` calibrate แล้ว, แบตเต็ม, กล่อง/ไข่ใส่ครบ, GPS 3D fix, safety pilot ถือ RC
2. เปิด console (คำสั่งข้อ 3 ด้านบน) → เห็น telemetry สด → **✏️ เลือก pad** ตาม id
   ที่กรรมการ/โจทย์กำหนด → **💾 บันทึก**
3. ทุกคนถอยพ้นวง → **🚀 บิน mission [REAL]** → ยืนยัน → โดรน arm-takeoff เอง
4. ระหว่างบิน: ปุ่มแก้ไข/ปล่อย servo ล็อกเทาหมดโดยอัตโนมัติ — **การแทรกแซงมีทางเดียว
   คือ RC ของ safety pilot** (mode switch / kill) ซึ่งเหนือกว่า offboard เสมอ
5. ลงจอด + disarm → ปุ่มปลดล็อกเอง → เก็บผล ULog/audit จาก CM4

⚠ **ห้าม arm เองหรือสลับ offboard เองก่อนกด 🚀** — ปุ่ม 🚀 คือคำสั่ง arm ของภารกิจ:
orchestrator บน CM4 จะเช็ค preflight แล้ว arm + takeoff เองทันทีที่ผ่าน เหตุผลที่ลำดับอื่นใช้ไม่ได้:

- **arm เองก่อน → ปุ่ม 🚀 จะปฏิเสธ** (interlock "ห้ามสั่งขณะ armed" กันสั่งซ้อนกลางอากาศ
  — ตั้งใจออกแบบไว้แบบนี้) และ PX4 จับ home ตอน arm: ต้อง arm ที่จุด L&R โดย
  orchestrator เพื่อให้ home/นาฬิกา window ตรงกับภารกิจ
- **สลับ RC เข้า OFFBOARD เอง → PX4 ปฏิเสธ/failsafe** เพราะโหมด offboard ต้องมี
  companion stream setpoint อยู่ก่อนแล้ว — ถ้า orchestrator ยังไม่ได้เริ่ม (ยังไม่กด 🚀)
  ไม่มีสัญญาณอะไรให้ตาม และถึงกดแล้ว mission ก็บินโหมด AUTO เกือบทั้งเที่ยว
  (takeoff/goto/land) โดย orchestrator เป็นคนสลับโหมดเองทุกจังหวะ
- บทบาทของคน: operator = เลือก pad + กด 🚀; **safety pilot = ถือ RC เฉย ๆ**
  ไม่แตะสวิตช์จนกว่าจะต้อง override (mode switch ออกจาก AUTO หรือ kill) —
  สวิตช์ RC เหนือกว่าคำสั่ง companion เสมอ จึงเป็นเบรกมือที่ใช้ได้ตลอดเวลา

ข้อควรระวังของจริง: ถ้า WiFi/ssh ถึง CM4 ไม่ได้ ปุ่ม 🚀 จะขึ้น error ทันที (ssh ล้มเหลว
= ข้อความโผล่บนหน้าเว็บ) — mission ที่**บินไปแล้ว**ไม่พึ่ง ssh/WiFi ต่อ (orchestrator
อยู่บนโดรน) หลุด link แล้ว mission บินต่อจนจบเอง มีแต่จอที่มืด

## ลิงก์วิทยุหน้างาน: WiFi ระยะสั้น + Nomad (ELRS)

WiFi ถึง CM4 ไปได้ไม่ไกล — **ไม่เป็นไร ระบบออกแบบมาแบบนั้น**: WiFi ถูกใช้เฉพาะตอน
โดรน**อยู่บนพื้นที่ L&R ข้างโต๊ะ** (ระยะไม่กี่เมตร) ส่วนลิงก์เดียวที่ต้องถึงตลอด
เที่ยวบินคือ **RC ของ safety pilot (Nomad/ELRS)** ซึ่งระยะไกลกว่าสนามหลายเท่าอยู่แล้ว

| ลิงก์ | ใช้ทำอะไร | ต้องถึงเมื่อไหร่ |
|---|---|---|
| WiFi → CM4 (ssh) | deploy, ปุ่ม 🚀, `status_sync`, เก็บ ULog/audit | เฉพาะโดรนอยู่พื้นที่ L&R (ก่อนบิน + หลังลง) |
| Nomad ELRS (RC) | override ของ safety pilot (mode/kill) | **ตลอดเที่ยวบิน** — ลิงก์ safety ตัวจริง |
| จอ console กลางเที่ยว | ดูเฉย ๆ ไม่ใช่ safety | ไม่บังคับ — หลุดแล้ว mission บินต่อจนจบเอง |

**status sync (จำเป็นในโหมด REAL):** orchestrator เขียน `captures/mission_status.json`
บน CM4 แต่ console อ่านไฟล์บนโน้ตบุ๊ก — เปิด terminal คู่กับ console:

```bash
cm4/status_sync.sh aavc@<cm4>
```

ดึงทุก 2 วิ ผ่าน ssh; หลุดระยะ = จอ stepper/pad ✓ ค้าง (console มี staleness gate 45 วิ
เทาให้เอง) แล้วเด้งกลับมาสดทันทีที่โดรนกลับเข้าระยะ (เช่น ตอนลงจอดที่ L&R —
ผล ✓ ครบทุก pad จะขึ้นตอนนั้น)

**อยากได้จอสดตลอดเที่ยว (ของแถม ไม่บังคับ):**
- ทางที่ 1 — **ELRS MAVLink mode** บน Nomad: ELRS ≥3.4 ทั้ง TX/RX สลับโหมด MAVLink,
  RX ต่อ UART ของ 6X, telemetry วิ่ง RC link → backpack ของ Nomad → UDP 14550 →
  console ตรง ๆ **VERIFY-AT-BENCH ก่อนเท่านั้น**: ต้องยืนยันว่า build fmu-v6x ของเรา
  รับ RC-over-MAVLink (`RADIO_RC_CHANNELS`) แล้ว failsafe ครบเหมือน CRSF —
  **ห้ามสลับโหมดครั้งแรกที่หน้างาน** เพราะมันแตะลิงก์ safety
- ทางที่ 2 — ซื้อวิทยุ telemetry SiK 915 MHz หนึ่งคู่ (~พันบาท) เสียบ TELEM1 → USB
  โน้ตบุ๊ก → 14550: จอสดเต็มสนามโดยไม่แตะลิงก์ RC เลย (ทางมาตรฐาน แนะนำถ้าจะซื้อ)

ค่าเริ่มต้นที่แนะนำสำหรับ test แรก ๆ: **Nomad คงเป็น RC ล้วน (CRSF)** — อย่าเพิ่งเอา
ลิงก์ safety ไปพ่วงงานจอ
