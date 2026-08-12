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

1. **บน CM4**: ติดตั้ง repo นี้ (`make install`) + mavlink-router แยกสตรีม FC:
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

## เครื่องจริง: ขั้นตอนสั่งบินหน้างาน (ลำดับจริงตอนซ้อม/แข่ง)

1. เช็คลิสต์ก่อนบิน: servo ต่อตาม `docs/SERVO_AUX_MAPPING.md` (ทดสอบ props-off มาแล้ว),
   `BAT1_*` calibrate แล้ว, แบตเต็ม, กล่อง/ไข่ใส่ครบ, GPS 3D fix, safety pilot ถือ RC
2. เปิด console (คำสั่งข้อ 3 ด้านบน) → เห็น telemetry สด → **✏️ เลือก pad** ตาม id
   ที่กรรมการ/โจทย์กำหนด → **💾 บันทึก**
3. ทุกคนถอยพ้นวง → **🚀 บิน mission [REAL]** → ยืนยัน → โดรน arm-takeoff เอง
4. ระหว่างบิน: ปุ่มแก้ไข/ปล่อย servo ล็อกเทาหมดโดยอัตโนมัติ — **การแทรกแซงมีทางเดียว
   คือ RC ของ safety pilot** (mode switch / kill) ซึ่งเหนือกว่า offboard เสมอ
5. ลงจอด + disarm → ปุ่มปลดล็อกเอง → เก็บผล ULog/audit จาก CM4

ข้อควรระวังของจริง: ถ้า WiFi/ssh ถึง CM4 ไม่ได้ ปุ่ม 🚀 จะขึ้น error ทันที (ssh ล้มเหลว
= ข้อความโผล่บนหน้าเว็บ) — mission ที่**บินไปแล้ว**ไม่พึ่ง ssh/WiFi ต่อ (orchestrator
อยู่บนโดรน) หลุด link แล้ว mission บินต่อจนจบเอง มีแต่จอที่มืด
