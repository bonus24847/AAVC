# Payload-release servo — AUX pin mapping (Pixhawk 6X)

**สถานะ:** กำหนดล่วงหน้า 2026-08-12 (ผู้ใช้สั่งล็อก mapping ก่อนเสียบจริง —
ยังไม่มี servo เสียบอยู่ ยกเว้นตัวทดลอง 1 ตัวที่ AUX1 เมื่อ 2026-08-06 ซึ่ง
ตรงกับตารางนี้พอดี ไม่ต้องย้าย)

คำสั่งปล่อยทุกเส้นทาง (orchestrator `drop_payload`, ปุ่ม "ปล่อย servo" บน
AAVC GCS) คือ **`MAV_CMD_DO_SET_ACTUATOR` (187)** — PX4 ไม่มี handler ของ
`DO_SET_SERVO` (183) จึงห้ามใช้เด็ดขาด. param7 = 0, param1..6 = ค่า actuator
ของ output function **"Peripheral via Actuator Set N"** (enum 301..306),
ช่องที่ไม่แตะส่ง NaN.

## ตาราง mapping (ล็อกแล้ว — sim กับของจริงใช้ชุดเดียวกัน)

| AUX pin | PX4 output function | payload_id | ตำแหน่ง servo บนแท่น (body frame: +x หน้า, +y ซ้าย) | Gazebo (SIM_GZ_SV) |
|---|---|---|---|---|
| **AUX 1** | Peripheral via Actuator Set 1 (301) | 0 | หน้า-ซ้าย (+0.10, +0.035) m จาก CG | SV_FUNC1 → `/model/eft_x6100_0/servo_0` |
| **AUX 2** | Peripheral via Actuator Set 2 (302) | 1 | หลัง-ขวา (−0.10, −0.035) m | SV_FUNC2 → `servo_1` |
| **AUX 3** | Peripheral via Actuator Set 3 (303) | 2 | หน้า-ขวา (+0.10, −0.035) m | SV_FUNC3 → `servo_2` |
| **AUX 4** | Peripheral via Actuator Set 4 (304) | 3 | หลัง-ซ้าย (−0.10, +0.035) m | SV_FUNC4 → `servo_3` |

ลำดับปล่อยของ mission คือ payload 0→1→2→3 (สลับมุมทแยง — โมเมนต์ CG
สุทธิเป็นศูนย์เมื่อเต็มและอีกครั้งหลังปล่อย 2 ตัวแรก)

## พารามิเตอร์บนบอร์ดจริง (ตั้งใน QGC → Actuators, props off)

```
PWM_AUX_FUNC1 = 301        # Peripheral via Actuator Set 1
PWM_AUX_FUNC2 = 302
PWM_AUX_FUNC3 = 303
PWM_AUX_FUNC4 = 304
PWM_AUX_MIN1..4  = 1000
PWM_AUX_MAX1..4  = 2000
PWM_AUX_DIS1..4  = 1100    # disarmed = ล็อก (ค่า hold เดียวกับ config)
PWM_AUX_FAIL1..4 = 1100    # failsafe = ล็อก — ห้ามทำของหล่นตอน failsafe
```

ค่า PWM ใช้งาน: **ปล่อย 1900 µs (+0.8) / ล็อก 1100 µs (−0.8)** — ตรงกับ
`sitl/aavc_config.yaml` (`drop_servo_pwm_release/hold`) และ
`gcs/kmutnb_field.yaml` (`released_us/held_us`, `outputs: [1,2,3,4]` =
actuator set index = เลข AUX pin พอดี)

## ข้อควรรู้จากการทดสอบ SITL (2026-08-12)

1. **ตอน DISARMED ช่อง servo ถูกบังคับเป็นค่า disarmed (ล็อก) เสมอ** —
   คำสั่งปล่อยมีผลเฉพาะตอน ARMED. ระหว่าง mission ไม่กระทบ (เครื่อง armed
   ตลอดรวมถึงตอนเกาะ pad เพราะ `COM_DISARM_LAND = -1`) แต่การทดสอบ
   bench/เดโมบนพื้นต้อง arm ก่อน (props off!)
2. pymavlink รุ่นใหม่ (≥2.4.49) ไม่มีชื่อ `MAV_CMD_DO_SET_ACTUATOR` ใน enum
   แล้ว — โค้ดทุกจุดใช้ `getattr(mavutil.mavlink, "MAV_CMD_DO_SET_ACTUATOR",
   187)` (ทำแล้วทั้งใน repo นี้และ aavc-gcs)
3. ใน SITL เส้นทางเดียวกันนี้ผ่านการทดสอบครบ: MAVSDK `set_actuator` /
   ปุ่ม GCS → PX4 → gz topic (มุม +0.63 rad = ปล่อย) →
   `payload_detach_bridge --servo` → กล่องหลุดจริง 4/4

## Bench checklist ตอนเสียบจริง (props off)

1. เสียบ servo ตามตาราง AUX 1–4 ข้างบน
2. ตั้งพารามิเตอร์ตามบล็อกข้างบนใน QGC แล้ว reboot FC
3. Arm (props off) → กดปุ่ม "ปล่อย servo" ทีละตัวบน AAVC GCS
   (`make aavc-gcs`) → servo ตัวที่ตรง AUX ต้องสวิงไป 1900 แล้วผู้ทดสอบ
   กด hold กลับ 1100 → เช็คว่าตัวอื่น **ไม่ขยับ** (NaN masking ทำงาน)
4. ทดสอบ failsafe: disarm → servo ทุกตัวต้องกลับ/ค้างที่ 1100 (ล็อก)
