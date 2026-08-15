# Payload-release servo — AUX pin mapping (Pixhawk 6X)

**สถานะ:** ✅ **AS-WIRED + ยืนยันด้วยฮาร์ดแวร์แล้ว 2026-08-15**
(ตารางเดิมที่ล็อกไว้แบบ "บนกระดาษ" เมื่อ 2026-08-12 สมมติว่า AUX pin เรียงตาม
ลำดับปล่อยพอดี ซึ่งของจริงไม่ตรง — ตารางนี้แทนที่ทั้งหมด). **สายไม่ต้องย้าย**
— แก้ที่ซอฟต์แวร์ (`connection.drop_servo_channels`) ตามที่ผู้ใช้เลือก

การยืนยัน (บอร์ด FMU v6X จริง, เซสชัน `mission-aavc-sitl-setup` ทำให้, ผู้ใช้ดูมุม
ที่ขยับจริงทีละช่อง): ตั้ง `PWM_AUX_FUNC1..4 = 301..304` แล้วยิง `ACTUATOR_TEST`
ตอน **disarm** ทีละพิน → **AUX1 = ท้าย-ขวา, AUX2 = หน้า-ขวา, AUX3 = ท้าย-ซ้าย,
AUX4 = หน้า-ซ้าย ครบทั้งสี่มุม** ⇒ `[4, 1, 2, 3]` ไม่ใช่ "การรายงานด้วยตา" อีกต่อไป
(SITL + unit test ยืนยันได้แค่ตรรกะซอฟต์แวร์ — ข้อนี้คือชิ้นที่ยืนยัน **สายเส้นไหน
ไปมุมไหน** ซึ่งไม่มีอะไรในรีโปนี้ทำแทนได้)

คำสั่งปล่อยทุกเส้นทาง (orchestrator `drop_payload`, ปุ่ม "ปล่อย servo" บน
AAVC GCS) คือ **`MAV_CMD_DO_SET_ACTUATOR` (187)** — PX4 ไม่มี handler ของ
`DO_SET_SERVO` (183) จึงห้ามใช้เด็ดขาด. param7 = 0, param1..6 = ค่า actuator
ของ output function **"Peripheral via Actuator Set N"** (enum 301..306),
ช่องที่ไม่แตะส่ง NaN.

## 1. การต่อสายจริง (ตำแหน่งบนแท่น → AUX pin)

"บน" = ด้านที่อยู่ใกล้ **หัวโดรน** (body frame: +x หน้า, +y ซ้าย)

| ตำแหน่งบนแท่น | พิกัดจาก CG | **AUX pin** | actuator set (= เลข AUX) | Gazebo |
|---|---|---|---|---|
| ซ้ายบน = **หน้า-ซ้าย** | (+0.10, +0.035) m | **AUX 4** | Set 4 (304) | `SV_FUNC4` → `servo_3` → `cargo_payload_3` |
| ขวาบน = **หน้า-ขวา** | (+0.10, −0.035) m | **AUX 2** | Set 2 (302) | `SV_FUNC2` → `servo_1` → `cargo_payload_1` |
| ซ้ายล่าง = **ท้าย-ซ้าย** | (−0.10, +0.035) m | **AUX 3** | Set 3 (303) | `SV_FUNC3` → `servo_2` → `cargo_payload_2` |
| ขวาล่าง = **ท้าย-ขวา** | (−0.10, −0.035) m | **AUX 1** | Set 1 (301) | `SV_FUNC1` → `servo_0` → `cargo_payload_0` |

> **actuator set index == เลข AUX pin เสมอ** (`PWM_AUX_FUNCn = 300+n`) — ไม่
> เปลี่ยน. ปุ่มบน AAVC GCS (`outputs: [1,2,3,4]` ใน `gcs/kmutnb_field.yaml`)
> จึงยิงตรงตามเลข AUX: กดปุ่ม 1 = AUX 1 = ท้าย-ขวา

## 2. ลำดับปล่อยของ mission (payload_id → AUX)

ลำดับ **ทแยงสลับ** — โมเมนต์ CG สุทธิเป็นศูนย์ตอนเต็ม และเป็นศูนย์อีกครั้ง
หลังปล่อยไข่ 2 ใบแรก:

| payload_id (ลำดับปล่อยในเที่ยวบิน) | ตำแหน่ง | AUX pin / actuator set |
|---|---|---|
| 0 | หน้า-ซ้าย | **4** |
| 1 | ท้าย-ขวา | **1** |
| 2 | หน้า-ขวา | **2** |
| 3 | ท้าย-ซ้าย | **3** |

→ `sitl/aavc_config.yaml`:

```yaml
connection:
  drop_servo_channels: [4, 1, 2, 3]   # payload_id 0..3 -> actuator set (= AUX pin)
  drop_payload_count: 4
```

โค้ดที่อ่านค่านี้: `ConnectionConfig.actuator_index()`
(`mavlink_adapter/commands.py`) — ตรวจ range 1..6, ห้ามซ้ำ, ต้องครอบคลุมครบ
`drop_payload_count` ตั้งแต่ตอนสร้าง config (ผิดแล้ว **ไม่ขึ้นบิน**).
ถ้าปล่อยว่าง (`[]`) จะกลับไปใช้สูตรเดิม `drop_servo_channel + payload_id`

## 3. พารามิเตอร์บนบอร์ดจริง (ตั้งใน QGC → Actuators, props off)

**ไม่เปลี่ยนจากเดิม** — pin ตรงกับ set หนึ่งต่อหนึ่ง:

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
`gcs/kmutnb_field.yaml` (`released_us/held_us`)

⚠ **อ่าน `PWM_AUX_FUNC1..4` จากบอร์ดก่อนเป็นอย่างแรก อย่าเพิ่งเชื่อว่าเป็น 301..304**
บอร์ด FMU v6X ตัวจริงของทีม (อ่านสด 2026-08-15 โดยเซสชัน `mission-aavc-sitl-setup`)
ขึ้นมาเป็น **`402, 405, 409, 410` = RC passthrough** (RC_Roll, RC_Yaw, RC_AUX3,
RC_AUX4) พร้อม `RC_MAP_AUX1..6 = 0` ทั้งหมด แปลว่า **AUX1/AUX2 ผูกกับสติ๊ก
roll/yaw** (บินจริงแล้วสลักจะขยับตามการเอียงเครื่อง!) ส่วน AUX3/AUX4 ไม่มีต้นทาง
และ**ไม่มีอะไรในซอฟต์แวร์สั่งสลักได้เลย** — ปุ่ม GCS / `set_actuator` /
`drop_payload` จะเงียบสนิทโดยไม่มี error ให้เห็น ถ้าไม่ได้อ่านพารามิเตอร์ชุดนี้ก่อน
จะเสียเวลาไล่หาบั๊กในโค้ดทั้งที่ปัญหาอยู่ที่ output mapping ของบอร์ด

⚠ **`PWM_AUX_MAX` แยกรายช่องได้ และอาจต้องไม่เท่ากัน** — สลักของทีม (บอร์ดจริง
2026-08-15) มีตัวหนึ่งที่ **เปิดไม่สุดที่ 2000 µs ต้องใช้ 2100** ค่าสั่งปล่อย
`+0.8` ถูกแมปเป็นสัดส่วนของช่วง `[MIN, MAX]` (PX4 map −1..+1 ลงช่วงนี้เชิงเส้น)
ดังนั้น 1900 µs = 80% ของระยะ ไม่ใช่ "สุดทาง" — **ถ้าที่ bench พบว่าสลักเปิด
ไม่พอจนไข่ไม่หลุด ให้เพิ่ม `drop_servo_pwm_release` เข้าหา MAX (เช่น 2000 = +1.0)
หรือขยาย `PWM_AUX_MAXn` ของช่องนั้น** — SITL มองไม่เห็นปัญหานี้เลย เพราะ
DetachableJoint ขาดที่มุม 0.35 rad ซึ่ง +0.8 ก็เกินไปแล้ว

## 4. ฝั่ง SITL

- `sitl/models/eft_x6100/model.sdf`: กล่อง `cargo_payload_N` ถูกจัดตำแหน่งใหม่
  (2026-08-15) ให้ **N = เลข AUX − 1** คือกล่องอยู่มุมเดียวกับที่ AUX นั้น
  คุมจริง (ชุดพิกัด 4 มุมเท่าเดิม → มวล/โมเมนต์ความเฉื่อยไม่เปลี่ยน)
- `sitl/payload_detach_bridge.py --servo`: ดู gz servo topic ตรง ๆ → ถูกอยู่แล้ว
- โหมด audit ต้องส่ง `--channels 4,1,2,3` (Makefile ตัวแปร `CHANNELS` ใส่ให้แล้ว
  ทั้ง `make payload-bridge` และ `make payload-bridge-servo`) มิฉะนั้นจะหล่นผิดมุม

## 4.1 หลักฐาน — ยืนยันโซ่ actuator set → servo topic → มุมจริง (2026-08-15)

**สมมติฐานร่วมที่เสี่ยงที่สุดของทั้งเรื่องนี้** คือ actuator set `n` → output
function `300+n` → gz slot `n−1` → topic `servo_{n-1}` ยืนยันแล้ว 2 ชั้น:

**(ก) จาก source** (PX4 v1.17 worktree `~/PX4-Autopilot-v1.17`, อ่านตรงในเซสชันนี้):
`GZMixingInterfaceServo.hpp:87` ประกาศ `MixingOutput{"SIM_GZ_SV", MAX_ACTUATORS, …}`
→ slot `i` ผูกกับ `SIM_GZ_SV_FUNC{i+1}`; `.cpp:108-113` วน `i = 0..7` advertise
`/model/<m>/servo_<i>` เรียงตาม slot ตรง ๆ; `.cpp:139-158` publish `outputs[i]`
ทีละ slot ตาม `isFunctionSet(i)`

**(ข) จากไฟลท์จริงใน SITL** — รันโดยเซสชันคู่ขนาน `mission-aavc-sitl-setup`
บน stack **`mission_AAVC`** (คนละ repo กับนี้ แต่ใช้ PX4 worktree + airframe
22000 + โมเดล `eft_x6100` ตัวเดียวกัน; world `test_field`, headless,
`--ids 1 --test-world`, LAND err 0.10 m):

```
servo_detach_bridge: AUX 4 RELEASE (angle +0.63 rad) → detach_payload_0
[    35.1s] DELIVERY START   pad=1 payload=0 aux=4 aim_offset=[0.1, 0.035]
[    80.9s] DELIVERY RELEASE pad=1 payload=0 aux=4 err=0.1 lat=13.7303948 lon=100.7875572
```

snapshot จาก `/world/test_field/pose/info` (pose เทียบโมเดลแม่ ทันทีหลังปล่อย —
ground truth ที่ไม่ต้องเชื่อ log ของทั้ง mission และ bridge):

| box | มุม | AUX | x | y | z | สถานะ |
|---|---|---|---|---|---|---|
| 0 | **front-left** | 4 | 0.067 | −0.027 | −2.723 | **RELEASED** |
| 1 | rear-right | 1 | −0.100 | −0.035 | 0.260 | attached |
| 2 | front-right | 2 | 0.100 | −0.035 | 0.260 | attached |
| 3 | rear-left | 3 | −0.100 | 0.035 | 0.260 | attached |

⇒ ไข่ใบแรก (`payload_id` 0) ออกทาง **AUX 4 = มุมหน้า-ซ้าย** ตามตาราง §2 และอีก
3 ใบยังนิ่งที่จุดแขวนเป๊ะ (พิสูจน์ว่า **ไม่มีใบอื่นหลุดพลอย** — NaN masking
ของ DO_SET_ACTUATOR ทำงาน)

ℹ เลข `detach_payload_N` ข้างบนเป็นของ repo `mission_AAVC` ซึ่งเรียงกล่องตาม
**ลำดับปล่อย** (box 0 = หน้า-ซ้าย) — repo นี้เรียงกล่องตาม **pin** (box 3 =
AUX 4 = หน้า-ซ้าย, §4) **มุมที่หล่นเหมือนกัน เลข box ต่างกัน — ไม่ใช่ความผิดพลาด**

**(ค) ไฟลท์ของ repo นี้เอง — ปิดช่องที่เหลือแล้ว (2026-08-15)**
`--assigned-ids 1` headless, world `kmutnb_skyfield`, profile `kmutnb_skyfield`
(เพดาน 5 m), สนามสะอาด (relaunch stack ก่อนบิน):

```
[mavlink] drop payload 0 (actuator set 4 = AUX 4)          ← flight core
[detach] SERVO 4 angle +0.63 rad: shed box 3 (detach_payload_3, 1/4 shed)
t=115.4s DELIVERY 1 RELEASE pad=1 payload=0 lat=13.8227794 lon=100.5121808
t=115.4s DELIVERY 1 END delivered=True pad=1 err=0.06m landed=True
```

`tools/box_truth.py` (gz `/world/kmutnb_skyfield/pose/info`) หลังบินจบ:

| box | มุม | AUX | ไข่ใบที่ | x | y | z | สถานะ |
|---|---|---|---|---|---|---|---|
| 0 | rear-right | 1 | 1 | −0.100 | −0.035 | 0.260 | attached |
| 1 | front-right | 2 | 2 | 0.100 | −0.035 | 0.260 | attached |
| 2 | rear-left | 3 | 3 | −0.100 | 0.035 | 0.260 | attached |
| 3 | **front-left** | 4 | **0** | −17.922 | 27.737 | 0.052 | ***RELEASED*** |

(กล่องที่หลุดรายงานเป็นพิกัดโลก = นอนอยู่บนแพดกลางสนาม ไม่ใช่จุดแขวนแล้ว)
`tools/verify_flight.py`: **PASS 10 checks / 0 warnings**, release 0.05 m จาก
ศูนย์กลางแพด id ถูกใบ, transit ครบทั้งไปกลับ, ลง L&R ห่าง 1.7 m, 155 s

## 4.2 ⚠ กับดักที่เจอระหว่างบินตรวจ: bridge "ปล่อย" ทั้งที่กล่องไม่หลุด

ไฟลท์ตรวจ**รอบแรก** log ทุกอย่างถูกหมด — flight core สั่ง AUX 4, bridge พิมพ์
`shed box 3` — แต่ `box_truth` บอกว่า **ไม่มีกล่องไหนหลุดเลย** สาเหตุ: publisher
ของ bridge ไม่ผ่าน **discovery** ของ gz-transport (ตอน bridge สตาร์ท มัลติแคสต์
ของเครื่องล่ม: `Exception sending a multicast message: Network is unreachable`)
ฝั่ง **subscribe ยังทำงานปกติ** (bridge เห็น servo topic จึงพิมพ์ shed ได้) และ
`publish()` คืนค่า `True` เสมอแม้ไม่มีใครรับ → ไม่มีใครรู้ว่ากล่องไม่หลุด
`gz topic -i -t /model/eft_x6100/detach_payload_3` ตอนนั้นขึ้น
`No publishers` ทั้งที่ bridge ยังรันอยู่ พอ restart bridge ตัวเดียว
publisher ก็กลับมาปกติและโซ่ทำงานครบทันที

แก้แล้ว (`sitl/payload_detach_bridge.py`): ใช้ `Publisher.has_connections()` —
เช็คตอนสตาร์ท (เตือนต่อช่องถ้าไม่มี subscriber) และเช็คซ้ำตอนปล่อยจริง
(ต่อท้ายบรรทัด shed ด้วย `⚠ NO SUBSCRIBER — box did NOT fall`) เพราะ `valid()`
บอกแค่ว่า advertise สำเร็จในเครื่องตัวเอง ไม่ได้บอกว่าถึงปลายทาง

**ข้อควรจำ:** นี่คือปัญหา **SITL Tier-2 (ภาพกล่องหล่น) เท่านั้น** — flight core,
PX4, servo และ audit ถูกต้องหมดทั้งรอบแรก ของจริงบนเครื่องบินไม่มี gz อยู่ในเส้นทาง
แต่มันเตือนเราว่า **อย่าใช้ log ของ bridge เป็นหลักฐานว่ากล่องหลุด** ให้ใช้
`tools/box_truth.py` ซึ่งอ่าน pose จากซิมโดยตรง

## 4.3 ✅ CLOSED: กล่องตกบนแพดจริง — 0.02 m (2026-08-16)

เกณฑ์จากผู้ใช้ (2026-08-15): **"err ห้ามเกินขอบวงกลมของ ArUco"** ⇒ ไข่ต้องตก
ในวงแหวนดำ ⌀750 mm = ห่างจากกลางแพดไม่เกิน **0.375 m** (ถ้าเอาแบบทั้งกล่องต้องอยู่
ในวง: 0.375 − ครึ่งเส้นทแยงหน้าตัดกล่อง 0.16×0.07 m = **0.288 m**)

⚠ **เกณฑ์นี้วัดคนละอย่างกับเลข `err` ที่รายงานกันมาตลอด** — `err` คือระยะที่
*โดรน* ห่างจากกลางแพดตอนแตะพื้น ส่วนเกณฑ์วัดที่ *กล่อง* ไปตกตรงไหน ต่างกันเพราะ
กล่องแขวนห่างจาก CG (±0.10, ±0.035) และกลิ้งต่อหลังตก

**ผลวัด** (ไฟลท์ 2026-08-16, ลม 10 m/s, `--assigned-ids 1`):

| | ENU | ห่างกลางแพด id 1 |
|---|---|---|
| จุดที่ audit บันทึกว่าปล่อย | (−10.44, 31.48) | 0.13 m |
| กล่อง `cargo_payload_3` ที่ตกจริง | (−10.38, 31.56) | **0.023 m** ✅ |

⇒ **ทั้งกล่องอยู่ในวงแหวน** (เกณฑ์เข้มสุด 0.288 m ก็ยังผ่านสบาย ๆ) ผ่านทั้งที่บิน
ในลม 10 m/s

### กับดักที่ทำให้ตอนแรกอ่านผิดไป 7.35 เมตร

รอบแรกอ่านได้ว่ากล่องอยู่ที่ (−17.01, 28.17) = **ห่างจุดปล่อย 7.35 m** และเกือบ
สรุปว่า "หลักฐานกล่องตกบนแพดจาก SITL เชื่อไม่ได้" ซึ่ง**ตรงข้ามกับความจริง**

สาเหตุคือ `/world/<w>/pose/info` ของ gz ส่ง pose ของ **nested model เทียบกับ
parent** ไม่ใช่ world frame — และกล่องยัง nested อยู่ใต้ตัวเครื่องแม้ joint ขาดแล้ว
เลขที่อ่านได้จึงเป็น "เวกเตอร์จากตัวเครื่องไปหากล่อง (ในกรอบตัวเครื่อง)" ต้อง
compose ผ่าน pose + yaw ของตัวเครื่องก่อนถึงจะเป็นพิกัดโลก

**ตัวบอกว่าอ่านผิดกรอบมีอยู่ในตารางเดิมแล้ว**: กล่องที่ยังไม่หลุดรายงานพิกัด
(±0.100, ±0.035, 0.260) เป๊ะ ๆ ซึ่งคือ *mount pose* — ถ้านั่นเป็นพิกัดโลกจริง
กล่องต้องลอยอยู่ที่จุดกำเนิดโลก ไม่ใช่ใต้ตัวเครื่องที่ (−0.52, 0.16)

แก้แล้วใน `tools/box_truth.py` (`compose()` + บรรทัด "where it landed" ที่พิมพ์
พิกัดโลก + ระยะถึงแพดที่ใกล้ที่สุด + คำตัดสินวงแหวน) ล็อกด้วย
`tests/test_box_truth.py` ซึ่ง pin เลขจริงของไฟลท์นี้ไว้ทั้งค่าที่ถูก **และ**
ค่าที่ผิด (กันคนมา "ทำให้ง่ายขึ้น" แล้วถอด compose ออก)

**บทเรียน:** เครื่องมือ audit ที่ตั้งใจไว้ว่า "ไม่แชร์สมมติฐานกับสิ่งที่มันตรวจ"
ยังพังได้ด้วยสมมติฐานของตัวเอง (frame) — ตัวเลขที่ดู "พังอย่างมีเหตุผล" ควรถูก
สงสัยพอ ๆ กับตัวเลขที่ดูดีเกินจริง

## 5. ข้อควรรู้จากการทดสอบ SITL (2026-08-12)

1. **ตอน DISARMED ช่อง servo ถูกบังคับเป็นค่า disarmed (ล็อก) เสมอ** —
   `DO_SET_ACTUATOR` มีผลเฉพาะตอน ARMED. ระหว่าง mission ไม่กระทบ (เครื่อง armed
   ตลอดรวมถึงตอนเกาะ pad เพราะ `COM_DISARM_LAND = -1`)
   ✅ **แต่ไม่ต้อง arm เพื่อเทสสลักบนโต๊ะ** (แก้คำแนะนำเดิม 2026-08-15):
   `MAV_CMD_ACTUATOR_TEST` (310) เป็น**ภาพสะท้อน**ของ `DO_SET_ACTUATOR` —
   ยืนยันจาก v1.17 source `Commander.cpp::handleCommandActuatorTest`:
   ```c
   if (isArmed() || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) return DENIED;
   if (_param_com_mot_test_en.get() != 1) return DENIED;
   ```
   คือรับเฉพาะตอน **DISARMED** (+ ต้อง `COM_MOT_TEST_EN=1` และปลด safety switch)
   ⇒ **เทสสลักได้โดยไม่ต้อง arm และไม่ต้องถอดใบพัด** ซึ่งปลอดภัยกว่ามาก
   พารามิเตอร์: `param5 = 1000 + output function` (สลัก = **1301..1304**),
   `param1 = ค่า −1..+1`, `param2 = timeout วินาที` (≤0 = คืนการควบคุม)
   MAVSDK ไม่มีคำสั่งนี้ ต้องส่งด้วย pymavlink (เหมือน `_drop_via_set_actuator`)
2. pymavlink รุ่นใหม่ (≥2.4.49) ไม่มีชื่อ `MAV_CMD_DO_SET_ACTUATOR` ใน enum
   แล้ว — โค้ดทุกจุดใช้ `getattr(mavutil.mavlink, "MAV_CMD_DO_SET_ACTUATOR",
   187)` (ทำแล้วทั้งใน repo นี้และ aavc-gcs)
3. ใน SITL เส้นทางเดียวกันนี้ผ่านการทดสอบครบ: MAVSDK `set_actuator` /
   ปุ่ม GCS → PX4 → gz topic (มุม +0.63 rad = ปล่อย) →
   `payload_detach_bridge --servo` → กล่องหลุดจริง 4/4

## 6. Bench checklist ตอนเสียบจริง (props off)

0. **อ่าน `PWM_AUX_FUNC1..4` ก่อนแตะอะไรทั้งสิ้น** (`param show PWM_AUX_FUNC*`
   หรือ QGC → Parameters) — บอร์ดจริงของทีมขึ้นมาเป็น RC passthrough
   402/405/409/410 ไม่ใช่ 301..304 (ดูคำเตือนข้อ 3) ถ้าไม่ใช่ 301..304
   **ยังไม่มีอะไรสั่งสลักได้ และจะไม่มี error ให้เห็น**
1. เช็คสายตามตารางข้อ 1 — **ซ้ายบน=AUX4, ขวาบน=AUX2, ซ้ายล่าง=AUX3,
   ขวาล่าง=AUX1** (ถ้าสายไม่ตรงนี้ ต้องแก้ `drop_servo_channels` ใหม่
   ไม่ใช่ย้ายสาย)
2. ตั้งพารามิเตอร์ตามบล็อกข้อ 3 ใน QGC แล้ว reboot FC → **อ่านกลับยืนยันทุกตัว**
3. **DISARMED** (ไม่ต้องถอดใบพัด): ตั้ง `COM_MOT_TEST_EN=1`, ปลด safety switch
   → ยิง `MAV_CMD_ACTUATOR_TEST` ทีละช่อง (`param5=1301..1304`) →
   **ช่อง N ต้องทำให้ servo ที่ AUX N ขยับ** และตัวอื่น **ไม่ขยับ**
   → เทียบกับตารางข้อ 1 ว่ามุมตรงป้ายจริง (นี่คือขั้นเดียวที่ยืนยันได้ว่า
   **สายเส้นไหนไปมุมไหน** — SITL และ unit test ยืนยันได้แค่ตรรกะซอฟต์แวร์)
   ⚠ อย่าลืมคืน `COM_MOT_TEST_EN=0` ก่อนบิน
4. **สลักต้องเปิด "สุด" พอที่ไข่จะหลุดจริง** ไม่ใช่แค่ขยับ — ถ้าไม่สุดที่ 1900 µs
   ให้เพิ่ม `drop_servo_pwm_release` หรือ `PWM_AUX_MAXn` ของช่องนั้น (ดูข้อ 3)
5. ARMED (props off) ทวนซ้ำด้วยเส้นทางจริง: กดปุ่ม "ปล่อย servo" บน AAVC GCS
   (`make aavc-gcs`) = `DO_SET_ACTUATOR` → ผลต้องเหมือนข้อ 3 ทุกช่อง
   (ข้อ 3 เทสสายกับกลไก ข้อนี้เทสเส้นทางคำสั่งที่ใช้บินจริง)
6. ตรวจลำดับ mission: ปล่อยไข่ใบแรกต้องเป็นมุม **หน้า-ซ้าย (AUX 4)**
   ใบที่สอง **ท้าย-ขวา (AUX 1)** — ดูใน audit `DELIVERY k RELEASE payload=…`
   คู่กับล็อกของ `[mavlink] drop payload N (actuator set M = AUX M)`
7. ทดสอบ failsafe: disarm → servo ทุกตัวต้องกลับ/ค้างที่ 1100 (ล็อก)
