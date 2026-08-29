# Landing test — สนามฟุตบอล ร.ร.บางบ่อวิทยาคม, คืน 29 ส.ค. 2569 (ไฟสนาม)

**จุดประสงค์:** พิสูจน์สายโซ่ลงจอดใหม่บนเครื่องจริงก่อนเที่ยวคะแนนสุดท้าย — lidar ladder (8/5/3/2 m),
ground-contact guard, goto แนวดิ่งที่ rung ต่ำ, projection ด้วยความสูง lidar, gate "วางไม่ตรงดีกว่าไม่วาง",
ปล่อยไข่หลัง touchdown — **ด้วยแพ็คขนาน 17000+15000 และ hover seed 0.65 เหมือนวันแข่ง**

**Envelope:** profile `kmutnb_skyfield` (เพดาน 10 m · transit 9 m · sweep 8 m · floor 2.5) บน
`sitl/bangbo_config.yaml` (deploy บน CM4ผ่าน MD5 แล้ว) · แผนที่ console `gcs/bangbo_field.yaml`
(mission `bangbo` ใน missions.yaml, tile z15–19 โหลดไว้แล้ว)

**สนาม (จากภาพดาวเทียม ±3–5 m — docs/evidence/bangbo_field_proposal_2026-08-29.png):**
L&R = 13.5785879, 100.8578589 (มุม SW ห่างอัฒจันทร์ ~8 m) · รั้ว = สนามดินเว้นขอบ 8 m (ฝั่งอัฒจันทร์ 16 m)
· พื้นที่ pad = 36×40 m กลางสนามค่อนเหนือ (ENU E 4–42 / N 22–64 จาก L&R) · P1/P2/P3 = (8,13) (15,25) (23,38)

## ก่อนขึ้น (ทำแล้วคืนนี้จากแลปท็อป)
- [x] `MPC_THR_HOVER = 0.65` เขียน+อ่านกลับ · BOARD ผ่านทุกตัว · lidar-check ✔ 10 Hz
- [x] deploy `--check` MD5 MATCH (โค้ด tip-over fix + gate ผ่อนเกณฑ์ + config บางบ่อ)

## ที่สนาม
1. **ENG** ต่อแบตขนาน: วัด 17000 ___ V / 15000 ___ V (ทั้งคู่ ≥ 25.0, ต่าง ≤ 0.1 V) → สาย Y เข้า 17000 ก่อน → 15000 → XT90 เข้าเครื่อง
   · ไข่ซ้อม 1 ใบใน **AUX4** (latch ล็อก) · แพ็คที่สองยึดแน่น ไม่บัง TFmini/กล้อง/ขา
2. **OP** แลปท็อปต่อ AP `AAVC-DRONE` → เสียบวิทยุ NOMAD → เปิด console ด้วย launcher (mission **bangbo**):
   `AAVC_NONINTERACTIVE=1 AAVC_MODE=real AAVC_MISSION=bangbo AAVC_HOST=drone@10.42.0.1 bash ~/Desktop/aavc-comp/cm4/launch_gcs_real_gui.sh`
   · รอ chip กล้อง/GPS/รั้ว เขียว · **เทียบ GPS โดรนกับจุด L&R บนแผนที่** — ต่าง > 5 m บอกก่อนกด Up
3. **ENG** วาง pad พิมพ์ (id ที่จะกด เช่น 1) กลางพื้นที่เหลือง **ใต้ไฟสนามที่สว่างที่สุด** ด้านขาวหงายขึ้น ไม่มีเงาทับ
4. **OP** (ถ้ามีเวลา 3 นาที) bench decode ใต้ไฟ: ยกเครื่องเหนือ pad ~1.5–2 m → console chip กล้องต้องเห็น pad/decode
   (หรือ `tools/hover_decode.py` บน CM4 แล้วให้ PIC hover 3/5/8 m 20 วิ ต่อระดับ — beacon บอก GOOD/WEAK/DARK)
5. **OP** กด id 1 ตัว → Up (stage) → รอ `RC-GO — staged` · **PIC** kill OFF → ARM → OFFBOARD ภายใน 10 วิ
6. ระหว่างบิน (**OP อ่านออกเสียง**): `TRANSIT_PASS P1/P2/P3` → sweep 8 m (~1 นาที) → `SWEEP paused: pad=… confirmed — delivering now`
   → `rung 8/5/3/2 m … locked` → `LAND on pad` → `DELIVERY 1 RELEASE … err=…` → ไต่กลับ → egress P3→P1 → ลง L&R
   · ถ้าเห็น `ground_contact_at_rung…` / `LAND GATE relaxed` / `lost@…→climb` = ระบบใหม่ทำงาน จดเวลา
   · **PIC**: POSCTL ทันทีถ้า เอียงเพิ่มขึ้นเรื่อย ๆ ขณะมอเตอร์เบา / ออกนอกรั้ว / สูงเกิน 12 m / อะไรก็ตามที่รู้สึกผิด
7. หลังลง: วัดระยะไข่จากกลาง pad ___ m · `verify_flight.py runs/<id>/audit.jsonl --config sitl/bangbo_config.yaml`
   · ดึง ULog จาก SD · ชาร์จ**ทั้งสองก้อน**เต็มสำหรับพรุ่งนี้ · จด motor mean/hover จาก log → ถ้าไม่ใช่ ~0.65 แก้ seed

## ถ้าไม่ผ่าน
- decode ไม่ได้ใต้ไฟ → ยกเลิกภารกิจอัตโนมัติ (ไม่มี pad = ไม่มีบันได) แต่ยังทดสอบ RC-GO/transit/egress/แพ็คขนาน/hover seed ได้
- kill/takeover → orchestrator หยุดถาวร: console ขึ้น STOOD DOWN → restart ด้วย launcher ก่อน arm ใหม่
