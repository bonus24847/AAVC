# AAVC 2026 — Presentation v2 (image/chart-first)

สร้างเมื่อ 2026-08-27 · ทีม AeroOptix · 14 สไลด์ · เวลาเป้า ~15 นาที + Q&A 5

| ไฟล์ | ใช้เมื่อ |
|---|---|
| `AAVC2026_Presentation_v2.html` | เปิดใน Chrome/Edge/Firefox (ไม่ต้องต่อเน็ต — ฟอนต์และภาพฝังในไฟล์) **ต้องอยู่ข้างโฟลเดอร์ `media/`** เพราะวิดีโอ 2 ไฟล์โหลดจากที่นั่น |
| `AAVC2026_Presentation_v2.pptx` | PowerPoint (วิดีโอฝังในไฟล์แล้ว 23 MB) · speaker notes ภาษาไทยอยู่ใน Notes ทุกสไลด์ |
| `AAVC2026_v2_Script_4Presenters_TH.md` / `.pdf` | **สคริปต์พูด 4 คน** (P1 สไลด์ 1–3 · P2 4–6 · P3 7–10 · P4 11–14) + ตาราง Q&A ใครตอบอะไร |
| `../../AAVC_Checklist_Competition_KMITL.tex` / `.pdf` | checklist ฉบับแข่งขัน KMITL (LaTeX ทางการ 6 หน้า, highlight) — หน้า 1 อยู่ในสไลด์ 10 |
| `media/sitl_replay.mp4` | SITL replay ×5 (134 s) — กล้อง nadir ใน sim + phase/AGL/แบต + แผนที่เส้นทาง |
| `media/real_flight_2026-08-26_decoded.mp4` | กล้องจริง 26 ส.ค. เที่ยว 3 พร้อมกรอบ decode (69 s) |

## คีย์ใน HTML
`→` / `Space` ถัดไป · `←` ย้อน · `Home`/`End` · **`N` เปิด/ปิด speaker notes** (ภาษาไทย) · `F` fullscreen · คลิกครึ่งขวา/ซ้ายของจอเพื่อเลื่อน

## ลำดับสไลด์
1. Title — AeroOptix · KMUTNB · **first-time entrant** (ช่องชื่อสมาชิก + รูปทีม)
2. Mission recap — แผนที่จากกติกา (Fig. 3) + spec pad + ตัวเลขสำคัญ
3. Flight path — profile + แผนที่ KMITL จาก planner จริง + เหตุผล "เวลาก่อน"
4. Aircraft — spec table + tiles + redundancy (6 rotors, 2 packs)
5. Architecture — PX4 + MAVLink
6. Full-autonomy flowchart
7. SITL — วิดีโอ replay + เส้นทางที่บินใน sim
8. SITL analysis — กราฟความสูง / แบต / ความแม่น / A-B overlap
9. GCS — screenshot โหมด demo + feature list
10. Failsafe · checklist · redundancy
11. Problems 1–6 (team / budget / airframes)
12. Problems 7–13 (hardware / field)
13. Flying for real — วิดีโอกล้องจริง + ตัวเลข 5 วัน / 12+ เที่ยว
14. Thank you / Q&A

## ช่องรูปที่เว้นไว้ (📷 กรอบเส้นประ) — ถ่ายมาแล้วบอกได้เลย จะแปะให้ทั้ง HTML และ PPTX
| สไลด์ | รูป | ขนาดกรอบ (px บน canvas 1280×720) |
|---|---|---|
| 1 | รูปทีม + โดรน | 500 × 470 (แนวตั้ง/จัตุรัส) |
| 4 | โดรน EFT X6100 บนแผ่น พร้อมไข่ | 520 × 380 (แนวนอน) |
| 9 | ภาพ GCS ขณะบินจริง (screenshot) | 820 × 90 (แถบกว้าง — หรือเปลี่ยนเป็นภาพเต็มแทน demo) |
| 11 | โดรนลำเล็ก (item 4) / เฟรมคาร์บอน (item 5) / EFT X6100 จาก DRON (item 6) | 380 × 175 ×3 |
| 12 | NOMAD / PM02D / สาย GPS | 440 × 140 |
| 13 | ทีมที่สนาม / โดรนขณะบิน | 640 × 185 (แนวนอนกว้าง) |

ชื่อสมาชิกและอาจารย์ที่ปรึกษาในสไลด์ 1 ยังเป็น `[ชื่อ]` — ส่งรายชื่อ+บทบาทมาแล้วจะใส่ให้

## วิธีสร้างใหม่ (ถ้าแก้เนื้อหา)
สคริปต์ตัวสร้าง `build_deck.py` อยู่ในโฟลเดอร์นี้ (ต้องการ assets ที่ generate ไว้ใน scratchpad: กราฟ/ไดอะแกรม/วิดีโอ) — ใช้ python-pptx + matplotlib; ภาพ/กราฟทั้งหมด generate จาก `docs/evidence/*.jsonl`, `runs/aavc_delivery_mission/frames`, `captures/decoded_2026-08-26_flight3`, และรูปในกติกา PDF
