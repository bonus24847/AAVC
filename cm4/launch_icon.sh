#!/usr/bin/env bash
# ตัวกลางของไอคอน AAVCGCS PRACTICE/COMP — ล้าง state เก่าก่อนเสมอ (คำสั่ง operator
# 2026-08-19) แล้วเปิด console โหมดเครื่องจริงของ mission ที่ระบุ:
#
#     bash cm4/launch_icon.sh kmutnb    # ไอคอน PRACTICE
#     bash cm4/launch_icon.sh aavc      # ไอคอน COMP (ใน repo aavc-comp)
#
# ทำไมต้องมีไฟล์นี้ (2026-08-20): บรรทัด Exec= เดิมของสองไอคอนใช้ `bash -c '…'`
# ครอบด้วย single quote ซึ่งสเปค desktop-entry ไม่รู้จัก (Exec รู้จักแต่ double
# quote + backslash) — GNOME จึงตัดคำผิด แล้ว bash ตายเงียบตั้งแต่ยังไม่ทันรัน
# clear_state: ไอคอนถูก "เปิดแล้ว" แต่ไม่มีอะไรเกิดขึ้นเลยบนจอ. ย้ายทั้งลำดับมา
# ไว้ในสคริปต์จริง ให้ Exec เหลือรูปแบบเดียวกับไอคอน AAVC-GCS-REAL ที่พิสูจน์
# แล้วว่าใช้ได้:  bash "<repo>/cm4/launch_icon.sh" <mission>
# (ไฟล์นี้เหมือนกัน byte ต่อ byte ทั้งสอง repo — REPO_ROOT หาเองจากตำแหน่งไฟล์)
set -u
MISSION="${1:?usage: launch_icon.sh <mission เช่น kmutnb|aavc>}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO_ROOT/clear_state.sh"

# ── ข้ามกล่อง "ที่อยู่ CM4" เมื่อรู้คำตอบอยู่แล้ว ────────────────────────────
# launch_gcs_real_gui.sh สแกนหา CM4 เจอเองอยู่แล้ว (10.42.0.1 ตอบใน ~30 ms) และ
# เขียนค่าที่ใช้ลง $HOSTFILE ทุกครั้ง แต่มันยัง **เปิด zenity --entry ถามเสมอ**
# เพราะไม่มีใครตั้ง AAVC_HOST ให้ ⇒ ตัวเปิด block รอคนกด OK ทั้งที่ไม่มีอะไรให้
# ตัดสินใจ (2026-08-25: กดไอคอน 14:18:53 → console ขึ้น 14:29:39 = 10 นาที 46 วิ
# ซึ่งเกือบทั้งหมดคือเวลารออยู่หน้ากล่องคำถาม ไม่ใช่เวลาเครื่องทำงาน)
#
# ตั้งให้เฉพาะเมื่อ **พิสูจน์แล้วว่าโฮสต์ที่จำไว้ตอบพอร์ต 22 จริง** — ผิดวง/ไม่ตอบ
# ก็ปล่อยให้ถามเหมือนเดิม เพราะนั่นคือกรณีที่ต้องให้คนดู. เรายังไม่ได้ตัด
# "คนรู้ว่ากำลังสั่งลำไหน" ทิ้ง: หน้าต่างสรุปตอนท้ายของ launch_gcs_real_gui.sh
# ยังพิมพ์ user@ip ที่ใช้จริง และปุ่ม 🚀 ยังเป็นการกดแยกอยู่เหมือนเดิม
HOSTFILE="$HOME/.config/aavc/cm4_host"
if [ -z "${AAVC_HOST:-}" ] && [ -s "$HOSTFILE" ]; then
    LAST="$(cat "$HOSTFILE")"
    LAST_IP="${LAST#*@}"
    # 3 วิ ไม่ใช่ 1 — ARP ที่ยังไม่มีในตารางใช้เวลาเกิน 1 วิได้ (เหตุผลเดียวกับ
    # ที่ launch_gcs_real_gui.sh ใช้ 3 วิในลูปสแกนของมัน)
    if [ "$LAST" != "$LAST_IP" ] \
       && timeout 3 bash -c "echo > /dev/tcp/$LAST_IP/22" 2>/dev/null; then
        export AAVC_HOST="$LAST"
    fi
fi

exec env AAVC_MODE=real AAVC_MISSION="$MISSION" \
     bash "$REPO_ROOT/cm4/launch_gcs_real_gui.sh"
