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
exec env AAVC_MODE=real AAVC_MISSION="$MISSION" \
     bash "$REPO_ROOT/cm4/launch_gcs_real_gui.sh"
