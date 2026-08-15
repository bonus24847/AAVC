#!/usr/bin/env bash
# เลือก "ทางเข้า telemetry" ให้ console เครื่องจริง — sourced โดย
# cm4/launch_gcs_real.sh และ cm4/launch_gcs_real_gui.sh (จุดเดียว ทั้งสอง
# ไอคอนจึงเลือกเหมือนกันเสมอ)
#
# ทำไมต้องมี (operator 2026-08-15): console เครื่องจริงเคย hardcode
# `udpin:0.0.0.0:14550` ซึ่งบังคับให้ต้องมี mavlink-router (บน CM4 หรือบน
# แล็ปท็อป) หรือ Nomad backpack ที่พ่น UDP ออกมา — แต่ที่สนามจริง แล็ปท็อป
# เสียบ **วิทยุ NOMAD ผ่าน USB** ตรง ๆ ซึ่ง aavc_gcs.py อ่านเป็น serial ได้อยู่แล้ว
# (`--url <dev> --baud 460800` แบบเดียวกับไอคอน "AAVC GCS" ของ aavc-gcs)
#
# ลำดับการเลือก:
#   1. AAVC_URL (+ AAVC_BAUD)      ← สั่งเองชนะทุกอย่าง
#   2. วิทยุ by-id (CP2102 / Silicon_Labs) @ 460800
#   3. /dev/ttyUSB* ตัวแรก @ 460800   ← วิทยุที่ by-id ไม่ตรงแพตเทิร์น
#   4. udpin:0.0.0.0:14550            ← ของเดิม: mavlink-router / backpack
#
# จงใจ **ไม่** fall back ไป /dev/ttyACM* (สาย USB ของ FC เอง): บน console
# "เครื่องจริง" การหยิบ FC ที่วางเทสอยู่บนโต๊ะมาแสดงแทนโดรนที่กำลังจะบิน
# อันตรายกว่าการไม่เจอลิงก์แล้วบอกตรง ๆ — ถ้าตั้งใจจะใช้ FC USB ให้ส่ง
# AAVC_URL=/dev/ttyACM0 AAVC_BAUD=115200 มาเอง
#
# ตั้งค่าให้ผู้เรียก: LINK_ARGS (array ส่งต่อ aavc_gcs.py) + LINK_DESC (ข้อความ)
#
# หมายเหตุ: ปุ่ม 🚀 กับตัวดึง captures (status_sync) วิ่งบน **ssh/WiFi ไป CM4**
# คนละเส้นกับ telemetry — เลือกวิทยุแล้วก็ยังต้องมีเครือข่ายถึง CM4 ถ้าจะสั่งบิน

pick_telemetry_link() {
    LINK_ARGS=(); LINK_DESC=""
    if [ -n "${AAVC_URL:-}" ]; then
        LINK_ARGS=(--url "$AAVC_URL")
        [ -n "${AAVC_BAUD:-}" ] && LINK_ARGS+=(--baud "$AAVC_BAUD")
        LINK_DESC="$AAVC_URL${AAVC_BAUD:+ @$AAVC_BAUD} (สั่งเองผ่าน AAVC_URL)"
        return 0
    fi
    local baud="${AAVC_BAUD:-460800}" dev=""
    dev=$(ls /dev/serial/by-id/*CP2102* /dev/serial/by-id/*Silicon_Labs* 2>/dev/null | head -1)
    if [ -z "$dev" ] && ls /dev/ttyUSB* >/dev/null 2>&1; then
        dev=$(ls /dev/ttyUSB* | head -1)
    fi
    if [ -n "$dev" ]; then
        LINK_ARGS=(--url "$dev" --baud "$baud")
        LINK_DESC="วิทยุ NOMAD — $(basename "$dev") @${baud}"
        return 0
    fi
    LINK_ARGS=(--url "udpin:0.0.0.0:14550")
    LINK_DESC="udp 14550 (mavlink-router / Nomad backpack — ไม่พบวิทยุที่เสียบ USB)"
}
