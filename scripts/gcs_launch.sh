#!/usr/bin/env bash
# ดับเบิลคลิกไอคอน Desktop "Sys_ID GCS" -> เปิด Web Ground Station "บนแล็ปท็อปเครื่องนี้"
# แล้วเปิดเบราว์เซอร์ให้อัตโนมัติ. สคริปต์นี้จะ:
#   1) เลือกลิงก์ให้เอง:  NOMAD radio (/dev/ttyUSB0 @460800)  >  FMU USB (/dev/ttyACM0)  >  demo
#   2) สตาร์ท gcs_server บนพอร์ต 8000:
#        - ยังไม่มี server            -> สตาร์ทใหม่
#        - มี server "demo" อยู่ + เพิ่งเสียบวิทยุ/FMU -> รีสตาร์ตมาอ่านอุปกรณ์จริงให้เอง
#        - มี server อ่านอุปกรณ์จริงอยู่แล้ว          -> ไม่สตาร์ทซ้ำ แค่เปิดเบราว์เซอร์
#   3) เปิดเบราว์เซอร์ไปที่ http://localhost:8000
#
# เสียบ NOMAD (สาย USB-C ของโมดูลวิทยุ) ก่อนคลิก จะได้ telemetry จริง; ไม่เสียบ = โหมด demo.
# ตั้ง GCS_NO_BROWSER=1 เพื่อไม่เปิดเบราว์เซอร์ (ทดสอบ);  GCS_PRINT_ONLY=1 เพื่อ dry-run.
set -u
cd "$(dirname "$0")/.."
PORT=8000
URL="http://localhost:$PORT"
PY="venv/bin/python"
[ -x "$PY" ] || PY="python3"
mkdir -p logs
LOG="logs/gcs_launch.log"
log(){ echo "[gcs_launch $(date '+%F %T')] $*" >> "$LOG"; }

wait_ready() {                       # รอจน server ตอบ /api/status (สูงสุด ~10 วิ)
    for _ in $(seq 1 20); do
        curl -s -m 1 -o /dev/null "$URL/api/status" && return 0
        sleep 0.5
    done
    return 1
}
open_browser() {
    [ "${GCS_NO_BROWSER:-0}" = "1" ] && return 0
    if   command -v xdg-open      >/dev/null 2>&1; then xdg-open      "$URL" >/dev/null 2>&1 &
    elif command -v firefox       >/dev/null 2>&1; then firefox       "$URL" >/dev/null 2>&1 &
    elif command -v google-chrome >/dev/null 2>&1; then google-chrome "$URL" >/dev/null 2>&1 &
    fi
}
server_is_demo() {                   # exit 0 = server ที่รันอยู่บน :8000 เป็นโหมด demo
    curl -s -m 2 "$URL/api/status" 2>/dev/null \
        | "$PY" -c "import sys,json; sys.exit(0 if json.load(sys.stdin).get('demo') else 1)" 2>/dev/null
}

# ---- เลือกลิงก์อัตโนมัติ ----
# NOMAD/ELRS วิทยุ = Silicon Labs CP2102: ใช้ path แบบถาวร (by-id) ก่อน เพื่อให้ถอด-เสียบใหม่
# ต่อกลับได้เอง แม้ Linux เปลี่ยนเลข ttyUSBn (server retry path เดิม -> symlink ชี้ตัวใหม่ให้)
RADIO=$(ls /dev/serial/by-id/*CP2102* /dev/serial/by-id/*Silicon_Labs* 2>/dev/null | head -1)
if [ -n "${RADIO:-}" ]; then
    ARGS=(--url "$RADIO" --baud 460800); MODE="NOMAD radio (by-id @460800)"; HAVE_DEV=1
elif ls /dev/ttyUSB* >/dev/null 2>&1; then
    DEV=$(ls /dev/ttyUSB* | head -1); ARGS=(--url "$DEV" --baud 460800); MODE="NOMAD radio ($DEV @460800)"; HAVE_DEV=1
elif ls /dev/ttyACM* >/dev/null 2>&1; then
    DEV=$(ls /dev/ttyACM* | head -1); ARGS=(--url "$DEV" --baud 115200); MODE="FMU USB ($DEV)"; HAVE_DEV=1
else
    ARGS=(--demo); MODE="demo (ไม่พบวิทยุ/FMU ที่เสียบอยู่)"; HAVE_DEV=0
fi

if [ "${GCS_PRINT_ONLY:-0}" = "1" ]; then     # dry-run: โชว์ว่าจะสั่งอะไร แล้วออก (ไว้ทดสอบ)
    echo "would run: $PY src/gcs_server.py ${ARGS[*]} --port $PORT   [$MODE]"
    exit 0
fi

# ---- บังคับรีสตาร์ต (GCS_FORCE_RESTART=1): ปิดตัวเดิมแล้วสตาร์ตใหม่ด้วยโค้ด/ลิงก์ล่าสุด ----
if [ "${GCS_FORCE_RESTART:-0}" = "1" ]; then
    log "force-restart -> $MODE"
    pkill -f "gcs_server.py .*--port $PORT" 2>/dev/null
    sleep 1.5
fi

# ---- server รันอยู่แล้วบน :8000? ----
if curl -s -m 2 -o /dev/null "$URL/api/status" 2>/dev/null; then
    if [ "$HAVE_DEV" = "1" ] && server_is_demo; then
        log "server เดิมเป็น demo + เพิ่งเสียบอุปกรณ์ -> รีสตาร์ตเป็น $MODE"
        pkill -f "gcs_server.py .*--port $PORT" 2>/dev/null
        sleep 1.5
    else
        log "server รันอยู่แล้ว (อ่านอุปกรณ์จริง หรือไม่มีอุปกรณ์ให้สลับ) -> เปิดเบราว์เซอร์เฉย ๆ"
        open_browser
        exit 0
    fi
fi

# ---- สตาร์ท server (detach ให้อยู่ต่อหลัง launcher/หน้าต่างปิด) ----
log "start gcs_server -> $MODE"
setsid nohup "$PY" src/gcs_server.py "${ARGS[@]}" --port "$PORT" >> "$LOG" 2>&1 < /dev/null &
disown 2>/dev/null || true
wait_ready || log "เตือน: server ยังไม่ตอบใน ~10 วิ (ดู $LOG)"
open_browser
