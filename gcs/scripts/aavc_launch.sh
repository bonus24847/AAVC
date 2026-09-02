#!/usr/bin/env bash
# ดับเบิลคลิกไอคอน Desktop "AAVC GCS" -> เปิด AAVC Ground Station บนแล็ปท็อปเครื่องนี้
# แล้วเปิดเบราว์เซอร์ให้อัตโนมัติ. pad-picker + telemetry + แผนที่ ใช้ได้ทันที
# (demo = เห็น pad ตัวอย่างบนแผนที่เลย). mission เป็นคนละตัว (สมองบนโดรน) รันเฉพาะตอนเทสบินจริง.
#
# เลือกลิงก์ให้เอง:  SITL (AAVC_SITL=1 -> udp:14550)  >  NOMAD radio  >  FMU USB  >  demo
# ตัวเลือก:  AAVC_SITL=1 ต่อ SITL · AAVC_NO_BROWSER=1 ไม่เปิดเบราว์เซอร์ · AAVC_PRINT_ONLY=1 dry-run
set -u
cd "$(dirname "$0")/.."
PORT=8010
URL="http://localhost:$PORT"
PY="venv/bin/python"; [ -x "$PY" ] || PY="python3"
mkdir -p logs
LOG="logs/aavc_launch.log"
log(){ echo "[aavc_launch $(date '+%F %T')] $*" >> "$LOG"; }

wait_ready(){ for _ in $(seq 1 20); do curl -s -m 1 -o /dev/null "$URL/api/status" && return 0; sleep 0.5; done; return 1; }
open_browser(){
    [ "${AAVC_NO_BROWSER:-0}" = "1" ] && return 0
    if   command -v xdg-open      >/dev/null 2>&1; then xdg-open      "$URL" >/dev/null 2>&1 &
    elif command -v firefox       >/dev/null 2>&1; then firefox       "$URL" >/dev/null 2>&1 &
    elif command -v google-chrome >/dev/null 2>&1; then google-chrome "$URL" >/dev/null 2>&1 &
    fi
}
server_is_demo(){ curl -s -m 2 "$URL/api/status" 2>/dev/null \
    | "$PY" -c "import sys,json;sys.exit(0 if json.load(sys.stdin).get('demo') else 1)" 2>/dev/null; }

# ---- เลือกลิงก์อัตโนมัติ ----
if [ "${AAVC_SITL:-0}" = "1" ]; then
    ARGS=(--url udpin:0.0.0.0:14550); MODE="SITL (udp:14550)"; HAVE_DEV=1
else
    RADIO=$(ls /dev/serial/by-id/*CP2102* /dev/serial/by-id/*Silicon_Labs* 2>/dev/null | head -1)
    if [ -n "${RADIO:-}" ]; then
        ARGS=(--url "$RADIO" --baud 460800); MODE="NOMAD radio (by-id @460800)"; HAVE_DEV=1
    elif ls /dev/ttyUSB* >/dev/null 2>&1; then
        DEV=$(ls /dev/ttyUSB* | head -1); ARGS=(--url "$DEV" --baud 460800); MODE="radio ($DEV)"; HAVE_DEV=1
    elif ls /dev/ttyACM* >/dev/null 2>&1; then
        DEV=$(ls /dev/ttyACM* | head -1); ARGS=(--url "$DEV" --baud 115200); MODE="FMU USB ($DEV)"; HAVE_DEV=1
    else
        ARGS=(--demo); MODE="demo (ไม่พบวิทยุ/FMU/SITL)"; HAVE_DEV=0
    fi
fi

if [ "${AAVC_PRINT_ONLY:-0}" = "1" ]; then
    echo "would run: $PY src/aavc_gcs.py ${ARGS[*]} --port $PORT   [$MODE]"; exit 0
fi
if [ "${AAVC_FORCE_RESTART:-0}" = "1" ]; then
    pkill -f "aavc_gcs.py .*--port $PORT" 2>/dev/null; sleep 1.5
fi

# ---- server รันอยู่แล้วบน :8010? ----
if curl -s -m 2 -o /dev/null "$URL/api/status" 2>/dev/null; then
    # server ที่รันอยู่ "แก่กว่าโค้ด" = หน้าเว็บค้างเวอร์ชันเก่า (เจอจริง
    # 2026-08-13: console ค้างข้ามวันเสิร์ฟ UI เก่าทั้งที่ไฟล์ใหม่แล้ว) —
    # เทียบเวลา start ของ process กับ mtime ของ src แล้วรีสตาร์ทให้เอง
    GPID=$(pgrep -f "aavc_gcs.py .*--port $PORT" | head -1)
    if [ -n "${GPID:-}" ] && [ -d "/proc/$GPID" ]; then
        PROC_START=$(stat -c %Y "/proc/$GPID" 2>/dev/null || echo 0)
        SRC_MTIME=$(stat -c %Y "src/aavc_gcs.py" 2>/dev/null || echo 0)
        if [ "$SRC_MTIME" -gt "$PROC_START" ]; then
            log "โค้ดใหม่กว่า server ที่รันอยู่ -> รีสตาร์ตให้เป็นเวอร์ชันล่าสุด"
            kill "$GPID" 2>/dev/null; sleep 1.5
        fi
    fi
    if curl -s -m 2 -o /dev/null "$URL/api/status" 2>/dev/null; then
        if [ "$HAVE_DEV" = "1" ] && server_is_demo; then
            log "demo อยู่ + เพิ่งเลือกลิงก์จริง -> รีสตาร์ต $MODE"
            pkill -f "aavc_gcs.py .*--port $PORT" 2>/dev/null; sleep 1.5
        else
            log "server รันอยู่แล้ว (เวอร์ชันล่าสุด) -> เปิดเบราว์เซอร์เฉย ๆ"; open_browser; exit 0
        fi
    fi
fi

# ---- สตาร์ท server (detach ให้อยู่ต่อหลัง launcher ปิด) ----
log "start aavc_gcs -> $MODE"
setsid nohup "$PY" src/aavc_gcs.py "${ARGS[@]}" --port "$PORT" >> "$LOG" 2>&1 < /dev/null &
disown 2>/dev/null || true
wait_ready || log "เตือน: server ยังไม่ตอบใน ~10 วิ (ดู $LOG)"
open_browser
