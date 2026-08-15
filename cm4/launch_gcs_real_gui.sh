#!/usr/bin/env bash
# ไอคอน "AAVC GCS เครื่องจริง" — ทำทุกอย่างด้วยการคลิก ไม่ต้องแตะ terminal
# (operator 2026-08-15: "ผม deploy ขึ้น cm4 ไม่เป็น" + "ไม่อยากเข้า terminal").
#
# ขั้นตอนที่สคริปต์นี้ทำให้ทั้งหมด:
#   1. ถามว่าจะบิน mission ไหน (อ่านจาก aavc-gcs/missions.yaml บล็อก `real:`)
#   2. หา/ถามที่อยู่ CM4 (สแกน <วงปัจจุบัน>.41 ให้ก่อน, จำค่าล่าสุด)
#   3. เช็ก ssh + เช็กว่า repo ของ mission นั้นอยู่บน CM4 แล้วหรือยัง
#      ถ้ายัง → กด "อัพขึ้นโดรน" แล้วมันรัน cm4/deploy.sh ให้พร้อมแถบความคืบหน้า
#   4. ปิด console เครื่องจริงตัวเก่า (ถ้ามี) — การสลับ mission จึงเป็นแค่
#      "เปิดไอคอนใหม่แล้วเลือกอีก mission" ไม่ต้อง Ctrl-C เอง
#   5. เปิด console + status_sync แบบ background (log ลงไฟล์) แล้วเปิดเบราว์เซอร์
#
# ทดสอบอัตโนมัติได้ด้วย AAVC_NONINTERACTIVE=1 + AAVC_MISSION/AAVC_HOST
# (ข้ามทุก dialog — ใช้ตอน CI/ไม่มี CM4 จริง)
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GCS="${AAVC_GCS:-$HOME/Desktop/aavc-gcs/src/aavc_gcs.py}"
REGISTRY="${AAVC_MISSIONS:-$HOME/Desktop/aavc-gcs/missions.yaml}"
STATE="$HOME/.config/aavc"
HOSTFILE="$STATE/cm4_host"
PIDFILE="$STATE/real_console.pids"
LOG="$STATE/real_console.log"
PORT="${AAVC_PORT:-8000}"
CM4_USER="${CM4_USER:-drone}"
CM4_OCTET="${CM4_OCTET:-41}"
CM4_KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"
SSH_ID=(); SSH_ID_STR=""
[ -f "$CM4_KEY" ] && { SSH_ID=(-i "$CM4_KEY"); SSH_ID_STR="-i $CM4_KEY "; }
NONINT="${AAVC_NONINTERACTIVE:-0}"
mkdir -p "$STATE"

msg()  { [ "$NONINT" = 1 ] && { echo "[info] $1"; return; }
         zenity --info --width=430 --title="AAVC GCS เครื่องจริง" --text="$1" 2>/dev/null; }
die()  { [ "$NONINT" = 1 ] && { echo "[error] $1" >&2; exit 1; }
         zenity --error --width=430 --title="AAVC GCS เครื่องจริง" --text="$1" 2>/dev/null; exit 1; }
ask()  { [ "$NONINT" = 1 ] && return 0
         zenity --question --width=470 --title="AAVC GCS เครื่องจริง" --text="$1" \
                --ok-label="${2:-ตกลง}" --cancel-label="${3:-ยกเลิก}" 2>/dev/null; }

# ── 1. mission ที่จะบิน ────────────────────────────────────────────────────
[ -f "$REGISTRY" ] || die "ไม่พบรายการ mission ที่\n$REGISTRY"
mapfile -t ROWS < <(/usr/bin/python3 - "$REGISTRY" <<'PY'
import sys, yaml
doc = yaml.safe_load(open(sys.argv[1])) or {}
for name, m in (doc.get("missions") or {}).items():
    r = (m or {}).get("real") or {}
    if r.get("repo") and r.get("dir") and r.get("entry"):
        print("\t".join([name, str(m.get("label") or name),
                         r["repo"], r["dir"], r["entry"],
                         str(m.get("field") or ""), str(m.get("captures") or "")]))
PY
)
[ ${#ROWS[@]} -gt 0 ] || die "ยังไม่มี mission ที่ตั้งค่าสำหรับเครื่องจริงใน\n$REGISTRY"

pick_row() {
    local want="$1"
    for r in "${ROWS[@]}"; do [ "${r%%$'\t'*}" = "$want" ] && { echo "$r"; return; }; done
}
if [ -n "${AAVC_MISSION:-}" ]; then
    ROW="$(pick_row "$AAVC_MISSION")"
elif [ ${#ROWS[@]} -eq 1 ]; then
    ROW="${ROWS[0]}"
else
    LIST=(); for r in "${ROWS[@]}"; do
        IFS=$'\t' read -r n l _ <<<"$r"; LIST+=(FALSE "$n" "$l"); done
    LIST[0]=TRUE
    SEL=$(zenity --list --radiolist --width=520 --height=260 \
          --title="AAVC GCS เครื่องจริง" --text="จะบิน mission ไหนบนโดรน?" \
          --column="" --column="ชื่อ" --column="สนาม" "${LIST[@]}" 2>/dev/null) || exit 0
    ROW="$(pick_row "$SEL")"
fi
[ -n "${ROW:-}" ] || die "ไม่รู้จัก mission นี้"
IFS=$'\t' read -r M_NAME M_LABEL M_REPO M_DIR M_ENTRY M_FIELD M_CAPT <<<"$ROW"
[ -d "$M_REPO" ] || die "ไม่พบโฟลเดอร์ repo ของ mission นี้:\n$M_REPO"

# ── 2. ที่อยู่ CM4 ─────────────────────────────────────────────────────────
GUESS=""
for net in $(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}'); do
    base="${net%/*}"; base="${base%.*}"; cand="$base.$CM4_OCTET"
    timeout 1 bash -c "echo > /dev/tcp/$cand/22" 2>/dev/null && { GUESS="$CM4_USER@$cand"; break; }
done
LAST="$(cat "$HOSTFILE" 2>/dev/null || true)"
DEFAULT="${GUESS:-${LAST:-$CM4_USER@192.168.x.$CM4_OCTET}}"
if [ -n "${AAVC_HOST:-}" ]; then
    HOST="$AAVC_HOST"
else
    HINT=$([ -n "$GUESS" ] && echo "พบ CM4 ในวงนี้แล้ว ✔" \
           || echo "ยังไม่พบ CM4 ในวง WiFi นี้ — เช็กว่าโดรนเปิดและต่อ WiFi วงเดียวกัน")
    HOST=$(zenity --entry --width=470 --title="AAVC GCS เครื่องจริง — $M_LABEL" \
      --text="ที่อยู่ CM4 บนโดรน (user@ip)\n\n$HINT" --entry-text="$DEFAULT" 2>/dev/null) || exit 0
fi
HOST="${HOST// /}"
[[ "$HOST" == *"@"* ]] || die "รูปแบบต้องเป็น <user>@<ip>\nเช่น $CM4_USER@192.168.2.$CM4_OCTET"

# ── 3. ssh + repo บน CM4 ──────────────────────────────────────────────────
if ! ssh "${SSH_ID[@]}" -o ConnectTimeout=6 -o BatchMode=yes -o StrictHostKeyChecking=accept-new "$HOST" true 2>/dev/null; then
    ask "ssh ไป <b>$HOST</b> ไม่สำเร็จ (ไม่ตอบ หรือยังต้องใส่รหัสผ่าน)\n\nเช็ก: โดรนเปิดอยู่ไหม · WiFi วงเดียวกันไหม\n\nจะเปิด console ต่อไปไหม? (ปุ่ม 🚀 จะล็อกจนกว่าจะเจอ CM4)" \
        "เปิดต่อไป" "ยกเลิก" || exit 0
else
    if ! ssh "${SSH_ID[@]}" -o BatchMode=yes -o StrictHostKeyChecking=accept-new "$HOST" "[ -d ~/$M_DIR ]" 2>/dev/null; then
        ask "โดรนยังไม่มี mission <b>$M_LABEL</b> อยู่ในเครื่อง\n\nจะอัพขึ้นให้เลยไหม? (ใช้เวลา ~1-3 นาที ครั้งแรกนานสุด)" \
            "อัพขึ้นโดรนเลย" "ข้ามไปก่อน" && {
            if [ "$NONINT" = 1 ]; then
                bash "$REPO_ROOT/cm4/deploy.sh" "$HOST" --install --repo "$M_REPO" --dir "$M_DIR" \
                    2>&1 | tee "$STATE/deploy.log" || die "อัพขึ้นโดรนไม่สำเร็จ — ดู $STATE/deploy.log"
            else
                ( bash "$REPO_ROOT/cm4/deploy.sh" "$HOST" --install --repo "$M_REPO" --dir "$M_DIR" \
                    > "$STATE/deploy.log" 2>&1; echo $? > "$STATE/deploy.rc" ) &
                zenity --progress --pulsate --auto-close --width=430 --no-cancel \
                    --title="กำลังอัพ mission ขึ้นโดรน" \
                    --text="กำลังส่ง $M_LABEL ไปที่ $HOST …\n(ครั้งแรกจะติดตั้ง library ด้วย)" \
                    < <(while kill -0 $! 2>/dev/null; do sleep 1; done) 2>/dev/null
                wait
                [ "$(cat "$STATE/deploy.rc" 2>/dev/null)" = 0 ] \
                    || die "อัพขึ้นโดรนไม่สำเร็จ\nดูรายละเอียดที่ $STATE/deploy.log"
                msg "อัพ <b>$M_LABEL</b> ขึ้นโดรนเรียบร้อย ✔"
            fi
        }
    fi
fi
printf '%s' "$HOST" > "$HOSTFILE"

# ── 4. ปิด console เครื่องจริงตัวเก่า (สลับ mission = เปิดไอคอนใหม่) ────────
if [ -f "$PIDFILE" ]; then
    while read -r pid; do [ -n "$pid" ] && kill "$pid" 2>/dev/null; done < "$PIDFILE"
    rm -f "$PIDFILE"
    sleep 1
fi

# ── 5. เปิด console + status_sync (background, ไม่ต้องมี terminal) ─────────
: > "$LOG"
nohup bash "$REPO_ROOT/cm4/status_sync.sh" "$HOST" "$M_DIR" >> "$LOG" 2>&1 &
echo $! > "$PIDFILE"
# telemetry เข้าทางไหน: วิทยุ NOMAD ที่เสียบ USB ก่อน แล้วค่อย udp 14550
# (cm4/pick_telemetry_link.sh — จุดเลือกร่วมกับไอคอนแบบ terminal)
. "$REPO_ROOT/cm4/pick_telemetry_link.sh"
pick_telemetry_link
echo "[real-gcs] telemetry link: $LINK_DESC" >> "$LOG"
nohup /usr/bin/python3 "$GCS" \
    --field "$M_FIELD" --captures "$M_CAPT" \
    "${LINK_ARGS[@]}" \
    --mission-cmd "ssh -o StrictHostKeyChecking=accept-new ${SSH_ID_STR}$HOST '$M_ENTRY'" \
    --mission-label REAL --port "$PORT" >> "$LOG" 2>&1 &
echo $! >> "$PIDFILE"

for _ in $(seq 1 20); do
    curl -s -m 1 -o /dev/null "http://127.0.0.1:$PORT/api/status" && break
    sleep 0.5
done
if [ "$NONINT" = 1 ]; then
    echo "[info] console up: http://127.0.0.1:$PORT ($M_LABEL → $HOST)"
    echo "[info] telemetry: $LINK_DESC"
else
    (xdg-open "http://127.0.0.1:$PORT" >/dev/null 2>&1 &)
    zenity --info --width=470 --title="AAVC GCS เครื่องจริง" \
      --text="เปิดแล้ว: <b>$M_LABEL</b>\nสั่งไปที่ <b>$HOST</b> → ~/$M_DIR\n\nTelemetry: <b>$LINK_DESC</b>\nหน้าเว็บ: http://127.0.0.1:$PORT\n\n• สลับ mission = เปิดไอคอนนี้ใหม่แล้วเลือกอีกอัน (ปิดตัวเก่าให้เอง)\n• ปิดทั้งหมด = เปิดไอคอนนี้แล้วกด 'ปิด console'" 2>/dev/null
fi
