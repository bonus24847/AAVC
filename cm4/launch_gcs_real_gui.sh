#!/usr/bin/env bash
# ไอคอน "AAVC GCS" — ทำทุกอย่างด้วยการคลิก ไม่ต้องแตะ terminal
# (operator 2026-08-15: "ผม deploy ขึ้น cm4 ไม่เป็น" + "ไม่อยากเข้า terminal").
#
# ไอคอนเดียวคุมทั้ง 2 โหมด (operator 2026-08-16: "รวม SIM กับ REAL เป็นตัวเดียว
# ได้ไหม") — ถามโหมดก่อนเป็นอย่างแรก:
#   * จำลอง  -> รัน reset_cmd ของ mission นั้น (ยกสแตก SITL + console) แล้วจบ
#   * เครื่องจริง -> ขั้นตอน CM4 ทั้งหมดข้างล่าง
#
# ขั้นตอนฝั่งเครื่องจริง:
#   1. ถามว่าจะบิน mission ไหน (อ่านจาก aavc-gcs/missions.yaml บล็อก `real:`)
#   2. หา/ถามที่อยู่ CM4 (สแกน <วงปัจจุบัน>.41 ให้ก่อน, จำค่าล่าสุด)
#   3. เช็ก ssh + เช็กว่า repo ของ mission นั้นอยู่บน CM4 แล้วหรือยัง
#      ถ้ายัง → กด "อัพขึ้นโดรน" แล้วมันรัน cm4/deploy.sh ให้พร้อมแถบความคืบหน้า
#   4. ปิด console เครื่องจริงตัวเก่า (ถ้ามี) — การสลับ mission จึงเป็นแค่
#      "เปิดไอคอนใหม่แล้วเลือกอีก mission" ไม่ต้อง Ctrl-C เอง
#   5. เปิด console + status_sync แบบ background (log ลงไฟล์) แล้วเปิดเบราว์เซอร์
#
# ทดสอบอัตโนมัติได้ด้วย AAVC_NONINTERACTIVE=1 + AAVC_MODE/AAVC_MISSION/AAVC_HOST
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

# ── 0. โหมด: จำลอง หรือ เครื่องจริง ───────────────────────────────────────
# ไอคอนเดียวคุมทั้งสองโหมด (ผู้ใช้ 2026-08-16: "รวม SIM กับ REAL เป็นตัวเดียว
# ได้ไหม") — missions.yaml เก็บทั้งสองอยู่ในรายการเดียวกันอยู่แล้ว (`mission_cmd`
# + `reset_cmd` = ฝั่งจำลอง, บล็อก `real:` = ฝั่งโดรน) ที่แยกกันมีแค่ตัวเปิด
#
# ⚠ สิ่งที่ "รวม" ได้คือ **ตัวเปิด** ไม่ใช่ตัว console — console หนึ่งตัวยังผูกกับ
# โหมดเดียวเสมอ (aavc_gcs.py ปฏิเสธการเอา console เครื่องจริงไปบิน mission จำลอง
# มาตั้งแต่ผู้ใช้ถามเอง 2026-08-14) เพราะความสับสนที่แพงที่สุดในโปรเจกต์นี้คือ
# **กด 🚀 คิดว่าเป็นซิม แต่ของจริง arm** ⇒ ถามโหมดก่อนเสมอ และค่าเริ่มต้นคือ
# จำลอง คนที่ตั้งใจบินของจริงต้องเลือกเอง
MODE="${AAVC_MODE:-}"
if [ -z "$MODE" ]; then
    MODE=$(zenity --list --radiolist --width=580 --height=280 \
        --title="AAVC GCS" --text="จะทำอะไร?" \
        --column="" --column="โหมด" --column="ทำอะไร" \
        TRUE  sim  "จำลอง (SITL) — ปลอดภัย ไม่มีอะไรขยับจริง" \
        FALSE real "🚁 เครื่องจริง — ปุ่ม 🚀 สั่งโดรนที่บินได้จริง" \
        FALSE stop "⏹ ปิดทุกอย่าง — หยุด SITL + Gazebo + หน้าเว็บให้หมดจริง" \
        2>/dev/null) || exit 0
fi

# ── 0b. ปิดทุกอย่างให้หมดจริง ──────────────────────────────────────────────
# ทำไมต้องมีเมนูนี้ (ผู้ใช้ 2026-08-16: "ถ้าผมกดปิดเว็บกับ gazebo มันจะปิดจริงไหม")
# **ไม่ปิดครับ** และเคยหลอกมาแล้วรอบหนึ่ง:
#   * ปิดแท็บเบราว์เซอร์ = ปิดแค่ "จอ" — console เป็นโปรเซสเซิร์ฟเวอร์ ยังรันต่อ
#   * ปิดหน้าต่าง Gazebo = ปิดแค่ตัว viewer (`gz sim -g`) — ตัวคำนวณโลก
#     (`gz sim -s`) ยังหมุนต่อ พร้อม PX4 + bridge + console ครบทีม
# ⇒ เครื่องดูเหมือนว่างแต่ยังกิน CPU/พอร์ตอยู่ และไปชนกับเซสชันข้างบ้าน
# ⚠ ปุ่มนี้ "ปิดของบนโน้ตบุ๊ก" เท่านั้น — จงใจ **ไม่แตะโดรน**: orchestrator ที่ตายกลาง
# อากาศแปลว่าไม่มีใครสั่งเครื่อง ซึ่งแย่กว่าคอนโซลค้างมาก การหยุดของบน CM4 ต้องเป็น
# การตัดสินใจแยกต่างหาก (RC ของ safety pilot คือทางแทรกแซงระหว่างบิน)
#
# ฆ่าแบบยกระดับ: TERM ก่อน (ให้โอกาสปิด log สวย ๆ) แล้ว KILL ถ้าไม่ยอมตาย —
# "โปรแกรมค้าง" คือกรณีที่ SIGTERM ไม่กินพอดี (operator 2026-08-18: "ปุ่มที่เชื่อว่า
# ปิดจริง ๆ")
kill_pattern() {
    local pat="$1" i
    pgrep -f "$pat" >/dev/null 2>&1 || return 0
    pkill -TERM -f "$pat" 2>/dev/null
    for i in 1 2 3 4 5 6; do
        sleep 0.5
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
    done
    pkill -KILL -f "$pat" 2>/dev/null
    sleep 0.5
}

stop_everything() {
    # จด PID ไว้ก็ใช้ แต่ไม่พึ่งมันอย่างเดียว: คอนโซลที่เปิดมาทางอื่น (เปิดมือ,
    # PIDFILE หาย, เครื่องรีสตาร์ต) เคยรอดจากปุ่มนี้มาแล้ว
    [ -f "$PIDFILE" ] && { while read -r pid; do
        [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null; done < "$PIDFILE"; }
    kill_pattern 'aavc_gcs[.]py'
    kill_pattern 'status_sync.s[h]'
    rm -f "$PIDFILE"
    bash "$REPO_ROOT/sitl/launch_stack.sh" stop 2>&1
}

# พิสูจน์ ไม่ใช่สันนิษฐาน — คืนรายการสิ่งที่ยังรอดอยู่ (ว่าง = ปิดหมดจริง)
stop_survivors() {
    local left=""
    pgrep -f 'aavc_gcs[.]py' >/dev/null 2>&1 && left+="· หน้าเว็บ console ยังรันอยู่\n"
    pgrep -f 'status_sync.s[h]' >/dev/null 2>&1 && left+="· status_sync ยังรันอยู่\n"
    pgrep -f 'gz si[m]' >/dev/null 2>&1 && left+="· Gazebo ยังรันอยู่\n"
    pgrep -f 'px4_sitl_default/bin/px[4]' >/dev/null 2>&1 && left+="· PX4 SITL ยังรันอยู่\n"
    ss -tln 2>/dev/null | grep -qE ':(8000|8020)\b' && left+="· พอร์ต 8000/8020 ยังถูกจับอยู่\n"
    printf '%b' "$left"
}

if [ "$MODE" = "stop" ]; then
    OUT="$(stop_everything)"
    RC=$?
    # launch_stack ออก 3 เมื่อมี SITL ของโปรเจกต์อื่นรันอยู่ — ไม่ใช่ความผิดพลาด
    # ของผู้ใช้ และไม่ควรฆ่าให้ (อาจกำลังบินอยู่) บอกไปตรง ๆ
    if [ "$RC" = 3 ]; then
        die "ไม่ได้ปิดให้ เพราะมี SITL ของอีกโปรเจกต์รันอยู่ (อาจกำลังบินอยู่)\n\n$OUT"
    fi
    LEFT="$(stop_survivors)"
    if [ -n "$LEFT" ]; then
        die "ปิดได้ไม่หมด ⚠\n\nยังเหลือ:\n$LEFT\nลองกดซ้ำอีกครั้ง หรือดูด้วย:\n  pgrep -af 'aavc_gcs|gz sim|px4'"
    fi
    msg "ปิดหมดแล้ว ✔ (ตรวจซ้ำแล้ว: ไม่เหลือโปรเซส พอร์ต 8000 ว่าง)\n\nSITL · Gazebo · bridge · หน้าเว็บ\n\nหมายเหตุ: ของบน CM4 (router/กล้อง/beacon/mission) ไม่ถูกแตะ — ตั้งใจ ไม่ให้ปุ่มนี้หยุดโดรนที่อาจกำลังบินอยู่"
    exit 0
fi

# ── 1. mission ที่จะบิน ────────────────────────────────────────────────────
[ -f "$REGISTRY" ] || die "ไม่พบรายการ mission ที่\n$REGISTRY"
mapfile -t ROWS < <(/usr/bin/python3 - "$REGISTRY" "$MODE" <<'PY'
import sys, yaml
doc = yaml.safe_load(open(sys.argv[1])) or {}
mode = sys.argv[2]
for name, m in (doc.get("missions") or {}).items():
    m = m or {}
    r = m.get("real") or {}
    if mode == "real":
        ok = bool(r.get("repo") and r.get("dir") and r.get("entry"))
    else:
        # a SIM entry is one that knows how to bring its own simulator up
        ok = bool(m.get("reset_cmd") and m.get("mission_cmd"))
    if ok:
        # \x1f (unit separator), NOT a tab: tab is IFS *whitespace*, so bash
        # collapses runs of them and an empty field silently shifts every
        # later column left. SIM rows have empty repo/dir/entry, which is
        # exactly that case (found by the smoke test, 2026-08-16).
        print("\x1f".join([name, str(m.get("label") or name),
                         str(r.get("repo") or ""), str(r.get("dir") or ""),
                         str(r.get("entry") or ""),
                         str(m.get("field") or ""), str(m.get("captures") or ""),
                         str(m.get("reset_cmd") or "")]))
PY
)
if [ ${#ROWS[@]} -eq 0 ]; then
    [ "$MODE" = "real" ] \
        && die "ยังไม่มี mission ที่ตั้งค่าสำหรับเครื่องจริงใน\n$REGISTRY" \
        || die "ยังไม่มี mission ที่เปิดตัวจำลองเองได้ใน\n$REGISTRY\n(ต้องมี reset_cmd)"
fi

pick_row() {
    local want="$1"
    for r in "${ROWS[@]}"; do [ "${r%%$'\x1f'*}" = "$want" ] && { echo "$r"; return; }; done
}
if [ -n "${AAVC_MISSION:-}" ]; then
    ROW="$(pick_row "$AAVC_MISSION")"
elif [ ${#ROWS[@]} -eq 1 ]; then
    ROW="${ROWS[0]}"
else
    LIST=(); for r in "${ROWS[@]}"; do
        IFS=$'\x1f' read -r n l _ <<<"$r"; LIST+=(FALSE "$n" "$l"); done
    LIST[0]=TRUE
    SEL=$(zenity --list --radiolist --width=520 --height=260 \
          --title="AAVC GCS เครื่องจริง" --text="จะบิน mission ไหนบนโดรน?" \
          --column="" --column="ชื่อ" --column="สนาม" "${LIST[@]}" 2>/dev/null) || exit 0
    ROW="$(pick_row "$SEL")"
fi
[ -n "${ROW:-}" ] || die "ไม่รู้จัก mission นี้"
IFS=$'\x1f' read -r M_NAME M_LABEL M_REPO M_DIR M_ENTRY M_FIELD M_CAPT M_RESET <<<"$ROW"

# ── 1b. โหมดจำลอง: ไม่มี CM4 ไม่มี ssh — ยกสแตกขึ้นมาแล้วจบ ──────────────
# reset_cmd ของแต่ละ mission คือคำสั่งที่ "ยกสนามจำลองขึ้นใหม่ทั้งชุด" อยู่แล้ว
# (ปุ่ม 🧹 ในหน้าเว็บใช้ตัวเดียวกัน) และมันเปิด console ให้ในตัว ⇒ ใช้ซ้ำได้เลย
# ไม่ต้องมีเส้นทางที่สองให้แตกต่างกันเงียบ ๆ
if [ "$MODE" != "real" ]; then
    [ -n "$M_RESET" ] || die "mission '$M_NAME' ไม่มี reset_cmd — เปิดตัวจำลองเองไม่ได้"
    # ⚠ KEEP_CONSOLE=1 -> 0. reset_cmd คือคำสั่งของปุ่ม 🧹 ซึ่งถูกสั่งจาก "ในหน้าเว็บ
    # ที่เปิดอยู่แล้ว" ⇒ มันตั้ง KEEP_CONSOLE=1 แปลว่า "ยกสนามใหม่ แต่อย่าฆ่า/อย่า
    # เปิด console" (ไม่งั้นจะตัดหน้าเว็บของคนกดเอง) พอเอามาเปิดเย็นแบบนี้ ความหมาย
    # นั้นกลายเป็น "ไม่ต้องเปิด console เลย" ⇒ สแตกขึ้นครบแต่ไม่มีอะไรฟังพอร์ต
    # และเบราว์เซอร์ขึ้น "unable to connect" (เจอจริง 2026-08-16)
    SIM_CMD="${M_RESET//KEEP_CONSOLE=1/KEEP_CONSOLE=0}"
    ( eval "$SIM_CMD" ) >>"$LOG" 2>&1 &
    msg "กำลังเปิดสนามจำลองของ <b>$M_LABEL</b>…\n\nใช้เวลาราวครึ่งนาที \
แล้วหน้าเว็บจะเปิดเอง\n\nlog: $LOG"
    # launch_stack ถอยไป 8020 เองถ้า 8000 ไม่ว่าง ⇒ อย่าเดาพอร์ตเดียว
    SIM_URL=""
    for _ in $(seq 1 90); do
        for p in "$PORT" 8020; do
            if curl -sf -m 1 "http://127.0.0.1:$p/" >/dev/null 2>&1; then
                SIM_URL="http://127.0.0.1:$p/"; break 2
            fi
        done
        sleep 1
    done
    [ -n "$SIM_URL" ] || die "สนามจำลองขึ้นแล้วแต่ไม่พบหน้าเว็บบนพอร์ต $PORT/8020\n\nดู log: $LOG"
    xdg-open "$SIM_URL" >/dev/null 2>&1 &
    exit 0
fi

[ -d "$M_REPO" ] || die "ไม่พบโฟลเดอร์ repo ของ mission นี้:\n$M_REPO"

# ── 2. ที่อยู่ CM4 ─────────────────────────────────────────────────────────
# ทาง ② (2026-08-20): CM4 ปล่อยวง AAVC-DRONE เอง (เป็น AP, ตัวมันเอง = 10.42.0.1)
# ถ้าแลปท็อปมีโปรไฟล์วงนี้แต่ยังไม่ได้เกาะ และวงลอยอยู่จริง ให้เกาะให้เองก่อนออกตามหา
# (AP ตัวนี้ "หูดับเป็นจังหวะ" ระหว่างที่มันสแกนหาวงอื่น — ลอง 2 รอบแล้วปล่อยผ่าน:
# ไม่ติดก็แค่ตกกลับไปเส้นทางถาม/สแกนแบบเดิม ไม่ใช่ความผิดร้ายแรง)
if nmcli -t -f NAME connection show 2>/dev/null | grep -qx 'AAVC-DRONE' \
   && ! nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx 'AAVC-DRONE'; then
    nmcli device wifi rescan >/dev/null 2>&1; sleep 2
    if nmcli -t -f SSID device wifi list 2>/dev/null | grep -qx 'AAVC-DRONE'; then
        nmcli -w 25 connection up AAVC-DRONE >/dev/null 2>&1 \
            || nmcli -w 25 connection up AAVC-DRONE >/dev/null 2>&1
    fi
fi
GUESS=""
for net in $(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}'); do
    base="${net%/*}"; base="${base%.*}"
    # วง 10.42.x = วงที่ CM4 เป็น AP เอง ⇒ ตัวมันคือ ".1" ของวง (คงที่ — keyfile
    # ปัก address1=10.42.0.1 ไว้); เลขท้าย ".41" เป็นธรรมเนียมของวง hotspot เดิม
    case "$base" in 10.42.*) CANDS=("$base.1" "$base.$CM4_OCTET") ;;
                    *)       CANDS=("$base.$CM4_OCTET") ;; esac
    for cand in "${CANDS[@]}"; do
        # 3 วิ ไม่ใช่ 1 — ARP ที่ยังไม่มีในตารางใช้เวลาเกิน 1 วิได้ แล้วไอคอนจะ
        # ขึ้น "ยังไม่พบ CM4" ทั้งที่โดรนออนไลน์อยู่ (2026-08-18)
        timeout 3 bash -c "echo > /dev/tcp/$cand/22" 2>/dev/null && { GUESS="$CM4_USER@$cand"; break 2; }
    done
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

# ── 5. เปิด console (background, ไม่ต้องมี terminal) ──────────────────────
# status_sync (ตัวดึง captures/ + เฟรมกล้องผ่าน WiFi) ถูกถอดออก 2026-08-18
# ตามคำสั่ง operator "เอาไหนที่ใช้วิทยุไม่ได้ เอาออกเลย" — คอนโซลตัวจริงอ่านทุกอย่าง
# จากวิทยุ NOMAD อย่างเดียว (beacon: phase/จำนวน/พิกัด pad/สาเหตุกลับบ้าน/สุขภาพกล้อง)
# รอบก่อนถอดไปแค่ `launch_gcs_real.sh` แต่ลืมไอคอนนี้ ซึ่งเป็นตัวที่ใช้จริง — ผลคือ
# ชิปกล้องไปคิดจากอายุไฟล์ที่ WiFi ดึงมา แทนที่จะเป็นสถานะที่วัดข้างกล้องบน CM4
: > "$LOG"
: > "$PIDFILE"
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
