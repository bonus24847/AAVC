#!/usr/bin/env bash
# Desktop-icon entry point for the REAL-aircraft console (operator request
# 2026-08-14: "ทำไอคอน AAVC GCS เครื่องจริง"). Asks for the CM4's address in a
# dialog — no typing commands at the field — remembers the last one, checks
# ssh, then hands over to cm4/launch_gcs_real.sh (console + status_sync).
#
# Why a separate icon: the console's mode is decided by HOW it starts. The
# plain "AAVC GCS SIM" icon launches the laptop-side simulator console
# (🚀 runs the mission HERE); this one wires 🚀 to ssh into the CM4, which is
# what makes the locked "🔒 เครื่องจริง" mission card appear.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATE="$HOME/.config/aavc"
HOSTFILE="$STATE/cm4_host"
mkdir -p "$STATE"
CM4_USER="${CM4_USER:-drone}"      # this build's CM4 login (see docs/CM4_ACCESS.md)
CM4_OCTET="${CM4_OCTET:-41}"       # the CM4 always takes .41 on whatever /24 it joins
CM4_KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"
SSH_ID=(); [ -f "$CM4_KEY" ] && SSH_ID=(-i "$CM4_KEY")

die() { zenity --error --width=420 --title="AAVC GCS (เครื่องจริง)" --text="$1" 2>/dev/null; exit 1; }

# Suggest an address instead of asking the operator to remember one: probe
# <this laptop's /24>.CM4_OCTET for an open ssh port, else fall back to the
# last address that worked (operator 2026-08-15: "ผมไม่รู้ว่าที่อยู่ CM4 คืออะไร").
GUESS=""
for net in $(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}'); do
    base="${net%.*}"; cand="$base.$CM4_OCTET"
    if timeout 1 bash -c "echo > /dev/tcp/$cand/22" 2>/dev/null; then
        GUESS="$CM4_USER@$cand"; break
    fi
done
LAST="$(cat "$HOSTFILE" 2>/dev/null || true)"
DEFAULT="${GUESS:-${LAST:-$CM4_USER@192.168.x.$CM4_OCTET}}"
HINT=$([ -n "$GUESS" ] && echo "พบ CM4 ในวงนี้แล้ว ✔" \
       || echo "ยังไม่พบ CM4 ในวง WiFi ปัจจุบัน — เช็กว่าโดรนเปิดและต่อ WiFi วงเดียวกัน")

HOST=$(zenity --entry --width=460 --title="AAVC GCS — เครื่องจริง" \
  --text="ที่อยู่ CM4 บนโดรน (user@ip)\n\n$HINT" \
  --entry-text="$DEFAULT" 2>/dev/null) || exit 0        # Cancel = เงียบ ๆ ออก
HOST="${HOST// /}"
[ -n "$HOST" ] || die "ยังไม่ได้ใส่ที่อยู่ CM4"
[[ "$HOST" == *"@"* ]] || die "รูปแบบต้องเป็น <user>@<ip>\nเช่น aavc@192.168.1.41"

# ssh must be passwordless — the 🚀 button uses the same path, so a password
# prompt would hang the GO with no visible error at the field.
if ! ssh "${SSH_ID[@]}" -o ConnectTimeout=6 -o BatchMode=yes "$HOST" true 2>/dev/null; then
    zenity --question --width=480 --title="ต่อ CM4 ไม่ได้" \
      --text="ssh ไป <b>$HOST</b> ไม่สำเร็จ (ไม่ตอบ หรือยังต้องใส่รหัสผ่าน)\n\nเช็ก: โดรนเปิดอยู่ไหม · WiFi วงเดียวกันไหม · key ถูกตัวไหม\n(ตั้งครั้งเดียว: <tt>ssh-copy-id -i $CM4_KEY $HOST</tt>)\n\nจะเปิด console ต่อไปไหม? (ปุ่ม 🚀 จะล็อกจนกว่าจะเจอ CM4)" \
      --ok-label="เปิดต่อไป" --cancel-label="ยกเลิก" 2>/dev/null || exit 0
fi

printf '%s' "$HOST" > "$HOSTFILE"          # remember for next launch

# Run in a terminal so the operator can watch the log and Ctrl-C to stop BOTH
# the console and status_sync (launch_gcs_real.sh traps it).
exec gnome-terminal --title="AAVC GCS — เครื่องจริง ($HOST)" -- \
     bash -c "'$REPO_ROOT/cm4/launch_gcs_real.sh' '$HOST'; echo; echo '[ปิดแล้ว] กด Enter เพื่อปิดหน้าต่าง'; read -r"
