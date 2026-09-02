#!/usr/bin/env bash
# ติดตั้งไอคอนเดสก์ท็อปของ AAVC GCS ให้ชี้มาที่ clone นี้
#
#     bash mission/cm4/install_icons.sh            # ดูว่าจะเขียนอะไรบ้าง
#     bash mission/cm4/install_icons.sh --apply    # เขียนจริง
#     bash mission/cm4/install_icons.sh --apply --desktop   # วางบนหน้า Desktop ด้วย
#
# ทำไมต้องมีไฟล์นี้ (2026-09-02): สเปค desktop-entry บังคับให้ `Exec=` และ
# `Icon=` เป็น **absolute path** ⇒ ไฟล์ .desktop ที่ commit ไว้จะชี้ไปที่บ้านของ
# เครื่องที่สร้างมันเสมอ และตายเงียบบนเครื่องอื่น (กดแล้วไม่มีอะไรเกิดขึ้น)
# สคริปต์นี้จึงเป็นตัว "ประกอบไอคอน ณ เครื่องที่ใช้จริง" — หา repo เองจากตำแหน่ง
# ไฟล์ แล้วเขียนไอคอนทั้งชุดด้วย path ของ clone นี้
#
# ไอคอนที่ได้ (ทั้งหมดเรียก cm4/launch_icon.sh ซึ่งล้าง state เก่าก่อนเสมอ):
#   AAVCGCS-PRACTICE   สนามซ้อม KMUTNB
#   AAVCGCS-COMP       สนามแข่ง KMITL
#   AAVCGCS-BANGBO     ทดสอบ landing ร.ร.บางบ่อวิทยาคม
#   AAVC-GCS-REAL      ตัวเลือกโหมด (จำลอง / เครื่องจริง) — ถามก่อนทุกครั้ง
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # …/mission
APPS="$HOME/.local/share/applications"
APPLY=0; TO_DESKTOP=0
for a in "$@"; do
    case "$a" in
        --apply) APPLY=1;;
        --desktop) TO_DESKTOP=1;;
        *) echo "ไม่รู้จักตัวเลือก: $a" >&2; exit 2;;
    esac
done

# desktop-entry Exec parsing knows double quotes and backslashes only — NOT the
# single quotes a shell would take (2026-08-20: two icons wrapped their command
# in '…', GNOME split it wrong and bash died before it ran anything, so the
# icon "opened" and nothing appeared). Script path in double quotes, plain
# argument after it, exactly the form that was proven to work.
emit() {   # emit <Name> <Name[th]> <Comment[th]> <icon.svg> <script> [arg]
    local name="$1" name_th="$2" comment_th="$3" icon="$4" script="$5" arg="${6:-}"
    cat <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=$name
Name[th]=$name_th
Comment[th]=$comment_th
Exec=bash "$REPO/cm4/$script"${arg:+ $arg}
Icon=$REPO/icons/$icon
Terminal=false
Categories=Utility;Science;
StartupNotify=true
EOF
}

write_one() {   # write_one <basename> <emit args…>
    local base="$1"; shift
    local body; body="$(emit "$@")"
    if [ "$APPLY" = 1 ]; then
        mkdir -p "$APPS"
        printf '%s\n' "$body" > "$APPS/$base.desktop"
        chmod +x "$APPS/$base.desktop"
        echo "เขียนแล้ว: $APPS/$base.desktop"
        if [ "$TO_DESKTOP" = 1 ]; then
            local dd; dd="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
            mkdir -p "$dd"
            printf '%s\n' "$body" > "$dd/$base.desktop"
            chmod +x "$dd/$base.desktop"
            echo "เขียนแล้ว: $dd/$base.desktop"
        fi
    else
        echo "── $base.desktop ─────────────────────────────"
        printf '%s\n\n' "$body"
    fi
}

echo "repo ที่ไอคอนจะชี้ไป: $REPO"
echo

write_one AAVCGCS-PRACTICE "AAVCGCS PRACTICE" "AAVC GCS ซ้อม (KMUTNB สนามฟุตบอล)" \
    "ล้าง state เก่าก่อนเสมอ แล้วเปิด console โหมดซ้อม KMUTNB (เพดาน 10 m)" \
    practice.svg launch_icon.sh kmutnb
write_one AAVCGCS-COMP "AAVCGCS COMP" "AAVC GCS แข่ง (KMITL)" \
    "ล้าง state เก่าก่อนเสมอ แล้วเปิด console โหมดแข่ง KMITL (เพดาน 30 m — briefing 28 ส.ค.)" \
    comp.svg launch_icon.sh aavc
write_one AAVCGCS-BANGBO "AAVCGCS BANGBO TEST" "AAVC GCS ทดสอบ landing บางบ่อ" \
    "ล้าง state เก่าก่อนเสมอ แล้วเปิด console สนามฟุตบอล ร.ร.บางบ่อวิทยาคม (เพดาน 10 m — ทดสอบ landing)" \
    practice.svg launch_icon.sh bangbo
write_one AAVC-GCS-REAL "AAVC GCS" "AAVC GCS (จำลอง / เครื่องจริง)" \
    "ไอคอนเดียวคุมทั้งสองโหมด — ค่าเริ่มต้นคือจำลอง โหมดเครื่องจริงต้องเลือกเอง" \
    practice.svg launch_gcs_real_gui.sh

if [ "$APPLY" != 1 ]; then
    echo "(เติม --apply เพื่อเขียนจริง, เติม --desktop ต่อท้ายเพื่อวางบนหน้า Desktop ด้วย)"
fi
