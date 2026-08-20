#!/usr/bin/env bash
# หา CM4 บนโดรนในวง WiFi ปัจจุบัน (operator 2026-08-15: "จะหา drone@<ip>
# มาจากไหน") — กวาดทุกเลขในวงหาเครื่องที่เปิดพอร์ต ssh แล้วลองเข้าด้วยกุญแจ
# ของโปรเจกต์ เพื่อบอกว่าเครื่องไหนคือ CM4 จริง ๆ (ไม่ใช่แค่ "มีอะไรสักอย่าง").
#
#   cm4/find_cm4.sh              # กวาดวงของโน้ตบุ๊กเอง
#   cm4/find_cm4.sh 192.168.14   # ระบุวงเอง (เช่นตอนใช้ hotspot มือถือ)
#
# ผลลัพธ์ที่ต้องการคือบรรทัด "✔ CM4: drone@<ip>" — เอาไปวางในไอคอน
# "AAVC GCS เครื่องจริง" ได้เลย (ปกติไอคอนเติมให้เองอยู่แล้ว)
set -uo pipefail

USER_NAME="${CM4_USER:-drone}"
KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"
CM4_OCTET="${CM4_OCTET:-41}"   # CM4 จองเลขท้ายนี้เสมอ (docs/CM4_ACCESS.md)
SSH_ID=(); [ -f "$KEY" ] && SSH_ID=(-i "$KEY")
PREFIXES=()

if [ $# -ge 1 ]; then
    PREFIXES=("${1%.}")
else
    while read -r cidr; do
        [ -n "$cidr" ] && PREFIXES+=("$(echo "${cidr%/*}" | cut -d. -f1-3)")
    done < <(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}')
fi
[ ${#PREFIXES[@]} -gt 0 ] || { echo "ไม่พบวง WiFi/LAN บนเครื่องนี้ — ต่อ WiFi ก่อน" >&2; exit 1; }

# ── ทางลัด: ถามเลขที่ CM4 จองไว้ก่อน "ทีละตัว" ────────────────────────────
# 2026-08-18: CM4 ออนไลน์อยู่ที่ .41 แต่การกวาดทั้งวงหาไม่เจอ — วัดแล้วไม่ใช่
# เรื่อง timeout (โพรบเดี่ยวตอบใน 0.02-0.17 วิ) แต่เป็นเพราะยิง 64 โพรบพร้อมกัน
# ทั้ง /24 ทำให้ ARP ของ hotspot ท่วมจน .41 ตกหล่น (ping sweep ก็พลาดเหมือนกัน).
# ถามตัวที่น่าจะใช่ก่อนโดยไม่มีใครแย่ง จึงทั้งเร็วกว่าและเชื่อถือได้กว่าการกวาด.
for base in "${PREFIXES[@]}"; do
    # วง 10.42.x = CM4 เป็น AP เอง (ทาง ② 2026-08-20) ⇒ ตัวมันคือ ".1" ของวง
    case "$base" in 10.42.*) CANDS=("$base.1" "$base.$CM4_OCTET") ;;
                    *)       CANDS=("$base.$CM4_OCTET") ;; esac
    for cand in "${CANDS[@]}"; do
        timeout 3 bash -c "echo > /dev/tcp/$cand/22" 2>/dev/null || continue
        name=$(timeout 8 ssh "${SSH_ID[@]}" -o ConnectTimeout=5 -o BatchMode=yes \
                 -o StrictHostKeyChecking=accept-new "$USER_NAME@$cand" \
                 'hostname; ls -d ~/mission >/dev/null 2>&1 && echo HAS_MISSION' 2>/dev/null)
        [ -n "$name" ] || continue
        host=$(echo "$name" | head -1)
        extra=$(echo "$name" | grep -q HAS_MISSION && echo "  (มี ~/mission แล้ว)")
        echo "  ✔ CM4: $USER_NAME@$cand   [hostname: $host]$extra"
        exit 0
    done
done

for base in "${PREFIXES[@]}"; do
    echo "[find] กวาดวง $base.0/24 …"
    hits=()
    # เช็กพอร์ต 22 พร้อมกันทีละ 24 ตัว (เดิม 64 — มากไปจน ARP ท่วมและตกเครื่อง)
    for i in $(seq 1 254); do
        ( timeout 2 bash -c "echo > /dev/tcp/$base.$i/22" 2>/dev/null \
          && echo "$base.$i" ) &
        (( i % 24 == 0 )) && wait
    done > /tmp/.cm4_scan_$$ 2>/dev/null
    wait
    mapfile -t hits < <(sort -t. -k4 -n /tmp/.cm4_scan_$$ 2>/dev/null)
    rm -f /tmp/.cm4_scan_$$
    [ ${#hits[@]} -gt 0 ] || { echo "  (ไม่พบเครื่องที่เปิด ssh ในวงนี้)"; continue; }

    for ip in "${hits[@]}"; do
        name=$(timeout 6 ssh "${SSH_ID[@]}" -o ConnectTimeout=4 -o BatchMode=yes \
                 -o StrictHostKeyChecking=accept-new "$USER_NAME@$ip" \
                 'hostname; ls -d ~/mission >/dev/null 2>&1 && echo HAS_MISSION' 2>/dev/null)
        if [ -n "$name" ]; then
            host=$(echo "$name" | head -1)
            extra=$(echo "$name" | grep -q HAS_MISSION && echo "  (มี ~/mission แล้ว)")
            echo "  ✔ CM4: $USER_NAME@$ip   [hostname: $host]$extra"
        else
            echo "  · $ip เปิด ssh แต่เข้าด้วยบัญชี '$USER_NAME' + กุญแจนี้ไม่ได้ (คนละเครื่อง)"
        fi
    done
done

cat <<'EOF'

ถ้าไม่เจอ CM4 เลย ให้ไล่ตามนี้:
  1. โดรนเปิดอยู่ไหม (CM4 ใช้เวลาบูต ~30-60 วินาที)
  2. CM4 เข้า WiFi วงเดียวกับโน้ตบุ๊กหรือยัง — วงใหม่ที่มันไม่เคยรู้จัก มันจะเข้าไม่ได้เอง
     (เคยใช้ hotspot มือถือมาก่อน: ลองเปิด hotspot ตัวเดิม แล้วให้โน้ตบุ๊กเข้าวงนั้นด้วย)
  3. ถ้ายังไม่เจอ: ต่อจอ+คีย์บอร์ดที่ CM4 ครั้งเดียวเพื่อตั้ง WiFi ใหม่
EOF
