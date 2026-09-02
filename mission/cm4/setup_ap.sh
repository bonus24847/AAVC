#!/usr/bin/env bash
# ให้ CM4 "ปล่อย WiFi ของตัวเอง" — เลิกพึ่ง hotspot มือถือ
#
# รันบน CM4 (ไม่ใช่บนโน้ตบุ๊ก):
#     ssh -i ~/.ssh/cm4_key drone@<ip>
#     bash ~/mission/cm4/setup_ap.sh            # ดูว่าจะทำอะไรบ้าง (ไม่แตะระบบ)
#     bash ~/mission/cm4/setup_ap.sh --apply    # ทำจริง (จะถามรหัส sudo)
#
# ทำไมต้องมีไฟล์นี้ (ผู้ใช้ 2026-08-16): "ไม่อยากให้พึ่งพามือถือผมตอนใช้งานจริง
# เพราะ WiFi ที่ใช้อยู่คือ hotspot จากมือถือผมเอง" — ถูกต้องแล้วที่ไม่อยากพึ่ง
# มือถือวันแข่ง มือถือดับ/สายเข้า/แบตหมด = คุมโดรนไม่ได้ทั้งระบบ
#
# วิธีนี้กลับด้านกัน: **โดรนเป็นคนปล่อยสัญญาณ โน้ตบุ๊กเป็นคนเข้าไปหา** ⇒ ไม่ต้อง
# มีมือถือ ไม่ต้องมีเราเตอร์ ไม่ต้องมีอินเทอร์เน็ต และใช้ได้ตอนบินด้วย (สายทำไม่ได้)
#
# ⚠ ข้อควรรู้ 3 ข้อ
#   1. AP = CM4 **กำลังส่งคลื่น** และกติกาห้ามส่งสัญญาณขณะอยู่ในพื้นที่พัก
#      (RULES_AAVC2026.md §6) ⇒ ในพิทให้ใช้ "สาย LAN" แทน (ดูท้ายไฟล์) แล้วค่อย
#      เปิด AP ตอนถึงไลน์บิน หรือปิดด้วย: sudo nmcli connection down AAVC-AP
#   2. วิทยุการ์ดเดียวเป็นได้อย่างเดียว — พอเป็น AP แล้ว CM4 จะ **เข้า WiFi อื่น
#      ไม่ได้** (รวมถึง hotspot เดิม) เลิก AP ด้วย --revert ถ้าจะกลับไปใช้แบบเดิม
#   3. ที่อยู่จะเปลี่ยนจาก "<วง>.41" เป็น **10.42.0.1** (CM4 เป็นเจ้าของวงเอง)
#      cm4/find_cm4.sh หาเจออยู่แล้วเพราะมันกวาดทั้งวง ไม่ได้ยึดเลข .41
set -uo pipefail

SSID="${AAVC_AP_SSID:-AAVC-DRONE}"
# ⚠ ไม่มีรหัสผ่านค่าเริ่มต้นอีกแล้ว (2026-09-02, ตอนเปิด repo เป็นสาธารณะ)
# เดิมบรรทัดนี้เป็น PASS="${AAVC_AP_PASS:-<รหัสตายตัว>}" ⇒ ใครอ่านไฟล์นี้ก็เข้า
# วง WiFi ของโดรนได้ และวงนั้นคือทางเข้า ssh ของ CM4 กับหน้า console ที่ปุ่ม
# ปล่อย servo ไม่มี auth ⇒ ต้องตั้งเองทุกครั้ง:
#     AAVC_AP_PASS='...' bash cm4/setup_ap.sh --apply
# ไม่อยากคิดเอง ให้สุ่ม:  AAVC_AP_PASS=$(openssl rand -base64 12)
PASS="${AAVC_AP_PASS:-}"
IFACE="${AAVC_AP_IFACE:-wlan0}"
CONN="AAVC-AP"

if [ -z "$PASS" ] && [ "${1:-}" != "--revert" ]; then
    echo "ต้องตั้งรหัสผ่านของวงเอง — ไม่มีค่าเริ่มต้นให้แล้ว:" >&2
    echo "    AAVC_AP_PASS='รหัสที่ต้องการ' bash cm4/setup_ap.sh --apply" >&2
    echo "  สุ่มให้:  AAVC_AP_PASS=\$(openssl rand -base64 12)" >&2
    echo "  (อย่างน้อย 8 ตัวอักษร ตามข้อกำหนดของ WPA2)" >&2
    exit 2
fi
if [ -n "$PASS" ] && [ "${#PASS}" -lt 8 ]; then
    echo "รหัสผ่าน WPA2 ต้องยาวอย่างน้อย 8 ตัวอักษร (ได้มา ${#PASS})" >&2
    exit 2
fi

if [ "${1:-}" = "--revert" ]; then
    echo "จะลบ AP แล้วให้ CM4 กลับไปเข้า WiFi ปกติ:"
    echo "    sudo nmcli connection down $CONN"
    echo "    sudo nmcli connection delete $CONN"
    [ "${2:-}" = "--apply" ] || { echo; echo "(เติม --apply ต่อท้ายเพื่อทำจริง)"; exit 0; }
    sudo nmcli connection down "$CONN" 2>/dev/null
    sudo nmcli connection delete "$CONN"
    echo "ลบแล้ว — รีบูตหรือ 'sudo nmcli device connect $IFACE' เพื่อกลับเข้าวงเดิม"
    exit 0
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "เครื่องนี้ไม่มี NetworkManager (nmcli) — สคริปต์นี้ใช้กับ Raspberry Pi OS" >&2
    echo "Bookworm ขึ้นไป ถ้าเป็นรุ่นเก่ากว่านั้นต้องตั้ง hostapd เองแทน" >&2
    exit 1
fi

# ทำงานบน CM4 เท่านั้น — รันผิดเครื่องแล้วโน้ตบุ๊กจะกลายเป็น AP เสียเอง
if [ ! -d "$HOME/mission" ] && [ "${AAVC_AP_FORCE:-0}" != "1" ]; then
    echo "⚠ ไม่พบ ~/mission — ดูเหมือนไม่ได้รันบน CM4" >&2
    echo "  ถ้าแน่ใจว่าถูกเครื่อง ใส่ AAVC_AP_FORCE=1 นำหน้าคำสั่ง" >&2
    exit 1
fi

cat <<EOF
จะตั้งให้ CM4 ปล่อย WiFi ของตัวเอง:

    ชื่อวง (SSID) : $SSID
    รหัสผ่าน      : (จาก AAVC_AP_PASS — ไม่พิมพ์ออกจอ)
    การ์ด         : $IFACE
    ที่อยู่ CM4    : 10.42.0.1        <- ใช้แทน <วง>.41 เดิม
    เปิดเองตอนบูต : ใช่

คำสั่งที่จะรัน:
    sudo nmcli device wifi hotspot ifname $IFACE con-name $CONN ssid $SSID password '<AAVC_AP_PASS>'
    sudo nmcli connection modify $CONN connection.autoconnect yes \\
                                       connection.autoconnect-priority 100
EOF

if [ "${1:-}" != "--apply" ]; then
    cat <<'EOF'

(ยังไม่ได้ทำอะไรกับระบบ — เติม --apply เพื่อทำจริง)

หลังทำเสร็จ ที่โน้ตบุ๊ก:
    1. เข้า WiFi ชื่อ AAVC-DRONE
    2. bash cm4/find_cm4.sh          -> ควรเจอ drone@10.42.0.1
    3. bash cm4/launch_gcs_real.sh drone@10.42.0.1
EOF
    exit 0
fi

sudo nmcli device wifi hotspot ifname "$IFACE" con-name "$CONN" \
     ssid "$SSID" password "$PASS" || exit 1
sudo nmcli connection modify "$CONN" connection.autoconnect yes \
     connection.autoconnect-priority 100 || exit 1

echo
echo "✔ เรียบร้อย — CM4 ปล่อยวง '$SSID' แล้ว และจะเปิดเองทุกครั้งที่บูต"
echo "  ที่โน้ตบุ๊ก: เข้าวงนี้ แล้ว  bash cm4/launch_gcs_real.sh drone@10.42.0.1"
echo "  ปิดชั่วคราว (เช่นตอนอยู่พื้นที่พัก): sudo nmcli connection down $CONN"
