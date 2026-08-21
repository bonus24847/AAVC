#!/usr/bin/env bash
# GCS "🚀 บิน mission" launcher — spawned by the AAVC GCS console
# (aavc_gcs.py --mission-cmd '<this script> {ids}'). $1 = comma-separated
# assigned pad ids from the operator's saved selection.
#
# SITL: launch_stack.sh / `make aavc-gcs` wire this script in automatically.
# REAL BIRD: the orchestrator runs on the CM4, not the GCS laptop — point the
# console at ssh instead:
#   --mission-cmd "ssh <user>@<cm4> 'REAL=1 ~/mission/sitl/run_mission.sh {ids}'"
# REAL=1 switches to the offboard link (mavlink-router 14540) and drops the
# SITL-only truth audit; everything else is identical to the SITL path.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
IDS="${1:?usage: run_mission.sh <id,id,...>}"

# One mission at a time — the GCS also guards this, but a CLI-started run
# must be protected too.
if pgrep -f 'orchestrator.mai[n]' >/dev/null; then
    echo "[run_mission] refused: an orchestrator is already running" >&2
    exit 1
fi

# ── the aircraft's own infrastructure, brought up by the SAME button ───────
# Until 2026-08-18 this script started ONLY the orchestrator, which then
# connects to udpin://…:14540 — a port that exists only because mavlink-router
# is running. Nothing in the 🚀 path started the router, the camera grabber or
# the status beacon; they were started by hand over ssh. On a CM4 that has
# rebooted at the field (a battery swap is enough) pressing 🚀 therefore spawned
# an orchestrator that logged one line — "connecting to udpin://0.0.0.0:14540" —
# and died with no MAVLink, no audit trail and nothing on the operator's screen
# to say why. Seen live at 14:33 on 2026-08-18. The operator cannot ssh in
# during a scored round, so the button has to be self-sufficient.
#
# Idempotent: anything already running is left exactly as it is. Detached with
# setsid so it OUTLIVES this script — that matters most when the mission refuses
# to launch, because the beacon is what carries the refusal ("swap the pack") to
# the console over the radio. Killing it with the mission would hide the answer.
ensure_infra() {
    local serial="${SERIAL:-/dev/ttyAMA0}" baud="${BAUD:-921600}"
    local routerd="${ROUTERD:-}"
    if [ -z "$routerd" ]; then
        # A non-interactive ssh does not read the login PATH, so the locally
        # built binary in ~/.local/bin is invisible unless we look for it.
        for c in "$HOME/.local/bin/mavlink-routerd" "$(command -v mavlink-routerd 2>/dev/null)"; do
            [ -n "$c" ] && [ -x "$c" ] && { routerd="$c"; break; }
        done
    fi

    if ! pgrep -f 'mavlink-route[r]d' >/dev/null 2>&1; then
        if [ -z "$routerd" ]; then
            echo "[run_mission] ERROR: mavlink-router not found — the orchestrator" >&2
            echo "               cannot reach the FC. Install it or pass ROUTERD=<path>." >&2
            exit 3
        fi
        [ -e "$serial" ] || { echo "[run_mission] ERROR: $serial not found (FC cable?)" >&2; exit 3; }
        cat > /tmp/aavc_flight_router.conf <<EOF
[General]
TcpServerPort = 0
[UartEndpoint fc]
Device = $serial
Baud = $baud
[UdpEndpoint offboard]
Mode = Normal
Address = 127.0.0.1
Port = 14540
[UdpEndpoint qgc]
Mode = Server
Address = ${GCS_HOST:-127.0.0.1}
Port = 14550
EOF
        echo "[run_mission] starting mavlink-router ($serial@$baud -> :14540, :14550)"
        setsid "$routerd" -c /tmp/aavc_flight_router.conf \
            >/tmp/aavc_router.log 2>&1 </dev/null &
        # 0.5 s, not 3 — and the difference is whether the operator sees errors.
        # The GCS spawns this over ssh, sleeps exactly 1.5 s, then checks whether
        # the process already exited (aavc_gcs.py::start_mission); a failure
        # inside that window is shown in the browser, one after it is invisible —
        # the button reports "✅ up" and then quietly reverts. A 3 s wait here
        # pushed every orchestrator startup failure (broken venv, ImportError,
        # rejected --assigned-ids) to t≈3.1 s on exactly the cold-CM4 case this
        # function exists to rescue. Nothing needs the longer wait: the
        # orchestrator's own connect() waits up to 15 s for a heartbeat, so the
        # router only has to be STARTING, not ready.
        sleep 0.5
    else
        echo "[run_mission] mavlink-router already up"
    fi

    if [ "${NO_CAMERA:-0}" != "1" ]; then
        if ! pgrep -f 'camera_grabber.p[y]' >/dev/null 2>&1; then
            # กล้อง UVC ย้ายเลข device ได้ทุกครั้งที่ไฟกระตุก (เจอจริง 2026-08-20:
            # หลุดจาก video0 ไปเกิดใหม่เป็น video1 → grabber เปิด '0' ไม่ได้ ตายเงียบ
            # แล้ว beacon ร้อง cam=DEAD) — ใช้ path แบบ by-id ที่ตามกล้องไปทุกการ
            # enumerate เมื่อหาได้ ผู้เรียกยัง override ได้ด้วย GRAB_ARGS เดิม
            if [ -z "${GRAB_ARGS:-}" ]; then
                cam_dev=$(ls /dev/v4l/by-id/*-video-index0 2>/dev/null | head -1)
                [ -n "$cam_dev" ] && GRAB_ARGS="--nadir-device $cam_dev"
            fi
            # บังคับ exposure สั้นบนกล้องจริงเป็นค่าเริ่มต้น (G7 2026-08-21:
            # เฟรมบิน blur ~10x เพราะ auto_exposure ค้าง 16.6 ms) — ensure_infra
            # รันเฉพาะฝั่ง REAL อยู่แล้ว SITL/gz ไม่โดน. หน่วยคือ 100 µs ของ UVC
            # (20 = 2 ms); CAM_EXPOSURE=0 = กลับไป auto; CAM_GAIN (0-128)
            # ชดเชยความสว่าง — OV9281 ไม่มี auto-gain
            CAM_EXPOSURE="${CAM_EXPOSURE:-20}"
            if [ "$CAM_EXPOSURE" != "0" ] && [ "${BACKEND:-v4l2}" = "v4l2" ]; then
                GRAB_ARGS="${GRAB_ARGS:+$GRAB_ARGS }--exposure-100us $CAM_EXPOSURE"
                [ -n "${CAM_GAIN:-}" ] && GRAB_ARGS="$GRAB_ARGS --gain $CAM_GAIN"
            fi
            # ทุกเฟรมที่เขียน = โอกาสอ่าน marker เพิ่มหนึ่งครั้ง (vision worker
            # decode ทุกเฟรมใหม่ตั้งแต่ 2026-08-21) — เซนเซอร์ให้ 10 fps ที่
            # 1280x720 จึงเขียน 10 Hz; ปิด mirror เพราะเป็น PNG encode ตัวที่
            # สอง (วัดบน CM4 = 62 ms/เฟรม พอ ๆ กับตัว detect ทั้งตัว) ซึ่งมีแค่
            # dashboard เว็บที่อ่าน — จอ GCS กับ vision worker ใช้ nadir ทั้งคู่
            GRAB_ARGS="${GRAB_ARGS:+$GRAB_ARGS }--interval-s ${CAM_INTERVAL:-0.1}"
            [ "${CAM_MIRROR:-0}" = "0" ] && GRAB_ARGS="$GRAB_ARGS --no-mirror"
            echo "[run_mission] starting camera grabber (BACKEND=${BACKEND:-v4l2}${GRAB_ARGS:+ $GRAB_ARGS})"
            setsid make -C "$REPO_ROOT" camera-real BACKEND="${BACKEND:-v4l2}" \
                GRAB_ARGS="${GRAB_ARGS:-}" \
                >/tmp/aavc_camera.log 2>&1 </dev/null &
        else
            echo "[run_mission] camera grabber already up"
        fi
    fi

    if [ "${BEACON:-1}" = "1" ]; then
        if ! pgrep -f 'status_beacon.p[y]' >/dev/null 2>&1; then
            echo "[run_mission] starting status beacon (radio STATUSTEXT)"
            setsid env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" \
                "$REPO_ROOT/cm4/status_beacon.py" \
                --captures "$REPO_ROOT/captures" \
                --endpoint "udpout:127.0.0.1:14550" \
                >/tmp/aavc_beacon.log 2>&1 </dev/null &
        else
            echo "[run_mission] status beacon already up"
        fi
    fi
}

EXTRA=()
if [ "${REAL:-0}" = "1" ]; then
    ensure_infra
    # INFRA_ONLY: bring the aircraft's own stack up (router + camera grabber +
    # status beacon) but DO NOT start the orchestrator — no arm, no flight. The
    # REAL console auto-triggers this on connect so the camera sensor chip lights
    # from the radio beacon without staging a mission (operator 2026-08-19).
    if [ "${INFRA_ONLY:-0}" = "1" ]; then
        echo "[run_mission] INFRA_ONLY: router + camera grabber + status beacon up —"
        echo "              orchestrator NOT started (no arm, no flight)."
        exit 0
    fi
    EXTRA+=(--connect "udpin://0.0.0.0:14540")
    # REAL bird defaults to RC-GO (operator conops 2026-08-12): the console's
    # 🚀 only STAGES the flight — the SAFETY PILOT arms via RC and flips to
    # OFFBOARD to launch; flipping to POSCTL mid-flight makes the orchestrator
    # stand down. The web never moves the aircraft. RC_GO=0 restores the
    # auto-launch behaviour (orchestrator arms itself right after preflight).
    RC_GO="${RC_GO:-1}"
else
    # SITL dirty-field precheck (2026-08-12, hit live by the operator): cargo
    # boxes shed by a PREVIOUS run stay lying ON the pads and hide the ArUco
    # markers — the next mission then can't decode those pads at all (looks
    # like "pads missing" on the GCS map) and flies home with eggs unserved.
    # The detach bridge's log is truncated at every stack (re)start, so any
    # "shed" line in it means THIS field already has boxes down. Refuse with
    # a clear message — the console surfaces this line in the browser.
    # (2026-08-15: the bridge's line reads "shed box <n>" since the release
    # rack stopped being wired in delivery order — box index != payload_id.
    # The old wording is kept in the pattern so a stale log still trips it.)
    if grep -qE 'shed (box|payload)' /tmp/aavc_detach.log 2>/dev/null; then
        echo "❌ สนามยังมีกล่องจากรอบก่อนวางบัง marker อยู่ — กดปุ่ม 🧹 รีเซ็ตสนาม ก่อนบินใหม่" >&2
        exit 1
    fi
    # SITL: enable the post-flight truth audit when the spawner wrote one.
    [ -f /tmp/aavc_targets.json ] && EXTRA+=(--truth-json /tmp/aavc_targets.json)
fi

if [ "${RC_GO:-0}" = "1" ]; then
    EXTRA+=(--rc-go)
fi

echo "[run_mission] assigned ids: $IDS (${REAL:+REAL bird}${REAL:-SITL})${RC_GO:+ rc-go=$RC_GO}"

# Announce simulated wind. It is set at RUNTIME (sitl/set_wind.sh), so it leaves
# no trace in the world file, the audit or the ULog — a flight in 10 m/s reads
# exactly like a calm one afterwards, and its numbers get compared against calm
# baselines by someone who has no way to know. Say it loudly instead.
WIND_STATE=/tmp/aavc_wind_state
if [ "${REAL:-0}" != "1" ] && [ -f "$WIND_STATE" ]; then
    read -r W_SPEED W_DIR < "$WIND_STATE"
    echo "🌬  WIND IS ON: ${W_SPEED} m/s from ${W_DIR}deg — this flight is NOT a"
    echo "   still-air baseline. 'bash sitl/set_wind.sh off' first if you wanted one."
fi

# ── SITE: which field this flight is for. Precedence: an icon's explicit env
# wins, then this repo's own .aavc_site marker (aavc-practice ships the KMUTNB
# one, aavc-comp the KMITL one — sync_core.sh never copies it, so each repo
# keeps its field), then the KMUTNB fallback below. A bare run is unchanged.
[ -f "$REPO_ROOT/.aavc_site" ] && . "$REPO_ROOT/.aavc_site"
AAVC_PROFILE="${AAVC_PROFILE:-kmutnb_skyfield}"
AAVC_CONFIG="${AAVC_CONFIG:-sitl/aavc_config.yaml}"
echo "🌐 SITE: profile=${AAVC_PROFILE}  config=${AAVC_CONFIG}"

if [ "${REAL:-0}" = "1" ]; then
    exec env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" -m orchestrator.main \
        --config "$AAVC_CONFIG" --profile "$AAVC_PROFILE" --no-dashboard \
        --assigned-ids "$IDS" "${EXTRA[@]}"
fi

# SITL: do NOT exec — the shed boxes only exist inside the running simulator,
# so where each egg actually landed has to be read BEFORE anything tears gz
# down. Kill gz first and the evidence is gone with it; the only way back is
# to fly the whole mission again (the parallel session lost a ~40-minute round
# exactly this way, 2026-08-16). Saved next to the audit so it survives the
# teardown that follows.
set +e
env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" -m orchestrator.main \
    --config "$AAVC_CONFIG" --profile "$AAVC_PROFILE" --no-dashboard \
    --assigned-ids "$IDS" "${EXTRA[@]}"
rc=$?
set -e

RUN_DIR="$(ls -dt "$REPO_ROOT"/runs/*/ 2>/dev/null | head -1)"
if [ -n "$RUN_DIR" ] && pgrep -f 'gz [s]im' >/dev/null; then
    echo "[run_mission] reading where the eggs landed (gz still up)…"
    env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" "$REPO_ROOT/tools/box_truth.py" \
        2>&1 | tee "${RUN_DIR}box_truth.txt" | tail -n 6
    # Stamp the air the numbers were measured in, next to the numbers.
    if [ -f "$WIND_STATE" ]; then
        read -r W_SPEED W_DIR < "$WIND_STATE"
        echo "wind: ${W_SPEED} m/s from ${W_DIR} deg" >> "${RUN_DIR}box_truth.txt"
    else
        echo "wind: still air" >> "${RUN_DIR}box_truth.txt"
    fi
fi
exit "$rc"
