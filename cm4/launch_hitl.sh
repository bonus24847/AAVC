#!/usr/bin/env bash
# AAVC 2026 — CM4-in-the-loop HITL launcher (real Pixhawk 6X on the HIL build).
#
# The REAL CM4 runs the REAL mission against the REAL 6X, while jMAVSim (on the
# laptop, owning the 6X USB) provides physics. No motors spin. This validates the
# actual flight-companion compute path — the point of HITL for THIS aircraft.
#
# Prereqs (see docs/HITL.md):
#   • 6X on the HIL firmware (px4_fmu-v6x_hil) + `make hitl-params` done (SYS_HITL=1).
#   • jMAVSim running on the laptop, owning the 6X USB HIL link:
#         jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 921600 -r 250
#   • The 6X MISSION link reaches THIS CM4 on TELEM2 (SERIAL=/dev/ttyAMA0), separate
#     from the USB HIL link.
#   • `make spawn-targets` has written /tmp/aavc_targets.json (synthetic-cam pads).
#
# Brings up + tears down, in order:
#   1) mavlink-router      6X TELEM2 -> :14540 (orchestrator) + :14541 (synthetic cam)
#   2) synthetic camera    GLOBAL_POSITION_INT -> /tmp/aavc_{nadir,frame}.png
#   3) orchestrator        the V1.3 multi-sortie mission, ONCE (not restarted)
#
# Env overrides:
#   SERIAL=/dev/ttyAMA0        6X MISSION link (CM4 TELEM2 UART; USB bench: /dev/ttyACM0)
#   BAUD=921600
#   CONNECT=udpin://0.0.0.0:14540    orchestrator endpoint (matches the router)
#   ROUTER_CONF=sitl/hitl_router.conf   (default: generated to /tmp with SERIAL/BAUD)
#   ROUTERD=mavlink-routerd    router binary (or an absolute path to a local build)
#   TARGETS=/tmp/aavc_targets.json      synthetic-camera pad layout
#   CONFIG=sitl/aavc_config.yaml
#   HEADLESS=1                 --no-dashboard (auto-GO when preflight criticals pass)
#   ASSIGNED_IDS="3,1,4,6"     headless committee stand-in (per-sortie assigned ids)
#   DASH_HOST=127.0.0.1        dashboard bind (0.0.0.0 ONLY behind your own auth)
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1
PY=(env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python")

SERIAL="${SERIAL:-/dev/ttyAMA0}"
BAUD="${BAUD:-921600}"
CONNECT="${CONNECT:-udpin://0.0.0.0:14540}"
ROUTERD="${ROUTERD:-mavlink-routerd}"
TARGETS="${TARGETS:-/tmp/aavc_targets.json}"
CONFIG="${CONFIG:-sitl/aavc_config.yaml}"
DASH_HOST="${DASH_HOST:-127.0.0.1}"

INFRA_PIDS=()
cleanup() {
    echo "[hitl] stopping the onboard HITL stack…"
    pkill -9 -P $$ 2>/dev/null
    for p in "${INFRA_PIDS[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    pkill -9 -f 'hitl_synthetic_camera.p[y]' 2>/dev/null
    pkill -9 -f 'orchestrator.mai[n]' 2>/dev/null
    pkill -9 -f 'mavsdk_serve[r]' 2>/dev/null
    pkill -9 -f 'mavlink-route[r]d' 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ ! -e "$SERIAL" ]; then
    echo "[hitl] ERROR: 6X mission link $SERIAL not found. Set SERIAL=… (TELEM2 UART or USB)."
    exit 1
fi
command -v "$ROUTERD" >/dev/null 2>&1 || { echo "[hitl] ERROR: $ROUTERD not on PATH (build mavlink-router or set ROUTERD=<path>)"; exit 1; }
if [ ! -f "$TARGETS" ]; then
    echo "[hitl] ERROR: $TARGETS not found — run 'make spawn-targets' first so the"
    echo "[hitl]        synthetic camera knows where to draw the ArUco pads."
    exit 1
fi

keep_alive() {
    local name="$1"; shift
    ( while true; do
        echo "[hitl] start $name"
        "$@"
        echo "[hitl] $name exited (rc=$?) — restarting in 2s"
        sleep 2
      done ) &
    INFRA_PIDS+=("$!")
}

# 1) mavlink-router — owns the 6X mission link, fans out to orch + synthetic cam.
ROUTER_CONF="${ROUTER_CONF:-}"
if [ -z "$ROUTER_CONF" ]; then
    ROUTER_CONF="/tmp/aavc_hitl_router.conf"
    cat > "$ROUTER_CONF" <<EOF
[General]
TcpServerPort = 0
[UartEndpoint fc]
Device = $SERIAL
Baud = $BAUD
[UdpEndpoint offboard]
Mode = Normal
Address = 127.0.0.1
Port = 14540
[UdpEndpoint hitlcam]
Mode = Normal
Address = 127.0.0.1
Port = 14541
[UdpEndpoint raw]
# Dashboard ESC/servo/consumed-mAh listener (config raw_telemetry_port=14551).
Mode = Normal
Address = 127.0.0.1
Port = 14551
[UdpEndpoint qgc]
Mode = Server
# S6: default LOOPBACK — do NOT expose unauthenticated MAVLink to the LAN. Set
# GCS_HOST=0.0.0.0 (or the CM4's LAN IP) to open it for a ground QGC.
Address = ${GCS_HOST:-127.0.0.1}
Port = 14550
EOF
fi
echo "[hitl] mavlink-router: $SERIAL@$BAUD -> :14540 (orch) + :14541 (cam) + :14550 (QGC @ ${GCS_HOST:-127.0.0.1})"
keep_alive "mavlink-router" "$ROUTERD" -c "$ROUTER_CONF"

# 2) synthetic camera — ArUco pads drawn from FC position (no real cameras in HITL).
echo "[hitl] synthetic camera: targets=$TARGETS feed=udpin:0.0.0.0:14541"
keep_alive "synthetic-camera" "${PY[@]}" sitl/hitl_synthetic_camera.py \
    --mavlink udpin:0.0.0.0:14541 --targets "$TARGETS"
for _ in $(seq 1 20); do [ -f /tmp/aavc_nadir.png ] && break; sleep 0.5; done

# 3) orchestrator — the V1.3 mission, ONCE (not restarted; a re-run would re-fly).
echo "[hitl] orchestrator: --connect $CONNECT --config $CONFIG"
ORCH_ARGS=(-m orchestrator.main --config "$CONFIG" --connect "$CONNECT")
if [ "${HEADLESS:-0}" = "1" ]; then
    [ -n "${ASSIGNED_IDS:-}" ] && ORCH_ARGS+=(--assigned-ids "$ASSIGNED_IDS")
    [ -f "$TARGETS" ] && ORCH_ARGS+=(--truth-json "$TARGETS")
    "${PY[@]}" "${ORCH_ARGS[@]}" --no-dashboard
else
    echo "[hitl] dashboard on http://$DASH_HOST:8765 — enter the assigned id + GO per sortie"
    [ -f "$TARGETS" ] && ORCH_ARGS+=(--truth-json "$TARGETS")
    "${PY[@]}" "${ORCH_ARGS[@]}" --host "$DASH_HOST" --port 8765
fi
rc=$?
echo "[hitl] orchestrator exited (rc=$rc) — stopping the onboard HITL stack."
exit "$rc"
