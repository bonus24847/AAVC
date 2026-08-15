#!/usr/bin/env bash
# AAVC 2026 — CM4 onboard FLIGHT launcher (real Pixhawk 6X, headless, offline).
#
# Brings up the onboard stack IN ORDER and tears it all down on exit:
#   1) mavlink-router   6X serial -> UDP (:14540 orchestrator, :14550 a ground QGC)
#   2) camera grabber   real nadir -> /tmp/aavc_*.png  (skip with NO_CAMERA=1)
#   3) orchestrator     the blind search-and-serve mission (--connect, --config)
#
# The INFRASTRUCTURE (router + camera) is kept alive (restart on crash). The
# ORCHESTRATOR runs the mission ONCE and is NOT auto-restarted — re-running it would
# re-arm and re-fly. If the CM4/orchestrator dies mid-flight, the FC-level failsafes
# (geofence RTL, datalink-loss RTL, battery RTH) are the net. No internet is used.
#
# Bench (G5/G6): default keeps the dashboard up (loopback) so you GO + watch + kill.
# Field  (G7/G8): HEADLESS=1 runs --no-dashboard (auto-GO when preflight criticals pass).
#
# Env overrides:
#   SERIAL=/dev/ttyACM0        6X device — USB CDC (/dev/ttyACM0) or UART (/dev/ttyAMA0)
#   BAUD=921600
#   CONNECT=udpin://0.0.0.0:14540   orchestrator MAVLink endpoint (matches the router)
#   ROUTER_CONF=<path>         mavlink-router config (default: generated to /tmp)
#   ROUTERD=mavlink-routerd    router binary (or an absolute path to a local build)
#   BACKEND=v4l2|picamera2     camera backend (default v4l2)
#   GRAB_ARGS="..."            extra camera-grabber flags (--nadir-device, --fourcc GREY, --fps, …)
#   CONFIG=sitl/aavc_config.yaml
#   NO_CAMERA=1                skip the grabber (bench test with synthetic/gz frames)
#   HEADLESS=1                 run --no-dashboard (auto-GO; unattended field runs)
#   DASH_HOST=127.0.0.1        dashboard bind host (use 0.0.0.0 ONLY behind your own auth)
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1
PY=(env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python")

SERIAL="${SERIAL:-/dev/ttyACM0}"
BAUD="${BAUD:-921600}"
CONNECT="${CONNECT:-udpin://0.0.0.0:14540}"
ROUTERD="${ROUTERD:-mavlink-routerd}"
BACKEND="${BACKEND:-v4l2}"
GRAB_ARGS="${GRAB_ARGS:-}"
CONFIG="${CONFIG:-sitl/aavc_config.yaml}"
DASH_HOST="${DASH_HOST:-127.0.0.1}"

INFRA_PIDS=()
cleanup() {
    echo "[flight] stopping onboard stack…"
    pkill -9 -P $$ 2>/dev/null      # kill the keep_alive supervisors we spawned
    for p in "${INFRA_PIDS[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    pkill -9 -f 'camera_grabber.p[y]' 2>/dev/null
    pkill -9 -f 'orchestrator.mai[n]' 2>/dev/null
    pkill -9 -f 'mavsdk_serve[r]' 2>/dev/null
    pkill -9 -f 'mavlink-route[r]d' 2>/dev/null   # reparented grandchild of keep_alive
}
trap cleanup EXIT INT TERM

if [ ! -e "$SERIAL" ]; then
    echo "[flight] ERROR: 6X serial $SERIAL not found. Plug in the FC or set SERIAL=…"
    exit 1
fi
command -v "$ROUTERD" >/dev/null 2>&1 || { echo "[flight] ERROR: $ROUTERD not on PATH (build/install mavlink-router or set ROUTERD=<path>)"; exit 1; }

# keep_alive: run a long-lived infra process, restart it ~2s after any crash, until
# this launcher exits. Used for the router + camera grabber (not the mission).
keep_alive() {
    local name="$1"; shift
    ( while true; do
        echo "[flight] start $name"
        "$@"
        echo "[flight] $name exited (rc=$?) — restarting in 2s"
        sleep 2
      done ) &
    INFRA_PIDS+=("$!")
}

# 1) mavlink-router — generate a minimal config if none supplied.
ROUTER_CONF="${ROUTER_CONF:-}"
if [ -z "$ROUTER_CONF" ]; then
    ROUTER_CONF="/tmp/aavc_flight_router.conf"
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
[UdpEndpoint raw]
# Dashboard ESC/servo/consumed-mAh listener (orchestrator RawMavlinkSubscriber).
# Match connection.raw_telemetry_port=14551 in the config for real-CM4 flight.
Mode = Normal
Address = 127.0.0.1
Port = 14551
[UdpEndpoint qgc]
Mode = Server
# S6: default LOOPBACK — do NOT expose an unauthenticated MAVLink control plane
# to the field LAN/Wi-Fi (MAVLink has no auth; anyone could command the armed
# aircraft). Set GCS_HOST=0.0.0.0 (or the CM4's LAN IP) to open it for a ground
# QGC — a deliberate, informed opt-in.
Address = ${GCS_HOST:-127.0.0.1}
Port = 14550
EOF
fi
echo "[flight] mavlink-router: $SERIAL@$BAUD -> :14540 (orchestrator) + :14550 (QGC @ ${GCS_HOST:-127.0.0.1})"
keep_alive "mavlink-router" "$ROUTERD" -c "$ROUTER_CONF"

# 2) camera grabber (real cameras). picamera2 needs system python3; the Makefile
#    target picks the interpreter by BACKEND. Skip for synthetic/gz-fed bench runs.
if [ "${NO_CAMERA:-0}" != "1" ]; then
    # A grabber left over from a bench preview (or a previous stack that died
    # without running cleanup()) still OWNS /dev/video0, so ours would fail with
    # "v4l2 device did not open" — seen live 2026-08-15. cleanup() only pkills on
    # EXIT, which is too late to help the run that is starting, and the
    # frame-file wait below cannot catch it either: the stale file already
    # exists, so the check passes and the mission flies on a frozen frame
    # (the vision worker then rejects every frame on age and sees no pads).
    if pgrep -f 'camera_grabber.p[y]' >/dev/null 2>&1; then
        echo "[flight] a camera grabber is already running — stopping it so this run owns the camera"
        pkill -9 -f 'camera_grabber.p[y]' 2>/dev/null
        sleep 1
    fi
    rm -f /tmp/aavc_nadir.png /tmp/aavc_frame.png   # never inherit a stale frame
    echo "[flight] camera grabber: backend=$BACKEND args='$GRAB_ARGS'"
    keep_alive "camera-grabber" make camera-real BACKEND="$BACKEND" GRAB_ARGS="$GRAB_ARGS"
    # let the first frames land so the preflight camera-age check passes
    for _ in $(seq 1 20); do [ -f /tmp/aavc_nadir.png ] && break; sleep 0.5; done
    [ -f /tmp/aavc_nadir.png ] || echo "[flight] WARNING: no nadir frame after 10 s — the camera did NOT start (check /dev/video*, and that nothing else holds it)"
else
    echo "[flight] NO_CAMERA=1 — expecting /tmp/aavc_*.png from a synthetic/gz feeder"
fi

# 3) orchestrator — the mission, ONCE (not restarted). Headless field run vs
#    bench run with the dashboard for the operator GO + an in-browser kill.
echo "[flight] orchestrator: --connect $CONNECT --config $CONFIG"
if [ "${HEADLESS:-0}" = "1" ]; then
    "${PY[@]}" -m orchestrator.main --config "$CONFIG" --connect "$CONNECT" --no-dashboard
else
    echo "[flight] dashboard on http://$DASH_HOST:8765 — operator GO via /api/cmd/preflight/go"
    "${PY[@]}" -m orchestrator.main --config "$CONFIG" --connect "$CONNECT" \
        --host "$DASH_HOST" --port 8765
fi
rc=$?
echo "[flight] orchestrator exited (rc=$rc) — stopping onboard stack."
exit "$rc"
