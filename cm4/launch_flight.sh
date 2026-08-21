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
#   CAM_EXPOSURE=20            v4l2 forced exposure, UVC 100 µs units (20 = 2 ms; 0 = auto)
#   CAM_GAIN=64                optional fixed gain 0-128 with CAM_EXPOSURE (OV9281 has no auto-gain)
#   CAM_INTERVAL=0.04          seconds between frame writes (0.04 = 25 Hz with passthrough)
#   CAM_PASSTHROUGH=0          disable MJPEG passthrough (fall back to YUYV + re-encode)
#   CAM_MIRROR=1               also write /tmp/aavc_frame.jpg for the WEB dashboard (costs a
#                              second encode ~12 ms/frame as JPEG; off by default)
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
CAM_EXPOSURE="${CAM_EXPOSURE:-20}"
CONFIG="${CONFIG:-sitl/aavc_config.yaml}"
DASH_HOST="${DASH_HOST:-127.0.0.1}"

INFRA_PIDS=()
cleanup() {
    echo "[flight] stopping onboard stack…"
    pkill -9 -P $$ 2>/dev/null      # kill the keep_alive supervisors we spawned
    for p in "${INFRA_PIDS[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    pkill -9 -f 'camera_grabber.p[y]' 2>/dev/null
    pkill -9 -f 'status_beacon.p[y]' 2>/dev/null
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
# ⚠ DO NOT set connection.raw_telemetry_port=14551 on THIS aircraft (the line
# here used to tell you to — 2026-08-18). The PM03D is out; since 2026-08-20 a
# PM02D feeds the Pixhawk (FC/avionics ONLY) while the motors still run from a
# board it cannot sense, so
# BATTERY_STATUS.current_consumed counts AVIONICS ONLY — measured 0.62 A / 91 mAh
# with the pack on the bench, against the ~35-43 A of flight. Feeding that in
# promotes energy_consumed_mah() from tier B (percent, voltage-derived and
# correct here) to tier A (coulomb count), and the GO gate then reads
# "5534 mAh usable left" on a pack that really has 2250 — it would wave through
# the flight it exists to refuse. Keep the port at 0 until the wiring can
# actually sense motor current; the dashboard only loses cosmetic widgets.
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
# The same "left over from a bench session" problem the camera grabber has
# below, with a worse ending (2026-08-18): a router still holding $SERIAL means
# OURS cannot open the UART, and keep_alive then restarts a process that fails
# forever. There is no error the operator sees on the GCS — just a console that
# never gets telemetry. cleanup() only pkills on EXIT, too late for the run
# starting now, so clear the field first. Same for a stale beacon, which would
# otherwise double every STATUSTEXT on the radio.
if pgrep -f 'mavlink-route[r]d' >/dev/null 2>&1; then
    echo "[flight] a mavlink-router is already running — stopping it so this run owns $SERIAL"
    pkill -9 -f 'mavlink-route[r]d' 2>/dev/null
    sleep 1
fi
pkill -9 -f 'status_beacon.p[y]' 2>/dev/null
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
    rm -f /tmp/aavc_nadir.jpg /tmp/aavc_frame.jpg \
          /tmp/aavc_nadir.png /tmp/aavc_frame.png   # incl. the pre-JPEG names
    # Short-exposure default (mirrors run_mission.sh ensure_infra — this
    # launcher IS the real bird; G7 2026-08-21 in-flight blur). CAM_EXPOSURE=0
    # restores the driver's auto exposure.
    if [ "$CAM_EXPOSURE" != "0" ] && [ "$BACKEND" = "v4l2" ]; then
        GRAB_ARGS="${GRAB_ARGS:+$GRAB_ARGS }--exposure-100us $CAM_EXPOSURE"
        [ -n "${CAM_GAIN:-}" ] && GRAB_ARGS="$GRAB_ARGS --gain $CAM_GAIN"
    fi
    # 10 Hz frames (the sensor's own rate at 1280x720) and no mirror encode —
    # see sitl/run_mission.sh for the measured numbers behind both.
    # MJPEG passthrough by default — see sitl/run_mission.sh for the measured
    # numbers and the decode-survival test behind it.
    if [ "${CAM_PASSTHROUGH:-1}" != "0" ] && [ "$BACKEND" = "v4l2" ]; then
        GRAB_ARGS="${GRAB_ARGS:+$GRAB_ARGS }--mjpeg-passthrough"
        CAM_INTERVAL="${CAM_INTERVAL:-0.04}"     # 25 Hz (see run_mission.sh)
    fi
    GRAB_ARGS="${GRAB_ARGS:+$GRAB_ARGS }--interval-s ${CAM_INTERVAL:-0.1}"
    [ "${CAM_MIRROR:-0}" = "0" ] && GRAB_ARGS="$GRAB_ARGS --no-mirror"
    echo "[flight] camera grabber: backend=$BACKEND args='$GRAB_ARGS'"
    keep_alive "camera-grabber" make camera-real BACKEND="$BACKEND" GRAB_ARGS="$GRAB_ARGS"
    # let the first frames land so the preflight camera-age check passes
    for _ in $(seq 1 20); do [ -f /tmp/aavc_nadir.jpg ] && break; sleep 0.5; done
    [ -f /tmp/aavc_nadir.jpg ] || echo "[flight] WARNING: no nadir frame after 10 s — the camera did NOT start (check /dev/video*, and that nothing else holds it)"
else
    echo "[flight] NO_CAMERA=1 — expecting /tmp/aavc_*.png from a synthetic/gz feeder"
fi

# 2b) radio status beacon — the phase/pad/camera summary as MAVLink STATUSTEXT,
#     so the operator still sees it when WiFi cannot reach the aircraft (the
#     console's own stepper/pad/camera panels all ride WiFi). ~30 B/s. Needs
#     MAV_<i>_FORWARD=1 on the FC's CM4 port for the lines to leave the
#     aircraft; without it this is simply inert. BEACON=0 to skip.
if [ "${BEACON:-1}" = "1" ]; then
    echo "[flight] status beacon: mission + camera -> STATUSTEXT (radio)"
    keep_alive "status-beacon" "${PY[@]}" cm4/status_beacon.py \
        --captures "$REPO_ROOT/captures" --endpoint "udpout:127.0.0.1:14550"
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
