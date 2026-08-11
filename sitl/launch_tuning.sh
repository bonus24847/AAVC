#!/usr/bin/env bash
# AAVC one-click TUNING launcher (wired to the Desktop/AAVC-Tuning.desktop icon).
#
# A SEPARATE program from the flight mission (sitl/launch_gcs.sh): this brings up
# SITL + the System-ID / Autotune tool ONLY — the mission never runs. Use it
# BEFORE the competition to identify the plant + find good gains; the gains you
# Apply are saved and the flight-mission program auto-loads them.
#
#   PX4 SITL + Gazebo (gz_eft_x6100 hexacopter, HEADLESS) → camera bridge (spectator) →
#   orchestrator(--mode tuning) + dashboard (http://127.0.0.1:8765/?mode=tuning).
#
# No targets are spawned (no vision, no mission), but the camera bridge DOES run
# so the Tuning view's right rail can show the Gazebo spectator camera watching
# the drone sweep. Close this window / Ctrl-C to tear the whole stack back down.
# (Run only ONE of the flight or tuning programs at a time — same MAVLink port.)
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
PY=(env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python")
# See launch_gcs.sh: without this the missing interpreter surfaces as a readiness
# probe that exits instantly, which the boot loop misreads as a slow SITL.
if [ ! -x "$REPO_ROOT/.venv/bin/python" ]; then
    echo "[tune] ERROR: $REPO_ROOT/.venv is missing — run 'make install' first."
    read -rp "[tune] press Enter to close…" _
    exit 1
fi
URL="http://127.0.0.1:8765/?mode=tuning"
HEALTH="http://127.0.0.1:8765/api/health"
SIM_PGID=""

cleanup() {
    echo "[tune] stopping SITL + tuning tool…"
    [ -n "$SIM_PGID" ] && kill -9 -"$SIM_PGID" 2>/dev/null
    pkill -9 -f 'gz[ ]sim.*kmutnb_skyfield' 2>/dev/null
    pkill -9 -f 'px4_sitl_default/bin/px[4]' 2>/dev/null
    pkill -9 -f 'gz_camera_bridg[e]' 2>/dev/null
    pkill -9 -f 'orchestrator.mai[n]' 2>/dev/null
    pkill -9 -f 'mavsdk_serve[r]' 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "============================================================"
echo "  AAVC — System-ID + Autotune (TUNING tool, no mission)"
echo "============================================================"
echo "[tune] clearing any previous run…"
cleanup 2>/dev/null
sleep 2

# Build the Svelte bundle once if it has never been built.
if [ ! -d "$REPO_ROOT/dashboard/web/dist" ]; then
    echo "[tune] building the web bundle (first run only)…"
    ( cd "$REPO_ROOT/dashboard/web" && npm i && npm run build )
fi

# 1) PX4 SITL + Gazebo (HEADLESS), retrying the flaky gz_bridge spawn.
echo "[tune] starting PX4 SITL + Gazebo (this can take a minute, may retry)…"
booted=0
for attempt in 1 2 3 4 5; do
    pkill -9 -f 'gz[ ]sim.*kmutnb_skyfield' 2>/dev/null
    pkill -9 -f 'px4_sitl_default/bin/px[4]' 2>/dev/null
    [ -n "$SIM_PGID" ] && kill -9 -"$SIM_PGID" 2>/dev/null
    rm -f /tmp/aavc_sitl.log
    setsid bash -c 'tail -f /dev/null | HEADLESS=1 PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-v1.17}" make sitl 2>&1 | grep --line-buffered -avE "pxh>|libEGL|dri2 screen|pci id for fd|MESA-LOADER" > /tmp/aavc_sitl.log' &
    SIM_PGID=$!
    # Readiness = the vehicle can actually fly (heartbeat + 3D fix), asked over
    # MAVLink. Grepping the console for "home set" looked equivalent but is not:
    # PX4 only prints it once something connects and interacts, and nothing does
    # until the orchestrator starts — which is after this gate. On a quiet boot
    # that waited forever and then killed a perfectly healthy SITL.
    # 12 x ~6 s ≈ 75 s per attempt, and rc!=1 means the PROBE broke, not SITL
    # (see launch_gcs.sh for both).
    for _ in $(seq 1 12); do
        grep -qaE "Service call timed out|gz_bridge failed to start" /tmp/aavc_sitl.log 2>/dev/null && break
        "${PY[@]}" sitl/wait_sitl_ready.py --timeout 6 --quiet 2>>/tmp/aavc_waitsitl.log
        rc=$?
        [ "$rc" = "0" ] && { booted=1; break; }
        if [ "$rc" != "1" ]; then
            echo "[tune] ERROR: the readiness probe failed (rc=$rc), SITL was not"
            echo "[tune]        the problem. Last lines:"
            tail -n 3 /tmp/aavc_waitsitl.log 2>/dev/null | sed 's/^/[tune]        /'
            read -rp "[tune] press Enter to close…" _
            exit 1
        fi
    done
    [ "$booted" = "1" ] && { echo "[tune] SITL up (attempt $attempt)."; break; }
    echo "[tune] boot attempt $attempt hit the gz_bridge race — retrying…"
done
if [ "$booted" != "1" ]; then
    echo "[tune] ERROR: SITL did not boot after 5 attempts (see /tmp/aavc_sitl.log)."
    read -rp "[tune] press Enter to close…" _
    exit 1
fi
sleep 3

# 2) Camera bridge — feed the Gazebo SPECTATOR camera to /tmp/aavc_spectator.png
#    (+ the onboard cams) so the Tuning view's right rail can show the drone.
#    /usr/bin/python3 is REQUIRED: python3-gz-transport is an apt package, not in
#    the project .venv. Best-effort — the orchestrator still runs if it fails.
echo "[tune] starting the camera bridge (spectator view)…"
rm -f /tmp/aavc_spectator.png
/usr/bin/python3 sitl/gz_camera_bridge.py >/tmp/aavc_bridge.log 2>&1 &

# 3) Orchestrator in TUNING mode + dashboard (NO mission, NO vision).
echo "[tune] starting the tuning tool (orchestrator --mode tuning + dashboard)…"
"${PY[@]}" -m orchestrator.main --config sitl/aavc_config.yaml --mode tuning \
    --host 127.0.0.1 --port 8765 >/tmp/aavc_tuning.log 2>&1 &
ORCH_PID=$!

# 4) Wait for the dashboard, then open the Tuning UI.
echo "[tune] waiting for the tool to come up…"
for _ in $(seq 1 40); do
    curl -sf -o /dev/null "$HEALTH" 2>/dev/null && break
    sleep 1
done
echo "------------------------------------------------------------"
echo "  TUNING tool is LIVE →  $URL"
echo "  Run Sys-ID / Autotune, then Apply (gains are saved for the mission)."
echo "  Close this window (or Ctrl-C) to stop. The mission does NOT run here."
echo "------------------------------------------------------------"
( xdg-open "$URL" >/dev/null 2>&1 || firefox --new-window "$URL" >/dev/null 2>&1 ) &

wait "$ORCH_PID" 2>/dev/null || true
echo "[tune] tool still live at $URL. Close this window to stop."
while true; do sleep 3600; done
