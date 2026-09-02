#!/usr/bin/env bash
# AAVC one-click GCS launcher (wired to the Desktop/AAVC-GCS.desktop icon).
#
# Brings up the whole SITL stack and the live GCS, then opens it in a browser:
#   PX4 SITL + Gazebo (gz_eft_x6100 hexacopter, HEADLESS) → camera bridge → 6 ArUco pads
#   (4 committee-assigned, 2 permanent distractors) →
#   orchestrator + GCS dashboard (http://127.0.0.1:8765) → browser.
#
# Boot takes ~1-2 min and the gz_bridge model-spawn occasionally times out (a
# known gz-sim 8 startup race), so the SITL launch is retried until it reports
# "home set". Close this window / Ctrl-C to tear the whole stack back down.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
PY=(env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python")
# Checked before anything is started. Without it the first symptom is the
# readiness probe exiting instantly, which the boot loop below would read as
# "SITL isn't up yet" — five kill-and-retry cycles later the operator is told
# Gazebo failed, on a machine that was only ever missing `make install`.
if [ ! -x "$REPO_ROOT/.venv/bin/python" ]; then
    echo "[gcs] ERROR: $REPO_ROOT/.venv is missing — run 'make install' first."
    read -rp "[gcs] press Enter to close…" _
    exit 1
fi
URL="http://127.0.0.1:8765/?mode=flight"
HEALTH="http://127.0.0.1:8765/api/health"
SIM_PGID=""

cleanup() {
    echo "[gcs] stopping SITL + GCS…"
    [ -n "$SIM_PGID" ] && kill -9 -"$SIM_PGID" 2>/dev/null
    pkill -9 -f 'gz[ ]sim.*kmutnb_skyfield' 2>/dev/null
    pkill -9 -f 'px4_sitl_default/bin/px[4]' 2>/dev/null
    pkill -9 -f 'gz_camera_bridg[e]' 2>/dev/null
    pkill -9 -f 'payload_detach_bridg[e]' 2>/dev/null
    pkill -9 -f 'aavc_gcs.p[y]' 2>/dev/null
    pkill -9 -f 'orchestrator.mai[n]' 2>/dev/null
    pkill -9 -f 'mavsdk_serve[r]' 2>/dev/null
    # NO `rm /tmp/px4_lock-0`: PX4's instance lock is an fcntl record lock, so
    # the kernel drops it the moment the holder dies — killing px4 above IS the
    # release. Deleting the file only matters when a px4 is still ALIVE holding
    # it, and then it is actively harmful: the next px4 gets a fresh unlocked
    # inode and boots as a SECOND instance-0, two vehicles in one world on the
    # same MAVLink ports.
}
trap cleanup EXIT INT TERM

echo "============================================================"
echo "  AAVC — Flight Mission GCS (the scored sortie)"
echo "============================================================"
echo "[gcs] clearing any previous run…"
cleanup 2>/dev/null
# Drop last run's ground-truth target layout so a failed spawn can't fly stale
# coords — spawn_targets rewrites it below (fresh random seed each run).
rm -f /tmp/aavc_targets.json
sleep 2

# Build the Svelte GCS bundle once if it has never been built.
if [ ! -d "$REPO_ROOT/dashboard/web/dist" ]; then
    echo "[gcs] building the GCS web bundle (first run only)…"
    ( cd "$REPO_ROOT/dashboard/web" && npm i && npm run build )
fi

# 1) PX4 SITL + Gazebo (HEADLESS), retrying the flaky gz_bridge spawn.
echo "[gcs] starting PX4 SITL + Gazebo (this can take a minute, may retry)…"
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
    #
    # 12 x ~6 s ≈ 75 s per attempt. The probe BLOCKS for its timeout, so the
    # iteration count is the whole budget — at the old 30 a failure mode that
    # prints neither string below (a wedged dGPU, an un-converged EKF) hung the
    # launcher ~16 minutes before saying anything.
    for _ in $(seq 1 12); do
        grep -qaE "Service call timed out|gz_bridge failed to start" /tmp/aavc_sitl.log 2>/dev/null && break
        "${PY[@]}" sitl/wait_sitl_ready.py --timeout 6 --quiet 2>>/tmp/aavc_waitsitl.log
        rc=$?
        [ "$rc" = "0" ] && { booted=1; break; }
        # rc 1 is an honest "not ready yet". Anything else means the PROBE could
        # not run — a held port, a broken venv — and retrying it 60 more times
        # while SIGKILLing a healthy SITL only hides the real error.
        if [ "$rc" != "1" ]; then
            echo "[gcs] ERROR: the readiness probe failed (rc=$rc), SITL was not"
            echo "[gcs]        the problem. Last lines:"
            tail -n 3 /tmp/aavc_waitsitl.log 2>/dev/null | sed 's/^/[gcs]        /'
            read -rp "[gcs] press Enter to close…" _
            exit 1
        fi
    done
    [ "$booted" = "1" ] && { echo "[gcs] SITL up (attempt $attempt)."; break; }
    echo "[gcs] boot attempt $attempt hit the gz_bridge race — retrying…"
done
if [ "$booted" != "1" ]; then
    echo "[gcs] ERROR: SITL did not boot after 5 attempts (see /tmp/aavc_sitl.log)."
    read -rp "[gcs] press Enter to close…" _
    exit 1
fi
sleep 3   # let the camera sensors finish publishing

# 2) Targets + camera bridge.
echo "[gcs] spawning the 6 ArUco landing pads (fresh random ids + layout)…"
# pipefail (set above) makes this detect a spawn failure through the tee. A
# partial/failed spawn used to be swallowed (|| true) and the mission flew into
# an incomplete field with no truth — halt instead.
if ! "${PY[@]}" sitl/spawn_targets.py --config sitl/aavc_config.yaml \
        --seed "$RANDOM" 2>&1 | tee /tmp/aavc_spawn.log; then
    echo "[gcs] ERROR: pad spawn failed — the field is INCOMPLETE (see /tmp/aavc_spawn.log)."
    read -rp "[gcs] press Enter to close…" _
    exit 1
fi
# Surface the assignable ids so the operator (playing committee) queues only
# ids that are ACTUALLY on the field (the 4-of-6 mission-queue editor).
IDS=$(grep -oE 'ArUco id [0-9]+' /tmp/aavc_spawn.log | grep -oE '[0-9]+$' | sort -n | tr '\n' ' ')
echo "============================================================"
echo "  Pads on the field — assignable ArUco ids:  ${IDS:-<none?>}"
echo "  In the pre-flight card, click ONLY these ids into the"
echo "  mission queue (in sortie order), then GO once per sortie."
echo "============================================================"
# Bridges: venv python + apt dist-packages appended (gz-transport13 is
# apt-level, cv2 is venv-level on this host — see Makefile BRIDGE_PY).
BRIDGE_PY=(env PYTHONPATH=/usr/lib/python3/dist-packages "$REPO_ROOT/.venv/bin/python")
echo "[gcs] starting the camera bridge…"
"${BRIDGE_PY[@]}" sitl/gz_camera_bridge.py >/tmp/aavc_bridge.log 2>&1 &
# Payload-detach bridge in SERVO mode: every release — the orchestrator's
# autonomous drop_payload AND the AAVC GCS "ปล่อย servo" buttons — arrives as
# DO_SET_ACTUATOR, shows on gz /model/eft_x6100[_0]/servo_0..3, and sheds the
# matching cargo box. No audit file needed on this path (audit-tail mode
# remains available: make payload-bridge RUN=runs/<id>/audit.jsonl).
echo "[gcs] starting the payload-detach bridge (servo mode)…"
"${BRIDGE_PY[@]}" sitl/payload_detach_bridge.py --servo --model eft_x6100 \
    >/tmp/aavc_detach.log 2>&1 &

# AAVC GCS console (user-mandated telemetry + manual servo release UI) on
# udp 14550 / http 8000 — see gcs/kmutnb_field.yaml. Missing checkout is
# non-fatal: the mission dashboard on 8765 still runs the sortie.
AAVC_GCS="${AAVC_GCS:-$HOME/Desktop/aavc-gcs/src/aavc_gcs.py}"
if [ -f "$AAVC_GCS" ]; then
    echo "[gcs] starting the AAVC GCS console (http://127.0.0.1:8000)…"
    # --captures = THIS repo's captures/: the console's map pads come from our
    # orchestrator's live mission_status.json (pads appear as they are
    # scanned), never a sibling project's stale file.
    /usr/bin/python3 "$AAVC_GCS" --field "$REPO_ROOT/gcs/kmutnb_field.yaml" \
        --captures "$REPO_ROOT/captures" \
        --url udpin:0.0.0.0:14550 --port 8000 >/tmp/aavc_gcs_console.log 2>&1 &
else
    echo "[gcs] WARNING: $AAVC_GCS not found — skipping the AAVC GCS console"
fi

# 3) Orchestrator + GCS dashboard (flies the multi-flight egg delivery —
#    briefing default eggs_aboard=4 means one flight serves all four
#    deliveries; the operator sets the 4-of-6 mission queue in the
#    pre-flight card — GO per flight then confirms the eggs aboard; a
#    manual id only overrides the WHOLE flight at eggs_aboard=1 or when the
#    queue has no chunk for it — otherwise the queue's chunk wins).
echo "[gcs] starting the orchestrator + GCS dashboard…"
"${PY[@]}" -m orchestrator.main --config sitl/aavc_config.yaml \
    --truth-json /tmp/aavc_targets.json \
    --host 127.0.0.1 --port 8765 >/tmp/aavc_run.log 2>&1 &
ORCH_PID=$!

# 4) Wait for the dashboard, then open the browser.
echo "[gcs] waiting for the GCS to come up…"
for _ in $(seq 1 40); do
    curl -sf -o /dev/null "$HEALTH" 2>/dev/null && break
    sleep 1
done
echo "------------------------------------------------------------"
echo "  GCS is LIVE →  $URL"
echo "  AAVC GCS console (telemetry + servo) →  http://127.0.0.1:8000"
echo "  Opening the mission dashboard in your browser now."
echo "  Close this window (or Ctrl-C) to stop SITL + the GCS."
echo "------------------------------------------------------------"
( xdg-open "$URL" >/dev/null 2>&1 || firefox --new-window "$URL" >/dev/null 2>&1 ) &

# Stay alive (so the trap tears everything down on window close). The mission
# runs once; the GCS keeps serving the final state until you close this window.
wait "$ORCH_PID" 2>/dev/null || true
echo "[gcs] mission finished — GCS still live at $URL. Close this window to stop."
while true; do sleep 3600; done
