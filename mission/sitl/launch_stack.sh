#!/usr/bin/env bash
# Debug/bring-up harness — everything EXCEPT the mission, in one command.
#
#   bash sitl/launch_stack.sh            # headless SITL + bridges + GCS + pads
#   GUI=1 bash sitl/launch_stack.sh      # same, with the Gazebo viewer
#   SEED=42 bash sitl/launch_stack.sh    # seeded pad layout (default: $RANDOM)
#   bash sitl/launch_stack.sh stop       # tear the whole stack down
#
# Codifies the hand-run bring-up used during the 2026-08-12 verification
# sessions, including every trap discovered along the way:
#   * pkill patterns use [b]racket self-match guards — a plain
#     `pkill -f 'gz sim'` matches the SHELL RUNNING THIS SCRIPT (its cmdline
#     contains the pattern) and kills the bring-up mid-flight.
#   * bridges run on the venv python WITH the apt dist-packages appended
#     (gz-transport13 is apt-only, cv2 is venv-only on this host) and -u so
#     their logs stream live into /tmp.
#   * the AAVC GCS console is pinned to THIS repo's field + captures dirs
#     (stale-pad fix) and falls back 8000→8020 when the port is taken.
#   * readiness = MAVLink heartbeat + 3D fix (wait_sitl_ready), retried
#     across the known gz_bridge spawn race.
#
# Logs (tail -f these when debugging):
#   /tmp/aavc_sitl.log      PX4 + gz server console
#   /tmp/aavc_bridge.log    camera bridge (nadir/spectator → /tmp/aavc_*.png)
#   /tmp/aavc_detach.log    payload-detach bridge (servo watch + sheds)
#   /tmp/aavc_gcs_console.log  AAVC GCS console (http://127.0.0.1:<port>)
#   /tmp/aavc_spawn.log     pad spawn + assignable ids
#
# After the stack is up:  make run TRUTH=/tmp/aavc_targets.json  (or add
# --assigned-ids via ARGS) flies the mission on top of it.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
PY="$REPO_ROOT/.venv/bin/python"
BRIDGE_PY=(env PYTHONPATH=/usr/lib/python3/dist-packages "$PY")
AAVC_GCS="${AAVC_GCS:-$HOME/Desktop/aavc-gcs/src/aavc_gcs.py}"

# Is somebody ELSE's SITL running on this machine? (2026-08-15: this script
# pkill'd `gz sim` and px4 by pattern, with no idea who owned them, and killed a
# parallel session's simulator mid-flight. The two projects share this laptop,
# these ports and this PX4 build, so "kill everything that matches" is a booby
# trap that goes off every time both are working.) A gz world outside THIS repo
# is the tell: ours always comes from $REPO_ROOT/sitl/worlds.
foreign_sitl() {
    # gz has the world right there in the command line — but match its NAME,
    # not its directory. launch_sitl.sh COPIES our world into PX4's own
    # Tools/simulation/gz/worlds/ (so PX4's resource lookup finds the current
    # one), and gz is then started from THAT path — never from $REPO_ROOT. A
    # directory test therefore called our own simulator foreign and made
    # `stop` refuse to stop it (2026-08-16). The basename is the honest
    # discriminator, and it is the same one the PX4 branch below already
    # trusts via PX4_GZ_WORLD.
    pgrep -af "gz sim" 2>/dev/null \
        | grep -v "pgrep" \
        | grep -oE "/[^ ]*\.sdf" \
        | grep -v "/kmutnb_skyfield\.sdf$" || true
    # PX4 is NOT: both projects run the same binary out of the same shared
    # worktree, so the command lines are byte-identical and no pattern can tell
    # them apart (the parallel session hit this from the other side — their
    # launcher pkill'd that path and took ours down 8+ times today). Its
    # ENVIRONMENT can: launch_sitl.sh exports PX4_GZ_WORLD, so anything not
    # flying our world belongs to somebody else. Credit: mission_AAVC session.
    local pid world
    for pid in $(pgrep -f "px4_sitl_default/bin/px[4]" 2>/dev/null); do
        world="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
                 | sed -n 's/^PX4_GZ_WORLD=//p' | head -1)"
        [ -n "$world" ] && [ "$world" != "kmutnb_skyfield" ] \
            && echo "px4 pid $pid flying world '$world'"
    done
    return 0
}

stack_stop() {
    local foreign
    foreign="$(foreign_sitl | head -3)"
    if [ -n "$foreign" ] && [ "${FORCE_KILL:-0}" != "1" ]; then
        echo "[stack] REFUSING to stop: a simulator from another project is running" >&2
        echo "$foreign" | sed 's/^/[stack]   /' >&2
        echo "[stack] killing it would take down someone else's flight mid-air." >&2
        echo "[stack] Coordinate first; FORCE_KILL=1 overrides if you are sure." >&2
        exit 3
    fi
    echo "[stack] stopping everything…"
    pkill -9 -f 'gz [s]im'                    2>/dev/null
    pkill -9 -f 'px4_sitl_default/bin/px[4]'  2>/dev/null
    pkill -9 -f 'launch_sitl.s[h]'            2>/dev/null
    pkill -9 -f 'gz_camera_bridg[e]'          2>/dev/null
    pkill -9 -f 'payload_detach_bridg[e]'     2>/dev/null
    # KEEP_CONSOLE=1 (the GCS 🧹 field-reset path): the reset is SPAWNED BY
    # the console — killing it here would cut the operator's own page dead.
    if [ "${KEEP_CONSOLE:-0}" != "1" ]; then
        pkill -9 -f 'aavc_gcs.p[y]'           2>/dev/null
    fi
    pkill -9 -f 'mavsdk_serve[r]'             2>/dev/null
    pkill -9 -f 'orchestrator.mai[n]'         2>/dev/null
    sleep 2
    echo "[stack] done."
}

if [ "${1:-}" = "stop" ]; then
    stack_stop
    exit 0
fi

if [ ! -x "$PY" ]; then
    echo "[stack] ERROR: .venv missing — run 'make install' first." >&2
    exit 1
fi

stack_stop

# A fresh world starts calm (kmutnb_skyfield.sdf ships <linear_velocity>0 0 0),
# so any wind breadcrumb from a previous session is now false. Clearing it here
# is the whole point: the restore is a SCRIPT's job, not somebody's memory.
rm -f /tmp/aavc_wind_state

# 1) SITL (+ optional viewer). tail keeps stdin open — a TTY-less PX4 pxh
#    console otherwise spins at 100% CPU spraying prompts into the log.
echo "[stack] starting PX4 SITL + Gazebo (GUI=${GUI:-0})…"
setsid bash -c 'tail -f /dev/null | HEADLESS='"$([ "${GUI:-0}" = "1" ] && echo 0 || echo 1)"' GUI='"${GUI:-0}"' make sitl 2>&1 \
    | grep --line-buffered -avE "pxh>|libEGL|dri2 screen|pci id for fd|MESA-LOADER" > /tmp/aavc_sitl.log' &

# 2) readiness (heartbeat + 3D fix), riding out the gz_bridge spawn race.
ready=0
for _ in 1 2 3 4 5; do
    "$PY" sitl/wait_sitl_ready.py --timeout 30 --quiet && { ready=1; break; }
done
if [ "$ready" != "1" ]; then
    echo "[stack] ERROR: SITL never became ready — see /tmp/aavc_sitl.log" >&2
    exit 1
fi
echo "[stack] SITL ready (heartbeat + 3D fix)."

# 3) bridges — live logs (-u), venv python + apt dist-packages.
"${BRIDGE_PY[@]}" -u sitl/gz_camera_bridge.py            >/tmp/aavc_bridge.log 2>&1 &
"${BRIDGE_PY[@]}" -u sitl/payload_detach_bridge.py --servo --model eft_x6100 \
                                                         >/tmp/aavc_detach.log 2>&1 &
echo "[stack] bridges up (camera + payload servo watch)."

# 4) AAVC GCS console on this repo's field/captures (stale-pad fix).
#    KEEP_CONSOLE=1 (the 🧹 reset path) leaves the running console untouched.
GCS_PORT="${GCS_PORT:-8000}"
if [ "${KEEP_CONSOLE:-0}" = "1" ]; then
    echo "[stack] console kept alive (KEEP_CONSOLE=1 — GCS 🧹 field reset)"
elif [ -f "$AAVC_GCS" ]; then
    if ss -tln 2>/dev/null | grep -q ":${GCS_PORT} "; then
        echo "[stack] port ${GCS_PORT} busy → 8020"
        GCS_PORT=8020
    fi
    /usr/bin/python3 "$AAVC_GCS" --field "$REPO_ROOT/gcs/kmutnb_field.yaml" \
        --captures "$REPO_ROOT/captures" --url udpin:0.0.0.0:14550 \
        --mission-cmd "'$REPO_ROOT/sitl/run_mission.sh' {ids}" \
        --reset-cmd "env KEEP_CONSOLE=1 GUI=${GUI:-0} bash '$REPO_ROOT/sitl/launch_stack.sh'" \
        --port "$GCS_PORT" >/tmp/aavc_gcs_console.log 2>&1 &
    echo "[stack] AAVC GCS console → http://127.0.0.1:${GCS_PORT}"
else
    echo "[stack] WARNING: $AAVC_GCS not found — console skipped"
fi

# 5) pads (seeded unless SEED= given; ids echoed for --assigned-ids).
sleep 2
"$PY" sitl/spawn_targets.py --config sitl/aavc_config.yaml \
    --seed "${SEED:-$RANDOM}" 2>&1 | tee /tmp/aavc_spawn.log | tail -n 7
IDS=$(grep -oE 'ArUco id [0-9]+' /tmp/aavc_spawn.log | grep -oE '[0-9]+$' | tr '\n' ',' | sed 's/,$//')

echo "------------------------------------------------------------"
echo "[stack] READY — pads on the field: ${IDS:-?}"
echo "[stack] fly:   make run TRUTH=/tmp/aavc_targets.json"
echo "[stack]        (.venv/bin/python -m orchestrator.main … --assigned-ids \"<4 of ${IDS:-…}>\")"
echo "[stack] stop:  bash sitl/launch_stack.sh stop"
echo "------------------------------------------------------------"
