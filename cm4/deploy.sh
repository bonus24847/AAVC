#!/usr/bin/env bash
# AAVC 2026 — laptop → CM4 deployment (rsync the repo, bootstrap the venv remotely).
#
# Usage:
#   cm4/deploy.sh <user@cm4-host> [--install]
#   CM4_DIR=mission cm4/deploy.sh aavc@10.42.0.12 --install
#
# What it does:
#   1) rsync the working tree → <host>:~/${CM4_DIR:-mission} — the path the GCS
#      console's REAL mission-cmd expects (docs/REAL_FLIGHT_GCS.md). Dev/heavy
#      dirs are excluded; the CM4's own .venv and runs/ are never touched.
#   2) with --install: create the venv ON the CM4 (its own aarch64 wheels — the
#      laptop's .venv is deliberately not copied) and `pip install -e .`
#      (flight core only). Needs internet ON THE BENCH — the competition's
#      no-internet rule applies to the flight, not the setup table.
#   3) prints the bench-test commands to run next.
#
# Re-run any time; rsync makes it incremental.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="${CM4_DIR:-mission}"

HOST=""
INSTALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --install) INSTALL=1 ;;
        # deploy ANY mission repo, not just this one (the GUI wizard uses this
        # to put the competition repo on the drone as well)
        --repo) shift; REPO_ROOT="$(cd "$1" && pwd)" ;;
        --dir) shift; DIR="$1" ;;
        -*) echo "unknown flag: $1" >&2; exit 2 ;;
        *) HOST="$1" ;;
    esac
    shift
done
if [ -z "$HOST" ]; then
    echo "usage: cm4/deploy.sh <user@cm4-host> [--install]" >&2
    echo "  e.g. cm4/deploy.sh aavc@10.42.0.12 --install" >&2
    exit 2
fi

# This laptop's CM4 key is NOT one of ssh's default names, so plain
# `ssh drone@…` would prompt for a password (and the GCS 🚀 button would hang
# at the field). Every CM4 path in this repo passes it explicitly.
CM4_KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"
SSH_ID=(); [ -f "$CM4_KEY" ] && SSH_ID=(-i "$CM4_KEY")

echo "[deploy] checking ssh to $HOST …"
if ! ssh "${SSH_ID[@]}" -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new "$HOST" true; then
    echo "[deploy] ERROR: cannot ssh to $HOST without a password." >&2
    echo "         Run: ssh-copy-id ${SSH_ID[*]:+-i $CM4_KEY }$HOST" >&2
    echo "         (the 🚀 button needs the same passwordless path)" >&2
    exit 1
fi

echo "[deploy] rsync → $HOST:~/$DIR"
# The aircraft carries ONLY what it flies with (operator 2026-08-14: "แก้เลย"
# — why ship gz models to a real drone?). Nothing on the CM4 ever opens a
# .sdf/.world: the flight path reads sitl/aavc_config.yaml (mission values)
# and runs sitl/run_mission.sh (the 🚀 entry) — both KEPT, since they live in
# sitl/ next to the simulator-only assets. Dropping ~10 MB of gz models,
# worlds, tests and the dashboard's web build keeps the on-aircraft tree to
# the flight core + its docs, so what's on the drone is auditable at a glance.
# SET_ME=1 SIM_TOO=1 ships everything (e.g. running SITL on a beefy companion).
SIM_EXCLUDES=(
    --exclude 'sitl/models/'      # gz meshes/SDF — simulator geometry only
    --exclude 'sitl/worlds/'      # gz worlds
    --exclude 'sitl/px4_patches/' # PX4 SITL airframe + diff
    --exclude 'tests/'            # run them on the dev machine, not the drone
    --exclude 'dashboard/web/'    # Svelte build; the CM4 flies headless
    --exclude 'docs/evidence/'    # flight-log archives
)
[ "${SIM_TOO:-0}" = "1" ] && SIM_EXCLUDES=()
rsync -az --delete --info=stats1 \
    -e "ssh ${SSH_ID[*]}" \
    --exclude '.git/' \
    --exclude '.venv/' \
    --exclude 'runs/' \
    --exclude 'captures/' \
    --exclude '__pycache__/' \
    --exclude '.mypy_cache/' \
    --exclude '.pytest_cache/' \
    --exclude '.ruff_cache/' \
    --exclude '*.egg-info' \
    --exclude 'dashboard/web/node_modules/' \
    --exclude '*.pdf' \
    "${SIM_EXCLUDES[@]}" \
    "$REPO_ROOT/" "$HOST:$DIR/"

if [ "$INSTALL" -eq 1 ]; then
    echo "[deploy] bootstrapping the venv on the CM4 …"
    REMOTE_CMD=$(printf 'set -e; cd %q; python3 --version; [ -d .venv ] || python3 -m venv .venv; .venv/bin/pip install -q -U pip; .venv/bin/pip install -q -e .; mkdir -p runs captures; env -u PYTHONPATH .venv/bin/python -c %q' \
        "$DIR" \
        'import orchestrator, mission_brain, vision, mavlink_adapter; print("import-clean OK")')
    ssh "${SSH_ID[@]}" "$HOST" "$REMOTE_CMD"
fi

cat <<EOF
[deploy] done — flight tree only (~3 MB: no gz models/worlds, tests or web
         build; SIM_TOO=1 ships those too).

Next, on the bench (props OFF — docs/REAL_FLIGHT_GCS.md + docs/HITL.md):
  # CM4-in-the-loop HITL (jMAVSim on the laptop owns the 6X USB):
  ssh ${SSH_ID[*]} $HOST 'cd $DIR && SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh'
  # real-camera bench stack (dashboard on loopback):
  ssh ${SSH_ID[*]} $HOST 'cd $DIR && bash cm4/launch_flight.sh'
  # GCS console on this laptop: just double-click the desktop icon
  #   "AAVC GCS เครื่องจริง" — it picks the mission, finds the CM4, deploys
  #   if needed and starts console + status_sync (cm4/launch_gcs_real_gui.sh).
EOF
