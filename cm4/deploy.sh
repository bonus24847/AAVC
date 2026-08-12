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
for arg in "$@"; do
    case "$arg" in
        --install) INSTALL=1 ;;
        -*) echo "unknown flag: $arg" >&2; exit 2 ;;
        *) HOST="$arg" ;;
    esac
done
if [ -z "$HOST" ]; then
    echo "usage: cm4/deploy.sh <user@cm4-host> [--install]" >&2
    echo "  e.g. cm4/deploy.sh aavc@10.42.0.12 --install" >&2
    exit 2
fi

echo "[deploy] checking ssh to $HOST …"
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true; then
    echo "[deploy] ERROR: cannot ssh to $HOST without a password." >&2
    echo "         Run: ssh-copy-id $HOST   (the 🚀 button needs passwordless ssh too)" >&2
    exit 1
fi

echo "[deploy] rsync → $HOST:~/$DIR"
rsync -az --delete --info=stats1 \
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
    "$REPO_ROOT/" "$HOST:$DIR/"

if [ "$INSTALL" -eq 1 ]; then
    echo "[deploy] bootstrapping the venv on the CM4 …"
    REMOTE_CMD=$(printf 'set -e; cd %q; python3 --version; [ -d .venv ] || python3 -m venv .venv; .venv/bin/pip install -q -U pip; .venv/bin/pip install -q -e .; mkdir -p runs captures; env -u PYTHONPATH .venv/bin/python -c %q' \
        "$DIR" \
        'import orchestrator, mission_brain, vision, mavlink_adapter; print("import-clean OK")')
    ssh "$HOST" "$REMOTE_CMD"
fi

cat <<EOF
[deploy] done.

Next, on the bench (props OFF — docs/REAL_FLIGHT_GCS.md + docs/HITL.md):
  # CM4-in-the-loop HITL (jMAVSim on the laptop owns the 6X USB):
  ssh $HOST 'cd $DIR && SERIAL=/dev/ttyAMA0 bash cm4/launch_hitl.sh'
  # real-camera bench stack (dashboard on loopback):
  ssh $HOST 'cd $DIR && bash cm4/launch_flight.sh'
  # GCS console on this laptop, REAL mode (no 🧹 button):
  /usr/bin/python3 ~/Desktop/aavc-gcs/src/aavc_gcs.py \\
    --field gcs/kmutnb_field.yaml --captures captures \\
    --url udpin:0.0.0.0:14550 \\
    --mission-cmd "ssh $HOST 'REAL=1 ~/$DIR/sitl/run_mission.sh {ids}'" \\
    --mission-label REAL
EOF
