#!/usr/bin/env bash
# AAVC 2026 — laptop-side REAL-flight console, ONE command (operator request
# 2026-08-13: "บินจริงผมต้องตั้งค่าเอง" — every GCS setting is encoded here so
# nothing has to be hand-typed at the field).
#
#   cm4/launch_gcs_real.sh <user@cm4-host> [console_port=8000]
#   e.g. cm4/launch_gcs_real.sh aavc@10.42.0.12
#
# What it starts (Ctrl-C stops both):
#   1) cm4/status_sync.sh  — pulls captures/ from the CM4 every 2 s so the
#      console's stepper + pad ✓ stay live whenever WiFi reaches the aircraft
#   2) the AAVC GCS console with the REAL-flight settings:
#        --field    gcs/kmutnb_field.yaml    (KMUTNB geofence/pads overlay)
#        --captures captures                 (the dir status_sync fills)
#        --url      udpin:0.0.0.0:14550      (telemetry in — Nomad backpack /
#                                             mavlink-router aim HERE)
#        --mission-cmd  ssh → run_mission.sh (REAL=1 ⇒ RC-GO: 🚀 only STAGES;
#                                             the safety pilot's RC launches)
#        --mission-label REAL                (badge on the 🚀 button)
#        (no --reset-cmd ⇒ the SIM-only 🧹 button hides itself)
#
# Prereqs (docs/REAL_FLIGHT_GCS.md): repo deployed with cm4/deploy.sh,
# passwordless ssh (ssh-copy-id), Nomad bench checklist passed.
set -uo pipefail

HOST="${1:?usage: cm4/launch_gcs_real.sh <user@cm4-host> [console_port]}"
PORT="${2:-8000}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GCS="${AAVC_GCS:-$HOME/Desktop/aavc-gcs/src/aavc_gcs.py}"
DIR="${CM4_DIR:-mission}"

[ -f "$GCS" ] || { echo "ERROR: console not found at $GCS (set AAVC_GCS=…)" >&2; exit 1; }

# This laptop's CM4 key has a non-default name, so it must be named
# explicitly — including INSIDE the mission-cmd the console spawns, or the
# 🚀 button hits a password prompt and hangs with no visible error.
CM4_KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"
SSH_ID=(); SSH_ID_STR=""
if [ -f "$CM4_KEY" ]; then SSH_ID=(-i "$CM4_KEY"); SSH_ID_STR="-i $CM4_KEY "; fi

# The 🚀 button needs passwordless ssh — fail loudly NOW, not at GO time.
if ! ssh "${SSH_ID[@]}" -o ConnectTimeout=5 -o BatchMode=yes "$HOST" true; then
    echo "ERROR: ssh $HOST still asks for a password — run: ssh-copy-id ${SSH_ID_STR}$HOST" >&2
    exit 1
fi

echo "[real-gcs] status sync + console → http://127.0.0.1:$PORT  (Ctrl-C stops both)"
bash "$REPO_ROOT/cm4/status_sync.sh" "$HOST" "$DIR" &
SYNC_PID=$!
trap 'kill "$SYNC_PID" 2>/dev/null' EXIT INT TERM

/usr/bin/python3 "$GCS" \
    --field "$REPO_ROOT/gcs/kmutnb_field.yaml" \
    --captures "$REPO_ROOT/captures" \
    --url "udpin:0.0.0.0:14550" \
    --mission-cmd "ssh ${SSH_ID_STR}$HOST 'REAL=1 ~/$DIR/sitl/run_mission.sh {ids}'" \
    --mission-label REAL \
    --port "$PORT"
