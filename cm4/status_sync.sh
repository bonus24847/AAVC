#!/usr/bin/env bash
# AAVC 2026 — laptop-side status puller for REAL flights.
#
# The orchestrator runs ON the CM4 and writes captures/mission_status.json
# THERE — the AAVC GCS console on the laptop reads its own local captures/.
# This loop pulls the CM4's captures/ over ssh so the console's phase stepper
# and pad ✓ ticks stay live whenever WiFi reaches the aircraft.
#
# Link-loss tolerant BY DESIGN: out of WiFi range every pull just fails and
# retries — the console's 45 s staleness gate grays the readout, and it snaps
# back the moment the aircraft is back in range (e.g. after landing at L&R).
# The mission itself never depends on this link.
#
# Usage: cm4/status_sync.sh <user@cm4-host> [remote_dir=mission] [interval_s=2]
set -u

HOST="${1:?usage: cm4/status_sync.sh <user@cm4-host> [remote_dir] [interval_s]}"
DIR="${2:-mission}"
IVL="${3:-2}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mkdir -p "$REPO_ROOT/captures"
echo "[status-sync] $HOST:~/$DIR/captures/ -> $REPO_ROOT/captures/ every ${IVL}s (Ctrl-C to stop)"
SSH_OPTS=(-o ConnectTimeout=3 -o BatchMode=yes)
while true; do
    # 1) PUSH the operator-drawn field (GCS map editor) up to the CM4 — the
    #    aircraft must fly exactly what the console shows. When the operator
    #    reverts (file deleted locally), delete it remotely too; doing this
    #    BEFORE the pull stops the old remote copy resurrecting locally.
    if [ -f "$REPO_ROOT/captures/field_override.json" ]; then
        rsync -az --timeout=4 -e "ssh ${SSH_OPTS[*]}" \
            "$REPO_ROOT/captures/field_override.json" \
            "$HOST:$DIR/captures/" 2>/dev/null || true
    else
        ssh "${SSH_OPTS[@]}" "$HOST" \
            "rm -f '$DIR/captures/field_override.json'" 2>/dev/null || true
    fi
    # 2) PULL the mission's live status home
    rsync -az --timeout=4 -e "ssh ${SSH_OPTS[*]}" \
        "$HOST:$DIR/captures/" "$REPO_ROOT/captures/" 2>/dev/null || true
    sleep "$IVL"
done
