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
while true; do
    rsync -az --timeout=4 -e "ssh -o ConnectTimeout=3 -o BatchMode=yes" \
        "$HOST:$DIR/captures/" "$REPO_ROOT/captures/" 2>/dev/null || true
    sleep "$IVL"
done
