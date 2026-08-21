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
# It ALSO pulls the live nadir frame (2026-08-15): the camera grabber writes
# /tmp/aavc_nadir.jpg ON THE CM4, but the console reads /tmp/aavc_nadir.jpg on
# ITS OWN machine (hard-coded in aavc_gcs.py — no flag to point it elsewhere),
# and captures/ never carried that file. So a perfectly working camera showed
# up on the console as "no camera" forever. The frame is ~1 MB, so it is pulled
# every CAM_EVERY-th loop (default 2 => ~4 s) rather than with the JSON, and
# rsync's write-then-rename keeps the console from ever reading a half file.
# CAM_SYNC=0 turns it off (e.g. a weak link where the JSON matters more).
#
# Usage: cm4/status_sync.sh <user@cm4-host> [remote_dir=mission] [interval_s=2]
set -u

HOST="${1:?usage: cm4/status_sync.sh <user@cm4-host> [remote_dir] [interval_s]}"
DIR="${2:-mission}"
IVL="${3:-2}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAM_SYNC="${CAM_SYNC:-1}"
CAM_EVERY="${CAM_EVERY:-2}"
CAM_REMOTE="${CAM_REMOTE:-/tmp/aavc_nadir.jpg}"
CAM_LOCAL="${CAM_LOCAL:-/tmp/aavc_nadir.jpg}"

mkdir -p "$REPO_ROOT/captures"
echo "[status-sync] $HOST:~/$DIR/captures/ -> $REPO_ROOT/captures/ every ${IVL}s (Ctrl-C to stop)"
[ "$CAM_SYNC" = "1" ] && echo "[status-sync] nadir frame $CAM_REMOTE -> $CAM_LOCAL every $((IVL * CAM_EVERY))s (CAM_SYNC=0 disables)"
CM4_KEY="${CM4_KEY:-$HOME/.ssh/cm4_key}"   # non-default name — pass it on
SSH_ID=""; [ -f "$CM4_KEY" ] && SSH_ID="-i $CM4_KEY "
SSH_CMD="ssh ${SSH_ID}-o ConnectTimeout=3 -o BatchMode=yes -o StrictHostKeyChecking=accept-new"
loop=0
while true; do
    rsync -az --timeout=4 -e "$SSH_CMD" \
        "$HOST:$DIR/captures/" "$REPO_ROOT/captures/" 2>/dev/null || true
    loop=$((loop + 1))
    if [ "$CAM_SYNC" = "1" ] && [ $((loop % CAM_EVERY)) -eq 0 ]; then
        # -z on an already-compressed PNG buys nothing; --inplace would risk a
        # torn read on the console side, so leave rsync's temp+rename alone.
        rsync -a --timeout=6 -e "$SSH_CMD" \
            "$HOST:$CAM_REMOTE" "$CAM_LOCAL" 2>/dev/null || true
    fi
    sleep "$IVL"
done
