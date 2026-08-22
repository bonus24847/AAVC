#!/usr/bin/env bash
# Live nadir-camera window for SITL test sessions (operator request
# 2026-08-13: "ตอน test SITL เปิดกล้องมองล่างให้ดูด้วย เพื่อมั่นใจว่าเจอจริง").
#
# Opens the camera bridge's frame (/tmp/aavc_nadir.jpg) in an auto-reloading
# viewer — eog re-reads the file every time the bridge overwrites it, so the
# window IS the live nadir feed. Run it alongside the stack:
#
#   make camera-view      (or: bash sitl/camera_view.sh)
#
# Prereq: the stack (or `make camera-bridge`) is up and writing frames.
set -uo pipefail

FRAME="${FRAME:-/tmp/aavc_nadir.jpg}"

if [ ! -f "$FRAME" ]; then
    echo "[camera-view] waiting for $FRAME (is the camera bridge up? make camera-bridge)…"
    for _ in $(seq 1 30); do
        [ -f "$FRAME" ] && break
        sleep 2
    done
    [ -f "$FRAME" ] || { echo "[camera-view] no frame after 60 s — start the stack first" >&2; exit 1; }
fi

if command -v eog >/dev/null; then
    exec eog "$FRAME"
elif command -v feh >/dev/null; then
    exec feh --reload 0.5 "$FRAME"
else
    exec xdg-open "$FRAME"
fi
