#!/usr/bin/env bash
# AAVC 2026 — HITL launcher: real Pixhawk 6X (running PX4 firmware) + jMAVSim physics.
#
# WHY jMAVSim and not gz: PX4 HITL is supported ONLY by jMAVSim and Gazebo Classic.
# The gz (Harmonic) sim this repo uses for SITL CANNOT drive a real flight controller,
# so HITL gives NO simulated cameras. The vision pipeline is fed by
# `sitl/hitl_synthetic_camera.py` instead (run it alongside). HITL here validates the
# real FC's compute / control / params / mission-sequence / drop-command / failsafes —
# NOT vision precision or real dynamics (those are SITL + the G6 tethered gate).
#
# Full runbook + param checklist + topology diagrams: docs/HITL.md
#
# Usage:   ./sitl/launch_hitl.sh                 # /dev/ttyACM0 @ 921600
#          HITL_SERIAL=/dev/ttyACM1 ./sitl/launch_hitl.sh
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DEV="${HITL_SERIAL:-/dev/ttyACM0}"
BAUD="${HITL_BAUD:-921600}"
RATE="${HITL_RATE:-250}"

cat <<EOF
[hitl] ── ONE-TIME on the 6X (see docs/HITL.md) — this script only starts jMAVSim ──
  • FIRMWARE: stock fmu-v6x CANNOT HITL. Flash the HIL build once:
        make px4_fmu-v6x_hil upload      (in the PX4 tree)  ⚠ reflash flight fw before G7
  • PARAMS: set via the nsh shell, NOT QGC/PARAM_SET (byte-wise gotcha):
        make hitl-params SERIAL=$DEV      → airframe 1001 + SYS_HITL=1 + the RC block
  • RC: bind TX16S+Nomad → DBR4, wire CRSF to a 6X UART (docs/HITL.md §RC).
  • MISSION LINK: mavlink-router owns the offboard link → :14540 (headless, no QGC).
        Topology (A) CM4-in-the-loop:  bash cm4/launch_hitl.sh
        Topology (B) single-PC bench:  mavlink-routerd -c sitl/hitl_router.conf
[hitl] ──────────────────────────────────────────────────────────────────────────
[hitl] jMAVSim owns this 6X USB HIL link:  serial=$DEV baud=$BAUD rate=$RATE
EOF

if [ ! -e "$DEV" ]; then
    echo "[hitl] ERROR: $DEV not found. Is the 6X plugged in? Try: ls /dev/ttyACM* /dev/ttyUSB*"
    exit 1
fi
JMAVSIM="$PX4_DIR/Tools/simulation/jmavsim/jmavsim_run.sh"
if [ ! -x "$JMAVSIM" ]; then
    echo "[hitl] ERROR: jmavsim_run.sh not found/executable at $JMAVSIM"
    echo "[hitl]   (jMAVSim ships with PX4-Autopilot; check your PX4_DIR / git submodules)"
    exit 1
fi

echo "[hitl] next: in a 2nd terminal run  make hitl-camera  then  make run-hitl"
echo "[hitl] starting jMAVSim against the 6X — Ctrl-C to stop."
cd "$PX4_DIR"
# -q connect to QGC, -s start simulator, -d serial device, -b baud, -r update rate (Hz)
exec ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$DEV" -b "$BAUD" -r "$RATE"
