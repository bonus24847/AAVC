#!/usr/bin/env bash
# AAVC GCS — mission dispatcher (2026-08-13): ONE entry point, the console
# itself is mission-agnostic; each target wires the right field yaml,
# captures dir and 🚀 mission command for that field.
#
#   ./gcs_launch.sh sim              # SITL stack + console (KMUTNB sky-field)
#   ./gcs_launch.sh real <user@cm4>  # real bird, CLI (RC-GO)
#   ./gcs_launch.sh gui              # real bird, click-through icon (any field)
#   ./gcs_launch.sh demo             # console alone, fake telemetry
#
# `sim` is the KMUTNB sky-field harness and nothing else: launch_stack.sh is
# wired to that world, that field yaml and sitl/aavc_config.yaml. To fly the
# KMITL or Bang Bo config, use the console's own 🚀 button (its registry has
# all three) or run mission/sitl/run_mission.sh with AAVC_CONFIG set.
#
# ⚠ 2026-09-02: this used to point at "$HOME/Desktop/mission AAVC in kmutnb"
# and "$HOME/Desktop/mission_AAVC" — two absolute paths into one laptop's home,
# one of which had already been retired. The console and the flight code are
# siblings in this repo now, so everything is found relative to this script and
# the same commands work on any machine that cloned it.
#
# The old verbs still work: kmutnb-sim / kmutnb-real map onto sim / real.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MISSION="$REPO_ROOT/mission"
GCS="$REPO_ROOT/gcs/src/aavc_gcs.py"

[ -d "$MISSION" ] || {
    echo "ไม่พบ $MISSION — repo นี้ต้องมี gcs/ กับ mission/ อยู่ข้างกัน" >&2
    exit 1
}

case "${1:-}" in
  sim|kmutnb-sim)
    shift
    exec env GUI="${GUI:-1}" bash "$MISSION/sitl/launch_stack.sh" "$@";;
  real|kmutnb-real)
    shift
    exec bash "$MISSION/cm4/launch_gcs_real.sh" "$@";;
  gui)
    shift
    exec bash "$MISSION/cm4/launch_gcs_real_gui.sh" "$@";;
  demo)
    shift
    exec /usr/bin/env python3 "$GCS" --demo "$@";;
  *)
    echo "usage: gcs_launch.sh sim | real <user@cm4> | gui | demo" >&2
    exit 2;;
esac
