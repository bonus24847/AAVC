#!/usr/bin/env bash
# AAVC GCS — mission dispatcher (2026-08-13): ONE entry point, the console
# itself is mission-agnostic; each target wires the right field yaml,
# captures dir and 🚀 mission command for that repo.
#
#   ./gcs_launch.sh kmutnb-sim              # KMUTNB practice: SITL stack + console
#   ./gcs_launch.sh kmutnb-real <user@cm4>  # KMUTNB practice: real bird (RC-GO)
#   ./gcs_launch.sh aavc-sim|aavc-real …    # competition repo (mission_AAVC) —
#                                           # pending its GCS contract pieces
#
# (This file is deliberately untracked in the aavc-gcs repo — the repo has a
# standing no-commit policy. The kmutnb launchers it calls live in that
# mission's own repo and ARE version-controlled there.)
set -uo pipefail

KMUTNB="$HOME/Desktop/mission AAVC in kmutnb"
AAVC_COMP="$HOME/Desktop/mission_AAVC"

case "${1:-}" in
  kmutnb-sim)
    shift
    exec env GUI="${GUI:-1}" bash "$KMUTNB/sitl/launch_stack.sh" "$@";;
  kmutnb-real)
    shift
    exec bash "$KMUTNB/cm4/launch_gcs_real.sh" "$@";;
  aavc-sim|aavc-real)
    echo "ยังเปิดไม่ได้: repo mission_AAVC ($AAVC_COMP) ต้องมี contract 3 ชิ้นก่อน" >&2
    echo "  (1) entry แบบ run_mission.sh ที่รับ {ids}  (2) field yaml ของสนามมัน" >&2
    echo "  (3) ตัวเขียน mission_status.json ลง captures — เสนอให้เซสชันที่ดูแล repo นั้นแล้ว รอตอบรับ" >&2
    exit 1;;
  *)
    echo "usage: gcs_launch.sh kmutnb-sim | kmutnb-real <user@cm4> | aavc-sim | aavc-real" >&2
    exit 2;;
esac
