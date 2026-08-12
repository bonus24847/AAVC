#!/usr/bin/env bash
# GCS "🚀 บิน mission" launcher — spawned by the AAVC GCS console
# (aavc_gcs.py --mission-cmd '<this script> {ids}'). $1 = comma-separated
# assigned pad ids from the operator's saved selection.
#
# SITL: launch_stack.sh / `make aavc-gcs` wire this script in automatically.
# REAL BIRD: the orchestrator runs on the CM4, not the GCS laptop — point the
# console at ssh instead:
#   --mission-cmd "ssh <user>@<cm4> 'REAL=1 ~/mission/sitl/run_mission.sh {ids}'"
# REAL=1 switches to the offboard link (mavlink-router 14540) and drops the
# SITL-only truth audit; everything else is identical to the SITL path.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
IDS="${1:?usage: run_mission.sh <id,id,...>}"

# One mission at a time — the GCS also guards this, but a CLI-started run
# must be protected too.
if pgrep -f 'orchestrator.mai[n]' >/dev/null; then
    echo "[run_mission] refused: an orchestrator is already running" >&2
    exit 1
fi

EXTRA=()
if [ "${REAL:-0}" = "1" ]; then
    EXTRA+=(--connect "udpin://0.0.0.0:14540")
    # REAL bird defaults to RC-GO (operator conops 2026-08-12): the console's
    # 🚀 only STAGES the flight — the SAFETY PILOT arms via RC and flips to
    # OFFBOARD to launch; flipping to POSCTL mid-flight makes the orchestrator
    # stand down. The web never moves the aircraft. RC_GO=0 restores the
    # auto-launch behaviour (orchestrator arms itself right after preflight).
    RC_GO="${RC_GO:-1}"
else
    # SITL dirty-field precheck (2026-08-12, hit live by the operator): cargo
    # boxes shed by a PREVIOUS run stay lying ON the pads and hide the ArUco
    # markers — the next mission then can't decode those pads at all (looks
    # like "pads missing" on the GCS map) and flies home with eggs unserved.
    # The detach bridge's log is truncated at every stack (re)start, so any
    # "shed" line in it means THIS field already has boxes down. Refuse with
    # a clear message — the console surfaces this line in the browser.
    if grep -q 'shed payload' /tmp/aavc_detach.log 2>/dev/null; then
        echo "❌ สนามยังมีกล่องจากรอบก่อนวางบัง marker อยู่ — กดปุ่ม 🧹 รีเซ็ตสนาม ก่อนบินใหม่" >&2
        exit 1
    fi
    # SITL: enable the post-flight truth audit when the spawner wrote one.
    [ -f /tmp/aavc_targets.json ] && EXTRA+=(--truth-json /tmp/aavc_targets.json)
fi

if [ "${RC_GO:-0}" = "1" ]; then
    EXTRA+=(--rc-go)
fi

# Operator-drawn field geometry from the GCS map editor (2026-08-13): if the
# console saved captures/field_override.json, fly THAT geometry — the
# orchestrator validates it and ABORTS on a bad file (never a silent
# fallback). Delete the file (GCS "ล้าง" button) to return to the yaml field.
if [ -f "$REPO_ROOT/captures/field_override.json" ]; then
    echo "[run_mission] field override active: captures/field_override.json"
    EXTRA+=(--field-override "$REPO_ROOT/captures/field_override.json")
fi

echo "[run_mission] assigned ids: $IDS (${REAL:+REAL bird}${REAL:-SITL})${RC_GO:+ rc-go=$RC_GO}"
exec env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" -m orchestrator.main \
    --config sitl/aavc_config.yaml --no-dashboard \
    --assigned-ids "$IDS" "${EXTRA[@]}"
