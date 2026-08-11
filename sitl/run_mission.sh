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
else
    # SITL: enable the post-flight truth audit when the spawner wrote one.
    [ -f /tmp/aavc_targets.json ] && EXTRA+=(--truth-json /tmp/aavc_targets.json)
fi

echo "[run_mission] assigned ids: $IDS (${REAL:+REAL bird}${REAL:-SITL})"
exec env -u PYTHONPATH "$REPO_ROOT/.venv/bin/python" -m orchestrator.main \
    --config sitl/aavc_config.yaml --no-dashboard \
    --assigned-ids "$IDS" "${EXTRA[@]}"
