#!/usr/bin/env bash
# Bring up ONLY the aircraft's own stack — mavlink-router + camera grabber +
# status beacon — and NOTHING else: no orchestrator, no arm, no flight.
#
# The REAL console auto-runs this over ssh the moment the CM4 becomes reachable
# (operator 2026-08-19), so the camera sensor chip lights from the radio beacon
# WITHOUT pressing 🚀 / sliding to stage a mission. It is a thin, fail-safe
# wrapper over run_mission.sh's INFRA_ONLY mode: a console pointed at a CM4 that
# predates this file just gets "no such file" over ssh and starts nothing — it
# can NEVER accidentally stage a flight. ensure_infra is idempotent, so running
# this while a mission is already up leaves everything exactly as it is.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# dummy id "1" satisfies run_mission.sh's usage check; INFRA_ONLY exits before it
# is ever looked at, so nothing is armed or flown.
# ⚠ Strip the CAM_* env before handing over (review 2026-08-30). The CM4's
# ~/.bashrc carries a night-camera hook gated on ~/.aavc_night_cam that exports
# CAM_EXPOSURE=180 CAM_GAIN=128, and a remote shell sources ~/.bashrc BEFORE
# running the ssh command — so the documented one-line revert
# (`rm -f ~/.aavc_night_cam; pkill grabber; bash start_infra.sh`) restarts the
# grabber with the night values still exported, and `--ae-highlight` is dropped
# (run_mission.sh only adds it when CAM_EXPOSURE=0). In sun that is a white
# frame and zero decodes, and the crew's verify step shows `--gain 128` again.
# Unsetting here fixes every caller at once; a deliberate night run sets the
# values on the command line instead.
exec env -u CAM_EXPOSURE -u CAM_GAIN -u CAM_AE -u CAM_AE_MAX -u CAM_AE_INIT \
     -u GRAB_ARGS REAL=1 INFRA_ONLY=1 "$REPO_ROOT/sitl/run_mission.sh" 1
