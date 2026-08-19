#!/usr/bin/env bash
# Reset all RUNTIME / SESSION state before a fresh mission. Each launcher icon
# (practice / competition) runs this FIRST, so "open the icon" always starts
# clean — a new flight can never inherit a stale pad assignment, an old camera
# frame, a finished mission's status, or a leftover stack process (operator
# 2026-08-19: "ทุกครั้งที่เปิด icon ต้องเคลียร์ข้อมูลเก่าก่อนเสมอ").
#
# It clears SESSION state only. It NEVER touches the flight controller's
# parameters, the uploaded geofence, the ground survey, or any config file —
# those are deliberate state a reset must preserve.
#
# Safe to run when nothing is up: every step tolerates "nothing to do".
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[clear_state] stopping leftover stack processes…"
# Match by module/script name so an unrelated process is never hit. This script
# itself and the launching shell do not match any pattern below.
for pat in 'orchestrator\.main' 'mavlink-router' 'sitl/camera_grabber' \
           'gz_camera_bridge' 'hitl_synthetic_camera' 'status_beacon\.py'; do
    pkill -f "$pat" 2>/dev/null || true
done

echo "[clear_state] clearing runtime files…"
# Pad assignment → empty, so the operator re-picks the ids for THIS mission and
# the 🚀 interlock refuses a launch until they do (fails safe).
mkdir -p "$REPO_ROOT/captures"
printf '{"ids": [], "updated": 0}\n' > "$REPO_ROOT/captures/pad_assignment.json" 2>/dev/null || true
# The last mission's status → gone, so the console/beacon can't show a finished
# flight as if it were live.
rm -f "$REPO_ROOT/captures/mission_status.json" 2>/dev/null || true
# Live camera frames + SITL truth/wind + a stale router conf the next run must
# not read as its own.
rm -f /tmp/aavc_nadir.png /tmp/aavc_frame.png /tmp/aavc_targets.json \
      /tmp/aavc_wind_state /tmp/aavc_flight_router.conf 2>/dev/null || true

# The CM4's own runtime (orchestrator / router / camera / beacon) is cleared by
# sitl/run_mission.sh's guards + ensure_infra when the icon stages the flight;
# this reset covers the LAPTOP session state the console reads.
echo "[clear_state] done — safe to launch."
