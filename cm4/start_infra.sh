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
exec env REAL=1 INFRA_ONLY=1 "$REPO_ROOT/sitl/run_mission.sh" 1
