#!/usr/bin/env bash
# Turn SITL wind ON (or off) at RUNTIME — no world edit, no restart.
#
#   bash sitl/set_wind.sh            # the configured wind (wind_sitl in the config)
#   bash sitl/set_wind.sh 10 225     # 10 m/s FROM 225 deg (meteorological)
#   bash sitl/set_wind.sh off        # back to still air
#
# WHY THE DEFAULT WORLD IS CALM (operator 2026-08-15: "อย่าลืมลบลมออกด้วย"):
# every validated number this project quotes — G4/G4' scatter, the 0.05-0.07 m
# releases — was measured in still air, so still air is the baseline runs must
# keep reproducing. Wind is a SEPARATE experiment you switch on deliberately,
# not a silent change to what "a normal run" means.
#
# It is worth switching on, though: with wind reaching the aircraft for the
# first time (2026-08-15) the same mission that passed 10 checks / 0 warnings
# went to 3 violations, which is how the transit-sag bug was found at all.
#
# Direction is METEOROLOGICAL — the compass bearing the wind comes FROM, the
# same convention as wind_sitl and every forecast. gz wants the vector it blows
# TOWARD, in ENU, which is why the conversion lives here rather than in
# somebody's head:  east = v*sin(bearing+180), north = v*cos(bearing+180).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${AAVC_CONFIG:-$REPO_ROOT/sitl/aavc_config.yaml}"
WORLD="${PX4_GZ_WORLD:-kmutnb_skyfield}"

if [ "${1:-}" = "off" ] || [ "${1:-}" = "0" ]; then
    SPEED=0; DIR=0
elif [ -n "${1:-}" ]; then
    SPEED="$1"; DIR="${2:-225}"
else
    read -r SPEED DIR < <(/usr/bin/python3 - "$CONFIG" <<'PY'
import sys, yaml
w = (yaml.safe_load(open(sys.argv[1])) or {}).get("wind_sitl") or {}
print(w.get("base_speed_mps", 0), w.get("direction_deg", 225))
PY
)
fi

read -r EAST NORTH < <(/usr/bin/python3 - "$SPEED" "$DIR" <<'PY'
import math, sys
v, bearing = float(sys.argv[1]), float(sys.argv[2])
toward = math.radians((bearing + 180.0) % 360.0)
print(f"{v*math.sin(toward):.3f}", f"{v*math.cos(toward):.3f}")
PY
)

if ! gz topic -t "/world/$WORLD/wind" -m gz.msgs.Wind \
        -p "linear_velocity: {x: $EAST, y: $NORTH, z: 0}, enable_wind: true" 2>/dev/null
then
    echo "[wind] could not publish — is SITL running on world '$WORLD'?" >&2
    exit 1
fi

# Leave a breadcrumb the NEXT run can read. Wind set here is runtime-only, so
# nothing in the world file records it and a mission flown afterwards looks
# identical to a calm one in every log it writes — the parallel session lost a
# set of numbers exactly this way (their harness left <wind> at 10 m/s and the
# following flights silently flew in it). run_mission.sh reads this file and
# says so out loud; launch_stack.sh deletes it, because a fresh world IS calm.
STATE_FILE=/tmp/aavc_wind_state
if [ "$SPEED" = "0" ]; then
    rm -f "$STATE_FILE"
    echo "[wind] still air (world $WORLD)"
else
    echo "${SPEED} ${DIR}" > "$STATE_FILE"
    echo "[wind] ${SPEED} m/s from ${DIR}deg -> ENU (${EAST}, ${NORTH}) on world $WORLD"
    echo "[wind] gusting comes from the world's WindEffects plugin (sine + noise);"
    echo "[wind] the airframe feels it only because model.sdf sets <enable_wind>."
    echo "[wind] recorded in $STATE_FILE — the next mission will announce it."
fi
