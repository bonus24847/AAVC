#!/usr/bin/env bash
# Link the PX4 Gazebo assets this repo's SITL models borrow.
#
#     bash sitl/link_px4_assets.sh                 # link from $PX4_DIR
#     PX4_DIR=~/PX4-Autopilot bash sitl/link_px4_assets.sh
#     bash sitl/link_px4_assets.sh --check         # report only, link nothing
#
# WHY: sitl/models/eft_x6100_base/model.sdf — the airframe that actually flies
# in SITL — draws its propellers from `model://x500_base/meshes/1345_prop_*.stl`,
# and x500_base's own meshes/materials/thumbnails belong to PX4-Autopilot, not
# to this repo. They used to be committed as ABSOLUTE symlinks into one
# machine's home (`/home/<someone>/PX4-Autopilot/…`), which meant they were
# broken for everyone else — and broken on that machine too once the tree
# moved. Untracked as of 2026-09-02; this script recreates them per clone.
#
# Nothing here is required to FLY the sim: gz falls back to no visual for a
# missing mesh, so the aircraft still simulates, just without visible props.
# Run it if you want the viewer to look right.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-v1.17}"
SRC="$PX4_DIR/Tools/simulation/gz/models"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

# repo path <- PX4 path (relative to $SRC)
LINKS=(
    "models/x500_base/materials|x500_base/materials"
    "models/x500_base/meshes|x500_base/meshes"
    "models/x500_base/thumbnails|x500_base/thumbnails"
    "models/x500/thumbnails|x500/thumbnails"
)

if [ ! -d "$SRC" ]; then
    echo "PX4 models not found at $SRC" >&2
    echo "  set PX4_DIR to your PX4-Autopilot clone, e.g." >&2
    echo "      PX4_DIR=\$HOME/PX4-Autopilot bash sitl/link_px4_assets.sh" >&2
    exit 1
fi

rc=0
for pair in "${LINKS[@]}"; do
    dst="$REPO_ROOT/sitl/${pair%%|*}"
    src="$SRC/${pair##*|}"
    if [ ! -e "$src" ]; then
        echo "✗ missing in PX4: $src" >&2; rc=1; continue
    fi
    if [ "$CHECK" = 1 ]; then
        if [ -e "$dst" ]; then echo "✔ $dst"; else echo "✗ not linked: $dst"; rc=1; fi
        continue
    fi
    mkdir -p "$(dirname "$dst")"
    rm -rf "$dst"
    ln -s "$src" "$dst" && echo "✔ $dst -> $src"
done

[ "$CHECK" = 1 ] || echo
[ "$CHECK" = 1 ] || echo "PX4_DIR=$PX4_DIR"
exit $rc
