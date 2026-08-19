#!/usr/bin/env bash
# Copy the FLIGHT CORE from this repo to the sibling repo, so a fix made once
# lands in both. The operator chose two separate repos (aavc-practice for KMUTNB,
# aavc-comp for KMITL); this is what keeps their cores from drifting apart.
#
#   bash sync_core.sh ~/Desktop/aavc-comp     # aavc-practice -> aavc-comp
#
# Copies the Python packages + tests ONLY — the code that is IDENTICAL between
# the two fields. It NEVER touches anything field-specific: the per-repo
# .aavc_site marker, the sitl/ configs (aavc_config.yaml / kmitl_config.yaml),
# the survey data, the launcher icons, or .git. Review `git diff` in the target
# before committing.
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${1:?usage: sync_core.sh <path-to-the-other-repo>}"
[ -d "$DST/.git" ] || { echo "refusing: '$DST' is not a git repo" >&2; exit 1; }
[ "$SRC" != "$(cd "$DST" && pwd)" ] || { echo "refusing: source == target" >&2; exit 1; }

echo "[sync_core] $SRC"
echo "        ->  $DST"
for d in orchestrator mission_brain vision mavlink_adapter tests; do
    rsync -a --delete "$SRC/$d/" "$DST/$d/"
done
# The reset + sync scripts themselves are shared verbatim.
rsync -a "$SRC/clear_state.sh" "$SRC/sync_core.sh" "$DST/"

echo "[sync_core] done — now: cd '$DST' && make test && git diff"
