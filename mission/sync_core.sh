#!/usr/bin/env bash
# ⚠ SUPERSEDED 2026-09-02 — the two repos became one. aavc-practice is now
# mission/ in the AAVC monorepo and aavc-comp is the `archive/competition-deploy`
# branch, so there is no sibling to sync to and nothing can drift. Kept because
# it documents how the two cores were held identical all season, and because it
# still works against any checkout you point it at.
#
# Copy the FLIGHT CORE from this repo to a sibling checkout, so a fix made once
# lands in both. The operator chose two separate repos (aavc-practice for KMUTNB,
# aavc-comp for KMITL); this is what kept their cores from drifting apart.
#
#   bash sync_core.sh ~/Desktop/aavc-comp     # aavc-practice -> aavc-comp
#
# Copies everything that is IDENTICAL between the two fields: the Python
# packages, the tests, the CM4/launcher scripts, the dashboard, the real-camera
# grabber, and BOTH field configs.
#
# ⚠ The configs ARE shared (changed 2026-08-22). Each repo carries a config for
# BOTH fields and flies whichever its own .aavc_site names, so a per-repo copy
# was never "field-specific" — it was just a second copy that drifted. It did:
# the 2026-08-22 review found the comp repo's kmitl_config.yaml missing
# MPC_YAW_MODE (its camera would spin at every sweep turn) and still quoting
# the retired LiPo's battery endpoints. ONE file decides the field —
# .aavc_site — and that is the one thing this script never copies.
#
# Syncing tests/ without the code they cover is what made the comp suite fail
# 11 tests: the camera grabber, cm4/ and dashboard/ were left behind while
# their tests came across. If a test can see it, this script must copy it.
#
# NEVER touched: .aavc_site, the survey data, sitl/ models/worlds/patches, .git.
# Review `git diff` in the target before committing.
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${1:?usage: sync_core.sh <path-to-the-other-repo>}"
[ -d "$DST/.git" ] || { echo "refusing: '$DST' is not a git repo" >&2; exit 1; }
[ "$SRC" != "$(cd "$DST" && pwd)" ] || { echo "refusing: source == target" >&2; exit 1; }

echo "[sync_core] $SRC"
echo "        ->  $DST"
for d in orchestrator mission_brain vision mavlink_adapter tests dashboard; do
    rsync -a --delete "$SRC/$d/" "$DST/$d/"
done
# cm4/ is the ON-AIRCRAFT stack plus the laptop launchers — same companion,
# same board, and launch_flight.sh resolves its field from .aavc_site. NO
# --delete: keep anything the target added. (Until 2026-08-22 the comp repo had
# no start_infra.sh at all, so the console's auto-infra — which the GCS icon
# calls by that exact name — could only ever fail there.)
rsync -a "$SRC/cm4/" "$DST/cm4/"
# The REAL-camera writer and the 🚀 entry point: aircraft-level, not field-level.
rsync -a "$SRC/sitl/camera_grabber.py" "$SRC/sitl/run_mission.sh" "$DST/sitl/"
# SITL launchers + bridges: simulator-only, but shared code all the same — a
# stale copy in the other repo fails the same tests this one passes (the comp
# repo's launch_stack.sh still had the un-bracketed pgrep the self-match test
# exists to catch).
for f in launch_stack.sh launch_sitl.sh camera_view.sh gz_camera_bridge.py \
         spawn_targets.py hitl_synthetic_camera.py payload_detach_bridge.py \
         sim_pilot.py; do
    [ -f "$SRC/sitl/$f" ] && rsync -a "$SRC/sitl/$f" "$DST/sitl/$f"
done
# Both field configs (see the note above). .aavc_site stays put.
rsync -a "$SRC/sitl/aavc_config.yaml" "$SRC/sitl/kmitl_config.yaml" "$SRC/sitl/bangbo_config.yaml" "$DST/sitl/"
# AIRCRAFT-level tools are shared too — same airframe, same board, so the same
# truth. Left out until 2026-08-21, and the drift it allowed was found by that
# day's review: the comp repo had no px4_type_audit.py at all and a
# preflight_params.py two days stale, missing SYS_HITL (a board left flagged
# HITL has no actuator output) and MPC_THR_HOVER. NO --delete here: the comp
# repo carries field tools of its own (survey/satellite helpers) that this
# repo has never had, and they must survive the sync.
for f in preflight_params.py px4_type_audit.py param_audit.py verify_flight.py board_param.py lidar_check.py \
         fence_probe.py alt_watch.py replay_frames.py landing_trial.py \
         gen_pads.py gen_aruco_glyphs.py gen_grass.py measure_mount_yaw.py \
         rc_check.py hover_decode.py serial_sniff.py gps_bench.py; do
    [ -f "$SRC/tools/$f" ] && rsync -a "$SRC/tools/$f" "$DST/tools/$f"
done

# …and CHECK that list, because it is hand-maintained and tests/ is not.
# 2026-08-22: tools/measure_mount_yaw.py was written with its test; the test
# crossed (tests/ is copied wholesale) and the tool did not, so the comp suite
# stopped at "1 error during collection" — 630 tests refusing to run because of
# one missing import. That is the same shape as the 11 failures the header
# above already records. A list nobody validates is a list that drifts, so this
# names the file to add instead of leaving the next person a collection error.
python3 - "$SRC" "$DST" <<'GUARD'
import pathlib, re, sys
src, dst = (pathlib.Path(a) for a in sys.argv[1:3])
missing = []
for t in sorted((dst / "tests").glob("test_*.py")):
    body = t.read_text()
    if '"tools"' not in body and "/ 'tools'" not in body:
        continue                       # not a tools-importing test
    for mod in re.findall(r"^from ([a-z_][a-z0-9_]*) import", body, re.M):
        if (dst / "tools" / f"{mod}.py").exists():
            continue
        if (src / "tools" / f"{mod}.py").exists():
            missing.append((t.name, f"{mod}.py"))
if missing:
    print("[sync_core] ✘ tests copied without the tools they import:",
          file=sys.stderr)
    for test, mod in missing:
        print(f"[sync_core]     {test} needs tools/{mod}", file=sys.stderr)
    print("[sync_core]   add it to the tools list in sync_core.sh and re-run",
          file=sys.stderr)
    raise SystemExit(1)
GUARD
# The reset + sync scripts themselves are shared verbatim.
rsync -a "$SRC/clear_state.sh" "$SRC/sync_core.sh" "$DST/"

echo "[sync_core] done — now: cd '$DST' && make test && git diff"
