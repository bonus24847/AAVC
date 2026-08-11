#!/usr/bin/env bash
# AAVC 2026 — Launch PX4 SITL + Gazebo Harmonic with AAVC field world.
#
# Required env:
#   PX4_DIR  Path to PX4-Autopilot clone (default ~/PX4-Autopilot-v1.17, the
#            worktree carrying our 22000_gz_eft_x6100 airframe — branch
#            aavc/sitl-v1.17). It matches the firmware on the real 6X.
#
# Usage:
#   ./sitl/launch_sitl.sh                  # default gz_eft_x6100 hexacopter
#   ./sitl/launch_sitl.sh gz_x500_mono_cam # legacy quad (needs PX4_DIR=~/PX4-Autopilot)
#
# Notes:
# - First run will compile PX4 + the SITL gazebo plugin (~5-15 min).
# - Gazebo Garden is launched by PX4 sim_gazebo target automatically.
# - The AAVC world file is added to GZ_SIM_RESOURCE_PATH so PX4's launcher
#   picks it up via the PX4_GZ_WORLD env var.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-v1.17}"
AIRFRAME="${1:-gz_eft_x6100}"   # EFT X6100 hexacopter + nadir camera (the AAVC aircraft)

# Dev-loop accelerator. PX4 SITL runs in lockstep with Gazebo — the world's
# <real_time_factor>1.0</real_time_factor> is a target, not a cap. Setting
# PX4_SIM_SPEED_FACTOR > 1 lets SITL chew through the mission faster, useful
# when iterating on orchestrator changes. Cap depends on the host CPU; 4x is
# usually safe on a modern laptop, 8x on a desktop. Keep at 1 for vision /
# camera-pipeline tests so frame timestamps look real.
# See: https://docs.px4.io/main/en/simulation/
SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1}"

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4_DIR=$PX4_DIR not found." >&2
    echo "       ./sitl/setup.sh builds it (step 4 creates this worktree and" >&2
    echo "       applies sitl/px4_patches/); the recipe is also in" >&2
    echo "       sitl/px4_patches/README.md if you would rather do it by hand." >&2
    exit 1
fi

# The hexacopter airframe is an AAVC patch, not upstream: it lives on the
# aavc/sitl-v1.17 branch of the v1.17 worktree (provenance copy in
# sitl/px4_patches/). Checking out a bare tag silently loses it, and the failure
# would otherwise surface as an opaque "unknown target gz_eft_x6100".
if [ "$AIRFRAME" = "gz_eft_x6100" ] && \
   [ ! -f "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_eft_x6100" ]; then
    echo "Error: $PX4_DIR has no 22000_gz_eft_x6100 airframe." >&2
    echo "       If the branch is still there:  git -C $PX4_DIR switch aavc/sitl-v1.17" >&2
    echo "       Otherwise rebuild the tree from sitl/px4_patches/README.md" >&2
    echo "       (the branch is local-only — a fresh clone will not have it)." >&2
    echo "       To fly the legacy quad instead:" >&2
    echo "       PX4_DIR=\$HOME/PX4-Autopilot bash sitl/launch_sitl.sh gz_x500_mono_cam" >&2
    exit 1
fi

# ── dGPU health preflight ─────────────────────────────────────────────────────
# The gz-sensors (camera) render is PINNED to the NVIDIA dGPU by the EGL force
# below. If that GPU is wedged — e.g. a prior run crashed its GSP firmware under
# render load — gz can't get a render context and the lockstep sim hangs; on this
# host that has HARD-FROZEN the whole machine (NVRM "Cannot initialize GSP firmware
# RM" → forced reboot, 2026-06-15). A warm reboot does NOT clear a wedged GSP —
# only a COLD power-off does. So refuse to launch onto a dead GPU instead of
# dragging the machine into another freeze. SKIP_GPU_CHECK=1 bypasses (non-NVIDIA
# host / alternate render path).
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && [ "${SKIP_GPU_CHECK:-0}" != "1" ]; then
    # NB: `nvidia-smi -L` exits 0 even when it prints "No devices found.", so we
    # must test for an actual "GPU 0: …" line, not the exit status.
    if ! nvidia-smi -L 2>/dev/null | grep -q '^GPU [0-9]'; then
        echo "Error: NVIDIA dGPU is not responding (nvidia-smi found no device)." >&2
        echo "       The sensor render path is pinned to it, so SITL would hang/freeze." >&2
        echo "       A warm reboot will NOT fix a wedged GSP — do a COLD power-off:" >&2
        echo "       shut down, stay off ~30 s, cold boot, then 'nvidia-smi' must list" >&2
        echo "       the RTX A1000.  Check: journalctl -k -b 0 | grep -c nvAssertFailedNoLog  (want 0)" >&2
        echo "       Bypass (non-NVIDIA host / other render path): SKIP_GPU_CHECK=1 make sitl" >&2
        exit 1
    fi
    if journalctl -k -b 0 2>/dev/null | grep -q nvAssertFailedNoLog; then
        echo "[launch] WARNING: NVIDIA GSP assertions seen this boot — the dGPU may be"
        echo "[launch]          degrading. If the sim stalls, COLD-boot before retrying."
    fi
fi

# PX4 v1.15.x's gz launcher resolves the world from its own
# `Tools/simulation/gz/worlds/` directory BEFORE consulting
# GZ_SIM_RESOURCE_PATH. The resolved path then becomes the working dir
# context for any `<include><uri>model://...</uri></include>` lookups
# inside the world — meaning our project's models/ are not visible if
# the world isn't physically inside our project. To make the choice
# deterministic we sync our latest world file into PX4's worlds dir on
# every launch. Idempotent + safe to clobber (PX4 ships no `kmutnb_skyfield`).
PX4_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
if [ ! -d "$PX4_WORLDS_DIR" ]; then
    echo "Error: PX4 worlds dir not found at $PX4_WORLDS_DIR"
    echo "Is PX4 v1.15+? (Earlier versions used different layout.)"
    exit 1
fi
cp "${REPO_ROOT}/sitl/worlds/kmutnb_skyfield.sdf" "${PX4_WORLDS_DIR}/kmutnb_skyfield.sdf"
echo "[launch] synced kmutnb_skyfield.sdf → $PX4_WORLDS_DIR"

# Make our world + model resources discoverable to gz-sim.
# Order matters: our world dir first (so kmutnb_skyfield.sdf wins over any
# same-named PX4 default), our models next, then PX4's own model + world
# directories so nested `model://` includes still resolve.
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/sitl/worlds:${REPO_ROOT}/sitl/models:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# WHICH model PX4 spawns is decided by PX4_GZ_MODELS, not by the resource path:
# v1.17's px4-rc.gzsim builds an absolute `file://${PX4_GZ_MODELS}/<model>/model.sdf`
# spawn URI (v1.15 passed a bare name that GZ_SIM_RESOURCE_PATH resolved, which is
# why repo model-shadowing used to be enough). Point it at our models so
# eft_x6100 is found. rc.gzsim only sources PX4's gz_env.sh when PX4 starts the
# world itself — we pre-start the server below, so this export survives.
export PX4_GZ_MODELS="${REPO_ROOT}/sitl/models"

# Lock the gz major version. v1.15's wrapper reads GZ_VERSION (it defaults to
# garden when both garden and harmonic libs are present); v1.17's CMake reads
# GZ_DISTRO. Only harmonic is installed here — set both so either tree is happy.
export GZ_VERSION="harmonic"
export GZ_DISTRO="harmonic"

# v1.17's run targets start px4 with GZ_IP=127.0.0.1. The pre-started gz server
# below must use the SAME setting or the two can fail to discover each other and
# PX4 times out in "Waiting for Gazebo world...".
export GZ_IP="127.0.0.1"

# Force the NVIDIA EGL vendor for gz's OFFSCREEN sensor (camera) render. On a
# hybrid AMD+NVIDIA host the default Mesa/GBM EGL path fails ("failed to create
# dri2 screen") and falls back to a software/broken path that renders the two
# 640x480 cameras so slowly the lockstep sim crawls and EKF never converges
# (no arm / no "Ready for takeoff"). NVIDIA's surfaceless EGL renders on the
# discrete GPU instead — verified via `eglinfo` ("Surfaceless platform: NVIDIA
# ... OpenGL 4.6"). Guarded so it is a no-op on hosts without the NVIDIA EGL
# ICD; the GUI uses GLX and is unaffected.
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

# …but the vendor force ALONE is not enough. Verified 2026-06-13: with a
# DISPLAY/WAYLAND_DISPLAY present, gz-rendering-ogre2 probes the NVIDIA EGL device,
# creates then DISCARDS that context, and inits the RenderSystem on GLX → the AMD
# 780M iGPU instead (ogre2.log: "GL_VENDOR = AMD … radeonsi" while the RTX A1000
# sat idle at 7%). That iGPU offscreen path intermittently STALLS the gz Sensors
# render thread, which halts the lockstep sim mid-flight — PX4 freezes while armed
# and climbing, and the companion then hangs on telemetry that never resumes (the
# "sim froze ~3 s after the first drop" failure). The sensor-rendering server is
# headless and needs no windowing, so we strip DISPLAY from it below: with no GLX
# fallback, ogre2 must keep the NVIDIA EGL surfaceless context. A GUI client, if
# launched, is a separate process and keeps its own DISPLAY.

export PX4_GZ_WORLD="kmutnb_skyfield"
# Spawn position = the Launch & Recovery point at IAAI KMITL (rules V1.3) — the
# world origin: sitl/aavc_config.yaml site.center == ground_operation.launch_recovery
# == the world <spherical_coordinates>. Keep all four in sync.
export PX4_HOME_LAT="13.8224940"
export PX4_HOME_LON="100.5122771"
export PX4_HOME_ALT="15"

# Spawn height. The vehicle is spawned by an <include> that does NOT inherit the
# model's own <pose>, so without this it starts at z=0 with its landing gear
# 0.30 m INSIDE the ground; the contact solver then pushes it out at ~1 mm/s
# (max_vel=0), and arming from that embedded state flings the aircraft. Must
# clear the X6100's gear (skids sit 0.298 m below base_link).
# Format is comma-separated x,y,z,roll,pitch,yaw.
export PX4_GZ_MODEL_POSE="0,0,0.35,0,0,0"

# Only pass the speed factor when it actually differs from real time. PX4
# v1.17's px4-rc.gzsim reacts to this variable by calling the world's
# `set_physics` service, and on gz Harmonic that call replaces the world's
# physics/gravity state with the (mostly empty) request — leaving the world at
# ZERO GRAVITY. The vehicle then coasts at whatever velocity it had, PX4's EKF
# sees a ship that never accelerates, and the aircraft flies off to orbit
# (diagnosed 2026-07-22: an unattached twin model spawned into the same world
# hung motionless at its spawn altitude). Skipping the call for the 1x default
# keeps gravity intact; if you DO ask for a speed-up, expect that bug.
# Compared NUMERICALLY, not as a string: "1.0" is the natural way to spell real
# time (the world SDF itself writes <real_time_factor>1.0</real_time_factor>) and
# a string test against "1" would let it through — re-arming the very trap this
# guard exists for, at a launch the operator believes is unmodified.
if awk -v f="$SIM_SPEED_FACTOR" 'BEGIN{exit !(f+0==1)}' 2>/dev/null; then
    unset PX4_SIM_SPEED_FACTOR
else
    export PX4_SIM_SPEED_FACTOR="$SIM_SPEED_FACTOR"
    echo "[launch] WARNING: PX4_SIM_SPEED_FACTOR=$SIM_SPEED_FACTOR triggers PX4's"
    echo "[launch]          set_physics call, which zeroes gravity on gz Harmonic."
fi

echo "[launch] PX4_DIR=$PX4_DIR"
echo "[launch] AIRFRAME=$AIRFRAME"
echo "[launch] WORLD=$PX4_GZ_WORLD"
echo "[launch] HOME=($PX4_HOME_LAT, $PX4_HOME_LON, $PX4_HOME_ALT)"
echo "[launch] GZ_VERSION=$GZ_VERSION"
echo "[launch] PX4_GZ_MODELS=$PX4_GZ_MODELS"
echo "[launch] PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-<unset: real time>}"
echo "[launch] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo ""

# Skip the gz GUI client by default. It continuously renders the FULL 3D scene at
# display rate — far heavier than the two offscreen camera sensors the mission
# needs, and a second render context on the same wedge-prone dGPU, compounding the
# load that crashed the GSP firmware on 2026-06-15. The mission + dashboard never
# use it. GUI=1 brings the viewer back for visual debugging.
#
# GUI=1 is honoured HERE rather than delegated to PX4. px4-rc.gzsim only starts
# its viewer when PX4 itself creates the world, and this script always pre-starts
# the server (the spawn-race fix below), so that branch is unreachable — GUI=1
# used to be a knob that silently did nothing, which on this host reads as a
# broken GPU rather than a broken flag.
WANT_GUI=0
if [ "${GUI:-0}" = "1" ]; then
    WANT_GUI=1
    export HEADLESS=0
else
    export HEADLESS="${HEADLESS:-1}"
fi
# Saved before the strip below so the viewer — a separate process that DOES need
# windowing — can be handed its display back.
GUI_DISPLAY="${DISPLAY:-}"
GUI_WAYLAND="${WAYLAND_DISPLAY:-}"
# Strip windowing from EVERYTHING this script starts, not just the gz server it
# pre-starts. If the pre-start times out, PX4 falls back to launching its own gz
# server, which would inherit DISPLAY from `make` and let gz-rendering-ogre2 fall
# to the AMD iGPU over GLX — the stall/GSP-freeze path this strip exists to
# prevent (2026-06-13). Unsetting once here covers both paths.
unset DISPLAY WAYLAND_DISPLAY
echo "[launch] HEADLESS=$HEADLESS  (run 'GUI=1 make sitl' for the gz viewer)"
echo ""

cd "$PX4_DIR"

# ── Pre-start the gz server, THEN PX4 (gz_bridge spawn-race fix) ───────────────
# Otherwise PX4's gz_bridge starts the gz server AND immediately calls the
# model-spawn service, racing the server's startup → "[gz_bridge] Service call
# timed out" / "gz_bridge failed to start and spawn model" (px4 return 256). The
# race is intermittent on a fresh host but becomes PERSISTENT once the host has
# churned through many SITL runs (verified 2026-06-06: even stock gz_x500 failed
# 4/4). Starting the server first + waiting until its create service answers makes
# PX4 reuse the ready server and spawn cleanly.
WORLD_SDF="${PX4_WORLDS_DIR}/kmutnb_skyfield.sdf"
# Scoped to OUR world. A bare `gz sim` pattern matches every Gazebo the user is
# running — another project's simulation, a second AAVC terminal — and SIGKILLs
# it, which also strands that stack's PX4 wedged in lockstep.
GZ_PATTERN='gz[ ]sim.*kmutnb_skyfield'
pkill -9 -f "$GZ_PATTERN" 2>/dev/null || true    # a stale server would block reuse
# A PX4 that outlived its launcher (the nohup/disown workflow, or a terminal
# closed so the trap never ran) still holds the instance-0 lock, and the new one
# would exit "PX4 server already running for instance 0" — AFTER this script has
# torn down the survivor's gz server, leaving it wedged. The lock is an fcntl
# record lock: the kernel releases it the moment the holder dies, so killing the
# process is the fix. Deleting the lock FILE is not — that only detaches the lock
# from its path and lets a second instance-0 PX4 boot beside the first, two
# vehicles in one world on the same MAVLink ports.
if pgrep -f 'px4_sitl_default/bin/px[4]' >/dev/null 2>&1; then
    echo "[launch] a previous PX4 is still running — stopping it (it holds the"
    echo "[launch] instance-0 lock, which would block this launch)."
    pkill -9 -f 'px4_sitl_default/bin/px[4]' 2>/dev/null || true
    for _ in $(seq 1 20); do
        pgrep -f 'px4_sitl_default/bin/px[4]' >/dev/null 2>&1 || break
        sleep 0.5
    done
fi
sleep 1
echo "[launch] pre-starting gz server for world '${PX4_GZ_WORLD}'…"
# DISPLAY/WAYLAND_DISPLAY were unset for this whole script above (see the GPU
# note): with no windowing, gz-rendering-ogre2 has no GLX fallback and must keep
# the NVIDIA EGL surfaceless context for the offscreen camera render.
gz sim -s -r -v1 "$WORLD_SDF" >/tmp/aavc_gzserver.log 2>&1 &
GZ_SERVER_PID=$!
# Trailing `|| true` keeps `set -e` from aborting the trap when the first kill
# fails (server already dead) — otherwise the pkill sweep is skipped (stale
# `gz sim` blocks the next launch) AND a clean shutdown reports rc=1.
trap 'kill "$GZ_SERVER_PID" 2>/dev/null; pkill -9 -f "$GZ_PATTERN" 2>/dev/null; true' EXIT INT TERM
gz_ready=0
for _ in $(seq 1 40); do
    if gz service --list 2>/dev/null | grep -q "/world/${PX4_GZ_WORLD}/create"; then
        echo "[launch] gz server ready — starting PX4."
        gz_ready=1
        break
    fi
    sleep 1
done
if [ "$gz_ready" != "1" ]; then
    echo "[launch] WARNING: gz create service did not appear after 40 s — the "
    echo "[launch] gz_bridge spawn race may recur (see /tmp/aavc_gzserver.log)."
fi

# The viewer, if asked for: a separate process, with its display handed back.
# It renders the full scene continuously on the same wedge-prone dGPU as the
# camera sensors, which is why it is opt-in.
if [ "$WANT_GUI" = "1" ]; then
    if [ -n "$GUI_DISPLAY" ] || [ -n "$GUI_WAYLAND" ]; then
        echo "[launch] starting the gz viewer (GUI=1)…"
        DISPLAY="$GUI_DISPLAY" WAYLAND_DISPLAY="$GUI_WAYLAND" \
            gz sim -g -v1 >/tmp/aavc_gzgui.log 2>&1 &
    else
        echo "[launch] GUI=1 but no DISPLAY/WAYLAND_DISPLAY — skipping the viewer."
    fi
fi

# NOT exec — so the trap above tears the gz server down when PX4 exits.
make px4_sitl "$AIRFRAME"
