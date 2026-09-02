#!/usr/bin/env bash
# AAVC 2026 — One-shot setup for PX4 SITL + Gazebo Harmonic + Python deps.
# Tested on Ubuntu 24.04 LTS.
#
# This script does NOT auto-run sudo commands. It prints what to do and
# asks for confirmation at each tier.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_VERSION="${PX4_VERSION:-v1.17.0}"
# The flight stack does not run on a bare PX4 tree: the hexacopter airframe is an
# AAVC patch (sitl/px4_patches/), and every launcher defaults PX4_DIR here. Step
# 3 builds it, because "run ./sitl/setup.sh first" was the launcher's advice for a
# missing worktree and setup.sh could not produce one — a closed loop on a fresh
# machine, with the real recipe documented only in sitl/px4_patches/README.md.
AAVC_PX4_DIR="${AAVC_PX4_DIR:-$HOME/PX4-Autopilot-v1.17}"
AAVC_PX4_BRANCH="aavc/sitl-v1.17"

say() { printf "\n\033[1;36m[setup]\033[0m %s\n" "$*"; }
ask() { read -p "$1 [y/N] " -n 1 -r; echo; [[ $REPLY =~ ^[Yy]$ ]]; }

say "Checking host environment..."
if ! command -v lsb_release >/dev/null; then
    echo "Warning: lsb_release missing; cannot confirm Ubuntu version."
else
    lsb_release -a
fi

say "Step 1/5 — System packages (requires sudo)"
echo "PX4 + Gazebo Harmonic need a number of system packages. To install:"
cat <<'EOF'

  sudo apt update
  sudo apt install -y \
      build-essential cmake ninja-build git ccache python3 python3-pip \
      python3-venv python3-jinja2 python3-numpy python3-yaml python3-toml \
      python3-setuptools python3-empy \
      lsb-release wget curl gnupg
EOF
# NOTE: `python3-pyros-genmsg` is a ROS-era package that older PX4 docs
# reference. It is NOT in Ubuntu 24.04 repos and PX4 v1.15+ does not need
# it — code generation has moved to in-tree Tools/. Removed from this list.
if ask "Run the apt install commands above now?"; then
    sudo apt update
    sudo apt install -y \
        build-essential cmake ninja-build git ccache python3 python3-pip \
        python3-venv python3-jinja2 python3-numpy python3-yaml python3-toml \
        python3-setuptools python3-empy \
        lsb-release wget curl gnupg
fi

say "Step 2/5 — Gazebo Harmonic"
echo "Gazebo Harmonic uses the gz-tools binary repository:"
cat <<'EOF'

  sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt update
  sudo apt install -y gz-harmonic
EOF
if ask "Run the Gazebo Harmonic install above now?"; then
    sudo wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    sudo apt update
    sudo apt install -y gz-harmonic
fi

say "Step 3/5 — PX4-Autopilot source (will clone to $PX4_DIR)"
if [ -d "$PX4_DIR/.git" ]; then
    echo "PX4 dir already exists at $PX4_DIR."
    if ask "Pull latest and checkout $PX4_VERSION?"; then
        (cd "$PX4_DIR" && git fetch --all --tags && git checkout "$PX4_VERSION" && \
             git submodule sync --recursive && git submodule update --init --recursive)
    fi
else
    if ask "Clone PX4-Autopilot to $PX4_DIR?"; then
        git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
        (cd "$PX4_DIR" && git checkout "$PX4_VERSION" && \
             git submodule update --init --recursive)
        # Bash setup script provided by PX4 for additional toolchain bits
        bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools
    fi
fi

say "Step 4/5 — AAVC PX4 worktree ($AAVC_PX4_DIR, branch $AAVC_PX4_BRANCH)"
# A worktree of the clone above, pinned to v1.17.0 and carrying the two AAVC
# patches. Kept separate from $PX4_DIR so the legacy tree stays as a rollback.
if [ -d "$AAVC_PX4_DIR" ]; then
    echo "Already present at $AAVC_PX4_DIR."
elif [ ! -d "$PX4_DIR/.git" ]; then
    echo "Skipped: $PX4_DIR is not a PX4 checkout yet (do step 3 first)."
elif ask "Create the AAVC worktree and apply sitl/px4_patches?"; then
    git -C "$PX4_DIR" worktree add "$AAVC_PX4_DIR" "$PX4_VERSION"
    git -C "$AAVC_PX4_DIR" switch -c "$AAVC_PX4_BRANCH"
    # gz submodule BEFORE the first cmake configure — the worlds glob feeds
    # target generation, so a later init leaves you with no gz_* make targets.
    git -C "$AAVC_PX4_DIR" submodule update --init --recursive Tools/simulation/gz
    cp "$REPO_ROOT/sitl/px4_patches/22000_gz_eft_x6100" \
       "$AAVC_PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
    git -C "$AAVC_PX4_DIR" apply "$REPO_ROOT/sitl/px4_patches/px4-v1.17-aavc.diff"
    git -C "$AAVC_PX4_DIR" add -A
    git -C "$AAVC_PX4_DIR" commit -qm "AAVC: EFT X6100 airframe + gz_env model-path fix"
    echo "Building (5-15 min)…"
    ( cd "$AAVC_PX4_DIR" && GZ_DISTRO=harmonic make px4_sitl )
fi

say "Step 5/5 — Python venv for aavc-2026"
if [ ! -d "$REPO_ROOT/.venv" ]; then
    "${PYTHON:-python3}" -m venv "$REPO_ROOT/.venv"   # PYTHON= overrides; needs >= 3.12
fi
"$REPO_ROOT/.venv/bin/pip" install --upgrade pip
"$REPO_ROOT/.venv/bin/pip" install -e "$REPO_ROOT[dev]"

say "Done. Next:"
echo "  - source .venv/bin/activate"
echo "  - make sitl            # hexacopter on $AAVC_PX4_DIR"
echo ""
echo "If the build needs redoing by hand:"
echo "  (cd $AAVC_PX4_DIR && GZ_DISTRO=harmonic make px4_sitl)"
echo "Legacy quad rollback (the $PX4_VERSION tree, no AAVC patches):"
echo "  PX4_DIR=$PX4_DIR bash sitl/launch_sitl.sh gz_x500_mono_cam"
