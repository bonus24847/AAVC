# PX4 patches AAVC needs in its SITL tree

The authoritative copy lives on branch **`aavc/sitl-v1.17`** of the PX4 v1.17.0
worktree at `~/PX4-Autopilot-v1.17` (that is what `sitl/launch_sitl.sh` runs).
These files are a provenance copy so the patch survives losing the worktree, and
so the airframe is reviewable in this repo's history.

| File | Goes to |
|---|---|
| `22000_gz_eft_x6100` | `ROMFS/px4fmu_common/init.d-posix/airframes/` |
| `px4-v1.17-aavc.diff` | `git apply` at the PX4 tree root (registers the airframe + one gz_env fix) |

## Re-creating the tree from scratch

```bash
git -C ~/PX4-Autopilot worktree add ~/PX4-Autopilot-v1.17 v1.17.0
cd ~/PX4-Autopilot-v1.17
git switch -c aavc/sitl-v1.17
git submodule update --init --recursive Tools/simulation/gz    # before the first build
cp <repo>/sitl/px4_patches/22000_gz_eft_x6100 ROMFS/px4fmu_common/init.d-posix/airframes/
git apply <repo>/sitl/px4_patches/px4-v1.17-aavc.diff
GZ_DISTRO=harmonic make px4_sitl
```

## What each patch is for

**`22000_gz_eft_x6100`** — the EFT X6100 hexacopter SITL airframe. 22000 is
PX4's reserved range for custom models, so it cannot collide with an upstream
addition. Its `CA_ROTOR*` table mirrors the real 6X's airframe-6001 allocation
(read off the board 2026-07-22) scaled to the X6100's 0.500 m arm radius
(1,000 mm wheelbase, Power-System-Guide-1.pdf), so SITL and
the aircraft allocate identically. The matching Gazebo model is
`sitl/models/eft_x6100` in this repo.

Registering it in `airframes/CMakeLists.txt` is not optional: the `gz_<model>`
make target comes from a configure-time glob, and editing that CMake file is
what triggers the re-configure that creates the target.

**`gz_env.sh.in`** — makes `PX4_GZ_MODELS` respect an ambient value
(`${PX4_GZ_MODELS:-…}`). v1.17 spawns the vehicle from an absolute
`file://$PX4_GZ_MODELS/<model>/model.sdf`, so the launcher exports that variable
to point at this repo's `sitl/models`. The patch only matters on the fallback
path where PX4 starts the Gazebo server itself (our launcher pre-starts it), but
without it that path silently spawns from PX4's own model dir — where
`eft_x6100` does not exist.
