"""The console's auto-infra ssh must put the CM4 camera back on the DAYTIME
settings before it starts the grabber.

Bang Bo night test, 29-30 Aug 2026: a `~/.bashrc` hook on the CM4, gated on
`~/.aavc_night_cam`, exports CAM_EXPOSURE=180 CAM_GAIN=128 (18 ms + max gain)
and sets v4l2 exposure_dynamic_framerate=1 so the floodlit pitch was not black.
The aircraft was powered off before the flag could be removed. In sunlight those
settings blow every frame to white and the mission decodes nothing all day, so
the console — the first thing that ssh-es the CM4 on a field morning — removes
the flag and starts the infra with the CAM_* env UNSET. The remote shell sources
~/.bashrc BEFORE the command runs, so the env must be unset explicitly; the
sourced hook's v4l2 flag is reset explicitly too; a grabber already running on
the night args is restarted (only that one — a daytime grabber is left alone).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import aavc_gcs  # noqa: E402


def test_infra_remote_cmd_restores_the_daytime_camera_before_start_infra():
    cmd = aavc_gcs._infra_remote_cmd("/home/drone/mission")
    assert "rm -f ~/.aavc_night_cam" in cmd
    assert "exposure_dynamic_framerate=0" in cmd
    assert 'pgrep -f "camera_grabbe[r].py.*--gain 128"' in cmd
    assert "env -u CAM_EXPOSURE -u CAM_GAIN -u CAM_AE_MAX" in cmd
    assert cmd.rstrip().endswith("/home/drone/mission/cm4/start_infra.sh")
    # order: restore first, start_infra last
    assert cmd.index("rm -f ~/.aavc_night_cam") < cmd.index("start_infra.sh")


def test_infra_remote_cmd_never_kills_a_daytime_grabber():
    cmd = aavc_gcs._infra_remote_cmd("/home/drone/mission")
    # the kill is conditional on the night argv, never a bare pkill of the grabber
    assert 'pgrep -f "camera_grabbe[r].py.*--gain 128" >/dev/null && pkill -f "camera_grabbe[r].py"' in cmd
    assert cmd.count("pkill") == 1
