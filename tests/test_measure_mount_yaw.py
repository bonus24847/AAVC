"""The mount measurement and the projection must agree, or both are wrong.

``tools/measure_mount_yaw.py`` reads ``cameras.nadir.mount_yaw_deg`` off a
single frame; ``vision/projection.py`` then consumes that number. They are
written in different coordinate conventions — image rows grow DOWNWARD, the
projection's ray grows UPWARD, and the mount angle is measured clockwise from
above — so a sign slip in either one produces a perfectly plausible number that
rotates every pad fix by 90 or 180 degrees. Nothing on the aircraft would
complain: the pads simply come out somewhere else, the aircraft flies to empty
grass, and the tracker's cluster never confirms.

These tests close the loop instead of checking each half against a comment: put
an object at a pixel, ask the tool what mount yaw that implies, feed that yaw to
the projection, and require the object to come back out straight ahead of the
nose.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from measure_mount_yaw import mount_yaw_deg  # noqa: E402

from vision.projection import CameraModel, project_pixel  # noqa: E402

_W, _H = 1280, 720
_LAT, _LON, _ALT = 13.8228032, 100.5116267, 1.0


def _bearing_from_drone(fix, heading_deg: float) -> float:
    """Direction of a fix relative to the drone's NOSE, degrees clockwise."""
    dn = fix.lat - _LAT
    de = (fix.lon - _LON) * math.cos(math.radians(_LAT))
    return (math.degrees(math.atan2(de, dn)) - heading_deg) % 360.0


@pytest.mark.parametrize("pixel, expect", [
    ((640, 120), 0.0),      # object appears ABOVE centre  -> image-up is the nose
    ((160, 360), 90.0),     # …LEFT   -> camera turned 90 deg clockwise
    ((640, 600), 180.0),    # …BELOW  -> camera upside down
    ((1120, 360), 270.0),   # …RIGHT  -> camera turned 270
])
def test_the_four_bolt_positions_read_out_as_the_named_angle(pixel, expect):
    u, v = pixel
    assert abs(mount_yaw_deg(u, v, _W, _H) - expect) < 1e-9


@pytest.mark.parametrize("pixel", [(640, 120), (160, 360), (640, 600),
                                   (1120, 360), (900, 200), (410, 545)])
@pytest.mark.parametrize("heading", [0.0, 37.0, 210.0])
def test_the_measured_yaw_makes_the_projection_point_at_the_nose(pixel, heading):
    """The round trip. Whatever pixel the object sits at, the yaw this tool
    reports must be the one that makes the projection agree the object is
    STRAIGHT AHEAD — for any aircraft heading, since the mount is a property of
    the airframe and not of where it happens to be pointing."""
    u, v = pixel
    psi = mount_yaw_deg(u, v, _W, _H)
    cam = CameraModel(name="nadir", width_px=_W, height_px=_H,
                      mount_yaw_rad=math.radians(psi))
    fix = project_pixel((u, v), _LAT, _LON, _ALT, heading, cam)
    assert fix is not None
    rel = _bearing_from_drone(fix, heading)
    assert min(rel, 360.0 - rel) < 0.05, (
        f"pixel {pixel} at mount {psi:.1f} deg should project straight ahead, "
        f"came out {rel:.2f} deg off the nose")


def test_an_object_placed_off_the_nose_is_accounted_for():
    """Placing the object dead ahead is the easy instruction, not the only one:
    state where it really is and the reading still comes out the same."""
    u, v = 160, 360                      # object appears LEFT of centre
    straight_ahead = mount_yaw_deg(u, v, _W, _H)
    assert abs(straight_ahead - 90.0) < 1e-9
    # Same frame, but the object was actually 90 deg to the RIGHT of the nose:
    # the mount must then read 90 less.
    assert abs(mount_yaw_deg(u, v, _W, _H, object_bearing_deg=90.0)) < 1e-9
    # …and 90 to the LEFT reads 90 more.
    assert abs(mount_yaw_deg(u, v, _W, _H, object_bearing_deg=-90.0) - 180.0) < 1e-9


def test_the_shipped_default_is_the_unrotated_mount():
    """0.0 in the config means "image-up is the nose" — the assumption the
    whole flight core has been making. Pinned so the day it is MEASURED the
    change is deliberate."""
    assert abs(mount_yaw_deg(_W / 2.0, 100.0, _W, _H)) < 1e-9
