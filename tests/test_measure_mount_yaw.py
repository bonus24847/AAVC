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


@pytest.mark.parametrize("bearing", [0.0, 90.0, 180.0, 270.0, 37.0, -125.0])
@pytest.mark.parametrize("pixel", [(160, 360), (640, 120), (900, 200), (410, 545)])
def test_an_object_placed_off_the_nose_is_accounted_for(pixel, bearing):
    """Placing the object dead ahead is the easy instruction, not the only one:
    state where it really is and the reading still comes out the same.

    This closes the loop through the projection rather than re-checking the
    tool's own arithmetic, which is how the sign slip below survived: the old
    version asserted ``psi = ang - b`` against nothing but itself, and
    ``ang - b`` happens to equal ``ang + b`` at exactly the two bearings it
    tested against (0 and 180). At the right wing it was wrong by 180 deg.
    """
    u, v = pixel
    psi = mount_yaw_deg(u, v, _W, _H, object_bearing_deg=bearing)
    cam = CameraModel(name="nadir", width_px=_W, height_px=_H,
                      mount_yaw_rad=math.radians(psi))
    fix = project_pixel((u, v), _LAT, _LON, _ALT, 0.0, cam)
    assert fix is not None
    rel = _bearing_from_drone(fix, 0.0)
    off = min((rel - bearing) % 360.0, (bearing - rel) % 360.0)
    assert off < 0.05, (
        f"object at pixel {pixel} declared {bearing} deg off the nose implies "
        f"mount {psi:.1f}; the projection then puts it {rel:.1f} deg off")


# The real bench measurement, 2026-08-23 at KMUTNB: the aircraft raised ~0.75 m
# on a crate, level, camera looking down at the floor, and ONE object walked
# clockwise round it — nose, right wing, tail, left wing — one nadir frame each.
# These are the pixels those four frames put it at. They are kept as data (not a
# comment) because they are the only evidence the shipped 180.0 rests on, and
# because four independent placements agreeing is what makes it evidence rather
# than a reading: the sign bug above shows a single nose-only placement cannot
# tell 180 from a formula that is wrong everywhere else.
_BENCH_2026_08_23 = [
    ((641, 530),   0.0, 180.3),   # marker sheet straight out from the NOSE
    ((295, 405),  90.0, 187.4),   # filament spool at the RIGHT wing
    ((622,  58), 180.0, 183.4),   # …at the TAIL
    ((970, 325), 270.0, 186.1),   # …at the LEFT wing
]


@pytest.mark.parametrize("pixel, bearing, expect", _BENCH_2026_08_23)
def test_the_bench_measurement_reads_the_mount_as_bolted(pixel, bearing, expect):
    """Each placement alone must land within a few degrees of 180, and all four
    must snap to the same bolted angle. Placing an object by eye is worth a few
    degrees, so the tolerance is 8 — but a SIGN error is worth 180, which is the
    failure this pins."""
    u, v = pixel
    psi = mount_yaw_deg(u, v, _W, _H, object_bearing_deg=bearing)
    assert abs(psi - expect) < 0.15, f"{pixel} @ {bearing} deg -> {psi:.1f}"
    assert abs(psi - 180.0) < 8.0, "every placement must agree the mount is 180"


def test_the_bench_measurement_is_what_the_config_ships():
    """The four placements average to the number in both field configs. A camera
    re-mount must break this test, not slide through as folklore."""
    import yaml

    mean = sum(psi for _, _, psi in _BENCH_2026_08_23) / len(_BENCH_2026_08_23)
    assert abs(mean - 184.3) < 0.1
    snapped = round(mean / 90.0) * 90.0
    assert snapped == 180.0
    root = Path(__file__).resolve().parents[1] / "sitl"
    for name in ("aavc_config.yaml", "kmitl_config.yaml"):
        cams = yaml.safe_load((root / name).read_text())["cameras"]
        assert cams["nadir"]["mount_yaw_deg"] == snapped, name


def test_the_formula_reads_an_object_above_centre_as_the_unrotated_mount():
    """An object that appears ABOVE the frame centre while sitting straight out
    from the nose means image-up IS the nose, i.e. mount 0. That is the
    arithmetic, not the aircraft: this airframe measured 180 (the camera is
    bolted upside down) — see ``test_the_bench_measurement_reads_the_mount_as_bolted``."""
    assert abs(mount_yaw_deg(_W / 2.0, 100.0, _W, _H)) < 1e-9


# ── the frame must be a picture of NOW ───────────────────────────────────────
# This tool reads one file and its answer gets typed into a config that shapes
# every pad fix afterwards. On 2026-08-23 the OV9281 browned out and
# re-enumerated while the grabber kept running on the dead descriptor: three
# "no marker found" results in a row were one 14-minute-old picture of the
# floor, and a stale frame with an object in it would have produced a
# confident WRONG angle with no symptom at all. Fail closed.


def _stale_frame(tmp_path, age_s: float):
    import os
    import time

    import cv2
    import numpy as np

    frame = tmp_path / "nadir.jpg"
    cv2.imwrite(str(frame), np.zeros((720, 1280, 3), np.uint8))
    when = time.time() - age_s
    os.utime(frame, (when, when))
    return frame


def test_frame_age_reads_the_file_clock_and_says_None_when_absent(tmp_path):
    import measure_mount_yaw as tool

    assert tool._frame_age_s(tmp_path / "nope.jpg") is None
    frame = _stale_frame(tmp_path, 900.0)
    age = tool._frame_age_s(frame)
    assert age is not None and 890.0 < age < 910.0
    # never negative, even if the file claims the future (clock skew on the CM4,
    # which boots with no RTC and no NTP)
    fresh = _stale_frame(tmp_path, -50.0)
    assert tool._frame_age_s(fresh) == 0.0


def test_a_stale_frame_is_refused_instead_of_measured(tmp_path, monkeypatch,
                                                      capsys):
    import sys as _sys

    import measure_mount_yaw as tool

    frame = _stale_frame(tmp_path, 900.0)
    monkeypatch.setattr(_sys, "argv", ["measure_mount_yaw", "--frame",
                                       str(frame), "--pixel", "300,200"])
    assert tool.main() == 2, "a 15-minute-old frame must not be measured"
    out = capsys.readouterr().out
    assert "900" in out, out


def test_a_saved_frame_can_still_be_measured_on_purpose(tmp_path, monkeypatch):
    """`--max-age-s 0` is the deliberate escape: measuring from an archived
    frame is legitimate, guessing from a stale live one is not."""
    import sys as _sys

    import measure_mount_yaw as tool

    frame = _stale_frame(tmp_path, 900.0)
    monkeypatch.setattr(_sys, "argv", ["measure_mount_yaw", "--frame",
                                       str(frame), "--pixel", "300,200",
                                       "--max-age-s", "0"])
    assert tool.main() == 0


def test_a_fresh_frame_passes_the_gate(tmp_path, monkeypatch):
    import sys as _sys

    import measure_mount_yaw as tool

    frame = _stale_frame(tmp_path, 0.0)
    monkeypatch.setattr(_sys, "argv", ["measure_mount_yaw", "--frame",
                                       str(frame), "--pixel", "300,200"])
    assert tool.main() == 0


def test_a_missing_frame_is_reported_as_no_usable_frame(tmp_path, monkeypatch):
    import sys as _sys

    import measure_mount_yaw as tool

    monkeypatch.setattr(_sys, "argv", ["measure_mount_yaw", "--frame",
                                       str(tmp_path / "nope.jpg"),
                                       "--pixel", "300,200"])
    assert tool.main() == 2
