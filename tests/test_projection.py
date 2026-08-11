"""Pixel→world geo-projection for the single-camera rig (vision.projection).

Invariants the land-and-drop loop depends on:

* The NADIR camera (optical axis straight down, gimbal-stabilized on the real
  bird) projects an image-CENTRED pixel to ~the drone's own ground position —
  the basis of the precise visual-servo alignment.
* The GENERAL depression math must stay correct for any mount angle: the G6
  camera calibration trims a residual gimbal pitch error via
  ``depression_deg``, so a depressed model is exercised with a LOCAL
  CameraModel (the dedicated oblique cue camera was retired with the
  single-OV9281 hardware decision).
"""

from __future__ import annotations

import math

from vision.projection import (
    NADIR,
    CameraModel,
    expected_radius_px,
    pixel_to_world,
    project_pixel,
)

# A drone hovering at a known fix and AGL, nose pointing North (yaw 0).
_LAT = 14.0
_LON = 100.0
_ALT = 15.0
_YAW = 0.0

# A 45°-depressed model (the retired oblique mount angle) — kept as a LOCAL
# fixture so the general ray→ground math retains coverage for non-nadir
# depressions (gimbal trim errors are exactly small versions of this).
_DEPRESSED = CameraModel(name="depressed45", depression_rad=math.pi / 4)


def test_nadir_centre_pixel_projects_to_drone_position() -> None:
    """The centre pixel of the nadir frame projects ~straight below the drone."""
    cx, cy = NADIR.width_px // 2, NADIR.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, NADIR)

    assert fix is not None
    # Centre of a straight-down camera = the drone's own ground point.
    assert fix.ground_dist_m < 1e-6
    assert abs(fix.forward_m) < 1e-6
    assert abs(fix.right_m) < 1e-6
    # lat/lon unchanged to well below a metre.
    assert math.isclose(fix.lat, _LAT, abs_tol=1e-7)
    assert math.isclose(fix.lon, _LON, abs_tol=1e-7)


def test_nadir_offset_pixel_lands_within_footprint() -> None:
    """A pixel to the image-right maps to a point East of the drone (yaw 0)."""
    cx = NADIR.width_px // 2 + 100
    cy = NADIR.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, NADIR)

    assert fix is not None
    # Right-of-centre with the nose North → East (lon increases), still on ground.
    assert fix.right_m > 0.0
    assert fix.lon > _LON
    assert math.isclose(fix.lat, _LAT, abs_tol=1e-7)
    # Within the camera footprint half-width at this altitude.
    half_footprint = _ALT * math.tan(NADIR.fov_rad / 2.0)
    assert fix.ground_dist_m < half_footprint


def test_depressed_camera_projects_forward_of_drone() -> None:
    """A 45°-depressed camera sees AHEAD of the nose, not straight down."""
    cx, cy = _DEPRESSED.width_px // 2, _DEPRESSED.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, _DEPRESSED)

    assert fix is not None
    # Forward of the drone (+ = ahead of the nose) and on the ground ahead.
    assert fix.forward_m > 0.0
    assert fix.ground_dist_m > 0.0
    # At a 45° depression the forward ground offset ≈ the altitude.
    assert math.isclose(fix.forward_m, _ALT, rel_tol=1e-3)
    # Nose North → the projected target is North of the drone.
    assert fix.lat > _LAT
    assert math.isclose(fix.lon, _LON, abs_tol=1e-7)
    # And farther out than the nadir centre point (which is directly below).
    nadir = project_pixel((NADIR.width_px // 2, NADIR.height_px // 2),
                          _LAT, _LON, _ALT, _YAW, NADIR)
    assert nadir is not None
    assert fix.ground_dist_m > nadir.ground_dist_m


def test_yaw_rotates_depressed_bearing() -> None:
    """Nose East (yaw 90°) puts the depressed-centre target East of the drone."""
    cx, cy = _DEPRESSED.width_px // 2, _DEPRESSED.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, 90.0, _DEPRESSED)

    assert fix is not None
    assert fix.lon > _LON                          # East
    assert math.isclose(fix.lat, _LAT, abs_tol=1e-7)


def test_zero_altitude_returns_none() -> None:
    """No usable AGL → no projection (guards the visual-servo loop)."""
    cx, cy = NADIR.width_px // 2, NADIR.height_px // 2
    assert project_pixel((cx, cy), _LAT, _LON, 0.0, _YAW, NADIR) is None


# ── attitude composition (the high-altitude stability fix) ──────────────────
# A translating quad holds 10-15° of tilt; without composing it the nadir
# projection drifts by ~alt·tan(tilt) and the close-in loop chases its attitude.


def test_pitch_nose_down_projects_nadir_aft() -> None:
    """Nose-down pitch (−θ) tips the nadir ray aft → forward_m = −alt·tan θ."""
    cx, cy = NADIR.width_px // 2, NADIR.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, NADIR, pitch_deg=-10.0)
    assert fix is not None
    assert math.isclose(fix.forward_m, -_ALT * math.tan(math.radians(10.0)), rel_tol=1e-6)
    assert abs(fix.right_m) < 1e-9
    # Nose-down at yaw 0 (North) → the projected point is SOUTH of the drone.
    assert fix.lat < _LAT


def test_roll_right_wing_down_projects_nadir_left() -> None:
    """Right-wing-down roll (+φ) tips the nadir ray left → right_m = −alt·tan φ."""
    cx, cy = NADIR.width_px // 2, NADIR.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, NADIR, roll_deg=10.0)
    assert fix is not None
    assert math.isclose(fix.right_m, -_ALT * math.tan(math.radians(10.0)), rel_tol=1e-6)
    assert abs(fix.forward_m) < 1e-9
    # Left of a North-facing drone = West → lon decreases.
    assert fix.lon < _LON


def test_depressed_camera_pitched_to_nadir_projects_straight_down() -> None:
    """A 45°-depressed camera at −45° pitch looks straight down → ground_dist ≈ 0.

    This is the attitude-composition invariant the gimbal relies on: mount
    depression and vehicle pitch compose into one effective ray direction."""
    cx, cy = _DEPRESSED.width_px // 2, _DEPRESSED.height_px // 2
    fix = project_pixel((cx, cy), _LAT, _LON, _ALT, _YAW, _DEPRESSED, pitch_deg=-45.0)
    assert fix is not None
    assert fix.ground_dist_m < 1e-6


def test_zero_attitude_is_exactly_the_legacy_path() -> None:
    """roll=pitch=0 must reproduce the no-attitude projection bit-for-bit."""
    for cam in (NADIR, _DEPRESSED):
        for px in ((cam.width_px // 2, cam.height_px // 2),
                   (cam.width_px // 2 + 73, cam.height_px // 2 - 41)):
            base = project_pixel(px, _LAT, _LON, _ALT, _YAW, cam)
            same = project_pixel(px, _LAT, _LON, _ALT, _YAW, cam,
                                 roll_deg=0.0, pitch_deg=0.0)
            assert base is not None and same is not None
            assert (base.lat, base.lon, base.forward_m, base.right_m) == (
                same.lat, same.lon, same.forward_m, same.right_m)


def test_expected_radius_px_follows_pinhole_law() -> None:
    """r_px = fx·R/d — halve the range, double the apparent radius."""
    r_far = expected_radius_px(NADIR, 0.75, 16.0)
    r_near = expected_radius_px(NADIR, 0.75, 8.0)
    assert math.isclose(r_far, NADIR.fx_px * 0.75 / 16.0, rel_tol=1e-9)
    assert math.isclose(r_near, 2.0 * r_far, rel_tol=1e-9)


def test_backcompat_wrapper_returns_latlon_tuple() -> None:
    """The 2-tuple wrapper keeps returning (lat, lon) for legacy callers."""
    cx, cy = NADIR.width_px // 2, NADIR.height_px // 2
    nadir = pixel_to_world((cx, cy), _LAT, _LON, _ALT, _YAW, NADIR)

    assert nadir is not None and len(nadir) == 2
    assert math.isclose(nadir[0], _LAT, abs_tol=1e-7)


def test_projection_module_has_no_oblique_surface() -> None:
    """The oblique cue camera was retired with the single-OV9281 decision —
    lock the deletion so it can't silently return."""
    import vision.projection as P

    assert not hasattr(P, "OBLIQUE")
    assert not hasattr(P, "pixel_to_world_oblique")
