"""Pixel→world geo-projection for the AAVC single-camera rig.

One camera (matching ``sitl/models/x500_mono_cam/model.sdf``):
  * NADIR — optical axis straight down (mount pitch +π/2; gimbal-stabilized
    on the real bird).  Drives target discovery AND the precise final
    alignment + land-and-drop loop.

It projects an image-frame pixel centroid to a ground (lat, lon) using a
general **ray→flat-ground intersection** model.  The model is built so the
straight-down case reduces *exactly* to the original nadir formula
(known-good against the SITL camera), while ``depression_rad`` stays fully
general — a residual gimbal pitch error is calibrated at G6 as a small
depression offset, exercising the same math the retired 45° oblique cue
camera once did.

Assumptions (v1, adequate for the AAVC field):
  * Flat ground at z = 0 below the drone's reported AGL altitude.
  * Drone roll/pitch ARE composed into the ray (``project_pixel`` takes
    ``roll_deg``/``pitch_deg``).  This matters because a translating quad holds
    10-15° of tilt, and an uncompensated nadir projection then drifts by
    ~alt·tan(tilt) (≈2 m at 12 m / 10°) — the close-in loop would chase its own
    attitude.  Both default to 0.0, so a hover (or a caller that omits them)
    reduces exactly to the legacy straight-down formula.

Sign convention: image origin top-left, x→right, y→down; drone yaw 0° = North,
increasing clockwise (MAVLink standard); ENU output.  Attitude follows MAVSDK
``attitude_euler``: roll + = right-wing-down, pitch + = nose-up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_R_EARTH_M = 6_378_137.0

# A projected ground point farther than this from the drone is non-physical for
# the sub-km AAVC field — it can only come from a near-horizon ray whose
# ground-intersection distance has blown up. project_pixel returns None past it.
_MAX_GROUND_DIST_M = 500.0

# Back-compat module constants (the SITL camera default; see CameraModel).
# MEASURED 2026-08-17 (was 1.74, an unmeasured placeholder): 50 mm on-screen
# marker at 0.495 m from the real WSD-9781-v12 lens → 85.5 px over three
# frames → fx 847 px on the 1280-px frame → HFOV 74.2° = 1.295 rad (±1°).
# The SITL gz camera and the config cameras: block carry the same value.
CAMERA_FOV_RAD = 1.295
CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480


@dataclass(frozen=True)
class CameraModel:
    """Intrinsics + mounting of one camera on the airframe.

    ``depression_rad`` is the angle of the optical axis BELOW horizontal:
    π/2 = straight down (nadir); smaller values pitch the axis forward.
    Matches the Gazebo ``<pose>`` pitch of the camera link.
    """

    name: str
    fov_rad: float = CAMERA_FOV_RAD          # horizontal field of view
    width_px: int = CAMERA_WIDTH_PX
    height_px: int = CAMERA_HEIGHT_PX
    depression_rad: float = math.pi / 2       # nadir by default

    @property
    def fx_px(self) -> float:
        """Focal length in pixels (square-pixel pinhole)."""
        return (self.width_px / 2.0) / math.tan(self.fov_rad / 2.0)


# The AAVC camera. FOV/res match x500_mono_cam/model.sdf; the real lens is
# calibrated in via configure_cameras() (config `cameras:`) so SITL→field
# transfers.
NADIR = CameraModel(name="nadir", depression_rad=math.pi / 2)


def _cam_overrides(vals: dict[str, float]) -> dict[str, float | int]:
    """Translate a config camera block to ``CameraModel`` field overrides.

    Accepts degrees (``fov_deg``/``depression_deg``) or radians
    (``fov_rad``/``depression_rad``); ``width_px``/``height_px`` as ints.
    """
    out: dict[str, float | int] = {}
    if "fov_rad" in vals:
        out["fov_rad"] = float(vals["fov_rad"])
    elif "fov_deg" in vals:
        out["fov_rad"] = math.radians(float(vals["fov_deg"]))
    if "depression_rad" in vals:
        out["depression_rad"] = float(vals["depression_rad"])
    elif "depression_deg" in vals:
        out["depression_rad"] = math.radians(float(vals["depression_deg"]))
    if "width_px" in vals:
        out["width_px"] = int(vals["width_px"])
    if "height_px" in vals:
        out["height_px"] = int(vals["height_px"])
    return out


def configure_cameras(*, nadir: dict[str, float] | None = None) -> None:
    """Apply real-lens calibration to the shared NADIR model IN PLACE.

    The flight core imports the module singleton ``NADIR`` by name and reuses
    it everywhere (``vision_worker``, ``target_tracker``, ``tactical_align``),
    so this mutates that exact object's fields — every already-imported
    reference then sees the calibrated values — rather than rebinding the
    module global (which would not reach names already imported).
    Call ONCE at orchestrator start from the config ``cameras:`` block; ``fx_px``
    recomputes from the updated fov/width. SITL defaults stay untouched when no
    config is given (so the unit tests, which never call this, are unaffected).
    """
    for key, value in _cam_overrides(nadir or {}).items():
        object.__setattr__(NADIR, key, value)   # frozen dataclass: deliberate


@dataclass(frozen=True)
class GroundFix:
    """Result of projecting a pixel to the ground."""

    lat: float
    lon: float
    ground_dist_m: float       # horizontal distance drone→target on the ground
    slant_range_m: float       # straight-line camera→target distance
    forward_m: float           # body-frame forward offset (+ = ahead of nose)
    right_m: float             # body-frame right offset (+ = starboard)


def _enu_to_latlon(
    east_m: float, north_m: float, lat0: float, lon0: float
) -> tuple[float, float]:
    dlat = math.degrees(north_m / _R_EARTH_M)
    # Guard the longitude scale against the cos→0 singularity at the poles. AAVC
    # flies at ~14°N so this never bites in practice; it just keeps a mis-set
    # origin (e.g. lat0=±90 from a config typo) from raising ZeroDivisionError.
    cos_lat0 = math.cos(math.radians(lat0))
    if abs(cos_lat0) < 1e-9:
        cos_lat0 = 1e-9
    dlon = math.degrees(east_m / (_R_EARTH_M * cos_lat0))
    return lat0 + dlat, lon0 + dlon


def project_pixel(
    pixel_xy: tuple[float, float],
    drone_lat: float,
    drone_lon: float,
    drone_alt_agl_m: float,
    drone_yaw_deg: float,
    camera: CameraModel = NADIR,
    *,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
) -> GroundFix | None:
    """General ray→flat-ground projection for either camera.

    ``roll_deg``/``pitch_deg`` are the drone's attitude (MAVSDK convention:
    roll + = right-wing-down, pitch + = nose-up); both default to 0.0 (hover /
    legacy callers). Returns ``None`` when the altitude is unreliable or the ray
    does not point at the ground (e.g. a frame-edge pixel pushed above the
    horizon once attitude is applied).
    """
    if drone_alt_agl_m <= 0.1:
        return None

    cx, cy = pixel_xy
    fx = camera.fx_px
    # Ray in a camera frame (x=right, y=up-flipped, z=forward).  Flip image-v so
    # "up in image" is positive — this makes the NADIR case reduce exactly to
    # the legacy straight-down formula (see module docstring).
    rx = (cx - camera.width_px / 2.0) / fx
    ry = -(cy - camera.height_px / 2.0) / fx
    rz = 1.0

    # Rotate ray camera→body by the mounting depression δ (rotation about the
    # body 'right' axis). Body frame: X=forward, Y=right, Z=up.
    d = camera.depression_rad
    sin_d, cos_d = math.sin(d), math.cos(d)
    vb_forward = ry * sin_d + rz * cos_d
    vb_right = rx
    vb_up = ry * cos_d - rz * sin_d

    # Compose the drone's own roll/pitch so the ray is expressed in a LEVEL
    # (gravity-aligned, heading-kept) frame before the ground intersection —
    # otherwise a tilted quad's nadir projection drifts by ~alt·tan(tilt) and the
    # close-in loop chases its attitude. Body(z-up)→FRD, apply the roll+pitch
    # body→level DCM (yaw is handled below in the ENU step), then FRD→body(z-up).
    # The branch keeps the roll=pitch=0 path bit-for-bit identical (every existing
    # projection invariant/test holds).
    if roll_deg or pitch_deg:
        phi, theta = math.radians(roll_deg), math.radians(pitch_deg)
        s_phi, c_phi = math.sin(phi), math.cos(phi)
        s_th, c_th = math.sin(theta), math.cos(theta)
        f, r, dn = vb_forward, vb_right, -vb_up               # body(z-up) → FRD
        f_l = c_th * f + s_phi * s_th * r + c_phi * s_th * dn
        r_l = c_phi * r - s_phi * dn
        d_l = -s_th * f + s_phi * c_th * r + c_phi * c_th * dn
        vb_forward, vb_right, vb_up = f_l, r_l, -d_l          # FRD → body(z-up)

    if vb_up >= -1e-6:
        # Ray is horizontal or points up — no ground intersection ahead.
        return None

    # Intersect with the ground plane (Δup = -h).
    t = -drone_alt_agl_m / vb_up           # > 0
    forward_m = t * vb_forward
    right_m = t * vb_right
    slant_range_m = t * math.sqrt(
        vb_forward * vb_forward + vb_right * vb_right + vb_up * vb_up
    )
    ground_dist_m = math.hypot(forward_m, right_m)

    # Sanity clamp: a ray grazing the horizon (vb_up just past the reject
    # threshold) makes t = -alt/vb_up blow up, projecting a point hundreds of
    # metres away — non-physical for a sub-km field. Reject it so a hard-tilted
    # frame edge can't emit a wild lat/lon. (A level nadir ray never reaches
    # here — it points straight down, vb_up ≈ -1.)
    if ground_dist_m > _MAX_GROUND_DIST_M:
        return None

    # Body (forward,right) → ENU using yaw (0°=N, CW).
    yaw = math.radians(drone_yaw_deg)
    rel_east = right_m * math.cos(yaw) + forward_m * math.sin(yaw)
    rel_north = -right_m * math.sin(yaw) + forward_m * math.cos(yaw)
    lat, lon = _enu_to_latlon(rel_east, rel_north, drone_lat, drone_lon)
    return GroundFix(
        lat=lat, lon=lon, ground_dist_m=ground_dist_m,
        slant_range_m=slant_range_m, forward_m=forward_m, right_m=right_m,
    )


def pixel_to_world(
    pixel_xy: tuple[int, int],
    drone_lat: float,
    drone_lon: float,
    drone_alt_agl_m: float,
    drone_yaw_deg: float,
    camera: CameraModel = NADIR,
    *,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
) -> tuple[float, float] | None:
    """Back-compat thin wrapper: returns just (lat, lon).

    Kept so existing callers/tests that expect a 2-tuple keep working. New
    code should call :func:`project_pixel` for the full :class:`GroundFix`.
    """
    fix = project_pixel(
        pixel_xy, drone_lat, drone_lon, drone_alt_agl_m, drone_yaw_deg, camera,
        roll_deg=roll_deg, pitch_deg=pitch_deg,
    )
    return (fix.lat, fix.lon) if fix is not None else None


def expected_radius_px(
    camera: CameraModel, target_radius_m: float, slant_range_m: float
) -> float:
    """Apparent radius (px) of a ``target_radius_m`` object at ``slant_range_m``.

    Pinhole law ``r_px = fx · R / d``. Used as a size prior (paper B): a hit
    whose characteristic size (a circle's radius, or a marker's half-side
    axis) is far from this — given the known target size and the fix's slant
    range — is geometry the target can't produce, so it is rejected before it can
    vote/steer. Independent of the detector (kept pure).
    """
    return camera.fx_px * target_radius_m / max(slant_range_m, 0.1)


def ground_sample_distance_m_per_px(
    camera: CameraModel, drone_alt_agl_m: float
) -> float:
    """Metres-per-pixel at image centre for the NADIR footprint (alt-dependent).

    Used to convert a pixel centring error into a metric correction for the
    visual-servo loop. For a depressed (non-nadir) axis this is only a rough
    scale.
    """
    if drone_alt_agl_m <= 0.0:
        return 0.0
    footprint_w_m = 2.0 * drone_alt_agl_m * math.tan(camera.fov_rad / 2.0)
    return footprint_w_m / camera.width_px
