"""SITL world assets — the 4 cargo-box dummies at the L&R pad (rules V1.3).

The organiser cargo is a heart-shaped box ~16×7×18 cm (one raw egg inside);
four dummies dress the launch/recovery pad so the sim mirrors the real
resupply scene. Visual set-dressing only: the top face (16×7 cm ≈ 4 px from
20 m) is far below the detector's 18 px blob floor and fails the square-aspect
gate, so there is no false-cue risk — locked here as geometry assertions.
"""

from __future__ import annotations

# stdlib ElementTree is fine HERE: the parsed XML is this repo's own committed
# SDF (trusted, reviewed input — not attacker-controlled), and the lean-deps
# doctrine (§4) rules out adding defusedxml for a test-only parse. Do NOT copy
# this pattern for any XML that crosses a trust boundary.
import math
import xml.etree.ElementTree as ET
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_WORLD = _REPO / "sitl" / "worlds" / "aavc_field.sdf"
_MODEL = _REPO / "sitl" / "models" / "cargo_box" / "model.sdf"
_MODEL_CONFIG = _REPO / "sitl" / "models" / "cargo_box" / "model.config"


def test_cargo_box_model_is_wellformed_and_static() -> None:
    root = ET.parse(_MODEL).getroot()          # raises on malformed XML
    model = root.find("model")
    assert model is not None
    assert (model.findtext("static") or "").strip() == "true"


def test_cargo_box_has_rules_v13_dimensions() -> None:
    """One collision box of exactly 0.16 x 0.07 x 0.18 m (the V1.3 cargo)."""
    root = ET.parse(_MODEL).getroot()
    collisions = root.findall(".//collision")
    assert len(collisions) == 1
    size = (collisions[0].findtext(".//box/size") or "").split()
    assert [float(v) for v in size] == [0.16, 0.07, 0.18]
    # White body + an accent visual (the heart-box print) — at least two visuals.
    assert len(root.findall(".//visual")) >= 2


def test_world_includes_four_cargo_boxes_on_the_launch_pad() -> None:
    root = ET.parse(_WORLD).getroot()
    includes = [
        inc for inc in root.iter("include")
        if (inc.findtext("uri") or "").strip() == "model://cargo_box"
    ]
    assert len(includes) == 4
    names = [(inc.findtext("name") or "").strip() for inc in includes]
    assert len(set(names)) == 4                # distinct instance names
    for inc in includes:
        pose = [float(v) for v in (inc.findtext("pose") or "").split()]
        x, y = pose[0], pose[1]
        # On the 10x10 m launch pad (spans ±5 in ENU), ≥2 m clear of the (0,0)
        # spawn, east side (clear of the H-marker bars at |x|<=2) and inside
        # the corner posts at (±4.5, ±4.5).
        assert 2.4 <= x <= 4.4, f"{inc.findtext('name')}: x={x}"
        assert -4.0 <= y <= 4.0, f"{inc.findtext('name')}: y={y}"


def test_cargo_box_model_config_exists() -> None:
    root = ET.parse(_MODEL_CONFIG).getroot()
    assert (root.findtext("name") or "").strip() == "cargo_box"


# ---------------------------------------------------------------------------
# Task 10: dynamic `cargo_payload` belly cargo — carried by the aircraft via a
# gz DetachableJoint per box (sitl/models/eft_x6100/model.sdf) and shed onto
# the pad on release. Distinct from the STATIC `cargo_box` L&R ground
# dressing tested above: cargo_payload has mass + inertia and loads the
# flight dynamics while aboard (Tier 1).
# ---------------------------------------------------------------------------
_PAYLOAD_MODEL = _REPO / "sitl" / "models" / "cargo_payload" / "model.sdf"
_PAYLOAD_MODEL_CONFIG = _REPO / "sitl" / "models" / "cargo_payload" / "model.config"
_AIRCRAFT_MODEL = _REPO / "sitl" / "models" / "eft_x6100" / "model.sdf"
# base_link_collision_body's world-frame UNDERSIDE (M7, review 2026-07-24):
# sitl/models/eft_x6100_base/model.sdf has local pose z=0.02, box height 0.08
# => half 0.04; the aircraft spawns at world z=0.35 via sitl/launch_sitl.sh's
# PX4_GZ_MODEL_POSE="0,0,0.35,0,0,0" => world z-range [0.33, 0.41] (the same
# arithmetic the world file's own I6 derivation comment re-does by hand).
# Hard-coded rather than parsed out of launch_sitl.sh's shell env var (the
# collision geometry is this repo's own SDF and parseable with the same
# ElementTree already used above; the spawn pose is not XML at all) — a
# regex shell-script parse is disproportionate machinery for one constant,
# and (like _launch_pad_top_z) it is re-derived and cross-checked by hand in
# the world file's own comment at every geometry change, not asserted here.
_BASE_LINK_COLLISION_UNDERSIDE_WORLD_Z = 0.33
# The same PX4_GZ_MODEL_POSE spawn height the constant above is derived from
# (0.33 = 0.35 + 0.02 - 0.04), kept separate because the sensor-occlusion
# checks below need the raw spawn z to convert the world-frame cargo poses
# into the base_link frame the camera and lidar are mounted in.
_AIRCRAFT_SPAWN_WORLD_Z = 0.35
_BASE_MODEL = _REPO / "sitl" / "models" / "eft_x6100_base" / "model.sdf"
# Clearance a belly box must keep from the edge of a downward sensor's cone.
# The DetachableJoint is a rigid weld, so the box does not actually move in
# the camera frame in flight — this is margin against the ~1 mm proud accent
# band, an FOV/mount tweak, and solver jitter at joint bind, not a slop budget.
_SENSOR_CLEARANCE_MARGIN_M = 0.03


def test_cargo_payload_model_is_dynamic_with_mass() -> None:
    root = ET.parse(_PAYLOAD_MODEL).getroot()
    model = root.find("model")
    assert model is not None
    assert (model.findtext("static") or "false").strip().lower() == "false"
    mass = float(model.findtext(".//inertial/mass") or "0")
    assert 0.05 <= mass <= 0.15


def test_cargo_payload_model_config_exists() -> None:
    root = ET.parse(_PAYLOAD_MODEL_CONFIG).getroot()
    assert (root.findtext("name") or "").strip() == "cargo_payload"


def test_aircraft_has_four_detach_topics() -> None:
    """Structural check, not a substring search: a raw `"detach_payload_{i}"
    in txt` test would still pass if all four <plugin> blocks pointed
    <child_model> at the SAME cargo_payload_0 while only the four
    <detach_topic> strings varied — exactly the copy-paste slip that costs a
    real detach. Parse the SDF and pin down that each of the four
    DetachableJoint plugins binds a DISTINCT cargo_payload_N, each paired
    with the MATCHING detach_topic index, all anchored the same way."""
    root = ET.parse(_AIRCRAFT_MODEL).getroot()
    model = root.find("model")
    assert model is not None
    plugins = [
        p for p in model.findall("plugin")
        if p.get("name") == "gz::sim::systems::DetachableJoint"
    ]
    assert len(plugins) == 4

    seen_children: set[str] = set()
    for p in plugins:
        child_model = (p.findtext("child_model") or "").strip()
        parent_link = (p.findtext("parent_link") or "").strip()
        child_link = (p.findtext("child_link") or "").strip()
        detach_topic = (p.findtext("detach_topic") or "").strip()

        assert parent_link == "base_link", child_model
        assert child_link == "payload", child_model
        assert child_model.startswith("cargo_payload_"), child_model
        idx = child_model.removeprefix("cargo_payload_")
        assert idx.isdigit(), child_model
        assert detach_topic == f"/model/eft_x6100/detach_payload_{idx}", (
            f"{child_model} is paired with the wrong topic: {detach_topic}")
        seen_children.add(child_model)

    # Distinctness is the crux of the guard: 4 plugins collapsing onto fewer
    # than 4 child models (the bug class above) shows up as a smaller set.
    assert seen_children == {
        "cargo_payload_0", "cargo_payload_1", "cargo_payload_2", "cargo_payload_3",
    }


def _launch_pad_top_z(world: ET.Element) -> float:
    """The launch-pad slab's TOP face height (world z) — derived from the
    world file's own ``<model name="launch_pad">`` pose + collision box size,
    not a hard-coded literal, so it tracks the slab if its geometry ever
    changes.

    I6 (review 2026-07-24): the belly cargo_payload boxes spawn directly over
    this slab (footprint x,y in [-5,5]) while still belly-strapped to the
    aircraft — their DetachableJoint has not formed yet, since PX4 spawns the
    aircraft seconds after gz starts running (`-r`) — so they rest ON the
    slab, not on the world GROUND PLANE (z=0). The ground plane is the WRONG
    clearance datum: it sits 0.10 m below the slab's real top face, and a box
    cleared only against z=0 can still be a free rigid body in penetrating
    contact with the slab, which gz ejects before any joint can weld it in
    place.
    """
    pad = next(m for m in world.findall("model") if m.get("name") == "launch_pad")
    pose_z = float((pad.findtext("pose") or "0 0 0").split()[2])
    size = [float(v) for v in
            (pad.findtext(".//collision/geometry/box/size") or "").split()]
    return pose_z + size[2] / 2.0


def test_world_includes_four_cargo_payloads() -> None:
    root = ET.parse(_WORLD).getroot()
    world = root.find("world")
    assert world is not None
    includes = [
        inc for inc in world.iter("include")
        if (inc.findtext("uri") or "").strip() == "model://cargo_payload"
    ]
    names = [(inc.findtext("name") or "").strip() for inc in includes]
    assert sorted(n for n in names if n) == [
        "cargo_payload_0",
        "cargo_payload_1",
        "cargo_payload_2",
        "cargo_payload_3",
    ]

    # Bounds on the poses the belly ground-clearance analysis (Task 10
    # report) actually depends on — names/URIs alone don't lock those.
    # Half-height comes from the model's own collision box, not a hardcoded
    # duplicate, so this bound tracks the model if its size ever changes.
    payload_root = ET.parse(_PAYLOAD_MODEL).getroot()
    collisions = payload_root.findall(".//collision")
    assert len(collisions) == 1
    size = [float(v) for v in (collisions[0].findtext(".//box/size") or "").split()]
    half_height = size[2] / 2.0
    launch_pad_top_z = _launch_pad_top_z(world)

    net_moment = [0.0, 0.0]
    for inc in includes:
        pose = [float(v) for v in (inc.findtext("pose") or "").split()]
        x, y, z = pose[0], pose[1], pose[2]
        yaw = pose[5] if len(pose) >= 6 else 0.0
        name = inc.findtext("name")
        net_moment[0] += x
        net_moment[1] += y
        # Belly placement: within the rail-to-rail belly width laterally, and
        # high enough that the box BOTTOM (z - half_height) still clears the
        # LAUNCH-PAD SLAB TOP it actually spawns resting on (I6) — NOT the
        # world ground plane 0.10 m below it.
        #
        # This used to also pin `x == 0.0` ("centred fore/aft, zero net
        # pitch moment" — Task 10 report). That was the wrong invariant and
        # it froze the bug in place: x=0 is precisely where the nadir camera
        # and the downward lidar look from, so the boxes blinded both (see
        # the sensor-clearance tests below, which now own the X placement).
        # The trim property it was really after is the NET moment over all
        # four boxes, asserted once after the loop.
        #
        # Lateral bound is edge-aware, not centre-only: the boxes are yawed
        # 90° so their 0.16 m axis is the lateral one, and a centre-only
        # bound would let an edge run out past the skid rails unnoticed.
        span_y = abs(y) + (size[0] / 2.0 * abs(math.sin(yaw))
                           + size[1] / 2.0 * abs(math.cos(yaw)))
        assert span_y <= 0.2, f"{name}: reaches y={span_y}"
        assert z - half_height >= launch_pad_top_z - 1e-9, (
            f"{name}: bottom z={z - half_height} is below the launch-pad "
            f"slab top ({launch_pad_top_z}) it actually rests on — a free "
            "body there is in penetrating contact before any DetachableJoint "
            "forms (I6)")
        # M7 (review 2026-07-24): the bottom bound above only stops a box from
        # sinking into the slab — nothing previously stopped a future RAISE
        # from pushing it up INTO base_link_collision_body instead, the same
        # class of penetrating-contact bug from the other direction.
        assert z + half_height <= _BASE_LINK_COLLISION_UNDERSIDE_WORLD_Z, (
            f"{name}: top z={z + half_height} is inside "
            f"base_link_collision_body (underside "
            f"{_BASE_LINK_COLLISION_UNDERSIDE_WORLD_Z}) — a free body there "
            "is in penetrating contact with the aircraft's own collision "
            "body before any DetachableJoint forms (I6, mirrored)")

    # The real trim invariant the old `x == 0.0` was reaching for: the four
    # boxes together must add no roll or pitch moment. Signs alternate across
    # both axes, so this also leaves the aircraft balanced again after the
    # first two of the four releases.
    assert net_moment == [0.0, 0.0], (
        f"belly cargo is off-centre as a set: sum(x,y) = {net_moment}")


# ---------------------------------------------------------------------------
# Belly cargo vs the DOWNWARD SENSORS (2026-07-25).
#
# The four boxes were first placed on the base_link centreline (x=0, y spread
# across ±0.14) — which is exactly where both downward sensors look from. The
# nadir camera sits at base_link z=-0.02 and the boxes' TOP face at z=-0.07,
# so 0.18 m of cardboard stood 0.05 m under the lens and filled a 99.7° x
# 67.3° cone: the operator's nadir feed was white box tops, orange accent
# bands and one ~11 %-of-frame slit of grass between the inner pair, with no
# ground visible at all. The TFmini-S surrogate at z=-0.05 was worse off — its
# single ray hit a box 0.02 m away, under its own 0.1 m minimum range.
#
# These two tests lock the fix: the belly cargo must sit OUTSIDE the camera
# frustum and off the lidar ray. They are geometry assertions derived from the
# SDFs themselves (mount pose, FOV, image aspect, collision size), so a future
# camera move, FOV change or box resize re-runs the arithmetic instead of
# silently re-blinding the aircraft.
# ---------------------------------------------------------------------------


def _payload_half_extents() -> tuple[float, float, float]:
    """Half (x, y, z) of the cargo box, from the model's own collision box."""
    root = ET.parse(_PAYLOAD_MODEL).getroot()
    collisions = root.findall(".//collision")
    assert len(collisions) == 1
    size = [float(v) for v in (collisions[0].findtext(".//box/size") or "").split()]
    return size[0] / 2.0, size[1] / 2.0, size[2] / 2.0


def _belly_cargo_boxes() -> list[tuple[str, float, float, float, float, float]]:
    """Each belly box as (name, x, y, z_baselink, half_x, half_y) — an
    axis-aligned footprint in the base_link frame, yaw folded in.

    z is converted out of the world frame the world file places them in (they
    are NOT children of the aircraft until the DetachableJoint binds) into the
    base_link frame the sensors are mounted in. A yawed box is reduced to its
    enclosing AABB (hx|cos|+hy|sin|), which is conservative for any yaw.
    """
    root = ET.parse(_WORLD).getroot()
    world = root.find("world")
    assert world is not None
    hx, hy = _payload_half_extents()[:2]
    boxes = []
    for inc in world.iter("include"):
        if (inc.findtext("uri") or "").strip() != "model://cargo_payload":
            continue
        pose = [float(v) for v in (inc.findtext("pose") or "").split()]
        yaw = pose[5] if len(pose) >= 6 else 0.0
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        boxes.append((
            (inc.findtext("name") or "").strip(),
            pose[0], pose[1], pose[2] - _AIRCRAFT_SPAWN_WORLD_Z,
            hx * c + hy * s, hx * s + hy * c,
        ))
    assert len(boxes) == 4
    return boxes


def _nadir_camera() -> tuple[float, float, float]:
    """(mount z in base_link, tan of the half-FOV along body X, along body Y).

    Valid only for this exact mount, which the parse asserts: on the model
    (== base_link) centreline, pitched +pi/2 about Y so the camera's optical
    axis (+X of the sensor frame) rotates onto body -Z, i.e. straight down.
    R_y(pi/2) then carries camera +Y (image LEFT) onto body +Y and camera +Z
    (image UP) onto body +X — so the image's WIDE axis (horizontal_fov) spans
    body Y and its NARROW axis (the aspect-scaled vertical FOV) spans body X.
    Get that mapping backwards and the cheap escape axis looks like the
    expensive one: at this 16:9 aspect the cone is 1.78x wider across Y.
    """
    root = ET.parse(_AIRCRAFT_MODEL).getroot()
    model = root.find("model")
    assert model is not None
    link = next(ln for ln in model.findall("link") if ln.get("name") == "camera_link")
    x, y, z, roll, pitch, yaw = [float(v) for v in (link.findtext("pose") or "").split()]
    assert (x, y, roll, yaw) == (0.0, 0.0, 0.0, 0.0), "nadir camera moved off-centre"
    assert abs(pitch - math.pi / 2) < 1e-3, "nadir camera no longer looks straight down"
    cam = link.find(".//sensor/camera")
    assert cam is not None
    tan_half_y = math.tan(float(cam.findtext("horizontal_fov") or "0") / 2.0)
    aspect = float(cam.findtext("image/height") or "0") / float(
        cam.findtext("image/width") or "1")
    return z, tan_half_y * aspect, tan_half_y


def test_belly_cargo_clears_the_nadir_camera_frustum() -> None:
    """No belly box may intrude on the nadir camera's view cone.

    The frustum's cross-section grows with depth, so the whole box is clear
    iff its footprint is disjoint from the cross-section at the box's DEEPEST
    point (every shallower slice is a subset of that one). Disjoint on either
    axis is enough — the frustum is a rectangular pyramid, not a circle.
    """
    cam_z, tan_x, tan_y = _nadir_camera()
    half_h = _payload_half_extents()[2]
    for name, x, y, z, half_x, half_y in _belly_cargo_boxes():
        depth = cam_z - (z - half_h)          # lens -> box bottom, along -Z
        assert depth > 0, f"{name} is above the camera"
        clearance = max(abs(x) - half_x - depth * tan_x,   # fore/aft (narrow)
                        abs(y) - half_y - depth * tan_y)   # lateral (wide)
        assert clearance >= _SENSOR_CLEARANCE_MARGIN_M, (
            f"{name} is inside the nadir camera's frustum by "
            f"{-clearance:.3f} m at its deepest corner ({depth:.3f} m under "
            f"the lens, where the cone is +-{depth * tan_x:.3f} m fore/aft "
            f"and +-{depth * tan_y:.3f} m laterally) — belly cargo on the "
            "camera centreline blinds the pad detector completely")


def test_belly_cargo_clears_the_downward_lidar_ray() -> None:
    """The TFmini-S surrogate is a SINGLE ray straight down the base_link
    centreline feeding EKF2 height aiding (EKF2_RNG_CTRL=1). A belly box in
    front of it returns a fraction of the sensor's own minimum range, which
    is both wrong and unrejectable-by-inspection — and unlike the camera it
    is invisible on the dashboard, so it gets its own guard rather than
    riding on the (currently wider) camera cone above.
    """
    root = ET.parse(_BASE_MODEL).getroot()
    model = root.find("model")
    assert model is not None
    link = next(ln for ln in model.findall("link")
                if ln.get("name") == "lidar_sensor_link")
    lx, ly, lz = [float(v) for v in (link.findtext("pose") or "").split()][:3]
    assert (lx, ly) == (0.0, 0.0), "lidar moved off the centreline"
    half_h = _payload_half_extents()[2]
    for name, x, y, z, half_x, half_y in _belly_cargo_boxes():
        if z + half_h > lz:                   # box top is above the emitter
            continue
        clearance = max(abs(x) - half_x, abs(y) - half_y)
        assert clearance >= _SENSOR_CLEARANCE_MARGIN_M, (
            f"{name} straddles the downward lidar ray (clearance "
            f"{clearance:.3f} m) — it would read "
            f"{lz - (z + half_h):.3f} m of cardboard instead of AGL")


def test_cargo_box_still_static_after_payload_addition() -> None:
    """Guard: cargo_box (static L&R dressing) must stay static — Task 10's
    DYNAMIC cargo_payload (tested above) is a deliberately separate model.
    Don't let a future edit "fix" cargo_box into a dynamic model by mistake.
    """
    root = ET.parse(_MODEL).getroot()
    model = root.find("model")
    assert model is not None
    assert (model.findtext("static") or "").strip() == "true"
