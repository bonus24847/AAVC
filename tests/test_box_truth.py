"""Pose-frame arithmetic behind tools/box_truth.py.

The tool answers ONE question the whole SITL landing evidence rests on: did
the egg land on the pad? It got that wrong for a day (2026-08-16) not through
bad data but through a frame mistake — gz publishes a nested model's pose
relative to its PARENT, and the cargo boxes stay nested under the aircraft
after their joint breaks. Read as world coordinates, a box that was 0.02 m
from the pad looked 7.35 m away, and the conclusion drawn from it ("the SITL
drop evidence is unreliable") was the opposite of the truth.

So the regression pinned here is not `compose()`'s algebra in the abstract —
it is that this specific real flight's numbers come out on the pad.
"""

from __future__ import annotations

import math

from tools.box_truth import (
    RING_RADIUS_M,
    box_half_diagonal,
    boxes_and_parent,
    compose,
    corner,
    enu_to_latlon,
)

# Recorded off the live simulator at the end of the 2026-08-16 wind flight
# (gz /world/kmutnb_skyfield/pose/info, mission runs/aavc_delivery_mission).
AIRCRAFT_POS = (-0.516, 0.161, 0.048)
AIRCRAFT_QUAT = (0.0, 0.0, -0.119, 0.993)          # ~13.7 deg of yaw
BOX3_PARENT_REL = (-17.013, 28.174, 0.052)
PAD_LAT, PAD_LON = 13.822779465026745, 100.51218122797816   # truth, marker 1
# ⚠ HISTORICAL, NOT LIVE GEOMETRY — do not "update" this when the field moves.
# It is the ENU origin THAT flight flew from, and every offset above is measured
# against it. The 2026-08-17 ground survey moved the live origin to the L&R on
# the running track; pointing these recorded poses at the new origin puts the box
# 78 m from its pad, which is a frame mistake of precisely the kind this file
# exists to catch. Live geometry belongs in sitl/aavc_config.yaml.
SITE_LAT, SITE_LON = 13.822494, 100.5122771


def _m_per_deg(lat0: float) -> tuple[float, float]:
    """(m per degree lat, m per degree lon) — inverted out of the tool's own
    converter so the test never grows a second copy of the ellipsoid."""
    one_north, _ = enu_to_latlon(0.0, 1.0, lat0, 0.0)
    _, one_east = enu_to_latlon(1.0, 0.0, lat0, 0.0)
    return 1.0 / (one_north - lat0), 1.0 / one_east


def test_compose_identity_is_translation() -> None:
    assert compose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), (0.5, -0.5, 0.25)) \
        == (1.5, 1.5, 3.25)


def test_compose_rotates_by_parent_yaw() -> None:
    """A quarter turn about +z sends body +x to world +y."""
    half = math.sqrt(0.5)
    x, y, z = compose((0.0, 0.0, 0.0), (0.0, 0.0, half, half), (1.0, 0.0, 0.0))
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(z) < 1e-9


def test_released_box_lands_on_the_pad() -> None:
    """THE regression: the real flight's box is on the pad, not 7 m away."""
    east, north, _ = compose(AIRCRAFT_POS, AIRCRAFT_QUAT, BOX3_PARENT_REL)
    lat, lon = enu_to_latlon(east, north, SITE_LAT, SITE_LON)
    m_lat, m_lon = _m_per_deg(SITE_LAT)
    d = math.hypot((lat - PAD_LAT) * m_lat, (lon - PAD_LON) * m_lon)
    assert d < 0.10, f"box {d:.3f} m from pad centre"
    # and it clears the operator's scoring criterion with the box's own size
    assert d <= RING_RADIUS_M - box_half_diagonal()


def test_box_footprint_comes_from_the_model() -> None:
    """0.16 x 0.065 m box -> 0.086 m half-diagonal -> 0.289 m usable radius."""
    assert abs(box_half_diagonal() - 0.5 * math.hypot(0.16, 0.065)) < 1e-9
    assert abs((RING_RADIUS_M - box_half_diagonal()) - 0.2887) < 1e-3


def test_uncomposed_pose_is_the_trap_not_the_answer() -> None:
    """Reading the published numbers as world coordinates must NOT look sane.

    Pinned so nobody 'simplifies' compose() away: the naive reading is metres
    off, which is exactly why it was believable as a real defect.
    """
    m_lat, m_lon = _m_per_deg(SITE_LAT)
    pad_e = (PAD_LON - SITE_LON) * m_lon
    pad_n = (PAD_LAT - SITE_LAT) * m_lat
    naive = math.hypot(BOX3_PARENT_REL[0] - pad_e, BOX3_PARENT_REL[1] - pad_n)
    assert naive > 5.0


def test_boxes_and_parent_picks_the_model_level_poses() -> None:
    blocks = [
        ("eft_x6100_0", AIRCRAFT_POS, AIRCRAFT_QUAT),
        ("eft_x6100_0::base_link", (9.0, 9.0, 9.0), (0.0, 0.0, 0.0, 1.0)),
        ("cargo_payload_3", BOX3_PARENT_REL, (0.0, 0.0, 0.0, 1.0)),
        ("cargo_payload_3::link", (8.0, 8.0, 8.0), (0.0, 0.0, 0.0, 1.0)),
        ("pad_2", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ]
    boxes, parent = boxes_and_parent(blocks)
    assert boxes == {3: BOX3_PARENT_REL}
    assert parent == (AIRCRAFT_POS, AIRCRAFT_QUAT)


def test_parent_absent_is_reported_not_guessed() -> None:
    boxes, parent = boxes_and_parent(
        [("cargo_payload_0", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))])
    assert boxes and parent is None


def test_corner_names_follow_body_frame() -> None:
    """+x nose, +y left — the loom table in docs/SERVO_AUX_MAPPING.md."""
    assert corner(0.1, 0.035) == "front-left"
    assert corner(-0.1, -0.035) == "rear-right"
