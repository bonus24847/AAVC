#!/usr/bin/env python3
"""Which cargo box actually left the aircraft, and where did it land?

    .venv/bin/python tools/box_truth.py [world]     # default: kmutnb_skyfield

Why this exists: after a release, three sources all CLAIM which egg fell — the
mission audit (``DELIVERY k RELEASE payload=N``), the detach bridge's log
(``shed box N``) and the SDF comments. All three can agree and all three can be
wrong together, because they share ONE assumption: payload_id -> actuator set
-> ``servo_<n-1>`` -> that corner's box. This reads the only witness that
shares nothing with them — the simulator's own pose stream.

A still-attached box sits exactly at its mount pose relative to the parent
model (the ``<pose>`` of its ``<include>`` in the aircraft SDF). The moment its
DetachableJoint breaks, that pose drifts, and z runs away as the aircraft
climbs out. So "far from its mount" == "this is the one that was released".

Reading it this way also proves the NEGATIVE: every other box must still be
pinned at its mount, which is what catches a release that opens more than one
latch (a DO_SET_ACTUATOR NaN-masking regression) — an error no log would show.

⚠ FRAME TRAP (cost a day, 2026-08-16 — see docs/SERVO_AUX_MAPPING.md §4.3).
``/world/<w>/pose/info`` publishes a NESTED model's pose **relative to its
parent**, and the cargo boxes stay nested under the aircraft even after their
joint breaks. Reading the released box's numbers as world coordinates put it
7.35 m from the release point and made the landing evidence look broken; the
same numbers composed through the aircraft's own world pose put it 0.04 m from
the pad. The composition below is therefore not a nicety — without it this
tool reports confident nonsense. It only bites the RELEASED box: an attached
one is compared against a mount pose that lives in the same parent frame.

Mount geometry is read from the aircraft SDF and the loom map from
sitl/aavc_config.yaml, so a rewire or a rack change follows automatically
instead of turning this into silently false evidence.

NOTE the box index is NOT the payload_id in this repo: ``cargo_payload_N``
hangs on actuator set N+1 (== AUX pin N+1, SIM_GZ_SV_FUNC(N+1) -> servo_N),
while the mission releases in the order given by
``connection.drop_servo_channels``. Both are printed. See
docs/SERVO_AUX_MAPPING.md.

Credit: shape and the two hard-won details (the `gz topic -e` hang when nothing
publishes; summarising by CORNER, the one name both repos share) come from the
peer session's sim/box_truth.py in the mission_AAVC project.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
AIRCRAFT_SDF = REPO / "sitl" / "models" / "eft_x6100" / "model.sdf"
CARGO_SDF = REPO / "sitl" / "models" / "cargo_payload" / "model.sdf"
CONFIG = REPO / "sitl" / "aavc_config.yaml"
TRUTH = Path("/tmp/aavc_targets.json")
TOL_M = 0.05
# The pad's black ring is 750 mm across, so an egg is "on the marker" within
# 0.375 m of pad centre (operator's scoring criterion, 2026-08-15). The box
# fits ENTIRELY inside the ring within 0.375 - half its footprint diagonal.
RING_RADIUS_M = 0.375
CORNERS = {(True, True): "front-left", (True, False): "front-right",
           (False, True): "rear-left", (False, False): "rear-right"}
_WGS84_A = 6378137.0
_WGS84_E2 = (1 / 298.257223563) * (2 - 1 / 298.257223563)

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]      # x, y, z, w


def corner(x: float, y: float) -> str:
    """Body frame: +x nose, +y left."""
    return CORNERS[(x > 0, y > 0)]


def compose(parent_pos: Vec3, parent_quat: Quat, child: Vec3) -> Vec3:
    """Child pose expressed in the PARENT frame -> world frame.

    ``world = parent_pos + R(parent_quat) . child``. Written out rather than
    pulled from a rotation library so the tool keeps its "no shared assumption
    with the thing it audits" property.
    """
    qx, qy, qz, qw = parent_quat
    cx, cy, cz = child
    # q * (0, c) * q^-1, expanded (Rodrigues form: v + 2q_v x (q_v x v + w v))
    tx = 2.0 * (qy * cz - qz * cy)
    ty = 2.0 * (qz * cx - qx * cz)
    tz = 2.0 * (qx * cy - qy * cx)
    rx = cx + qw * tx + (qy * tz - qz * ty)
    ry = cy + qw * ty + (qz * tx - qx * tz)
    rz = cz + qw * tz + (qx * ty - qy * tx)
    return (parent_pos[0] + rx, parent_pos[1] + ry, parent_pos[2] + rz)


def enu_to_latlon(east: float, north: float,
                  lat0: float, lon0: float) -> tuple[float, float]:
    """Local ENU (m) -> (lat, lon) on WGS84, matching gz's EARTH_WGS84 frame
    and sitl/spawn_targets.py (whose equatorial-radius bug this mirrors the
    FIX of, not the bug — see CLAUDE.md 'Truth-coordinate fix')."""
    s = math.sin(math.radians(lat0))
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    m_lat = math.radians(1.0) * _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
    m_lon = math.radians(1.0) * _WGS84_A / w * math.cos(math.radians(lat0))
    return lat0 + north / m_lat, lon0 + east / m_lon


def box_half_diagonal() -> float:
    """Half the cargo box's FOOTPRINT diagonal, from the model's own
    collision box — the margin the ring must give up for the whole box (not
    just its centre) to sit inside. Read from the SDF rather than typed here
    so a box respec cannot leave this tool scoring against a stale size."""
    root = ET.parse(CARGO_SDF).getroot()
    for coll in root.iter("collision"):
        size = coll.find("./geometry/box/size")
        if size is not None and size.text:
            w, d, _h = (float(v) for v in size.text.split())
            return 0.5 * math.hypot(w, d)
    raise AssertionError(f"{CARGO_SDF} has no <collision> box size")


def mounts() -> dict[int, Vec3]:
    """``{box index: (x, y, z)}`` from the aircraft SDF's cargo includes."""
    model = ET.parse(AIRCRAFT_SDF).getroot().find("model")
    assert model is not None, f"{AIRCRAFT_SDF} has no <model>"
    out: dict[int, Vec3] = {}
    for inc in model.iter("include"):
        if (inc.findtext("uri") or "").strip() != "model://cargo_payload":
            continue
        name = (inc.findtext("name") or "").strip()
        pose = [float(v) for v in (inc.findtext("pose") or "").split()]
        out[int(name.rsplit("_", 1)[1])] = (pose[0], pose[1], pose[2])
    return out


def _pose_blocks(world: str) -> list[tuple[str, Vec3, Quat]]:
    """``[(name, position, quaternion)]`` from one ``pose/info`` sample."""
    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1",
                              "-t", f"/world/{world}/pose/info"],
                             capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        # `gz topic -e` blocks forever when nothing publishes — that IS the
        # answer (no sim on this world), so say so instead of a traceback.
        return []
    except FileNotFoundError:
        print("gz CLI not on PATH — source the Harmonic setup first",
              file=sys.stderr)
        return []
    blocks: list[tuple[str, Vec3, Quat]] = []
    for block in re.findall(r"pose\s*\{(.*?)\n\}", out, re.S):
        name_m = re.search(r'name:\s*"([^"]+)"', block)
        pos_m = re.search(r"position\s*\{(.*?)\}", block, re.S)
        if not name_m or not pos_m:
            continue
        p = {k: float(v) for k, v in
             re.findall(r"([xyz]):\s*(-?[\d.eE+-]+)", pos_m.group(1))}
        ori_m = re.search(r"orientation\s*\{(.*?)\}", block, re.S)
        q = {k: float(v) for k, v in
             re.findall(r"([xyzw]):\s*(-?[\d.eE+-]+)", ori_m.group(1))} \
            if ori_m else {}
        blocks.append((
            name_m.group(1),
            (p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)),
            (q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0),
             q.get("w", 1.0)),
        ))
    return blocks


def boxes_and_parent(
    blocks: list[tuple[str, Vec3, Quat]],
) -> tuple[dict[int, Vec3], tuple[Vec3, Quat] | None]:
    """Split one pose sample into ``{box index: parent-relative xyz}`` and the
    aircraft's own WORLD pose.

    PX4 spawns the airframe as ``eft_x6100_0`` (instance suffix), so the
    aircraft is matched by prefix; link/visual entries carry a ``:`` in their
    scoped name and are skipped in favour of the model-level pose.
    """
    found: dict[int, tuple[str, Vec3]] = {}
    parent: tuple[Vec3, Quat] | None = None
    parent_name = ""
    for name, pos, quat in blocks:
        if ":" in name:
            continue
        if "cargo_payload_" in name:
            idx = int(name.rsplit("cargo_payload_", 1)[1])
            # the model-level pose, not its child links (shortest name wins)
            if idx not in found or len(name) < len(found[idx][0]):
                found[idx] = (name, pos)
        elif name.startswith("eft_x6100"):
            if not parent_name or len(name) < len(parent_name):
                parent, parent_name = (pos, quat), name
    return {i: v for i, (_, v) in found.items()}, parent


def _truth_pads() -> list[tuple[str, int, float, float]]:
    """``[(name, marker_id, lat, lon)]`` from the SITL ground truth, if any."""
    try:
        data = json.loads(TRUTH.read_text())
    except (OSError, ValueError):
        return []
    return [(t.get("name", "?"), int(t.get("marker_id", -1)),
             float(t["lat"]), float(t["lon"]))
            for t in data.get("targets", []) if "lat" in t and "lon" in t]


def _nearest_pad(lat: float, lon: float,
                 lat0: float) -> tuple[str, int, float] | None:
    """Closest truth pad to (lat, lon) as ``(name, marker_id, distance_m)``."""
    pads = _truth_pads()
    if not pads:
        return None
    s = math.sin(math.radians(lat0))
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    m_lat = math.radians(1.0) * _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
    m_lon = math.radians(1.0) * _WGS84_A / w * math.cos(math.radians(lat0))
    best = min(pads, key=lambda p: math.hypot((p[2] - lat) * m_lat,
                                              (p[3] - lon) * m_lon))
    return best[0], best[1], math.hypot((best[2] - lat) * m_lat,
                                        (best[3] - lon) * m_lon)


def main() -> int:
    world = sys.argv[1] if len(sys.argv) > 1 else "kmutnb_skyfield"
    mount = mounts()
    cfg = yaml.safe_load(CONFIG.read_text())
    chans = cfg["connection"].get("drop_servo_channels") or []
    lat0 = cfg["site"]["center_lat"]
    lon0 = cfg["site"]["center_lon"]
    # box index -> which egg of the flight opens it (inverse of the loom map)
    payload_of_box = {ch - 1: pid for pid, ch in enumerate(chans)}

    seen, parent = boxes_and_parent(_pose_blocks(world))
    if not seen:
        print(f"no cargo_payload poses on /world/{world}/pose/info — is SITL "
              "up on this world?", file=sys.stderr)
        return 1
    if parent is None:
        print("aircraft pose not in the dump — cannot place a released box in "
              "the world frame (see the FRAME TRAP note in this file)",
              file=sys.stderr)

    print(f"{'box':<4} {'corner':<12} {'AUX':<4} {'egg':<4} "
          f"{'x':>8} {'y':>8} {'z':>8}  state")
    released = []
    for i in sorted(mount):
        mx, my, mz = mount[i]
        egg = payload_of_box.get(i)
        egg_s = "—" if egg is None else str(egg)
        if i not in seen:
            print(f"{i:<4} {corner(mx, my):<12} {i + 1:<4} {egg_s:<4} "
                  f"{'—':>8} {'—':>8} {'—':>8}  NOT IN POSE DUMP")
            continue
        x, y, z = seen[i]
        attached = (abs(x - mx) < TOL_M and abs(y - my) < TOL_M
                    and abs(z - mz) < TOL_M)
        if not attached:
            released.append(i)
        print(f"{i:<4} {corner(mx, my):<12} {i + 1:<4} {egg_s:<4} "
              f"{x:8.3f} {y:8.3f} {z:8.3f}  "
              f"{'attached' if attached else '*** RELEASED ***'}")

    print(f"\nreleased: {released or 'none'}"
          + (f"  → corners {[corner(*mount[i][:2]) for i in released]}"
             f"  (eggs {[payload_of_box.get(i) for i in released]})"
             if released else ""))

    # Where a released box actually IS. The printed x/y/z above are relative
    # to the aircraft (see FRAME TRAP); only these lines are world truth.
    if released and parent is not None:
        p_pos, p_quat = parent
        print(f"\nwhere it landed (world frame; aircraft at "
              f"{p_pos[0]:.2f}, {p_pos[1]:.2f}):")
        for i in released:
            wx, wy, wz = compose(p_pos, p_quat, seen[i])
            lat, lon = enu_to_latlon(wx, wy, lat0, lon0)
            near = _nearest_pad(lat, lon, lat0)
            line = (f"  box {i}: ENU=({wx:8.3f},{wy:8.3f},{wz:6.3f})  "
                    f"{lat:.7f},{lon:.7f}")
            if near:
                name, mid, d = near
                if d <= RING_RADIUS_M - box_half_diagonal():
                    verdict = "INSIDE the ring, whole box"
                elif d <= RING_RADIUS_M:
                    verdict = "centre inside the ⌀750 ring"
                else:
                    verdict = "OUTSIDE the ⌀750 ring"
                line += f"  → {d:.3f} m from {name} (id {mid}) — {verdict}"
            print(line)

    return 0


if __name__ == "__main__":
    sys.exit(main())
