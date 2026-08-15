#!/usr/bin/env python3
"""Which cargo box actually left the aircraft? — gz ground truth for a release.

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

import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
AIRCRAFT_SDF = REPO / "sitl" / "models" / "eft_x6100" / "model.sdf"
CONFIG = REPO / "sitl" / "aavc_config.yaml"
TOL_M = 0.05
CORNERS = {(True, True): "front-left", (True, False): "front-right",
           (False, True): "rear-left", (False, False): "rear-right"}


def corner(x: float, y: float) -> str:
    """Body frame: +x nose, +y left."""
    return CORNERS[(x > 0, y > 0)]


def mounts() -> dict[int, tuple[float, float, float]]:
    """``{box index: (x, y, z)}`` from the aircraft SDF's cargo includes."""
    model = ET.parse(AIRCRAFT_SDF).getroot().find("model")
    assert model is not None, f"{AIRCRAFT_SDF} has no <model>"
    out: dict[int, tuple[float, float, float]] = {}
    for inc in model.iter("include"):
        if (inc.findtext("uri") or "").strip() != "model://cargo_payload":
            continue
        name = (inc.findtext("name") or "").strip()
        pose = [float(v) for v in (inc.findtext("pose") or "").split()]
        out[int(name.rsplit("_", 1)[1])] = (pose[0], pose[1], pose[2])
    return out


def poses(world: str) -> dict[int, tuple[float, float, float]]:
    """Latest ``/world/<world>/pose/info`` → ``{box index: (x, y, z)}``."""
    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1",
                              "-t", f"/world/{world}/pose/info"],
                             capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        # `gz topic -e` blocks forever when nothing publishes — that IS the
        # answer (no sim on this world), so say so instead of a traceback.
        return {}
    except FileNotFoundError:
        print("gz CLI not on PATH — source the Harmonic setup first",
              file=sys.stderr)
        return {}
    found: dict[int, tuple[str, tuple[float, float, float]]] = {}
    for block in re.findall(r"pose\s*\{(.*?)\n\}", out, re.S):
        name_m = re.search(r'name:\s*"([^"]+)"', block)
        pos_m = re.search(r"position\s*\{(.*?)\}", block, re.S)
        if not name_m or not pos_m or "cargo_payload_" not in name_m.group(1):
            continue
        name = name_m.group(1)
        idx = int(name.rsplit("cargo_payload_", 1)[1].split(":")[0])
        xyz = {k: float(v) for k, v in
               re.findall(r"([xyz]):\s*(-?[\d.e+-]+)", pos_m.group(1))}
        vec = (xyz.get("x", 0.0), xyz.get("y", 0.0), xyz.get("z", 0.0))
        # the model-level pose, not its child links (shortest name wins)
        if idx not in found or len(name) < len(found[idx][0]):
            found[idx] = (name, vec)
    return {i: v for i, (_, v) in found.items()}


def main() -> int:
    world = sys.argv[1] if len(sys.argv) > 1 else "kmutnb_skyfield"
    mount = mounts()
    chans = (yaml.safe_load(CONFIG.read_text())["connection"]
             .get("drop_servo_channels") or [])
    # box index -> which egg of the flight opens it (inverse of the loom map)
    payload_of_box = {ch - 1: pid for pid, ch in enumerate(chans)}

    seen = poses(world)
    if not seen:
        print(f"no cargo_payload poses on /world/{world}/pose/info — is SITL "
              "up on this world?", file=sys.stderr)
        return 1

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
    return 0


if __name__ == "__main__":
    sys.exit(main())
