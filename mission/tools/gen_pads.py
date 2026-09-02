"""Generate the SITL landing-pad models (AAVC 2026 V1.3, rules Figure 6).

Renders the official pad face for every valid marker id (ArUco DICT_4X4_50,
ids 1-6) with vision.detectors.aruco.render_pad_bgr — the SAME renderer the
detector tests and the HITL synthetic camera use, so the sim texture can never
drift from what the vision stack is validated on — and writes one Gazebo model
per id under sitl/models/:

    landing_pad_id_<k>/
        model.config
        model.sdf                      (static 1x1 m textured thin box)
        materials/textures/pad_<k>.png

Every texture is decode self-checked before it is written: a pad the detector
cannot decode must never reach the sim. Idempotent — re-run after changing the
pad geometry constants (e.g. re-measured at the event briefing).

Usage:  .venv/bin/python tools/gen_pads.py [--out sitl/models] [--size 1024]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from vision.detectors.aruco import (  # noqa: E402
    PAD_SIZE_M,
    VALID_MARKER_IDS,
    find_landing_pads,
    render_pad_bgr,
)

_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="landing_pad_id_{mid}">
    <static>true</static>
    <link name="pad">
      <pose>0 0 0.005 0 0 0</pose>
      <visual name="pad_visual">
        <geometry>
          <box>
            <size>{size} {size} 0.01</size>
          </box>
        </geometry>
        <material>
          <pbr>
            <metal>
              <albedo_map>materials/textures/pad_{mid}.png</albedo_map>
            </metal>
          </pbr>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
      <collision name="pad_collision">
        <geometry>
          <box>
            <size>{size} {size} 0.01</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

_MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>landing_pad_id_{mid}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>AAVC</name></author>
  <description>AAVC 2026 landing pad (rules V1.3 Fig. 6): 1x1 m white pad,
black ring d0.75 m, central 0.4 m ArUco DICT_4X4_50 marker id {mid}.</description>
</model>
"""


def generate(out_dir: Path, size_px: int) -> int:
    written = 0
    for mid in sorted(VALID_MARKER_IDS):
        face = render_pad_bgr(mid, size_px)

        # Decode self-check: the generated texture MUST decode as its own id.
        hits = find_landing_pads(face)
        got = [h.marker_id for h in hits if h.marker_id is not None]
        if got != [mid]:
            raise RuntimeError(
                f"pad texture id {mid} failed the decode self-check (got {got}) — "
                f"pad geometry constants produce an undecodable pad")

        model = out_dir / f"landing_pad_id_{mid}"
        tex = model / "materials" / "textures"
        tex.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(tex / f"pad_{mid}.png"), face):
            raise RuntimeError(f"could not write texture for id {mid}")
        (model / "model.sdf").write_text(
            _MODEL_SDF.format(mid=mid, size=PAD_SIZE_M), encoding="utf-8")
        (model / "model.config").write_text(
            _MODEL_CONFIG.format(mid=mid), encoding="utf-8")
        written += 1
        print(f"landing_pad_id_{mid}: texture {size_px}px, decode self-check OK")
    return written


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Generate the SITL landing-pad models (AAVC 2026 V1.3)")
    ap.add_argument("--out", default=str(REPO / "sitl" / "models"),
                    help="models output dir (default: sitl/models)")
    ap.add_argument("--size", type=int, default=1024,
                    help="texture size in px (default 1024)")
    args = ap.parse_args()
    n = generate(Path(args.out), args.size)
    print(f"{n} pad models written to {args.out}")


if __name__ == "__main__":
    main()
