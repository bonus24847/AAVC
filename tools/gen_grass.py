#!/usr/bin/env python3
"""Generate the procedural grass/earth ground texture for the AAVC SITL field.

Realistic green-grass variation (grass shades + sparse olive dirt patches +
dead-grass tufts) verified against the LANDING-PAD detector's white-cue colour
gate (low-S / high-V) so the textured ground can never false-trigger
``find_landing_pads``'s pad-blob pass.

Output: sitl/models/aavc_ground/materials/textures/grass.png (referenced by the
ground_plane in sitl/worlds/aavc_field.sdf). Re-run after changing the palette.

    python tools/gen_grass.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from vision.detectors.aruco import _PAD_S_MAX, _PAD_V_MIN  # noqa: E402

N = 1024
rng = np.random.default_rng(7)               # fixed seed → reproducible texture


def octave(n: int, cells: int) -> np.ndarray:
    """Smooth value noise at ``cells`` resolution, upsampled to n×n, 0..1."""
    base = rng.random((cells, cells))
    return cv2.resize(base, (n, n), interpolation=cv2.INTER_CUBIC).clip(0, 1)


def main() -> int:
    grass = (0.5 * octave(N, 8) + 0.3 * octave(N, 24) + 0.2 * octave(N, 96)).clip(0, 1)
    blades = octave(N, 512)                  # fine high-freq blade speckle

    # Palette (BGR). All green/olive — G >= R so the hue stays off the red bands.
    g_dark = np.array([40, 78, 45], float)
    g_light = np.array([78, 158, 100], float)
    dead = np.array([90, 150, 130], float)   # yellow-green dead grass (safe)
    soil = np.array([95, 118, 120], float)   # desaturated olive-tan (NOT red clay)

    t = grass[..., None]
    img = g_dark * (1 - t) + g_light * t     # green gradient by noise
    img = img * (0.85 + 0.3 * blades[..., None])
    img = np.where((octave(N, 40) > 0.72)[..., None], 0.6 * img + 0.4 * dead, img)
    img = np.where((octave(N, 6) < 0.34)[..., None], 0.5 * img + 0.5 * soil, img)
    img = img.clip(0, 255).astype(np.uint8)

    # The pad detector's white-cue gate must never fire on bare ground: no
    # pixel may read as "pad white" (low saturation AND high value).
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    white = (hsv[:, :, 1] <= _PAD_S_MAX) & (hsv[:, :, 2] >= _PAD_V_MIN)
    frac = int(white.sum()) / white.size
    safe = frac < 0.0005
    print(f"pad-white coverage = {frac * 100:.4f}%  ({'SAFE' if safe else 'TOO WHITE'})")
    if not safe:
        print("Palette produces pad-white pixels — retune before using.", file=sys.stderr)
        return 1

    out = REPO / "sitl/models/aavc_ground/materials/textures"
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "grass.png"), img)
    print(f"wrote {out / 'grass.png'} {img.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
