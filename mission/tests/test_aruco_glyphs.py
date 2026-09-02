"""The GCS ArUco glyphs must BE the markers the detector decodes.

The operator compares the *picture* on the committee's card with the *picture*
on the queue chip — so a glyph that drifts from ``DICT_4X4_50`` would silently
send the aircraft to the wrong pad. These tests close that loop the same way
``tools/gen_pads.py`` self-checks the pad textures: rasterise the glyph the
browser actually renders and decode it with the REAL flight detector.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision.detectors.aruco import VALID_MARKER_IDS, _aruco_detector

GLYPH_TS = (Path(__file__).resolve().parents[1]
            / "dashboard/web/src/lib/aruco-glyphs.ts")


def _parse_glyph_rows() -> dict[int, list[str]]:
    """Parse the generated TS module the browser imports.

    Deliberately parses the checked-in artefact (not the Python generator) —
    that is the thing shipped to the operator's screen.
    """
    src = GLYPH_TS.read_text(encoding="utf-8")
    body = re.search(
        r"ARUCO_GLYPH_ROWS[^=]*=\s*(\{.*?\n\});", src, re.S)
    assert body, f"no ARUCO_GLYPH_ROWS object literal in {GLYPH_TS}"
    # The literal is JSON once the numeric keys are quoted and the trailing
    # commas dropped — keep the parse strict so a hand-edit cannot sneak past.
    obj = re.sub(r"(\d+):", r'"\1":', body.group(1))
    obj = re.sub(r",(\s*[}\]])", r"\1", obj)
    return {int(k): v for k, v in json.loads(obj).items()}


def _rasterise(rows: list[str], *, cell_px: int = 12, quiet_cells: int = 2) -> np.ndarray:
    """Blow the glyph's bit rows up into a decodable grayscale image."""
    bits = np.array([[255 if c == "1" else 0 for c in row] for row in rows],
                    dtype=np.uint8)
    marker = np.kron(bits, np.ones((cell_px, cell_px), np.uint8))
    pad = quiet_cells * cell_px
    return cv2.copyMakeBorder(marker, pad, pad, pad, pad,
                              cv2.BORDER_CONSTANT, value=255)


def test_glyph_rows_cover_every_valid_marker_id() -> None:
    assert set(_parse_glyph_rows()) == set(VALID_MARKER_IDS)


@pytest.mark.parametrize("marker_id", sorted(VALID_MARKER_IDS))
def test_glyph_decodes_to_its_own_id(marker_id: int) -> None:
    """The glyph on the chip decodes to the id it is labelled with."""
    rows = _parse_glyph_rows()[marker_id]
    assert len(rows) == 6 and all(len(r) == 6 for r in rows), (
        "a 4x4 marker glyph is 6x6 cells including its black border")

    corners, ids, _ = _aruco_detector().detectMarkers(_rasterise(rows))

    assert ids is not None, f"glyph for id {marker_id} did not decode at all"
    assert [int(i) for i in ids.flatten()] == [marker_id]


def test_glyphs_are_distinguishable_from_each_other() -> None:
    """Six chips the operator must tell apart at a glance are six patterns."""
    rows = _parse_glyph_rows()
    patterns = {mid: "".join(r) for mid, r in rows.items()}
    assert len(set(patterns.values())) == len(patterns)


def test_checked_in_module_matches_the_generator() -> None:
    """Regenerating must be a no-op — the artefact cannot go stale in git."""
    from tools.gen_aruco_glyphs import render_module

    assert GLYPH_TS.read_text(encoding="utf-8") == render_module(), (
        "dashboard/web/src/lib/aruco-glyphs.ts is stale — "
        "run .venv/bin/python tools/gen_aruco_glyphs.py")
