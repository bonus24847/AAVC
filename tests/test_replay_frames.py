"""tools/replay_frames.py — offline detector replay over recorded frames.

Fixtures are synthetic frames from render_pad_bgr (the SAME renderer behind
the SITL textures and the detector's own tests) written as JPEGs into
tmp_path — no committed image files (runs/ is gitignored, and the renderer IS
the contract).
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from tools.replay_frames import collect_paths, replay
from vision.detectors.aruco import render_pad_bgr

GRASS = (60, 120, 70)


def _frame(marker_id: int | None, pad_px: int = 90) -> np.ndarray:
    img = np.full((720, 1280, 3), GRASS, np.uint8)
    if marker_id is not None:
        pad = cv2.resize(render_pad_bgr(marker_id, 512), (pad_px, pad_px),
                         interpolation=cv2.INTER_AREA)
        img[360 - pad_px // 2:360 - pad_px // 2 + pad_px,
            640 - pad_px // 2:640 - pad_px // 2 + pad_px] = pad
    return img


def _write_frames(tmp_path: Path) -> list[Path]:
    # sequence order matters: first/last decode must track the file names
    specs = [("nadir_000000.jpg", None),      # blank grass
             ("nadir_000001.jpg", 3),
             ("nadir_000002.jpg", 1),
             ("nadir_000003.jpg", 3),
             ("nadir_000004.jpg", None)]
    for name, marker in specs:
        cv2.imwrite(str(tmp_path / name), _frame(marker),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
    return sorted(tmp_path.iterdir())


def test_replay_counts_ids_and_tracks_first_last(tmp_path: Path) -> None:
    paths = _write_frames(tmp_path)
    summary = replay(paths)
    assert summary.n_frames == 5
    assert summary.n_decoded_frames == 3
    assert summary.id_counts[3] == 2 and summary.id_counts[1] == 1
    assert summary.first_decode == "nadir_000001.jpg"
    assert summary.last_decode == "nadir_000003.jpg"


def test_replay_valid_ids_filter_drops_unassigned(tmp_path: Path) -> None:
    _write_frames(tmp_path)
    summary = replay(collect_paths(tmp_path), valid_ids=frozenset({1}))
    # id 3 frames no longer decode as marker hits
    assert summary.id_counts.get(3) is None
    assert summary.id_counts[1] == 1


def test_collect_paths_sorts_sequence_order(tmp_path: Path) -> None:
    paths = _write_frames(tmp_path)
    assert [p.name for p in collect_paths(tmp_path)] == [p.name for p in paths]
    # a single file is accepted directly
    assert collect_paths(paths[1]) == [paths[1]]


def test_annotate_writes_copies_not_in_place(tmp_path: Path) -> None:
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    out_dir = tmp_path / "annot"
    _write_frames(frames_dir)
    before = {p.name: p.stat().st_size for p in frames_dir.iterdir()}
    replay(collect_paths(frames_dir), annotate_dir=out_dir)
    after = {p.name: p.stat().st_size for p in frames_dir.iterdir()}
    assert before == after, "source frames are evidence — must never change"
    assert any(out_dir.iterdir()), "annotated copies expected"
