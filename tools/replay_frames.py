"""Offline ArUco replay: run the REAL pad detector over recorded camera frames.

Answers the operator's question "does the ArUco scan actually work?" from
evidence instead of trust: point it at a frame_recorder directory
(``runs/<mission_id>/frames/`` — ``nadir_{seq:06d}.jpg`` at ~1 Hz) or a single
image, and it runs ``vision.detectors.aruco.find_landing_pads`` — the exact
function the mission flies with, pure pixels, no camera model — over every
frame, then reports decode rate, per-id counts, and the first/last decoding
frame.

Context that keeps the report honest (2026-08-20 survey):

* the laptop's only archive (670 frames) is Gazebo SITL, not the real camera
  — its baseline is 127/670 decoded, ids 1-6 all present;
* the real-camera frames live only on the CM4 (pull them when it is
  reachable: ``rsync drone@10.42.0.1:~/mission/runs/ …``);
* NO printed pad has ever been laid out at the KMUTNB field, so real frames
  recorded so far contain no marker to decode — a zero on them is a fact
  about the field, not the detector. The real proof needs the printed-pad
  bench test, then a daytime flight over one (folded into G7; see
  .claude/skills/PX4MASTER/references/ops-field.md).

Usage:
    .venv/bin/python tools/replay_frames.py runs/<mission_id>/frames
    .venv/bin/python tools/replay_frames.py frame.jpg --valid-ids 1,3,4,6
    .venv/bin/python tools/replay_frames.py <dir> --annotate /tmp/annot

Exit codes: 0 = at least one frame decoded · 1 = zero decodes (fail-visible,
see the field-context note above) · 2 = usage/input error. ``--annotate``
writes marked-up COPIES into the given directory — the source frames are
flight evidence and are never touched.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import cv2  # noqa: E402

from vision.detectors.aruco import VALID_MARKER_IDS, find_landing_pads  # noqa: E402

_EXTENSIONS = (".jpg", ".jpeg", ".png")


@dataclass
class FrameResult:
    path: Path
    decoded_ids: list[int]
    cue_only: int  # undecoded white-pad candidates


@dataclass
class ReplaySummary:
    n_frames: int = 0
    n_unreadable: int = 0
    n_decoded_frames: int = 0
    id_counts: Counter = field(default_factory=Counter)
    first_decode: str | None = None
    last_decode: str | None = None
    frames: list[FrameResult] = field(default_factory=list)


def collect_paths(target: Path) -> list[Path]:
    if target.is_file():
        return [target]
    if target.is_dir():
        # frame_recorder names are zero-padded (nadir_000123.jpg), so sorted
        # lexical order IS sequence order.
        return sorted(p for p in target.iterdir()
                      if p.suffix.lower() in _EXTENSIONS)
    return []


def replay(paths: list[Path], *, valid_ids: frozenset[int] = VALID_MARKER_IDS,
           annotate_dir: Path | None = None) -> ReplaySummary:
    summary = ReplaySummary()
    for path in paths:
        img = cv2.imread(str(path))
        if img is None:
            summary.n_unreadable += 1
            continue
        summary.n_frames += 1
        hits = find_landing_pads(img, valid_ids=valid_ids)
        decoded = [h.marker_id for h in hits if h.marker_id is not None]
        cue_only = sum(1 for h in hits if h.marker_id is None)
        summary.frames.append(FrameResult(path, decoded, cue_only))
        if decoded:
            summary.n_decoded_frames += 1
            summary.id_counts.update(decoded)
            if summary.first_decode is None:
                summary.first_decode = path.name
            summary.last_decode = path.name
        if annotate_dir is not None and hits:
            out = img.copy()
            for h in hits:
                r = max(int(h.radius_px), 6)
                colour = (0, 220, 0) if h.marker_id is not None else (0, 160, 255)
                cv2.rectangle(out, (int(h.cx) - r, int(h.cy) - r),
                              (int(h.cx) + r, int(h.cy) + r), colour, 2)
                label = f"id {h.marker_id}" if h.marker_id is not None else "cue"
                cv2.putText(out, label, (int(h.cx) - r, int(h.cy) - r - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            annotate_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(annotate_dir / path.name), out)
    return summary


def print_report(summary: ReplaySummary, *, quiet: bool) -> None:
    if not quiet:
        for fr in summary.frames:
            if fr.decoded_ids or fr.cue_only:
                ids = ",".join(map(str, fr.decoded_ids)) or "-"
                print(f"{fr.path.name}  ids={ids}  cue_only={fr.cue_only}")
    rate = (100.0 * summary.n_decoded_frames / summary.n_frames
            if summary.n_frames else 0.0)
    hist = " ".join(f"id{i}:{n}" for i, n in sorted(summary.id_counts.items()))
    print(f"[replay] {summary.n_frames} frames read"
          + (f" ({summary.n_unreadable} unreadable)" if summary.n_unreadable else ""))
    print(f"[replay] decoded on {summary.n_decoded_frames} frames ({rate:.1f}%)"
          + (f" — {hist}" if hist else ""))
    if summary.first_decode:
        print(f"[replay] first decode {summary.first_decode} · "
              f"last decode {summary.last_decode}")
    else:
        print("[replay] ZERO decodes — if these are real-camera frames, check "
              "whether a printed pad was actually in view (none has ever been "
              "laid out at KMUTNB as of 2026-08-20) before blaming the detector")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("target", type=Path,
                        help="frames directory or a single image")
    parser.add_argument("--valid-ids", default=None,
                        help="comma list, e.g. 1,3,4,6 (default: all 1-6)")
    parser.add_argument("--annotate", type=Path, default=None,
                        help="write marked-up copies into this directory")
    parser.add_argument("--quiet", action="store_true",
                        help="summary only, no per-frame lines")
    args = parser.parse_args()

    paths = collect_paths(args.target)
    if not paths:
        print(f"ERROR: no frames at {args.target}")
        return 2
    valid = VALID_MARKER_IDS
    if args.valid_ids:
        try:
            valid = frozenset(int(x) for x in args.valid_ids.split(","))
        except ValueError:
            print(f"ERROR: bad --valid-ids {args.valid_ids!r}")
            return 2

    summary = replay(paths, valid_ids=valid, annotate_dir=args.annotate)
    print_report(summary, quiet=args.quiet)
    return 0 if summary.n_decoded_frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
