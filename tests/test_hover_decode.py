"""Which WORD the hover test reports, because the word is the whole product.

`cam=OK` never meant the pictures were usable: G7 attempt 1 flew a healthy
camera for a sortie and decoded 0 of 402 frames. `tools/hover_decode.py` answers
the missing question during a hand-flown hover, and its output is one word
because a number would need two reference values held in the operator's head
(the bench scored 680-780, the failed flight 41-76) while a word needs none.

The four failing words are not shades of the same thing — they point at four
different fixes: fly lower (BLUR), you are simply too high (HIGH), raise the
gain (DARK), the camera is not writing (NOFRAMES). Mislabelling one sends
someone to the wrong repair mid-flight, so the ordering is what these tests
pin.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from hover_decode import (  # noqa: E402
    BRIGHT_MIN,
    GOOD_FRAC,
    SHARP_MIN,
    _median,
    verdict,
)

_LIT = 120.0          # a comfortably exposed frame
_SHARP = 640.0        # bench-grade sharpness


def test_a_decode_beats_every_other_signal() -> None:
    """If frames READ, nothing else matters — not sharpness, not brightness.
    The metrics exist to explain a failure, never to overrule a success."""
    assert verdict(frames=200, decodes=200, sharpness=1.0,
                   brightness=1.0) == "GOOD"


def test_the_good_threshold_is_a_fraction_not_a_single_lucky_frame() -> None:
    n = 200
    need = int(n * GOOD_FRAC)
    assert verdict(frames=n, decodes=need, sharpness=_SHARP,
                   brightness=_LIT) == "GOOD"
    assert verdict(frames=n, decodes=need - 1, sharpness=_SHARP,
                   brightness=_LIT) == "WEAK"


def test_one_decode_in_a_tiny_window_is_still_good() -> None:
    """The fraction must not round down to "zero decodes required" on a short
    window and call a dead camera GOOD."""
    assert verdict(frames=2, decodes=1, sharpness=_SHARP,
                   brightness=_LIT) == "GOOD"
    assert verdict(frames=2, decodes=0, sharpness=_SHARP,
                   brightness=_LIT) == "HIGH"


def test_dark_is_decided_before_blur() -> None:
    """This ordering is the point of the test. A 2 ms exposure with NO
    auto-gain behind it (the OV9281 has none) underexposes as the light drops,
    and an underexposed frame ALSO scores low sharpness. Checking blur first
    would send the operator to the camera mount when the answer is gain."""
    dark_and_flat = dict(frames=200, decodes=0, sharpness=10.0,
                         brightness=BRIGHT_MIN - 1)
    assert verdict(**dark_and_flat) == "DARK"


def test_blur_and_high_are_told_apart_by_sharpness_alone() -> None:
    """Both read "no marker found" on screen today, and they are opposite
    repairs: BLUR is a camera problem, HIGH just means come down."""
    assert verdict(frames=200, decodes=0, sharpness=SHARP_MIN - 1,
                   brightness=_LIT) == "BLUR"
    assert verdict(frames=200, decodes=0, sharpness=SHARP_MIN + 1,
                   brightness=_LIT) == "HIGH"


def test_the_flight_that_failed_reads_as_blur() -> None:
    """The real numbers from G7 attempt 1: 402 frames, zero decodes, sharpness
    41-76 in daylight. The tool must name that BLUR — if it had existed, the
    sortie would not have been flown to the end."""
    assert verdict(frames=402, decodes=0, sharpness=58.0,
                   brightness=_LIT) == "BLUR"


def test_the_bench_walk_test_reads_as_good() -> None:
    """The other real measurement: the same marker decoded continuously from
    1.9 to 14 m on the bench at 680-780."""
    assert verdict(frames=200, decodes=195, sharpness=720.0,
                   brightness=_LIT) == "GOOD"


def test_no_frames_is_its_own_word() -> None:
    """Not BLUR: there is nothing to be blurry. This is the grabber, and the
    camera-liveness line is the one to read."""
    assert verdict(frames=0, decodes=0, sharpness=0.0, brightness=0.0) == "NOFRAMES"


@pytest.mark.parametrize("values, want", [
    ([], 0.0),
    ([5.0], 5.0),
    ([3.0, 1.0], 2.0),
    ([9.0, 1.0, 5.0], 5.0),
])
def test_median_survives_the_empty_window(values, want) -> None:
    """The window is empty for the first frame of every run; a crash there
    would take the instrument out exactly when the hover starts."""
    assert _median(values) == want


def test_the_median_is_used_so_one_bad_frame_cannot_flip_the_word() -> None:
    """A hover produces the occasional smeared frame while the aircraft
    corrects. A mean would let a handful of those drag a healthy window under
    the threshold."""
    mostly_sharp = [700.0] * 19 + [5.0]
    assert _median(mostly_sharp) >= SHARP_MIN
