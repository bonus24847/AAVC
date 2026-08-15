"""The radio status beacon's line composer (cm4/status_beacon.py).

The beacon exists because the console's stepper/pad ✓/camera readouts all ride
WiFi, which does not reach the aircraft mid-flight. These lines are what the
operator sees INSTEAD, over the radio — so the two things worth locking are the
50-char STATUSTEXT budget (a longer line is chunked into extra packets on a link
that is already the narrow one) and that a dead camera is reported as a WARNING
rather than blending into the INFO stream.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "status_beacon", Path(__file__).resolve().parents[1] / "cm4" / "status_beacon.py")
assert _SPEC and _SPEC.loader
beacon = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(beacon)

_SEV_INFO = 6
_SEV_WARN = 4


def _texts(lines):
    return [t for _, t in lines]


def test_every_line_fits_one_statustext_packet() -> None:
    """50 chars is the STATUSTEXT payload; past it MAVLink v2 chunks the string
    into several packets, which is exactly the cost this beacon exists to
    avoid. Worst case: long phase name, 4 delivered ids, stale camera."""
    status = {"phase": "TRANSIT_EGRESS", "assigned": [1, 2, 3, 4],
              "delivered": [1, 2, 3, 4], "pads_mapped": {"1": [0, 0], "2": [0, 0]}}
    for _, text in beacon.compose_lines(status, 123.4):
        assert len(text) <= 50, f"{len(text)} chars: {text}"
        assert text.isascii(), text          # STATUSTEXT is ASCII on the wire


def test_mission_line_carries_the_numbers_the_operator_counts() -> None:
    status = {"phase": "SERVE", "assigned": [3, 1, 4, 6], "delivered": [3, 1],
              "pads_mapped": {"3": [0, 0], "1": [0, 0], "4": [0, 0]}}
    line = _texts(beacon.compose_lines(status, 0.5))[0]
    assert "p=SERVE" in line
    assert "d=2/4" in line          # delivered / assigned
    assert "m=3" in line            # pads actually on the map ("looking" vs "stuck")
    assert "ok=3,1" in line         # WHICH ids are done, in delivery order


def test_no_mission_yet_is_reported_not_omitted() -> None:
    """Before the first GO there is no mission_status.json. Sending nothing
    would be indistinguishable from a dead beacon."""
    assert "idle" in _texts(beacon.compose_lines(None, 0.2))[0]


def test_camera_state_maps_to_severity() -> None:
    """A stale/absent frame must arrive as WARNING: on the console these lines
    land in one scrolling message list, and an INFO line saying the camera died
    is a line nobody reads in time."""
    ok = beacon.compose_lines(None, 0.9)[1]
    dead = beacon.compose_lines(None, 60.0)[1]
    missing = beacon.compose_lines(None, None)[1]
    assert ok[0] == _SEV_INFO and "cam=OK" in ok[1]
    assert dead[0] == _SEV_WARN and "cam=DEAD" in dead[1]
    assert missing[0] == _SEV_WARN and "cam=NONE" in missing[1]


def test_camera_threshold_sits_above_the_grabber_rate_not_at_it() -> None:
    """The grabber writes several times a second and the mission's own vision
    gate rejects frames older than 2 s, so the beacon must not cry DEAD at the
    first hiccup — nor stay quiet long enough for a wedged grabber to look fine
    for a whole delivery."""
    assert 2.0 < beacon._CAM_DEAD_S <= 10.0
    assert "cam=OK" in beacon.compose_lines(None, beacon._CAM_DEAD_S - 0.1)[1][1]
    assert "cam=DEAD" in beacon.compose_lines(None, beacon._CAM_DEAD_S + 0.1)[1][1]
