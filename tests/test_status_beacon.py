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
    avoid. Worst case: the LONGEST phase the orchestrator writes
    ("recon (preflight)", 17), 4 delivered ids, stale camera."""
    status = {"phase": "recon (preflight)", "assigned": [1, 2, 3, 4],
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


def test_pad_coordinates_ride_the_radio_in_one_packet_chunks() -> None:
    """Operator 2026-08-17: the map's pad markers must survive WiFi loss, so
    the beacon relays pads_mapped ENU verbatim — chunked so no line ever
    exceeds one STATUSTEXT packet even with all six pads mapped at the far
    corner of the field (worst-case digit count)."""
    pads = {str(i): [-112.3 + i, 87.6 - i] for i in range(1, 7)}
    status = {"phase": "SERVE", "assigned": [1, 2, 3, 4], "delivered": [],
              "pads_mapped": pads}
    plines = [t for t in _texts(beacon.compose_lines(status, 0.5))
              if t.startswith("AAVC pads")]
    assert plines, "no pads lines emitted"
    for t in plines:
        assert len(t) <= 50 and t.isascii(), t
    entries = " ".join(t[len("AAVC pads "):] for t in plines).split()
    assert sorted(e.split(":")[0] for e in entries) == sorted(pads)
    pid, en = entries[0].split(":")
    e, n = (float(v) for v in en.split(","))
    assert abs(e - pads[pid][0]) <= 0.05 and abs(n - pads[pid][1]) <= 0.05


def test_no_mapped_pads_means_no_pads_line() -> None:
    """Before the sweep finds anything there is nothing to place — an empty
    pads line would just burn a packet on the narrow link."""
    status = {"phase": "recon", "assigned": [1, 2], "delivered": [],
              "pads_mapped": {}}
    assert not [t for t in _texts(beacon.compose_lines(status, 0.5))
                if t.startswith("AAVC pads")]


def test_home_reason_code_rides_the_radio_as_a_warning() -> None:
    """The WHY of an unfinished homecoming must reach the operator over the
    radio too — as an ascii code (STATUSTEXT is ascii on the wire; the console
    maps it back to Thai), WARN severity, one packet."""
    status = {"phase": "RTH", "assigned": [1, 2, 3, 4], "delivered": [1, 2],
              "pads_mapped": {}, "home_reason_code": "budget"}
    lines = beacon.compose_lines(status, 0.5)
    whys = [(sev, t) for sev, t in lines if t.startswith("AAVC why=")]
    assert whys == [(_SEV_WARN, "AAVC why=budget")]
    assert whys[0][1].isascii() and len(whys[0][1]) <= 50


def test_no_reason_means_no_why_line() -> None:
    status = {"phase": "SERVE", "assigned": [1], "delivered": [],
              "pads_mapped": {}}
    assert not [t for _, t in beacon.compose_lines(status, 0.5)
                if t.startswith("AAVC why=")]


def test_progress_line_carries_what_the_awareness_pack_switches_on() -> None:
    """``progress`` is not one readout among many: the console's %-bar AND its
    milestone strip both test ``typeof mission.progress === 'number'`` before
    drawing anything, so a feed without it silently loses the whole awareness
    pack rather than degrading. The two derived fields ride along because the
    console reads them out of Thai strings an ASCII STATUSTEXT cannot carry —
    ``tp`` replaces searching the event feed for "ผ่านจุด Pn", ``cur`` replaces
    searching progress_label for "pad N "."""
    status = {"phase": "localize", "assigned": [1, 3, 4, 6], "delivered": [1, 3],
              "pads_mapped": {}, "progress": 73, "eta_s": 210,
              "progress_label": "ส่งของ pad 4 (3/4)",
              "events": [{"text": "✅ ผ่านจุด P1"}, {"text": "✅ ผ่านจุด P2"}]}
    line = next(t for t in _texts(beacon.compose_lines(status, 0.3))
                if t.startswith("AAVC prg="))
    assert "prg=73" in line
    assert "eta=210" in line
    assert "tp=110" in line          # P1, P2 passed; P3 not yet
    assert "cur=4" in line           # the pad being served RIGHT NOW


def test_progress_line_is_omitted_when_the_writer_has_no_progress_field() -> None:
    """The sibling repo's writer does not emit progress. Sending ``prg=0`` for
    it would park the console's bar at zero for a whole flight, which reads as
    "stuck" rather than "not reported"; the console already falls back to the
    plain stepper when the field is absent."""
    status = {"phase": "search", "assigned": [1], "delivered": [], "pads_mapped": {}}
    assert not [t for t in _texts(beacon.compose_lines(status, 0.3))
                if t.startswith("AAVC prg=")]


def test_progress_line_fits_the_packet_at_its_longest() -> None:
    status = {"phase": "transit_ingress", "assigned": [1, 3, 4, 6],
              "delivered": [1, 3, 4, 6], "pads_mapped": {}, "progress": 100,
              "eta_s": 1200, "progress_label": "ส่งของ pad 6 (4/4)",
              "events": [{"text": "✅ ผ่านจุด P%d" % n} for n in (1, 2, 3)]}
    line = next(t for t in _texts(beacon.compose_lines(status, 0.3))
                if t.startswith("AAVC prg="))
    assert len(line) <= 50 and line.isascii(), f"{len(line)}: {line}"


def test_phase_survives_intact_because_the_console_keys_off_its_text() -> None:
    """The console does not just print the phase. The 🚀 button reports
    "staged" — the aircraft has CONFIRMED it holds the mission — only when the
    phase contains "preflight". Truncated to 9 chars that arrived as
    "recon (pr", the match failed, and the radio path jumped straight to
    "flying": a button claiming the mission reached the drone with nothing
    behind the claim. Phases travel whole."""
    status = {"phase": "recon (preflight)", "assigned": [], "delivered": [],
              "pads_mapped": {}}
    line = _texts(beacon.compose_lines(status, 0.3))[0]
    assert "p=recon (preflight)" in line
    assert "preflight" in line          # what the button actually searches for

    egress = {"phase": "transit_egress", "assigned": [1], "delivered": [1],
              "pads_mapped": {}}
    assert "p=transit_egress" in _texts(beacon.compose_lines(egress, 0.3))[0]
