"""The RC verdict, which is the part that was wrong for two days.

"RC unavailable" was reported by every tool this project had while the radio
was working perfectly: the kill switch was up, PX4 raised an arming-check error
on the `remote_control` component, and `SYS_STATUS.hpp::fillOutComponent` only
sets the RC_RECEIVER health bit when that component has NO such error. A switch
position and a dead transmitter produced the identical indicator.

These tests pin the distinction, not the plumbing: given what was seen on the
wire, does the tool say "the link" or "the switches"? Getting that wrong sends
the next person to debug a radio that is not broken.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from rc_check import RcObservation, classify  # noqa: E402


def _obs(*, frames: int = 0, healthy: bool | None = None,
         channels: list[list[int]] | None = None) -> RcObservation:
    o = RcObservation()
    for values in (channels or []):
        o.note_channels(values, rssi=100, chancount=len(values))
    if frames and not channels:
        for _ in range(frames):
            o.note_channels([1500] * 16, rssi=100, chancount=16)
    o.healthy = healthy
    return o


def test_no_frames_at_all_is_the_link() -> None:
    """Nothing arriving is the only case that IS the radio."""
    code, verdict = classify(_obs(frames=0))
    assert code == 2
    assert "link" in verdict


def test_frames_arriving_but_unhealthy_points_at_the_switches() -> None:
    """The 2026-08-21 blocker. The verdict must NOT send anyone to the radio:
    frames were arriving the whole time at rssi 100 with live sticks, and the
    kill switch was the entire fault."""
    code, verdict = classify(_obs(frames=20, healthy=False))
    assert code == 1
    assert "SWITCH" in verdict
    assert "kill" in verdict


def test_frames_and_health_is_a_pass() -> None:
    code, verdict = classify(_obs(frames=20, healthy=True))
    assert code == 0
    assert "healthy" in verdict


def test_frames_without_a_sys_status_is_not_reported_as_fine() -> None:
    """No SYS_STATUS means the health bit was never read. Saying "OK" there
    would be the same failure in the other direction — asserting something that
    was not observed."""
    code, verdict = classify(_obs(frames=20, healthy=None))
    assert code == 1
    assert "cannot say" in verdict


def test_moved_channels_sees_a_stick_and_ignores_jitter() -> None:
    """Live-vs-frozen was the experiment that killed the failsafe hypothesis on
    the day; the tool has to be able to make the same call. One microsecond of
    wobble is not a stick, 900 is."""
    still = [1500] * 16
    jitter = [1502] + [1500] * 15          # 2 us — noise
    stick = [1500, 1500, 1918] + [1500] * 13
    assert _obs(channels=[still, jitter]).moved_channels == []
    assert _obs(channels=[still, stick]).moved_channels == [3]


def test_a_channel_value_is_reported_only_when_it_was_seen() -> None:
    o = _obs(channels=[[1500] * 8])        # an 8-channel frame
    assert o.value(8) == 1500
    assert o.value(16) is None
