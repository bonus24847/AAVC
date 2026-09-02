"""Bang Bo flight 5 (30 Aug 2026 00:41, ULog 17_41_39): at t=39 s the FC
stopped receiving ANYTHING from the CM4 on TELEM2 (rx count frozen, heartbeat
gone at t=43) while the NOMAD radio kept reporting a healthy, armed, airborne
aircraft. PX4 held the last reposition setpoint, the aircraft hovered on the
spot for 60 s and the pilot landed it by hand, not knowing why it "would not
turn". The console had every fact — armed + in_air from the FC heartbeat, and
no `AAVC …` beacon line for a minute — and said nothing.

`cm4_silent(s, now)` is true when the radio says the aircraft is armed and in
the air, the radio itself is alive, and no beacon line has arrived for
`CM4_SILENT_S`. It is NOT raised when the radio is the thing that died (that
is the existing "📻 ขาด" badge), nor on the ground, nor before the first
beacon line of a session.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402

NOW = 1_000_000.0


def _s(**kw):
    s = {"armed": True, "in_air": True, "last_hb": NOW - 1.0,
         "radio_last_aavc": NOW - 20.0}
    s.update(kw)
    return s


def test_silent_cm4_in_flight_is_flagged() -> None:
    assert aavc_gcs.cm4_silent(_s(), NOW) is True


def test_recent_beacon_line_is_not_silent() -> None:
    assert aavc_gcs.cm4_silent(_s(radio_last_aavc=NOW - 5.0), NOW) is False


def test_on_the_ground_is_not_flagged() -> None:
    assert aavc_gcs.cm4_silent(_s(in_air=False), NOW) is False
    assert aavc_gcs.cm4_silent(_s(armed=False, in_air=False), NOW) is False


def test_a_dead_radio_is_not_blamed_on_the_cm4() -> None:
    # no FC heartbeat for 20 s: the radio badge covers this, not the CM4 banner
    assert aavc_gcs.cm4_silent(_s(last_hb=NOW - 20.0), NOW) is False


def test_no_beacon_line_yet_this_session_is_not_flagged() -> None:
    s = _s(); del s["radio_last_aavc"]
    assert aavc_gcs.cm4_silent(s, NOW) is False


def test_every_beacon_line_stamps_radio_last_aavc() -> None:
    class _Bare:
        def __init__(self):
            self.s = {"messages": []}
        _parse_beacon = aavc_gcs.Link._parse_beacon
    link = _Bare()
    link._parse_beacon("AAVC cam=OK 0.9s")
    assert "radio_last_aavc" in link.s
    link._parse_beacon("AAVC p=recon (search) d=0/1 m=0 ok=-")
    assert link.s["radio_last_aavc"] > 0
