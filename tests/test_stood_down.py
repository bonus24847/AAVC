"""The one beacon line written for the 2026-08-29 failure never reached the
screen: `AAVC STOOD DOWN - RESTART STACK, THEN ARM` matched none of
`_parse_beacon`'s branches and fell off the end unrecorded, while the crew
swapped a pack and armed twice into a mission that had already ended.

Pinned here: the line lands in its own slot AND in the message log, any
future `AAVC ` line the parser does not know is logged rather than dropped,
and the planned battery egress has a reason text the console can show.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402


class _Bare:
    def __init__(self):
        self.s = {"messages": []}

    _parse_beacon = aavc_gcs.Link._parse_beacon


def _parsed(text):
    link = _Bare()
    link._parse_beacon(text)
    return link.s


def test_the_stood_down_line_is_recorded_and_logged() -> None:
    s = _parsed("AAVC STOOD DOWN - RESTART STACK, THEN ARM")
    assert "radio_stood_down" in s, s
    assert s["radio_stood_down"]["txt"].startswith("AAVC STOOD DOWN")
    assert any("STOOD DOWN" in m["txt"] for m in s["messages"]), s["messages"]


def test_an_unknown_beacon_line_is_logged_not_dropped() -> None:
    s = _parsed("AAVC newthing=42")
    assert any("newthing=42" in m["txt"] for m in s["messages"]), s["messages"]


def test_the_planned_battery_egress_has_operator_text() -> None:
    assert "batt-egress" in aavc_gcs.WHY_TH
    assert "แบต" in aavc_gcs.WHY_TH["batt-egress"]
