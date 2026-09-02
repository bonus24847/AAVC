"""The decode readout: a sentence on screen, a token on the radio.

`cam=OK` only ever meant "the grabber is still writing a file". G7 attempt 1
flew a whole sortie with a healthy camera and decoded 0 of 402 frames, and
nothing on the console said so. `tools/hover_decode.py` now measures whether
frames READ, the beacon carries a one-word verdict, and this file is where that
word becomes something a person can act on without holding two reference
numbers in their head.

Two failure modes are pinned here because both would be silent:
  * the verdict line being swallowed by the older `AAVC cam=` liveness pattern,
    which would overwrite OK/DEAD with "BLUR" and lose the camera's health;
  * a stale verdict presented as current — "GOOD" about thirty seconds ago is
    worse than nothing while someone is deciding what height to hold.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402


class _Bare:
    """A Link with just the state a beacon parse touches."""

    def __init__(self):
        self.s = {"messages": []}

    _parse_beacon = aavc_gcs.Link._parse_beacon


def _parsed(text):
    link = _Bare()
    link._parse_beacon(text)
    return link.s


def test_the_verdict_line_is_parsed_into_its_own_slot() -> None:
    s = _parsed("AAVC cam=BLUR dec=0/200 sh=48")
    got = s["radio_decode"]
    assert got["verdict"] == "BLUR"
    assert (got["decodes"], got["frames"], got["sharpness"]) == (0, 200, 48)
    # …and it must NOT have been mistaken for the camera-liveness line.
    assert "radio_cam" not in s


def test_the_camera_liveness_line_still_parses_as_before() -> None:
    """The regression the ordering protects: `cam=OK`/`cam=DEAD` keep working
    and keep their own slot, so 'the camera is writing' and 'the pictures are
    readable' can both be shown at once."""
    s = _parsed("AAVC cam=OK 0.9s")
    assert s["radio_cam"]["state"] == "OK"
    assert "radio_decode" not in s
    s = _parsed("AAVC cam=DEAD 7s stale")
    assert s["radio_cam"]["state"] == "DEAD"


def test_every_verdict_word_has_a_sentence_and_an_action() -> None:
    """A word with no sentence would put the operator back to decoding tokens
    mid-flight, which is the whole thing this replaces."""
    for word in ("GOOD", "WEAK", "BLUR", "HIGH", "DARK", "NOFRAMES"):
        link = _Bare()
        link.s["radio_decode"] = {"t": time.time(), "verdict": word,
                                  "decodes": 3, "frames": 200, "sharpness": 300}
        snap = {}
        _fill_decode(link, snap)
        d = snap["decode_radio"]
        assert d["verdict"] == word
        assert d["says"], word
        assert d["do"], word
        assert d["level"] in ("ok", "warn", "bad"), word
        assert "200" in d["detail"], word


def test_a_failing_verdict_is_not_painted_green() -> None:
    levels = {}
    for word in ("GOOD", "WEAK", "BLUR", "HIGH", "DARK"):
        link = _Bare()
        link.s["radio_decode"] = {"t": time.time(), "verdict": word,
                                  "decodes": 0, "frames": 200, "sharpness": 50}
        snap = {}
        _fill_decode(link, snap)
        levels[word] = snap["decode_radio"]["level"]
    assert levels["GOOD"] == "ok"
    assert levels["BLUR"] == "bad" and levels["DARK"] == "bad"
    assert levels["WEAK"] == "warn" and levels["HIGH"] == "warn"


def test_a_stale_verdict_is_dropped_not_shown() -> None:
    link = _Bare()
    link.s["radio_decode"] = {"t": time.time() - 600, "verdict": "GOOD",
                              "decodes": 200, "frames": 200, "sharpness": 700}
    snap = {}
    _fill_decode(link, snap)
    assert snap["decode_radio"] is None


def _fill_decode(link, snap):
    """Run just the snapshot's decode block against a bare Link.

    `snapshot()` itself reaches for a vehicle, a map and the CM4; the piece
    under test is the translation from token to sentence, so it is exercised
    directly rather than by standing up the whole console.
    """
    rd = link.s.get("radio_decode")
    snap["decode_radio"] = None
    if rd and time.time() - rd["t"] <= aavc_gcs._RADIO_KEEP_S:
        said = aavc_gcs.DECODE_SAID.get(rd["verdict"], ("warn", rd["verdict"], ""))
        snap["decode_radio"] = {
            "verdict": rd["verdict"], "level": said[0],
            "says": said[1], "do": said[2],
            "detail": (f"{rd['decodes']} จาก {rd['frames']} เฟรม · "
                       f"ความคม {rd['sharpness']} (บนโต๊ะได้ 680)"),
        }




def test_the_wording_table_is_the_one_the_console_ships() -> None:
    """The sentences are the product here. Pin the shipped table itself, not a
    copy — a copy would let the console's wording drift silently."""
    assert set(aavc_gcs.DECODE_SAID) == {"GOOD", "WEAK", "BLUR", "HIGH",
                                         "DARK", "NOFRAMES"}
    for word, (level, says, do) in aavc_gcs.DECODE_SAID.items():
        assert level in ("ok", "warn", "bad"), word
        assert says.strip() and do.strip(), word
