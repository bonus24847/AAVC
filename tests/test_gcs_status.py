"""Live AAVC-GCS pad feed (orchestrator/gcs_status.py).

The console draws one map marker per ``pads_mapped`` entry in
``captures/mission_status.json``. The user's contract (2026-08-12): pads
appear when the drone SCANS them — so the writer must (1) clobber any stale
file at startup, (2) add a pad only on a confirmed, id-decoded tracker
target, and (3) emit ENU that inverts the console's own spherical ``aavcEN``
so the marker lands on the true pad.
"""

from __future__ import annotations

import json
import math
import types
from pathlib import Path

from orchestrator.gcs_status import GcsMissionStatus
from orchestrator.target_tracker import TargetState

_LAT0, _LON0 = 13.8224940, 100.5122771   # KMUTNB L&R (gen_geo)
_R = 6_378_137.0


def _read(p: Path) -> dict:
    return json.loads(p.read_text())


def test_startup_clobbers_a_stale_status_file(tmp_path: Path) -> None:
    p = tmp_path / "mission_status.json"
    p.write_text(json.dumps({"phase": "done",
                             "pads_mapped": {"1": [74.3, 67.4]}}))
    GcsMissionStatus(p, _LAT0, _LON0, assigned=[3, 1])
    doc = _read(p)
    assert doc["pads_mapped"] == {}          # stale pads GONE before first poll
    assert doc["phase"] == "load (preflight)"  # console stepper: eggs loading
    assert doc["assigned"] == [3, 1]


def test_pad_enu_inverts_the_consoles_spherical_aavcen(tmp_path: Path) -> None:
    feed = GcsMissionStatus(tmp_path / "s.json", _LAT0, _LON0, assigned=[])
    lat, lon = 13.8227272, 100.5121023       # a real baseline pad truth
    feed.pad_confirmed(3, lat, lon)
    e, n = _read(tmp_path / "s.json")["pads_mapped"]["3"]
    # Round-trip through the console's exact JS formula (aavcEN).
    lat2 = _LAT0 + math.degrees(n / _R)
    lon2 = _LON0 + math.degrees(e / (_R * math.cos(math.radians(_LAT0))))
    assert abs(lat2 - lat) * 111_320 < 0.05  # marker within 5 cm of truth
    assert abs(lon2 - lon) * 111_320 < 0.05


def test_release_audit_line_marks_the_pad_delivered(tmp_path: Path) -> None:
    p = tmp_path / "s.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[3])
    feed.on_audit("t=103.5s DELIVERY 1 RELEASE pad=3 payload=0 lat=1 lon=2")
    feed.on_audit("t=103.5s DELIVERY 1 RELEASE pad=3 payload=0 lat=1 lon=2")
    feed.on_audit("t=9.0s DELIVERY 2 START pad=6 payload=1")     # not a release
    feed.on_audit("t=9.0s DELIVERY 2 RELEASE pad=None payload=1")  # unverified
    assert _read(p)["delivered"] == [3]      # deduped; None/START ignored


def test_tracker_pusher_shows_only_confirmed_decoded_pads(tmp_path: Path) -> None:
    p = tmp_path / "s.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[])
    targets = [
        types.SimpleNamespace(target_id=1, marker_id=3, lat=_LAT0, lon=_LON0,
                              state=TargetState.CONFIRMED),
        types.SimpleNamespace(target_id=2, marker_id=None, lat=_LAT0, lon=_LON0,
                              state=TargetState.CONFIRMED),   # blob-only: hidden
        types.SimpleNamespace(target_id=3, marker_id=5, lat=_LAT0, lon=_LON0,
                              state=TargetState.CANDIDATE),   # unconfirmed: hidden
    ]
    tracker = types.SimpleNamespace(snapshot=lambda: targets)
    push = feed.tracker_pusher(tracker)
    push(None)
    push(None)                                # idempotent on repeat fixes
    assert sorted(_read(p)["pads_mapped"]) == ["3"]


def test_progress_maps_real_phases_onto_the_console_stepper(tmp_path: Path) -> None:
    """The console's phaseIdx() substring-matches [recon, load, deliver, done]
    and stops its mission clock ONLY on the exact string 'done'."""
    p = tmp_path / "s.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[])
    for raw, step in (("preflight", "load"), ("takeoff", "recon"),
                      ("search", "recon"), ("localize", "deliver"),
                      ("drop", "deliver"), ("transit_egress", "deliver"),
                      ("rth", "deliver")):
        feed.set_progress(raw, 42.0)
        doc = _read(p)
        assert doc["phase"].startswith(step), (raw, doc["phase"])
        assert raw in doc["phase"]           # operator still sees the real phase
        assert doc["mission_time"] == 42.0
    feed.set_done(214.0)
    assert _read(p)["phase"] == "done"       # exact — the console clock-stop key
