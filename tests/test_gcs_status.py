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

_LAT0, _LON0 = 13.8228032, 100.5116267   # KMUTNB L&R (gen_geo)
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
    assert doc["phase"] == "recon (preflight)"  # stepper starts on recon
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
    for raw, step in (("preflight", "recon"), ("takeoff", "recon"),
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


def test_progress_bar_fields_track_the_mission(tmp_path: Path) -> None:
    # Operator request 2026-08-14 ("แถบ % แบบโปรแกรมโหลด"): the writer
    # publishes progress/progress_label/eta_s — milestone model, monotonic
    # within a flight, 100 exactly at done.
    p = tmp_path / "mission_status.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[3, 1], serve_cost_s=80)
    feed.set_progress("takeoff", 5.0)
    d1 = _read(p)
    assert d1["progress"] == 6 and d1["eta_s"] > 0
    feed.set_progress("search", 30.0)
    feed.set_progress("search", 60.0)          # creep while sweeping
    d2 = _read(p)
    assert 20 <= d2["progress"] <= 55 and d2["progress_label"] == "กวาดหา pad"
    feed.on_audit("t=90s DELIVERY 1 RELEASE pad=3 payload=0 lat=1 lon=2")
    feed.set_progress("drop", 100.0, delivered=1, assigned=2)
    d3 = _read(p)
    assert d3["progress"] >= d2["progress"]    # monotonic
    assert "pad 1" in d3["progress_label"]     # next undelivered pad named
    feed.set_done(120.0)
    d4 = _read(p)
    assert d4["progress"] == 100 and d4["eta_s"] == 0


def test_events_feed_for_console_toasts(tmp_path: Path) -> None:
    p = tmp_path / "mission_status.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[3])
    feed.pad_confirmed(3, _LAT0, _LON0)
    feed.on_audit("t=50s TRANSIT_PASS P2 ingress flight=1 d=0.8m")
    feed.on_audit("t=90s DELIVERY 1 RELEASE pad=3 payload=0 lat=1 lon=2")
    feed.on_audit("t=95s PILOT TAKEOVER mode=POSCTL — orchestrator standing down")
    ev = _read(p)["events"]
    texts = [e["text"] for e in ev]
    assert any("เจอ pad 3" in t for t in texts)
    assert any("ผ่านจุด P2" in t for t in texts)
    assert any("วางแล้ว pad 3" in t for t in texts)
    assert ev[-1]["warn"] is True              # takeover flagged as warning


def test_home_reason_surfaces_first_cause_and_clears_on_next_flight(
        tmp_path: Path) -> None:
    """Operator 2026-08-18: 'ผมจะได้รู้ว่ากลับ home เพราะอะไร'. The audit's own
    words become a persistent reason field: the FIRST terminal cause of a
    flight sticks (the energy refusal that FOLLOWS a budget abort must not
    overwrite it), and the next FLIGHT START wipes it."""
    p = tmp_path / "mission_status.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1, 2, 3, 4])
    assert _read(p)["home_reason"] is None

    g.on_audit("t=568.0s DELIVERY abort: flight 1 skipping remaining "
               "ids=3,4 (remaining=632s batt=36%) — returning with the egg(s)")
    doc = _read(p)
    assert doc["home_reason_code"] == "budget"
    assert "กลับพร้อมไข่" in doc["home_reason"]

    g.on_audit("t=627.3s sortie 2 refused (energy reserve)")
    assert _read(p)["home_reason_code"] == "budget"   # first cause sticks

    g.on_audit("t=700.0s FLIGHT 2 START eggs=2 ids=3,4 remaining=500s")
    doc = _read(p)
    assert doc["home_reason"] is None and doc["home_reason_code"] is None


def test_home_reason_maps_watchdog_kinds(tmp_path: Path) -> None:
    """The safety watchdog's audited anomaly kinds land as operator text —
    including the 2026-08-17 GPS policy (loss -> LAND in place)."""
    p = tmp_path / "mission_status.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1])
    g.on_audit("t=120.0s gps_loss_sustained")
    doc = _read(p)
    assert doc["home_reason_code"] == "gps"
    assert "ลงจอด" in doc["home_reason"]
