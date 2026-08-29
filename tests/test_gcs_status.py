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
                              state=TargetState.CANDIDATE),   # unconfirmed: orange lane
    ]
    tracker = types.SimpleNamespace(
        snapshot=lambda: targets,
        identified_unconfirmed=lambda marker_id=None: [targets[2]])
    push = feed.tracker_pusher(tracker)
    push(None)
    push(None)                                # idempotent on repeat fixes
    doc = _read(p)
    assert sorted(doc["pads_mapped"]) == ["3"]
    # 2026-08-21: the identified-but-unconfirmed id is no longer hidden — it
    # rides its own lane so the console can show it ORANGE.
    assert sorted(doc["pads_identified"]) == ["5"]


def test_identified_lane_promotes_and_does_not_rewrite_every_tick(
        tmp_path: Path) -> None:
    """G7 2026-08-21: ids seen live must reach the console immediately —
    but at the ~3 Hz on_fix cadence an UNCHANGED identified set must not
    rewrite the file (SD hammer, same class as the anomaly-rewrite guard),
    and a pad that CONFIRMS must leave the orange lane for the map lane."""
    p = tmp_path / "s.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[])
    cand = types.SimpleNamespace(target_id=3, marker_id=5, lat=_LAT0, lon=_LON0,
                                 state=TargetState.CANDIDATE)
    identified = [cand]
    tracker = types.SimpleNamespace(
        snapshot=lambda: [cand],
        identified_unconfirmed=lambda marker_id=None: list(identified))
    push = feed.tracker_pusher(tracker)
    push(None)
    assert sorted(_read(p)["pads_identified"]) == ["5"]

    writes = 0
    orig = feed._write

    def _counting() -> None:
        nonlocal writes
        writes += 1
        orig()
    feed._write = _counting                   # type: ignore[method-assign]
    push(None)
    push(None)
    assert writes == 0                        # unchanged set -> no rewrite

    identified.clear()                        # the tracker promoted id 5
    promoted = types.SimpleNamespace(target_id=3, marker_id=5, lat=_LAT0,
                                     lon=_LON0, state=TargetState.CONFIRMED)
    tracker.snapshot = lambda: [promoted]
    push(None)
    doc = _read(p)
    assert sorted(doc["pads_mapped"]) == ["5"]
    assert doc["pads_identified"] == {}       # orange lane cleared on confirm


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


def test_transit_passes_survive_a_flooded_event_feed(tmp_path: Path) -> None:
    """The console (and the radio beacon) rebuild the P1·P2·P3 transit chip by
    scanning events for 'ผ่านจุด Pn'. The event feed is a rolling cap of 10, so a
    flight's 3 ingress passes were evicted by the pad-confirm/release events that
    followed — the chip un-ticked MID-FLIGHT on the radio-only console. Transit
    passes must ride a lane the cap cannot evict."""
    p = tmp_path / "s.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1, 2, 3, 4])
    for pt in ("P1", "P2", "P3"):
        g.on_audit(f"t=10s TRANSIT_PASS {pt} ingress flight=1 d=0.5m")
    for i in range(15):                 # well past the 10-event rolling cap
        g._event(f"toast {i}")
    g._write()
    texts = " ".join(e["text"] for e in _read(p)["events"])
    for pt in ("P1", "P2", "P3"):
        assert f"ผ่านจุด {pt}" in texts, f"{pt} evicted from the feed: {texts}"


def test_a_new_flight_clears_the_previous_transit_ticks(tmp_path: Path) -> None:
    """The sticky transit lane must reset per flight, or flight 2's chip would
    start already ticked from flight 1."""
    p = tmp_path / "s.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1, 2, 3, 4])
    g.on_audit("t=10s TRANSIT_PASS P1 ingress flight=1 d=0.5m")
    g.on_audit("t=700s FLIGHT 2 START eggs=2 ids=3,4 remaining=500s")
    texts = " ".join(e["text"] for e in _read(p)["events"])
    assert "ผ่านจุด P1" not in texts


def test_a_recurring_anomaly_is_not_rewritten_every_tick(tmp_path, monkeypatch) -> None:
    """A low-battery flight audits battery_critical_36%, _35%, _34%… every tick.
    The first sets the home reason; the rest change nothing, so they must NOT
    rewrite the SD file — the None-guard gated only the assignment, not the
    _write() beside it, so a critical-battery moment hammered the card."""
    p = tmp_path / "s.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1])
    g.on_audit("t=1s battery_critical_36%")          # first cause — one write
    writes: list[int] = []
    monkeypatch.setattr(g, "_write", lambda: writes.append(1))
    g.on_audit("t=2s battery_critical_35%")          # same reason, already set
    g.on_audit("t=3s battery_critical_34%")
    assert writes == []                              # no rewrite for the repeats


def test_home_reason_maps_the_preflight_refusals(tmp_path: Path) -> None:
    """A GO refused at the gate must tell the radio-only operator WHY, the same
    way the energy refusal already does — the envelope and time-reserve
    refusals had no needle, so the flight silently would not stage with no code
    on the console."""
    p = tmp_path / "mission_status.json"
    g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1])
    g.on_audit("t=5.0s sortie 1 refused (envelope params)")
    assert _read(p)["home_reason_code"] == "envelope"

    q = tmp_path / "s2.json"
    g2 = GcsMissionStatus(q, _LAT0, _LON0, assigned=[1])
    g2.on_audit("t=5.0s sortie 1 refused (time reserve)")
    assert _read(q)["home_reason_code"] == "time-gate"


def test_plan_pusher_writes_the_console_map_path(tmp_path: Path) -> None:
    """Fix 3 (G7 debrief 2026-08-21): the console map must show where the
    aircraft is going NEXT — _plan_pusher mirrors every rebuilt live plan
    into mission_status.json as [[lat, lon, kind, seq], ...] + plan_ptr,
    dashboard present or not. DROP_PAYLOAD rides its GOTO's position and is
    skipped; seq is a 1-based display index over the kept points.

    ``plan_ptr`` is TRANSLATED into that same display index (2026-08-22): the
    mission counts commands, the map counts drawn waypoints, and every skipped
    command slides the two apart — a raw pointer would have highlighted the
    wrong stop, which is why the console never drew it at all."""
    from mission_brain.schemas import (
        CommandKind,
        Coordinate,
        MissionCommand,
        MissionPhase,
        MissionPlan,
    )
    from orchestrator.main import _plan_pusher

    p = tmp_path / "s.json"
    feed = GcsMissionStatus(p, _LAT0, _LON0, assigned=[])
    coord = Coordinate(lat=_LAT0, lon=_LON0, alt_m=16.0)
    cmds = [
        MissionCommand(seq=0, kind=CommandKind.TAKEOFF, phase=MissionPhase.TAKEOFF,
                       coord=coord, altitude_m=16.0),
        MissionCommand(seq=1, kind=CommandKind.GOTO, phase=MissionPhase.LOCALIZE,
                       coord=coord, altitude_m=16.0, stop_index=0),
        MissionCommand(seq=2, kind=CommandKind.DROP_PAYLOAD, phase=MissionPhase.DROP,
                       coord=coord, payload_id=0, stop_index=0),
        MissionCommand(seq=3, kind=CommandKind.RTH, phase=MissionPhase.RTH,
                       coord=coord),
    ]
    plan = MissionPlan(mission_id="plan-test", expected_duration_s=100.0,
                       commands=cmds, target_group_strategy="x",
                       fallback_strategy="y")
    push = _plan_pusher(None, feed)          # headless: no dashboard at all
    push(plan, 2)
    doc = _read(p)
    # command 2 is the DROP_PAYLOAD, which is not drawn — the pointer must land
    # on the next stop that IS (the RTH, display seq 3), never on seq 2, which
    # is a waypoint the aircraft already left.
    assert doc["plan_ptr"] == 3
    assert [row[2] for row in doc["plan"]] == ["takeoff", "localize", "rth"]
    assert [row[3] for row in doc["plan"]] == [1, 2, 3]
    assert abs(doc["plan"][0][0] - _LAT0) < 1e-6
    assert abs(doc["plan"][0][1] - _LON0) < 1e-6

    # unchanged plan + pointer -> no rewrite (same guard class as the
    # identified lane); a moved pointer alone DOES rewrite.
    writes = 0
    orig = feed._write

    def _counting() -> None:
        nonlocal writes
        writes += 1
        orig()
    feed._write = _counting                   # type: ignore[method-assign]
    push(plan, 2)
    assert writes == 0
    # a DIFFERENT raw pointer that means the same drawn leg is also no news
    push(plan, 3)
    assert writes == 0
    push(plan, 0)
    assert writes == 1 and _read(p)["plan_ptr"] == 1


def test_home_reason_maps_the_planned_battery_egress(tmp_path: Path) -> None:
    """The planned 30 % egress (operator 2026-08-27) audits its own lines —
    `FLIGHT n SWEEP battery egress …` from the sweep, `FLIGHT n BATTERY EGRESS
    …` from the delivery gate — and neither matched the reason table, so a
    flight that came home to swap the pack showed NO reason on the console
    and no `why=` on the radio (found 2026-08-29)."""
    for line in ("t=300.0s FLIGHT 1 SWEEP battery egress batt=29% < 30% — "
                 "returning via the corridor for resupply",
                 "t=300.0s FLIGHT 1 BATTERY EGRESS batt=41% < 30% floor + 12% "
                 "delivery cost before a delivery — returning via the corridor "
                 "for resupply"):
        p = tmp_path / f"s{abs(hash(line))}.json"
        g = GcsMissionStatus(p, _LAT0, _LON0, assigned=[1, 2, 3])
        g.on_audit(line)
        doc = _read(p)
        assert doc["home_reason_code"] == "batt-egress", (line, doc)
        assert "แบต" in doc["home_reason"], doc
