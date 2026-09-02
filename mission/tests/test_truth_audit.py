"""Discovered-vs-truth scoring (orchestrator.audit).

SITL knows the ground-truth target positions; the blind search never reads them
for planning, but a post-flight audit scores what it discovered against truth.
These lock the matching, the served/missed accounting, and the truth loader.
"""

from __future__ import annotations

import json

from orchestrator.audit import compare_with_truth, read_truth_targets
from orchestrator.target_tracker import TargetState, TrackedTarget

_LAT, _LON = 14.6526, 101.1880


def _tt(tid: int, lat: float, lon: float,
        state: TargetState = TargetState.SERVED) -> TrackedTarget:
    return TrackedTarget(
        target_id=tid, lat=lat, lon=lon, votes_nadir=3,
        best_confidence=0.8, first_t=0.0, last_t=1.0, state=state, attempts=1,
    )


def test_matched_and_served_target_is_scored() -> None:
    truth = [{"name": "pad_1", "lat": _LAT, "lon": _LON}]
    discovered = [_tt(0, _LAT, _LON + 1.0e-5)]   # ~1 m east, SERVED
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 1
    assert comp.served == 1
    assert comp.missed == []
    assert any("matched id=0" in line and "served=True" in line for line in comp.lines)


def test_confirmed_but_unserved_counts_as_matched_not_served() -> None:
    truth = [{"name": "c1", "lat": _LAT, "lon": _LON}]
    discovered = [_tt(2, _LAT, _LON, state=TargetState.CONFIRMED)]
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 1 and comp.served == 0


def test_missed_truth_target_is_reported() -> None:
    truth = [{"name": "far", "lat": _LAT, "lon": _LON}]
    discovered = [_tt(0, _LAT + 1.0e-3, _LON)]   # ~111 m away → outside 10 m
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 0
    assert comp.missed == ["far"]
    assert any("MISSED" in line for line in comp.lines)


def test_no_detections_all_missed() -> None:
    truth = [{"name": "a", "lat": _LAT, "lon": _LON},
             {"name": "b", "lat": _LAT, "lon": _LON + 1e-3}]
    comp = compare_with_truth([], truth)
    assert comp.matched == 0 and comp.total_truth == 2
    assert set(comp.missed) == {"a", "b"}


def test_read_truth_targets_list_and_dict_forms(tmp_path) -> None:
    p = tmp_path / "targets.json"
    p.write_text(json.dumps({"targets": [
        {"name": "pad_1", "lat": _LAT, "lon": _LON},
        {"name": "pad_2", "lat": _LAT, "lon": _LON + 1e-4},
    ]}))
    got = read_truth_targets(p)
    assert [t["name"] for t in got] == ["pad_1", "pad_2"]

    p2 = tmp_path / "list.json"
    p2.write_text(json.dumps([{"lat": _LAT, "lon": _LON}]))   # no name → T1
    got2 = read_truth_targets(p2)
    assert got2[0]["name"] == "T1"


def test_read_truth_targets_missing_file_is_empty() -> None:
    assert read_truth_targets("/nonexistent/aavc_truth.json") == []


# ── V1.3 marker-id scoring ──

def _pad(tid: int, lat: float, lon: float, marker_id: int | None,
         state: TargetState = TargetState.SERVED) -> TrackedTarget:
    return TrackedTarget(
        target_id=tid, lat=lat, lon=lon, votes_nadir=3,
        best_confidence=0.9, first_t=0.0, last_t=1.0, state=state, attempts=1,
        marker_id=marker_id,
    )


def test_id_match_wins_even_with_position_scatter() -> None:
    truth = [{"name": "pad_1", "marker_id": 3, "lat": _LAT, "lon": _LON}]
    # Cluster fused 8 m off (coarse GPS) but decoded the RIGHT id.
    discovered = [_pad(0, _LAT + 7.2e-5, _LON, marker_id=3)]
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 1 and comp.missed == []
    assert any("id-matched" in line and "pad 3" in line for line in comp.lines)
    assert any("ids correct 1/1" in line for line in comp.lines)


def test_position_match_with_wrong_id_is_a_mismatch() -> None:
    truth = [{"name": "pad_1", "marker_id": 3, "lat": _LAT, "lon": _LON}]
    # Right position, WRONG decoded id — a delivery keyed on it goes astray.
    discovered = [_pad(0, _LAT, _LON + 1.0e-5, marker_id=5)]
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 0
    assert comp.missed == ["pad_1"]
    assert any("ID-MISMATCH" in line for line in comp.lines)
    assert any("ids correct 0/1" in line for line in comp.lines)


def test_unidentified_cluster_still_matches_by_distance() -> None:
    truth = [{"name": "pad_1", "marker_id": 3, "lat": _LAT, "lon": _LON}]
    discovered = [_pad(0, _LAT, _LON + 1.0e-5, marker_id=None,
                       state=TargetState.CONFIRMED)]
    comp = compare_with_truth(discovered, truth)
    assert comp.matched == 1        # position fallback (id never decoded)
    assert any("ids correct 0/1" in line for line in comp.lines)


def test_read_truth_targets_passes_marker_id(tmp_path) -> None:
    import json as _json
    p = tmp_path / "targets.json"
    p.write_text(_json.dumps({"targets": [
        {"name": "pad_1", "marker_id": 4, "lat": _LAT, "lon": _LON},
        {"name": "old", "lat": _LAT, "lon": _LON + 1e-4},   # no id → None
    ]}))
    got = read_truth_targets(p)
    assert got[0]["marker_id"] == 4
    assert got[1]["marker_id"] is None
