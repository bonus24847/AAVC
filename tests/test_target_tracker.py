"""Pad-registry confirmation from streaming fixes (orchestrator.target_tracker).

The blind search confirms landing pads from noisy single detections. These lock
the rules the mission depends on: nadir temporal-consistency confirmation
(non-nadir fixes are ignored — single-camera doctrine), decoded-vote confirmation
for identified clusters, marker-id cluster keying, median position fusion, the
ingest gates, the serve state machine, served-duplicate suppression, and
thread-safe ingest.
"""

from __future__ import annotations

import threading

from orchestrator.target_tracker import TargetState, TargetTracker
from orchestrator.vision_worker import TargetFix

_LAT, _LON = 13.7308, 100.7886


def _fix(lat=_LAT, lon=_LON, *, camera="nadir", conf=0.8, radius_px=3.4,
         ground=1.0, slant=16.0, t=0.0, marker_id=None) -> TargetFix:
    """A fix that PASSES every gate by default (nadir, conf 0.8, marker-equivalent
    radius ≈ expected for the 0.4 m marker at 16 m slant)."""
    return TargetFix(
        lat=lat, lon=lon, pixel_xy=(320, 240), confidence=conf, radius_px=radius_px,
        camera=camera, ground_dist_m=ground, slant_range_m=slant, t_monotonic=t,
        marker_id=marker_id,
    )


def test_three_nadir_votes_over_span_confirm() -> None:
    tr = TargetTracker()
    tr.ingest(_fix(t=0.0))
    tr.ingest(_fix(t=0.3))
    assert not tr.confirmed_pending()          # only 2 votes
    tr.ingest(_fix(t=0.7))                      # 3 votes, span 0.7s >= 0.6
    pend = tr.confirmed_pending()
    assert len(pend) == 1
    assert pend[0].votes_nadir == 3
    assert pend[0].state is TargetState.CONFIRMED


def test_two_decoded_votes_confirm_at_confirm_votes_2() -> None:
    """2026-07-07 tuning: at sweep altitude a pad often decodes only 2x. With
    confirm_votes=2 those pads CONFIRM into the registry on the sortie-1 sweep
    (so later sorties fly direct, no re-sweep); the old default of 3 left them
    CANDIDATE -> registry=unknown -> a full re-sweep. The land gate re-verifies
    the id AT the pad, so a lower registry threshold is safe."""
    # default (3): two decoded votes do NOT confirm into the registry
    tr3 = TargetTracker()
    tr3.ingest(_fix(marker_id=4, t=0.0))
    tr3.ingest(_fix(marker_id=4, t=0.7))          # 2 id_votes, span 0.7 s
    assert tr3.confirmed_by_marker(4) is None
    # confirm_votes=2: the same two decoded votes CONFIRM
    tr2 = TargetTracker(confirm_votes=2)
    tr2.ingest(_fix(marker_id=4, t=0.0))
    tr2.ingest(_fix(marker_id=4, t=0.7))
    got = tr2.confirmed_by_marker(4)
    assert got is not None and got.marker_id == 4


def test_identified_unconfirmed_lists_only_short_id_clusters() -> None:
    """2026-07-08 structural fix: a pad decoded at least once but still short
    of confirm_votes has a KNOWN position — the mission tops its votes up with
    a cheap decode visit instead of the full re-sweep an unregistered
    assignment otherwise costs. The accessor lists identified CANDIDATEs
    (optionally filtered by marker id) and excludes CONFIRMED clusters and
    unidentified (cue-only) candidates."""
    tr = TargetTracker()                                # confirm_votes=3
    tr.ingest(_fix(marker_id=4, t=0.0))                 # 1 decoded vote
    tr.ingest(_fix(lat=_LAT + 0.0004, t=0.1))           # cue-only candidate
    for ts in (0.0, 0.4, 0.8):                          # marker 2 → CONFIRMED
        tr.ingest(_fix(lat=_LAT + 0.0008, marker_id=2, t=ts))
    got = tr.identified_unconfirmed()
    assert [c.marker_id for c in got] == [4]
    assert [c.marker_id for c in tr.identified_unconfirmed(4)] == [4]
    assert tr.identified_unconfirmed(2) == []           # confirmed — no top-up
    assert tr.identified_unconfirmed(6) == []           # never seen


def test_votes_without_time_span_do_not_confirm() -> None:
    """Three frames in a 0.1 s burst is not temporal consistency."""
    tr = TargetTracker(min_span_s=0.6)
    for t in (0.0, 0.05, 0.1):
        tr.ingest(_fix(t=t))
    assert not tr.confirmed_pending()


def test_non_nadir_camera_fix_is_ignored() -> None:
    """The nadir camera is the SOLE control authority (single-camera rig):
    a fix labelled with any other camera never enters the registry at all."""
    tr = TargetTracker()
    for t in (0.0, 0.3, 0.6, 0.9, 1.2):
        tr.ingest(_fix(camera="oblique", t=t))
    assert not tr.confirmed_pending()
    assert tr.snapshot() == []


def test_median_position_is_robust_to_an_outlier() -> None:
    tr = TargetTracker(cluster_radius_m=8.0)
    # Three tight fixes + one 5 m outlier (still same cluster); median ≈ tight set.
    tr.ingest(_fix(lat=_LAT, t=0.0))
    tr.ingest(_fix(lat=_LAT, t=0.3))
    tr.ingest(_fix(lat=_LAT + 4.5e-5, t=0.6))   # ~5 m north outlier
    tr.ingest(_fix(lat=_LAT, t=0.9))
    tgt = tr.confirmed_pending()[0]
    assert abs(tgt.lat - _LAT) < 1e-7           # median pinned to the tight set


def test_low_confidence_is_gated_out() -> None:
    tr = TargetTracker(min_confidence=0.5)
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(conf=0.45, t=t))
    assert not tr.snapshot()                     # nothing even tracked


def test_wrong_radius_is_gated_out() -> None:
    tr = TargetTracker()
    # expected marker half-side @16 m ≈ 3.4 px; 40 px is way outside (0.4, 2.5).
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(radius_px=40.0, t=t))
    assert not tr.snapshot()


def test_far_ground_distance_is_gated_out() -> None:
    tr = TargetTracker(max_fix_ground_dist_m=50.0)
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(ground=120.0, t=t))       # banked-frame far-projection junk
    assert not tr.snapshot()


def test_serve_state_machine() -> None:
    tr = TargetTracker()
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(t=t))
    tid = tr.confirmed_pending()[0].target_id

    claimed = tr.claim(tid)
    assert claimed is not None and claimed.state is TargetState.SERVING
    assert claimed.attempts == 1
    assert not tr.confirmed_pending()            # no longer pending while serving

    tr.defer(tid)                                # attempt failed → back to pending
    again = tr.confirmed_pending()
    assert len(again) == 1 and again[0].target_id == tid

    reclaimed = tr.claim(tid)
    assert reclaimed is not None and reclaimed.attempts == 2
    tr.mark_served(tid)
    assert tr.claim(tid) is None                 # served can't be reclaimed
    assert any(t.state is TargetState.SERVED for t in tr.snapshot())


def test_served_duplicate_is_suppressed() -> None:
    """A second confirmed cluster within serve_dedupe_m of a SERVED target is
    not dropped on again."""
    tr = TargetTracker(cluster_radius_m=8.0, serve_dedupe_m=12.0)
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(lat=_LAT, t=t))
    a = tr.confirmed_pending()[0].target_id
    assert tr.claim(a) is not None
    tr.mark_served(a)
    # A separate cluster ~10 m away (outside cluster radius, inside dedupe).
    for t in (1.0, 1.3, 1.7):
        tr.ingest(_fix(lat=_LAT + 9.0e-5, t=t))  # ~10 m north
    b = [t for t in tr.confirmed_pending()][0].target_id
    assert b != a
    assert tr.claim(b) is None                   # suppressed as a duplicate


def test_concurrent_ingest_is_consistent() -> None:
    """Many threads ingesting distinct targets → every target confirmed, no loss
    or corruption under the lock."""
    tr = TargetTracker()
    n_targets = 8

    def feed(i: int) -> None:
        lat = _LAT + i * 1.0e-3              # ~111 m apart → distinct clusters
        for k in range(5):
            tr.ingest(_fix(lat=lat, t=0.2 * k))

    threads = [threading.Thread(target=feed, args=(i,)) for i in range(n_targets)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()

    assert len(tr.confirmed_pending()) == n_targets
    assert len(tr.snapshot()) == n_targets


# ── V1.3 marker-id keying ──

def test_same_id_merges_at_any_distance_different_ids_never_merge() -> None:
    tr = TargetTracker()
    tr.ingest(_fix(marker_id=3, t=0.0))
    # Same id 30 m away (projection scatter/GPS drift) → SAME cluster.
    tr.ingest(_fix(lat=_LAT + 2.7e-4, marker_id=3, t=0.3))
    # A different id right on top of the first → its OWN cluster.
    tr.ingest(_fix(marker_id=5, t=0.4))
    snap = tr.snapshot()
    assert len(snap) == 2
    by_marker = {s.marker_id: s for s in snap}
    assert by_marker[3].votes_nadir == 2 and by_marker[5].votes_nadir == 1


def test_identified_cluster_needs_decoded_votes_to_confirm() -> None:
    tr = TargetTracker()
    tr.ingest(_fix(marker_id=2, t=0.0))
    # Three more POSITIONAL (undecoded) fixes on the same pad: plenty of nadir
    # points, but only one decoded vote — must NOT confirm as id 2 yet.
    for t in (0.3, 0.7, 1.0):
        tr.ingest(_fix(t=t))
    assert tr.confirmed_by_marker(2) is None
    tr.ingest(_fix(marker_id=2, t=1.3))
    tr.ingest(_fix(marker_id=2, t=1.6))          # 3 decoded votes, span ok
    got = tr.confirmed_by_marker(2)
    assert got is not None and got.marker_id == 2
    assert got.votes_nadir == 6                   # position used every fix


def test_cue_cluster_upgrades_on_first_decode() -> None:
    tr = TargetTracker()
    for t in (0.0, 0.3):
        tr.ingest(_fix(t=t))                      # unidentified white-pad cues
    cands = tr.unidentified_candidates(min_votes=2)
    assert len(cands) == 1 and cands[0].marker_id is None
    for t in (0.7, 1.0, 1.3):
        tr.ingest(_fix(marker_id=4, t=t))         # decoded on the revisit
    assert tr.unidentified_candidates(min_votes=1) == []
    got = tr.confirmed_by_marker(4)
    assert got is not None and got.votes_nadir == 5


def test_claim_by_marker_and_served_reclaim() -> None:
    tr = TargetTracker()
    for t in (0.0, 0.3, 0.7):
        tr.ingest(_fix(marker_id=6, t=t))
    assert tr.distinct_confirmed_ids() == {6}

    first = tr.claim_by_marker(6)
    assert first is not None and first.state is TargetState.SERVING
    tr.mark_served(first.target_id)
    assert tr.confirmed_by_marker(6) is not None  # registry keeps SERVED pads

    again = tr.claim_by_marker(6)                 # committee re-assigns the pad
    assert again is not None and again.attempts == 2
    assert tr.claim_by_marker(1) is None          # unknown id → nothing to claim
