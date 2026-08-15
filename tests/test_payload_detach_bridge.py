"""Unit tests for the gz-free surface of sitl/payload_detach_bridge.py.

parse_release(), _iter_new_releases() and _dedupe() are the bridge's
gz-free, importable surface — the gz publish side itself (Empty on
/model/<model>/detach_payload_<N>) needs a running Gazebo and is not
exercised here; see .superpowers/sdd/task-11-report.md for the manual/
import-smoke validation that stood in for it that session. _run() bundles
those three with the gz import/Node()/publish() calls, so its own behaviour
is only covered indirectly (via the pieces above) plus the one
gz-unavailable smoke test below.

Every RELEASE/START/END/FLIGHT line below is transcribed verbatim from the
real emitters (not guessed): orchestrator/tactical_align.py:557-560 (RELEASE),
orchestrator/mission.py (DELIVERY START/END, DELIVERY abort, FLIGHT START).
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from sitl.payload_detach_bridge import (
    _dedupe,
    _iter_new_releases,
    _run,
    box_for_payload,
    parse_release,
)


def test_parse_release_extracts_delivery_and_payload() -> None:
    """orchestrator/tactical_align.py:557-560's actual f-string output."""
    line = ("t=3.2s DELIVERY 3 RELEASE pad=4 payload=2 "
            "lat=13.7307000 lon=100.7880000")
    assert parse_release(line) == (3, 2)


def test_parse_release_handles_pad_none() -> None:
    """AlignParams.assigned_marker_id: int | None (tactical_align.py:136) can
    genuinely be None at a RELEASE (id-unverified touchdown, audited) — the
    pad token must not be required to be digits."""
    line = ("t=12.0s DELIVERY 1 RELEASE pad=None payload=0 "
            "lat=13.7307000 lon=100.7880000")
    assert parse_release(line) == (1, 0)


def test_parse_release_ignores_delivery_start() -> None:
    """DELIVERY ... START (mission.py:432-434) also carries payload= — the
    discriminator must be the literal RELEASE keyword, not payload='s presence."""
    line = "t=3.1s DELIVERY 3 START pad=4 payload=2 stop_index=2"
    assert parse_release(line) is None


def test_parse_release_ignores_delivery_end_variants() -> None:
    """mission.py:459-462 (delivered=True) and :474-476 (delivered=False)."""
    assert parse_release(
        "t=3.4s DELIVERY 3 END delivered=True pad=4 err=0.19m landed=True"
    ) is None
    assert parse_release(
        "t=3.4s DELIVERY 3 END delivered=False pad=4 notes=align timed out"
    ) is None


def test_parse_release_ignores_delivery_abort() -> None:
    """mission.py:624-629 — 'DELIVERY abort:' has no numeric delivery index
    right after DELIVERY, so it must not match \\d+."""
    line = ("t=500.0s DELIVERY abort: flight 2 skipping remaining ids=4,6 "
            "(remaining=30s batt=41%) — returning with the egg(s)")
    assert parse_release(line) is None


def test_parse_release_ignores_flight_start() -> None:
    """mission.py:535-538."""
    line = "t=1.0s FLIGHT 1 START eggs=4 ids=3,1,4,6 remaining=1200s"
    assert parse_release(line) is None


def test_parse_release_ignores_other_audit_lines() -> None:
    assert parse_release(
        "t=5.0s TELEM phase=search lat=13.73 lon=100.78 alt=15.0 armed=True"
    ) is None
    assert parse_release("t=2.0s TRANSIT_PASS P1 ingress sortie=1") is None
    assert parse_release("") is None


def test_parse_release_matches_inside_the_on_disk_jsonl_wrapper() -> None:
    """orchestrator/audit.py:AuditLog.record wraps every entry as a JSON row
    ({"ts": ..., "entry": "<line>"}) before appending to audit.jsonl — that
    row, not the bare f-string, is what the bridge's tailer reads line by
    line in production. parse_release finds the pattern by substring search
    either way (the entry text needs no JSON escaping)."""
    row = json.dumps({
        "ts": "2026-07-24T00:00:00+00:00",
        "entry": ("t=198.5s DELIVERY 2 RELEASE pad=6 payload=1 "
                   "lat=13.7308657 lon=100.7894120"),
    })
    assert parse_release(row) == (2, 1)


def test_run_degrades_gracefully_without_gz_transport(capsys) -> None:
    """gz-transport is an apt-only package the project .venv deliberately
    does not install (see the module docstring) — so calling _run() from
    under this project's pytest genuinely exercises the ImportError branch,
    no mocking needed, and proves it returns 0 *before* ever constructing a
    live gz Node() (the import is the first thing _run() does; Node() is
    only reached after it succeeds). Safe to run with no simulator present."""
    rc = _run(Path("/nonexistent/audit.jsonl"), "eft_x6100", 0.2)

    assert rc == 0
    out = capsys.readouterr().out
    assert "gz-transport unavailable" in out
    assert "bridge disabled" in out


# ── _iter_new_releases: the pure file-tailer extracted from _run (Task 12) ──
#
# _run bundled the gz import + Node()/publish() calls with the plain
# file-tailing loop, so the two guarantees that matter most — EOF-seek
# before the first read, and the live tail actually picking up new lines —
# had no automated coverage and could not get any (gz is not in .venv by
# design). Extracting the tailer as a gz-free generator fixes that.


def _release_line(delivery_k: int, payload: int, t: float = 1.0) -> str:
    return (f"t={t}s DELIVERY {delivery_k} RELEASE pad=3 payload={payload} "
            "lat=13.7307000 lon=100.7880000\n")


def test_iter_new_releases_skips_pre_existing_lines(tmp_path: Path) -> None:
    """EOF seek: RELEASE lines already in the file when the generator starts
    must never be replayed — a re-used mission_id's audit.jsonl genuinely
    appends across runs (AuditLog.record, orchestrator/audit.py), so a
    tailer that started at offset 0 would re-shed every past mission's cargo
    boxes onto whatever is attached to the aircraft right now."""
    audit = tmp_path / "audit.jsonl"
    audit.write_text(_release_line(1, 0) + _release_line(2, 1))

    out = list(_iter_new_releases(audit, poll_s=0.01, max_idle_polls=20))

    assert out == []


def test_iter_new_releases_yields_newly_appended_lines(tmp_path: Path) -> None:
    """A line appended AFTER the generator starts tailing IS yielded — with
    a repeated payload among the appends to show the raw stream is NOT
    deduped here (that is _dedupe's job, tested below) — while a
    pre-existing line in the SAME file still is not, in the same run."""
    audit = tmp_path / "audit.jsonl"
    audit.write_text(_release_line(1, 0))   # pre-existing — must be skipped
    seen: list[tuple[int, int]] = []

    def _collect() -> None:
        for item in _iter_new_releases(audit, poll_s=0.01, max_idle_polls=50):
            seen.append(item)

    thread = threading.Thread(target=_collect, daemon=True)
    thread.start()
    time.sleep(0.1)   # generous vs poll_s=0.01: let the tailer reach EOF
    with audit.open("a") as fh:
        fh.write(_release_line(2, 1))
        fh.write(_release_line(2, 1))   # duplicate payload, raw/undeduped
        fh.write(_release_line(3, 2))
    # daemon=True + this join timeout mean a broken idle budget fails THIS
    # test (the is_alive assert below) rather than hanging the suite.
    thread.join(timeout=5.0)

    assert not thread.is_alive(), "generator outlived its idle budget"
    assert seen == [(2, 1), (2, 1), (3, 2)]


def test_iter_new_releases_gives_up_within_its_idle_budget(tmp_path: Path) -> None:
    """The safety valve the two tests above rely on, checked directly: with
    the file never even created, the generator must return on its own
    within a bounded time — never block the caller forever. This is what
    makes "a test must not hang" structurally true rather than a promise."""
    audit = tmp_path / "never_created.jsonl"
    poll_s, budget = 0.01, 10
    t0 = time.monotonic()

    out = list(_iter_new_releases(audit, poll_s=poll_s, max_idle_polls=budget))

    elapsed = time.monotonic() - t0
    assert out == []
    # Generous multiplier: robust on a loaded CI box while still bounding
    # this far below anything resembling "hang" (expected ~budget*poll_s).
    assert elapsed < poll_s * budget * 20


# ── _dedupe: the idempotence guard _run applies over the raw tail stream ──


def test_dedupe_publishes_each_payload_once() -> None:
    """Mirrors _run's fired-set bookkeeping without needing gz: a payload
    index seen twice (idempotence-ledger bug, a replayed line, or simply the
    raw duplicate the tailer itself does not filter — previous test) must
    come out of the dedup stage exactly once."""
    releases = [(1, 2), (2, 2), (3, 0)]   # payload 2 "released" twice
    fired: set[int] = set()

    out = list(_dedupe(releases, known_boxes={0, 1, 2, 3}, fired=fired))

    # No channel map → payload index IS the box index (a rack wired in
    # delivery order); yields (delivery_k, payload, box).
    assert out == [(1, 2, 2), (3, 0, 0)]
    assert fired == {0, 2}


def test_dedupe_ignores_a_payload_with_no_publisher(capsys) -> None:
    """A payload index outside the configured release count (a
    drop_payload_count/eggs_aboard mismatch — see mission.py's own
    FLIGHT n CONFIG WARN guard) is skipped, loudly, never raised."""
    fired: set[int] = set()

    out = list(_dedupe([(1, 9)], known_boxes={0, 1, 2, 3}, fired=fired))

    assert out == []
    assert fired == set()
    assert "no matching cargo_payload model" in capsys.readouterr().out


# ── the as-wired payload_id -> AUX map (2026-08-15) ──────────────────────────


def test_box_for_payload_follows_the_channel_map() -> None:
    """KMUTNB's rack is NOT wired in delivery order: payload 0..3 sit on AUX
    4/1/2/3 (front-left, rear-right, front-right, rear-left). The gz cargo box
    is indexed by AUX pin - 1 (SIM_GZ_SV_FUNCn -> servo_<n-1> ->
    cargo_payload_<n-1>), so the FIRST egg released sheds box 3, not box 0.
    Getting this wrong drops a visibly wrong corner in SITL while the real
    aircraft drops the right one — i.e. it silently breaks sim fidelity."""
    chans = (4, 1, 2, 3)
    assert [box_for_payload(p, chans) for p in range(4)] == [3, 0, 1, 2]
    # No map (or a payload past its end) = the historical identity mapping.
    assert [box_for_payload(p, ()) for p in range(4)] == [0, 1, 2, 3]
    assert box_for_payload(9, chans) == 9


def test_dedupe_maps_payloads_through_the_channel_map() -> None:
    """The idempotence guard keys on the BOX, not the payload — so the audit
    path and the servo path (which sees per-AUX topics) de-dupe against each
    other even on the as-wired rack."""
    fired: set[int] = set()

    out = list(_dedupe([(1, 0), (2, 1), (3, 0)], known_boxes={0, 1, 2, 3},
                       fired=fired, channels=(4, 1, 2, 3)))

    assert out == [(1, 0, 3), (2, 1, 0)]      # payload 0 -> box 3, payload 1 -> box 0
    assert fired == {0, 3}


def test_makefile_channel_map_matches_the_shipped_config() -> None:
    """The bridge's audit path gets the map from the command line (Makefile
    CHANNELS), the flight core gets it from sitl/aavc_config.yaml. Two copies
    of one wiring fact — pin them together so a rewire updates both."""
    import re as _re

    import yaml

    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root / "sitl" / "aavc_config.yaml").read_text())
    m = _re.search(r"^CHANNELS \?= *(\S+)", (root / "Makefile").read_text(),
                   _re.MULTILINE)
    assert m, "Makefile lost its CHANNELS default"
    assert [int(c) for c in m.group(1).split(",")] == \
        cfg["connection"]["drop_servo_channels"]
