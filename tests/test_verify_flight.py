"""Post-flight verifier (tools/verify_flight.py) — audit-grammar + fail-closed.

Two things are pinned here.

1. **Transit-altitude corridor classification.** The mission deliberately climbs
   the last ~2 m to the transit altitude EN-ROUTE (two-stage climb, mission.py:
   a full-rate climb straight to 19.5 m overshoots through the 20 m ceiling), so
   a transit segment can legitimately end below the 18.8 m corridor while still
   climbing — a 1 Hz-sampling artifact of a documented, rules-compliant profile.
   That is a WARN. A flat below-band hold, or a flight that never demonstrates
   the corridor at all, stays a FAIL: the verifier fails CLOSED (review
   2026-07-04) and this file pins that too.

2. **The FLIGHT ⊃ DELIVERY audit grammar** (2026-07-24 briefing). Every fixture
   below is written in the EXACT shapes ``orchestrator/mission.py`` and
   ``orchestrator/tactical_align.py::_drop_once`` emit — a FLIGHT is one
   arm→disarm cycle, a DELIVERY is one pad served inside it, and a delivery's
   release position is matched to it by the DELIVERY index in the line (never
   by a stop_index/sortie offset inference).
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import yaml

_PATH = Path(__file__).resolve().parents[1] / "tools" / "verify_flight.py"
_spec = importlib.util.spec_from_file_location("verify_flight", _PATH)
assert _spec is not None and _spec.loader is not None
vf = importlib.util.module_from_spec(_spec)
# dataclasses resolve `from __future__ import annotations` strings through
# sys.modules[cls.__module__] — register before exec or the @dataclass breaks.
sys.modules["verify_flight"] = vf
_spec.loader.exec_module(vf)

_CFG = {"mission": {"altitude_ceiling_m": 20.0, "transit_alt_m": 20.0,
                    "search_floor_m": 10.0}}


def _telem(t: float, alt: float, phase: str = "transit_ingress",
           batt: float = 81.0, vbat: float = 24.58, mode: str = "HOLD") -> str:
    # Mirrors mission.py::_telemetry_sampler's CURRENT shape (batt/vbat joined
    # the grammar 2026-08-20; mode joined 2026-08-21, G7 zombie debrief).
    # test_telem_legacy_format_still_parses covers the older archives.
    return (f"t={t:.1f}s TELEM phase={phase} flight=1 lat=13.7303000 "
            f"lon=100.7874000 alt={alt:.2f} armed=1 "
            f"batt={batt:.1f} vbat={vbat:.2f} mode={mode}")


def _run(entries: list[str]):
    return vf.verify(entries, [], _CFG, land_acc_m=0.5, window_s=1200.0)


def _transit_alt_fails(rep) -> list[str]:
    return [f for f in rep.fails if "transit altitude" in f]


def test_climbing_nocapture_segment_warns_not_fails() -> None:
    """2026-07-08: a leg that ends mid-climb below the band (all samples
    < 18.8 m, monotonically closing) is the two-stage climb closure — WARN,
    not FAIL, when another segment demonstrates the corridor. This was the
    intermittent FAIL that hit healthy 4/4 runs (I2a overnight)."""
    entries = [_telem(t, 19.5) for t in range(100, 108)]      # captured leg
    entries.append(_telem(110.0, 12.0, phase="search"))       # segment break
    entries += [_telem(t, alt, phase="transit_egress")        # ends mid-climb
                for t, alt in zip(range(200, 205),
                                  (15.2, 16.1, 17.0, 17.8, 18.5))]
    rep = _run(entries)
    assert _transit_alt_fails(rep) == []
    assert any("climb closure" in w for w in rep.warns)


def test_flat_below_band_nocapture_segment_still_fails() -> None:
    """A sustained FLAT hold below the corridor is a real violation — the
    climb-closure carve-out must not swallow it."""
    entries = [_telem(t, 19.5) for t in range(100, 108)]
    entries.append(_telem(110.0, 12.0, phase="search"))
    entries += [_telem(t, 17.0, phase="transit_egress") for t in range(200, 207)]
    rep = _run(entries)
    assert _transit_alt_fails(rep)


def test_corridor_never_captured_anywhere_still_fails() -> None:
    """If NO transit segment ever reaches the corridor the flight never held
    it — that is not a sampling artifact, climbing or not. Fail closed."""
    entries = [_telem(t, alt)
               for t, alt in zip(range(100, 105),
                                 (15.0, 15.8, 16.6, 17.4, 18.2))]
    rep = _run(entries)
    assert _transit_alt_fails(rep)


# ── FLIGHT ⊃ DELIVERY grammar ───────────────────────────────────────────────
# Fixtures below are the real emitted lines (orchestrator/mission.py +
# tactical_align.py::_drop_once). Keep them byte-faithful to the f-strings.

_TRUTH = [{"marker_id": 3, "lat": 13.7307, "lon": 100.7880},
          {"marker_id": 1, "lat": 13.7306, "lon": 100.7883}]
_MCFG = {"mission": {"altitude_ceiling_m": 20, "transit_alt_m": 20,
                     "search_floor_m": 10,
                     "landing_accuracy_threshold_m": 0.5},
         "ground_operation": {"launch_recovery": [13.73025, 100.7873],
                              "launch_recovery_zone_radius_m": 25.0},
         "controlled_airspace": []}

_REL_1 = ("t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 "
          "lat=13.7307000 lon=100.7880000")
_REL_2 = ("t=4.2s DELIVERY 2 RELEASE pad=1 payload=1 "
          "lat=13.7306000 lon=100.7883000")


def _one_flight_two_deliveries() -> list[str]:
    """One FLIGHT (arm→disarm) carrying two eggs, both delivered."""
    lines = ["t=1.0s FLIGHT 1 START eggs=2 ids=3,1 remaining=1200s"]
    for n, d in [("1", "ingress"), ("2", "ingress"), ("3", "ingress"),
                 ("3", "egress"), ("2", "egress"), ("1", "egress")]:
        lines.append(f"t=2.0s TRANSIT_PASS P{n} {d} flight=1 d=1.0m")
    lines += [
        "t=3.0s TELEM phase=drop flight=1 lat=13.7307000 lon=100.7880000 "
        "alt=0.20 armed=1",
        "t=3.1s DELIVERY 1 START pad=3 payload=0 stop_index=0",
        _REL_1,
        "t=3.3s DELIVERY 1 END delivered=True pad=3 err=0.10m landed=True",
        "t=4.0s TELEM phase=drop flight=1 lat=13.7306000 lon=100.7883000 "
        "alt=0.20 armed=1",
        "t=4.1s DELIVERY 2 START pad=1 payload=1 stop_index=1",
        _REL_2,
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True",
        "t=5.0s TELEM phase=land flight=1 lat=13.7302500 lon=100.7873000 "
        "alt=0.10 armed=0",
        "t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m remaining=900s",
        "t=5.2s FLIGHT 1 ENERGY 1800mAh total=1800mAh",
        # Deliberately NO TELEM after this — the realistic shape at the
        # shipping eggs_aboard=4 default (ONE flight): there is no next
        # flight's preflight hold to hand the disarm-after-L&R check a bonus
        # sample, and mission.py has no await between writing this END line
        # and the loop's own `finally: sampler.cancel()`. A trailing sample
        # here used to paper over that (I4, review 2026-07-24) — the fixed
        # verifier falls back to the last sample AT/BEFORE this END line
        # instead, which is the "t=5.0s ... armed=0" line above.
    ]
    return lines


def _verify(lines: list[str], truth=_TRUTH, cfg=_MCFG):
    return vf.verify(lines, truth, cfg, land_acc_m=0.5, window_s=1200.0)


def test_verify_passes_single_flight_two_deliveries() -> None:
    """The shape a clean G4′ run emits must verify with ZERO fails."""
    rep = _verify(_one_flight_two_deliveries())
    assert rep.fails == [], rep.fails


def test_verify_fails_release_off_pad() -> None:
    """The central terminal check: the egg must leave ON the assigned pad."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(_REL_1)] = ("t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 "
                                  "lat=13.7320000 lon=100.7880000")
    rep = _verify(lines)
    assert any("from truth pad 3" in f for f in rep.fails), rep.fails


def test_release_matched_by_delivery_index_not_by_offset() -> None:
    """Delivery k is scored against delivery k's RELEASE line — never against
    a neighbour's. The retired `stop_index == sortie-1` inference paired
    delivery i with delivery i+1's release, which would blame the wrong pad
    (and let a genuinely off-pad release score against the wrong truth)."""
    lines = _one_flight_two_deliveries()
    # Move ONLY delivery 2's release ~33 m off its pad; delivery 1 is untouched.
    lines[lines.index(_REL_2)] = ("t=4.2s DELIVERY 2 RELEASE pad=1 payload=1 "
                                  "lat=13.7309000 lon=100.7883000")
    rep = _verify(lines)
    assert any("delivery 2" in f and "from truth pad 1" in f
               for f in rep.fails), rep.fails
    assert not any("delivery 1" in f for f in rep.fails), rep.fails


def test_release_without_touchdown_fails() -> None:
    """landed=False on the delivered END = the egg left while airborne."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(
        "t=3.3s DELIVERY 1 END delivered=True pad=3 err=0.10m landed=True"
    )] = "t=3.3s DELIVERY 1 END delivered=True pad=3 err=0.10m landed=False"
    rep = _verify(lines)
    assert any("WITHOUT confirmed touchdown" in f for f in rep.fails), rep.fails


def test_release_with_no_truth_fails_closed() -> None:
    """A release scored against nothing is a silent pass — fail closed."""
    rep = _verify(_one_flight_two_deliveries(), truth=[])
    assert any("no ground truth" in f for f in rep.fails), rep.fails


def test_undelivered_delivery_warns_but_does_not_fail() -> None:
    """`reason=not_found` (and its flight{n}_pad{id}_not_found anomaly) is the
    fail-closed design WORKING — the egg came home rather than being released
    blind. A scoring loss, not a rules violation: WARN, never FAIL."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True"
    )] = "t=4.3s DELIVERY 2 END delivered=False pad=1 reason=not_found"
    lines.remove(_REL_2)
    lines[lines.index("t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m "
                      "remaining=900s")] = (
        "t=5.1s FLIGHT 1 END delivered=1/2 d_home=1.0m remaining=900s")
    lines.insert(0, "t=4.25s flight1_pad1_not_found")
    rep = _verify(lines)
    assert rep.fails == [], rep.fails
    assert any("not_found" in w for w in rep.warns), rep.warns
    assert any("1 of 2" in w for w in rep.warns), rep.warns


def test_deferred_delivery_end_with_notes_parses() -> None:
    """The other delivered=False shape (`notes=…`, free text joined from
    _serve's res.notes) must parse too — an unparsed END would silently vanish
    from the coverage sum. The note below is a REAL one and itself contains
    `err=`, the token the delivered=True shape uses: the optional groups must
    not be fooled by it."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True"
    )] = ("t=4.3s DELIVERY 2 END delivered=False pad=1 "
          "notes=acquired conf=0.91; not-centred (err=2.46 m) → defer")
    lines.remove(_REL_2)
    lines[lines.index("t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m "
                      "remaining=900s")] = (
        "t=5.1s FLIGHT 1 END delivered=1/2 d_home=1.0m remaining=900s")
    rep = _verify(lines)
    assert rep.fails == [], rep.fails
    assert any("1 of 2" in w for w in rep.warns), rep.warns


def test_transit_checked_per_flight() -> None:
    """Two flights: the second one missing its egress P1 must fail on ITS OWN
    sequence — the per-FLIGHT order check is not pooled across flights."""
    lines = _one_flight_two_deliveries()
    lines += ["t=7.0s FLIGHT 2 START eggs=1 ids=4 remaining=900s"]
    for n, d in [("1", "ingress"), ("2", "ingress"), ("3", "ingress"),
                 ("3", "egress"), ("2", "egress")]:
        lines.append(f"t=8.0s TRANSIT_PASS P{n} {d} flight=2 d=1.0m")
    lines += [
        "t=9.0s TELEM phase=drop flight=2 lat=13.7307000 lon=100.7880000 "
        "alt=0.20 armed=1",
        "t=9.1s DELIVERY 3 START pad=3 payload=0 stop_index=2",
        "t=9.2s DELIVERY 3 RELEASE pad=3 payload=0 "
        "lat=13.7307000 lon=100.7880000",
        "t=9.3s DELIVERY 3 END delivered=True pad=3 err=0.11m landed=True",
        "t=10.0s TELEM phase=land flight=2 lat=13.7302500 lon=100.7873000 "
        "alt=0.10 armed=0",
        "t=10.1s FLIGHT 2 END delivered=1/1 d_home=1.2m remaining=600s",
        "t=11.0s TELEM phase=preflight flight=2 lat=13.7302500 "
        "lon=100.7873000 alt=0.05 armed=0",
    ]
    rep = _verify(lines)
    assert any("flight 2" in f and "transit pass order" in f
               for f in rep.fails), rep.fails
    assert not any("flight 1" in f for f in rep.fails), rep.fails


def test_transit_miss_fails_the_flight() -> None:
    lines = _one_flight_two_deliveries()
    lines[lines.index("t=2.0s TRANSIT_PASS P2 egress flight=1 d=1.0m")] = (
        "t=2.0s TRANSIT_MISS P2 egress flight=1 d=41.0m")
    rep = _verify(lines)
    assert any("transit MISS" in f for f in rep.fails), rep.fails


def test_flight_end_far_from_lr_fails() -> None:
    lines = _one_flight_two_deliveries()
    lines[lines.index("t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m "
                      "remaining=900s")] = (
        "t=5.1s FLIGHT 1 END delivered=2/2 d_home=90.0m remaining=900s")
    rep = _verify(lines)
    assert any("from the L&R point" in f for f in rep.fails), rep.fails


def test_self_reported_d_home_cannot_certify_a_wrong_home() -> None:
    """The 2026-07-22 trap: PX4 re-captures home at every arm, so a healthy
    d_home can sit next to a fix 100+ m from the CONFIGURED L&R. Cross-check
    against the config, not the flight code's own number."""
    lines = _one_flight_two_deliveries()
    lines[lines.index("t=5.0s TELEM phase=land flight=1 lat=13.7302500 "
                      "lon=100.7873000 alt=0.10 armed=0")] = (
        "t=5.0s TELEM phase=land flight=1 lat=13.7312500 lon=100.7873000 "
        "alt=0.10 armed=0")
    rep = _verify(lines)
    assert any("from the configured L&R" in f for f in rep.fails), rep.fails


def test_still_armed_after_flight_end_fails() -> None:
    """Resupply crew approaches between flights — the aircraft MUST disarm.

    I4 (review 2026-07-24): this is the LAST flight, so there is no TELEM
    after its END line at all — `after` is empty and the check must fall
    back to the last sample AT/BEFORE END. That sample reading armed=1 must
    still fail the check; an empty `after` must never silently pass."""
    lines = _one_flight_two_deliveries()
    lines[lines.index("t=5.0s TELEM phase=land flight=1 lat=13.7302500 "
                      "lon=100.7873000 alt=0.10 armed=0")] = (
        "t=5.0s TELEM phase=land flight=1 lat=13.7302500 "
        "lon=100.7873000 alt=0.10 armed=1")
    rep = _verify(lines)
    assert any("never observed DISARMED" in f for f in rep.fails), rep.fails


def test_disarm_confirmed_by_fallback_when_nothing_follows_flight_end() -> None:
    """I4 companion (PASS case): the single-flight shape — literally nothing
    in the trail after FLIGHT 1 END, exactly what mission.py now emits at the
    shipping eggs_aboard=4 default. An empty `after` must not be treated as
    an automatic pass OR an automatic fail — it must fall back to the last
    sample AT/BEFORE END (armed=0 here) and PASS on real evidence."""
    lines = _one_flight_two_deliveries()
    assert lines[-1] == "t=5.2s FLIGHT 1 ENERGY 1800mAh total=1800mAh"
    rep = _verify(lines)
    assert rep.fails == [], rep.fails


def test_flight_with_no_telem_of_its_own_fails_closed() -> None:
    """I4 (review 2026-07-24): fail-closed doctrine (CLAUDE.md §8) — when
    there is NO usable sample at all for a flight (no `after`, and no
    same-flight sample in `before` either — the fallback is scoped to THIS
    flight's own `flight=` tag so a PRECEDING flight's stale sample can't
    silently certify a later flight's disarm), that must FAIL, never a
    silent pass. Flight 2 here contributes zero TELEM of its own and is the
    last flight, so both `after` and its own `before` are empty."""
    lines = _one_flight_two_deliveries() + [
        "t=7.0s FLIGHT 2 START eggs=1 ids=4 remaining=900s",
        "t=7.1s FLIGHT 2 END delivered=0/1 d_home=1.0m remaining=900s",
    ]
    rep = _verify(lines)
    assert any("flight 2" in f and "never observed DISARMED" in f
               for f in rep.fails), rep.fails
    # Flight 1's own check must be unaffected (it has its own armed=0 sample).
    assert not any("flight 1" in f and "never observed DISARMED" in f
                   for f in rep.fails), rep.fails


def test_pre_go_telem_dropped_by_file_position() -> None:
    """state.start_window() RESETS the audit clock at the first GO, so pre-GO
    samples carry a LARGER t than the mission that follows. Anchor on file
    POSITION (first FLIGHT … START), never on time."""
    lines = ["t=1310.0s TELEM phase=preflight flight=0 lat=13.7302500 "
             "lon=100.7873000 alt=0.00 armed=0"] + _one_flight_two_deliveries()
    rep = _verify(lines)
    assert rep.fails == [], rep.fails


def test_nan_position_samples_warn_and_are_excluded() -> None:
    """A GPS dropout must not silently vanish from the geometry checks.

    Inserted BEFORE the "t=5.0s ... armed=0" sample (not after FLIGHT END):
    its own t=4.5s keeps it chronologically inside the flight, so it can't
    also become the disarm check's fallback sample (I4) — this fixture is
    about the NaN-position warning, not about disarm evidence.
    """
    lines = _one_flight_two_deliveries()
    lines.insert(
        lines.index("t=5.0s TELEM phase=land flight=1 lat=13.7302500 "
                    "lon=100.7873000 alt=0.10 armed=0"),
        "t=4.5s TELEM phase=land flight=1 lat=nan lon=nan alt=nan armed=1")
    rep = _verify(lines)
    assert any("NaN position/alt" in w for w in rep.warns), rep.warns


def test_release_without_a_delivery_end_fails_closed() -> None:
    """An egg that left the aircraft with no DELIVERY END behind it was never
    scored for touchdown or accuracy — that is unverified, so it fails."""
    lines = _one_flight_two_deliveries()
    lines.remove("t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m "
                 "landed=True")
    rep = _verify(lines)
    assert any("no DELIVERY 2 END" in f for f in rep.fails), rep.fails


# ── FIX 1 (2026-07-24 review): a recoverable land-gate defer must not FAIL an
# otherwise fully-delivered flight ──────────────────────────────────────────
# The id-verified LAND gate records `land_gate_id_not_confirmed` the moment an
# approach's id votes fall short and it climbs off to defer — BEFORE `_serve`
# (mission.py) decides whether to retry. The retry (one per delivery) often
# recovers it. Treating the anomaly as unconditionally fatal FAILed a run that
# delivered everything and merely deferred one approach along the way — a
# false failure, about to matter for a scored validation flight.

def test_land_gate_defer_warns_when_every_started_delivery_succeeded() -> None:
    """Both deliveries still show delivered=True — the defer is evidence the
    gate + retry did their job, so it must WARN, not FAIL. (This is the test
    that is RED against the pre-fix code: the anomaly was unconditionally
    fatal, so `rep.fails` used to be non-empty here.)"""
    lines = _one_flight_two_deliveries()
    lines.insert(0, "t=3.05s land_gate_id_not_confirmed")
    rep = _verify(lines)
    assert rep.fails == [], rep.fails
    assert any("land_gate_id_not_confirmed" in w and "retry delivered" in w
               for w in rep.warns), rep.warns


def test_land_gate_defer_still_fails_when_a_delivery_was_lost() -> None:
    """Companion case (must hold BEFORE and AFTER the fix): delivery 2 ends
    delivered=False, so the same defer anomaly may be why the egg came home —
    it must stay FATAL, exactly as before FIX 1."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True"
    )] = "t=4.3s DELIVERY 2 END delivered=False pad=1 reason=not_found"
    lines.remove(_REL_2)
    lines[lines.index("t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m "
                      "remaining=900s")] = (
        "t=5.1s FLIGHT 1 END delivered=1/2 d_home=1.0m remaining=900s")
    lines.insert(0, "t=3.05s land_gate_id_not_confirmed")
    rep = _verify(lines)
    assert any("land_gate_id_not_confirmed" in f for f in rep.fails), rep.fails


def test_land_gate_defer_with_no_delivery_evidence_stays_fatal() -> None:
    """A defer anomaly with NO DELIVERY END lines at all (flight killed before
    any delivery finished) has nothing to correlate against — fail closed
    rather than assume an unrecorded recovery. `phase="search"` keeps this
    fixture out of the (unrelated) transit-altitude corridor check."""
    lines = [_telem(1.0, 15.0, phase="search"),
             "t=3.05s land_gate_id_not_confirmed"]
    rep = _verify(lines, truth=[])
    assert any("land_gate_id_not_confirmed" in f for f in rep.fails), rep.fails


# ── FIX 2 (2026-07-24 review): a zero-delivery run must not read like a clean
# PASS ───────────────────────────────────────────────────────────────────────
# A flight that correctly refuses to release blind on every pad IS
# rules-compliant (the locked "never release blind" doctrine) — PASS/FAIL
# stays governed by rules-compliance. But 0/2 delivered must not be visually
# indistinguishable from a clean 2/2 run with one benign warning: `verify()`
# now carries the delivered-vs-planned headline on the Report itself, and
# `main()` prints it, unmissably, next to the PASS/FAIL verdict.

def _one_flight_zero_deliveries() -> list[str]:
    """Same flight, but the id gate never confirmed either pad — both eggs
    correctly came home undelivered. This must still PASS 0 fails: refusing
    to release blind is the design working, not a violation."""
    lines = _one_flight_two_deliveries()
    lines.remove(_REL_1)
    lines.remove(_REL_2)
    lines[lines.index(
        "t=3.3s DELIVERY 1 END delivered=True pad=3 err=0.10m landed=True"
    )] = "t=3.3s DELIVERY 1 END delivered=False pad=3 reason=not_found"
    lines[lines.index(
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True"
    )] = "t=4.3s DELIVERY 2 END delivered=False pad=1 reason=not_found"
    lines[lines.index("t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m "
                      "remaining=900s")] = (
        "t=5.1s FLIGHT 1 END delivered=0/2 d_home=1.0m remaining=900s")
    lines[0:0] = ["t=3.05s flight1_pad3_not_found",
                  "t=4.05s flight1_pad1_not_found"]
    return lines


def test_deliveries_summary_fields_populated_on_report() -> None:
    """`verify()` must populate the Report's delivery-summary fields (from
    the FLIGHT n END delivered=X/Y lines) so `main()` can print the headline
    without re-parsing the trail itself."""
    rep = _verify(_one_flight_two_deliveries())
    assert (rep.deliveries_done, rep.deliveries_planned,
            rep.flights_flown) == (2, 2, 1)


def test_deliveries_summary_line_normal_format() -> None:
    rep = _verify(_one_flight_two_deliveries())
    assert vf._deliveries_summary_line(rep) == "deliveries: 2/2 across 1 flight(s)"


def test_deliveries_summary_line_zero_delivered_is_loud() -> None:
    rep = _verify(_one_flight_zero_deliveries())
    assert rep.fails == [], rep.fails
    assert (vf._deliveries_summary_line(rep)
            == "deliveries: 0/2 — NO CARGO DELIVERED")


def test_zero_delivered_also_emits_an_explicit_warn() -> None:
    rep = _verify(_one_flight_zero_deliveries())
    assert any("NO CARGO DELIVERED" in w for w in rep.warns), rep.warns


def test_main_prints_deliveries_line_next_to_the_verdict(
        tmp_path, monkeypatch, capsys) -> None:
    """End-to-end: the CLI's final block prints the deliveries headline right
    next to the PASS/FAIL verdict, in the exact format the fix specifies."""
    lines = _one_flight_two_deliveries()
    audit = tmp_path / "audit.jsonl"
    audit.write_text("\n".join(json.dumps({"entry": e}) for e in lines) + "\n")
    truth_path = tmp_path / "truth.json"
    truth_path.write_text(json.dumps({"targets": _TRUTH}))
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(yaml.safe_dump(_MCFG))
    monkeypatch.setattr(sys, "argv", [
        "verify_flight.py", str(audit),
        "--truth", str(truth_path), "--config", str(cfg_path)])
    rc = vf.main()
    out = capsys.readouterr().out
    assert rc == 0
    assert "deliveries: 2/2 across 1 flight(s)" in out
    assert "drone-response verification: PASS" in out


def test_main_zero_delivered_still_passes_but_prints_the_loud_line(
        tmp_path, monkeypatch, capsys) -> None:
    """A 0/2 run stays a rules-compliance PASS (exit 0) — but the printed
    headline must be the loud, unmistakable form, not the quiet default."""
    lines = _one_flight_zero_deliveries()
    audit = tmp_path / "audit.jsonl"
    audit.write_text("\n".join(json.dumps({"entry": e}) for e in lines) + "\n")
    truth_path = tmp_path / "truth.json"
    truth_path.write_text(json.dumps({"targets": _TRUTH}))
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(yaml.safe_dump(_MCFG))
    monkeypatch.setattr(sys, "argv", [
        "verify_flight.py", str(audit),
        "--truth", str(truth_path), "--config", str(cfg_path)])
    rc = vf.main()
    out = capsys.readouterr().out
    assert rc == 0
    assert "deliveries: 0/2 — NO CARGO DELIVERED" in out


# ── FIX 3 (2026-07-24 review): pin two untested fail-closed branches + one
# untested warn branch ───────────────────────────────────────────────────────

def test_release_position_nan_fails_closed() -> None:
    """Before the FLIGHT/DELIVERY rewrite, `nan > threshold` was False, so a
    NaN release position silently produced an INFO '✓' line. Pin the FAIL."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(_REL_1)] = (
        "t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 lat=nan lon=nan")
    rep = _verify(lines)
    assert any("RELEASE position is NaN" in f for f in rep.fails), rep.fails


def test_release_pad_mismatches_delivery_end_pad_fails() -> None:
    """Defense-in-depth: today's emitters always agree (both come from the
    same `_drop_once` call, so this can't fire under the current mission
    code) — but if a future change ever decoupled them, the mismatch must
    fail rather than silently trust the END line's pad."""
    lines = _one_flight_two_deliveries()
    lines[lines.index(_REL_1)] = (
        "t=3.2s DELIVERY 1 RELEASE pad=5 payload=0 "
        "lat=13.7307000 lon=100.7880000")
    rep = _verify(lines)
    assert any("RELEASE line says pad 5" in f and "scored as pad 3" in f
               for f in rep.fails), rep.fails


def test_delivered_with_missing_release_line_warns() -> None:
    """A delivered=True DELIVERY END with NO matching RELEASE line at all
    (distinct from the already-pinned orphan-RELEASE-without-END case) must
    still surface — as a WARN, since the delivery itself is confirmed, just
    unscored for position."""
    lines = _one_flight_two_deliveries()
    lines.remove(_REL_1)
    rep = _verify(lines)
    assert any("delivery 1" in w and "no RELEASE position line found" in w
               for w in rep.warns), rep.warns
    assert rep.fails == [], rep.fails


# ---- batt/vbat grammar addition (2026-08-20) --------------------------------


def test_telem_legacy_format_still_parses() -> None:
    # All three TELEM vintages must parse: pre-2026-08-20 (no batt/vbat/mode
    # — the CM4's real Aug-17/18 flights), the batt-era (2026-08-20 field
    # day: batt/vbat, no mode), and the absent groups must surface as None.
    pre_batt = ("t=5.0s TELEM phase=transit_ingress flight=1 lat=13.7303000 "
                "lon=100.7874000 alt=19.50 armed=1")
    m = vf._TELEM.match(pre_batt)
    assert m is not None
    assert m.group("alt") == "19.50"
    assert m.group("batt") is None and m.group("vbat") is None
    assert m.group("mode") is None

    batt_era = ("t=240.6s TELEM phase=search flight=1 lat=13.8228179 "
                "lon=100.5119188 alt=-0.03 armed=0 batt=50.0 vbat=23.85")
    m = vf._TELEM.match(batt_era)          # verbatim G7 attempt-1 line
    assert m is not None
    assert m.group("batt") == "50.0" and m.group("vbat") == "23.85"
    assert m.group("mode") is None


def test_telem_emitter_shape_fullmatches_parser() -> None:
    # Lockstep pin: a line in the CURRENT emitter shape (mission.py
    # _telemetry_sampler f-string — batt/vbat + mode, NaN battery and the
    # UNKNOWN mode fallback included) must fullmatch.
    for batt, vbat, mode in ((81.0, 24.58, "HOLD"),
                             (float("nan"), float("nan"), "UNKNOWN"),
                             (62.0, 24.13, "POSCTL")):
        line = (f"t=12.3s TELEM phase=search flight=2 lat=13.8227944 "
                f"lon=100.5116412 alt=8.52 armed=1 "
                f"batt={batt:.1f} vbat={vbat:.2f} mode={mode}")
        m = vf._TELEM.fullmatch(line)
        assert m is not None, line
        assert m.group("batt") is not None
        assert m.group("mode") == mode


def test_battery_readout_reported_per_flight() -> None:
    entries = [
        "t=0.0s FLIGHT 1 START eggs=4 ids=1,2,3,4 remaining=1200s",
        _telem(1.0, 5.0, batt=80.0, vbat=24.60),
        _telem(2.0, 8.0, batt=62.0, vbat=23.10),
        _telem(3.0, 8.0, batt=70.0, vbat=23.90),
    ]
    rep = _run(entries)
    battery_lines = [i for i in rep.infos if "battery" in i]
    assert battery_lines, rep.infos
    # first→last with the under-load minimum called out
    assert "80%→70%" in battery_lines[0]
    assert "min 62% under load" in battery_lines[0]
    assert "23.10" in battery_lines[0]


def test_battery_readout_absent_for_legacy_archives() -> None:
    legacy = ("t=1.0s TELEM phase=transit_ingress flight=1 lat=13.7303000 "
              "lon=100.7874000 alt=8.00 armed=1")
    rep = _run(["t=0.0s FLIGHT 1 START eggs=4 ids=1,2,3,4 remaining=1200s",
                legacy])
    assert not any("battery" in i for i in rep.infos), rep.infos


# ── a takeover flight must not PASS (2026-08-22 review) ────────────────────
# _fire_takeover sets the terminal directly, so no FLIGHT END is written and
# every per-flight check is skipped: the tool printed
# "PASS · deliveries: 0/0 across 0 flight(s)" on a flight the safety pilot had
# to take away.


def test_pilot_takeover_is_fatal_not_a_pass() -> None:
    entries = [
        "t=1.0s FLIGHT 1 START eggs=4 ids=3,1,4,6 remaining=1200s",
        *[_telem(t, 19.5) for t in range(2, 10)],
        "t=11.0s pilot_takeover_posctl",
        "t=11.0s PILOT TAKEOVER mode=POSCTL — orchestrator standing down",
    ]
    rep = _run(entries)
    assert rep.fails, "a takeover flight passed verification"
    assert any("takeover" in f.lower() or "no matching END" in f for f in rep.fails)


def test_an_unfinished_flight_is_reported_even_without_an_anomaly() -> None:
    """A process killed mid-air leaves a START with no END, and every
    downstream check keys off the END."""
    entries = ["t=1.0s FLIGHT 1 START eggs=1 ids=3 remaining=1200s",
               *[_telem(t, 19.5) for t in range(2, 12)]]
    rep = _run(entries)
    assert any("no matching END" in f for f in rep.fails)


def test_a_completed_flight_does_not_trip_the_pairing_check() -> None:
    entries = ["t=1.0s FLIGHT 1 START eggs=1 ids=3 remaining=1200s",
               "t=90.0s FLIGHT 1 END delivered=1/1 d_home=2.0m"]
    rep = _run(entries)
    assert not any("no matching END" in f for f in rep.fails)
