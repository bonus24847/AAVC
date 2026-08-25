"""The ranking, which is the part that was wrong when read off the GCS.

Two receivers benched on 2026-08-25 minutes apart at one spot: the one showing
MORE satellites scattered 5x wider. Satellite count — the one GPS number every
screen displays — ordered them backwards. These tests pin the arithmetic that
gets the order right, and the verdict thresholds that tie it to constants this
repo already flies on (``cluster_radius_m`` 8.0, ``rung_tol_m[0]`` 1.5).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from gps_bench import GpsSample, classify, local_metres, summarise  # noqa: E402

# The two runs that motivated the tool, as measured (docs value, kept as data).
_BENCH_2026_08_25 = {
    "unit1": {"sats_avg": 16.3, "hdop_max": 1.11, "sigma_e": 2.38, "cep95": 4.96},
    "unit2": {"sats_avg": 15.8, "hdop_max": 0.97, "sigma_e": 0.27, "cep95": 0.97},
}


def _run(offsets_m: list[tuple[float, float]], *, sats: int = 14,
         fix: int = 3, lat0: float = 13.8217, lon0: float = 100.5134) -> list[GpsSample]:
    """Samples placed at exact (north, east) metre offsets from one origin."""
    phi = math.radians(lat0)
    a, e2 = 6378137.0, 6.69437999014e-3
    w = math.sqrt(1.0 - e2 * math.sin(phi) ** 2)
    m_lat = math.pi / 180.0 * a * (1.0 - e2) / w**3
    m_lon = math.pi / 180.0 * a * math.cos(phi) / w
    return [GpsSample(t=float(i), lat=lat0 + n / m_lat, lon=lon0 + e / m_lon,
                      fix_type=fix, sats=sats, hdop=1.0, alt_m=10.0)
            for i, (n, e) in enumerate(offsets_m)]


def test_metre_conversion_round_trips() -> None:
    """A fix placed 3 m north and 4 m east must measure 3 m north, 4 m east.

    The north axis is the one that has been wrong here before: spawn_targets
    used the EQUATORIAL radius and biased every truth distance (2026-07-04).
    """
    north, east = local_metres(_run([(0.0, 0.0), (3.0, 4.0)]))
    assert math.isclose(north[1] - north[0], 3.0, abs_tol=0.01)
    assert math.isclose(east[1] - east[0], 4.0, abs_tol=0.01)


def test_a_perfectly_still_receiver_scores_zero_spread() -> None:
    q = summarise(_run([(0.0, 0.0)] * 20), "still")
    assert q.sigma_n_m == 0.0 and q.sigma_e_m == 0.0
    assert q.cep95_m == 0.0 and q.drift_m == 0.0
    assert classify(q)[0] == 0


def test_more_satellites_does_not_beat_tighter_scatter() -> None:
    """The exact inversion seen on the bench: the 16-sat module loses."""
    many_sats_wide = summarise(
        _run([(0.0, e) for e in (-4.0, -2.0, 0.0, 2.0, 4.0)] * 4, sats=18), "unit1")
    fewer_sats_tight = summarise(
        _run([(0.0, e) for e in (-0.3, -0.1, 0.0, 0.1, 0.3)] * 4, sats=15), "unit2")
    assert many_sats_wide.sats_avg > fewer_sats_tight.sats_avg
    assert many_sats_wide.cep95_m > fewer_sats_tight.cep95_m
    assert classify(fewer_sats_tight)[0] < classify(many_sats_wide)[0]


def test_east_only_scatter_is_reported_on_the_east_axis() -> None:
    """unit1's signature was axis-specific — a summary that smeared it across
    both axes would have hidden the multipath clue."""
    q = summarise(_run([(0.0, e) for e in (-2.0, -1.0, 0.0, 1.0, 2.0)] * 4), "east")
    assert q.sigma_n_m == 0.0
    assert q.sigma_e_m > 1.0


def test_scatter_near_the_cluster_radius_fails() -> None:
    """8.0 m is TargetTracker.cluster_radius_m: at that spread one pad can
    register as two, which is a wrong-pad landing rather than a wobble."""
    q = summarise(_run([(0.0, e) for e in (-5.0, -2.5, 0.0, 2.5, 5.0)] * 4), "wide")
    assert classify(q)[0] == 2


def test_no_3d_fix_is_never_a_pass_however_tight() -> None:
    """A 2D fix has no altitude and a fictitious horizontal spread; tightness
    must not launder it into a green verdict."""
    q = summarise(_run([(0.0, 0.0)] * 20, fix=2), "twod")
    code, verdict = classify(q)
    assert code == 2
    assert "3D" in verdict


def test_too_few_samples_is_reported_not_averaged() -> None:
    code, verdict = classify(summarise(_run([(0.0, 0.0)] * 2), "short"))
    assert code == 2
    assert "จุด" in verdict


def test_frozen_coordinates_after_a_lost_fix_do_not_flatter_the_module() -> None:
    """The trap the first real run walked into (2026-08-25).

    Both sources keep serving the LAST KNOWN lat/lon once the fix is gone. Those
    frozen rows do not just dilute the spread — they pull it toward zero, so the
    worse the reception the better the module scores. Scatter must therefore be
    computed over 3D fixes only, and a run that lost the fix must not be
    summarised at all.
    """
    good = _run([(0.0, e) for e in (-3.0, -1.5, 0.0, 1.5, 3.0)] * 2, fix=3)
    frozen = _run([(0.0, 0.0)] * 40, fix=0)          # same coordinate, no fix
    for s in frozen:
        s.t = good[-1].t + 1.0
    q = summarise(good + frozen, "half-lost")

    assert q.n == len(good) + len(frozen)
    assert q.n_used == len(good)          # the frozen rows never reach the maths
    assert q.sigma_e_m > 1.0              # NOT flattened toward zero
    code, verdict = classify(q)
    assert code == 2
    assert "หลุด" in verdict              # and the operator is told why


def test_an_all_stale_run_is_rejected_not_scored_as_perfect() -> None:
    """20 frozen rows read a flawless 0.00 m spread — the exact output of the
    first field run. It must fail on the fix gate, never pass on tightness."""
    q = summarise(_run([(0.0, 0.0)] * 20, fix=0), "stale")
    assert q.n_used == 0
    assert classify(q)[0] == 2
