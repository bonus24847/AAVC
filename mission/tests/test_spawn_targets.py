"""SITL pad placement (sitl/spawn_targets.py).

The event briefing: the real field carries SIX physical ArUco pads, and the
committee assigns only 4 of them per team — so the two unassigned ids must
actually exist in the SITL world as permanent distractors, or the wrong-id
LAND-gate rejection path is never exercised across a whole mission. These
lock the pure id/position math `_layout` (and its `BASELINE_PADS` fallback)
does WITHOUT a live Gazebo — no `gz service` calls are exercised here; that
needs a live simulator (see the brief).
"""

from __future__ import annotations

import itertools
import math
from pathlib import Path

import yaml

from sitl.spawn_targets import (
    _ALL_PAD_NAMES,
    BASELINE_PADS,
    DEFAULT_N_PADS,
    MIN_SEPARATION_M,
    POLYGON_INSET_M,
    _inset_ok,
    _latlon_to_local_enu,
    _layout,
)

_REPO = Path(__file__).resolve().parents[1]
_POLY = [(0.0, 0.0), (200.0, 0.0), (200.0, 60.0), (0.0, 60.0)]
_VALID_IDS = [1, 2, 3, 4, 5, 6]


def test_layout_places_six_distinct_ids_when_asked() -> None:
    pads = _layout(seed=42, search_poly_enu=_POLY, valid_ids=[1, 2, 3, 4, 5, 6],
                   n_pads=6)
    ids = [p[0] for p in pads]
    assert len(pads) == 6
    assert sorted(ids) == [1, 2, 3, 4, 5, 6]


def test_layout_default_n_pads_is_six() -> None:
    """No explicit ``n_pads`` -> DEFAULT_N_PADS (6), not the old hardcoded 4."""
    assert DEFAULT_N_PADS == 6
    pads = _layout(seed=7, search_poly_enu=_POLY, valid_ids=_VALID_IDS)
    ids = [p[0] for p in pads]
    assert len(pads) == 6
    assert sorted(ids) == [1, 2, 3, 4, 5, 6]


def test_layout_n_pads_four_still_works() -> None:
    """``n_pads`` is a real, honoured parameter, not cosmetic — asking for
    fewer pads returns exactly that many DISTINCT ids (the old 4-of-6
    behaviour), not always 6."""
    pads = _layout(seed=11, search_poly_enu=_POLY, valid_ids=_VALID_IDS, n_pads=4)
    ids = [p[0] for p in pads]
    assert len(pads) == 4
    assert len(set(ids)) == 4
    assert set(ids) <= {1, 2, 3, 4, 5, 6}


def test_layout_n_pads_caps_at_valid_ids_count() -> None:
    """``k = min(n_pads, len(valid_ids))`` — asking for more pads than there
    are valid ids caps rather than raising."""
    pads = _layout(seed=3, search_poly_enu=_POLY, valid_ids=[1, 2, 3], n_pads=6)
    ids = [p[0] for p in pads]
    assert len(pads) == 3
    assert sorted(ids) == [1, 2, 3]


def test_no_seed_returns_the_six_pad_baseline_regardless_of_n_pads() -> None:
    """No seed -> the static world-file baseline, unconditionally — ``n_pads``
    has no effect on this path (it only shapes a --seed layout)."""
    pads = _layout(seed=None, search_poly_enu=_POLY, valid_ids=_VALID_IDS, n_pads=4)
    assert pads == list(BASELINE_PADS)
    assert len(pads) == 6


def test_baseline_pads_has_six_distinct_ids_covering_one_to_six() -> None:
    assert len(BASELINE_PADS) == 6
    ids = [p[0] for p in BASELINE_PADS]
    assert sorted(ids) == [1, 2, 3, 4, 5, 6]
    assert len(set(ids)) == 6


def test_baseline_pads_are_pairwise_separated_by_at_least_25m() -> None:
    """The constraint most easily broken by hand-picked coordinates — assert
    it numerically rather than trusting the numbers look spread out."""
    for (id1, x1, y1, _), (id2, x2, y2, _) in itertools.combinations(BASELINE_PADS, 2):
        d = math.hypot(x1 - x2, y1 - y2)
        assert d >= MIN_SEPARATION_M, (
            f"id{id1} and id{id2} are only {d:.2f} m apart "
            f"(need >= {MIN_SEPARATION_M} m)")


def test_baseline_pads_are_inset_inside_the_real_search_area() -> None:
    """Every baseline pad must sit >= POLYGON_INSET_M inside the ACTUAL
    competition search-area polygon (sitl/aavc_config.yaml), not just some
    synthetic test rectangle."""
    cfg = yaml.safe_load((_REPO / "sitl" / "aavc_config.yaml").read_text())
    lat0 = float(cfg["site"]["center_lat"])
    lon0 = float(cfg["site"]["center_lon"])
    search_poly = [(float(v[0]), float(v[1])) for v in cfg["search_area"]]
    poly_enu = [_latlon_to_local_enu(lat, lon, lat0, lon0) for lat, lon in search_poly]
    for mid, x, y, _yaw in BASELINE_PADS:
        assert _inset_ok(x, y, poly_enu, POLYGON_INSET_M), (
            f"id{mid} at ({x}, {y}) is not >= {POLYGON_INSET_M} m inside the "
            "search-area polygon")


def test_baseline_pads_have_distinct_yaws() -> None:
    yaws = [p[3] for p in BASELINE_PADS]
    assert len(set(yaws)) == len(yaws)


def test_all_pad_names_covers_all_six_slots() -> None:
    """The idempotent pre-spawn cleanup must remove every name that could ever
    have been spawned (ids 1-6), so a stale pad_5/pad_6 from a previous
    6-pad run can never survive into a later, smaller-n_pads run."""
    assert _ALL_PAD_NAMES == tuple(f"pad_{i}" for i in range(0, 7))
    assert len(_ALL_PAD_NAMES) == 7          # ids 0-6 since 2026-08-27
