"""Ground-survey fit (tools/survey_extract.py).

The professor expanded the practice area to the WHOLE KMUTNB field
(2026-08-14) and the boundary is measured by carrying the aircraft to each
corner — so this fit turns dwell points into the field frame every other
number is derived from. A synthetic walk with KNOWN truth (rotated
rectangle + GPS noise + a constant bias) must come back within a few cm:
the bias is common to all points, so the fitted frame cancels it.
"""

from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from survey_extract import dwells, fit_frame, load  # noqa: E402

_LAT0, _LON0 = 13.822669, 100.512146
_AXIS = 143.8
_CORNERS_ST = [(-52.0, -28.0), (-52.0, 28.0), (52.0, 28.0), (52.0, -28.0)]
_LR_ST = (-46.0, -20.0)
_BIAS_ST = (1.4, -0.9)          # constant GPS offset, must cancel in the fit


def _m_per_deg(lat: float) -> tuple[float, float]:
    a, f = 6_378_137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    s = math.sin(math.radians(lat))
    w = math.sqrt(1 - e2 * s * s)
    return (math.pi * a * (1 - e2) / (180 * w ** 3),
            math.pi * a * math.cos(math.radians(lat)) / (180 * w))


def _write_walk(path: Path) -> None:
    """Carry-and-stand walk: 35 s still at each corner, moving in between."""
    rng = random.Random(7)
    ml, mo = _m_per_deg(_LAT0)
    th = math.radians(_AXIS)
    u, v = (math.sin(th), math.cos(th)), (math.cos(th), -math.sin(th))

    def ll(s: float, t: float) -> tuple[float, float]:
        s, t = s + _BIAS_ST[0], t + _BIAS_ST[1]
        e, n = s * u[0] + t * v[0], s * u[1] + t * v[1]
        return _LAT0 + n / ml, _LON0 + e / mo

    rows, clock = [], 1_700_000_000.0
    prev = None
    for s, t in _CORNERS_ST + [_LR_ST]:
        legs = ([] if prev is None else
                [(prev[0] + (s - prev[0]) * k / 20,
                  prev[1] + (t - prev[1]) * k / 20) for k in range(1, 21)])
        for ss, tt in legs + [(s, t)] * 35:
            lat, lon = ll(ss, tt)
            rows.append({"t": round(clock, 2),
                         "lat": lat + rng.gauss(0, 0.35) / ml,
                         "lon": lon + rng.gauss(0, 0.35) / mo,
                         "alt_m": 15.0, "rel_alt_m": 0.0,
                         "fix": 3, "sats": 12, "eph": 0.8})
            clock += 1.0
        prev = (s, t)
    path.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")


def test_survey_walk_recovers_the_field_frame(tmp_path: Path) -> None:
    track = tmp_path / "survey.jsonl"
    _write_walk(track)
    marks = dwells(load(track, 3), min_s=20.0, radius_m=1.5)
    assert len(marks) == 5, "one dwell per corner + the L&R point"

    corners, lr = marks[:4], marks[4]
    fr = fit_frame(corners)
    # heading is mod 180 (a rectangle has no front) — compare that way
    assert min(abs(fr["axis_deg"] - _AXIS), 180 - abs(fr["axis_deg"] - _AXIS)) < 1.0
    ss = [s for s, _ in fr["st"]]
    ts = [t for _, t in fr["st"]]
    assert abs((max(ss) - min(ss)) - 104.0) < 1.5      # length
    assert abs((max(ts) - min(ts)) - 56.0) < 1.5       # width

    # the L&R mark lands where it was walked, in the FITTED frame
    ml, mo = _m_per_deg(fr["lat0"])
    th = math.radians(fr["axis_deg"])
    e = (lr["lon"] - fr["lon0"]) * mo
    n = (lr["lat"] - fr["lat0"]) * ml
    cs = (max(ss) + min(ss)) / 2
    ct = (max(ts) + min(ts)) / 2
    s = e * math.sin(th) + n * math.cos(th) - cs
    t = e * math.cos(th) - n * math.sin(th) - ct
    assert math.hypot(s - _LR_ST[0], t - _LR_ST[1]) < 1.5


def test_axis_is_the_long_side_not_the_diagonal(tmp_path: Path) -> None:
    """Regression: the first fit took the widest point PAIR as the axis —
    on a rectangle that is the diagonal (fitted 115° for a 143.8° field and
    doubled the area). The minimum-area rectangle must win."""
    track = tmp_path / "survey.jsonl"
    _write_walk(track)
    fr = fit_frame(dwells(load(track, 3), 20.0, 1.5)[:4])
    diag = math.degrees(math.atan2(56.0, 104.0))       # ~28.3° off the axis
    off = min(abs(fr["axis_deg"] - _AXIS), 180 - abs(fr["axis_deg"] - _AXIS))
    assert off < diag / 2, f"axis {fr['axis_deg']:.1f}° drifted toward the diagonal"
