"""Measured-plant calibration: anchor the model's rate-loop plant gain ``b`` to a
frequency-sweep system identification (``tuning.sysid``) instead of the
first-principles ``τ_max / I`` estimate in ``plant.py``.

**What the measured FRF actually showed (quad, SITL 2026-06-01, coherence ≥ 0.97):**
``b_roll ≈ 121`` (model 141, ×0.86), ``b_pitch ≈ 207`` (model 141, ×1.46),
``b_yaw ≈ 15`` (model 60, **×0.25**). So roll/pitch are well-modeled, but the
first-principles model **OVER-estimates yaw authority ~4×** — the opposite of an
earlier (now-refuted) assumption that it *under*-estimated yaw. The override is
unit-consistent (both are the normalized-command → angular-acceleration integrator
gain, rad/s² per unit command), so using the measured ``b`` for **roll/pitch** is a
direct, principled improvement.

**IMPORTANT — measured-b does NOT fix the yaw over-tune; it would worsen it.**
``P = 2ζω/b`` ⇒ a *lower* measured ``b_yaw`` (15 < 60) makes the designed yaw P
*higher*, not lower. The yaw over-tune the gain-compare saw (model P ≈ 4× stock,
worse yaw RMS) is really a **bandwidth/authority** problem: yaw is the weakest,
slowest loop, and designing it at the roll/pitch bandwidth (6 Hz) is too aggressive
for the (genuinely small) yaw authority. The correct yaw fix is a **yaw rate-
bandwidth de-rate** (``PerfSpec.yaw_rate_bandwidth_hz``) — or simply the empirical
safe-optimizer / stock yaw. The engine warns when ``b_yaw`` is far below the model
so a blind substitution can't quietly over-tune yaw.

**Two regimes, kept separate.** The hover sweep excites the MULTICOPTER rate
loop, so its measured ``b`` lands in ``b_measured`` (consumed by the MC
synthesis). A fixed-wing CRUISE sweep excites the FW rate loop, so its measured
``b`` lands in the separate ``b_measured_fw`` (consumed by the FW synthesis).
Both are the SAME unit basis — rad/s² per unit *normalized* command — but
measured at different flight conditions, so they are stored apart and never
cross-applied.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from mission_brain.schemas import Airframe

from . import plant
from .schemas import _NEEDS_MC, PhysicalParams

_AXES = ("roll", "pitch", "yaw")
DEFAULT_SYSID_DIR = Path("runs/sysid")


@dataclass
class PlantCalibration:
    """Measured rate-loop plant gains ``b`` (rad/s² per unit normalized command),
    per body axis, from a frequency-sweep system-ID. Consumed by
    ``engine.design_gains`` as an OPTIONAL override of the first-principles ``b``.

    ``b_measured``    — multicopter / hover regime → MC rate synthesis.
    ``b_measured_fw`` — fixed-wing / cruise regime → FW rate synthesis.
    Kept apart so a VTOL/twin-boom's two regimes never mix.
    """

    airframe: str
    b_measured: dict[str, float] = field(default_factory=dict)
    source: str = ""    # ULog name / run stamp the measurement came from
    b_measured_fw: dict[str, float] = field(default_factory=dict)

    def b_for(self, axis: str) -> float | None:
        """Measured MC (hover) ``b`` for an axis, or None if absent / non-positive."""
        v = self.b_measured.get(axis)
        return float(v) if v is not None and v > 0 else None

    def b_fw_for(self, axis: str) -> float | None:
        """Measured FW (cruise) ``b`` for an axis, or None if absent / non-positive."""
        v = self.b_measured_fw.get(axis)
        return float(v) if v is not None and v > 0 else None

    def to_dict(self) -> dict[str, Any]:
        return {
            "airframe": self.airframe,
            "b_measured": dict(self.b_measured),
            "b_measured_fw": dict(self.b_measured_fw),
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> PlantCalibration:
        raw = d.get("b_measured") or {}
        raw_fw = d.get("b_measured_fw") or {}
        return cls(
            airframe=str(d.get("airframe", "")),
            b_measured={k: float(v) for k, v in raw.items() if v is not None},
            source=str(d.get("source", "")),
            b_measured_fw={k: float(v) for k, v in raw_fw.items() if v is not None},
        )


def save_calibration(
    calib: PlantCalibration,
    out_dir: str | Path = DEFAULT_SYSID_DIR,
    stamp: str | None = None,
) -> Path:
    """Write ``<airframe>_latest.json`` (what ``load_calibration`` reads) and,
    if ``stamp`` is given, a stamped archival copy. Returns the latest path."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(calib.to_dict(), indent=2)
    latest = out / f"{calib.airframe}_latest.json"
    latest.write_text(payload)
    if stamp:
        (out / f"{calib.airframe}_{stamp}.json").write_text(payload)
    return latest


def load_calibration(
    airframe: str, out_dir: str | Path = DEFAULT_SYSID_DIR,
) -> PlantCalibration | None:
    """Load the latest calibration for an airframe, or None if absent/unreadable."""
    p = Path(out_dir) / f"{airframe}_latest.json"
    if not p.exists():
        return None
    try:
        return PlantCalibration.from_dict(json.loads(p.read_text()))
    except Exception:    # noqa: BLE001 — a corrupt file must not break design
        return None


def is_mc_airframe(airframe: str) -> bool:
    """True for multicopter-family airframes (the override target)."""
    try:
        return Airframe(airframe) in _NEEDS_MC
    except ValueError:
        return False


def measured_vs_model(
    airframe: str, calib: PlantCalibration, physical: PhysicalParams,
) -> list[dict[str, Any]]:
    """Per-axis comparison of measured ``b`` vs the first-principles MC plant ``b``.

    Only meaningful for multicopter-family airframes (the FRF measures the same
    normalized-command integrator the MC model uses). Returns [] for fixed-wing.
    """
    rows: list[dict[str, Any]] = []
    if not is_mc_airframe(airframe):
        return rows
    for axis in _AXES:
        b_meas = calib.b_for(axis)
        try:
            b_model: float | None = plant.mc_plant_gain(physical, axis)
        except Exception:    # noqa: BLE001
            b_model = None
        ratio = (b_meas / b_model) if (b_meas and b_model) else None
        rows.append({
            "axis": axis,
            "b_model": round(b_model, 4) if b_model is not None else None,
            "b_meas": round(b_meas, 4) if b_meas is not None else None,
            "ratio_meas_over_model": round(ratio, 3) if ratio is not None else None,
        })
    return rows
