"""Persisted tuned gains — the bridge from the pre-flight tuning module to the
mission.

The operator applies a gain set (model-based design, or PX4's native autotune) in
the GCS Tuning view; it is saved here, and ``orchestrator.main`` loads + re-applies
it at the start of EVERY mission so the competition sortie flies with the tuned
inner-loop gains without a manual re-apply (and so SITL gains survive a fresh boot,
which resets FC params). Plain JSON — no numpy.
"""

from __future__ import annotations

import json
from pathlib import Path

from mission_brain.schemas import active_airframe

DEFAULT_GAINS_DIR = Path("runs/sysid")


def gains_filename(airframe: str | None = None) -> str:
    """Gains file for an airframe. Keying the filename on the airframe is a
    safety property, not cosmetics: gains identified on the quad must never be
    auto-applied to the hexa, so a vehicle swap silently falls back to the FC's
    own params instead of loading a mis-tuned set."""
    return f"{airframe or active_airframe().value}_gains.json"


def gains_path(out_dir: str | Path = DEFAULT_GAINS_DIR, airframe: str | None = None) -> Path:
    return Path(out_dir) / gains_filename(airframe)


def save_gains(
    params: dict[str, float], source: str = "",
    out_dir: str | Path = DEFAULT_GAINS_DIR,
    airframe: str | None = None,
) -> Path:
    """Write the param→value map to apply at mission start. Returns the path."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / gains_filename(airframe)
    path.write_text(json.dumps(
        {"params": {k: float(v) for k, v in params.items()}, "source": source},
        indent=2,
    ))
    return path


def load_gains(
    out_dir: str | Path = DEFAULT_GAINS_DIR, airframe: str | None = None,
) -> dict[str, float]:
    """Load the saved tuned gains (param→value), or {} if absent/unreadable.

    A corrupt or missing file must NEVER break a mission launch — returns {} and
    the mission flies with the FC's existing params."""
    path = Path(out_dir) / gains_filename(airframe)
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text())
        raw = data.get("params", {}) if isinstance(data, dict) else {}
        return {str(k): float(v) for k, v in raw.items()}
    except Exception:  # noqa: BLE001 — a bad file must not ground the mission
        return {}
