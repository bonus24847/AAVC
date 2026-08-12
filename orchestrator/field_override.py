"""Operator-drawn field geometry (the GCS map editor, 2026-08-13).

The AAVC GCS console lets the operator DRAW the field at the site — the
controlled-airspace polygon, the search-area polygon, the transit route
P1→P2→P3 and optionally the L&R point — and saves it as
``captures/field_override.json``. ``run_mission.sh`` hands that file to the
orchestrator via ``--field-override``; this module merges it over the yaml
config so ONE build flies any field (the competition's real coordinates only
settle at the event briefing).

Schema (version 1, shared with the sibling competition repo — keep stable):

    {"version": 1, "updated": <epoch>,
     "controlled_airspace": [[lat, lon], ...],   # >= 3 vertices
     "search_area":         [[lat, lon], ...],   # >= 3 vertices
     "transit_route":       [[lat, lon], ...],   # >= 1 point
     "lr_point":            [lat, lon] | null}   # optional new L&R / home

FAILS CLOSED: any structural or range problem raises ``FieldOverrideError``
— flying a hand-drawn field with a silently-dropped polygon would be worse
than not flying, so main aborts the run instead of falling back quietly.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from loguru import logger

SCHEMA_VERSION = 1


class FieldOverrideError(ValueError):
    """The override file exists but cannot be trusted — refuse to fly it."""


def _points(doc: dict[str, Any], key: str, min_len: int) -> list[list[float]]:
    val = doc.get(key)
    if not isinstance(val, list) or len(val) < min_len:
        raise FieldOverrideError(
            f"{key}: need a list of >= {min_len} [lat, lon] points, got "
            f"{val!r:.80}")
    out: list[list[float]] = []
    for i, p in enumerate(val):
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or not all(isinstance(x, (int, float)) for x in p)):
            raise FieldOverrideError(f"{key}[{i}]: not a [lat, lon] pair: {p!r}")
        lat, lon = float(p[0]), float(p[1])
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise FieldOverrideError(f"{key}[{i}]: out of range: {p!r}")
        out.append([lat, lon])
    return out


def apply_field_override(cfg: dict[str, Any], path: Path) -> dict[str, Any]:
    """Merge the operator-drawn geometry over ``cfg`` IN PLACE (and return it).

    Raises ``FieldOverrideError`` on a missing/invalid file — the caller
    (orchestrator.main) treats that as a startup abort, never a fallback.
    """
    try:
        doc = json.loads(Path(path).read_text())
    except FileNotFoundError:
        raise FieldOverrideError(f"override file not found: {path}") from None
    except (OSError, json.JSONDecodeError) as e:
        raise FieldOverrideError(f"override file unreadable: {e}") from None
    if not isinstance(doc, dict):
        raise FieldOverrideError("override root must be a JSON object")
    ver = doc.get("version")
    if ver != SCHEMA_VERSION:
        raise FieldOverrideError(
            f"override schema version {ver!r} != {SCHEMA_VERSION}")

    airspace = _points(doc, "controlled_airspace", 3)
    search = _points(doc, "search_area", 3)
    transit = _points(doc, "transit_route", 1)

    cfg["controlled_airspace"] = airspace
    cfg["search_area"] = search
    cfg["transit_route"] = transit

    lr = doc.get("lr_point")
    if lr is not None:
        lr_pt = _points({"lr": [lr]}, "lr", 1)[0]
        site = cfg.setdefault("site", {})
        site["center_lat"], site["center_lon"] = lr_pt[0], lr_pt[1]
        logger.warning(
            f"[field-override] L&R moved to {lr_pt[0]:.7f}, {lr_pt[1]:.7f} — "
            "SITL worlds do NOT follow this point; use lr_point on the REAL "
            "field only")

    logger.info(
        f"[field-override] applied {path}: airspace {len(airspace)} vtx, "
        f"search {len(search)} vtx, transit {len(transit)} pts"
        f"{', L&R moved' if lr is not None else ''}")
    return cfg
