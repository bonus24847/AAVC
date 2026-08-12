"""Operator-drawn field geometry merge (orchestrator/field_override.py).

The GCS map editor writes captures/field_override.json; the orchestrator must
fly EXACTLY that geometry or refuse to fly at all — every invalid shape is a
hard FieldOverrideError (fail closed), never a silent fallback to yaml.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from orchestrator.field_override import (
    SCHEMA_VERSION,
    FieldOverrideError,
    apply_field_override,
)

_TRI = [[13.8225, 100.5121], [13.8227, 100.5121], [13.8226, 100.5123]]
_ROUTE = [[13.8225, 100.5122], [13.8226, 100.5121]]


def _doc(**over: object) -> dict:
    doc: dict = {
        "version": SCHEMA_VERSION,
        "controlled_airspace": _TRI,
        "search_area": _TRI,
        "transit_route": _ROUTE,
    }
    doc.update(over)
    return doc


def _write(tmp_path: Path, doc: dict) -> Path:
    p = tmp_path / "field_override.json"
    p.write_text(json.dumps(doc))
    return p


def test_valid_override_replaces_all_three_geometries(tmp_path: Path) -> None:
    cfg: dict = {"controlled_airspace": [[0, 0]], "search_area": [],
                 "transit_route": [], "site": {"center_lat": 1.0,
                                               "center_lon": 2.0}}
    apply_field_override(cfg, _write(tmp_path, _doc()))
    assert cfg["controlled_airspace"] == _TRI
    assert cfg["search_area"] == _TRI
    assert cfg["transit_route"] == _ROUTE
    # no lr_point in the doc → the L&R / site centre is untouched
    assert cfg["site"] == {"center_lat": 1.0, "center_lon": 2.0}


def test_lr_point_moves_the_site_centre(tmp_path: Path) -> None:
    cfg: dict = {"site": {"center_lat": 1.0, "center_lon": 2.0}}
    apply_field_override(
        cfg, _write(tmp_path, _doc(lr_point=[13.9999, 100.4444])))
    assert cfg["site"]["center_lat"] == 13.9999
    assert cfg["site"]["center_lon"] == 100.4444


@pytest.mark.parametrize("bad", [
    _doc(controlled_airspace=_TRI[:2]),        # polygon needs >= 3 vertices
    _doc(search_area="not-a-list"),
    _doc(transit_route=[]),                    # route needs >= 1 point
    _doc(controlled_airspace=[[13.8, 100.5], [13.8, 100.6], [999.0, 0.0]]),
    _doc(controlled_airspace=[[13.8, 100.5], [13.8, 100.6], [13.9]]),
    _doc(version=99),                          # unknown schema
    _doc(lr_point=[91.0, 0.0]),                # L&R out of range
])
def test_invalid_documents_fail_closed(tmp_path: Path, bad: dict) -> None:
    with pytest.raises(FieldOverrideError):
        apply_field_override({}, _write(tmp_path, bad))


def test_missing_and_corrupt_files_fail_closed(tmp_path: Path) -> None:
    with pytest.raises(FieldOverrideError):
        apply_field_override({}, tmp_path / "nope.json")
    p = tmp_path / "broken.json"
    p.write_text("{not json")
    with pytest.raises(FieldOverrideError):
        apply_field_override({}, p)
