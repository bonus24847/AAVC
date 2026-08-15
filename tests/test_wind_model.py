"""The SITL wind must actually reach the aircraft.

Found 2026-08-15, in answer to "have we tested with wind?": both worlds carried
a WindEffects plugin and aavc_config.yaml carried a wind_sitl block, so wind
LOOKED modelled — but gz only pushes links that opt in with <enable_wind>, and
no link did. Every validated flight, G4 and G4' included, flew in still air.
KMITL is exposed and windy, which makes still-air scatter the wrong number to
plan the landing tolerance around.

These lock the three pieces that have to line up, because any one of them
failing is silent: the plugin, the opt-in, and a base vector that matches the
written intent.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

_ROOT = Path(__file__).resolve().parent.parent
_CONFIG = _ROOT / "sitl/aavc_config.yaml"
_WORLDS = [_ROOT / "sitl/worlds/kmutnb_skyfield.sdf"]
_AIRFRAME = _ROOT / "sitl/models/eft_x6100_base/model.sdf"


def _wind_sitl() -> dict:
    return yaml.safe_load(_CONFIG.read_text(encoding="utf-8"))["wind_sitl"]


def test_airframe_opts_into_wind() -> None:
    """Without this the WindEffects plugin is decoration: gz applies wind force
    only to links/models carrying <enable_wind>."""
    root = ET.parse(_AIRFRAME).getroot()
    model = root.find("model")
    assert model is not None
    flags = [e.text for e in model.iter("enable_wind")]
    assert flags and any((f or "").strip().lower() in ("true", "1") for f in flags), (
        "the airframe does not opt into wind — the wind plugin will not touch it")


@pytest.mark.parametrize("world", _WORLDS, ids=lambda p: p.name)
def test_world_has_the_wind_plugin(world: Path) -> None:
    text = world.read_text(encoding="utf-8")
    assert "gz-sim-wind-effects-system" in text, f"{world.name} lost the wind plugin"


@pytest.mark.parametrize("world", _WORLDS, ids=lambda p: p.name)
def test_world_base_vector_matches_the_configured_intent(world: Path) -> None:
    """No code reads wind_sitl — the world file is what actually blows — so the
    two can drift apart silently. Pin them together: speed and compass bearing.

    direction_deg is meteorological (the direction the wind comes FROM), so a
    225 deg wind blows toward 45 deg: east = v*sin(bearing), north = v*cos.
    """
    cfg = _wind_sitl()
    speed = float(cfg["base_speed_mps"])
    toward = math.radians((float(cfg["direction_deg"]) + 180.0) % 360.0)
    want_e, want_n = speed * math.sin(toward), speed * math.cos(toward)

    root = ET.parse(world).getroot()
    # `is None`, not `or`: an ElementTree element with no children is FALSY, so
    # `found or fallback` silently discards a perfectly good element.
    node = root.find("world/wind/linear_velocity")
    assert node is not None, f"{world.name} has no <wind><linear_velocity>"
    got = [float(v) for v in (node.text or "").split()]
    assert len(got) == 3, f"{world.name}: wind linear_velocity is {node.text!r}"
    assert math.isclose(got[0], want_e, abs_tol=0.05), f"east {got[0]} vs {want_e:.2f}"
    assert math.isclose(got[1], want_n, abs_tol=0.05), f"north {got[1]} vs {want_n:.2f}"
    assert got[2] == 0.0, "vertical base wind is not part of the model"


def test_configured_wind_is_worth_flying_against() -> None:
    """A token breeze would let the wind path pass while proving nothing. The
    Bangkok monsoon figure is 4 m/s with gusts — keep it in a band that actually
    loads the position controller during the 0.2 m final rung."""
    cfg = _wind_sitl()
    assert 3.0 <= float(cfg["base_speed_mps"]) <= 10.0
    assert float(cfg["gust_amplitude_mps"]) > 0.0
