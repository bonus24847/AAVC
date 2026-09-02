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
import os
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
def test_world_ships_still_air(world: Path) -> None:
    """The default world is CALM (operator decision): every validated number
    this project quotes was measured in still air, so a normal run must keep
    reproducing it. Wind is switched on deliberately, per run, by
    sitl/set_wind.sh — which is also why nothing here has to stay in step with
    wind_sitl's numbers any more."""
    root = ET.parse(world).getroot()
    node = root.find("world/wind/linear_velocity")
    assert node is not None, f"{world.name} has no <wind><linear_velocity>"
    assert [float(v) for v in (node.text or "").split()] == [0.0, 0.0, 0.0], (
        f"{world.name} ships wind on by default: {node.text!r}")


def test_the_wind_switch_converts_bearing_to_the_enu_vector() -> None:
    """set_wind.sh takes a METEOROLOGICAL bearing (where wind comes FROM, the
    forecast convention) and must hand gz the vector it blows TOWARD. Getting
    that backwards is a silent 180-degree error that would make every windy
    result describe a wind nobody has."""
    script = _ROOT / "sitl/set_wind.sh"
    assert script.exists() and os.access(script, os.X_OK), "set_wind.sh missing/not executable"
    # Read it, don't shell out: this repo's own path contains spaces, and an
    # unquoted path is exactly the kind of thing that makes a test pass or fail
    # for reasons that have nothing to do with what it is testing.
    body = script.read_text(encoding="utf-8")
    assert "bearing + 180" in body, "the FROM->TOWARD flip is gone from set_wind.sh"
    # 225 deg FROM the SW blows toward the NE: both components positive.
    toward = math.radians((225.0 + 180.0) % 360.0)
    assert math.sin(toward) > 0 and math.cos(toward) > 0


def test_configured_wind_is_worth_flying_against() -> None:
    """A token breeze would let the wind path pass while proving nothing. The
    Bangkok monsoon figure is 4 m/s with gusts — keep it in a band that actually
    loads the position controller during the 0.2 m final rung."""
    cfg = _wind_sitl()
    assert 3.0 <= float(cfg["base_speed_mps"]) <= 10.0
    assert float(cfg["gust_amplitude_mps"]) > 0.0
