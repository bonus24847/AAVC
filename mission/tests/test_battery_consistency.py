"""The pack's identity, checked across every file that states it.

The gauge on this aircraft is `interpolate(cell_v, V_EMPTY, V_CHARGED)` and
nothing else — the PM02D feeds the FC only, motors run from a board it cannot
sense, so there is no coulomb counting to fall back on. That makes four numbers
load-bearing (cells, capacity, and the two endpoints) and puts them in more
than one file: the preflight tool that STOPS a field day, the field configs the
mission flies, and the power reference the operator reads.

They have already disagreed. The ledger records "a live `BAT1_V_CHARGED`
4.15-vs-4.05 conflict" between the two repos on one aircraft, and the endpoints
moved again when the 7500 LiPo was replaced by the 17000 semi-solid. A wrong
endpoint does not look wrong: every percentage the operator reads, every
`rth_battery_pct` decision and every energy refusal is computed from it, and
they all stay plausible while being wrong.

The manual rule lived in `power-battery.md` ("keep these in step by hand").
This is that rule, executed.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from tools.preflight_params import BOARD

_ROOT = Path(__file__).resolve().parents[1]
_CONFIGS = sorted((_ROOT / "sitl").glob("*config.yaml"))
_POWER_DOC = _ROOT / ".claude/skills/PX4MASTER/references/power-battery.md"


def _cfg(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def test_every_field_config_describes_the_same_pack() -> None:
    """One aircraft, one pack. The two repos each carry both configs, so a
    per-field edit is how they drift apart."""
    assert len(_CONFIGS) >= 2, f"expected both field configs, found {_CONFIGS}"
    packs = {p.name: (_cfg(p)["battery"]["cells"], _cfg(p)["battery"]["capacity_mah"])
             for p in _CONFIGS}
    assert len(set(packs.values())) == 1, f"configs disagree about the pack: {packs}"


def test_the_config_cell_count_matches_the_board_check() -> None:
    """`BAT1_N_CELLS` turns pack volts into the cell volts the gauge
    interpolates. Six cells read as five and every percentage is wrong by
    ~17 % while looking entirely reasonable."""
    for path in _CONFIGS:
        assert _cfg(path)["battery"]["cells"] == BOARD["BAT1_N_CELLS"], path.name


def test_the_power_reference_quotes_the_endpoints_the_preflight_enforces() -> None:
    """power-battery.md is what a human reads before touching the board; BOARD
    is what STOPS the field day. If they disagree, one of them is teaching the
    wrong number — and the doc is the one that gets believed at 07:30."""
    if not _POWER_DOC.exists():
        # The PX4MASTER skill lives in the practice repo only; sync_core.sh
        # deliberately does not copy .claude/. Skipping HERE rather than
        # softening the assertion keeps the check real where the doc exists —
        # and the doc is where the operator reads the number.
        pytest.skip(f"{_POWER_DOC.name} not in this repo (skill not synced)")
    doc = _POWER_DOC.read_text(encoding="utf-8")
    for param in ("BAT1_V_CHARGED", "BAT1_V_EMPTY"):
        m = re.search(rf"{param}\s*=\s*([0-9.]+)", doc)
        assert m, (
            f"{_POWER_DOC.name} no longer states {param} in a readable form — "
            "either the doc changed shape or the number was dropped; look, do "
            "not relax this pattern")
        assert float(m.group(1)) == BOARD[param], (
            f"{_POWER_DOC.name} says {param}={m.group(1)}, preflight enforces "
            f"{BOARD[param]}")


def test_the_voltage_only_gauge_stays_selected() -> None:
    """The two pins that keep coulomb counting shut. `BAT1_CAPACITY <= 0` picks
    PX4's voltage-only branch; `raw_telemetry_port: 0` keeps the only path that
    ever fills `battery_consumed_mah` switched off. An avionics-only coulomb
    counter reads a pack that never empties — it fails OPTIMISTIC, which is the
    one direction a battery gauge must never fail."""
    assert BOARD["BAT1_CAPACITY"] <= 0
    for path in _CONFIGS:
        conn = _cfg(path).get("connection") or {}
        assert conn.get("raw_telemetry_port", 0) == 0, (
            f"{path.name}: raw telemetry on — consumed-mAh would start flowing "
            "from a power module that cannot see the motors")


def test_the_endpoints_are_a_plausible_pack_not_a_typo() -> None:
    """Cheap arithmetic against the operator's own spec (25.1 V full,
    ~22.6 V empty on 6S). Catches a decimal slip that every equality check
    above would happily agree on."""
    cells = BOARD["BAT1_N_CELLS"]
    full, empty = BOARD["BAT1_V_CHARGED"] * cells, BOARD["BAT1_V_EMPTY"] * cells
    assert 24.5 <= full <= 25.5, f"pack full = {full:.2f} V"
    # empty floor 22.0 -> 20.0 across 2026-08-29/30: V_EMPTY 3.77 -> 3.65 -> 3.40
    # (20.4 V), each step measured from the Bang Bo parallel-pack ULogs — see
    # tools/preflight_params.py BOARD. Below 20.0 V (3.33 V/cell) a 6S LiPo IS
    # empty, so that stays the floor of the band.
    assert 20.0 <= empty <= 23.0, f"pack empty = {empty:.2f} V"
    assert full - empty >= 1.5, "usable voltage band collapsed"
