"""Config values main.py must not take on trust: assigned ids, and the L&R point.

--assigned-ids (CLI) and mission.assigned_marker_ids (config) fed straight into
int() with no range check: an out-of-range id (-1, 7, 9) was silently accepted
and then never decoded, so the sortie burned window time deferring. Validate up
front against the competition id set with a clear error.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from loguru import logger

from orchestrator.main import _parse_assigned_ids, _resolve_site_origin


def test_valid_ids_parse_in_order() -> None:
    assert _parse_assigned_ids("3,1,4,6") == [3, 1, 4, 6]


def test_empty_is_empty() -> None:
    assert _parse_assigned_ids("") == []
    assert _parse_assigned_ids([]) == []


def test_list_input_from_config() -> None:
    assert _parse_assigned_ids([3, 1]) == [3, 1]


# 0 is a real pad since 2026-08-27 (the PDF figure encodes 1,2,0,4,5,6)
@pytest.mark.parametrize("bad", ["-1", "7", "9", "3,7", "-1,1"])
def test_out_of_range_rejected(bad: str) -> None:
    with pytest.raises(ValueError, match="valid"):
        _parse_assigned_ids(bad)


@pytest.mark.parametrize("bad", ["x", "3,x", "1.5"])
def test_non_integer_rejected(bad: str) -> None:
    with pytest.raises(ValueError):
        _parse_assigned_ids(bad)


def test_out_of_range_in_list_rejected() -> None:
    with pytest.raises(ValueError):
        _parse_assigned_ids([9])


# ── the L&R point: one place, two spellings, and they must agree ───────────
#
# 2026-08-22: sitl/kmitl_config.yaml shipped the PRACTICE field's whole site:
# block over the competition field's L&R — 31.5 km apart — while the comment on
# that line asserted they were equal. Nothing broke visibly, because the
# aircraft takes its home from GPS; what broke was every pad coordinate the
# console and the radio beacon carry, all anchored 31.5 km from where the
# console re-anchors them. main.py now reads the point the AIRCRAFT uses.

_KMITL_LR = [13.730322, 100.787446]
_KMUTNB_CENTER = {"center_lat": 13.8228032, "center_lon": 100.5116267}


def _errors(fn):
    """Run fn, returning the ERROR lines loguru emitted while it ran."""
    msgs: list[str] = []
    sink = logger.add(msgs.append, level="ERROR", format="{message}")
    try:
        result = fn()
    finally:
        logger.remove(sink)
    return result, msgs


def test_launch_recovery_wins_over_a_stale_site_block() -> None:
    cfg = {"ground_operation": {"launch_recovery": _KMITL_LR},
           "site": dict(_KMUTNB_CENTER)}
    got, errs = _errors(lambda: _resolve_site_origin(cfg))
    assert got == (13.730322, 100.787446)
    # and it must SAY SO: the SITL spawner and the Svelte dashboard still read
    # site.center, so resolving this quietly would fix one consumer of three.
    assert len(errs) == 1 and "31,518 m" in errs[0]


def test_agreeing_values_resolve_silently() -> None:
    """The practice config's shape: the same point written twice."""
    cfg = {"ground_operation": {"launch_recovery": [13.8228032, 100.5116267]},
           "site": dict(_KMUTNB_CENTER)}
    got, errs = _errors(lambda: _resolve_site_origin(cfg))
    assert got == (13.8228032, 100.5116267) and errs == []


def test_site_center_is_the_fallback_for_a_config_without_the_block() -> None:
    got, errs = _errors(lambda: _resolve_site_origin({"site": dict(_KMUTNB_CENTER)}))
    assert got == (13.8228032, 100.5116267) and errs == []


def test_a_config_naming_no_field_at_all_resolves_to_nothing() -> None:
    """None, not (0, 0): there is no sensible default for where the field is,
    and a silent origin at Null Island would draw every pad in the Atlantic."""
    for cfg in ({}, {"site": {}}, {"ground_operation": {"launch_recovery": []}}):
        assert _resolve_site_origin(cfg) is None


def test_both_shipped_configs_resolve_to_their_own_field() -> None:
    import yaml
    root = Path(__file__).resolve().parents[1]
    kmutnb = yaml.safe_load((root / "sitl" / "aavc_config.yaml").read_text())
    kmitl = yaml.safe_load((root / "sitl" / "kmitl_config.yaml").read_text())
    assert _resolve_site_origin(kmutnb) == (13.8228032, 100.5116267)
    assert _resolve_site_origin(kmitl) == (13.730322, 100.787446)


# ── PX4's battery simulator must not be written to real hardware ───────────
#
# The gate was `_is_sitl_endpoint`, whose own docstring says the premise is
# false in this repo: cm4/launch_flight.sh runs the REAL aircraft through a
# mavlink-router on udpin://0.0.0.0:14540, so the real bird looks like SITL.
# The 2026-08-20 logs show what that cost — three SIM_BAT_* TIMEOUTs and an
# "applied 0/3" line at every real mission start, ~9 s of the scored window,
# and an operator taught that "applied 0/N" is normal. `_detect_simulator` was
# written for exactly this and then never called.

class _FakeCommander:
    """Answers param reads the way a given autopilot would."""

    def __init__(self, *, has_sim_gz: bool, link_alive: bool = True) -> None:
        self.has_sim_gz = has_sim_gz
        self.link_alive = link_alive
        self.reads: list[str] = []

    async def get_param_int(self, name: str) -> int:
        self.reads.append(name)
        if not self.link_alive:
            raise TimeoutError("link dead")
        if name == "SIM_GZ_EN":
            if not self.has_sim_gz:
                raise RuntimeError("param not found")
            return 1
        return 4001


def _decide(commander: object, address: str) -> bool:
    import asyncio

    from orchestrator.main import _should_push_sim_battery

    return asyncio.run(_should_push_sim_battery(commander, address))


def test_real_hardware_behind_a_udp_router_is_not_treated_as_a_simulator() -> None:
    """The exact shape of the real aircraft: a UDP endpoint, no SIM_GZ_EN."""
    cmd = _FakeCommander(has_sim_gz=False)
    assert _decide(cmd, "udpin://0.0.0.0:14540") is False
    assert "SIM_GZ_EN" in cmd.reads


def test_sitl_still_gets_its_battery_simulator() -> None:
    cmd = _FakeCommander(has_sim_gz=True)
    assert _decide(cmd, "udpin://0.0.0.0:14540") is True


def test_a_dead_link_falls_back_to_the_endpoint_rather_than_guessing() -> None:
    """"Param absent" and "link dead" look identical from one failed read, so
    the detector returns None and the caller decides. Getting this wrong costs
    a param timeout on a battery simulator — never safety — which is why a
    fallback is acceptable HERE and would not be for a safety pin."""
    assert _decide(_FakeCommander(has_sim_gz=False, link_alive=False),
                   "udpin://0.0.0.0:14540") is True
    assert _decide(_FakeCommander(has_sim_gz=False, link_alive=False),
                   "serial:///dev/ttyAMA0:921600") is False
