"""Pins for tools/px4_type_audit.py — the static param-type auditor.

The class of bug it guards against shipped twice (EKF2_HGT_REF, then
MAV_1_FORWARD, both 2026-08-20): an INT32 param pushed through
``set_param_float`` is rejected by PX4 on every connect, the log reads like a
link hiccup, and the pin never reaches the board. The integration test at the
bottom is the permanent regression pin: the audit of the REAL pushed set
against the REAL worktree must stay clean.
"""

from pathlib import Path

import pytest

from mavlink_adapter.commands import _INT_PARAMS
from tools.px4_type_audit import (
    _PARAM_DEFINE_RE,
    _walk_module_yaml,
    resolve_type,
    run_audit,
    template_variants,
)

_PX4 = Path.home() / "PX4-Autopilot-v1.17"
_REPO = Path(__file__).resolve().parent.parent


def test_param_define_regex_reads_both_kinds() -> None:
    text = (
        "PARAM_DEFINE_FLOAT(RTL_RETURN_ALT, 60.f);\n"
        "PARAM_DEFINE_INT32(COM_DISARM_LAND, 2);\n"
        "PARAM_DEFINE_INT32( GF_ACTION , 2 );\n"  # whitespace tolerated
    )
    found = dict((name, kind) for kind, name in _PARAM_DEFINE_RE.findall(text))
    assert found == {
        "RTL_RETURN_ALT": "FLOAT",
        "COM_DISARM_LAND": "INT32",
        "GF_ACTION": "INT32",
    }


def test_module_yaml_walker_maps_boolean_to_int32() -> None:
    # The exact shape that hid MAV_1_FORWARD: an instance-templated key with
    # type: boolean, nested a few levels down. Structural keys (lowercase)
    # must not be picked up even when they carry a `type` key.
    tree = {
        "module_name": "mavlink",
        "parameters": {
            "group": {
                "MAV_${i}_FORWARD": {"type": "boolean", "default": [True]},
                "MAV_${i}_RADIO_CTL": {"type": "boolean"},
                "EKF2_HGT_REF": {"type": "enum", "default": 1},
                "MPC_XY_CRUISE": {"type": "float"},
                "not_a_param": {"type": "boolean"},
            },
        },
    }
    out: dict[str, str] = {}
    _walk_module_yaml(tree, out)
    assert out == {
        "MAV_${i}_FORWARD": "INT32",
        "MAV_${i}_RADIO_CTL": "INT32",
        "EKF2_HGT_REF": "INT32",
        "MPC_XY_CRUISE": "FLOAT",
    }


def test_template_variants_cover_instanced_names() -> None:
    assert "MAV_${i}_FORWARD" in template_variants("MAV_1_FORWARD")
    # multiple digit runs: each alone, then all together
    variants = template_variants("BAT1_V_CHARGED")
    assert "BAT${i}_V_CHARGED" in variants


def test_resolve_type_falls_back_to_template() -> None:
    index = {"MAV_${i}_FORWARD": "INT32", "MPC_XY_CRUISE": "FLOAT"}
    assert resolve_type("MAV_1_FORWARD", index) == "INT32"
    assert resolve_type("MPC_XY_CRUISE", index) == "FLOAT"
    assert resolve_type("NO_SUCH_PARAM", index) is None


def test_mav_1_forward_is_pinned_as_int() -> None:
    # The 2026-08-20 fix itself: the beacon's forwarding pin must go through
    # the INT32 setter or it never reaches the board.
    assert "MAV_1_FORWARD" in _INT_PARAMS
    assert "EKF2_HGT_REF" in _INT_PARAMS


@pytest.mark.skipif(not (_PX4 / "src").is_dir(), reason="PX4 worktree absent")
def test_full_audit_is_clean_against_the_real_worktree() -> None:
    rc = run_audit(
        _PX4,
        [
            _REPO / "sitl" / "aavc_config.yaml",
            _REPO / "sitl" / "kmitl_config.yaml",
        ],
    )
    assert rc == 0
