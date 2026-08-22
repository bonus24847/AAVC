"""Static audit: every PX4 param this repo pushes must use the right setter type.

Why this tool exists — the same bug shipped TWICE before it was caught:

* ``EKF2_HGT_REF`` (INT32 on the FC) was pushed through ``set_param_float`` on
  every connect for its whole life. PX4 rejects a float write to an INT32
  param outright, so the log said ``EKF2_HGT_REF=1 failed: TIMEOUT`` on every
  flight and the pin NEVER reached the board (found 2026-08-20, live at the
  KMUTNB field).
* ``MAV_1_FORWARD`` (boolean -> INT32) had the identical defect the whole
  time, sitting one entry below in the same dict. Its job is to carry the
  radio status beacon; had a param reset wiped it, the beacon would have gone
  silent with no error anywhere. THIS tool is what finally caught it.

The failure is invisible in flight logs ("failed: TIMEOUT" reads like a link
hiccup) and invisible to readback tools (``param_audit.py`` classifies these
reboot-latched params as "differs, but needs a REBOOT" — the true failure
wears an expected label). The only reliable detection is static: resolve the
declared type of every param we push from the PX4 source tree itself, and
assert both directions against ``_INT_PARAMS``:

* every param PX4 declares as INT32 (or boolean/enum/bitmask in a
  ``module.yaml``) MUST be listed in ``_INT_PARAMS``;
* everything listed in ``_INT_PARAMS`` must really be INT32 (a stale entry
  would silently break a float pin the same way).

Usage (no board, no network — pure source scan):

    .venv/bin/python tools/px4_type_audit.py
    .venv/bin/python tools/px4_type_audit.py --px4 ~/PX4-Autopilot-v1.17 \
        --config sitl/aavc_config.yaml --config sitl/kmitl_config.yaml

Exit codes: 0 clean · 1 violations or unresolved params · 2 PX4 worktree
missing. Wired as ``make type-audit``; run it whenever a key is added to
``DEFAULT_PX4_TUNING``, any config ``px4_tuning``/``sim_battery``/``gimbal``
block, or after a PX4 worktree bump.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

_REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO))

from mavlink_adapter.commands import _INT_PARAMS, DEFAULT_PX4_TUNING  # noqa: E402

# module.yaml `type:` values that PX4's generate_params.py maps onto INT32
# (Tools/module_config/generate_params.py: boolean/enum/bitmask/int32 all
# emit PARAM_DEFINE_INT32).
_YAML_INT_TYPES = frozenset({"boolean", "enum", "bitmask", "int32"})
_YAML_FLOAT_TYPES = frozenset({"float"})

_PARAM_DEFINE_RE = re.compile(
    r"PARAM_DEFINE_(INT32|FLOAT)\s*\(\s*([A-Z0-9_]+)\s*,"
)
# A module.yaml key that names a param: uppercase/digits/underscore with an
# optional ${i} instance template anywhere in it. Rejects structural keys
# like `parameters:` or `group:` (lowercase).
_PARAM_NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*(?:\$\{i\}[A-Z0-9_]*)*$")


def collect_pushed_params(config_paths: list[Path]) -> dict[str, set[str]]:
    """Every param name the repo can push, mapped to where it came from."""
    pushed: dict[str, set[str]] = {}

    def add(name: str, source: str) -> None:
        pushed.setdefault(name, set()).add(source)

    for name in DEFAULT_PX4_TUNING:
        add(name, "DEFAULT_PX4_TUNING")
    for cfg_path in config_paths:
        cfg = yaml.safe_load(cfg_path.read_text()) or {}
        for block in ("px4_tuning", "sim_battery"):
            for name in (cfg.get(block) or {}):
                add(str(name), f"{cfg_path.name}:{block}")
        # The gimbal block nests its params one level down (gimbal.params).
        for name in ((cfg.get("gimbal") or {}).get("params") or {}):
            add(str(name), f"{cfg_path.name}:gimbal.params")
    return pushed


# A literal-name param write: ``set_param_int("NAV_RCL_ACT", 2)``. The generic
# ``set_param_float(name, ...)`` forms (which push DEFAULT_PX4_TUNING and the
# config blocks) carry a variable, not a literal, so they do not match — those
# are already covered by collect_pushed_params.
_HARDCODED_SETTER_RE = re.compile(
    r"set_param_(float|int)\(\s*[\"']([A-Z][A-Z0-9_]*)[\"']"
)
_SCAN_DIRS = ("orchestrator", "mavlink_adapter", "dashboard", "vision",
              "mission_brain", "tools", "cm4", "sitl")


def collect_hardcoded_setters(repo: Path) -> dict[str, dict[str, set[str]]]:
    """Param writes with the name spelled INLINE, mapped to setter -> sites.

    These bypass ``_INT_PARAMS`` entirely: the caller picked the setter by hand
    at the call site, so a wrong choice cannot be caught by the dict audit
    above — and it fails exactly the way ``EKF2_HGT_REF`` did, with a TIMEOUT
    that reads like a link hiccup. There are eleven of them and they are the
    failsafe chain: GF_ACTION, NAV_RCL_ACT, NAV_DLL_ACT, COM_LOW_BAT_ACT and
    the battery thresholds — i.e. the writes whose silent failure leaves the
    aircraft flying with PX4's defaults where the design assumed ours."""
    found: dict[str, dict[str, set[str]]] = {}
    for sub in _SCAN_DIRS:
        root = repo / sub
        if not root.is_dir():
            continue
        for py in sorted(root.rglob("*.py")):
            try:
                lines = py.read_text(errors="ignore").splitlines()
            except OSError:
                continue
            for n, line in enumerate(lines, 1):
                for setter, name in _HARDCODED_SETTER_RE.findall(line):
                    entry = found.setdefault(name, {"float": set(), "int": set()})
                    entry[setter].add(f"{py.relative_to(repo)}:{n}")
    return found


def _walk_module_yaml(node: object, out: dict[str, str]) -> None:
    if not isinstance(node, dict):
        if isinstance(node, list):
            for item in node:
                _walk_module_yaml(item, out)
        return
    for key, value in node.items():
        if (
            isinstance(key, str)
            and isinstance(value, dict)
            and "type" in value
            and _PARAM_NAME_RE.fullmatch(key)
        ):
            ptype = str(value.get("type", "")).lower()
            if ptype in _YAML_INT_TYPES:
                out[key] = "INT32"
            elif ptype in _YAML_FLOAT_TYPES:
                out[key] = "FLOAT"
            # unknown types are skipped: resolution then fails loudly below
        _walk_module_yaml(value, out)


def build_type_index(px4_root: Path) -> dict[str, str]:
    """param name (possibly ${i}-templated) -> INT32|FLOAT from the PX4 tree."""
    index: dict[str, str] = {}
    src = px4_root / "src"
    for c_file in list(src.rglob("*.c")) + list(src.rglob("*.cpp")):
        try:
            text = c_file.read_text(errors="ignore")
        except OSError:
            continue
        if "PARAM_DEFINE_" not in text:
            continue
        for kind, name in _PARAM_DEFINE_RE.findall(text):
            index[name] = "INT32" if kind == "INT32" else "FLOAT"
    # Param declarations live in module.yaml AND satellite files (EKF2 splits
    # them across params_*.yaml) — scan every yaml; the walker only accepts
    # dicts with a `type:` under an uppercase param-shaped key.
    for yaml_file in src.rglob("*.yaml"):
        try:
            data = yaml.safe_load(yaml_file.read_text())
        except (OSError, yaml.YAMLError):
            continue
        _walk_module_yaml(data, index)
    return index


def template_variants(name: str) -> list[str]:
    """Instance-templated lookups: MAV_1_FORWARD -> MAV_${i}_FORWARD, etc."""
    variants: list[str] = []
    runs = list(re.finditer(r"\d+", name))
    for match in runs:  # one run at a time
        variants.append(name[: match.start()] + "${i}" + name[match.end():])
    if len(runs) > 1:  # then all at once
        variants.append(re.sub(r"\d+", "${i}", name))
    return variants


def resolve_type(name: str, index: dict[str, str]) -> str | None:
    if name in index:
        return index[name]
    for variant in template_variants(name):
        if variant in index:
            return index[variant]
    return None


def run_audit(px4_root: Path, config_paths: list[Path]) -> int:
    if not (px4_root / "src").is_dir():
        print(f"ERROR: PX4 worktree not found at {px4_root} (need its src/)")
        return 2

    pushed = collect_pushed_params(config_paths)
    index = build_type_index(px4_root)

    violations: list[str] = []
    unresolved: list[str] = []
    int_count = 0
    for name in sorted(pushed):
        ptype = resolve_type(name, index)
        if ptype is None:
            unresolved.append(
                f"UNRESOLVED  {name}  (pushed from {', '.join(sorted(pushed[name]))}) "
                f"— not declared anywhere in {px4_root}/src"
            )
        elif ptype == "INT32":
            int_count += 1
            if name not in _INT_PARAMS:
                violations.append(
                    f"VIOLATION   {name} is INT32 on the FC but NOT in _INT_PARAMS "
                    f"— the float push is rejected on every connect "
                    f"(pushed from {', '.join(sorted(pushed[name]))})"
                )
    hardcoded = collect_hardcoded_setters(_REPO)
    for name in sorted(hardcoded):
        ptype = resolve_type(name, index)
        want = "float" if ptype == "FLOAT" else "int"
        if ptype is None:
            unresolved.append(
                f"UNRESOLVED  {name}  (hardcoded setter at "
                f"{', '.join(sorted(hardcoded[name]['float'] | hardcoded[name]['int']))}) "
                f"— not declared anywhere in {px4_root}/src"
            )
            continue
        wrong = hardcoded[name]["int" if want == "float" else "float"]
        if wrong:
            violations.append(
                f"VIOLATION   {name} is {ptype} on the FC but written with "
                f"set_param_{'int' if want == 'float' else 'float'} at "
                f"{', '.join(sorted(wrong))} — PX4 rejects the write and the pin "
                f"never lands (the failure logs as a TIMEOUT)"
            )

    for name in sorted(_INT_PARAMS):
        ptype = resolve_type(name, index)
        if ptype == "FLOAT":
            violations.append(
                f"VIOLATION   {name} is in _INT_PARAMS but PX4 declares it FLOAT "
                f"— the int push is rejected the same way"
            )

    for line in violations + unresolved:
        print(line)
    print(
        f"[type-audit] {len(pushed)} pushed params · {int_count} INT32 · "
        f"{len(_INT_PARAMS)} in _INT_PARAMS · "
        f"{len(hardcoded)} hardcoded setter(s) · "
        f"{len(violations)} violation(s) · {len(unresolved)} unresolved"
    )
    if violations or unresolved:
        return 1
    print("[type-audit] ✔ every pushed param uses the setter PX4 will accept")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--px4", type=Path, default=Path.home() / "PX4-Autopilot-v1.17",
        help="PX4 source worktree (default: ~/PX4-Autopilot-v1.17)",
    )
    parser.add_argument(
        "--config", type=Path, action="append",
        help="mission config yaml (repeatable; default: both site configs)",
    )
    args = parser.parse_args()
    configs = args.config or [
        _REPO / "sitl" / "aavc_config.yaml",
        _REPO / "sitl" / "kmitl_config.yaml",
    ]
    return run_audit(args.px4, configs)


if __name__ == "__main__":
    raise SystemExit(main())
