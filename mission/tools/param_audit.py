#!/usr/bin/env python3
"""Read every px4_tuning key back off the flight controller — at the bench.

    .venv/bin/python tools/param_audit.py --connect serial:///dev/ttyAMA0:921600
    .venv/bin/python tools/param_audit.py            # SITL default endpoint

Why: the mission pushes ~30 parameters at start and `apply_param_overrides` is
best-effort by necessity — one unknown key must not abort a sortie. That
tolerance hides two failures until they matter:

  * a key this firmware does not HAVE (wrong PX4 version, a module not built
    into fmu-v6x). Each one silently burns a param timeout on the pad, inside
    the scored window, every flight.
  * a key it has but does not KEEP (rejected value, out of range, or a
    reboot_required param that will not take effect until the next boot).

`verify_envelope_pins` already read-checks the handful whose PX4 defaults would
fly an illegal mission — this is the wider, slower sweep meant for the bench,
where being wrong costs minutes instead of a flight.

Exit code: 0 = every key present and holding, 1 = something to look at. Nothing
here writes: it is safe to run on an armed-capable board (props off anyway).
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mavlink_adapter.commands import (  # noqa: E402
    DEFAULT_PX4_TUNING,
    ConnectionConfig,
    DroneCommander,
)

# Params PX4 only applies at module start; a mismatch here is expected right
# after a push and only means "reboot before you trust it".
_REBOOT_REQUIRED = {"MAV_1_FORWARD", "EKF2_HGT_REF"}


def _wanted(config: Path | None) -> dict[str, float]:
    if config is None:
        return dict(DEFAULT_PX4_TUNING)
    cfg = yaml.safe_load(config.read_text(encoding="utf-8")) or {}
    block = cfg.get("px4_tuning") or {}
    return {k: float(v) for k, v in block.items()} or dict(DEFAULT_PX4_TUNING)


async def _audit(endpoint: str, want: dict[str, float], tol: float) -> int:
    commander = DroneCommander(ConnectionConfig(system_address=endpoint))
    await commander.connect()
    missing: list[str] = []
    drifted: list[str] = []
    reboot: list[str] = []
    ok = 0
    for name in sorted(want):
        target = want[name]
        try:
            got = float(await commander.get_param_float(name))
        except Exception as e:  # noqa: BLE001 — absent and unreadable look alike
            missing.append(f"{name}: cannot read ({type(e).__name__})")
            continue
        # Relative tolerance: PX4 stores 32-bit floats, whose spacing grows with
        # magnitude (see verify_envelope_pins).
        if abs(got - target) <= tol * max(1.0, abs(target)) or (
                math.isnan(got) and math.isnan(target)):
            ok += 1
        elif name in _REBOOT_REQUIRED:
            reboot.append(f"{name}: board has {got:g}, config wants {target:g}")
        else:
            drifted.append(f"{name}: board has {got:g}, config wants {target:g}")

    print(f"\n{ok}/{len(want)} parameters present and holding")
    for title, rows in (("NOT READABLE on this firmware", missing),
                        ("NOT HOLDING the configured value", drifted),
                        ("differs, but needs a REBOOT to apply", reboot)):
        if rows:
            print(f"\n{title}:")
            for r in rows:
                print(f"  - {r}")
    if missing:
        print("\n  ^ each of these costs a param timeout at every mission start; "
              "drop them from px4_tuning or fix the name for this PX4 build.")
    return 0 if not (missing or drifted) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--connect", default="udpin://0.0.0.0:14540",
                    help="MAVLink endpoint (default: the SITL/router port)")
    ap.add_argument("--config", type=Path,
                    default=Path(__file__).resolve().parents[1] / "sitl/aavc_config.yaml",
                    help="config whose px4_tuning block is the reference")
    ap.add_argument("--tol", type=float, default=1e-3,
                    help="relative tolerance (default 1e-3)")
    args = ap.parse_args()
    want = _wanted(args.config if args.config.exists() else None)
    print(f"[audit] {len(want)} keys from {args.config} -> {args.connect}")
    try:
        return asyncio.run(_audit(args.connect, want, args.tol))
    except KeyboardInterrupt:
        return 1


if __name__ == "__main__":
    sys.exit(main())
