#!/usr/bin/env python3
"""Read or set ONE PX4 parameter on the board through MAVSDK, with read-back.

    .venv/bin/python tools/board_param.py get MAV_1_RATE
    .venv/bin/python tools/board_param.py set MAV_1_RATE 0
    .venv/bin/python tools/board_param.py set BAT1_V_EMPTY 3.77 --float

Runs where MAVSDK can reach the FC — on the CM4 through the router's
offboard endpoint (the default ``udpin://0.0.0.0:14540``; the orchestrator
must NOT be running, it owns that port) or over a USB/serial URL. Integer
parameters need the plain form; ``--float`` selects ``set_param_float``.
Prints the value before and after; exit 0 only when the read-back matches.

Written 2026-08-28 for ``MAV_1_RATE`` (1200 -> 0): the KMITL trial showed the
TELEM2 stream set throttled to 1200 B/s — battery every ~30 s, sparse GPS
time, param sets timing out. ``MAV_*`` changes take effect after a REBOOT.
"""

from __future__ import annotations

import argparse
import asyncio
import sys


async def _main(args: argparse.Namespace) -> int:
    from mavsdk import System

    drone = System()
    await drone.connect(system_address=args.connect)
    async for st in drone.core.connection_state():
        if st.is_connected:
            break
    p = drone.param
    name = args.name.upper()

    async def read() -> float:
        return (await p.get_param_float(name)) if args.float else float(
            await p.get_param_int(name))

    before = await read()
    print(f"{name} = {before:g}")
    if args.cmd == "get":
        return 0
    want = float(args.value)
    if args.float:
        await p.set_param_float(name, want)
    else:
        await p.set_param_int(name, int(want))
    await asyncio.sleep(0.5)
    after = await read()
    ok = abs(after - want) < 1e-6
    note = "  ⚠ reboot the FC for MAV_* changes to apply" if ok and name.startswith("MAV_") else ""
    print(f"{name} -> {after:g}  ({'OK' if ok else 'MISMATCH'}){note}")
    return 0 if ok else 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("get", "set"))
    ap.add_argument("name")
    ap.add_argument("value", nargs="?")
    ap.add_argument("--float", action="store_true", help="a FLOAT parameter (default: INT32)")
    ap.add_argument("--connect", default="udpin://0.0.0.0:14540")
    args = ap.parse_args()
    if args.cmd == "set" and args.value is None:
        ap.error("set needs a value")
    return asyncio.run(_main(args))


if __name__ == "__main__":
    sys.exit(main())
