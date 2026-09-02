"""GPS/altitude stability gate — run after EVERY FC reboot, before staging.

Why: the GPS vertical solution at the KMUTNB field walks hard after a cold FC
start (measured 2026-08-20: the parked field read 12.7 → 36.6 → 13.3 m MSL
across one evening, with +1.6 m and +1.3 m EKF steps INSIDE a 30 s flight).
Staging while the frame is still moving flew the 8.5 m AGL transit command
into the ground (stale home cache, flight 1) and phantom-tripped the 10 m
ceiling watchdog (estimate steps, flight 2). The practice site now flies baro
height reference (EKF2_HGT_REF=0), which removes the step behaviour — this
gate remains the proof the frame is quiet before every launch, and the guard
if that reference ever moves again.

What it does: samples ``telemetry.position().absolute_altitude_m`` every
``--interval`` seconds and declares STABLE (exit 0) when 4 consecutive
samples span < 0.8 m with a usable GPS fix. Exits 1 at ``--timeout`` with the
last window printed.

Usage:
    .venv/bin/python tools/alt_watch.py [--connect …] [--interval 5]
        [--timeout 300]

⚠ PORT RULE: this binds the SAME udpin port the orchestrator uses (14540 by
default). Run it to completion BEFORE pressing GO — a copy left running while
a mission stages shares the port via SO_REUSE* and silently splits datagrams
with the orchestrator (mission-protocol transfers are what break first).
It exits by itself on STABLE; never background it past staging.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mavsdk import System  # noqa: E402

_WINDOW = 4          # consecutive samples…
_SPAN_M = 0.8        # …that must fit inside this band


async def _watch(endpoint: str, interval_s: float, timeout_s: float) -> int:
    drone = System()
    await drone.connect(system_address=endpoint)

    async def _wait_heartbeat() -> None:
        async for s in drone.core.connection_state():
            if s.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_heartbeat(), min(150.0, timeout_s))
    except asyncio.TimeoutError:
        print("[alt-watch] no heartbeat — FC still booting, or router down")
        return 1

    history: list[float] = []
    rounds = max(1, int(timeout_s / max(interval_s, 1.0)))
    for _ in range(rounds):
        try:
            async for p in drone.telemetry.position():
                alt = p.absolute_altitude_m
                break
            async for g in drone.telemetry.gps_info():
                sats, fix = g.num_satellites, str(g.fix_type)
                break
        except Exception as e:  # noqa: BLE001 — a dropped read is a data point
            print(f"[alt-watch] read error {type(e).__name__}")
            await asyncio.sleep(interval_s)
            continue
        history.append(alt)
        print(f"[alt-watch] MSL={alt:.1f} m  sats={sats} {fix}")
        window = history[-_WINDOW:]
        good_fix = any(k in fix for k in ("3D", "DGPS", "RTK"))
        if len(window) == _WINDOW and max(window) - min(window) < _SPAN_M \
                and good_fix:
            print(f"[alt-watch] STABLE at {alt:.1f} m MSL — GO for staging")
            print("[alt-watch] (port 14540 released — safe to press GO now)")
            return 0
        await asyncio.sleep(interval_s)

    window = history[-_WINDOW:]
    span = (max(window) - min(window)) if window else float("nan")
    print(f"[alt-watch] TIMEOUT after {timeout_s:.0f} s — last window span "
          f"{span:.1f} m (need < {_SPAN_M}). Frame still moving: wait longer "
          "before staging.")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--connect", default="udpin://0.0.0.0:14540",
                        help="MAVLink endpoint (default: the router's offboard "
                             "port; serial:///dev/ttyAMA0:921600 = direct FC)")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="seconds between samples (default 5)")
    parser.add_argument("--timeout", type=float, default=300.0,
                        help="give up after this many seconds (default 300)")
    args = parser.parse_args()
    return asyncio.run(_watch(args.connect, args.interval, args.timeout))


if __name__ == "__main__":
    raise SystemExit(main())
