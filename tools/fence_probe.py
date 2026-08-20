"""FC mission/dataman aliveness probe + SD-card flag check (field tool).

Why: on 2026-08-20 at KMUTNB the FC's mission/geofence path WEDGED — every
fence upload timed out (staging correctly refused to fly) while the param
path kept answering, on BOTH the router and a direct-serial connection. The
signature is precise: heartbeats/telemetry/params fine, mission-protocol
requests silent. Params live on FRAM, missions/fence go through dataman on
the SD — a wedged SD wedges exactly this half. The fix that worked was an FC
power-cycle. MAVFTP (also SD-backed) timed out the same way, which is the
corroborating symptom this tool checks second.

What it does (read-only — nothing is cleared or uploaded):
  1. ``mission_raw.download_geofence()`` with a 10 s guard. Silence/timeout →
     "WEDGED: power-cycle the FC", exit 1. An empty fence downloads fine and
     is NOT a wedge (staging uploads a fresh fence every GO).
  2. ``ftp.list_directory("/fs/microsd")`` with a 20 s guard. FTP being flaky
     on this board is documented (pymavlink note in CLAUDE.md G5), so an FTP
     failure alone is a WARN, never a "wedged" verdict. If the listing works,
     flag ``fault_*.log`` (PX4 refuses to ARM while a crash dump sits on the
     card — READ IT BEFORE DELETING, docs/evidence/hardfault_6X_2026-08-18)
     and ``param_import_fail.txt`` (mtd_params BSON corruption history — the
     SD backup is the only net under the motor map), exit 2.

Usage:
    .venv/bin/python tools/fence_probe.py [--connect udpin://0.0.0.0:14540]

⚠ Binds the same udpin port the orchestrator uses — run BEFORE staging, never
alongside a staged mission. Exit codes: 0 healthy · 1 mission path wedged ·
2 SD flags found · 3 no heartbeat.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mavsdk import System  # noqa: E402

_FENCE_TIMEOUT_S = 10.0
_FTP_TIMEOUT_S = 20.0
_SD_FLAGS = ("fault_", "param_import_fail")


async def _probe(endpoint: str) -> int:
    drone = System()
    await drone.connect(system_address=endpoint)

    async def _wait_heartbeat() -> None:
        async for s in drone.core.connection_state():
            if s.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_heartbeat(), 15.0)
    except asyncio.TimeoutError:
        print("[fence-probe] no heartbeat in 15 s — FC off, or router down")
        return 3
    print("[fence-probe] connected")

    # 1) mission-protocol aliveness (the 2026-08-20 wedge detector)
    try:
        fence = await asyncio.wait_for(
            drone.mission_raw.download_geofence(), _FENCE_TIMEOUT_S)
        print(f"[fence-probe] fence download OK ({len(fence)} item(s)) — "
              "mission/dataman path is alive")
    except Exception as e:  # noqa: BLE001 — any silence here IS the finding
        print(f"[fence-probe] fence download FAILED ({type(e).__name__}) — "
              "mission/dataman path WEDGED while params still answer: "
              "POWER-CYCLE THE FC, then wait for GPS to settle "
              "(tools/alt_watch.py) before staging")
        return 1

    # 2) SD-card flags (WARN-only transport: MAVFTP is documented-flaky here)
    try:
        listing = await asyncio.wait_for(
            drone.ftp.list_directory("/fs/microsd"), _FTP_TIMEOUT_S)
        names = [getattr(entry, "name", str(entry))
                 for entry in (getattr(listing, "dirs", []) or [])
                 + (getattr(listing, "files", []) or [])] \
            if hasattr(listing, "dirs") or hasattr(listing, "files") \
            else [getattr(entry, "name", str(entry)) for entry in listing]
        flagged = [n for n in names
                   if any(str(n).lstrip("DF/").startswith(f) for f in _SD_FLAGS)]
        if flagged:
            print(f"[fence-probe] ⚠ SD flags present: {', '.join(map(str, flagged))}")
            print("[fence-probe]   fault_*.log blocks ARMING — read/archive it "
                  "BEFORE deleting (procedure: PX4MASTER references/fc-params.md)")
            return 2
        print(f"[fence-probe] SD listing OK ({len(names)} entries, no flags)")
    except Exception as e:  # noqa: BLE001
        print(f"[fence-probe] WARN: SD listing failed ({type(e).__name__}) — "
              "MAVFTP is flaky on this board; not treated as a wedge "
              "(the fence download above already proved the mission path)")

    print("[fence-probe] ✔ mission path alive")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--connect", default="udpin://0.0.0.0:14540",
                        help="MAVLink endpoint (default: the router's offboard "
                             "port; serial:///dev/ttyAMA0:921600 = direct FC)")
    args = parser.parse_args()
    return asyncio.run(_probe(args.connect))


if __name__ == "__main__":
    raise SystemExit(main())
