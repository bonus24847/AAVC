#!/usr/bin/env python3
"""Block until PX4 SITL is actually ready to fly, then exit 0 (else exit 1).

The launchers used to gate on the string "home set" appearing in the PX4 console
log. That message only shows up once something connects to the vehicle and
interacts with it — but the launcher does not start the orchestrator until the
gate passes, so on a quiet boot it waits forever, decides the gz_bridge spawn
raced, kills a perfectly healthy SITL and retries. Gate on the real condition
instead: a PX4 heartbeat plus a 3D GPS fix and a valid global position.

Exit codes are what the launchers branch on, so they distinguish "not ready yet"
from "this probe could not run at all" — treating a crash as a slow boot is how
a healthy SITL gets killed and retried five times:
    0  ready
    1  not ready before the timeout
    2  the probe itself failed (bad endpoint, port already held, missing dep)

Usage:  wait_sitl_ready.py [--timeout S] [--connect udpin:0.0.0.0:14540]

Note the SINGLE colon. MAVSDK's `udpin://host:port` form — the one CLAUDE.md
documents for the orchestrator — is not what pymavlink parses: it reads the host
as "//0.0.0.0" and dies with a name-resolution error.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from pymavlink import mavutil

GCS_HEARTBEAT_HZ = 2.0


def _safe_recv(conn, **kw):
    """recv_match that tolerates pymavlink's instanced-message bookkeeping bug
    (it raises TypeError on some PX4 1.17 messages instead of returning)."""
    try:
        return conn.recv_match(**kw)
    except TypeError:
        return None


def _beat(conn) -> None:
    """Announce ourselves as a GCS. PX4 reports 'no connection to the GCS' as a
    preflight failure otherwise, and a silent listener never clears it."""
    while True:
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
        except Exception:  # noqa: BLE001 — no peer address yet; keep trying
            pass
        time.sleep(1.0 / GCS_HEARTBEAT_HZ)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connect", default="udpin:0.0.0.0:14540")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    def say(msg: str) -> None:
        if not args.quiet:
            print(f"[wait-sitl] {msg}", flush=True)

    try:
        conn = mavutil.mavlink_connection(args.connect)
    except Exception as e:  # noqa: BLE001 — report it; do NOT look like a timeout
        print(f"[wait-sitl] cannot listen on {args.connect}: {e}",
              file=sys.stderr, flush=True)
        return 2
    threading.Thread(target=_beat, args=(conn,), daemon=True).start()

    deadline = time.monotonic() + args.timeout
    have_heartbeat = False
    fix_type = 0
    sats = 0
    have_position = False

    while time.monotonic() < deadline:
        msg = _safe_recv(conn, blocking=True, timeout=1.0)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            # Ignore other ground stations chattering on the same port.
            if (msg.type != mavutil.mavlink.MAV_TYPE_GCS
                    and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                if not have_heartbeat:
                    say("autopilot heartbeat")
                have_heartbeat = True
        elif kind == "GPS_RAW_INT":
            fix_type, sats = msg.fix_type, msg.satellites_visible
        elif kind == "GLOBAL_POSITION_INT":
            have_position = msg.lat != 0 or msg.lon != 0

        if have_heartbeat and fix_type >= 3 and have_position:
            say(f"ready — 3D fix, {sats} sats")
            conn.close()
            return 0

    say(f"TIMEOUT after {args.timeout:.0f}s "
        f"(heartbeat={have_heartbeat} fix={fix_type} sats={sats} pos={have_position})")
    conn.close()
    return 1


if __name__ == "__main__":
    sys.exit(main())
