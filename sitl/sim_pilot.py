"""SITL stand-in for the SAFETY PILOT's RC — drives the RC-GO conops headlessly.

RC-GO (operator 2026-08-12): the web/ssh GO only STAGES a flight; the pilot
ARMS via RC and flips to OFFBOARD to launch, and flips to POSCTL to take the
aircraft back (orchestrator stands down, audit ``PILOT TAKEOVER``). In SITL
there is no RC, so this script performs those exact actions over plain
MAVLink on the SITL GCS link.

Bind note: default endpoint is ``udpin:0.0.0.0:14550`` — run it INSTEAD of
the AAVC GCS console (one process per port), or point it elsewhere with
``--url``. NOT used on the real bird: there the pilot's Nomad does this.

Usage (venv python, PYTHONPATH scrubbed like every orchestrator entry point):
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py go        # arm + OFFBOARD
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py posctl    # pilot takeover
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py arm|offboard
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def _connect(url: str) -> mavutil.mavfile:
    m = mavutil.mavlink_connection(url)
    print(f"[sim_pilot] waiting for heartbeat on {url} …")
    m.wait_heartbeat(timeout=30)
    if m.target_system == 0:
        raise SystemExit("[sim_pilot] no heartbeat — is SITL up?")
    print(f"[sim_pilot] vehicle: sys {m.target_system}")
    return m


def _ack(m: mavutil.mavfile, what: str) -> None:
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    result = getattr(ack, "result", None)
    print(f"[sim_pilot] {what}: ack={result if ack else 'TIMEOUT'}")
    if not ack or result != 0:   # MAV_RESULT_ACCEPTED
        raise SystemExit(1)


def _arm(m: mavutil.mavfile) -> None:
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    _ack(m, "arm")


def _mode(m: mavutil.mavfile, name: str) -> None:
    mapping = m.mode_mapping()
    if not mapping or name not in mapping:
        raise SystemExit(f"[sim_pilot] mode {name} not in mapping "
                         f"{sorted(mapping or {})}")
    m.set_mode(mapping[name])
    _ack(m, f"mode {name}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("arm", "offboard", "posctl", "go"))
    p.add_argument("--url", default="udpin:0.0.0.0:14550")
    p.add_argument("--arm-to-offboard-s", type=float, default=2.0,
                   help="'go' only: pilot's pause between arming and the "
                        "OFFBOARD flip")
    a = p.parse_args()

    m = _connect(a.url)
    if a.action == "arm":
        _arm(m)
    elif a.action == "offboard":
        _mode(m, "OFFBOARD")
    elif a.action == "posctl":
        _mode(m, "POSCTL")
    else:  # go — the full RC-GO launch gesture
        _arm(m)
        time.sleep(a.arm_to_offboard_s)
        _mode(m, "OFFBOARD")
    return 0


if __name__ == "__main__":
    sys.exit(main())
