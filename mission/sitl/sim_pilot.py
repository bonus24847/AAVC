"""SITL stand-in for the SAFETY PILOT's RC — drives the RC-GO conops headlessly.

RC-GO (operator 2026-08-12): the web/ssh GO only STAGES a flight; the pilot
ARMS via RC and flips to OFFBOARD to launch, and flips to POSCTL to take the
aircraft back (orchestrator stands down, audit ``PILOT TAKEOVER``). In SITL
there is no RC, so this script performs those actions over plain MAVLink.

Validated 2026-08-12 (three live SITL runs). Two lessons baked in:
  * Default endpoint is ``udpout:127.0.0.1:14280`` — PX4's ONBOARD-PAYLOAD
    mavlink instance — with commands blind-sent to sys 1. Confirmation comes
    from the ORCHESTRATOR's audit (``armed+OFFBOARD — launching`` / ``PILOT
    TAKEOVER``), which is the ground truth anyway. ⚠ Do NOT point this at
    the GCS instance (18570): PX4 partner-locks onto sim_pilot's ephemeral
    port and a real console on udpin:14550 goes PERMANENTLY silent until a
    PX4 restart (sibling-session finding 2026-08-14, validated live — on
    14280 the RC stream and a live console coexist cleanly). Binding
    udpin:14550 + wait_heartbeat is equally flaky for the same reason.
  * PX4 needs a live "RC": POSCTL (arming AND the in-flight flip) requires a
    fresh manual-control source, and the pinned NAV_DLL_ACT needs a GCS
    heartbeat to keep the vehicle armable. The ``rc`` action provides both —
    run it in the background for the WHOLE test (kill it afterwards to
    exercise the RC-loss → RTL failsafe).

Typical validation sequence (orchestrator already staged with --rc-go):
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py rc &   # RC is ON
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py go     # arm + OFFBOARD
    ...mid-flight...
    env -u PYTHONPATH .venv/bin/python sitl/sim_pilot.py posctl # takeover

NOT used on the real bird — there the pilot's Nomad does all of this.
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

# PX4 custom main modes (param2 of DO_SET_MODE with CUSTOM_MODE_ENABLED)
_PX4_MODES = {"POSCTL": 3.0, "OFFBOARD": 6.0}
_TARGET_SYS = 1


def _connect(url: str) -> mavutil.mavfile:
    m = mavutil.mavlink_connection(url, source_system=251)
    print(f"[sim_pilot] link {url} → blind target sys {_TARGET_SYS}")
    return m


def _arm(m: mavutil.mavfile, arm: bool = True) -> None:
    m.mav.command_long_send(
        _TARGET_SYS, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0)
    print(f"[sim_pilot] {'arm' if arm else 'disarm'} sent")


def _mode(m: mavutil.mavfile, name: str) -> None:
    m.mav.command_long_send(
        _TARGET_SYS, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        1.0, _PX4_MODES[name], 0.0, 0, 0, 0, 0)
    print(f"[sim_pilot] mode {name} sent — confirm via the orchestrator "
          "log/audit")


def _sticks(m: mavutil.mavfile) -> None:
    m.mav.manual_control_send(_TARGET_SYS, 0, 0, 500, 0, 0)  # neutral, mid-thr


def _rc_stream(m: mavutil.mavfile, duration_s: float) -> None:
    """Neutral sticks @10 Hz + GCS heartbeat @1 Hz — 'the RC is switched on'."""
    print(f"[sim_pilot] RC stream for {duration_s:.0f}s (Ctrl-C to cut RC)")
    t_end = time.monotonic() + duration_s
    last_hb = 0.0
    while time.monotonic() < t_end:
        _sticks(m)
        if time.monotonic() - last_hb >= 1.0:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = time.monotonic()
        time.sleep(0.1)
    print("[sim_pilot] RC stream ended — PX4 RC-loss failsafe takes it "
          "from here")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action",
                   choices=("rc", "arm", "disarm", "offboard", "posctl", "go"))
    p.add_argument("--url", default="udpout:127.0.0.1:14280",
                   help="PX4 mavlink endpoint — keep the onboard-payload "
                        "instance (14280); the GCS instance (18570) partner-"
                        "locks and silences a live console on 14550")
    p.add_argument("--arm-to-offboard-s", type=float, default=2.0,
                   help="'go' only: pilot's pause between arming and the "
                        "OFFBOARD flip")
    p.add_argument("--rc-s", type=float, default=600.0,
                   help="'rc' only: how long the RC stays on")
    a = p.parse_args()

    m = _connect(a.url)
    if a.action == "rc":
        _rc_stream(m, a.rc_s)
    elif a.action == "arm":
        _arm(m)
    elif a.action == "disarm":
        _arm(m, arm=False)
    elif a.action in ("offboard", "posctl"):
        # a burst of sticks first so the mode's manual-source check passes
        # even if the background 'rc' stream just hiccuped
        for _ in range(10):
            _sticks(m)
            time.sleep(0.1)
        _mode(m, a.action.upper())
    else:  # go — the full RC-GO launch gesture
        _arm(m)
        time.sleep(a.arm_to_offboard_s)
        _mode(m, "OFFBOARD")
    return 0


if __name__ == "__main__":
    sys.exit(main())
