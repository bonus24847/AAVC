#!/usr/bin/env python3
"""Configure a real Pixhawk 6X for HITL — via the PX4 nsh shell (NOT PARAM_SET).

WHY the shell and not a MAVLink PARAM_SET / QGC: this 6X firmware stores PARAM_SET
byte-wise — a float 1.0 sent with type INT32 lands as 1065353216 (=0x3F800000). It
round-trips on read so it LOOKS fine but is garbage. The nsh `param set` parses by
the param's NATIVE type, so it is the only reliable path (found the hard way on the
bench 2026-06-15). This drives PX4's SERIAL_CONTROL "shell" device exactly like
PX4's own Tools/mavlink_shell.py.

Sequence (AAVC V1.3 HITL, see docs/HITL.md §1):
  1. param set SYS_AUTOSTART 1001   (airframe "HIL Quadcopter X")
  2. param save ; reboot            (airframe only takes on reboot)
  3. param set SYS_HITL 1           (enable HITL)
     param set COM_RC_IN_MODE 0     (RC Transmitter ON — safety-pilot DBR4 live)
     param set NAV_RCL_ACT 1        (RC-loss → Hold; test it with the real RC)
     param set COM_RCL_EXCEPT 4     (Offboard tolerates a momentary RC gap)
     param save
  4. verify SYS_HITL / SYS_AUTOSTART / the RC block read back correctly.

The orchestrator still pushes the runtime tuning (MPC_*/MC_*/geofence/RTL_RETURN_ALT/
COM_DISARM_LAND) over MAVLink at mission start — nothing else to preload here.

Usage:
    .venv/bin/python sitl/hitl_param_config.py --serial /dev/ttyACM0 --baud 921600
    .venv/bin/python sitl/hitl_param_config.py --connect udpin:0.0.0.0:14540   # via router
    .venv/bin/python sitl/hitl_param_config.py --dry-run                        # print the plan
"""

from __future__ import annotations

import argparse
import sys
import time

# ── the AAVC V1.3 HITL param plan ────────────────────────────────────────────
_AIRFRAME = ("SYS_AUTOSTART", 1001)          # set first, needs a reboot to take
_POST_REBOOT = [                             # set after the airframe reboot
    ("SYS_HITL", 1),
    ("COM_RC_IN_MODE", 0),
    ("NAV_RCL_ACT", 1),
    ("COM_RCL_EXCEPT", 4),
]
_VERIFY = ["SYS_HITL", "SYS_AUTOSTART", "COM_RC_IN_MODE",
           "NAV_RCL_ACT", "COM_RCL_EXCEPT"]


def _print_plan() -> None:
    print("# nsh commands this tool will send (run them by hand if you prefer):")
    print(f"param set {_AIRFRAME[0]} {_AIRFRAME[1]}")
    print("param save")
    print("reboot")
    print("#   … wait for reboot, reconnect …")
    for name, val in _POST_REBOOT:
        print(f"param set {name} {val}")
    print("param save")
    for name in _VERIFY:
        print(f"param show {name}")


class NshShell:
    """PX4 SERIAL_CONTROL 'shell' (device 10) over a mavutil connection."""

    _DEV_SHELL = 10
    # RESPOND | EXCLUSIVE | MULTI
    _FLAGS = 1 | 2 | 4

    def __init__(self, master):
        self.master = master
        self.mav = master.mav

    def send(self, line: str) -> None:
        data = (line + "\n").encode("ascii", "replace")
        while data:
            chunk, data = data[:70], data[70:]
            payload = chunk + b"\0" * (70 - len(chunk))
            self.mav.serial_control_send(
                self._DEV_SHELL, self._FLAGS, 0, 0, len(chunk), payload)

    def drain(self, seconds: float) -> str:
        """Collect shell output for `seconds`."""
        out, t_end = [], time.time() + seconds
        while time.time() < t_end:
            try:
                msg = self.master.recv_match(type="SERIAL_CONTROL", blocking=True,
                                             timeout=0.3)
            except TypeError:
                # pymavlink 2.4.49 raises from its instanced-message bookkeeping
                # on some PX4 1.17 messages instead of returning. Dropping the
                # message is right; aborting the bench session is not.
                continue
            if msg is None:
                continue
            n = msg.count if msg.count else len(msg.data)
            out.append(bytes(msg.data[:n]).decode("ascii", "replace"))
        return "".join(out)


def _connect(args):
    from pymavlink import mavutil
    dev = args.connect or args.serial
    print(f"[hitl-params] connecting to {dev}"
          f"{'' if args.connect else f' @ {args.baud}'} …")
    master = (mavutil.mavlink_connection(dev)
              if args.connect else
              mavutil.mavlink_connection(dev, baud=args.baud))
    master.wait_heartbeat(timeout=30)
    print(f"[hitl-params] heartbeat: system {master.target_system} "
          f"component {master.target_component}")
    return master


def main() -> int:
    ap = argparse.ArgumentParser(description="Configure a real 6X for HITL via nsh")
    ap.add_argument("--serial", default="/dev/ttyACM0",
                    help="6X serial device (USB CDC or a UART)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--connect", default=None,
                    help="MAVLink endpoint instead of --serial (e.g. udpin:0.0.0.0:14540)")
    ap.add_argument("--no-reboot", action="store_true",
                    help="skip the airframe reboot (only if SYS_AUTOSTART is already 1001)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the nsh commands and exit (no connection)")
    args = ap.parse_args()

    if args.dry_run:
        _print_plan()
        return 0

    try:
        from pymavlink import mavutil  # noqa: F401
    except ImportError:
        print("[hitl-params] ERROR: pymavlink not installed "
              "(pip install -e '.[dev]' or run under .venv)", file=sys.stderr)
        return 2

    master = _connect(args)
    sh = NshShell(master)

    # 1) airframe — needs a reboot to take effect.
    if not args.no_reboot:
        print(f"[hitl-params] set {_AIRFRAME[0]}={_AIRFRAME[1]} + save + reboot")
        sh.send(f"param set {_AIRFRAME[0]} {_AIRFRAME[1]}")
        print(sh.drain(1.0).strip())
        sh.send("param save")
        print(sh.drain(1.5).strip())
        sh.send("reboot")
        sh.drain(0.5)
        master.close()
        print("[hitl-params] rebooting — waiting 12 s then reconnecting …")
        time.sleep(12)
        master = _connect(args)
        sh = NshShell(master)

    # 2) HITL + RC block.
    for name, val in _POST_REBOOT:
        print(f"[hitl-params] set {name}={val}")
        sh.send(f"param set {name} {val}")
        print(sh.drain(0.8).strip())
    sh.send("param save")
    print(sh.drain(1.5).strip())

    # 3) verify — read every param back through the shell (native type).
    print("[hitl-params] ── verify ──")
    ok = True
    want = dict([_AIRFRAME, *_POST_REBOOT])
    for name in _VERIFY:
        sh.send(f"param show {name}")
        resp = sh.drain(0.8)
        print(f"  {name}: {resp.strip().splitlines()[-1] if resp.strip() else '<no reply>'}")
        # crude native check: the target value should appear on the line
        if str(want.get(name, "")) not in resp:
            ok = False
    master.close()

    if ok:
        print("[hitl-params] DONE — 6X configured for HITL. Start jMAVSim + the "
              "mission (docs/HITL.md §4).")
        return 0
    print("[hitl-params] WARNING: at least one param did not read back the expected "
          "value. Re-run, or set it by hand in the nsh shell (--dry-run prints the "
          "commands). Do NOT trust a MAVLink PARAM_SET for these.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
