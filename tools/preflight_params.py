"""Field-day FC parameter readback — the "before you fly" glance, in one command.

    .venv/bin/python tools/preflight_params.py                 # via a running router
    .venv/bin/python tools/preflight_params.py --serial /dev/ttyAMA0

Two classes of parameter, and conflating them is the whole point of this tool:

**BOARD** — nothing in the flight stack writes these. What the FC holds now is
what it will fly with, so a mismatch here is a STOP. This is where the motor map
lives: ``PWM_MAIN_FUNC1..6`` read **0** on 2026-08-16 (motors unassigned to
outputs, aircraft unflyable), cause never found, restored 2026-08-17. Treat it
as a regression that can recur — that is exactly why this list is read out loud
before every field day rather than trusted from a doc.

**PINNED** — ``DroneCommander`` pushes these at every connect and read-back
verifies the envelope ones (``verify_envelope_pins``). A parked board holds
bench state, not flight state, so PX4 defaults here are EXPECTED and reported
as notes, not failures (CLAUDE.md §G5 says the same in prose).

int32 params arrive bit-cast into PARAM_VALUE's float field — unpack them or
every integer reads 0.0, which looks exactly like the wiped motor map this tool
exists to catch. See the ``px4-param-int-bitcast`` note.

Exit code: 0 all BOARD params correct · 2 a BOARD mismatch (do not fly) ·
3 no heartbeat.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

from pymavlink import mavutil

# ── what the FC must already hold, because nothing will fix it for us ──
BOARD: dict[str, float] = {
    # motor map — the 2026-08-16 regression (all six read 0 = unflyable)
    "PWM_MAIN_FUNC1": 101, "PWM_MAIN_FUNC2": 102, "PWM_MAIN_FUNC3": 103,
    "PWM_MAIN_FUNC4": 104, "PWM_MAIN_FUNC5": 105, "PWM_MAIN_FUNC6": 106,
    # egg latches — these sat at 402/405/409/410 (RC passthrough) until
    # 2026-08-17, i.e. two eggs wired to the roll and yaw sticks
    "PWM_AUX_FUNC1": 301, "PWM_AUX_FUNC2": 302,
    "PWM_AUX_FUNC3": 303, "PWM_AUX_FUNC4": 304,
    "SYS_AUTOSTART": 6001,            # Generic Hexarotor X
    "CA_ROTOR_COUNT": 6,
    # battery: the PM03D is out, so the voltage-only branch carries the gauge
    "BAT1_CAPACITY": -1,              # <=0 selects estimateStateOfCharge's else
    "BAT1_N_CELLS": 6,
    "SENS_TFMINI_CFG": 103,           # TFmini-S on TELEM3
}

# ── pushed by DroneCommander at every connect; bench values are fine ──
PINNED: dict[str, float] = {
    "RTL_RETURN_ALT": 9.0,
    "GF_MAX_VER_DIST": 20.0,          # the rules' altitude fence
    "GF_ACTION": 3,                   # 3 = Return. NEVER 2 (Hold) — see CLAUDE.md
    "EKF2_RNG_CTRL": 1,
    "EKF2_OF_CTRL": 0,                # no flow module in the kit
    "MPC_Z_V_AUTO_DN": 0.4,
    "COM_DISARM_LAND": -1,            # stay armed on a mid-flight pad landing
    "MAV_1_FORWARD": 1,               # CM4 -> radio STATUSTEXT
}

_INT_TYPES = frozenset({
    mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
    mavutil.mavlink.MAV_PARAM_TYPE_INT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
    mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
})


def _decode(msg) -> float:
    """PARAM_VALUE carries int32 params bit-cast into a float field."""
    if msg.param_type in _INT_TYPES:
        return float(struct.unpack("<i", struct.pack("<f", msg.param_value))[0])
    return float(msg.param_value)


def read_params(conn, names: list[str], timeout_s: float = 25.0) -> dict[str, float]:
    """Ask for every name, re-asking the stragglers — a param request is a
    datagram, and the radio link drops them."""
    got: dict[str, float] = {}
    deadline = time.time() + timeout_s
    last_request = 0.0
    while time.time() < deadline and len(got) < len(names):
        if time.time() - last_request > 4:
            for name in names:
                if name not in got:
                    conn.mav.param_request_read_send(
                        conn.target_system, conn.target_component, name.encode(), -1)
            last_request = time.time()
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id in names and msg.param_id not in got:
            got[msg.param_id] = _decode(msg)
    return got


def _report(title: str, expected: dict[str, float], got: dict[str, float],
            *, fatal: bool) -> list[str]:
    print(f"\n{title}")
    off: list[str] = []
    for name, want in expected.items():
        have = got.get(name)
        if have is None:
            print(f"  ?  {name:18s} = (ไม่ตอบ)")
            off.append(name)
        elif abs(have - float(want)) <= 1e-6 * max(1.0, abs(float(want))):
            print(f"  ✔  {name:18s} = {have:g}")
        else:
            note = "" if fatal else "  (commander เขียนทับตอนต่อ — ไม่ต้องแก้)"
            print(f"  {'✘' if fatal else '·'}  {name:18s} = {have:g}"
                  f"  <- ควรเป็น {float(want):g}{note}")
            off.append(name)
    return off


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="udpin:127.0.0.1:14540",
                    help="MAVLink endpoint (default: the router's offboard port)")
    ap.add_argument("--serial", help="talk to the FC directly, e.g. /dev/ttyAMA0 "
                                     "(needs pyserial; the router path does not)")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    target = args.serial or args.url
    print(f"[preflight] {target}")
    conn = (mavutil.mavlink_connection(args.serial, baud=args.baud)
            if args.serial else mavutil.mavlink_connection(args.url))
    if not conn.wait_heartbeat(timeout=15):
        print("NO HEARTBEAT — FC ไม่ตอบ (เสียบแพ็คแล้วหรือยัง / router ขึ้นไหม)")
        return 3
    print(f"HEARTBEAT ok — sysid={conn.target_system}")

    names = list(BOARD) + list(PINNED)
    got = read_params(conn, names)

    bad = _report("ต้องถูกบนบอร์ดเอง (ไม่มีใครแก้ให้):", BOARD, got, fatal=True)
    _report("commander เขียนให้ตอนต่อ (ค่าโต๊ะ = ปกติ):", PINNED, got, fatal=False)

    print()
    if bad:
        print(f"✘ อย่าเพิ่งบิน — ต้องแก้ก่อน: {', '.join(bad)}")
        return 2
    print("✔ พารามิเตอร์ฝั่งบอร์ดครบถูกต้อง — บินได้")
    return 0


if __name__ == "__main__":
    sys.exit(main())
