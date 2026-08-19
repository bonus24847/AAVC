#!/usr/bin/env python3
"""Field-day FC parameter readback — the "before you fly" glance, in one command.

    .venv/bin/python tools/preflight_params.py                      # via the router
    .venv/bin/python tools/preflight_params.py --connect serial:///dev/ttyAMA0:921600

Complements ``tools/param_audit.py``, which sweeps the ``px4_tuning`` block the
mission pushes. This one asks about the params NOBODY pushes — the ones the
board must already hold, where being wrong means the aircraft cannot fly at all.
Two classes, and conflating them is what makes such a check get ignored:

**BOARD** — nothing in the flight stack writes these, so what the FC holds now
is what it will fly with. A mismatch is a STOP. This is where the motor map
lives: ``PWM_MAIN_FUNC1..6`` read **0** on 2026-08-16 (motors unassigned to
outputs, aircraft unflyable), cause never found, restored 2026-08-17. Treat it
as a regression that can recur — which is exactly why the list is read out loud
before every field day rather than trusted from a doc. The egg latches are here
too: they sat at 402/405/409/410 (RC passthrough) until 2026-08-17, i.e. two
eggs wired to the roll and yaw sticks.

**PINNED** — ``DroneCommander`` pushes these at every mission start and
read-back verifies the envelope ones (``verify_envelope_pins``). A parked board
holds bench state, not flight state, so PX4 defaults here are EXPECTED and
reported as notes, not failures. Note they are pushed by
``orchestrator/main.py``, NOT by ``connect()`` — connecting alone leaves them at
whatever the board had.

Reads through MAVSDK, the same stack the mission flies on. An earlier version
asked over raw pymavlink and, on the CM4's UART link with the GCS console also
attached, got silence from every request while MAVSDK on the same link answered
every one — a preflight check that reports "cannot read the motor map" on a
healthy aircraft is worse than none, because the next person learns to ignore it.

Exit code: 0 all BOARD params correct · 2 a BOARD mismatch (do not fly) ·
3 no link.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mavlink_adapter.commands import ConnectionConfig, DroneCommander  # noqa: E402

# ── what the FC must already hold, because nothing will fix it for us ──
BOARD: dict[str, float] = {
    # motor map — the 2026-08-16 regression (all six read 0 = unflyable)
    "PWM_MAIN_FUNC1": 101, "PWM_MAIN_FUNC2": 102, "PWM_MAIN_FUNC3": 103,
    "PWM_MAIN_FUNC4": 104, "PWM_MAIN_FUNC5": 105, "PWM_MAIN_FUNC6": 106,
    # egg latches — RC passthrough until 2026-08-17
    "PWM_AUX_FUNC1": 301, "PWM_AUX_FUNC2": 302,
    "PWM_AUX_FUNC3": 303, "PWM_AUX_FUNC4": 304,
    "SYS_AUTOSTART": 6001,            # Generic Hexarotor X
    "CA_ROTOR_COUNT": 6,
    # battery: the PM03D is out, so the voltage-only branch carries the gauge
    "BAT1_CAPACITY": -1,              # <=0 selects estimateStateOfCharge's else
    "BAT1_N_CELLS": 6,
    # 17000 mAh semi-solid endpoints (operator 2026-08-19: full 25.1 V, empty
    # ~22.6 V, 6S -> per cell 4.18 / 3.77). With BAT1_CAPACITY=-1 the whole
    # gauge is interpolate(cell_v, V_EMPTY, V_CHARGED), so a board still holding
    # the LiPo defaults (4.05 / 3.60) reads a wrong %. Set + read-back verified
    # on the board 2026-08-19.
    "BAT1_V_CHARGED": 4.18,
    "BAT1_V_EMPTY": 3.77,
    "SENS_TFMINI_CFG": 103,           # TFmini-S on TELEM3
}

# ── pushed by orchestrator/main.py at mission start; bench values are fine ──
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

# Params where reading the value back proves the board STORED it and nothing
# more — the module or estimator using it latched its copy at boot, so a value
# pushed since then reads correct while the old one is still in force. This
# tool exists to not mislead, and an unqualified ✔ next to one of these is
# exactly the kind of reassurance the GF_ACTION incident was made of (readback
# passed for months while the stored number meant "Hold", not "Return").
# ``MAV_1_FORWARD`` is reboot_required in PX4's own module.yaml —
# tools/param_audit.py has treated it that way since it shipped.
# ``EKF2_HGT_REF`` verified in the v1.17 source 2026-08-18:
# ``Ekf::checkHeightSensorRefFallback`` (EKF/height_control.cpp:61-66) returns
# early whenever ``_height_sensor_ref != HeightSensor::UNKNOWN``, and the
# parameter is only read AFTER that guard — so once any height source is
# fusing, the reference never consults the parameter again.
# NOT here, and deliberately: ``EKF2_RNG_CTRL`` reads fresh every update cycle
# (EKF/aid_sources/range_finder/range_height_control.cpp:130,131,141,145), so
# its readback IS trustworthy. Marking it would train the reader to ignore the
# marker, which is the only thing that makes the marker worth printing.
_STORED_NOT_PROVEN = {"MAV_1_FORWARD", "EKF2_HGT_REF"}


async def _read(commander: DroneCommander, names: list[str]) -> dict[str, float]:
    """Every param as a float, asking for BOTH storage types.

    MAVSDK's getters are type-strict: ``get_param_float`` on an INT32 param
    fails, and vice versa. Asking only one way made this tool report "(ไม่ตอบ)"
    for the entire motor map — a check crying that the aircraft is unflyable
    when every value was in fact correct. Which type a param uses is not
    memorable (``BAT1_CAPACITY`` is float, ``BAT1_N_CELLS`` is int), so ask
    both and let the board decide."""
    out: dict[str, float] = {}
    for name in names:
        for getter in (commander.get_param_int, commander.get_param_float):
            try:
                out[name] = float(await getter(name))
                break
            except Exception:  # noqa: BLE001,PERF203 — wrong type OR absent
                continue
    return out


def _board_ok(name: str, have: float, want: float) -> bool:
    """Whether a live reading passes. Relative-tolerance exact match for every
    param EXCEPT ``BAT1_CAPACITY``, where any value ``<= 0`` is correct: it
    selects PX4's voltage-only state-of-charge branch, the one this airframe
    flies on since the PM03D was removed. Pinning it to exactly ``-1`` STOPped
    the flight over a ``0`` that flies fine — the runtime gate
    (``main.py``: ``if fc_capacity <= 0``) accepts any non-positive value, so
    the field-day check must too."""
    if name == "BAT1_CAPACITY":
        return have <= 0.0
    return abs(have - want) <= 1e-6 * max(1.0, abs(want))


def _report(title: str, expected: dict[str, float], got: dict[str, float],
            *, fatal: bool) -> list[str]:
    print(f"\n{title}")
    off: list[str] = []
    for name, want in expected.items():
        have = got.get(name)
        if have is None:
            print(f"  ?  {name:18s} = (ไม่ตอบ)")
            off.append(name)
        elif _board_ok(name, have, float(want)):
            note = "  (เก็บค่าแล้ว — ต้องรีบูตถึงจะมีผลจริง)" \
                if name in _STORED_NOT_PROVEN else ""
            print(f"  ✔  {name:18s} = {have:g}{note}")
        else:
            note = "" if fatal else "  (mission เขียนทับตอนสตาร์ต — ไม่ต้องแก้)"
            print(f"  {'✘' if fatal else '·'}  {name:18s} = {have:g}"
                  f"  <- ควรเป็น {float(want):g}{note}")
            off.append(name)
    return off


async def _run(endpoint: str) -> int:
    print(f"[preflight] {endpoint}")
    commander = DroneCommander(ConnectionConfig(system_address=endpoint))
    try:
        await commander.connect()
    except Exception as e:  # noqa: BLE001
        print(f"เชื่อมต่อไม่ได้ ({type(e).__name__}: {e}) — "
              "เสียบแพ็คแล้วหรือยัง / mavlink-router ขึ้นไหม")
        return 3

    got = await _read(commander, list(BOARD) + list(PINNED))
    bad = _report("ต้องถูกบนบอร์ดเอง (ไม่มีใครแก้ให้):", BOARD, got, fatal=True)
    _report("mission เขียนให้ตอนสตาร์ต (ค่าโต๊ะ = ปกติ):", PINNED, got, fatal=False)

    print()
    if bad:
        print(f"✘ อย่าเพิ่งบิน — ต้องแก้ก่อน: {', '.join(bad)}")
        return 2
    print("✔ พารามิเตอร์ฝั่งบอร์ดครบถูกต้อง — บินได้")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--connect", default="udpin://0.0.0.0:14540",
                    help="MAVLink endpoint (default: the router's offboard port; "
                         "serial:///dev/ttyAMA0:921600 talks to the FC directly)")
    args = ap.parse_args()
    return asyncio.run(_run(args.connect))


if __name__ == "__main__":
    sys.exit(main())
