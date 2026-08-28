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
eggs wired to the roll and yaw sticks. And since 2026-08-26 the MAVLink PORT
MAP (``MAV_0_CONFIG``/``MAV_1_CONFIG``): with TELEM2 unassigned the companion
cannot reach the FC at all, and that never reads as a wrong parameter — it
reads as a camera chip that will not light, a blank sensor strip, and a 🚀 that
refuses with ``unmet critical checks: link``.

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

**BOOT-LATCHED** — read from the field config in force (``.aavc_site``, or
``--config``), and checked as a BOARD-class STOP. ``EKF2_HGT_REF`` is the
member that matters: the EKF latches its height reference the first time any
source fuses, so the mission-start push cannot change the flight about to
happen — only the value the board BOOTED with counts. It has already cost a
flight (2026-08-20, GPS reference, 10.8 m of baro-vs-GPS divergence and a
phantom ceiling breach that RTH'd a healthy flight).

⚠ Run this over a WIDE link — the CM4 router or the FC's USB cable. The param
protocol is one request/response per name, and the ELRS radio carries about
440 B/s: over it every read times out and the tool used to answer
"✘ อย่าเพิ่งบิน — ต้องแก้ก่อน: PWM_MAIN_FUNC1, …" naming all twenty. The board
was perfect; the same run over the router read 19/19 correct four hours later.
A false alarm of that shape on competition morning is worse than no check at
all, so an unread param is now reported as UNREAD and never as wrong.

Exit code: 0 all BOARD params correct · 2 a BOARD value is WRONG (do not fly) ·
3 no link · 4 --strict and a PINNED value never landed · 5 could not READ some
BOARD params over this link (unverified, so still do not fly — but the fix is a
different link, not the board).
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

import yaml

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
    # battery: the FC is fed by a PM02D that powers ONLY the avionics
    # (2026-08-20; replaced the converter that replaced the failed PM03D) —
    # motors still run from a board the FC cannot sense, so the voltage-only
    # branch carries the gauge. The PM02D's ~0.7 A avionics current must NEVER
    # be allowed to feed coulomb counting: it reads a pack that never empties.
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
    # TELEM2 -> CM4 byte budget. 1200 B/s (the PX4 default) starves the
    # onboard stream set: PX4 scales EVERY stream down to fit, and on the
    # 2026-08-28 KMITL trial the battery reading reached the mission every
    # ~30 s (36 % held on the companion while the FC read 27 %), raw_gps
    # time arrived every 13-20 s, and per-rung MPC_Z_V_AUTO_DN param sets
    # timed out. 0 = "half the baud rate" (46 KB/s at 921600) — the value
    # PX4 documents for a companion link. REBOOT REQUIRED after setting.
    "MAV_1_RATE": 0,
    # The MAVLink PORT MAP — which instance serves which physical port. Nothing
    # in the flight stack writes these, and wrong they cost the whole day:
    # 2026-08-26 the board held ``MAV_0_CONFIG=0`` (TELEM1 disabled) and
    # ``MAV_1_CONFIG=101`` (the COMPANION's instance pointed at the RADIO), so
    # **TELEM2 — the CM4's own line — had no MAVLink instance at all**.
    # Consequences, all seen in one session and none of them naming the cause:
    #   * the CM4's mavlink-router opened /dev/ttyAMA0, wrote happily, and read
    #     ZERO bytes from the FC for an entire session. ``stat /dev/ttyAMA0``
    #     is the one-command proof — mtime advancing every second while atime
    #     stayed frozen at boot (the kernel moves a tty's atime only when a read
    #     RETURNS bytes). The orchestrator could not have flown the aircraft:
    #     🚀 would have died at ``unmet critical checks: link``.
    #   * ``cm4/status_beacon.py`` was writing ``AAVC cam=OK`` into a port
    #     nobody was listening to, so the GCS camera chip could never go green
    #     no matter what the camera did — three days of "the camera status does
    #     not show" that had nothing to do with the camera.
    #   * it put the companion's ONBOARD stream set on the RADIO, which is the
    #     2026-08-25 symptom: HIGHRES_IMU/ATTITUDE_QUATERNION/ODOMETRY flooding
    #     a 440 B/s link while SYS_STATUS never arrived and every sensor chip
    #     sat blank behind a green "online" badge.
    # The values are not a preference: every archived ULog of a flight that
    # actually flew carries 101/102 (2026-08-20 ``07_21_36`` + ``08_11_09``,
    # G7 2026-08-21 ``07_54_40``). Same shape as the PWM_MAIN_FUNC regression
    # above — a param reading 0 means a subsystem is unassigned, and nothing
    # anywhere says so out loud.
    "MAV_0_CONFIG": 101,              # TELEM1 = NOMAD radio     (MAV_0_MODE 0 Normal)
    "MAV_1_CONFIG": 102,              # TELEM2 = CM4 companion   (MAV_1_MODE 2 Onboard)
    # The HITL firmware has NO real actuator output (docs/HITL.md: reflash a
    # flight fw before G7) — a board still flagged SYS_HITL=1 is unflyable in
    # exactly the silent way this BOARD block exists to catch, and nothing at
    # mission start pins it back. Added 2026-08-20 (gap G-7).
    "SYS_HITL": 0,
    # Hover-thrust SEED. Measured true hover ≈0.60 (motors mean across the
    # 2026-08-20 flights) while the board shipped PX4's 0.5 default and the
    # hover-thrust estimator (MPC_USE_HTE=1) logged all-NaN — it never
    # converged in flights this short, so the SEED is what the takeoff ramp,
    # the land detector and every post-reset first flight actually fly on.
    # Written + read-back on the board 2026-08-21 (operator-approved).
    "MPC_THR_HOVER": 0.58,
}

# ── pushed by orchestrator/main.py at mission start; bench values are fine ──
PINNED: dict[str, float] = {
    # ⚠ PER-FIELD: 9.0 is the KMUTNB value (ceiling 10). The KMITL config pins
    # 25.0 (ceiling 30 / transit 20 / floor 10 since the 28-Aug-2026 briefing;
    # it was 19.5 under the 20 m ceiling), so this line reads "wrong" on a
    # competition board — PINNED is informational, so it prints rather than
    # stops, but do not "fix" the board to match it at KMITL.
    "RTL_RETURN_ALT": 9.0,
    # gross-runaway net only — PX4 1.17 moves home.alt in flight (2026-08-26)
    "GF_MAX_VER_DIST": 50.0,
    "GF_ACTION": 3,                   # 3 = Return. NEVER 2 (Hold) — see CLAUDE.md
    "EKF2_RNG_CTRL": 1,
    "EKF2_OF_CTRL": 0,                # no flow module in the kit
    "MPC_Z_V_AUTO_DN": 0.4,
    # 5 = yaw fixed. Listed here for a REASON beyond completeness: PX4's own
    # metadata for this param declares "@max 4" while its enum defines
    # "@value 5 yaw fixed" and FlightTaskAuto handles it — so any layer that
    # validates against the metadata (a GCS, a future param tool) would refuse
    # the value the mission needs, and a heading param that silently stays at
    # the factory 0 is what spun the camera through 867 deg in one 122 s flight
    # on 2026-08-20 (ULog 08_11_09; 1 of 457 frames decodable). Reading it back
    # after staging costs nothing and answers "did 5 actually land?".
    "MPC_YAW_MODE": 5,
    "COM_DISARM_LAND": -1,            # stay armed on a mid-flight pad landing
    "MAV_1_FORWARD": 1,               # CM4 -> radio STATUSTEXT
}

# ── params the EKF LATCHES AT BOOT: a mission-start push cannot fix them ──
#
# ``EKF2_HGT_REF`` decides which sensor the whole flight's altitude is measured
# against, and ``Ekf::checkHeightSensorRefFallback`` (EKF/height_control.cpp:61)
# returns early once any height source is fusing — the parameter is read only
# BEFORE that guard. So the value that matters is the one the board held when it
# BOOTED; ``apply_param_overrides`` writing it at mission start changes the next
# flight, not this one. That makes it a BOARD-class check even though the config
# carries it, and it is why it appears in neither list until now.
#
# It has already cost a flight. 2026-08-20 at KMUTNB, on ``EKF2_HGT_REF=1``
# (GPS): baro-vs-GPS divergence 10.8 m peak-to-peak, a "ceiling breach" at
# 12.0 m against a 10 m ceiling, and the watchdog RTH'd a flight that was
# tracking transit to 1.7-1.8 m. Flight 3 the same afternoon on ``=0`` (baro)
# flew transit 3/3 at 1.4-2.0 m with no altitude event at all.
#
# The EXPECTED value comes from the field config in force rather than a
# constant here, because the two fields disagree today and a check that is
# wrong at one of them is a check that gets ignored at both.
BOOT_LATCHED: tuple[str, ...] = ("EKF2_HGT_REF",)


def _active_config_path(explicit: str | None) -> Path | None:
    """The config this repo flies: ``--config``, else ``.aavc_site``'s."""
    if explicit:
        return Path(explicit)
    site = Path(__file__).resolve().parents[1] / ".aavc_site"
    if not site.exists():
        return None
    for line in site.read_text().splitlines():
        if line.strip().startswith("AAVC_CONFIG="):
            raw = line.split("=", 1)[1].strip().strip('"')
            # AAVC_CONFIG="${AAVC_CONFIG:-sitl/aavc_config.yaml}"
            if ":-" in raw:
                raw = raw.split(":-", 1)[1].rstrip("}").strip('"')
            return Path(__file__).resolve().parents[1] / raw
    return None


def boot_latched_expected(config_path: Path | None) -> dict[str, float]:
    """What the BOARD must already hold, read from the field config's own
    ``px4_tuning`` block. Empty when the config cannot be read — the check then
    simply does not run, rather than inventing a value to compare against."""
    if config_path is None or not config_path.exists():
        return {}
    try:
        cfg = yaml.safe_load(config_path.read_text()) or {}
    except Exception:  # noqa: BLE001 — a broken config is the caller's problem
        return {}
    tuning = cfg.get("px4_tuning") or {}
    return {k: float(tuning[k]) for k in BOOT_LATCHED if k in tuning}


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
# ``MAV_0_CONFIG``/``MAV_1_CONFIG`` are the newest members and earned it the
# hard way on 2026-08-26: both were written and read back CORRECT, and TELEM2
# stayed stone dead afterwards — PX4 assigns ports in ``rc.mavlink`` at boot and
# never revisits them. What finally moved it was cycling the PACK. Worth saying
# once because it cost an hour: **pulling the USB cable is not a reboot** while
# the pack is connected — the FC keeps running on pack power and only loses its
# USB link, so the "reboot" appears to happen and nothing changes. The proof
# that a port map is in EFFECT is never this readback; it is traffic arriving
# (``stat /dev/ttyAMA0`` atime moving, or this very tool answering over the
# router, which cannot happen unless TELEM2 is live).
_STORED_NOT_PROVEN = {"MAV_0_CONFIG", "MAV_1_CONFIG",
                      "MAV_1_FORWARD", "EKF2_HGT_REF"}


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
    flies on since the PM03D failed (the PM02D that now feeds the FC senses
    avionics draw only). Pinning it to exactly ``-1`` STOPped
    the flight over a ``0`` that flies fine — the runtime gate
    (``main.py``: ``if fc_capacity <= 0``) accepts any non-positive value, so
    the field-day check must too."""
    if name == "BAT1_CAPACITY":
        return have <= 0.0
    return abs(have - want) <= 1e-6 * max(1.0, abs(want))


def _report(title: str, expected: dict[str, float], got: dict[str, float],
            *, fatal: bool) -> tuple[list[str], list[str]]:
    """(wrong, unread) — kept apart on purpose.

    They demand opposite actions: a WRONG value is fixed on the board, an UNREAD
    one is re-checked over a link wide enough to carry the param protocol. Both
    used to land in one list under "ต้องแก้ก่อน", which sent the crew looking
    for a wiped motor map that was never wiped (2026-08-25, over the ELRS
    radio).
    """
    print(f"\n{title}")
    off: list[str] = []
    unread: list[str] = []
    for name, want in expected.items():
        have = got.get(name)
        if have is None:
            print(f"  ?  {name:18s} = (ไม่ตอบ — อ่านไม่ได้ ไม่ใช่ค่าผิด)")
            unread.append(name)
        elif _board_ok(name, have, float(want)):
            note = "  (เก็บค่าแล้ว — ต้องรีบูตถึงจะมีผลจริง)" \
                if name in _STORED_NOT_PROVEN else ""
            print(f"  ✔  {name:18s} = {have:g}{note}")
        else:
            note = "" if fatal else "  (mission เขียนทับตอนสตาร์ต — ไม่ต้องแก้)"
            print(f"  {'✘' if fatal else '·'}  {name:18s} = {have:g}"
                  f"  <- ควรเป็น {float(want):g}{note}")
            off.append(name)
    return off, unread


async def _run(endpoint: str, *, strict: bool = False,
               config: str | None = None) -> int:
    print(f"[preflight] {endpoint}")
    commander = DroneCommander(ConnectionConfig(system_address=endpoint))
    try:
        await commander.connect()
    except Exception as e:  # noqa: BLE001
        print(f"เชื่อมต่อไม่ได้ ({type(e).__name__}: {e}) — "
              "เสียบแพ็คแล้วหรือยัง / mavlink-router ขึ้นไหม")
        return 3

    cfg_path = _active_config_path(config)
    latched = boot_latched_expected(cfg_path)
    got = await _read(commander, list(BOARD) + list(latched) + list(PINNED))
    bad, unread = _report("ต้องถูกบนบอร์ดเอง (ไม่มีใครแก้ให้):", BOARD, got,
                          fatal=True)
    if latched:
        b2, u2 = _report(
            f"EKF ล็อกตอนบูต — push ตอนสตาร์ตไม่ทัน ({cfg_path.name}):",
            latched, got, fatal=True)
        bad += b2
        unread += u2
    elif cfg_path is not None:
        print(f"\n(ข้าม boot-latched: อ่าน {cfg_path} ไม่ได้)")
    pinned_off, pinned_unread = _report(
        "mission เขียนให้ตอนสตาร์ต (ค่าโต๊ะ = ปกติ):", PINNED, got, fatal=strict)

    print()
    # WRONG first: it is the only outcome that means the BOARD needs work.
    if bad:
        print(f"✘ อย่าเพิ่งบิน — ค่าบนบอร์ดผิด: {', '.join(bad)}")
        if unread:
            print(f"   (และอ่านไม่ได้อีก {len(unread)} ตัว — ดูด้านล่าง)")
        return 2
    if unread:
        asked = len(BOARD) + len(latched)
        print(f"✘ ตรวจไม่ครบ — อ่านไม่ได้ {len(unread)}/{asked} ตัว: "
              f"{', '.join(unread)}")
        if len(unread) >= max(3, asked // 2):
            # Wholesale silence is the LINK, not the board. Say so, because the
            # instinct on seeing twenty names is to go looking at the board.
            print("   ทั้งชุดเงียบพร้อมกัน = ลิงก์แคบเกินไปสำหรับ param protocol "
                  "(วิทยุ ELRS วัดได้ ~440 B/s) ไม่ใช่บอร์ดถูกล้าง")
            print("   รันใหม่ผ่านลิงก์ที่กว้างกว่า:")
            print("     CONNECT=\"serial:///dev/ttyACM0:115200\"   # สาย FC USB")
            print("     CONNECT=\"udpin://0.0.0.0:14540\"          # บน CM4 ผ่าน router")
        print("   ยังไม่ควรบิน: ค่าที่อ่านไม่ได้คือค่าที่ยังไม่ได้ตรวจ")
        return 5
    if strict and pinned_off:
        # Post-staging mode: once orchestrator/main.py has pushed the pins,
        # a PINNED value still off means the push failed (the "applied 0/24"
        # class — CLAUDE.md's stale-mavsdk_server story) and the flight would
        # run on PX4 defaults (RTL at 60 m, 1.5 m/s onto the pad).
        print(f"✘ --strict: PINNED ยังไม่ลง — {', '.join(pinned_off)} "
              "(push จาก orchestrator ล้มเหลว?)")
        return 4
    if strict and pinned_unread:
        # Same rule as BOARD above: --strict exists to PROVE the push landed,
        # and a value that would not read has not been proven either way.
        print(f"✘ --strict: อ่าน PINNED ไม่ได้ {len(pinned_unread)} ตัว: "
              f"{', '.join(pinned_unread)} — ยังพิสูจน์ไม่ได้ว่า push ลงจริง")
        return 5
    print("✔ พารามิเตอร์ฝั่งบอร์ดครบถูกต้อง — บินได้")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--connect", default="udpin://0.0.0.0:14540",
                    help="MAVLink endpoint (default: the router's offboard port; "
                         "serial:///dev/ttyAMA0:921600 talks to the FC directly)")
    ap.add_argument("--strict", action="store_true",
                    help="PINNED mismatches exit 4 — use AFTER the orchestrator "
                         "has staged (it pushes the pins at connect); at the "
                         "bench they are informational")
    ap.add_argument("--config",
                    help="field config whose px4_tuning holds the boot-latched "
                         "values to check (default: the one .aavc_site names)")
    args = ap.parse_args()
    return asyncio.run(_run(args.connect, strict=args.strict, config=args.config))


if __name__ == "__main__":
    sys.exit(main())
