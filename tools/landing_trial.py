"""Landing-precision bench (SITL-only tuning aid, NOT the scored mission).

Repeats the real terminal controller — ``acquire_and_land_drop`` (align rungs,
id-verified LAND gate, touchdown-gated release) — N times against one truth
pad and reports the touchdown-vs-truth scatter. This is the measurement rig
for the G6 OPEN item (touchdown drift under wind) and for A/B-ing the PX4
position/velocity gains: run it, change ``--set`` params, run it again.

Each cycle approaches from a randomized ~12 m offset so landings are
independent; the vehicle stays ARMED throughout (COM_DISARM_LAND=-1), exactly
like a mid-mission serve. Requires SITL + the camera bridge + spawned pads.

Usage:
    .venv/bin/python tools/landing_trial.py --pad-index 0 --n 8 \
        [--set MPC_XY_VEL_P_ACC=2.4 --set MPC_XY_VEL_D_ACC=0.3]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import yaml  # noqa: E402
from loguru import logger  # noqa: E402

from mavlink_adapter.commands import (  # noqa: E402
    DEFAULT_PX4_TUNING,
    ConnectionConfig,
    DroneCommander,
)
from mavlink_adapter.telemetry import TelemetrySubscriber  # noqa: E402
from mission_brain.live_plan import render_live_plan  # noqa: E402
from mission_brain.profile import load_profile  # noqa: E402
from mission_brain.schemas import Coordinate  # noqa: E402
from mission_brain.search_pattern import build_search_pattern  # noqa: E402
from orchestrator.state import OrchestratorMode, OrchestratorState  # noqa: E402
from orchestrator.tactical_align import AlignParams, acquire_and_land_drop  # noqa: E402
from tuning.gains_io import load_gains  # noqa: E402
from vision.projection import configure_cameras  # noqa: E402


def _connection_from_cfg(cc: dict) -> ConnectionConfig:
    """Same connection wiring the scored mission uses (drop servo/PWM/timeouts),
    so the bench actuates the vehicle exactly as the mission does."""
    kw: dict = {}
    if cc.get("system_address"):
        kw["system_address"] = str(cc["system_address"])
    for k in ("drop_servo_channel", "drop_servo_pwm_release",
              "drop_servo_pwm_hold", "drop_payload_count"):
        if k in cc:
            kw[k] = int(cc[k])
    if "drop_fallback_endpoint" in cc:
        kw["drop_fallback_endpoint"] = str(cc["drop_fallback_endpoint"])
    for k in ("connect_timeout_s", "arming_timeout_s"):
        if k in cc:
            kw[k] = float(cc[k])
    return ConnectionConfig(**kw)

_R = 6_378_137.0


def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = math.radians(lat2 - lat1) * _R
    de = math.radians(lon2 - lon1) * _R * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def _offset(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    return (lat + math.degrees(north_m / _R),
            lon + math.degrees(east_m / (_R * math.cos(math.radians(lat)))))


async def run_trial(args: argparse.Namespace) -> int:
    cfg = yaml.safe_load(Path(args.config).read_text())
    profile = load_profile("competition")
    cams = cfg.get("cameras") or {}
    if cams:
        configure_cameras(nadir=cams.get("nadir"))

    truth = json.loads(Path(args.truth).read_text())["targets"]
    pad = truth[args.pad_index]
    pad_id = int(pad["marker_id"])
    logger.info(f"[trial] pad {pad_id} truth=({pad['lat']:.7f},{pad['lon']:.7f}) "
                f"n={args.n}")

    commander = DroneCommander(_connection_from_cfg(cfg.get("connection") or {}))
    telem = TelemetrySubscriber(commander.system)
    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE,
        plan=render_live_plan(
            Coordinate(lat=pad["lat"], lon=pad["lon"]),
            build_search_pattern(cfg["search_area"],
                                 Coordinate(lat=pad["lat"], lon=pad["lon"]),
                                 sweep_alt_m=12.0),
            discovered=[], profile=profile),
        telemetry=telem.state,
    )

    await commander.connect()
    await telem.start()
    for _ in range(60):
        t = state.telemetry
        if not math.isnan(t.lat) and t.gps_fix_type >= 3:
            break
        await asyncio.sleep(0.5)

    # Fly the SAME vehicle the scored mission flies: config px4_tuning (outer
    # loop) + the sysid/autotune inner-loop gains the mission auto-applies —
    # THEN the --set A/B knob on top. Without the sysid gains the bench measured
    # a different inner loop than the mission, so the scatter didn't transfer.
    overrides: dict[str, float] = dict(cfg.get("px4_tuning") or DEFAULT_PX4_TUNING)
    tuned = load_gains()
    if tuned:
        overrides.update(tuned)
        logger.info(f"[trial] merged {len(tuned)} sysid inner-loop gains")
    for kv in args.set or []:
        k, v = kv.split("=", 1)
        overrides[k.strip()] = float(v)
    n_set = await commander.apply_param_overrides(overrides)
    logger.info(f"[trial] applied {n_set} params "
                f"({', '.join(args.set) if args.set else 'config px4_tuning + sysid'})")

    rng = random.Random(args.seed)
    errs: list[float] = []
    align = AlignParams(assigned_marker_id=pad_id, accept_radius_m=8.0)
    rc = 0
    try:
        for i in range(args.n):
            await commander.arm_and_takeoff(12.0)
            # Approach from a fresh random direction ~12 m out, like a real serve.
            ang = rng.uniform(0, 2 * math.pi)
            slat, slon = _offset(pad["lat"], pad["lon"],
                                 12.0 * math.cos(ang), 12.0 * math.sin(ang))
            await commander.goto(slat, slon, 12.0)
            await asyncio.sleep(6.0)
            res = await acquire_and_land_drop(
                commander, state,
                Coordinate(lat=pad["lat"], lon=pad["lon"]),
                stop_index=i, params=align)
            t = state.telemetry
            if not res.landed:
                logger.warning(f"[trial] cycle {i + 1}: no confirmed touchdown "
                               f"({'; '.join(res.notes)})")
                continue
            err = _dist_m(t.lat, t.lon, float(pad["lat"]), float(pad["lon"]))
            errs.append(err)
            logger.info(f"[trial] cycle {i + 1}/{args.n}: touchdown err={err:.2f} m "
                        f"(align lock {res.final_error_m:.2f} m)")
        rc = 0 if errs else 1   # a zero-landing config is a FAILURE, not success
    except (asyncio.CancelledError, KeyboardInterrupt):
        logger.warning("[trial] interrupted — parking + cleaning up")
        rc = 130
    finally:
        # Cleanup MUST survive cancellation: BaseException (incl. CancelledError)
        # is caught around each await, and commander.close() (sync) ALWAYS runs —
        # skipping it leaves mavsdk_server + its non-daemon logging thread alive
        # and the process deadlocks at shutdown (the reason close() exists).
        try:
            await commander.goto(pad["lat"], pad["lon"], 12.0)
            await asyncio.sleep(2.0)
            await commander.land()
        except BaseException:  # noqa: BLE001 — parking must not block teardown
            pass
        try:
            await telem.stop()
        except BaseException:  # noqa: BLE001
            pass
        commander.close()
        if errs:
            errs.sort()
            # Nearest-rank p90: index ceil(0.9·n)-1 (int(0.9·n) is one rank high
            # whenever 0.9·n is integral — n = 10, 20, 30, the round bench counts).
            p90 = errs[max(0, math.ceil(0.9 * len(errs)) - 1)]
            print(f"\n[trial] pad {pad_id} n={len(errs)} touchdown-vs-truth: "
                  f"median={statistics.median(errs):.2f} m  "
                  f"mean={statistics.mean(errs):.2f} m  p90={p90:.2f} m  "
                  f"max={max(errs):.2f} m")
            print(f"[trial] all: {[round(e, 2) for e in errs]}")
        else:
            print("\n[trial] no successful landings recorded")
    return rc


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Repeated land-ON precision bench (SITL tuning aid)")
    ap.add_argument("--config", default=str(REPO / "sitl" / "aavc_config.yaml"))
    ap.add_argument("--truth", default="/tmp/aavc_targets.json")
    ap.add_argument("--pad-index", type=int, default=0,
                    help="index into the truth targets list")
    ap.add_argument("--n", type=int, default=8, help="landing cycles")
    ap.add_argument("--seed", type=int, default=1,
                    help="approach-direction randomization seed")
    ap.add_argument("--set", action="append", default=[],
                    metavar="PARAM=VALUE",
                    help="PX4 param override(s) on top of config px4_tuning "
                         "(repeatable) — the A/B knob")
    args = ap.parse_args()
    raise SystemExit(asyncio.run(run_trial(args)))


if __name__ == "__main__":
    main()
