"""Rangefinder aliveness probe — does the TFmini-S actually DELIVER? (field tool)

Why: ``tools/preflight_params.py`` ticks ``SENS_TFMINI_CFG = 103`` green while
the sensor is stone dead, because the param check proves a value is STORED —
never that the hardware behind it works. That is the same trap GF_ACTION and
MAV_1_FORWARD sprang before it, and on 2026-08-25 it hid a lidar that PX4
could not see at all: SYS_STATUS LASER_POSITION present=False, zero
DISTANCE_SENSOR in 20 s, ``ALTITUDE.bottom_clearance`` all NaN — while every
board param read back correct and survived a reboot.

It is not rare. Counting only flights that actually left the ground with the
lidar configured, **5 of 11 (45%) flew with it dead the whole time**
(``12_12_22`` ``12_13_15`` ``12_35_16`` ``08_58_57`` ``07_22_22``) — and
``07_22_22`` sat between two good flights on the same day with identical
params and no error message anywhere. Nothing in the preflight caught any of
them.

That matters because ``EKF2_RNG_CTRL=1`` + ``EKF2_RNG_A_HMAX=7.0`` make this
sensor the height reference for exactly the slow pad approach and touchdown
the mission is scored on. Without it the last metres run on baro + GPS, which
were measured on 2026-08-25 wandering **1.11 m with the aircraft parked**.
When the lidar IS alive it is excellent — EKF innovation +0.005 ± 0.020 m,
test_ratio 0.004 of a 1.0 gate — so this is an availability problem, not an
accuracy one, and availability is what a gate can catch.

What it does (read-only — no params are written):
  1. ``set_rate_distance_sensor()`` to ask PX4 for the stream. NOTE an ACK
     here proves nothing: on 2026-08-25 the FC ACKed the equivalent
     SET_MESSAGE_INTERVAL with result 0 and then sent nothing at all, exactly
     as it does for ESC_STATUS on ESCs with no telemetry lead. Only received
     messages count.
  2. Counts ``distance_sensor()`` samples over ``--window`` seconds.

⚠ PASS/FAIL is on MESSAGE ARRIVAL, deliberately NOT on the values. A parked
aircraft sits 3.5 cm off the ground — far below the driver's 0.4 m minimum —
so a HEALTHY sensor reports ~0.0 m and PX4 flags it invalid while still
publishing. Do not "improve" this into a range-sanity check: it would fail
every bench run and, worse, teach the crew to ignore it. Values are printed
for eyeballing only; the ladder that proves accuracy is a separate procedure
(lift the airframe to tape-measured heights, ignore anything under 0.4 m).

Usage:
    .venv/bin/python tools/lidar_check.py [--connect udpin://0.0.0.0:14540]
        [--window 12] [--min-samples 5] [--rate 10]

⚠ Binds the same udpin port the orchestrator uses — run BEFORE staging, never
alongside a staged mission. It exits by itself.

Exit codes: 0 delivering · 2 NO DATA (do not fly the delivery on it) ·
3 no heartbeat.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mavsdk import System  # noqa: E402


async def _collect(drone, out: list) -> None:
    """Append (monotonic_ts, sample) forever — the caller times us out."""
    async for d in drone.telemetry.distance_sensor():
        out.append((time.monotonic(), d))


async def _probe(endpoint: str, window_s: float, min_samples: int,
                 rate_hz: float) -> int:
    drone = System()
    await drone.connect(system_address=endpoint)

    async def _wait_heartbeat() -> None:
        async for s in drone.core.connection_state():
            if s.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_heartbeat(), 15.0)
    except asyncio.TimeoutError:
        print("[lidar-check] no heartbeat in 15 s — FC off, or router down")
        return 3
    print(f"[lidar-check] connected — listening {window_s:.0f} s "
          f"(need >= {min_samples} samples)")

    try:
        await drone.telemetry.set_rate_distance_sensor(rate_hz)
    except Exception as e:  # noqa: BLE001 — an ACK proves nothing anyway
        print(f"[lidar-check] note: rate request refused ({type(e).__name__}) "
              "— continuing; PX4 may stream it regardless")

    samples: list = []
    try:
        await asyncio.wait_for(_collect(drone, samples), window_s)
    except asyncio.TimeoutError:
        pass  # the window elapsing IS the normal path
    except Exception as e:  # noqa: BLE001
        print(f"[lidar-check] read error {type(e).__name__}")

    if not samples:
        print("[lidar-check] ✘ NO DISTANCE_SENSOR DATA — PX4 has no working "
              "rangefinder aboard")
        print("[lidar-check]   The config being correct does NOT contradict "
              "this: SENS_TFMINI_CFG/HW and EKF2_RNG_* are all about what to "
              "LISTEN to, not whether anything is talking.")
        print("[lidar-check]   Check, in this order: flight pack connected "
              "(the sensor may be fed from a BEC, not TELEM3's 5 V pin) · "
              "TELEM3 connector reseated both ends · 5 V at the sensor and "
              "TX/RX continuity · flex the loom while re-running this.")
        print("[lidar-check]   Flying anyway means the pad approach and "
              "touchdown run on baro + GPS alone.")
        return 2

    elapsed = samples[-1][0] - samples[0][0]
    hz = (len(samples) - 1) / elapsed if elapsed > 0 else float("nan")
    dists = [s.current_distance_m for _, s in samples]
    first = samples[0][1]
    print(f"[lidar-check] {len(samples)} samples in {window_s:.0f} s "
          f"({hz:.1f} Hz measured)")
    print(f"[lidar-check]   distance {min(dists):.2f} .. {max(dists):.2f} m "
          f"(driver limits {first.minimum_distance_m:.2f} .. "
          f"{first.maximum_distance_m:.2f} m)")
    if max(dists) < first.minimum_distance_m:
        print("[lidar-check]   all readings below the driver minimum — normal "
              "for a parked airframe, the sensor is still delivering")

    if len(samples) < min_samples:
        print(f"[lidar-check] ✘ only {len(samples)} samples (need "
              f"{min_samples}) — intermittent, treat as NO DATA")
        return 2

    print("[lidar-check] ✔ rangefinder is delivering")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--connect", default="udpin://0.0.0.0:14540",
                        help="MAVLink endpoint (default: the router's offboard "
                             "port; serial:///dev/ttyAMA0:921600 = direct FC)")
    parser.add_argument("--window", type=float, default=12.0,
                        help="seconds to listen (default 12)")
    parser.add_argument("--min-samples", type=int, default=5,
                        help="samples required to pass (default 5)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="stream rate to request in Hz (default 10)")
    args = parser.parse_args()
    return asyncio.run(_probe(args.connect, args.window, args.min_samples,
                              args.rate))


if __name__ == "__main__":
    raise SystemExit(main())
