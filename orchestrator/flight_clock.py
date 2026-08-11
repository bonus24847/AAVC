"""The mission clock, in the AIRCRAFT's time base rather than the host's.

Every deadline the mission owns is a claim about how much FLYING is left: the
20-minute operation window, the TimePolicy reserves, the leg timeouts, the
align rung budgets. `time.monotonic()` answers that correctly only while the
host advances the aircraft in real time.

SITL does not. Under PX4 lockstep the aircraft lives in simulated time, and a
loaded host runs the simulation slower than the wall. The 2026-07-25 flight
recorded 443 s of sim across 776 s of wall (ULog 04_51_49), dipping to 0.20x
during the P3 -> sweep-wp0 leg. Consequences, all from this one mismatch:
the operation window was consumed ~1.8x too fast, the audit's own 12.9-minute
window was really 7.4 minutes of flying, and a distance-derived leg timeout
fired on an aircraft that was still closing on the waypoint at 8 m/s.

On the real aircraft wall time and flight time are the same thing, so this is
a no-op there by construction — and the fallback below makes the degraded case
(no vehicle timestamps at all) bit-for-bit the old wall-clock behaviour.

Source of the vehicle time: `CurrentTelemetry.vehicle_time_s`, filled from
MAVSDK `raw_gps().timestamp_us`, which PX4 stamps with `hrt_absolute_time()` —
the lockstep simulation clock in SITL, the real one on hardware. Deliberately
NOT the pymavlink raw subscriber, which is optional (config-gated) and
documents that it must never affect the orchestrator.
"""

from __future__ import annotations

import math
import time
from typing import Callable


class FlightClock:
    """A monotonic clock that advances with the aircraft when it can.

    Two invariants matter more than accuracy, because every timeout in the
    mission is a difference of this clock:

    * it never runs backwards — a vehicle reboot, a stream resync or a bogus
      timestamp resyncs the epoch instead of rewinding;
    * it never freezes — losing the vehicle stream falls back to the wall,
      because a stopped clock stops EVERY timeout in the mission, which is a
      far worse failure than a clock that runs a little fast.
    """

    #: No vehicle timestamp for this long (wall) => resume counting the wall.
    #: Must exceed the vehicle stream's period (raw_gps is ~1-5 Hz) so ordinary
    #: gaps between samples are held rather than papered over with wall time.
    STALE_S = 3.0
    __slots__ = ("_wall", "_t", "_veh_last", "_wall_at_veh", "_wall_last")

    def __init__(self, *, wall: Callable[[], float] = time.monotonic) -> None:
        self._wall = wall
        self._t = 0.0
        self._veh_last: float | None = None
        self._wall_at_veh: float | None = None
        self._wall_last = wall()

    def feed(self, vehicle_s: float) -> None:
        """Offer the vehicle's own timestamp, in seconds (NaN = unavailable).

        Cheap and idempotent: re-feeding the same timestamp advances nothing,
        so callers may feed on every read.
        """
        if not math.isfinite(vehicle_s):
            return
        now_wall = self._wall()
        if self._veh_last is not None and self._wall_at_veh is not None:
            # Credit the SMALLER of the two elapsed times. Flight time cannot
            # outrun the wall — lockstep SITL throttles the simulation to at
            # most real time, and on hardware they are the same clock — so the
            # minimum is the honest credit, and it makes three awkward cases
            # fall out with no special-casing:
            #   * normal SITL (vehicle slower)  -> credits the vehicle;
            #   * a vehicle REBOOT (a jump of hours) -> credits only the wall
            #     time that actually passed, instead of teleporting the clock
            #     past the whole operation window in one sample;
            #   * a long gap between reads -> still credits the full vehicle
            #     interval, so the clock does not depend on how often callers
            #     happen to poll it.
            veh_step = max(0.0, vehicle_s - self._veh_last)   # 0 also absorbs
            wall_step = max(0.0, now_wall - self._wall_at_veh)  # a backwards jump
            self._t += min(veh_step, wall_step)
        self._veh_last = vehicle_s
        self._wall_at_veh = now_wall
        self._wall_last = now_wall

    def now(self) -> float:
        """Seconds of flight time since this clock was created."""
        now_wall = self._wall()
        stale = (self._wall_at_veh is None
                 or (now_wall - self._wall_at_veh) > self.STALE_S)
        if stale:
            self._t += max(0.0, now_wall - self._wall_last)
        self._wall_last = now_wall
        return self._t
