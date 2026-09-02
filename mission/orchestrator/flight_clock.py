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
    #: A single vehicle step this far AHEAD of the wall is a resync (reboot,
    #: stream re-establish), not jitter: start a new epoch instead of letting
    #: the cumulative comparison ride a bogus offset.
    JUMP_S = 5.0
    __slots__ = ("_wall", "_t", "_veh_last", "_wall_at_veh", "_wall_last",
                 "_stale_credit", "_t_base", "_veh_epoch", "_wall_epoch")

    def __init__(self, *, wall: Callable[[], float] = time.monotonic) -> None:
        self._wall = wall
        self._t = 0.0
        self._veh_last: float | None = None
        # Wall time at the last NEW vehicle sample. Re-feeding the same
        # timestamp must NOT move it (2026-08-26): state.now() feeds on every
        # read (~10 Hz) while the hardware timestamp changes only every so
        # often, and an anchor reset on each re-feed shrank the wall step to
        # ~0.1 s by the time the value moved — the real aircraft's mission
        # clock ran at 1/20 speed and every deadline stretched with it.
        self._wall_at_veh: float | None = None
        self._wall_last = wall()
        # Wall seconds now() has credited since the last new sample (the
        # stale fallback), netted off when that sample lands so the same
        # interval is never counted twice.
        self._stale_credit = 0.0
        # The current EPOCH: within one, flight time is
        #   _t_base + min(vehicle - _veh_epoch, wall - _wall_epoch)
        # — a CUMULATIVE comparison, so arrival jitter (a late sample followed
        # by an early one) cancels instead of being lost step by step. A
        # reboot, a jump, a backwards timestamp or a stale gap starts a new
        # epoch from the flight time credited so far.
        self._t_base = 0.0
        self._veh_epoch = 0.0
        self._wall_epoch = 0.0

    def _new_epoch(self, vehicle_s: float, now_wall: float) -> None:
        self._t_base = self._t
        self._veh_epoch = vehicle_s
        self._wall_epoch = now_wall

    def feed(self, vehicle_s: float) -> None:
        """Offer the vehicle's own timestamp, in seconds (NaN = unavailable).

        Cheap and idempotent: re-feeding the same timestamp advances nothing
        AND moves nothing, so callers may feed on every read.
        """
        if not math.isfinite(vehicle_s):
            return
        if vehicle_s == self._veh_last:
            return                      # same sample: nothing new to credit
        now_wall = self._wall()
        if self._veh_last is None or self._wall_at_veh is None:
            self._new_epoch(vehicle_s, now_wall)
        else:
            veh_step = vehicle_s - self._veh_last
            wall_step = max(0.0, now_wall - self._wall_at_veh)  # absorbs a
            # backwards wall jump
            resync = (veh_step < 0.0                   # reboot / bogus stamp
                      or veh_step > wall_step + self.JUMP_S   # forward jump
                      or self._stale_credit > 0.0)     # gap the wall covered
            if resync:
                # Credit the SMALLER of the two elapsed times — flight time
                # cannot outrun the wall (lockstep throttles the sim to at
                # most real time; on hardware they are the same clock) — net
                # of what the stale fallback already credited for this gap.
                # A reboot's jump of hours therefore credits only the wall
                # that actually passed instead of teleporting the clock past
                # the operation window; a backwards stamp credits nothing.
                credit = min(max(0.0, veh_step), wall_step) - self._stale_credit
                self._t += max(0.0, credit)
                self._new_epoch(vehicle_s, now_wall)
            else:
                cum_veh = vehicle_s - self._veh_epoch
                cum_wall = now_wall - self._wall_epoch
                # max(): never backwards, even if float noise says otherwise
                self._t = max(self._t, self._t_base + min(cum_veh, cum_wall))
        self._stale_credit = 0.0
        self._veh_last = vehicle_s
        self._wall_at_veh = now_wall
        self._wall_last = now_wall

    def now(self) -> float:
        """Seconds of flight time since this clock was created."""
        now_wall = self._wall()
        stale = (self._wall_at_veh is None
                 or (now_wall - self._wall_at_veh) > self.STALE_S)
        if stale:
            step = max(0.0, now_wall - self._wall_last)
            self._t += step
            self._stale_credit += step
        self._wall_last = now_wall
        return self._t
