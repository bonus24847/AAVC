"""The mission clock runs on the AIRCRAFT's time base, not the host's.

Every deadline the mission owns — the 20-minute operation window, the
TimePolicy reserves, the leg and rung timeouts — is a statement about how much
flying is left. `time.monotonic()` answers that correctly only while the host
advances the aircraft in real time.

SITL does not. Under PX4 lockstep the aircraft lives in SIMULATED time, and a
loaded host runs the simulation slower than the wall: the 2026-07-25 flight
logged 443 s of sim across 776 s of wall (ULog 04_51_49), dipping to 0.20x on
the P3 -> sweep-wp0 leg. Everything the mission believed about time was wrong
by that factor — the window was consumed ~1.8x too fast, the reported 12.9-min
window was really 7.4 min of flying, and a leg timeout fired on an aircraft
that was still closing at 8 m/s.

On the real aircraft the two clocks are the same thing, so this is a no-op
there by construction: FlightClock falls back to the wall the moment the
vehicle stops reporting, which is exactly the old behaviour.
"""

from __future__ import annotations

import math

from orchestrator.flight_clock import FlightClock


class _Wall:
    def __init__(self) -> None:
        self.t = 500.0

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float) -> None:
        self.t += dt


def test_without_a_vehicle_clock_it_is_the_wall_clock() -> None:
    """The real-aircraft path, and the degraded-SITL path: no vehicle
    timestamp means nothing is known that beats the wall."""
    wall = _Wall()
    clock = FlightClock(wall=wall)
    assert clock.now() == 0.0
    wall.tick(10.0)
    assert clock.now() == 10.0
    wall.tick(5.0)
    assert clock.now() == 15.0


def test_it_tracks_the_vehicle_not_the_host() -> None:
    """The regression: the host runs at 0.20x, so 100 s of wall buys 20 s of
    flying. The mission must be told 20."""
    wall = _Wall()
    clock = FlightClock(wall=wall)
    veh = 1000.0                       # vehicle boot time, arbitrary origin
    clock.feed(veh)
    for _ in range(100):
        wall.tick(1.0)                 # 1 s of host time...
        veh += 0.20                    # ...buys 0.20 s of flight time
        clock.feed(veh)
    assert math.isclose(clock.now(), 20.0, abs_tol=0.05), (
        f"mission clock read {clock.now():.1f} s for 20 s of flying")


def test_a_real_time_host_is_indistinguishable() -> None:
    wall = _Wall()
    clock = FlightClock(wall=wall)
    veh = 7.5
    clock.feed(veh)
    for _ in range(30):
        wall.tick(1.0)
        veh += 1.0
        clock.feed(veh)
    assert math.isclose(clock.now(), 30.0, abs_tol=1e-6)


def test_it_falls_back_to_the_wall_when_the_vehicle_goes_quiet() -> None:
    """A frozen clock is far more dangerous than a fast one: every timeout in
    the mission would stop firing. Losing the vehicle stream must degrade to
    the old wall-clock behaviour, not to a stopped clock."""
    wall = _Wall()
    clock = FlightClock(wall=wall)
    clock.feed(2000.0)
    wall.tick(1.0)
    clock.feed(2001.0)
    assert math.isclose(clock.now(), 1.0, abs_tol=1e-6)
    for _ in range(60):                # stream dies; only the wall moves
        wall.tick(1.0)
        clock.now()
    assert clock.now() > 50.0, "the clock froze when the vehicle went quiet"


def test_it_never_runs_backwards() -> None:
    """Deadlines are differences of this clock. A vehicle reboot, a stream
    resync or a bogus timestamp must not rewind it."""
    wall = _Wall()
    clock = FlightClock(wall=wall)
    seen = 0.0
    for veh in (100.0, 101.0, 102.0, 5.0, 6.0, 7.0,      # reboot to a new epoch
                math.nan, math.inf, -1.0, 8.0, 1e12, 9.0):
        wall.tick(0.5)
        clock.feed(veh)
        now = clock.now()
        assert now >= seen, f"clock went backwards at vehicle t={veh}"
        seen = now


def test_a_vehicle_jump_does_not_teleport_the_mission_clock() -> None:
    """A resync gap must resume counting, not credit the whole gap at once —
    that would blow the operation window in a single sample."""
    wall = _Wall()
    clock = FlightClock(wall=wall)
    clock.feed(10.0)
    wall.tick(0.5)
    clock.feed(10_000.0)               # a ~3-hour jump
    assert clock.now() < 10.0
    wall.tick(0.5)
    clock.feed(10_001.0)               # counting resumes from the new epoch
    assert math.isclose(clock.now(), 1.0, abs_tol=0.6)


def test_the_operation_window_is_measured_in_flight_time() -> None:
    """End-to-end wiring: OrchestratorState.time_elapsed_s — which is what the
    20-minute window and every TimePolicy reserve are read from — must follow
    the vehicle, not the host.

    This is the number the 2026-07-25 run got wrong: it reported a 12.9-minute
    window for 7.4 minutes of flying, so the mission spent its budget on host
    load rather than on flight.
    """
    from mavlink_adapter.telemetry import CurrentTelemetry
    from orchestrator.state import OrchestratorMode, OrchestratorState
    from tests.test_delivery_mission import _state

    wall = _Wall()
    base = _state()
    telem = CurrentTelemetry()
    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE,
        plan=base.plan,
        telemetry=telem,
        flight_clock=FlightClock(wall=wall),
    )
    telem.vehicle_time_s = 4242.0
    state.start_window()
    assert state.time_elapsed_s() == 0.0

    for _ in range(300):                    # 300 s of host time at 0.5x
        wall.tick(1.0)
        telem.vehicle_time_s += 0.5
    assert math.isclose(state.time_elapsed_s(), 150.0, abs_tol=1.0), (
        f"window read {state.time_elapsed_s():.0f} s of a 1200 s budget for "
        "150 s of flying")
    assert math.isclose(state.time_remaining_s(), 1050.0, abs_tol=1.0)
