"""Per-stream liveness in the MAVSDK telemetry fan-in (2026-08-22 review).

``CurrentTelemetry.age_s()`` is the only freshness signal the safety watchdog
had, and ALL fourteen subscribers touch it — so one dead stream was invisible:
the other thirteen kept the timestamp fresh while the dead one's field stayed
frozen at its last value forever. A frozen ``is_armed`` blinds the disarm
detector permanently (the exact hole the takeover fix closed); a frozen
``relative_alt_m`` feeds the ceiling watchdog a number that stopped moving.

Eight subscribers also had no exception handling: a gRPC stream error killed
the task silently — no log, no retry, no restart.
"""

from __future__ import annotations

import asyncio

from mavlink_adapter.telemetry import TelemetrySubscriber


def _sub() -> TelemetrySubscriber:
    return TelemetrySubscriber(system=None)   # type: ignore[arg-type]


def test_a_stream_that_never_delivered_is_infinitely_old() -> None:
    s = _sub()
    assert s.stream_age_s("armed") == float("inf")
    assert s.dead_streams() == []          # never started != stale


def test_one_dead_stream_is_visible_even_while_the_others_are_fresh() -> None:
    s = _sub()
    for name in ("position", "armed", "flight_mode"):
        s._touch(name)
    assert s.dead_streams(max_age_s=5.0) == []

    # armed stopped delivering; the shared timestamp keeps being touched by
    # the others, which is exactly what used to hide this.
    import time as _t
    s._stream_seen["armed"] = _t.monotonic() - 30.0
    s._touch("position")
    assert s.state.age_s() < 1.0            # the SHARED stamp looks healthy…
    assert s.dead_streams(max_age_s=5.0) == ["armed"]   # …the stream does not
    assert s.stream_age_s("armed") > 25.0


def test_a_failing_stream_is_restarted_and_counted() -> None:
    """A stream that raises must come back, not die silently."""
    s = _sub()
    calls: list[int] = []

    async def flaky() -> None:
        calls.append(1)
        if len(calls) < 3:
            raise RuntimeError("gRPC stream broke")
        await asyncio.sleep(3600)           # third attempt stays up

    async def run() -> None:
        task = asyncio.create_task(s._supervise("flaky", flaky))
        for _ in range(200):                # let the backoff play out
            await asyncio.sleep(0.02)
            if len(calls) >= 3:
                break
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    asyncio.run(run())
    assert len(calls) >= 3, "the supervisor did not restart the stream"
    assert s.stream_failures >= 2
