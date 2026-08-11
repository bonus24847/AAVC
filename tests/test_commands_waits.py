"""Unit tests for the de-silenced timeout helpers + the payload-id guard.

The old `_wait_until_altitude_reached` / `_wait_until_disarmed` logged and
returned *success* on timeout, so the mission flew on through takeoff/land on
unverified state. They now return a bool; `arm_and_takeoff` raises on a False
altitude (→ the orchestrator's emergency-RTH boundary). `drop_payload` now
refuses an out-of-range `payload_id` instead of addressing a stray servo channel.

Async helpers are driven with `asyncio.run(...)` from sync tests (no
pytest-asyncio), with tiny timeouts so the deadline branch is reached instantly.
"""
import asyncio

import pytest

from mavlink_adapter.commands import ConnectionConfig, DroneCommander


def _commander_with_system(system: object) -> DroneCommander:
    c = DroneCommander.__new__(DroneCommander)  # skip real MAVSDK System()
    c.system = system  # type: ignore[assignment]
    return c


# ── _wait_until_altitude_reached → bool ──

class _Pos:
    def __init__(self, alt: float) -> None:
        self.relative_altitude_m = alt


class _AltTelem:
    """Fake telemetry whose position() stream forever reports a fixed altitude."""

    def __init__(self, alt: float) -> None:
        self._alt = alt

    async def position(self):
        while True:
            yield _Pos(self._alt)
            await asyncio.sleep(0.002)


def test_altitude_reached_returns_true():
    sys = type("_S", (), {"telemetry": _AltTelem(12.0)})()
    ok = asyncio.run(_commander_with_system(sys)._wait_until_altitude_reached(10.0, timeout_s=0.5))
    assert ok is True


def test_altitude_timeout_returns_false():
    sys = type("_S", (), {"telemetry": _AltTelem(0.5)})()  # never climbs
    ok = asyncio.run(_commander_with_system(sys)._wait_until_altitude_reached(10.0, timeout_s=0.05))
    assert ok is False  # was a silent `return` (success) before


# ── stalled-stream regression (the climb-out hang observed 2026-06-13) ──
# A frozen SITL lockstep / stalled telemetry link makes the stream stop emitting
# ENTIRELY (distinct from _AltTelem above, which keeps yielding a low altitude).
# The old code tested its timeout deadline *inside* `async for`, so with no
# emission the deadline never ran and the wait hung forever. The fix bounds the
# whole wait on the wall clock; these tests pin that — under the old code they
# hang (caught by the outer wait_for guard so the suite fails loudly, not wedges).

class _StalledTelem:
    """Telemetry whose streams subscribe but never emit — a frozen sim / link."""

    async def position(self):
        await asyncio.Event().wait()  # blocks forever; never yields
        yield _Pos(0.0)  # pragma: no cover — unreachable; marks this a generator

    async def health(self):
        await asyncio.Event().wait()  # blocks forever; never yields
        yield  # pragma: no cover — unreachable; marks this a generator


def test_altitude_stalled_stream_times_out_not_hangs():
    sys = type("_S", (), {"telemetry": _StalledTelem()})()

    async def _run():
        return await asyncio.wait_for(  # outer guard: a regression fails, never wedges
            _commander_with_system(sys)._wait_until_altitude_reached(10.0, timeout_s=0.05),
            timeout=2.0,
        )

    assert asyncio.run(_run()) is False


def test_arm_ready_stalled_stream_returns_not_hangs():
    sys = type("_S", (), {"telemetry": _StalledTelem()})()

    async def _run():  # _wait_arm_ready warns + returns on timeout; must not hang
        await asyncio.wait_for(
            _commander_with_system(sys)._wait_arm_ready(timeout_s=0.05),
            timeout=2.0,
        )

    asyncio.run(_run())


# ── _wait_until_disarmed → bool ──

class _ArmTelem:
    def __init__(self, armed: bool) -> None:
        self._armed = armed

    async def armed(self):
        # _is_armed() does `async for v in armed(): return bool(v)` — one value.
        yield self._armed


def test_disarm_observed_returns_true():
    sys = type("_S", (), {"telemetry": _ArmTelem(False)})()
    ok = asyncio.run(_commander_with_system(sys)._wait_until_disarmed(timeout_s=0.5, poll_s=0.01))
    assert ok is True


def test_disarm_timeout_returns_false():
    sys = type("_S", (), {"telemetry": _ArmTelem(True)})()  # stays armed
    ok = asyncio.run(_commander_with_system(sys)._wait_until_disarmed(timeout_s=0.05, poll_s=0.01))
    assert ok is False


# ── connect() bounds System.connect() too (no-SITL silent-hang regression) ──
# mavsdk_server 3.15 opens its gRPC port only AFTER it discovers a MAVLink system,
# so with no vehicle present System.connect() blocks in channel_ready() forever.
# connect() used to bound only the heartbeat wait, leaving THAT call uncovered →
# a silent indefinite hang (the "make run with no SITL just freezes" report). It
# now times out, reaps the orphaned server, and raises. Outer wait_for guard: a
# regression fails loudly here instead of wedging the suite.

class _NeverReadySystem:
    """Fake MAVSDK System whose connect() never returns (gRPC never ready)."""

    async def connect(self, system_address=None):
        await asyncio.Event().wait()  # blocks forever — mirrors channel_ready() hang


def test_connect_times_out_when_backend_never_ready():
    c = _commander_with_system(_NeverReadySystem())
    c.config = ConnectionConfig(connect_timeout_s=0.05)

    async def _run():
        with pytest.raises(RuntimeError, match="gRPC backend not ready"):
            await asyncio.wait_for(c.connect(), timeout=2.0)  # was an infinite hang

    asyncio.run(_run())


# ── drop_payload payload-id guard ──

def test_drop_payload_rejects_out_of_range():
    c = DroneCommander.__new__(DroneCommander)
    c.config = ConnectionConfig()  # drop_payload_count = 1 (one egg per sortie)
    with pytest.raises(ValueError):
        asyncio.run(c.drop_payload(1))   # 1 is out of [0, 1)
    with pytest.raises(ValueError):
        asyncio.run(c.drop_payload(-1))
