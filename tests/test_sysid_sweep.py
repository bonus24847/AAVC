"""Unit tests for the sys-ID excitation driver (orchestrator.sysid_sweep): the
numpy log-chirp generator, the altitude-hold collective P-loop, and the
run_sweep teardown (land → EXPLICIT disarm — the mission pins
COM_DISARM_LAND=-1, so without it the vehicle stays armed forever, the PX4
ULog never rotates, and every sweep stalled 120 s waiting for a disarm that
never came; that ever-growing shared log was the 2026-07-04 "hang after the
pitch chirp").
"""

import asyncio
import types

import numpy as np
import pytest

from orchestrator.sysid_sweep import (
    _THR_MAX,
    _THR_MIN,
    _alt_hold_thrust,
    _chirp_samples,
    run_sweep,
    sweep_spec_for,
)

FS = 50.0


def _crossings(s: np.ndarray) -> int:
    return int(np.count_nonzero(np.diff(np.sign(s[s != 0.0]))))


def test_chirp_length_and_amplitude_bound():
    spec = sweep_spec_for("roll", "attitude", dur_s=25.0)
    s = _chirp_samples(spec, fs=FS)
    assert len(s) == int(25.0 * FS)
    assert np.max(np.abs(s)) <= spec.amp + 1e-9


def test_chirp_frequency_increases():
    spec = sweep_spec_for("pitch", "attitude", dur_s=25.0)
    s = _chirp_samples(spec, fs=FS)
    # Compare a low-frequency early window with a high-frequency late window
    # (both away from the tapered ends): the chirp sweeps 0.6 → 13 Hz.
    early = s[int(1 * FS):int(2 * FS)]
    late = s[int(20 * FS):int(21 * FS)]
    assert _crossings(late) > _crossings(early) * 3


def test_chirp_tapered_ends():
    spec = sweep_spec_for("yaw", "attitude", dur_s=25.0)
    s = _chirp_samples(spec, fs=FS)
    # The half-cosine taper pulls the very first/last samples to ~0 (no step jolt).
    assert abs(s[0]) < 0.05 * spec.amp
    assert abs(s[-1]) < 0.05 * spec.amp


def test_alt_hold_neutral_at_target():
    assert _alt_hold_thrust(0.6, 15.0, 15.0) == pytest.approx(0.6)


def test_alt_hold_raises_thrust_when_sinking():
    # 5 m below target → more collective, but never above the clamp.
    thr = _alt_hold_thrust(0.6, 15.0, 10.0)
    assert thr > 0.6
    assert thr <= _THR_MAX


def test_alt_hold_lowers_thrust_when_high():
    thr = _alt_hold_thrust(0.6, 15.0, 20.0)
    assert thr < 0.6
    assert thr >= _THR_MIN


def test_alt_hold_clamps_extremes():
    assert _alt_hold_thrust(0.6, 15.0, -1000.0) == _THR_MAX
    assert _alt_hold_thrust(0.6, 15.0, 1000.0) == _THR_MIN


def test_sweep_spec_axis_and_mode():
    assert sweep_spec_for("yaw", "attitude").amp == 8.0       # yaw angle amp
    assert sweep_spec_for("roll", "rate").amp == 35.0          # roll rate amp
    with pytest.raises(ValueError):
        sweep_spec_for("bogus")


# ── run_sweep teardown: land → explicit disarm (per-sweep ULog rotation) ──


class _FakeTelemetry:
    """Streams the shared vehicle state at 100 Hz (the real MAVSDK shape)."""

    def __init__(self, st: dict) -> None:
        self._st = st

    async def armed(self):
        while True:
            yield self._st["armed"]
            await asyncio.sleep(0.01)

    async def position(self):
        while True:
            yield types.SimpleNamespace(relative_altitude_m=self._st["alt"])
            await asyncio.sleep(0.01)

    async def velocity_ned(self):
        while True:
            yield types.SimpleNamespace(down_m_s=0.0)
            await asyncio.sleep(0.01)


class _FakeAction:
    """Instant-response PX4 stand-in: arm/takeoff/land mutate the state; disarm
    is REFUSED while airborne (like the real commander gate)."""

    def __init__(self, st: dict, calls: list[str]) -> None:
        self._st, self._calls = st, calls

    async def arm(self) -> None:
        self._st["armed"] = True
        self._calls.append("arm")

    async def set_takeoff_altitude(self, alt: float) -> None:
        pass

    async def takeoff(self) -> None:
        # Arrive AT the profile-derived hold altitude (a literal 15.0 breached
        # the KMUTNB profile's 4 m abort ceiling the moment the envelope
        # started scaling with the active profile).
        from orchestrator.sysid_sweep import DEFAULT_HOLD_ALT_M
        self._st["alt"] = DEFAULT_HOLD_ALT_M
        self._calls.append("takeoff")

    async def hold(self) -> None:
        self._calls.append("hold")

    async def land(self) -> None:
        self._st["alt"] = 0.0
        self._calls.append("land")

    async def disarm(self) -> None:
        if self._st["alt"] > 1.0:
            raise RuntimeError("disarm refused in air")
        self._st["armed"] = False
        self._calls.append("disarm")


class _FakeCommander:
    def __init__(self) -> None:
        self.st = {"armed": False, "alt": 0.0}
        self.calls: list[str] = []
        self.system = types.SimpleNamespace(
            telemetry=_FakeTelemetry(self.st), action=_FakeAction(self.st, self.calls))

    async def get_param_float(self, name: str) -> float:
        return 0.66

    async def set_attitude(self, r: float, p: float, y: float, thr: float) -> None:
        pass

    async def set_attitude_rate(self, r: float, p: float, y: float, thr: float) -> None:
        pass

    async def offboard_start(self) -> None:
        pass

    async def offboard_stop(self) -> None:
        pass


def test_run_sweep_disarms_after_landing_without_stalling():
    """COM_DISARM_LAND=-1 means PX4 never auto-disarms after the sweep's land —
    run_sweep must disarm EXPLICITLY (rotates the ULog per sweep) instead of
    stalling its old 120 s wait-for-a-disarm-that-never-comes. The wait_for is
    the stall detector: the old teardown times out here."""
    cmd = _FakeCommander()
    spec = sweep_spec_for("roll", "attitude", dur_s=0.3)

    async def _go():
        return await asyncio.wait_for(run_sweep(cmd, spec), timeout=20.0)  # type: ignore[arg-type]

    res = asyncio.run(_go())
    assert res.ok, res.detail
    assert "disarm" in cmd.calls          # explicit ground disarm issued
    assert cmd.st["armed"] is False       # vehicle actually ended disarmed


# ── altitude aborts must name their cause ──


def test_sweep_ceiling_tracks_the_mission_ceiling():
    """A literal 19.0 would keep aborting at 19 m after the competition ceiling
    moved — the sweep is bounded BY the envelope, so derive it from the profile."""
    from mission_brain.profile import load_profile
    from orchestrator.sysid_sweep import DEFAULT_ALT_CEILING_ABORT_M

    assert DEFAULT_ALT_CEILING_ABORT_M < load_profile().altitude_ceiling_m


class _StreamOnlyCommander:
    """Accepts the offboard setpoint stream and nothing else — _stream_sweep only
    needs those calls to succeed. (Distinct from the fuller _FakeCommander above,
    which drives the whole run_sweep flight.)"""

    async def set_attitude(self, *_a) -> None: ...
    async def set_attitude_rate(self, *_a) -> None: ...
    async def offboard_start(self) -> None: ...


def _run_stream(alt: float):
    from orchestrator.sysid_sweep import (
        DEFAULT_ALT_CEILING_ABORT_M,
        DEFAULT_ALT_FLOOR_ABORT_M,
        DEFAULT_HOLD_ALT_M,
        SweepResult,
        _stream_sweep,
        sweep_spec_for,
    )

    spec = sweep_spec_for("roll", dur_s=1.0)
    res = SweepResult(axis="roll")
    return asyncio.run(_stream_sweep(
        _StreamOnlyCommander(), spec, {"alt": alt, "climb": 0.0},
        target_alt=DEFAULT_HOLD_ALT_M, alt_floor_m=DEFAULT_ALT_FLOOR_ABORT_M,
        res=res, alt_ceiling_m=DEFAULT_ALT_CEILING_ABORT_M))


def test_a_ceiling_abort_says_it_hit_the_ceiling():
    """run_sweep replaces res.detail on every abort, so a climb through the
    ceiling used to read exactly like an offboard rejection — silent on the one
    failure this guard exists to surface. Altitudes derive from the module's
    own profile-scaled envelope, not literals, so the test tracks the active
    profile (competition 20 m OR the KMUTNB 5 m field)."""
    from orchestrator.sysid_sweep import DEFAULT_ALT_CEILING_ABORT_M

    breach = DEFAULT_ALT_CEILING_ABORT_M + 1.0
    ok, why = _run_stream(alt=breach)
    assert not ok
    assert "ceiling" in why and f"{breach:.1f}" in why


def test_a_floor_abort_says_it_sank():
    from orchestrator.sysid_sweep import DEFAULT_ALT_FLOOR_ABORT_M

    ok, why = _run_stream(alt=DEFAULT_ALT_FLOOR_ABORT_M * 0.5)
    assert not ok
    assert "floor" in why


def test_a_clean_sweep_reports_no_abort_reason():
    from orchestrator.sysid_sweep import DEFAULT_HOLD_ALT_M

    ok, why = _run_stream(alt=DEFAULT_HOLD_ALT_M)
    assert ok and why == ""
