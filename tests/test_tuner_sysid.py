"""The tuner's sys-ID pipeline task (dashboard.tuner._run_sysid) must SURVIVE a
fit failure: on 2026-07-04 a pyulog error on the ever-growing shared log killed
the asyncio task silently — the UI froze at "fitting" with no terminal status
(the perceived hang). A fit exception must push a per-axis "failed" status and
the run must still complete with a result.
"""

from __future__ import annotations

import asyncio
import types
from pathlib import Path
from typing import Any

import dashboard.tuner as tuner_mod
from dashboard.tuner import _run_sysid, _TunerState
from orchestrator.sysid_sweep import SweepResult


class _RecordingBroadcaster:
    def __init__(self) -> None:
        self.statuses: list[dict[str, Any]] = []
        self.results: list[dict[str, Any]] = []

    def now_relative(self) -> float:
        return 0.0

    def push_sysid_status(self, payload: dict[str, Any]) -> None:
        self.statuses.append(payload)

    def push_sysid_result(self, payload: dict[str, Any]) -> None:
        self.results.append(payload)


def test_fit_exception_pushes_failed_status_and_run_completes(monkeypatch) -> None:
    async def _ok_sweep(commander: Any, spec: Any) -> SweepResult:
        return SweepResult(axis=spec.axis, ok=True, detail="stub sweep")

    def _boom(ulog: Any, axis: str) -> Any:
        raise RuntimeError("truncated ULog mid-write")

    monkeypatch.setattr(tuner_mod.sysid_sweep, "run_sweep", _ok_sweep)
    monkeypatch.setattr(tuner_mod, "_newest_ulog", lambda after_epoch=0.0: Path("x.ulg"))
    monkeypatch.setattr(tuner_mod.sysid, "estimate_frf", _boom)
    monkeypatch.setattr(tuner_mod, "save_calibration", lambda calib: None)

    bc = _RecordingBroadcaster()
    ts = _TunerState()
    state = types.SimpleNamespace()  # _run_sysid does not touch state fields

    # The task itself must NOT raise (a raise = silent task death in the router).
    asyncio.run(_run_sysid(state, commander=None, broadcaster=bc,  # type: ignore[arg-type]
                           ts=ts, axes=["roll"], mode="attitude"))

    failed = [s for s in bc.statuses if s.get("state") == "failed"]
    assert failed, f"no 'failed' status pushed; statuses={bc.statuses}"
    assert "fit" in failed[0]["detail"] or "ULog" in failed[0]["detail"]
    assert bc.results, "run must still publish a terminal sysid_result"
    done = [s for s in bc.statuses if s.get("state") == "done"]
    assert done, "run must still reach the terminal 'done' status"
