"""System-ID + Autotune command router — the pre-flight tuning module.

NOT part of the scored sortie (CLAUDE.md §2/§4). Mirrors dashboard/commands.py
(same ``/api/cmd`` prefix, shared CommandSession arm-gate, CSRF header,
background-task pattern) and exposes:

    GET  /api/cmd/tuner/params     live MC rate/attitude gains read off the FC
    POST /api/cmd/tuner/design     model-based gain synthesis (pure compute)
    POST /api/cmd/sysid/run        fly per-axis chirp sweeps → identify the plant
    POST /api/cmd/autotune/start   drive PX4's built-in empirical autotune
    POST /api/cmd/autotune/abort   stop the autotune
    POST /api/cmd/tuner/apply      write chosen gains to the FC (vehicle disarmed)

The two gain engines (model-based-from-identified-plant and PX4-native) are
surfaced side by side; the operator picks before Apply. Sys-ID identifies the
transfer function from the PX4 ULog (the open-loop torque→rate plant is not on
the MAVSDK wire), fits ``b``/``τ`` (tuning.sysid), and feeds the measured ``b``
into the model-based synthesis (tuning.engine) as a PlantCalibration.
"""

from __future__ import annotations

import asyncio
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Header, HTTPException, status
from loguru import logger
from pydantic import BaseModel, ConfigDict, Field

from mission_brain.schemas import active_airframe
from orchestrator import sysid_sweep
from orchestrator.state import OrchestratorState
from tuning import sysid
from tuning.calibration import PlantCalibration, measured_vs_model, save_calibration
from tuning.engine import design_gains
from tuning.gains_io import save_gains
from tuning.schemas import DesignRequest, PerfSpec, PhysicalParams

from .commands import _REQUIRED_CMD_VALUE, CommandSession
from .realtime import RealtimeBroadcaster

# Live FC params surfaced in the panel + the apply whitelist.
_READBACK_PARAMS: list[tuple[str, str, str]] = [
    ("MC_ROLLRATE_P", "rate", "roll"), ("MC_ROLLRATE_I", "rate", "roll"),
    ("MC_ROLLRATE_D", "rate", "roll"),
    ("MC_PITCHRATE_P", "rate", "pitch"), ("MC_PITCHRATE_I", "rate", "pitch"),
    ("MC_PITCHRATE_D", "rate", "pitch"),
    ("MC_YAWRATE_P", "rate", "yaw"), ("MC_YAWRATE_I", "rate", "yaw"),
    ("MC_YAWRATE_D", "rate", "yaw"),
    ("MC_ROLL_P", "attitude", "roll"), ("MC_PITCH_P", "attitude", "pitch"),
    ("MC_YAW_P", "attitude", "yaw"),
]
# Apply is restricted to control-tuning params — never anything else over MAVLink.
_APPLY_PREFIXES = ("MC_", "MPC_", "NAV_")
_AXES = ("roll", "pitch", "yaw")
# PX4 SITL writes ULogs under whichever tree is flying, so follow PX4_DIR — the
# same variable the launchers use — rather than naming one tree here. Pinning it
# meant a sweep flown on another tree wrote its log where the tuner never looked,
# and every axis then reported "no ULog" after a perfectly good flight.
# AAVC_PX4_LOG_DIR still wins, for hardware or an out-of-tree build.
_PX4_LOG_DIR = Path(
    os.environ.get("AAVC_PX4_LOG_DIR", "")
    or str(Path(os.environ.get("PX4_DIR", "")
                or Path.home() / "PX4-Autopilot-v1.17")
           / "build/px4_sitl_default/rootfs/log")
)


class TunerDesignRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    physical: dict[str, Any]
    spec: dict[str, Any] = Field(default_factory=dict)
    use_calibration: bool = True


class TunerApplyRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    gains: list[dict[str, Any]]
    operator_note: str = Field(default="", max_length=200)


class SysIdRunRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    axes: list[str] = Field(default_factory=lambda: ["roll", "pitch", "yaw"])
    mode: str = "attitude"
    operator_note: str = Field(default="", max_length=200)


class AutotuneRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    operator_note: str = Field(default="", max_length=200)


@dataclass
class _TunerState:
    """Router-scoped mutable state (the last sys-ID calibration + running tasks)."""
    calibration: PlantCalibration | None = None
    sysid_task: asyncio.Task[None] | None = None
    autotune_task: asyncio.Task[None] | None = None
    tasks: set[asyncio.Task[None]] = field(default_factory=set)


def _newest_ulog(after_epoch: float = 0.0) -> Path | None:
    """Newest ``*.ulg`` under the PX4 log dir (optionally modified after a time)."""
    if not _PX4_LOG_DIR.exists():
        return None
    ulgs = [p for p in _PX4_LOG_DIR.rglob("*.ulg") if p.stat().st_mtime >= after_epoch]
    if not ulgs:
        return None
    return max(ulgs, key=lambda p: p.stat().st_mtime)


def make_tuner_router(
    state: OrchestratorState,
    commander: Any,
    broadcaster: RealtimeBroadcaster,
    session: CommandSession,
) -> APIRouter:
    r = APIRouter(prefix="/api/cmd", tags=["tuner"])
    ts = _TunerState()

    def _check_header(x: str | None) -> None:
        if x != _REQUIRED_CMD_VALUE:
            raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                                detail="missing X-AAVC-CMD header")

    def _require_armed() -> None:
        if not session.armed:
            raise HTTPException(status_code=status.HTTP_409_CONFLICT,
                                detail="command channel not armed (POST /api/cmd/arm first)")

    def _track(task: asyncio.Task[None]) -> None:
        ts.tasks.add(task)
        task.add_done_callback(ts.tasks.discard)

    # ── model-based design (pure compute; no flight) ──

    @r.get("/tuner/params")
    async def tuner_params() -> dict[str, Any]:
        params = []
        for name, loop, axis in _READBACK_PARAMS:
            value: float | None
            try:
                value = float(await commander.get_param_float(name))
            except Exception:
                value = None
            params.append({"param": name, "value": value, "loop": loop, "axis": axis})
        return {"t_monotonic": broadcaster.now_relative(), "params": params}

    @r.post("/tuner/design")
    async def tuner_design(
        req: TunerDesignRequest, x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        try:
            phys = PhysicalParams(**req.physical)
            spec = PerfSpec(**req.spec)
            design_req = DesignRequest(airframe=active_airframe(), physical=phys, spec=spec)
        except Exception as e:
            raise HTTPException(status_code=422, detail=f"invalid design request: {e}") from e
        calib = ts.calibration if req.use_calibration else None
        result = design_gains(design_req, calibration=calib)
        payload = result.model_dump()
        payload["t_monotonic"] = broadcaster.now_relative()
        payload["calibration_source"] = calib.source if calib else None
        payload["measured_vs_model"] = (
            measured_vs_model(active_airframe().value, calib, phys) if calib else []
        )
        broadcaster.push_tuner_design(payload)
        return payload

    # ── system identification (flies per-axis chirp sweeps) ──

    @r.post("/sysid/run")
    async def sysid_run(
        req: SysIdRunRequest, x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        if ts.sysid_task is not None and not ts.sysid_task.done():
            raise HTTPException(status_code=409, detail="a sys-ID run is already in progress")
        axes = [a for a in req.axes if a in _AXES] or list(_AXES)
        ts.sysid_task = asyncio.get_running_loop().create_task(
            _run_sysid(state, commander, broadcaster, ts, axes, req.mode))
        _track(ts.sysid_task)
        return {"ok": True, "axes": axes, "mode": req.mode}

    # ── PX4 built-in empirical autotune ──

    @r.post("/autotune/start")
    async def autotune_start(
        req: AutotuneRequest, x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        if ts.autotune_task is not None and not ts.autotune_task.done():
            raise HTTPException(status_code=409, detail="autotune already running")
        ts.autotune_task = asyncio.get_running_loop().create_task(
            _run_autotune(commander, broadcaster, ts))
        _track(ts.autotune_task)
        return {"ok": True}

    @r.post("/autotune/abort")
    async def autotune_abort(
        req: AutotuneRequest, x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        try:
            await commander.set_param_int("MC_AT_START", 0)
        except Exception as e:
            logger.warning(f"[autotune] abort set MC_AT_START=0 failed: {e}")
        if ts.autotune_task is not None:
            ts.autotune_task.cancel()
        broadcaster.push_autotune_status(
            {"state": "aborted", "axis": "", "progress_pct": 0.0, "detail": "operator abort"})
        return {"ok": True}

    # ── apply gains (vehicle MUST be disarmed) ──

    @r.post("/tuner/apply")
    async def tuner_apply(
        req: TunerApplyRequest, x_aavc_cmd: str | None = Header(default=None),
    ) -> dict[str, Any]:
        _check_header(x_aavc_cmd)
        _require_armed()
        if state.telemetry.is_armed:
            raise HTTPException(status_code=409,
                                detail="vehicle is ARMED — disarm before applying gains")
        applied: list[dict[str, Any]] = []
        for g in req.gains:
            name = str(g.get("param", ""))
            if not name.startswith(_APPLY_PREFIXES):
                applied.append({"param": name, "ok": False,
                                "detail": "not a whitelisted tuning param"})
                continue
            try:
                await commander.set_param_float(name, float(g["value"]))
                rb = float(await commander.get_param_float(name))
                applied.append({"param": name, "ok": True, "value": rb})
            except Exception as e:
                applied.append({"param": name, "ok": False, "detail": str(e)})
        # Persist the applied gains so the mission auto-loads them at startup.
        saved_to: str | None = None
        ok_gains = {a["param"]: a["value"] for a in applied if a.get("ok") and "value" in a}
        if ok_gains:
            try:
                saved_to = str(save_gains(ok_gains, source="model-based design"))
            except Exception as e:
                logger.warning(f"[tuner] save_gains failed: {e}")
        payload = {
            "t_monotonic": broadcaster.now_relative(),
            "applied": applied,
            "ok": all(a["ok"] for a in applied),
            "saved_to": saved_to,
            "operator_note": req.operator_note,
        }
        state.record_audit(f"[tuner] applied {sum(a['ok'] for a in applied)}/{len(applied)} gains")
        broadcaster.push_tuner_apply(payload)
        return payload

    return r


# ──────────────────────────── background flight tasks ────────────────────────────

async def _run_sysid(
    state: OrchestratorState, commander: Any, broadcaster: RealtimeBroadcaster,
    ts: _TunerState, axes: list[str], mode: str,
) -> None:
    """Fly one chirp sweep per axis, then fit the plant from the ULog."""
    fits: list[dict[str, Any]] = []
    b_measured: dict[str, float] = {}
    last_ulog = ""
    for axis in axes:
        broadcaster.push_sysid_status(
            {"state": "sweeping", "axis": axis, "detail": f"{mode} chirp on {axis}"})
        t_before = time.time()
        spec = sysid_sweep.sweep_spec_for(axis, mode)
        try:
            res = await sysid_sweep.run_sweep(commander, spec)
        except Exception as e:
            logger.exception(f"[sysid] {axis} sweep failed: {e}")
            broadcaster.push_sysid_status({"state": "failed", "axis": axis, "detail": str(e)[:120]})
            continue
        if not res.ok:
            broadcaster.push_sysid_status({"state": "failed", "axis": axis, "detail": res.detail})
            continue
        broadcaster.push_sysid_status({"state": "fitting", "axis": axis, "detail": "reading ULog"})
        ulog = _newest_ulog(after_epoch=t_before - 2.0)
        if ulog is None:
            broadcaster.push_sysid_status(
                {"state": "failed", "axis": axis,
                 "detail": f"no ULog found in {_PX4_LOG_DIR} — is logging on, "
                           "and is that the tree that flew? (PX4_DIR / "
                           "AAVC_PX4_LOG_DIR select it)"})
            continue
        last_ulog = ulog.name
        try:
            # to_thread: a big ULog parse must not block the loop. A parse
            # error (e.g. a still-open log truncated mid-write) must fail THIS
            # axis loudly, never kill the whole task silently — that was the
            # 2026-07-04 "hang": the UI froze at "fitting" with no status.
            frf = await asyncio.to_thread(sysid.estimate_frf, ulog, axis)
            fit = await asyncio.to_thread(sysid.fit_plant, frf)
        except Exception as e:
            logger.exception(f"[sysid] {axis} fit failed on {ulog.name}: {e}")
            broadcaster.push_sysid_status(
                {"state": "failed", "axis": axis,
                 "detail": f"fit error on {ulog.name}: {e!s:.100}"})
            continue
        if fit.b is not None and fit.b > 0:
            b_measured[axis] = fit.b
        fits.append({"frf": frf.to_dict(), "fit": fit.to_dict()})
        detail = (f"b={fit.b:.1f} τ={fit.tau_eff_s} r²={fit.r2}"
                  if fit.b else (fit.note or "no fit"))
        broadcaster.push_sysid_status({"state": "fit", "axis": axis, "detail": detail})

    calib = PlantCalibration(
        airframe=active_airframe().value, b_measured=b_measured, source=last_ulog,
    )
    ts.calibration = calib
    try:
        save_calibration(calib)
    except Exception as e:
        logger.warning(f"[sysid] save_calibration failed: {e}")
    broadcaster.push_sysid_result({
        "t_monotonic": broadcaster.now_relative(),
        "fits": fits,
        "calibration": calib.to_dict(),
    })
    broadcaster.push_sysid_status(
        {"state": "done", "axis": "", "detail": f"{len(b_measured)} axes identified"})


async def _run_autotune(commander: Any, broadcaster: RealtimeBroadcaster, ts: _TunerState) -> None:
    """Drive PX4's built-in MC autotune: arm → takeoff → hold → MC_AT_START=1,
    stream STATUSTEXT progress, apply+save on success, then land."""
    sys = commander.system

    def _emit(state_: str, detail: str = "", axis: str = "", pct: float = 0.0) -> None:
        broadcaster.push_autotune_status(
            {"state": state_, "axis": axis, "progress_pct": pct, "detail": detail})

    armed = {"v": False, "alt": 0.0}

    async def _w_armed() -> None:
        async for a in sys.telemetry.armed():
            armed["v"] = bool(a)

    async def _w_pos() -> None:
        async for p in sys.telemetry.position():
            armed["alt"] = float(p.relative_altitude_m)

    watchers = [asyncio.create_task(_w_armed()), asyncio.create_task(_w_pos())]

    async def _statustext() -> str:
        """Consume STATUSTEXT, emit progress, return the terminal verdict."""
        async for stt in sys.telemetry.status_text():
            text = str(getattr(stt, "text", "")).strip()
            low = text.lower()
            if "autotune" not in low and "tune" not in low:
                continue
            if "fail" in low or "abort" in low:
                _emit("failed", text)
                return "failed"
            if "success" in low or "complete" in low or "done" in low:
                _emit("success", text, pct=100.0)
                return "success"
            axis = next((a for a in _AXES if a in low), "")
            _emit("running", text, axis=axis, pct=50.0)
        return "failed"

    try:
        _emit("starting", "arming for autotune")
        try:
            await sys.action.arm()
        except Exception as e:
            logger.warning(f"[autotune] arm denied: {e}")
        for _ in range(40):
            if armed["v"]:
                break
            await asyncio.sleep(0.25)
        if not armed["v"]:
            _emit("failed", "never armed")
            return
        _emit("taking_off", "climbing to hold altitude")
        try:
            await sys.action.set_takeoff_altitude(12.0)
        except Exception:
            pass
        await sys.action.takeoff()
        for _ in range(240):
            if armed["alt"] >= 10.0:
                break
            await asyncio.sleep(0.25)
        _emit("mode_switch", "holding; starting PX4 autotune")
        try:
            await sys.action.hold()
        except Exception:
            pass
        await asyncio.sleep(2.0)
        await commander.set_param_int("MC_AT_APPLY", 2)   # apply + save on success
        await commander.set_param_int("MC_AT_START", 1)
        try:
            verdict = await asyncio.wait_for(_statustext(), timeout=720.0)
        except asyncio.TimeoutError:
            verdict = "failed"
            _emit("failed", "autotune timed out (12 min)")
        logger.info(f"[autotune] verdict={verdict}")
        if verdict == "success":
            # Read the autotuned rate/attitude gains back off the FC and persist
            # them so the mission auto-applies them (and they survive a SITL reboot).
            read: dict[str, float] = {}
            for name, _loop, _axis in _READBACK_PARAMS:
                try:
                    read[name] = float(await commander.get_param_float(name))
                except Exception:
                    pass
            if read:
                try:
                    save_gains(read, source="px4 autotune")
                    _emit("success", f"gains saved ({len(read)}) — mission auto-applies", pct=100.0)
                except Exception as e:
                    logger.warning(f"[autotune] save read-back failed: {e}")
    except asyncio.CancelledError:
        _emit("aborted", "cancelled")
        raise
    except Exception as e:
        logger.exception(f"[autotune] error: {e}")
        _emit("failed", str(e)[:120])
    finally:
        try:
            await commander.set_param_int("MC_AT_START", 0)
        except Exception:
            pass
        _emit("landing", "landing after autotune")
        try:
            await sys.action.land()
        except Exception as e:
            logger.warning(f"[autotune] land: {e}")
        for w in watchers:
            w.cancel()
