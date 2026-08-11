"""Standalone dashboard launcher (GCS-only mode).

Spins up the FastAPI dashboard and connects MAVSDK to a running PX4 SITL
or real vehicle. Unlike `orchestrator.main`, this does NOT run a mission
loop — it's the GCS-only mode for situational awareness + the manual
command channel (Takeoff / Hold / Resume / RTL / Land / Drop). Use case:
pre-flight checks, post-mission monitoring, debugging the UI.

Two operating modes:

  CONNECTED   — MAVSDK heartbeat received within the timeout. Telemetry
                sidebar shows live state; command channel flies the vehicle.

  NO LINK     — MAVSDK connect timed out. Dashboard still serves but
                `state.link_connected` is False, so the UI renders a
                NO-LINK banner. Command endpoints stay mounted (an
                operator-error click surfaces as a CommandResultEvent
                ok=false rather than vanishing).

Usage:
    python -m dashboard.scripts.run_dashboard
    python -m dashboard.scripts.run_dashboard --connect udpin://0.0.0.0:14540
    python -m dashboard.scripts.run_dashboard --no-mavlink   (skip connect)
"""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import yaml
from loguru import logger

from dashboard.integration import start_dashboard
from mavlink_adapter.commands import ConnectionConfig, DroneCommander
from mavlink_adapter.telemetry import TelemetrySubscriber
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import load_profile
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator.state import OrchestratorMode, OrchestratorState

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_PATH = REPO_ROOT / "sitl" / "aavc_config.yaml"


async def _try_connect(commander: DroneCommander, timeout_s: float) -> bool:
    """Best-effort MAVSDK connect. Returns True on a heartbeat, False on
    timeout — the dashboard serves either way (NO-LINK banner)."""
    try:
        await asyncio.wait_for(commander.connect(), timeout=timeout_s)
        return True
    except Exception as e:
        logger.warning(f"[run_dashboard] MAVSDK connect failed/timed out: {e}")
        return False


async def run(args: argparse.Namespace) -> int:
    cfg_path = Path(args.config)
    profile = load_profile(args.profile)

    # Minimal valid plan so OrchestratorState is well-formed: the stage-1 blind
    # search sweep over the controlled airspace. home is a placeholder until a fix
    # arrives (coarse, no-RTK GPS — the map just re-centres later).
    cfg = yaml.safe_load(cfg_path.read_text()) if cfg_path.exists() else {}
    geofence = [tuple(v) for v in (cfg or {}).get("controlled_airspace", [])]
    placeholder_home = Coordinate(lat=0.0, lon=0.0)
    spec = build_search_pattern(geofence, placeholder_home,
                                ceiling_m=profile.altitude_ceiling_m)
    plan = render_live_plan(placeholder_home, spec, discovered=[], profile=profile)

    commander = DroneCommander(ConnectionConfig(system_address=args.connect)
                               if args.connect else ConnectionConfig())
    telem = TelemetrySubscriber(commander.system)

    state = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem.state,
        operation_window_s=profile.operation_window_s,
    )

    connected = False
    if not args.no_mavlink:
        connected = await _try_connect(commander, args.connect_timeout_s)
        if connected:
            state.link_connected = True
            await telem.start()
            logger.info("[run_dashboard] MAVSDK connected; telemetry streaming")
        else:
            logger.info("[run_dashboard] serving in NO-LINK mode")
    else:
        logger.info("[run_dashboard] --no-mavlink: serving without a vehicle link")

    dash = await start_dashboard(
        state, commander, host=args.host, port=args.port, aavc_config_path=cfg_path,
    )
    logger.info(f"[run_dashboard] dashboard up at http://{args.host}:{args.port}")

    try:
        # Serve until interrupted — no mission loop, just situational awareness.
        while True:
            await asyncio.sleep(3600)
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("[run_dashboard] shutting down")
    finally:
        try:
            await dash.stop()
        except Exception:
            pass
        if connected:
            try:
                await telem.stop()
            except Exception:
                pass
    return 0


def main() -> None:
    p = argparse.ArgumentParser(description="AAVC standalone GCS dashboard monitor")
    p.add_argument("--config", default=str(CONFIG_PATH),
                   help="field config (geofence, site)")
    p.add_argument("--connect", default=None,
                   help="MAVLink endpoint (default udpin://0.0.0.0:14540)")
    p.add_argument("--connect-timeout-s", type=float, default=20.0)
    p.add_argument("--no-mavlink", action="store_true",
                   help="skip the MAVSDK connect attempt (NO-LINK mode)")
    p.add_argument("--profile", default="competition")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    raise SystemExit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
