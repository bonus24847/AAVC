"""Dashboard integration seam — the decoupled entry point orchestrator/main.py imports.

`start_dashboard(state, commander, *, host, port)` builds the live GCS stack
(RealtimeBroadcaster + FastAPI app + CommandSession + LoggedCommander),
starts uvicorn in the background, starts the broadcaster, and returns a
DashboardHandle. A dashboard failure must NEVER take down the mission:
uvicorn runs under `serve_safe` (swallows + logs) and the handle's `stop()`
is fully defensive.

The handle exposes exactly what main.py expects:
  - `.broadcaster`   — RealtimeBroadcaster (record_vision, record_drop,
                       record_detected_objects, record_command, …)
  - `.record_drop`   — broadcaster.record_drop, used as on_drop_prediction
  - `async def stop(self)` — graceful shutdown of uvicorn + broadcaster
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from pathlib import Path
from typing import Any

from loguru import logger

from orchestrator.drop_trajectory import DropPrediction
from orchestrator.state import OrchestratorState

from .command_proxy import LoggedCommander
from .realtime import RealtimeBroadcaster
from .server import DashboardServer, make_app

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = REPO_ROOT / "sitl" / "aavc_config.yaml"


class DashboardHandle:
    """Live handle returned by `start_dashboard`. Holds the running uvicorn
    task + broadcaster so the mission can push events and shut things down."""

    def __init__(
        self,
        broadcaster: RealtimeBroadcaster,
        server: DashboardServer,
        serve_task: asyncio.Task[None],
    ) -> None:
        self.broadcaster = broadcaster
        self._server = server
        self._serve_task = serve_task
        # Used by orchestrator/main.py as on_drop_prediction. Bound here so a
        # missing/renamed method surfaces at wiring time, not mid-flight.
        self.record_drop: Callable[[DropPrediction], None] = broadcaster.record_drop

    async def stop(self) -> None:
        """Graceful shutdown — never raises into the mission's finally block."""
        try:
            await self.broadcaster.stop()
        except Exception as e:
            logger.warning(f"[dashboard] broadcaster stop failed: {e}")
        try:
            await self._server.shutdown()
        except Exception as e:
            logger.warning(f"[dashboard] server shutdown failed: {e}")
        if self._serve_task is not None:
            self._serve_task.cancel()
            try:
                # Bound the await: a wedged uvicorn shutdown must not hang the
                # mission's teardown (force_exit above should make this prompt).
                await asyncio.wait_for(self._serve_task, timeout=5.0)
            except (asyncio.CancelledError, Exception):
                pass


async def start_dashboard(
    state: OrchestratorState,
    commander: Any,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    aavc_config_path: Path | None = None,
    app_mode: str = "mission",
) -> DashboardHandle:
    """Build + launch the dashboard, returning a DashboardHandle.

    `commander` is the live DroneCommander. It is wrapped in a
    LoggedCommander so dashboard command-channel verbs and the mirrored
    MAVLink trace land on the WS. The mission keeps using its own raw
    commander reference — this wrapper only backs the command endpoints.

    uvicorn is started as a background task under `serve_safe`, so a bound
    port / missing SPA dir just means "no dashboard this run" — the mission
    still flies.
    """
    cfg_path = aavc_config_path or DEFAULT_CONFIG_PATH

    broadcaster = RealtimeBroadcaster(state)

    # Wrap so command-channel + explicit verbs feed the command trace WS.
    logged = LoggedCommander(commander, on_command=broadcaster.record_command)

    app = make_app(
        state=state,
        plan=state.plan,
        broadcaster=broadcaster,
        aavc_config_path=cfg_path,
        commander=logged,
        app_mode=app_mode,
    )

    server = DashboardServer(app, host=host, port=port)
    serve_task = asyncio.create_task(server.serve_safe())
    await broadcaster.start()

    logger.info(f"[dashboard] integration up at http://{host}:{port}")
    return DashboardHandle(broadcaster, server, serve_task)
