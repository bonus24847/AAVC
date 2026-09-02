"""FastAPI app factory + uvicorn server wrapper for the dashboard.

Caller (orchestrator) builds the app with a reference to live state and
runs `DashboardServer.serve()` as an asyncio task. A dashboard crash
must NEVER take down the orchestrator — `DashboardServer.serve_safe()`
swallows exceptions and logs them.
"""

from __future__ import annotations

import contextlib
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from loguru import logger

from mission_brain.schemas import MissionPlan
from orchestrator.state import OrchestratorState

from .realtime import RealtimeBroadcaster
from .routes import (
    CAMERA_FRAME_PATH,
        CAMERA_SPECTATOR_PATH,
    make_router,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
WEB_DIST_DIR = REPO_ROOT / "dashboard" / "web" / "dist"


@contextlib.contextmanager
def _no_signal_capture() -> Iterator[None]:
    """No-op stand-in for uvicorn.Server.capture_signals (see DashboardServer.__init__).
    Same return type as the original so it drops in cleanly."""
    yield


def make_app(
    state: OrchestratorState,
    plan: MissionPlan,
    broadcaster: RealtimeBroadcaster,
    aavc_config_path: Path,
    camera_frame_path: Path = CAMERA_FRAME_PATH,
    camera_spectator_path: Path = CAMERA_SPECTATOR_PATH,
    enable_mjpeg: bool = True,
    commander: Any = None,
    app_mode: str = "mission",
) -> FastAPI:
    app = FastAPI(
        title="AAVC Dashboard",
        description="Live flight monitoring + command channel for AAVC 2026",
        docs_url="/api/docs",
    )
    app.include_router(make_router(
        state=state,
        plan=plan,
        broadcaster=broadcaster,
        aavc_config_path=aavc_config_path,
        camera_frame_path=camera_frame_path,
        camera_spectator_path=camera_spectator_path,
        enable_mjpeg=enable_mjpeg,
        commander=commander,
        app_mode=app_mode,
    ))

    if WEB_DIST_DIR.exists():
        app.mount("/static", StaticFiles(directory=WEB_DIST_DIR, html=True), name="static")

        @app.get("/", include_in_schema=False)
        async def root() -> RedirectResponse:
            return RedirectResponse("/static/index.html")
    else:
        @app.get("/", include_in_schema=False)
        async def root_no_static() -> dict[str, str]:
            return {
                "message": "Dashboard frontend not built. Run `npm run build` in dashboard/web/.",
                "api_docs": "/api/docs",
                "websocket": "/ws/realtime",
            }

    return app


class DashboardServer:
    """Lifecycle wrapper. Runs uvicorn inside the orchestrator's event loop."""

    def __init__(
        self,
        # Bind localhost by default. The command channel has NO token auth — its
        # only guard is the X-AAVC-CMD header (a CSRF mitigation, trivially set
        # by a non-browser client) + the arm toggle. That trust model assumes a
        # private/loopback bind; binding 0.0.0.0 previously let any host on the
        # LAN command the drone. Opt into LAN exposure explicitly via
        # AAVC_DASHBOARD_HOST (orchestrator main / run_dashboard), and add auth
        # before doing so.
        app: FastAPI,
        host: str = "127.0.0.1",
        port: int = 8765,
    ) -> None:
        self.app = app
        self.host = host
        self.port = port
        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="warning",  # don't double-log alongside loguru
            access_log=False,
            # Bound graceful shutdown: without this uvicorn waits FOREVER for
            # in-flight connections (a left-open dashboard WebSocket) to close,
            # hanging the orchestrator's post-mission teardown. See shutdown().
            timeout_graceful_shutdown=3,
            # The dashboard app declares no ASGI lifespan (startup/shutdown)
            # handlers; turning the protocol off stops uvicorn spawning a lifespan
            # task that logs a noisy CancelledError traceback when it's cancelled
            # at teardown (cosmetic, but it looks like a crash on every clean run).
            lifespan="off",
        )
        self.server = uvicorn.Server(config)
        # uvicorn's serve() wraps itself in `capture_signals()`, which installs its
        # own SIGINT/SIGTERM handler (handle_exit — it only flips uvicorn's
        # should_exit). Embedded as a background task in the orchestrator, that
        # HIJACKS Ctrl-C from asyncio.run: the operator's SIGINT then merely nudges
        # the web server and never cancels the mission task, so the vehicle flies on
        # and the process hangs (confirmed via an asyncio task dump — the run() task
        # was still parked in run_search_serve_mission after Ctrl-C). No-op the
        # capture so asyncio.run keeps its native SIGINT→cancel-the-mission path,
        # which then unwinds through the teardown finally (dash.stop + commander.close).
        # NB: uvicorn 0.49 installs via capture_signals(), not install_signal_handlers().
        self.server.capture_signals = lambda: _no_signal_capture()  # type: ignore[method-assign]

    async def serve_safe(self) -> None:
        """Catch any uvicorn-level exception and log; don't propagate to
        the orchestrator. If the port is taken or the SPA dir is missing,
        the dashboard just doesn't exist for this run — the mission still
        flies."""
        try:
            logger.info(f"[dashboard] starting on http://{self.host}:{self.port}")
            await self.server.serve()
        except Exception as e:
            logger.error(f"[dashboard] uvicorn crashed: {e}; continuing without dashboard")

    async def shutdown(self) -> None:
        # force_exit so uvicorn tears down WITHOUT draining in-flight connections:
        # an open dashboard WebSocket (a browser tab left on the page) otherwise
        # blocks shutdown indefinitely and hangs the post-mission teardown.
        self.server.should_exit = True
        self.server.force_exit = True
