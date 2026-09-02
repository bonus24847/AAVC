"""HTTP + WebSocket routes for the dashboard.

GET  /api/plan                  — full MissionPlan JSON (one-shot at page load)
GET  /api/config                — controlled_airspace + search_area + transit polygons
GET  /api/health                — orchestrator status
GET  /api/camera/frame.png      — single nadir frame from /tmp/aavc_frame.jpg
GET  /api/camera/stream         — multipart/x-mixed-replace MJPEG from /tmp/aavc_frame.jpg
GET  /api/camera/spectator.png  — single spectator frame from /tmp/aavc_spectator.png
GET  /api/tiles/{z}/{x}/{y}.jpg — pre-cached satellite tiles (404 → MapView empty cell)
WS   /ws/realtime               — live event stream

Plus the command sub-router (see commands.py): POST /api/cmd/* for
operator-issued mission verbs.
"""

from __future__ import annotations

import asyncio
import json
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml
from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from loguru import logger

from mission_brain.schemas import MissionPlan
from orchestrator.state import OrchestratorState, TerminalState

from .realtime import RealtimeBroadcaster

CAMERA_FRAME_PATH = Path("/tmp/aavc_frame.jpg")


def _frame_media_type(path: Path) -> str:
    """Content type of a frame file, from its SUFFIX.

    The URLs keep their historical ``.png`` names (bookmarks, the Svelte
    widget), so the type has to come from the file itself — browsers follow
    the header, not the path."""
    return ("image/jpeg" if path.suffix.lower() in (".jpg", ".jpeg")
            else "image/png")
CAMERA_SPECTATOR_PATH = Path("/tmp/aavc_spectator.png")
CAMERA_STREAM_HZ = 5.0
# Pre-cached OSM/ESRI tiles for the AAVC field. The directory may not
# exist (downloads are one-shot + gitignored); MapView falls back to
# OSM online if a tile 404s.
TILE_CACHE_DIR = Path(__file__).resolve().parent / "tiles"


def _ws_origin_ok(ws: WebSocket) -> bool:
    """Reject cross-origin WebSocket handshakes so a malicious page open in the
    operator's browser can't tap the live feed. A handshake with NO Origin
    (CLI / non-browser client, our own tests) is allowed — it can't be a CSRF
    vector. Browser handshakes pass only when same-origin (Origin == the Host
    header) or from a loopback origin. The command channel is already POST +
    custom-header gated; this closes the read-only telemetry leak the WS left."""
    origin = ws.headers.get("origin")
    if not origin:
        return True
    parsed = urlparse(origin)
    if parsed.hostname in ("localhost", "127.0.0.1", "::1"):
        return True
    return parsed.netloc == ws.headers.get("host", "")


def make_router(
    state: OrchestratorState,
    plan: MissionPlan,
    broadcaster: RealtimeBroadcaster,
    aavc_config_path: Path,
    camera_frame_path: Path = CAMERA_FRAME_PATH,
    camera_spectator_path: Path = CAMERA_SPECTATOR_PATH,
    enable_mjpeg: bool = True,
    commander: Any = None,
    app_mode: str = "mission",
) -> APIRouter:
    r = APIRouter()

    @r.get("/api/plan")
    async def get_plan() -> JSONResponse:
        # Read the LIVE plan: blind search rebuilds state.plan as targets are
        # discovered, so the startup-frozen `plan` arg would be stale. The
        # `plan_update` WS event repaints clients in between page loads.
        return JSONResponse(json.loads(state.plan.model_dump_json()))

    @r.get("/api/config")
    async def get_config() -> JSONResponse:
        try:
            cfg = yaml.safe_load(aavc_config_path.read_text())
        except Exception as e:
            return JSONResponse({"error": f"could not read config: {e}"}, status_code=500)
        return JSONResponse({
            "site": cfg.get("site", {}),
            "controlled_airspace": cfg.get("controlled_airspace", []),
            "search_area": cfg.get("search_area", []),
            "ground_operation": cfg.get("ground_operation", {}),
            "emergency_egress": cfg.get("emergency_egress", []),
            "no_fly_zones": cfg.get("no_fly_zones", []),
            # V1.3 mandatory corridor (rules Table 1); the old placeholder
            # key is retired — the flight core flies this route now.
            "primary_transit_route": cfg.get("transit_route", []),
            "marker": cfg.get("marker", {}),
            "mission": cfg.get("mission", {}),
            # Camera intrinsics (single nadir) — the map coverage overlay uses
            # the measured FOV/aspect instead of rebuilt-in constants.
            "cameras": cfg.get("cameras", {}),
        })

    @r.get("/api/health")
    async def get_health() -> JSONResponse:
        return JSONResponse({
            "running": state.terminal == TerminalState.RUNNING,
            "terminal": state.terminal.value,
            "mode": state.mode.value,
            # Which program this backend is: "tuning" (System-ID/Autotune, no
            # mission) or "mission" (the flight sortie). The SPA falls back to
            # this when the URL carries no ?mode= so a reused browser tab can't
            # land it in the wrong view.
            "app_mode": app_mode,
            "phase": state.phase.value,
            "command_pointer": state.command_pointer,
            "anomalies_count": len(state.anomalies),
        })

    # Single-frame snapshot endpoint. Some browsers no longer render
    # multipart/x-mixed-replace via <img>, so the CameraFeed widget polls
    # this URL with a cache-busting query param instead. Reads
    # /tmp/aavc_frame.jpg at request time, or 503 if no frame yet.
    @r.get("/api/camera/frame.png")
    async def camera_frame() -> FileResponse:
        if not camera_frame_path.is_file():
            raise HTTPException(
                status_code=503,
                detail="no camera frame on disk — start the camera bridge",
            )
        return FileResponse(
            camera_frame_path,
            # the frame file is JPEG since 2026-08-21 (the URL keeps its
            # historical .png name; browsers follow the Content-Type)
            media_type=_frame_media_type(camera_frame_path),
            headers={"Cache-Control": "no-store"},
        )

    @r.get("/api/camera/spectator.png")
    async def camera_spectator() -> FileResponse:
        """Single spectator-camera frame (/tmp/aavc_spectator.png) — the fixed
        third-person view of the launch pad / sys-ID hover column shown in the
        Tuning view's right rail."""
        if not camera_spectator_path.is_file():
            raise HTTPException(
                status_code=503,
                detail="no spectator frame on disk — start the camera bridge",
            )
        return FileResponse(
            camera_spectator_path,
            media_type="image/png",
            headers={"Cache-Control": "no-store"},
        )

    if enable_mjpeg:
        @r.get("/api/camera/stream")
        async def camera_stream(request: Request) -> StreamingResponse:
            boundary = "frame"

            async def gen() -> AsyncIterator[bytes]:
                period = 1.0 / CAMERA_STREAM_HZ
                last_bytes: bytes | None = None
                while True:
                    if await request.is_disconnected():
                        return
                    try:
                        b = camera_frame_path.read_bytes() if camera_frame_path.exists() else None
                    except Exception:
                        b = None
                    if b is not None and b != last_bytes:
                        last_bytes = b
                        # The part type must follow the FILE, not the URL:
                        # frames became JPEG on 2026-08-21 while this said
                        # image/png, and every consumer of the stream rendered
                        # nothing (the snapshot endpoint was fixed, this was
                        # missed).
                        yield (
                            f"--{boundary}\r\n"
                            f"Content-Type: {_frame_media_type(camera_frame_path)}\r\n"
                            f"Content-Length: {len(b)}\r\n\r\n"
                        ).encode("utf-8") + b + b"\r\n"
                    await asyncio.sleep(period)

            return StreamingResponse(
                gen(),
                media_type=f"multipart/x-mixed-replace; boundary={boundary}",
            )

    @r.get("/api/tiles/{z}/{x}/{y}.jpg")
    async def get_tile(z: int, x: int, y: int) -> FileResponse:
        """Serve a pre-cached satellite tile from disk. Cache layout is
        `dashboard/tiles/{z}/{x}/{y}.jpg`. Returns 404 if missing —
        MapView renders the cell empty (no remote fallback)."""
        # Path traversal is impossible — z/x/y are integer-parsed by FastAPI.
        path = TILE_CACHE_DIR / str(z) / str(x) / f"{y}.jpg"
        if not path.is_file():
            raise HTTPException(status_code=404, detail="tile not in local cache")
        return FileResponse(
            path,
            media_type="image/jpeg",
            headers={"Cache-Control": "public, max-age=86400, immutable"},
        )

    # Command channel — mounted only when a commander is wired through. In
    # read-only mode (tests, replay) the POST endpoints are absent so the
    # UI's command panel knows to grey out.
    from .commands import CommandSession, make_command_router
    command_session = CommandSession()

    if commander is not None:
        r.include_router(make_command_router(
            state=state, commander=commander, broadcaster=broadcaster,
            session=command_session,
        ))

    @r.websocket("/ws/realtime")
    async def ws_realtime(ws: WebSocket) -> None:
        if not _ws_origin_ok(ws):
            logger.warning(
                "[dashboard] rejecting cross-origin WS handshake "
                f"origin={ws.headers.get('origin')!r}"
            )
            await ws.close(code=1008)   # 1008 = policy violation
            return
        await ws.accept()
        broadcaster.add_client(ws)
        try:
            await broadcaster.send_hello(ws)
            # Just keep the connection open; broadcaster pushes to us.
            while True:
                try:
                    await asyncio.wait_for(ws.receive_text(), timeout=30.0)
                except asyncio.TimeoutError:
                    # Send a keepalive so HTTP proxies don't kill the conn.
                    try:
                        await ws.send_text('{"kind":"ping","payload":{}}')
                    except Exception:
                        break
        except WebSocketDisconnect:
            pass
        except Exception as e:
            logger.warning(f"[dashboard] ws closed: {e}")
        finally:
            broadcaster.remove_client(ws)

    return r
