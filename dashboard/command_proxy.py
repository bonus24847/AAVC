"""LoggedCommander — wraps DroneCommander to fire a callback after each
MAVLink command method call. The orchestrator already constructs the
commander at the top of `_run`; we swap in the wrapped version so the
dashboard's TacticalLog/CommandLog widgets can show what's been sent.

Read-only observation — the wrapper never blocks, never alters arguments,
and propagates exceptions from the underlying call unchanged. If a
callback raises, we log and continue (a busted dashboard must NOT take
down the orchestrator's command path).
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from loguru import logger

from mavlink_adapter.commands import DroneCommander


class LoggedCommander:
    """Proxy that fires `on_command(method_name, kwargs)` after each method.

    Delegates every public method on DroneCommander via __getattr__. Async
    methods are auto-wrapped to await the inner call before firing the
    callback. Sync attributes pass through untouched.
    """

    def __init__(
        self,
        wrapped: DroneCommander,
        on_command: Callable[[str, dict[str, Any]], None] | None = None,
    ) -> None:
        self._wrapped = wrapped
        self._on_command = on_command
        self._start = time.monotonic()

    # ---- subscriber registry ----

    def set_on_command(self, callback: Callable[[str, dict[str, Any]], None]) -> None:
        self._on_command = callback

    # ---- explicit wrappers for methods the orchestrator calls ----

    async def connect(self) -> None:
        await self._wrapped.connect()

    @property
    def system(self) -> Any:
        return self._wrapped.system

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        await self._wrapped.arm_and_takeoff(altitude_m)
        self._fire("arm_and_takeoff", {"altitude_m": altitude_m})

    async def goto(
        self, lat: float, lon: float, alt_m: float, yaw_deg: float = float("nan")
    ) -> None:
        await self._wrapped.goto(lat, lon, alt_m, yaw_deg)
        self._fire("goto", {"lat": lat, "lon": lon, "alt_m": alt_m, "yaw_deg": yaw_deg})

    async def drop_payload(self, payload_id: int = 0) -> None:
        await self._wrapped.drop_payload(payload_id)
        self._fire("drop_payload", {"payload_id": payload_id})

    async def land(self) -> None:
        await self._wrapped.land()
        self._fire("land", {})

    async def rth(self) -> None:
        await self._wrapped.rth()
        self._fire("rth", {})

    async def run_mission(
        self,
        items: list[Any],
        on_progress: Callable[[Any], None] | None = None,
        watchdog_should_stop: Callable[[], bool] | None = None,
    ) -> Any:
        result = await self._wrapped.run_mission(
            items, on_progress=on_progress, watchdog_should_stop=watchdog_should_stop,
        )
        self._fire("run_mission", {"n_items": len(items)})
        return result

    async def set_geofence_action_rtl(self) -> None:
        await self._wrapped.set_geofence_action_rtl()
        self._fire("set_geofence_action_rtl", {})

    async def set_datalink_loss_rtl(self, timeout_s: float = 10.0) -> None:
        await self._wrapped.set_datalink_loss_rtl(timeout_s)
        self._fire("set_datalink_loss_rtl", {"timeout_s": timeout_s})

    # ---- pass-through for anything else ----

    def __getattr__(self, name: str) -> Any:
        # Called only when normal attribute lookup fails on this wrapper.
        return getattr(self._wrapped, name)

    # ---- internal ----

    def _fire(self, method: str, kwargs: dict[str, Any]) -> None:
        if self._on_command is None:
            return
        try:
            self._on_command(method, kwargs)
        except Exception:
            logger.exception(f"[LoggedCommander] on_command callback raised for {method}")
