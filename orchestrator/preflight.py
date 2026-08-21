"""Pre-flight readiness check — the GO/NO-GO gate run before arm + takeoff.

A *detailed* checklist (vs the FC's opaque ``is_armable`` alone): each item is
``pass`` / ``warn`` / ``fail`` / ``pending`` with a human-readable reason, split
into CRITICAL (hard-block launch) and ADVISORY (operator-confirmed) tiers.

``run_preflight`` is a **pure** snapshot evaluator (reads ``state.telemetry`` +
the loaded plan + a camera-frame file stat) so it is trivially unit-testable and
cheap to re-run at ~1 Hz while the mission holds for the operator's GO. The gate
wiring (hold-for-GO with a dashboard, auto-proceed/timeout headless) lives in
``orchestrator.main``; the ``/api/cmd/preflight/*`` endpoints fire the GO.

Reuses ``safety._point_in_polygon`` (home-in-geofence) and the existing
telemetry/health fields — no new flight-control surface here.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from mission_brain.schemas import CommandKind, MissionPhase

from .safety import _point_in_polygon
from .state import OrchestratorState

# The camera bridge mirrors the nadir frame here continuously (independent of
# mission phase), so its file mtime is the right "camera feed alive" signal —
# the VisionWorker only *detects* during active phases, so its fixes are stale
# during PREFLIGHT.
NADIR_FRAME = Path("/tmp/aavc_nadir.jpg")

# Statuses (string-typed so the dict serialises straight onto the WS).
PASS, WARN, FAIL, PENDING = "pass", "warn", "fail", "pending"


@dataclass
class PreflightItem:
    """One checklist line."""

    id: str
    label: str
    status: str          # pass | warn | fail | pending
    critical: bool
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "status": self.status,
            "critical": self.critical,
            "detail": self.detail,
        }


@dataclass
class PreflightReport:
    """Full board for one evaluation."""

    items: list[PreflightItem]
    t_monotonic: float

    @property
    def all_critical_pass(self) -> bool:
        """True only when EVERY critical item is ``pass`` — the launch gate."""
        return all(i.status == PASS for i in self.items if i.critical)

    def failures(self) -> list[PreflightItem]:
        return [i for i in self.items if i.critical and i.status != PASS]

    def to_dict(self) -> dict[str, Any]:
        return {
            "t_monotonic": self.t_monotonic,
            "all_critical_pass": self.all_critical_pass,
            "items": [i.to_dict() for i in self.items],
        }


def _camera_status(frame: Path, max_age_s: float, now_wall: float) -> tuple[str, str]:
    try:
        if not frame.exists():
            return FAIL, f"{frame} missing — start the camera bridge"
        age = now_wall - frame.stat().st_mtime
        if age <= max_age_s:
            return PASS, f"nadir frame {age:.1f}s old"
        return FAIL, f"nadir frame stale ({age:.1f}s > {max_age_s:.0f}s)"
    except OSError as e:  # pragma: no cover — stat race
        return FAIL, f"camera frame unreadable: {e}"


def run_preflight(
    state: OrchestratorState,
    *,
    geofence: list[list[float]] | list[tuple[float, float]],
    home_lat: float,
    home_lon: float,
    min_battery_pct: float = 50.0,
    min_gps_sats: int = 6,
    min_gps_fix_type: int = 3,
    min_time_remaining_s: float = 180.0,
    camera_frame: Path = NADIR_FRAME,
    camera_max_age_s: float = 5.0,
    now_wall: float | None = None,
) -> PreflightReport:
    """Evaluate the readiness board from a single state snapshot. Pure."""
    t = state.telemetry
    now_wall = time.time() if now_wall is None else now_wall
    items: list[PreflightItem] = []

    def add(id_: str, label: str, status: str, critical: bool, detail: str = "") -> None:
        items.append(PreflightItem(id_, label, status, critical, detail))

    # ── CRITICAL — these hard-block launch ──
    add("link", "Datalink / heartbeat",
        PASS if (t.is_connected and state.link_connected) else FAIL,
        True, "MAVLink heartbeat present" if t.is_connected else "no link")

    add("armable", "FC armable (PX4 pre-arm)",
        PASS if t.is_armable else FAIL, True,
        "PX4 reports armable" if t.is_armable else "PX4 pre-arm checks NOT satisfied")

    ekf_ok = t.is_global_position_ok and t.is_local_position_ok
    add("ekf", "EKF position estimate",
        PASS if ekf_ok else FAIL, True,
        "global+local position OK" if ekf_ok
        else f"global={t.is_global_position_ok} local={t.is_local_position_ok}")

    add("home", "Home position set",
        PASS if t.is_home_position_ok else FAIL, True,
        "home set (RTH reference)" if t.is_home_position_ok else "home not yet set")

    cal_ok = (t.is_gyrometer_calibrated and t.is_accelerometer_calibrated
              and t.is_magnetometer_calibrated)
    add("sensors", "Gyro / accel / mag calibrated",
        PASS if cal_ok else FAIL, True,
        "all calibrated" if cal_ok
        else f"gyro={t.is_gyrometer_calibrated} accel={t.is_accelerometer_calibrated} "
             f"mag={t.is_magnetometer_calibrated}")

    gps_ok = t.gps_fix_type >= min_gps_fix_type and t.gps_satellites >= min_gps_sats
    add("gps", "GPS fix quality",
        PASS if gps_ok else FAIL, True,
        f"fix={t.gps_fix_type} sats={t.gps_satellites} "
        f"(need ≥{min_gps_fix_type}D, ≥{min_gps_sats} sats)")

    # CRITICAL, and deliberately a hard floor rather than a comfortable margin:
    # `min_battery_pct` is the FC's own low-battery threshold, so at or below it
    # the aircraft would take off straight into its failsafe. That is not an
    # operator judgment call, so this row has no force escape. Judging whether
    # the charge ABOVE that floor covers another sortie belongs to the energy
    # budget's advisory row + forceable refusal — it knows the measured cost.
    if t.battery_percent != t.battery_percent:  # NaN
        add("battery", "Battery charge", FAIL, True, "battery telemetry unavailable (NaN)")
    elif t.battery_percent >= min_battery_pct:
        add("battery", "Battery charge", PASS, True,
            f"{t.battery_percent:.0f}% (FC failsafe at {min_battery_pct:.0f}%)")
    else:
        add("battery", "Battery charge", FAIL, True,
            f"{t.battery_percent:.0f}% — at or below the FC's own low-battery "
            f"failsafe ({min_battery_pct:.0f}%); launching means taking off "
            "into it")

    on_ground = (not t.is_armed) and (
        t.relative_alt_m != t.relative_alt_m or t.relative_alt_m < 1.0)
    add("on_ground", "On ground & disarmed",
        PASS if on_ground else FAIL, True,
        "disarmed on the pad" if on_ground else f"armed={t.is_armed} alt={t.relative_alt_m:.1f}m")

    verts = [list(v) for v in geofence]
    geo_label = "Geofence + home inside"
    if len(verts) < 3:
        add("geofence", geo_label, FAIL, True, "geofence not loaded")
    elif not _point_in_polygon(home_lat, home_lon, verts):
        add("geofence", geo_label, FAIL, True, "home is OUTSIDE the geofence")
    else:
        add("geofence", geo_label, PASS, True, "home inside controlled airspace")

    # Blind search has NO pre-loaded pad coordinates — a sortie plan either
    # carries the boustrophedon sweep (assigned pad not yet in the registry) or
    # goes straight to a LOCALIZE goto (registry-known pad from an earlier
    # sortie). Either shape is launch-worthy; a plan with neither has no way to
    # reach the assigned pad.
    search_legs = sum(1 for c in state.plan.commands
                      if c.kind == CommandKind.GOTO and c.phase == MissionPhase.SEARCH)
    localize_legs = sum(1 for c in state.plan.commands
                        if c.kind == CommandKind.GOTO
                        and c.phase == MissionPhase.LOCALIZE)
    plan_ok = search_legs >= 2 or localize_legs >= 1
    add("search", "Sortie route loaded",
        PASS if plan_ok else FAIL, True,
        (f"{search_legs} search waypoints; pads discovered in flight"
         if search_legs >= 2 else
         f"registry-known pad ({localize_legs} localize goto)") if plan_ok
        else "plan has neither a search pattern nor a known-pad goto")

    # ADVISORY by design: the window-too-short refusal is owned by the
    # TimePolicy gate (`state.sortie_time_ok`) and the GO endpoint's
    # `sortie_time_ok || force` — the overtime penalty is the OPERATOR'S call
    # (locked decision). As a critical row it made FORCE a dead path: the
    # board could never be green exactly when force was needed (found live
    # 2026-07-15, sortie-4 hold at 2:57 remaining).
    remaining = state.time_remaining_s()
    add("time", "Operation window",
        PASS if remaining >= min_time_remaining_s else WARN, False,
        f"{remaining:.0f}s remaining (need ≥{min_time_remaining_s:.0f}s"
        + ("" if remaining >= min_time_remaining_s else " — FORCE to launch late")
        + ")")

    # ADVISORY for the same reason the window row is: the refusal lives in
    # `state.sortie_energy_ok` and the GO endpoint's `sortie_energy_ok || force`.
    # A critical row would make FORCE a dead path — the board could never be
    # green exactly when force is needed (the 2026-07-15 lesson, one row up).
    add("energy", "Battery energy budget",
        PASS if getattr(state, "sortie_energy_ok", True) else WARN, False,
        getattr(state, "energy_detail", "") or "not evaluated")

    # Third of the same family, and advisory for the same reason: the refusal is
    # `state.param_pins_ok || force` at the GO endpoint. The pins are read back
    # from the FC after the tuning block is pushed, because applying is
    # best-effort — "applied 0/24" (a stale mavsdk_server holding the ports) would
    # otherwise fly the mission on PX4 defaults: RTL at 60 m through the 20 m
    # ceiling, and 1.5 m/s onto the pad with the egg aboard.
    add("params", "Flight-envelope params",
        PASS if getattr(state, "param_pins_ok", True) else WARN, False,
        getattr(state, "param_pins_detail", "") or "not evaluated")

    cam_status, cam_detail = _camera_status(camera_frame, camera_max_age_s, now_wall)
    add("camera", "Nadir camera feed", cam_status, True, cam_detail)

    # ── ADVISORY — surfaced but do not block; operator confirms ──
    if t.datalink_rssi < 0:
        add("rssi", "RC / datalink signal", WARN, False, "not available (SITL / no RC link)")
    elif t.datalink_rssi >= 50:
        add("rssi", "RC / datalink signal", PASS, False, f"{t.datalink_rssi}%")
    else:
        add("rssi", "RC / datalink signal", WARN, False, f"weak ({t.datalink_rssi}%)")

    add("payload", "Egg cargo loaded (×1)", PENDING, False,
        "operator must confirm the egg is loaded + secured and enter the "
        "committee-assigned marker id before GO")

    return PreflightReport(items=items, t_monotonic=state.time_elapsed_s())
