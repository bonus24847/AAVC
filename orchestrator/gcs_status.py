"""Live ``mission_status.json`` feed for the AAVC GCS console (KMUTNB).

The console (~/Desktop/aavc-gcs/src/aavc_gcs.py) polls
``<captures>/mission_status.json`` and draws one map marker per entry in
``pads_mapped`` — so a pad appears on the operator's map exactly when THIS
feed says so. The user's contract (2026-08-12): pads show up **when the
drone actually scans/confirms them**, never from a stale file — which is
also why the constructor clobbers any pre-existing status file immediately
(the stale pads the operator saw came from another project's captures dir
holding a finished mission's layout).

File contract (aavc_gcs.py header + its ``aavcEN`` map helper)::

    {"phase": str, "assigned": [ids], "delivered": [ids],
     "pads_mapped": {"<marker_id>": [east_m, north_m]},
     "pads_identified": {"<marker_id>": [east_m, north_m]},   # orange lane
     "plan": [[lat, lon, kind, seq], ...], "plan_ptr": int,   # console map path
     #   plan_ptr is the DISPLAY seq of the leg being flown (1-based, matching
     #   the seq column), NOT an index into the mission's command list — the
     #   two differ wherever a command carries no coordinate.
     "run": str,                                             # mission run id
     "updated": epoch}

``pads_mapped`` ENU is about the field yaml's ``local_origin`` (== this
repo's ``site.center``), and the console converts it back with a SPHERICAL
earth (``aavcEN``, R=6378137) — so this module inverts that exact formula
rather than using the WGS84 metres-per-degree the rest of the repo flies
with: matching the consumer puts the marker on the true pad; "better" math
here would land it ~0.3 m off at this field's scale.

Threading/robustness: confirmations arrive on the VisionWorker THREAD and
releases on the asyncio loop, so state is lock-guarded; writes are atomic
(tmp + os.replace, the same pattern as the camera bridge) and every failure
is swallowed — this is a display aid, never flight-critical.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from pathlib import Path
from typing import Any, Callable

from loguru import logger

_R_EARTH_M = 6_378_137.0
# Grid the identified-pad lane snaps to before it counts as "changed" (m).
_IDENT_GRID_M = 0.5

# Same discriminator as sitl/payload_detach_bridge.parse_release, minus the
# payload capture: pad=None (an id-unverified touchdown release) deliberately
# fails the \d+ and is not shown as a delivered pad.
_RELEASE = re.compile(r"DELIVERY \d+ RELEASE pad=(?P<pad>\d+)")
_TRANSIT = re.compile(r"(?P<what>TRANSIT_PASS|TRANSIT_MISS) (?P<pt>P\d)")

# MissionPhase.value -> the console's mission bar (its phaseIdx() does
# SUBSTRING matching on [recon, deliver, done], so the label carries the raw
# phase in parentheses for operator detail without breaking the stepper).
# The "load" step was removed from the console 2026-08-12 (operator request —
# all four eggs ride one flight, so it never lit up): preflight now maps onto
# recon. The whole serve-egress-land tail lives under "deliver" (3 steps is
# the console's vocabulary, not ours). Terminal must be EXACTLY "done" — the
# console stops its mission clock only on that exact string (MCLOCK.done).
_STEP_OF = {
    "preflight": "recon",
    "takeoff": "recon",
    "transit_ingress": "recon",
    "search": "recon",
    "localize": "deliver",
    "drop": "deliver",
    "track": "deliver",
    "transit_egress": "deliver",
    "land": "deliver",
    "rth": "deliver",
    "abort": "deliver",
}


class GcsMissionStatus:
    """Best-effort writer of the AAVC GCS console's mission_status.json."""

    def __init__(self, path: Path | str, origin_lat: float, origin_lon: float,
                 assigned: list[int], serve_cost_s: float = 80.0,
                 run_id: str = "") -> None:
        self.path = Path(path)
        # Identity of THIS mission run, published so the console can tell a new
        # mission from a re-read of the same one. It matters for the plan
        # polyline: the console deliberately KEEPS the last plan drawn while
        # the link is dead, and with no run id it had no way to ever drop it —
        # so the route from the previous flight stayed on the map through the
        # next mission's whole preflight, reading as "this is where it is
        # going" (2026-08-22 review).
        self._run_id = str(run_id)
        self._origin = (float(origin_lat), float(origin_lon))
        self._lock = threading.Lock()
        self._pads: dict[str, list[float]] = {}
        # Identified-but-unconfirmed pads (marker id decoded at least once,
        # still short of the confirm votes) — the operator sees these ORANGE
        # the moment they are first read (request 2026-08-21: flight 1 was
        # pulled down while ids 4,5 were being identified live, because the
        # screen showed nothing until CONFIRMED).
        self._pads_identified: dict[str, list[float]] = {}
        # Live-plan polyline for the console map ([[lat, lon, kind, seq], …]
        # + the leg being flown) — first written at gate release while the
        # launch-point WiFi still reaches the console, so the operator can
        # see where the aircraft is going NEXT after the link dies (Fix 3,
        # G7 debrief 2026-08-21: takeover-before-time because the screen
        # could not answer exactly that).
        self._plan: list[list[Any]] = []
        self._plan_ptr = 0
        self._delivered: list[int] = []
        self._assigned = [int(i) for i in assigned]
        self._phase = "recon (preflight)"
        self._mission_time: float | None = None
        # ── progress + events (operator request 2026-08-14: "แถบ % แบบ
        # โปรแกรมโหลด") — display-aid numbers ONLY, never read by the mission ──
        self._serve_cost_s = float(serve_cost_s)
        self._progress = 0                 # 0..100 (monotonic within a flight)
        self._progress_label = "เตรียมพร้อม"
        self._eta_s: int | None = None
        self._search_t0: float | None = None   # mission_time when search began
        self._events: list[dict[str, Any]] = []  # rolling, newest last, max 10
        # transit passes ride a SEPARATE lane the rolling cap cannot evict, so
        # the P1·P2·P3 chip never un-ticks mid-flight; cleared at each FLIGHT START
        self._transit: list[dict[str, Any]] = []
        # WHY the aircraft came home / landed with the mission unfinished
        # (operator request 2026-08-18: "ผมจะได้รู้ว่ากลับ home เพราะอะไร").
        # First cause wins for the flight — the later consequences (an energy
        # refusal after a budget abort) don't overwrite the reason the
        # operator actually needs. Cleared at every FLIGHT START.
        # `code` is ASCII for the radio beacon; `reason` is the operator text.
        self._home_reason: str | None = None
        self._home_reason_code: str | None = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        # Startup write = the stale-pads fix: whatever mission_status.json a
        # previous run (or another project) left behind is replaced by an
        # empty pads_mapped before the console's next 1 Hz poll.
        self._write()
        logger.info(f"[gcs_status] live pad feed → {self.path}")

    # ── inverse of the console's aavcEN (spherical, see module docstring) ──
    def _enu(self, lat: float, lon: float) -> list[float]:
        lat0, lon0 = self._origin
        e = math.radians(lon - lon0) * _R_EARTH_M * math.cos(math.radians(lat0))
        n = math.radians(lat - lat0) * _R_EARTH_M
        return [round(e, 2), round(n, 2)]

    def pad_confirmed(self, marker_id: int, lat: float, lon: float) -> None:
        with self._lock:
            self._pads[str(int(marker_id))] = self._enu(lat, lon)
            # a pad that just got CONFIRMED leaves the identified lane
            self._pads_identified.pop(str(int(marker_id)), None)
        self._event(f"🎯 เจอ pad {int(marker_id)}!")
        self._write()

    def set_identified(self, mapping: dict[str, list[float]]) -> None:
        """Replace the identified-but-unconfirmed pad set ({id: [e, n]}).

        Writes only on CHANGE — and the change has to be one the OPERATOR
        could see. This rides the vision on_fix cadence (now every decoded
        frame, up to ~25 Hz), and the lane's ENU comes from a fused median
        that shifts by millimetres on every new vote, so comparing
        centimetre-rounded values made the guard almost never fire and the
        status file was rewritten per frame (2026-08-21 review). Quantising to
        _IDENT_GRID_M keeps the guard meaningful: a marker's position on the
        console map is not useful to half a metre anyway.
        A pad promoted to CONFIRMED simply stops appearing in ``mapping``
        (the tracker's identified_unconfirmed() no longer returns it)."""
        mapping = {k: [round(v[0] / _IDENT_GRID_M) * _IDENT_GRID_M,
                       round(v[1] / _IDENT_GRID_M) * _IDENT_GRID_M]
                   for k, v in mapping.items()}
        with self._lock:
            if mapping == self._pads_identified:
                return
            new_ids = [k for k in mapping if k not in self._pads_identified]
            self._pads_identified = {str(k): list(v) for k, v in mapping.items()}
        for k in sorted(new_ids, key=str):
            self._event(f"🔶 เห็น pad {k} (รอยืนยัน)")
        self._write()

    def set_plan(self, points: list[list[Any]], pointer: int) -> None:
        """Replace the console-map plan path ([[lat, lon, kind, seq], …]) and
        the DISPLAY seq of the leg being flown (see the file contract above:
        the caller translates, because command index != drawn index). Written
        on CHANGE only: a rebuild
        fires per gate release and per serve (cheap), but the pointer rides
        every rebuild too, so identical payloads short-circuit."""
        with self._lock:
            if points == self._plan and int(pointer) == self._plan_ptr:
                return
            self._plan = [list(p) for p in points]
            self._plan_ptr = int(pointer)
        self._write()

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self._phase = str(phase)
        self._write()

    def set_progress(self, raw_phase: str, mission_time_s: float, *,
                     delivered: int | None = None,
                     assigned: int | None = None) -> None:
        """1 Hz heartbeat from the orchestrator's REAL MissionPhase: drives the
        console's mission stepper + ⏱ clock + the milestone %-bar, and keeps
        ``updated`` fresh so the console's 45 s staleness gate never hides a
        live mission between sparse events."""
        step = _STEP_OF.get(str(raw_phase), "recon")
        with self._lock:
            self._phase = f"{step} ({raw_phase})"
            self._mission_time = float(mission_time_s)
            self._update_progress(str(raw_phase), float(mission_time_s),
                                  delivered, assigned)
        self._write()

    def _update_progress(self, phase: str, t: float,
                         delivered: int | None, assigned: int | None) -> None:
        """Milestone %-model (display aid; the mission never reads it):
        preflight 2 → takeoff 6 → transit 12 → search 20..55 (time creep,
        ~0.4 %/s ≈ a full sweep of this field) → serve 55..90 (by deliveries)
        → egress 92 → final land 97 → done 100. Monotonic within a flight —
        only a PREFLIGHT hold (next flight) may reset it."""
        n_d = len(self._delivered) if delivered is None else int(delivered)
        n_a = len(self._assigned) if assigned is None else int(assigned)
        n_a = max(n_a, 1)
        # MissionPhase.LAND covers BOTH the pad landing (tactical_align) and
        # the final L&R landing (mission.py) — indistinguishable here. So a
        # "land with eggs left" was read as "still delivering", and after a
        # DELIVERY abort the console said "ส่งของ pad 4 (3/4)" with pad 4
        # marked current while the aircraft was landing at home with the eggs
        # still aboard. The radio carries the same lie: the beacon's `cur=`
        # is scraped out of this label. A home_reason means the mission has
        # ALREADY decided it is coming home — after that, LAND is the L&R
        # landing (2026-08-22 review).
        coming_home = self._home_reason_code is not None
        serving = phase in ("localize", "drop", "track") or (
            phase == "land" and n_d < n_a and not coming_home)
        pending = [i for i in self._assigned if i not in self._delivered]
        cur_pad = pending[0] if pending else None
        eta = 0.0
        if phase == "preflight":
            pct, label = 2, "เตรียมพร้อม / รอปล่อย"
            eta = 90 + n_a * self._serve_cost_s + 70
            self._search_t0 = None
        elif phase == "takeoff":
            pct, label = 6, "กำลังขึ้นบิน"
            eta = 90 + n_a * self._serve_cost_s + 70
        elif phase == "transit_ingress":
            pct, label = 12, "บินเข้าเส้นทาง P1→P3"
            eta = 90 + (n_a - n_d) * self._serve_cost_s + 70
        elif phase == "search":
            if self._search_t0 is None:
                self._search_t0 = t
            pct = int(min(55.0, 20.0 + 0.4 * (t - self._search_t0)))
            label = "กวาดหา pad"
            eta = (55 - pct) / 0.4 + (n_a - n_d) * self._serve_cost_s + 70
        elif serving:
            base = 55.0 + 35.0 * (n_d / n_a)
            pct = int(min(90.0, base + 0.5 * 35.0 / n_a))
            label = (f"ส่งของ pad {cur_pad} ({n_d + 1}/{n_a})"
                     if cur_pad is not None else f"ส่งของ ({n_d}/{n_a})")
            eta = (n_a - n_d - 0.5) * self._serve_cost_s + 70
        elif phase in ("transit_egress", "rth") or (phase == "land" and coming_home
                                                    and n_d < n_a):
            pct, label = 92, ("บินกลับ (ยกเลิกที่เหลือ)" if n_d < n_a else "บินกลับ")
            eta = 70
        elif phase == "land":
            pct, label = 97, "กลับมาลงจอด"
            eta = 25
        else:                                   # abort/unknown — hold position
            pct, label = self._progress, self._progress_label
            eta = self._eta_s or 0
        if phase == "preflight":
            self._progress = pct                # next-flight hold may reset
        else:
            self._progress = max(self._progress, pct)
        self._progress_label = label
        self._eta_s = int(max(0.0, round(eta / 10.0) * 10))

    def set_done(self, mission_time_s: float) -> None:
        """Terminal write — exactly "done" (see _STEP_OF note)."""
        with self._lock:
            self._phase = "done"
            self._mission_time = float(mission_time_s)
            self._progress = 100
            self._progress_label = "จบภารกิจ"
            self._eta_s = 0
        self._write()

    def _event(self, text: str, warn: bool = False, sticky: bool = False) -> None:
        """Append to the operator-event feed (console toasts). ``sticky`` events
        — the transit passes the P1·P2·P3 chip is rebuilt from — go to a lane the
        rolling 10-cap cannot evict, else a flight's ingress passes get pushed
        out by the pad/release events that follow and the chip un-ticks."""
        with self._lock:
            row = {"t": round(time.time(), 1), "text": text, "warn": warn}
            if sticky:
                self._transit.append(row)
                del self._transit[:-8]        # 3 ingress + 3 egress + slack
            else:
                self._events.append(row)
                del self._events[:-10]

    # WHY-did-it-come-home table: (audit substring, ascii code for the radio
    # beacon, operator text). Ordered — first match in an entry wins. These
    # substrings are the audit grammar's own words (safety.py anomaly kinds +
    # the mission's decision lines), so a wording change there must land here.
    _HOME_REASONS = (
        ("DELIVERY abort", "budget",
         "งบเวลา/แบตไม่พอก่อนส่ง — กลับพร้อมไข่ที่เหลือ"),
        ("refused (energy reserve)", "energy",
         "พลังงานไม่พอสำหรับเที่ยวถัดไป — สลับแบตก่อนกด GO"),
        ("refused (envelope params)", "envelope",
         "พารามิเตอร์ซองบินบน FC ไม่ผ่าน — ตรวจด้วย preflight_params ก่อนบิน"),
        ("refused (time reserve)", "time-gate",
         "งบเวลาไม่พอสำหรับเที่ยวนี้ — ยังไม่ออกบิน"),
        ("gps_loss_sustained", "gps",
         "GPS หลุดต่อเนื่อง — ลงจอด ณ จุดที่อยู่"),
        ("battery_critical_", "batt-crit",
         "แบตวิกฤต — ลงจอดทันที ณ จุดที่อยู่"),
        ("battery_low_", "batt-low",
         "แบตต่ำกว่าเกณฑ์ — กลับบ้าน (RTH)"),
        ("battery_telemetry_nan_sustained", "batt-nan",
         "อ่านค่าแบตไม่ได้ต่อเนื่อง — กลับบ้าน (RTH)"),
        ("geofence_breach", "fence",
         "หลุดรั้ว geofence — กลับบ้าน (RTH)"),
        ("no_fly_zone_breach", "nofly",
         "เข้าเขตห้ามบิน — กลับบ้าน (RTH)"),
        ("altitude_ceiling_breach_sustained", "ceiling",
         "ทะลุเพดานบินค้าง — กลับบ้าน (RTH)"),
        ("telemetry_stale_sustained", "telem",
         "telemetry ขาดต่อเนื่อง — กลับบ้าน (RTH)"),
        ("datalink_loss_sustained", "datalink",
         "ลิงก์สั่งการหลุด — กลับบ้าน (RTH)"),
        ("time_budget_exhausted", "time",
         "หมดงบเวลา — กลับบ้าน (RTH)"),
        ("PILOT TAKEOVER", "pilot",
         "นักบินยึดเครื่องคืน — ระบบหยุดสั่งแล้ว"),
        ("FC FAILSAFE", "fc",
         "FC เข้า failsafe เอง (RTL/LAND) — ระบบหยุดสั่งแล้ว"),
    )

    def on_audit(self, entry: str) -> None:
        """Audit-sink tee: delivered ticks + the operator-event feed."""
        # A new flight wipes the previous flight's homecoming reason…
        if "FLIGHT" in entry and "START" in entry and "DELIVERY" not in entry:
            with self._lock:
                self._home_reason = None
                self._home_reason_code = None
                self._transit = []          # a new flight's transit chip starts blank
            self._write()
        else:
            # …and the FIRST terminal cause of this flight sticks (later
            # consequences — e.g. the energy refusal after a budget abort —
            # must not overwrite the reason the operator actually needs).
            for needle, code, text in self._HOME_REASONS:
                if needle in entry:
                    changed = False
                    with self._lock:
                        if self._home_reason_code is None:
                            self._home_reason = text
                            self._home_reason_code = code
                            changed = True
                    if changed:            # a repeat of the same cause changes nothing
                        self._write()
                    break
        m = _RELEASE.search(entry)
        if m is not None:
            pad = int(m.group("pad"))
            with self._lock:
                if pad not in self._delivered:
                    self._delivered.append(pad)
            self._event(f"📦 วางแล้ว pad {pad}")
            self._write()
            return
        mt = _TRANSIT.search(entry)
        if mt is not None:
            ok = mt.group("what") == "TRANSIT_PASS"
            self._event(("✅ ผ่านจุด " if ok else "⚠️ พลาดจุด ") + mt.group("pt"),
                        warn=not ok, sticky=True)
            self._write()
            return
        if "PILOT TAKEOVER" in entry:
            self._event("🛑 นักบินยึดเครื่องคืน — ระบบหยุดสั่งแล้ว", warn=True)
            self._write()
        elif "FC FAILSAFE" in entry:
            self._event("🛑 FC เข้า failsafe เอง — ระบบหยุดสั่งแล้ว ให้นักบินดูแล", warn=True)
            self._write()
        elif "DELIVERY abort" in entry:
            self._event("⚠️ ข้ามการส่งที่เหลือ (งบเวลา/แบตไม่พอ)", warn=True)
            self._write()

    def tracker_pusher(self, tracker: Any) -> Callable[[Any], None]:
        """VisionWorker ``on_fix`` callback: mirror each newly CONFIRMED,
        id-DECODED pad to the map the moment the tracker promotes it —
        independent of the web dashboard's own pusher, so headless
        (--no-dashboard) runs feed the console too. Unidentified blob-only
        clusters are withheld: the operator's map shows pads, not maybes.

        Since 2026-08-21 it ALSO pushes the identified-but-unconfirmed lane
        (id decoded, votes still short → ``pads_identified``): the G7 flight
        was pulled down while ids 4,5 were being identified live because the
        confirmed-only feed showed nothing. The confirmed ``seen`` latch is
        untouched — a pad flows identified → confirmed independently."""
        from .target_tracker import TargetState
        show = (TargetState.CONFIRMED, TargetState.SERVING, TargetState.SERVED)
        seen: set[int] = set()

        def _push(_fix: Any) -> None:
            for t in tracker.snapshot():
                if (t.marker_id is None or t.target_id in seen
                        or t.state not in show):
                    continue
                seen.add(t.target_id)
                self.pad_confirmed(t.marker_id, t.lat, t.lon)
                logger.info(f"[gcs_status] pad {t.marker_id} on the GCS map "
                            f"({t.lat:.7f}, {t.lon:.7f})")
            self.set_identified({
                str(t.marker_id): self._enu(t.lat, t.lon)
                for t in tracker.identified_unconfirmed()})

        return _push

    def _write(self) -> None:
        try:
            with self._lock:
                doc = {
                    "phase": self._phase,
                    "assigned": list(self._assigned),
                    "delivered": list(self._delivered),
                    "pads_mapped": dict(self._pads),
                    "pads_identified": dict(self._pads_identified),
                    "plan": [list(p) for p in self._plan],
                    "plan_ptr": self._plan_ptr,
                    "run": self._run_id,
                    "progress": int(self._progress),
                    "progress_label": self._progress_label,
                    "eta_s": self._eta_s,
                    "events": self._transit + list(self._events),
                    "home_reason": self._home_reason,
                    "home_reason_code": self._home_reason_code,
                    "updated": time.time(),
                }
                if self._mission_time is not None:
                    doc["mission_time"] = round(self._mission_time, 1)
            # A per-write temp name: the doc is built under the lock but the
            # file ops are not, and two writers (the vision thread and the
            # event loop) sharing one ".tmp" path truncate each other's
            # half-written file. The beacon then reads a torn document, gets
            # None, and broadcasts "no mission yet" mid-flight (2026-08-21).
            tmp = self.path.with_name(f"{self.path.name}.{os.getpid()}.{id(doc):x}.tmp")
            try:
                tmp.write_text(json.dumps(doc), encoding="utf-8")
                os.replace(tmp, self.path)
            finally:
                try:
                    tmp.unlink(missing_ok=True)   # no-op after a successful replace
                except OSError:
                    pass
        except Exception as e:  # display aid — never raise into the flight path
            logger.debug(f"[gcs_status] write skipped: {e}")
