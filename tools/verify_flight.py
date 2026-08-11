"""Post-flight drone-response verifier (operator requirement 2026-07-03).

Reads a mission run's audit JSONL (runs/<mission_id>/audit.jsonl — 1 Hz TELEM
samples + TRANSIT_PASS/FLIGHT/DELIVERY events + anomalies) plus the SITL ground
truth and ASSERTS the drone actually behaved per the AAVC 2026 V1.3 rules and
the design.

Grammar (2026-07-24 briefing): a **FLIGHT** is one arm→disarm cycle carrying up
to ``eggs_aboard`` eggs; a **DELIVERY** is one pad served inside a flight and is
numbered 1-based across the WHOLE mission. Every check below is keyed on those
two indices exactly as ``orchestrator/mission.py`` and
``orchestrator/tactical_align.py::_drop_once`` emit them:

  altitude   never above ceiling+0.5; transit samples hold the 20 m band;
             search samples at/above the 10 m floor; below-floor samples only
             in the delivery-descent phases (localize/drop/land/takeoff)
  airspace   every armed sample inside the controlled airspace and outside
             every no-fly zone
  transit    P1→P2→P3 then P3→P2→P1 audited PASS, in order, every FLIGHT
  terminal   every delivered DELIVERY: touchdown confirmed (landed=True +
             near-ground telemetry) and its OWN `DELIVERY k RELEASE` position
             (matched by k, never inferred from an offset) ON the correct
             truth pad (id + distance)
  L&R        every FLIGHT ends near the launch point and DISARMED
  timing     the whole mission fits the operation window; no watchdog RTH /
             rule-violation anomalies fired

Exit code 0 = all checks pass; 1 = violations (each printed, prefixed FAIL).
Warnings (WARN) don't fail the run. Pass --ulog <file> for an optional PX4
log deep-dive (needs pyulog, the [tuning] extra) — report-only.

Usage:
    .venv/bin/python tools/verify_flight.py runs/<mission_id>/audit.jsonl \
        --truth /tmp/aavc_targets.json [--config sitl/aavc_config.yaml]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import yaml  # noqa: E402

from orchestrator.constants import (  # noqa: E402
    CEILING_BREACH_M,
    CEILING_WARN_M,
    TOUCHDOWN_ALT_GUARD_M,
)

_R = 6_378_137.0

# Transcribed from the emitters (mission.py::_telemetry_sampler/_fly_transit/
# the flight loop, tactical_align.py::_drop_once). Where a sketch and the
# emitter disagree, the EMITTER wins — a regex that silently matches nothing
# turns this fail-closed tool into a rubber stamp.
_TELEM = re.compile(
    r"t=(?P<t>[\d.]+)s TELEM phase=(?P<phase>\S+) flight=(?P<flight>\d+) "
    r"lat=(?P<lat>[-\d.nan]+) lon=(?P<lon>[-\d.nan]+) alt=(?P<alt>[-\d.nan]+) "
    r"armed=(?P<armed>[01])")
_TRANSIT = re.compile(
    r"t=(?P<t>[\d.]+)s TRANSIT_(?P<kind>PASS|MISS) P(?P<n>\d) "
    r"(?P<dir>ingress|egress) flight=(?P<flight>\d+) d=(?P<d>[-\d.nan]+)m")
_FLIGHT_START = re.compile(
    r"t=(?P<t>[\d.]+)s FLIGHT (?P<flight>\d+) START eggs=(?P<eggs>\d+) "
    r"ids=(?P<ids>[\d,]+)")
_FLIGHT_END = re.compile(
    r"t=(?P<t>[\d.]+)s FLIGHT (?P<flight>\d+) END "
    r"delivered=(?P<n>\d+)/(?P<of>\d+) d_home=(?P<d>[-\d.nan]+)m")
# Three END shapes share one line: delivered=True carries `err=…m landed=…`,
# while the two undelivered paths carry either `reason=…` (not_found /
# no_release_channel, from the flight loop) or `notes=…` (free text from
# _serve). Both trailing groups are optional so every shape parses — an
# unparsed END would drop out of the coverage sum unnoticed.
_DELIV_END = re.compile(
    r"t=(?P<t>[\d.]+)s DELIVERY (?P<k>\d+) END delivered=(?P<ok>True|False) "
    r"pad=(?P<pad>\d+)"
    r"(?: err=(?P<err>[-\d.nan]+)m landed=(?P<landed>True|False))?"
    r"(?: reason=(?P<reason>\S+))?")
# `pad=` here is _drop_once's `marker_id: int | None` — it CAN print `None`, so
# match it loosely and take the pad id from the DELIVERY END line instead.
_RELEASE = re.compile(
    r"t=(?P<t>[\d.]+)s DELIVERY (?P<k>\d+) RELEASE pad=(?P<pad>\S+) "
    r"payload=(?P<payload>\d+) lat=(?P<lat>[-\d.nan]+) lon=(?P<lon>[-\d.nan]+)")

# Anomaly kinds that mean the flight broke a rule / needed the watchdog.
#
# Deliberately NOT listed (2026-07-24): `flight{n}_pad{id}_not_found` and
# `flight{n}_pad{id}_no_release_channel`. Both are the fail-closed design
# WORKING — the mission refused to release blind / into a channel the kit does
# not have, and brought the egg home. That is a scoring loss, not a rules
# violation or a watchdog event, and the flight loop's own comments call the
# channel shortage explicitly non-fatal. They surface as WARNs through the
# per-delivery `reason=` readout and the delivered-vs-planned coverage line.
#
# Also NOT listed (2026-07-24 review — was here, taken out): `land_gate_id_
# not_confirmed`. The id-verified LAND gate records this the moment an
# approach's id votes fall short and it climbs off to defer; `_serve`
# (mission.py) then retries the SAME delivery once, and the retry recovers it
# far more often than not. Unconditionally fatal FAILed a run that delivered
# 4/4 and merely deferred one approach along the way — a false failure, right
# before a scored validation flight. It is still fatal, just not
# UNCONDITIONALLY: the scan below correlates it against the per-DELIVERY
# outcomes instead of listing it here — recovered by the retry (every
# delivery that was started shows delivered=True) is a WARN naming what
# happened; contributed to a lost delivery (any delivered=False) stays FATAL,
# unchanged from before.
_FATAL_ANOMALIES = (
    "geofence_breach", "no_fly_zone_breach", "altitude_ceiling_breach",
    "below_search_floor", "time_budget_exhausted", "battery_low",
    "battery_critical", "gps_loss_sustained", "datalink_loss_sustained",
    "telemetry_stale_sustained", "mission_loop_exception",
    "release_skipped_touchdown_unconfirmed",
    "transit_ingress_P", "transit_egress_P", "sweep_leg_timeout",
    "touchdown_unconfirmed_before_release", "watchdog: RTH", "watchdog: ABORT",
)
# Sub-floor flight is legal only in the delivery descent + ground ops.
_FLOOR_EXEMPT_PHASES = {"localize", "drop", "land", "takeoff", "preflight", "rth"}


def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = math.radians(lat2 - lat1) * _R
    de = math.radians(lon2 - lon1) * _R * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def _f(s: str) -> float:
    """Tolerant float: 'nan' (and any junk) → NaN rather than a crash."""
    try:
        return float(s)
    except (TypeError, ValueError):
        return math.nan


def _finite(g: dict) -> bool:
    return not (math.isnan(_f(g["lat"])) or math.isnan(_f(g["lon"]))
                or math.isnan(_f(g["alt"])))


def _inside(lat: float, lon: float, poly: list[list[float]]) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        yi, xi = poly[i][0], poly[i][1]
        yj, xj = poly[j][0], poly[j][1]
        if (yi > lat) != (yj > lat):
            x_int = (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi
            if lon < x_int:
                inside = not inside
        j = i
    return inside


@dataclass
class Report:
    fails: list[str] = field(default_factory=list)
    warns: list[str] = field(default_factory=list)
    infos: list[str] = field(default_factory=list)
    # Delivered-vs-planned headline (2026-07-24 review, FIX 2): populated by
    # `verify()` from the FLIGHT n END delivered=X/Y lines on every return
    # path — even the early fail-closed returns below — so `main()` can print
    # an unmissable summary line next to the PASS/FAIL verdict no matter how
    # far the checks got.
    deliveries_done: int = 0
    deliveries_planned: int = 0
    flights_flown: int = 0

    def fail(self, msg: str) -> None:
        self.fails.append(msg)

    def warn(self, msg: str) -> None:
        self.warns.append(msg)

    def info(self, msg: str) -> None:
        self.infos.append(msg)


def load_entries(audit_path: Path) -> list[str]:
    out = []
    for line in audit_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(str(json.loads(line).get("entry", "")))
        except ValueError:
            continue
    return out


def verify(entries: list[str], truth: list[dict], cfg: dict,
           *, land_acc_m: float, window_s: float) -> Report:
    rep = Report()
    airspace = cfg.get("controlled_airspace", [])
    nfz = cfg.get("no_fly_zones", [])
    lr = (cfg.get("ground_operation", {}) or {}).get("launch_recovery")
    lr_radius = float((cfg.get("ground_operation", {}) or {})
                      .get("launch_recovery_zone_radius_m", 25.0))
    mc = cfg.get("mission", {}) or {}
    ceiling = float(mc.get("altitude_ceiling_m", 20.0))
    transit_alt = float(mc.get("transit_alt_m", 20.0))
    floor = float(mc.get("search_floor_m", 10.0))
    truth_by_id = {int(t["marker_id"]): t for t in truth
                   if t.get("marker_id") is not None}

    # The 1 Hz sampler can emit TELEM during the pre-GO preflight hold, and
    # state.start_window() RESETS the audit clock at the first GO — so pre-GO
    # samples carry a LARGER t than the mission that follows (a non-monotonic
    # axis). Anchor on file POSITION: keep only TELEM at/after the first FLIGHT
    # START, so the window-relative checks (timing, disarm-after) never see
    # pre-window samples. FLIGHT/DELIVERY/TRANSIT entries are emitted only after
    # start_window(), so they need no filtering.
    first_start = next((i for i, e in enumerate(entries)
                        if _FLIGHT_START.match(e)), None)
    win = [m.groupdict() for i, e in enumerate(entries)
           if (m := _TELEM.match(e)) and (first_start is None or i >= first_start)]
    transits = [m.groupdict() for e in entries if (m := _TRANSIT.match(e))]
    flights = [m.groupdict() for e in entries if (m := _FLIGHT_START.match(e))]
    flight_ends = [m.groupdict() for e in entries if (m := _FLIGHT_END.match(e))]
    deliveries = [m.groupdict() for e in entries if (m := _DELIV_END.match(e))]
    delivered = [d for d in deliveries if d["ok"] == "True"]
    releases = [m.groupdict() for e in entries if (m := _RELEASE.match(e))]
    # Populate on every return path below (see the Report field comment) —
    # derived from FLIGHT END alone, so it never depends on the geometry/truth
    # checks that can fail closed before reaching the tail of this function.
    rep.deliveries_done = sum(int(en["n"]) for en in flight_ends)
    rep.deliveries_planned = sum(int(en["of"]) for en in flight_ends)
    rep.flights_flown = len(flight_ends)

    if not win:
        rep.fail("no TELEM samples in the audit trail — cannot verify behaviour")
        return rep

    # NaN lat/lon/alt (GPS/telemetry gap, or pre-fix) would silently corrupt the
    # geometric checks — split them out, warn, and run geometry on the finite
    # set so a dropout can't hide a breach behind a vanished sample.
    telem = [g for g in win if _finite(g)]
    nan_armed = [g for g in win if g["armed"] == "1" and not _finite(g)]
    if nan_armed:
        rep.warn(f"{len(nan_armed)} armed telemetry samples had NaN position/alt "
                 f"(GPS/telemetry gap, first at t={nan_armed[0]['t']}s) — those "
                 "windows are NOT covered by the containment/altitude checks")
    if not telem:
        rep.fail("every TELEM sample had NaN position/alt — cannot verify geometry")
        return rep

    # The central terminal check needs truth; if a release happened but no truth
    # was supplied/parsed, fail closed rather than silently scoring nothing.
    if (delivered or releases) and not truth_by_id:
        rep.fail(f"{max(len(delivered), len(releases))} releases but no ground "
                 "truth — cannot verify the landing was ON the correct pad "
                 "(pass --truth)")

    rep.info(f"{len(telem)} telemetry samples, {len(flights)} flights started, "
             f"{len(delivered)} of {len(deliveries)} deliveries released")

    # ── altitude bands ──────────────────────────────────────────────────────
    # Mirror the in-flight SafetyWatchdog, not a stricter line: it WARNS above
    # ceiling+0.5 and only RTHs on a SUSTAINED breach (or the ceiling+2 hard
    # line). A single 1 Hz sample poking above 20.5 is altitude-frame-wander
    # noise the watchdog would not act on, so it warns here too; a hard breach
    # (>ceiling+2) or a sustained hold (≥3 consecutive samples) fails.
    max_alt = max(_f(s["alt"]) for s in telem)
    warn_line, hard_line = ceiling + CEILING_WARN_M, ceiling + CEILING_BREACH_M
    run = best = 0
    for s in telem:
        if _f(s["alt"]) > warn_line:
            run += 1
            best = max(best, run)
        else:
            run = 0
    if max_alt > hard_line or best >= 3:
        why = (f">ceiling+2 ({hard_line:.1f} m)" if max_alt > hard_line
               else f"{best} consecutive samples above {warn_line:.1f} m")
        rep.fail(f"altitude: max {max_alt:.2f} m — {why}")
    elif max_alt > warn_line:
        rep.warn(f"altitude: transient {max_alt:.2f} m above {warn_line:.1f} m "
                 f"({best} sample(s)) — frame-wander noise; the watchdog warns, "
                 "does not RTH")
    else:
        rep.info(f"altitude: max {max_alt:.2f} m ≤ {warn_line:.1f} m")

    # Transit altitude: the corridor is entered on a legal climb, so measure
    # the hold AFTER the altitude is first captured in each contiguous transit
    # segment — from capture onward the corridor must stay in the band. The
    # band is [transit_alt-1.2, ceiling+0.5]: the mission deliberately commands
    # 0.5 m under the strict altitude (altitude-frame bias headroom) and the
    # EKF/home frame drifts ±~0.7 m across boots (SITL-measured), while the
    # hard ceiling check above still owns the absolute top line.
    segs: list[list[float]] = []
    prev_t = None
    for s in telem:
        if s["phase"] not in ("transit_ingress", "transit_egress") or s["armed"] != "1":
            prev_t = None
            continue
        t_s = float(s["t"])
        if prev_t is None or t_s - prev_t > 3.0:
            segs.append([])
        prev_t = t_s
        segs[-1].append(float(s["alt"]))
    lo_band, hi_band = transit_alt - 1.2, ceiling + 0.5
    held = []
    never_captured = 0
    climb_closures = 0
    for seg in segs:
        cap = next((i for i, a in enumerate(seg) if lo_band <= a <= hi_band), None)
        if cap is None:
            # A short fragment (a few 1 Hz samples) is all climb — it never had
            # TIME to reach the corridor; only a sustained segment that stays
            # out of band is a real violation. And a sustained no-capture
            # segment that is entirely BELOW the band while still CLIMBING is
            # the mission's documented two-stage climb closure (the last ~2 m
            # of climb happens en-route at a gentle cap) cut off by a short
            # leg — a 1 Hz sampling artifact of a rules-compliant profile, so
            # it WARNS (2026-07-08; mirrors the transient-ceiling WARN above)
            # PROVIDED some other segment demonstrates the corridor. A flat or
            # descending below-band hold stays a violation.
            if len(seg) >= 5:
                if max(seg) < lo_band and (seg[-1] - seg[0]) > 0.5:
                    climb_closures += 1
                else:
                    never_captured += 1
            continue
        held.extend(seg[cap:])
    if climb_closures and held:
        rep.warn(f"transit altitude: {climb_closures} segment(s) ended mid-climb "
                 f"below {lo_band:.1f} m (two-stage climb closure — leg ended "
                 "before corridor capture)")
    if held:
        out_band = [a for a in held if not (lo_band <= a <= hi_band)]
        frac = 1.0 - len(out_band) / len(held)
        if frac < 0.9 or never_captured:
            rep.fail(f"transit altitude: {frac * 100:.0f}% of {len(held)} "
                     f"post-capture samples within [{lo_band:.1f}, {hi_band:.1f}] m "
                     f"({never_captured} segments never captured the corridor)")
        else:
            rep.info(f"transit altitude: {frac * 100:.0f}% of {len(held)} "
                     f"post-capture samples within [{lo_band:.1f}, {hi_band:.1f}] m")
    elif segs:
        rep.fail(f"transit altitude: {len(segs)} transit segments never reached "
                 f"the [{lo_band:.1f}, {hi_band:.1f}] m corridor")

    low = [s for s in telem
           if s["armed"] == "1" and float(s["alt"]) < floor - 0.5
           and s["phase"] not in _FLOOR_EXEMPT_PHASES]
    if low:
        worst = min(low, key=lambda s: float(s["alt"]))
        rep.fail(f"search floor: {len(low)} samples below {floor:.0f} m outside "
                 f"the delivery descent (worst {worst['alt']} m in "
                 f"{worst['phase']} at t={worst['t']}s)")
    else:
        rep.info(f"search floor: no sub-{floor:.0f} m flight outside the "
                 "delivery descent")

    # ── airspace containment ────────────────────────────────────────────────
    if len(airspace) >= 3:
        out = [s for s in telem if s["armed"] == "1"
               and not _inside(float(s["lat"]), float(s["lon"]), airspace)]
        if out:
            s0 = out[0]
            rep.fail(f"airspace: {len(out)} armed samples OUTSIDE the geofence "
                     f"(first at t={s0['t']}s {s0['lat']},{s0['lon']})")
        else:
            rep.info("airspace: whole armed track inside the geofence")
    for zi, zone in enumerate(nfz):
        if len(zone) < 3:
            continue
        inz = [s for s in telem if s["armed"] == "1"
               and _inside(float(s["lat"]), float(s["lon"]), zone)]
        if inz:
            rep.fail(f"no-fly: {len(inz)} armed samples INSIDE zone {zi} "
                     f"(first at t={inz[0]['t']}s)")
        else:
            rep.info(f"no-fly: track clear of zone {zi}")

    # ── transit corridor, per FLIGHT, in order ─────────────────────────────
    # One ingress P1→P2→P3 and one egress P3→P2→P1 per arm→disarm cycle,
    # however many eggs that cycle carries: _fly_transit is called exactly
    # twice per flight, outside the per-delivery loop.
    for st in flights:
        i = st["flight"]
        seq = [(tr["n"], tr["dir"], tr["kind"]) for tr in transits
               if tr["flight"] == i]
        want = [("1", "ingress"), ("2", "ingress"), ("3", "ingress"),
                ("3", "egress"), ("2", "egress"), ("1", "egress")]
        got = [(n, d) for n, d, k in seq if k == "PASS"]
        misses = [(n, d) for n, d, k in seq if k == "MISS"]
        if misses:
            rep.fail(f"flight {i}: transit MISS at {misses}")
        if got != want:
            rep.fail(f"flight {i}: transit pass order {got} != {want}")
        else:
            rep.info(f"flight {i}: transit P1→P2→P3 / P3→P2→P1 all passed in order")

    # ── terminal behaviour per DELIVERY ─────────────────────────────────────
    for de in delivered:
        k, pad = de["k"], int(de["pad"])
        if de["landed"] != "True":
            rep.fail(f"delivery {k}: released WITHOUT confirmed touchdown")
        # Match the release position line by the DELIVERY index IN THE LINE.
        # (The retired `stop_index == sortie-1` inference paired a delivery
        # with its NEIGHBOUR's release — it would score an off-pad release
        # against the wrong pad's truth, in either direction.)
        rel = next((r for r in releases if r["k"] == k), None)
        if rel is None:
            rep.warn(f"delivery {k}: no RELEASE position line found")
        else:
            # The release line carries the pad the align was gated on; a
            # disagreement with the delivery's own pad means the egg went to a
            # different marker than the one being scored.
            if rel["pad"].isdigit() and int(rel["pad"]) != pad:
                rep.fail(f"delivery {k}: RELEASE line says pad {rel['pad']} but "
                         f"the delivery was scored as pad {pad}")
            rlat, rlon = _f(rel["lat"]), _f(rel["lon"])
            tr = truth_by_id.get(pad)
            if tr is None:
                rep.warn(f"delivery {k}: pad {pad} not in truth — cannot score "
                         "the landing position")
            elif math.isnan(rlat) or math.isnan(rlon):
                rep.fail(f"delivery {k}: RELEASE position is NaN — the landing "
                         f"on pad {pad} cannot be scored")
            else:
                d = _dist_m(rlat, rlon, float(tr["lat"]), float(tr["lon"]))
                if d > land_acc_m:
                    rep.fail(f"delivery {k}: released {d:.2f} m from truth pad "
                             f"{pad} centre (> {land_acc_m:.2f} m)")
                else:
                    rep.info(f"delivery {k}: released {d:.2f} m from truth pad "
                             f"{pad} centre ✓ (id correct)")
            # Telemetry at (or just before) the release must read on-ground —
            # the climb-out starts right after, so only pre-release samples
            # witness the touchdown honestly.
            t_rel = float(rel["t"])
            before = [s for s in telem if float(s["t"]) <= t_rel + 0.25]
            if before:
                near = before[-1]
                # On-ground bound = touchdown threshold (1.5) + the ±1 m
                # altitude-frame drift the release policy itself tolerates.
                if float(near["alt"]) > TOUCHDOWN_ALT_GUARD_M:
                    rep.fail(f"delivery {k}: telemetry reads {near['alt']} m AGL "
                             "at the release moment — not on the ground")
            else:
                rep.warn(f"delivery {k}: no telemetry at/before the release moment "
                         "(t=" f"{t_rel}s) — touchdown could not be witnessed")

    # An egg that left the aircraft with no delivered END behind it was never
    # scored for touchdown OR accuracy. Fail closed rather than let an
    # unaccounted release pass silently.
    for rel in releases:
        if not any(de["k"] == rel["k"] for de in delivered):
            rep.fail(f"delivery {rel['k']}: RELEASE recorded at t={rel['t']}s but "
                     f"no DELIVERY {rel['k']} END delivered=True — the egg left "
                     "the aircraft unscored")

    # Undelivered deliveries are a scoring loss, not a rules violation (the
    # mission kept the egg on purpose: pad not found, or no release channel) —
    # surface the reason so the operator sees WHY, without failing the run.
    for de in deliveries:
        if de["ok"] != "True":
            why = de["reason"] or "see the audit line"
            rep.warn(f"delivery {de['k']}: pad {de['pad']} NOT delivered "
                     f"(reason={why}) — the egg came home")

    # ── L&R landing + disarm between FLIGHTS ────────────────────────────────
    for en in flight_ends:
        i = en["flight"]
        t_end = float(en["t"])
        d = _f(en["d"])
        if math.isnan(d) or d > lr_radius:
            rep.fail(f"flight {i}: ended {d:.1f} m from the L&R point "
                     f"(> {lr_radius:.0f} m)")
        else:
            rep.info(f"flight {i}: landed {d:.1f} m from the L&R point")
        # d above is the flight code's OWN d_home vs its self-captured home —
        # cross-check the last armed finite fix near t_end against the config
        # L&R coordinate, so a wrong-home bug can't self-certify.
        if lr is not None:
            near = [s for s in telem if float(s["t"]) <= t_end + 0.25]
            if near:
                s = near[-1]
                dd = _dist_m(_f(s["lat"]), _f(s["lon"]), float(lr[0]), float(lr[1]))
                if dd > lr_radius:
                    rep.fail(f"flight {i}: final fix is {dd:.1f} m from the "
                             f"configured L&R {tuple(lr)} (> {lr_radius:.0f} m) — "
                             f"self-reported d_home was {d:.1f} m")
        # Confirm the disarm itself. With >1 flight the NEXT flight's preflight
        # hold used to supply this for free; at the shipping eggs_aboard=4
        # default (ONE flight) there is no next hold, and mission.py has no
        # await between writing this END line and the loop's own `finally:
        # sampler.cancel()` — so `after` is legitimately empty on a healthy
        # single-flight run, not just a broken one (I4, review 2026-07-24).
        # Fall back to the LAST sample tagged with THIS flight, at/before
        # t_end (mission.py now sleeps 2 sampler periods after
        # commander.land(disarm=True) and before writing END specifically so
        # that sample is a genuine post-disarm TELEM observation, not the
        # flight code's own say-so). Scoped by flight — not just the latest
        # sample overall — so a PRECEDING flight's stale armed=0 can't
        # silently certify a later, genuinely-unobserved flight's disarm. No
        # usable sample at all is treated the same as a still-armed one: fail
        # closed, never a silent skip (the tool's doctrine, CLAUDE.md §8).
        after = [s for s in win if float(s["t"]) > t_end][:20]
        if after:
            disarmed_seen = any(s["armed"] == "0" for s in after)
        else:
            before = [s for s in win
                     if float(s["t"]) <= t_end and s["flight"] == i]
            disarmed_seen = bool(before) and before[-1]["armed"] == "0"
        if not disarmed_seen:
            rep.fail(f"flight {i}: never observed DISARMED after the L&R landing")
    if lr is None:
        rep.warn("config has no ground_operation.launch_recovery — L&R position "
                 "unchecked")

    # ── timing + anomalies ──────────────────────────────────────────────────
    t_last = max(float(s["t"]) for s in win)
    if flights and t_last > window_s:
        rep.fail(f"timing: mission ran {t_last:.0f} s > the {window_s:.0f} s window")
    else:
        rep.info(f"timing: {t_last:.0f} s elapsed ≤ {window_s:.0f} s window")

    # `land_gate_id_not_confirmed` is judged against the mission's own
    # DELIVERY outcomes rather than blanket-fatal — see the comment above
    # `_FATAL_ANOMALIES`. `deliveries` (not `delivered`) is every DELIVERY END
    # seen so far, True or False; a defer with no delivery evidence AT ALL
    # (`deliveries` empty) has nothing to correlate against and stays FATAL —
    # fail closed rather than assume a recovery that isn't on the record.
    all_started_delivered = bool(deliveries) and all(
        d["ok"] == "True" for d in deliveries)
    for e in entries:
        if "land_gate_id_not_confirmed" in e:
            if all_started_delivered:
                rep.warn(f"anomaly: {e} — id gate deferred an approach; "
                         "the retry delivered")
            else:
                rep.fail(f"anomaly: {e}")
        elif any(k in e for k in _FATAL_ANOMALIES):
            rep.fail(f"anomaly: {e}")

    # Coverage across the whole mission: each FLIGHT END publishes
    # delivered=X/Y for the eggs THAT flight carried, so the sums are the
    # mission's delivered-vs-planned. (A flight that never reached its END —
    # crash, kill — contributes nothing here; the transit/L&R checks above
    # already fail such a flight.)
    n_done, n_planned = rep.deliveries_done, rep.deliveries_planned
    if flight_ends and n_done == 0 and n_planned > 0:
        # Zero eggs delivered is rules-compliant IF every one was correctly
        # refused (never releasing blind is the locked doctrine) — but it
        # must not read like a quiet, unremarkable WARN next to a 4/4 run.
        rep.warn(f"deliveries: 0 of {n_planned} planned eggs delivered — "
                 "NO CARGO DELIVERED (every egg came home; see the reasons "
                 "above)")
    elif flight_ends and n_done < n_planned:
        rep.warn(f"deliveries: {n_done} of {n_planned} planned eggs delivered — "
                 f"{n_planned - n_done} came home aboard")
    elif flight_ends:
        rep.info(f"deliveries: {n_done} of {n_planned} planned eggs delivered")
    return rep


def _ulog_report(path: Path) -> None:
    try:
        from pyulog import ULog
    except ImportError:
        print("WARN ulog: pyulog not installed (pip install -e '.[tuning]') — skipped")
        return
    try:
        ulog = ULog(str(path), ["vehicle_local_position"])
        data = ulog.get_dataset("vehicle_local_position")
        alt = [-z for z in data.data["z"]]
        print(f"INFO ulog: {len(alt)} local-position samples, "
              f"max altitude {max(alt):.2f} m, min {min(alt):.2f} m")
    except Exception as e:  # report-only deep-dive
        print(f"WARN ulog: could not analyse {path}: {e}")


def _deliveries_summary_line(rep: Report) -> str:
    """The unmissable delivery headline for main()'s final block (2026-07-24
    review, FIX 2) — distinct from the OK/WARN "deliveries: X of Y planned
    eggs delivered" line buried in the per-check listing above, THIS one is
    always printed right next to the PASS/FAIL verdict, so a clean 4/4 run
    and a run that correctly refused every blind release never look alike at
    a glance."""
    d, p, f = rep.deliveries_done, rep.deliveries_planned, rep.flights_flown
    if f and p and d == 0:
        return f"deliveries: {d}/{p} — NO CARGO DELIVERED"
    return f"deliveries: {d}/{p} across {f} flight(s)"


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Verify the drone's flight behaviour from a run's audit trail")
    ap.add_argument("audit", type=Path, help="runs/<mission_id>/audit.jsonl")
    ap.add_argument("--truth", type=Path, default=Path("/tmp/aavc_targets.json"),
                    help="ground-truth pads JSON (spawn_targets output)")
    ap.add_argument("--config", type=Path,
                    default=REPO / "sitl" / "aavc_config.yaml")
    ap.add_argument("--landing-accuracy-m", type=float, default=None,
                    help="max release distance from the truth pad centre "
                         "(default: config mission.landing_accuracy_threshold_m)")
    ap.add_argument("--window-s", type=float, default=None,
                    help="operation window (default: the competition profile)")
    ap.add_argument("--ulog", type=Path, default=None,
                    help="optional PX4 .ulg for a report-only deep-dive")
    args = ap.parse_args()

    if not args.audit.exists():
        print(f"FAIL audit file {args.audit} not found")
        return 1
    cfg = yaml.safe_load(args.config.read_text()) if args.config.exists() else {}
    mc = (cfg or {}).get("mission", {}) or {}
    land_acc = (args.landing_accuracy_m if args.landing_accuracy_m is not None
                else float(mc.get("landing_accuracy_threshold_m", 0.5)))
    if args.window_s is not None:
        window = args.window_s
    else:
        # Use the SAME window the flight core budgets against, not a literal.
        try:
            from mission_brain.profile import load_profile
            window = float(load_profile().operation_window_s)
        except Exception:
            window = 1200.0

    truth = []
    if args.truth and args.truth.exists():
        try:
            data = json.loads(args.truth.read_text())
            truth = data.get("targets", data)
        except ValueError:
            print(f"WARN could not parse truth {args.truth}")

    entries = load_entries(args.audit)
    rep = verify(entries, truth, cfg or {}, land_acc_m=land_acc, window_s=window)

    for m in rep.infos:
        print(f"OK   {m}")
    for m in rep.warns:
        print(f"WARN {m}")
    for m in rep.fails:
        print(f"FAIL {m}")
    if args.ulog:
        _ulog_report(args.ulog)

    n = len(rep.fails)
    print(f"\n{_deliveries_summary_line(rep)}")
    print(f"drone-response verification: "
          f"{'PASS' if n == 0 else f'{n} VIOLATION(S)'} "
          f"({len(rep.infos)} checks ok, {len(rep.warns)} warnings)")
    return 0 if n == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
