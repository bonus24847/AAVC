#!/usr/bin/env python3
"""Is this GPS module good enough to fly, and is that one better?

    .venv/bin/python tools/gps_bench.py                      # 90 s, via the CM4 router
    .venv/bin/python tools/gps_bench.py --label m10-a --save
    .venv/bin/python tools/gps_bench.py --from-console       # while a console owns the link
    .venv/bin/python tools/gps_bench.py --compare m10-a m10-b

**Satellite count is not quality, and believing it was cost this project a
module swap that had to be re-measured.** Two receivers benched back to back on
2026-08-25, same spot, minutes apart, warmed to the same sat count:

    unit          sats     HDOP      sigma_E     CEP95     alt swing
    #1            16.3     1.11      2.38 m      4.96 m      0.2 m
    #2            15.8     0.97      0.27 m      0.97 m      0.0 m

#1 saw MORE satellites and scattered **5x wider**. The number every GCS puts on
screen ranked them backwards. What separates them is the spread of the fixes
about their own mean, which nothing in the flight stack displays — hence this
tool.

Why the spread is the number that matters HERE, in metres this repo already
uses:

  * ``TargetTracker.cluster_radius_m`` = **8.0 m** — a pad fix further than this
    from an existing cluster starts a NEW one. A receiver whose 95% radius is a
    large fraction of that can split one pad into two registry entries, or (with
    two real pads nearby) merge them. That is a wrong-pad landing, not a wobble.
  * ``AlignParams.rung_tol_m[0]`` = **1.5 m** — the horizontal lock the first
    descent rung demands. Vision owns the final metre, but GPS has to deliver
    the aircraft into the camera's footprint first.

So the verdict thresholds are CEP95 against those constants, not against a
generic "good GPS" number: <= 2.0 m clears the first rung with margin, <= 4.0 m
is half the cluster radius (one pad still lands in one cluster), and beyond that
the registry itself is at risk.

⚠ What this does NOT prove: one 90 s window at one spot. It ranks modules under
the conditions of the moment. It cannot tell a bad receiver from a bad place to
stand — for that, re-measure the loser at the same spot and see if the result
follows the module or the ground.

Exit: 0 good · 1 usable but marginal · 2 poor (or never got a 3D fix) ·
3 could not reach the vehicle.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

# Where --save/--compare keep their runs. /tmp is deliberate: these are bench
# measurements of a swappable part, not repo history.
_STORE = Path("/tmp")

# Verdict thresholds on CEP95 (m) — see the module docstring for where these
# two numbers come from. They are mission constants, not taste.
_CEP95_GOOD = 2.0
_CEP95_USABLE = 4.0


@dataclass
class GpsSample:
    """One GPS_RAW_INT, reduced to what quality depends on."""
    t: float
    lat: float
    lon: float
    fix_type: int
    sats: int
    hdop: float | None = None
    alt_m: float | None = None


@dataclass
class GpsQuality:
    label: str = ""
    n: int = 0
    n_used: int = 0          # of those, the ones with a 3D fix (scatter uses ONLY these)
    secs: float = 0.0
    fix_best: int = 0
    sats_min: int = 0
    sats_max: int = 0
    sats_avg: float = 0.0
    hdop_min: float | None = None
    hdop_max: float | None = None
    sigma_n_m: float = 0.0
    sigma_e_m: float = 0.0
    cep50_m: float = 0.0
    cep95_m: float = 0.0
    worst_m: float = 0.0
    drift_m: float = 0.0
    alt_swing_m: float | None = None
    radii_m: list[float] = field(default_factory=list)


_FIX = {0: "no fix", 1: "dead-reckoning", 2: "2D", 3: "3D",
        4: "3D-DGPS", 5: "RTK-float", 6: "RTK-fixed"}


def local_metres(samples: list[GpsSample]) -> tuple[list[float], list[float]]:
    """(north, east) offsets in metres from the run's own mean position.

    Ellipsoidal scale, matching sitl/spawn_targets.py — the equatorial radius on
    the north axis is the bug that once biased every measured touchdown distance
    (CLAUDE.md, truth-coordinate fix 2026-07-04), and a benchmark that repeats it
    would quietly under-report north scatter.
    """
    lat0 = statistics.fmean(s.lat for s in samples)
    lon0 = statistics.fmean(s.lon for s in samples)
    phi = math.radians(lat0)
    # WGS84 meridional / prime-vertical radii at this latitude
    a, e2 = 6378137.0, 6.69437999014e-3
    w = math.sqrt(1.0 - e2 * math.sin(phi) ** 2)
    m_per_deg_lat = math.pi / 180.0 * a * (1.0 - e2) / w**3
    m_per_deg_lon = math.pi / 180.0 * a * math.cos(phi) / w
    north = [(s.lat - lat0) * m_per_deg_lat for s in samples]
    east = [(s.lon - lon0) * m_per_deg_lon for s in samples]
    return north, east


def summarise(samples: list[GpsSample], label: str = "") -> GpsQuality:
    """Reduce a run to the numbers that rank one receiver against another."""
    q = GpsQuality(label=label, n=len(samples))
    if len(samples) < 2:
        return q
    q.secs = round(samples[-1].t - samples[0].t, 1)
    q.fix_best = max(s.fix_type for s in samples)
    # Scatter is computed over 3D fixes ONLY, and this is not fussiness. Both
    # sources here keep serving the LAST KNOWN lat/lon after the fix is lost —
    # the console by design, PX4 in GPS_RAW_INT — so a run that walks indoors
    # half way through ends with a column of identical coordinates. Averaged in,
    # frozen positions do not merely dilute the spread, they shrink it towards
    # zero: the worse the reception, the better the module would score. Caught
    # on the first real run of this tool (2026-08-25), which recorded 20 stale
    # samples reading a flawless 0.00 m.
    usable = [s for s in samples if s.fix_type >= 3]
    q.n_used = len(usable)
    if len(usable) < 2:
        return q
    samples = usable
    sats = [s.sats for s in samples]
    q.sats_min, q.sats_max = min(sats), max(sats)
    q.sats_avg = round(statistics.fmean(sats), 1)
    hd = [s.hdop for s in samples if s.hdop is not None]
    if hd:
        q.hdop_min, q.hdop_max = round(min(hd), 2), round(max(hd), 2)
    north, east = local_metres(samples)
    # Population sigma (pstdev), not sample: these ARE the whole population of
    # fixes taken, and n is small enough that the Bessel correction would move
    # the number the operator compares.
    q.sigma_n_m = round(statistics.pstdev(north), 2)
    q.sigma_e_m = round(statistics.pstdev(east), 2)
    radii = sorted(math.hypot(n, e) for n, e in zip(north, east))
    q.radii_m = [round(r, 3) for r in radii]
    q.cep50_m = round(radii[len(radii) // 2], 2)
    q.cep95_m = round(radii[min(len(radii) - 1, int(len(radii) * 0.95))], 2)
    q.worst_m = round(radii[-1], 2)
    q.drift_m = round(math.hypot(north[-1] - north[0], east[-1] - east[0]), 2)
    alts = [s.alt_m for s in samples if s.alt_m is not None]
    if alts:
        q.alt_swing_m = round(max(alts) - min(alts), 1)
    return q


def classify(q: GpsQuality) -> tuple[int, str]:
    """(exit code, one-line verdict) — thresholds justified in the docstring."""
    if q.fix_best < 3:
        return 2, f"ไม่เคยได้ 3D fix เลย (ดีสุด = {_FIX.get(q.fix_best, q.fix_best)})"
    if q.n_used < 5:
        return 2, (f"มี 3D fix แค่ {q.n_used} จุดจาก {q.n} — น้อยเกินกว่าจะสรุป "
                   "(ต้องอยู่กลางแจ้ง และรอ cold start ~1 นาทีก่อนวัด)")
    if q.n_used < q.n * 0.8:
        return 2, (f"fix หลุดระหว่างวัด — ใช้ได้ {q.n_used} จาก {q.n} จุด "
                   "ตัวเลขกระจายตัวไม่น่าเชื่อถือ วัดใหม่")
    if q.cep95_m <= _CEP95_GOOD:
        return 0, (f"ดี — CEP95 {q.cep95_m} m อยู่ในระยะ lock ของ rung แรก "
                   f"({_CEP95_GOOD} m)")
    if q.cep95_m <= _CEP95_USABLE:
        return 1, (f"พอใช้ — CEP95 {q.cep95_m} m ยังไม่ถึงครึ่งของ cluster_radius "
                   "8 m แต่กินระยะ lock ของ rung แรกไปแล้ว")
    return 2, (f"แย่ — CEP95 {q.cep95_m} m เทียบ cluster_radius 8 m: "
               "แพดเดียวอาจแตกเป็นสอง cluster ในทะเบียน")


# ---- collection ----------------------------------------------------------

def _install_pymavlink_guard() -> None:
    """pymavlink 2.4.49 raises TypeError from its instanced-message bookkeeping
    on PX4 1.17 messages and DROPS whatever arrived with it (CLAUDE.md G5
    tooling note). Same guard aavc_gcs.py installs."""
    import copy

    from pymavlink import mavutil

    def _safe(messages, mtype, msg):  # noqa: ANN001, ANN202
        if not hasattr(msg, "instance_field") or msg.instance_field is None:
            messages[mtype] = msg
            return
        try:
            iv = getattr(msg, msg.instance_field)
        except AttributeError:
            messages[mtype] = msg
            return
        prev = getattr(messages[mtype], "_instances", None) if mtype in messages else None
        if prev is None:
            messages[mtype] = copy.copy(msg)
            messages[mtype]._instances = {iv: msg}
            messages[f"{mtype}[{iv}]"] = copy.copy(msg)
            return
        prev[iv] = msg
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = prev
        messages[f"{mtype}[{iv}]"] = copy.copy(msg)

    mavutil.add_message = _safe


def collect_mavlink(endpoint: str, seconds: float, *, quiet: bool = False) -> list[GpsSample]:
    """GPS_RAW_INT off the wire — the RAW receiver output, deliberately not the
    EKF-fused GLOBAL_POSITION_INT: fusing baro and IMU would flatter a bad
    module, which is the opposite of what a receiver bench is for."""
    _install_pymavlink_guard()
    from pymavlink import mavutil

    m = mavutil.mavlink_connection(endpoint, source_system=254, source_component=190)
    # Several heartbeats, not one: a mavlink-router UDP *server* endpoint only
    # learns a client from traffic, and a single packet before a blocking wait
    # loses the race (seen 2026-08-25).
    for _ in range(5):
        m.mav.heartbeat_send(6, 8, 0, 0, 0)
        time.sleep(0.2)
    if m.wait_heartbeat(timeout=15) is None:
        raise ConnectionError(f"ไม่มี heartbeat จาก {endpoint}")
    # Ask for GPS_RAW_INT at 1 Hz. Best-effort: PX4's default rate on some links
    # is far below that and a 90 s run would then yield a handful of samples.
    try:
        m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, 24, 1000000, 0, 0, 0, 0, 0)
    except Exception:  # noqa: BLE001 — the stream may already be fast enough
        pass

    out: list[GpsSample] = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1)
        except Exception:  # noqa: BLE001 — flaky serial: keep sampling
            continue
        if msg is None:
            continue
        if msg.lat == 0 and msg.lon == 0:
            continue
        out.append(GpsSample(
            t=time.time(), lat=msg.lat / 1e7, lon=msg.lon / 1e7,
            fix_type=msg.fix_type, sats=msg.satellites_visible,
            hdop=None if msg.eph >= 9999 else msg.eph / 100.0,
            alt_m=msg.alt / 1000.0))
        if not quiet and len(out) % 10 == 0:
            print(f"  … {len(out)} จุด  fix {_FIX.get(msg.fix_type, msg.fix_type)}"
                  f"  sats {msg.satellites_visible}")
    return out


def collect_console(url: str, seconds: float, *, quiet: bool = False) -> list[GpsSample]:
    """Same numbers, read from a running console's /api/status.

    For when the console already owns the only link to the aircraft: two readers
    on one serial port split the byte stream and both see nonsense, so borrowing
    the console's copy beats fighting it for the port."""
    import urllib.request

    _FIXNUM = {v: k for k, v in _FIX.items()}
    out: list[GpsSample] = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            with urllib.request.urlopen(f"{url.rstrip('/')}/api/status", timeout=4) as fh:
                g = json.load(fh)["gps"]
        except Exception:  # noqa: BLE001 — console restarting; keep trying
            time.sleep(1.5)
            continue
        if g.get("lat") is not None:
            out.append(GpsSample(
                t=time.time(), lat=g["lat"], lon=g["lon"],
                fix_type=_FIXNUM.get(g.get("fix_str", ""), g.get("fix", 0)),
                sats=g.get("sats", 0), hdop=g.get("hdop"), alt_m=g.get("alt")))
            if not quiet and len(out) % 10 == 0:
                print(f"  … {len(out)} จุด  fix {g.get('fix_str')}  sats {g.get('sats')}")
        time.sleep(1.5)
    return out


# ---- reporting -----------------------------------------------------------

def report(q: GpsQuality) -> None:
    print(f"\n===== {q.label or 'gps'} =====")
    dropped = q.n - q.n_used
    print(f"  {q.n} จุด / {q.secs} วิ · fix ดีสุด = {_FIX.get(q.fix_best, q.fix_best)}"
          + (f" · ทิ้ง {dropped} จุดที่ไม่มี 3D fix" if dropped else ""))
    if q.n_used < 2:
        # Print NOTHING numeric here. Every scatter field is still 0.0 (nothing
        # was measured), and a column of 0.00 m reads as a flawless receiver at
        # a glance — the precise misreading this tool exists to prevent.
        print("  ยังไม่มีข้อมูลพอจะคิดการกระจายตัว (ไม่มีจุดที่ 3D fix)")
        return
    print(f"  ดาวเทียม      {q.sats_min}-{q.sats_max} (เฉลี่ย {q.sats_avg})"
          "   <- อย่าตัดสินจากบรรทัดนี้")
    print(f"  HDOP          {q.hdop_min}-{q.hdop_max}")
    print(f"  กระจายแนวราบ   sigma_N {q.sigma_n_m} m · sigma_E {q.sigma_e_m} m")
    print(f"  CEP50 {q.cep50_m} m · CEP95 {q.cep95_m} m · ไกลสุด {q.worst_m} m")
    print(f"  ไหลต้น-ท้าย    {q.drift_m} m")
    if q.alt_swing_m is not None:
        print(f"  ความสูง GPS แกว่ง  {q.alt_swing_m} m")


def _path(label: str) -> Path:
    return _STORE / f"gps_bench_{label}.json"


def save(q: GpsQuality) -> Path:
    p = _path(q.label)
    body = {k: v for k, v in q.__dict__.items() if k != "radii_m"}
    p.write_text(json.dumps(body, indent=1))
    return p


def compare(labels: list[str]) -> int:
    runs = []
    for lab in labels:
        p = _path(lab)
        if not p.exists():
            print(f"[gps-bench] ไม่พบผลของ '{lab}' ที่ {p} — รันด้วย --label {lab} --save ก่อน")
            return 3
        runs.append(json.loads(p.read_text()))
    rows = [("ดาวเทียม (เฉลี่ย)", "sats_avg", False), ("HDOP แย่สุด", "hdop_max", True),
            ("sigma เหนือ-ใต้ (m)", "sigma_n_m", True), ("sigma ออก-ตก (m)", "sigma_e_m", True),
            ("CEP50 (m)", "cep50_m", True), ("CEP95 (m)", "cep95_m", True),
            ("หนีไกลสุด (m)", "worst_m", True), ("ไหลต้น-ท้าย (m)", "drift_m", True),
            ("ความสูงแกว่ง (m)", "alt_swing_m", True)]
    w = max(len(r[0]) for r in rows)
    print("\n" + "ตัวชี้วัด".ljust(w) + "".join(f"{r['label']:>12s}" for r in runs))
    print("-" * (w + 12 * len(runs)))
    for name, key, lower_better in rows:
        vals = [r.get(key) for r in runs]
        best = None
        got = [v for v in vals if v is not None]
        if got:
            best = min(got) if lower_better else max(got)
        line = name.ljust(w)
        for v in vals:
            mark = " *" if (v is not None and v == best and len(runs) > 1) else "  "
            line += f"{str(v):>10s}{mark}"
        print(line)
    print("\n  * = ดีกว่าในบรรทัดนั้น")
    print("  ⚠ วัดคนละช่วงเวลา/อาจคนละจุด — จัดอันดับได้ แต่ยังไม่พิสูจน์ว่าตัวที่แพ้เสีย")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14550",
                    help="MAVLink endpoint (default: the CM4 router's QGC port)")
    ap.add_argument("--from-console", nargs="?", const="http://127.0.0.1:8000",
                    metavar="URL",
                    help="อ่านผ่าน /api/status ของ console แทน (ใช้เมื่อ console "
                         "ถือลิงก์อยู่ — สองคนอ่านพอร์ตเดียวกันจะเห็นขยะทั้งคู่)")
    ap.add_argument("--seconds", type=float, default=90.0,
                    help="ระยะเวลาเก็บ (ค่าเริ่มต้น 90) — วางเครื่องนิ่งตลอดช่วงนี้")
    ap.add_argument("--label", default="gps", help="ชื่อของโมดูลตัวนี้ (ใช้กับ --save/--compare)")
    ap.add_argument("--save", action="store_true", help="เก็บผลไว้เทียบกับตัวอื่น")
    ap.add_argument("--compare", nargs="+", metavar="LABEL",
                    help="เทียบผลที่เคย --save ไว้ แล้วจบ (ไม่วัดใหม่)")
    args = ap.parse_args()

    if args.compare:
        return compare(args.compare)

    where = args.from_console or args.endpoint
    print(f"[gps-bench] {args.label}: เก็บ {args.seconds:.0f} วิ จาก {where}")
    print("[gps-bench] วางเครื่องนิ่ง ๆ อย่าขยับ และอย่าบังท้องฟ้า")
    try:
        samples = (collect_console(args.from_console, args.seconds)
                   if args.from_console
                   else collect_mavlink(args.endpoint, args.seconds))
    except Exception as exc:  # noqa: BLE001
        print(f"[gps-bench] ต่อไม่ได้ที่ {where}: {exc}")
        return 3

    q = summarise(samples, args.label)
    report(q)
    code, verdict = classify(q)
    print(f"\n[gps-bench] {'OK' if code == 0 else 'ระวัง' if code == 1 else 'ไม่ผ่าน'}: {verdict}")
    if q.n >= 5 and q.sats_avg >= 12 and code != 0:
        print("[gps-bench] ดาวเทียมเยอะแต่กระจายกว้าง = multipath/ถูกบัง — "
              "ย้ายที่วางแล้ววัดซ้ำก่อนโทษโมดูล")
    if args.save:
        print(f"[gps-bench] เก็บไว้ที่ {save(q)}  (เทียบด้วย --compare {args.label} <อีกตัว>)")
    return code


if __name__ == "__main__":
    sys.exit(main())
