"""AAVC 2026 competition web ground station — built ON the proven Sys_ID GCS.

Runs ON the CM4 (it owns the /dev/serial0 link to the FMU) and serves a browser
dashboard over WiFi, so the drone can be monitored + calibrated + armed from any
phone or laptop on the hotspot -- no QGC, no Qt, no display server needed.

    python3 src/gcs_server.py -c config/real.yaml [--port 8000]

Then open  http://<cm4-ip>:8000  in a browser.

Scope (v1): live telemetry, accelerometer + level-horizon calibration, arm/disarm
test, and emergency LAND / KILL. Buttons are USER-initiated (clicked in the
browser) -- the server only exposes them.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import socket
import struct
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml
from pymavlink import mavutil

# gcs_server (the MONITOR -- not flight_runner) speaks MAVLink 2 + the `common`
# dialect so it can tag a geofence upload with mission_type=FENCE (the default v1
# `ardupilotmega` MISSION messages lack that field). Every telemetry message we
# parse lives in `common`, so this is the standard/safe PX4 choice; flight_runner
# runs in its own process and is untouched.
os.environ.setdefault("MAVLINK20", "1")
try:
    mavutil.set_dialect("common")
except Exception:
    pass


def _safe_add_message(messages, mtype, msg):
    """Drop-in for pymavlink's add_message that survives an instance-field
    message whose first arrival had a None instance value. The stock 2.4.49
    version leaves ``._instances`` as None in that case and then crashes the
    reader thread with 'NoneType object does not support item assignment'
    on the next message (e.g. PX4 BATTERY_STATUS.id)."""
    import copy
    if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
        messages[mtype] = msg
        return
    instance_value = getattr(msg, msg._instance_field)
    prev = getattr(messages[mtype], "_instances", None) if mtype in messages else None
    if prev is None:
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = {instance_value: msg}
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
        return
    prev[instance_value] = msg
    messages[mtype] = copy.copy(msg)
    messages[mtype]._instances = prev
    messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)


mavutil.add_message = _safe_add_message

# ---- PX4 mode tables ------------------------------------------------------
MAIN_MODE = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
             6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL",
            6: "LAND", 7: "RTGS", 8: "FOLLOW", 9: "PRECLAND"}
FIX = {0: "no fix", 1: "no fix", 2: "2D", 3: "3D", 4: "3D-DGPS",
       5: "RTK-float", 6: "RTK-fixed"}
# SYS_STATUS sensor bits (MAV_SYS_STATUS_SENSOR) -- exact enum values
SENS = {"gyro": 0x01, "accel": 0x02, "mag": 0x04, "baro": 0x08,
        "gps": 0x20, "rc": 0x10000, "ahrs": 0x200000, "battery": 0x2000000}

# PX4 COM_FLTMODEn values -> flight-mode names (for the RC mode-switch mapping)
FLTMODE_NAMES = {-1: "(ไม่ได้ตั้ง)", 0: "Manual", 1: "Altitude", 2: "Position",
                 3: "Mission", 4: "Hold", 5: "Return", 6: "Acro", 7: "Offboard",
                 8: "Stabilized", 9: "Rattitude", 10: "Takeoff", 11: "Land",
                 12: "Follow", 13: "Precland", 14: "Orbit"}
# PX4 GF_ACTION -> what the vehicle does when it breaches the geofence
GF_ACTIONS = {0: "ปิด (None)", 1: "เตือน (Warning)", 2: "ลอยค้าง (Hold)",
              3: "บินกลับบ้าน (Return/RTL)", 4: "Terminate", 5: "ลงจอด (Land)"}

# ---- onboard CM4 (runs the autonomous chirp flight via run_pi.sh) ----------
# IP changes with the phone hotspot but the host stays the .41 octet (see the
# project memory); we ping-sweep the laptop's own /24s for an open ssh on .41.
CM4_USER = os.environ.get("CM4_USER", "drone")
CM4_KEY = os.path.expanduser(os.environ.get("CM4_KEY", "~/.ssh/cm4_key"))
CM4_HOST_OCTET = os.environ.get("CM4_OCTET", "41")
CM4_REPO = os.environ.get("CM4_REPO", "~/Sys_ID")
CM4_FIXED = os.environ.get("CM4_HOST")          # optional: skip discovery

# where pulled .ulg logs are saved on the CM4 (browser downloads them from here)
LOG_DIR = os.path.expanduser(os.environ.get("GCS_LOG_DIR", "~/logs"))


def f2i(f):
    return struct.unpack("<i", struct.pack("<f", f))[0]


# ============================ AAVC 2026 additions ============================
# Competition console reads/writes a small FILE contract shared with the touch-and-go
# mission (paths from --captures / --field; see aavc_field.yaml):
#   pad_assignment.json  (GCS -> mission)  {"ids": [...]}          which pads to service
#   mission_status.json  (mission -> GCS)  {phase, pads_mapped:{id:[e,n]}, ...}
# All of this is read-only aggregation layered on top — the proven Link / MAVLink /
# command code below is left untouched (memory: build from Sys_ID, don't rewrite it).
PAD_IDS = [1, 2, 3, 4, 5, 6]                       # ArUco pad IDs (rules: dict 4x4, IDs 1-6)
_HERE = os.path.dirname(os.path.abspath(__file__))
def _default_captures():
    """Auto-share files with the touch-and-go mission if it's a sibling repo, else a local
    captures/ — so live use needs no --captures (the mission's captures is found for you)."""
    for c in (os.path.join(_HERE, "..", "..", "mission_AAVC", "captures"),
              os.path.join(_HERE, "..", "..", "touch_and_go_for_race", "captures"),
              os.path.join(_HERE, "..", "captures")):
        if os.path.isdir(c):
            return os.path.abspath(c)
    return os.path.abspath(os.path.join(_HERE, "..", "captures"))


AAVC_CAPTURES = _default_captures()                        # overridden by --captures
AAVC_FIELD = os.path.join(_HERE, "..", "aavc_field.yaml")  # overridden by --field
# Ground-side "black box": last-known-position trail written on the LAPTOP (survives a
# lost aircraft, unlike the FMU SD-card ulog which needs the drone recovered to read).
BLACKBOX_DIR = os.path.abspath(os.path.join(_HERE, "..", "blackbox"))


# Frame files became JPEG on 2026-08-21 (aircraft-side: 48 -> 12 ms to encode).
# The console only stats them for the camera-age chip, so accept EITHER name and
# prefer whichever is fresher — an aircraft that has not been re-deployed still
# writes the .png, and a chip stuck at "n/a" beside a healthy camera is exactly
# the kind of false negative the operator cannot debug in the field.
_NADIR_CANDIDATES = ("/tmp/aavc_nadir.jpg", "/tmp/aavc_nadir.png")


def _nadir_frame_path():
    newest, best = None, -1.0
    for path in _NADIR_CANDIDATES:
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if mtime > best:
            newest, best = path, mtime
    return newest or _NADIR_CANDIDATES[0]


def _read_vendor(name):
    """Bundled Leaflet (src/vendor/) served locally so the map loads with NO internet at
    the field. Only the OSM tiles still need a connection — the vector overlays (zones,
    pads, drone) render on the blank map regardless."""
    try:
        with open(os.path.join(_HERE, "vendor", name), "rb") as fh:
            return fh.read()
    except Exception:
        return b""


VENDOR_JS = _read_vendor("leaflet.js")
VENDOR_CSS = _read_vendor("leaflet.css")


def _en_to_ll(e, n, lat0, lon0):
    """Local (East, North) metres about (lat0,lon0) -> [lat, lon]. Inverse of the usual
    equirectangular projection; good enough at field scale for plotting detected pads."""
    R = 6378137.0
    lat = lat0 + (n / R) * 180.0 / math.pi
    lon = lon0 + (e / (R * math.cos(math.radians(lat0)))) * 180.0 / math.pi
    return [round(lat, 7), round(lon, 7)]


def load_zones():
    """Rulebook GPS polygons (controlled_airspace + search_area) + home P1 from the field
    yaml. Corners angle-ordered around the centroid so they form a clean quad on the map."""
    try:
        gf = (yaml.safe_load(open(AAVC_FIELD)) or {}).get("geofence", {})
    except Exception:
        return None

    def poly(key):
        pts = [[float(a), float(b)] for a, b in gf.get(key, [])]
        if len(pts) >= 3:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        return pts

    out = {"airspace": poly("controlled_airspace"), "search": poly("search_area")}
    if gf.get("transit_waypoints"):
        out["transit"] = [[float(a), float(b)] for a, b in gf["transit_waypoints"]]
    if gf.get("local_origin"):
        out["home"] = [float(gf["local_origin"][0]), float(gf["local_origin"][1])]
    return out


# Radio "AAVC why=<code>" -> operator text (mirror of the repo's
# gcs_status._HOME_REASONS table — the WiFi path carries the Thai text
# directly in mission_status.home_reason; the radio carries only the code).
WHY_TH = {
    "budget": "งบเวลา/แบตไม่พอก่อนส่ง — กลับพร้อมไข่ที่เหลือ",
    "energy": "พลังงานไม่พอสำหรับเที่ยวถัดไป — สลับแบตก่อนกด GO",
    "gps": "GPS หลุดต่อเนื่อง — ลงจอด ณ จุดที่อยู่",
    "batt-crit": "แบตวิกฤต — ลงจอดทันที ณ จุดที่อยู่",
    "batt-low": "แบตต่ำกว่าเกณฑ์ — กลับบ้าน (RTH)",
    "batt-nan": "อ่านค่าแบตไม่ได้ต่อเนื่อง — กลับบ้าน (RTH)",
    "fence": "หลุดรั้ว geofence — กลับบ้าน (RTH)",
    "nofly": "เข้าเขตห้ามบิน — กลับบ้าน (RTH)",
    "ceiling": "ทะลุเพดานบินค้าง — กลับบ้าน (RTH)",
    "telem": "telemetry ขาดต่อเนื่อง — กลับบ้าน (RTH)",
    "datalink": "ลิงก์สั่งการหลุด — กลับบ้าน (RTH)",
    "time": "หมดงบเวลา — กลับบ้าน (RTH)",
    "pilot": "นักบินยึดเครื่องคืน — ระบบหยุดสั่งแล้ว",
}

# Phase -> the caption under the %-bar, for the RADIO feed. The WiFi feed sends
# gcs_status's own progress_label; STATUSTEXT is ascii-only, so over the radio
# the phase travels as a keyword and the Thai is rebuilt here. Keep the wording
# matched to orchestrator/gcs_status.py::_update_progress so the operator does
# not learn two vocabularies for the same aircraft.
PHASE_TH = {
    "preflight": "เตรียมพร้อม / รอปล่อย",
    "recon": "เตรียมพร้อม / รอปล่อย",
    "recon (preflight)": "เตรียมพร้อม / รอปล่อย",
    "takeoff": "กำลังขึ้นบิน",
    "transit_ingress": "บินเข้าเส้นทาง P1→P3",
    "search": "กวาดหา pad",
    "localize": "เข้าหา pad",
    "track": "เข้าหา pad",
    "drop": "ปล่อยไข่",
    "transit_egress": "บินกลับ",
    "rth": "บินกลับ",
    "land": "กลับมาลงจอด",
    "done": "จบภารกิจ",
}


def read_mission_status():
    """The mission's live status file (phase, detected pads, deliveries, time) or None."""
    try:
        st = json.load(open(os.path.join(AAVC_CAPTURES, "mission_status.json")))
        st["age_s"] = round(time.time() - st.get("updated", 0), 1)
        return st
    except Exception:
        return None


def default_assignment():
    """Pads the mission will service: the GCS pick file > field default > all IDs."""
    try:
        p = os.path.join(AAVC_CAPTURES, "pad_assignment.json")
        if os.path.exists(p):
            return [int(x) for x in json.load(open(p)).get("ids", [])]
    except Exception:
        pass
    try:
        a = (yaml.safe_load(open(AAVC_FIELD)) or {}).get("aruco_assignment")
        if a:
            return [int(x) for x in a]
    except Exception:
        pass
    return list(PAD_IDS)


def selected_pads():
    """Pads the operator EXPLICITLY saved (pad_assignment.json), or [] if none yet.
    Unlike default_assignment() this does NOT fall back to the field/all-IDs default —
    it is the interlock signal for 'has a drop been chosen?' (gates the servo release)."""
    try:
        p = os.path.join(AAVC_CAPTURES, "pad_assignment.json")
        if os.path.exists(p):
            return [int(x) for x in json.load(open(p)).get("ids", [])]
    except Exception:
        pass
    return []


# ── GCS-triggered mission launch (operator request 2026-08-12) ────────────────
# `--mission-cmd` hands the console ONE spawnable command template; {ids} is
# replaced with the operator's SAVED pad selection (selected_pads()). SITL
# points it at the mission repo's sitl/run_mission.sh; the real bird points it
# at ssh into the CM4 (the orchestrator runs THERE, never on the GCS laptop).
# No template configured -> the 🚀 button stays hidden and /api/mission/start
# refuses. While the spawned process is alive the UI locks pad editing.
MISSION_CMD = None
MISSION_LABEL = ""            # "SIM"/"REAL" badge on the 🚀 button (--mission-label)
MISSION_LOG = "/tmp/aavc_mission.log"
RESET_CMD = None              # SIM-only field reset template (--reset-cmd)
RESET_LOG = "/tmp/aavc_reset.log"
_MISSION_LOCK = threading.Lock()
_MISSION_PROC = None
_RESET_PROC = None


# ── in-UI mission switcher (operator request 2026-08-13) ─────────────────────
# missions.yaml (next to the repo root) lists every mission this console can
# command; the dropdown in the Mission card re-points field/captures/🚀 at
# runtime. Entries missing field/mission_cmd show as "not ready" (contract
# pending) and cannot be selected.
MISSIONS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "missions.yaml")
MISSIONS = {}
CURRENT_MISSION = None
# True when this console was wired to a REAL aircraft at startup (its
# --mission-cmd ssh'es into the CM4). The registry holds laptop-local SIM
# commands, so switching templates there would silently replace the real
# ssh GO with a simulator one — refused (operator question 2026-08-14:
# "AAVC GCS สลับไปรัน mission ข้างบ้านได้ไหม" — on the real bird it must not).
REAL_CONSOLE = False


def load_missions():
    global MISSIONS
    try:
        doc = yaml.safe_load(open(MISSIONS_PATH)) or {}
        MISSIONS = doc.get("missions") or {}
    except Exception:
        MISSIONS = {}


def mission_registry_snapshot():
    out = []
    for name, m in MISSIONS.items():
        m = m or {}
        out.append({"name": name, "label": m.get("label") or name,
                    "available": bool(m.get("field")) and bool(m.get("mission_cmd"))})
    return out


def apply_mission(name):
    """Re-point the console at a registry mission; returns an error string or
    None. Refused while a mission/reset is running — the aircraft must never
    be mid-flight under one mission while the UI points at another."""
    global AAVC_FIELD, AAVC_CAPTURES, MISSION_CMD, MISSION_LABEL, RESET_CMD
    global CURRENT_MISSION
    m = MISSIONS.get(name) or {}
    if not m:
        return f"ไม่รู้จัก mission: {name}"
    if REAL_CONSOLE:
        return ("🔒 console นี้ผูกกับเครื่องจริง (ปุ่ม 🚀 ssh ไป CM4) — "
                "สลับ template ไม่ได้ เพราะ template ในรายการเป็นคำสั่งฝั่ง "
                "SIM บนโน้ตบุ๊ก; ถ้าจะบิน mission อื่นบนเครื่องจริง ต้อง "
                "deploy repo นั้นขึ้น CM4 แล้วเปิด console ใหม่ด้วย "
                "--mission-cmd ของ repo นั้น")
    if not m.get("field") or not m.get("mission_cmd"):
        return (f"mission '{name}' ยังไม่พร้อม — repo นั้นยังไม่มี "
                "field yaml / entry สั่งบิน (รอ contract)")
    if mission_running() or reset_running():
        return "🔒 mission กำลังทำงาน — สลับไม่ได้ รอให้จบก่อน"
    AAVC_FIELD = os.path.expanduser(m["field"])
    AAVC_CAPTURES = os.path.expanduser(m["captures"])
    MISSION_CMD = m["mission_cmd"]
    MISSION_LABEL = m.get("mission_label") or ""
    RESET_CMD = m.get("reset_cmd") or None
    CURRENT_MISSION = name
    return None


# ── CM4 reachability (operator request 2026-08-14) ───────────────────────────
# When the 🚀 command is an ssh into the CM4, probe that host's ssh port in the
# background: the button LOCKS while the CM4 is unreachable (better than an
# error after the press), and the sensor board gets a CM4 chip. A local
# mission_cmd (SIM) has no host — probe result None, no gating, chip n/a.
_CM4_OK = None
# REAL console: auto bring-up of the aircraft's camera+beacon infra on connect
# (operator 2026-08-19) so the camera sensor chip lights without staging a flight.
_INFRA_STARTED = False
_INFRA_INFLIGHT = False


def _mission_cmd_ssh_host(cmd):
    if not cmd or "ssh" not in cmd:
        return None
    for tok in str(cmd).replace("'", " ").replace('"', " ").split():
        if "@" in tok and not tok.startswith("-"):
            return tok.split("@", 1)[1]
    return None


def _mission_cmd_ssh_target(cmd):
    """The FULL ``user@host`` the GO command ssh's to — not just the hostname.

    _maybe_start_infra used to pass the bare host from _mission_cmd_ssh_host
    (which exists for the TCP probe), so ssh fell back to the LOCAL username
    and every auto-infra start died with "bonus-linux@10.42.0.1: Permission
    denied (publickey,password)" — seen on the console message feed
    2026-08-21. Harmless while the CM4 stack was already up by hand, which is
    exactly why it went unnoticed."""
    if not cmd or "ssh" not in cmd:
        return None
    for tok in str(cmd).replace("'", " ").replace('"', " ").split():
        if "@" in tok and not tok.startswith("-"):
            return tok
    return None


def _mission_cmd_ssh_identity(cmd):
    """The ``-i <key>`` of the GO command. The CM4 key is NOT the default
    id_rsa, so a BatchMode ssh without it cannot authenticate either."""
    toks = str(cmd or "").replace("'", " ").replace('"', " ").split()
    for i, tok in enumerate(toks):
        if tok == "-i" and i + 1 < len(toks):
            return toks[i + 1]
    return None


def _mission_cmd_remote_dir(cmd):
    """The repo dir the ssh GO runs on the CM4 (e.g. ~/mission) — shown on the
    real-console mission card so the operator can SEE which mission is loaded
    (operator 2026-08-14: restart-based switching is fine 'แต่ต้องบอกด้วย')."""
    if not cmd:
        return None
    for tok in str(cmd).replace("'", " ").replace('"', " ").split():
        if "run_mission.sh" in tok:
            parts = tok.split("/")
            return "/".join(parts[:-2]) if len(parts) >= 3 else tok
    return None


def _maybe_start_infra(host):
    """REAL console only: once the CM4 is reachable over ssh, bring up ONLY the
    aircraft's infra (mavlink-router + camera grabber + status beacon) via the
    fail-safe cm4/start_infra.sh — never the orchestrator, so no arm and no
    flight. This lights the camera sensor chip from the radio beacon WITHOUT
    pressing 🚀 (operator 2026-08-19). Done once; a CM4 that predates the script
    just returns 'no such file' and nothing is staged."""
    global _INFRA_STARTED, _INFRA_INFLIGHT
    if _INFRA_STARTED or _INFRA_INFLIGHT:
        return
    remote_dir = _mission_cmd_remote_dir(MISSION_CMD)
    if not host or not remote_dir:
        return
    _INFRA_INFLIGHT = True

    # Use the SAME credentials the 🚀 GO command uses — user@host and its
    # identity file — not the bare hostname the TCP probe works with.
    target = _mission_cmd_ssh_target(MISSION_CMD) or host
    identity = _mission_cmd_ssh_identity(MISSION_CMD)
    argv = ["ssh", "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=6", "-o", "BatchMode=yes"]
    if identity:
        argv += ["-i", identity]
    argv += [target, f"{remote_dir}/cm4/start_infra.sh"]

    def _run():
        global _INFRA_STARTED, _INFRA_INFLIGHT
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=45)
            _INFRA_STARTED = True   # command ran (infra now up, or a mission already has it)
            if LINK is not None:
                if r.returncode == 0:
                    LINK._note("[infra] CM4 กล้อง+beacon สตาร์ทแล้ว (auto) — chip กล้องจะเขียวใน 2-3 วิ")
                else:
                    tail = ((r.stderr or r.stdout or "").strip().splitlines() or [""])[-1]
                    LINK._note(f"[infra] CM4 start_infra rc={r.returncode}: {tail[:80]}")
        except Exception as e:
            # couldn't reach/run — leave the flag down so the next probe retries
            if LINK is not None:
                LINK._note(f"[infra] ssh CM4 infra ยังไม่ได้ ({type(e).__name__}) — จะลองใหม่")
        finally:
            _INFRA_INFLIGHT = False

    threading.Thread(target=_run, daemon=True, name="infra-start").start()


def _cm4_probe_loop():
    global _CM4_OK
    while True:
        host = _mission_cmd_ssh_host(MISSION_CMD)
        if not host:
            _CM4_OK = None
        else:
            try:
                c = socket.create_connection((host, 22), timeout=2)
                c.close()
                _CM4_OK = True
                _maybe_start_infra(host)     # REAL: auto camera+beacon on connect
            except OSError:
                _CM4_OK = False
        time.sleep(4)


def mission_running():
    """True while the console-spawned mission process is still alive."""
    global _MISSION_PROC
    with _MISSION_LOCK:
        if _MISSION_PROC is not None and _MISSION_PROC.poll() is None:
            return True
        _MISSION_PROC = None
        return False


def reset_running():
    """True while the console-spawned field reset is still working."""
    global _RESET_PROC
    with _MISSION_LOCK:
        if _RESET_PROC is not None and _RESET_PROC.poll() is None:
            return True
        _RESET_PROC = None
        return False


def start_mission(ids):
    """Spawn MISSION_CMD with {ids} filled in. Returns (ok, detail).

    An immediate non-zero exit (the launcher's own prechecks, e.g. the SITL
    dirty-field refusal) is caught here and its last log line returned, so
    the operator sees WHY in the browser instead of a silent no-fly."""
    global _MISSION_PROC
    cmd = MISSION_CMD.replace("{ids}", ",".join(str(i) for i in ids))
    with _MISSION_LOCK:
        if _MISSION_PROC is not None and _MISSION_PROC.poll() is None:
            return False, "mission กำลังบินอยู่แล้ว"
        logf = open(MISSION_LOG, "ab", buffering=0)
        logf.write(f"\n===== GCS mission start ids={ids} =====\n".encode())
        # start_new_session: the flight must survive a console restart/crash.
        _MISSION_PROC = subprocess.Popen(cmd, shell=True, stdout=logf,
                                         stderr=logf, start_new_session=True)
        proc = _MISSION_PROC
    time.sleep(1.5)                       # long enough for a precheck refusal
    if proc.poll() is not None and proc.returncode != 0:
        try:
            with open(MISSION_LOG, "rb") as fh:
                lines = [ln for ln in
                         fh.read()[-500:].decode(errors="replace").splitlines()
                         if ln.strip()]
            detail = lines[-1] if lines else f"launcher exit {proc.returncode}"
        except Exception:
            detail = f"launcher exit {proc.returncode}"
        with _MISSION_LOCK:
            if _MISSION_PROC is proc:
                _MISSION_PROC = None
        return False, detail
    return True, cmd


def start_reset():
    """Spawn the SIM-only field reset (RESET_CMD). Returns (ok, detail)."""
    global _RESET_PROC
    with _MISSION_LOCK:
        if _RESET_PROC is not None and _RESET_PROC.poll() is None:
            return False, "กำลังรีเซ็ตสนามอยู่แล้ว"
        logf = open(RESET_LOG, "wb", buffering=0)
        _RESET_PROC = subprocess.Popen(RESET_CMD, shell=True, stdout=logf,
                                       stderr=logf, start_new_session=True)
    return True, RESET_CMD


def load_payload_servos():
    """Payload-release servo map from the field yaml (payload_servos:), with safe
    placeholder defaults. Each entry -> a DO_SET_ACTUATOR slot (see _servo_send:
    output 9..12 = AUX channel → actuator-set index num-8; 1..6 = the index)."""
    try:
        cfg = (yaml.safe_load(open(AAVC_FIELD)) or {}).get("payload_servos", {}) or {}
    except Exception:
        cfg = {}
    outs = cfg.get("outputs", [1, 2, 3, 4]) or []
    rel = int(cfg.get("released_us", 2000))
    held = int(cfg.get("held_us", 1000))
    # labels: which CORNER of the airframe that pin opens. The loom decides
    # this, not the drop order — an operator staring at the aircraft needs the
    # corner, and "servo 1" alone has caused enough confusion already.
    labels = {str(k): str(v) for k, v in (cfg.get("labels") or {}).items()}
    return [{"num": int(n), "released_us": rel, "held_us": held,
             "label": labels.get(str(n), "")} for n in outs]


class Link:
    """Owns the MAVLink connection: a background reader + thread-safe senders."""

    def __init__(self, url, baud, demo=False):
        self.demo = demo
        self.url = url
        self.baud = baud
        self.send_lock = threading.Lock()
        # Payload-latch keep-alive. ACTUATOR_TEST holds an output only while
        # commands keep arriving (PX4 drops the override at its timeout), so a
        # latch that must STAY open on the bench needs re-sending. The buttons
        # only EDIT this desired-set {idx: value}; ONE supervisor thread (started
        # on the first hold) re-sends the held channels and RELEASE_CONTROLs the
        # rest. Replaces a per-press-thread design that orphaned keep-alive loops
        # under rapid clicks (latch drove itself; only a process kill stopped it).
        self._servo_desired: dict[int, float] = {}
        self._servo_lock = threading.Lock()
        self._servo_sup_started = False
        self.lock = threading.Lock()
        self.s = {
            "link": False, "last_hb": 0.0, "armed": False, "mode": "-",
            "in_air": False,
            # attitude for the artificial horizon + heading (deg)
            "att": {"roll": None, "pitch": None, "yaw": None, "heading": None},
            # per-motor output (normalised 0..1) for the motor-speed bars
            "motors": [],
            "gps": {"fix": 0, "fix_str": "no fix", "sats": 0, "sats_max": 0,
                    "hdop": None, "lat": None, "lon": None,
                    "alt": None, "rel_alt": None},
            # NED velocity (m/s) from GLOBAL_POSITION_INT (for sim replay)
            "vel": {"vx": None, "vy": None, "vz": None},
            # AAVC: EKF/GPS origin (lat/lon of local 0,0) + drone local NED, to convert
            # the mission's local-frame detected pads to lat/lon for the OSM map.
            "origin": {"lat": None, "lon": None},
            "local": {"n": None, "e": None, "z": None},
            # GPS hardware diagnostics: GPS_1_CONFIG port + any boot detect line
            "gps_detect": {"config": None, "config_str": None,
                           "last_msg": None, "busy": False},
            # RC switch channel maps (verify params didn't get reset)
            "rc_maps": {"arm": None, "kill": None, "fltmode": None,
                        "busy": False},
            # COM_DISARM_PRFLT — ground auto-disarm timeout (s); extend it so the
            # pilot has time to arm + press chirp before PX4 auto-disarms.
            "disarm_prflt": None,
            # RC mode-switch mapping (RC_MAP_FLTMODE + COM_FLTMODE1..6)
            "fltmodes": {"map": None, "slots": {}, "busy": False},
            # circular geofence around home: GF_MAX_HOR_DIST / VER_DIST / ACTION
            "geofence": {"hor": None, "ver": None, "action": None, "busy": False},
            # polygon (rectangle) fence uploaded via the mission protocol
            "fence": {"pts": [], "action": None, "count": None, "busy": False},
            # AAVC payload-release servos: {output: {num, released, pwm}} (DO_SET_SERVO)
            "servos": {},
            "batt": {"volt": None, "pct": None},
            "rc": {"rssi": 0, "throttle": None, "roll": None, "pitch": None,
                   "yaw": None, "count": 0},
            "health": {}, "present": {},
            "accel_off": {"x": None, "y": None, "z": None},
            "cal_active": False,
            "cal": {"active": False, "type": None, "instr": "",
                    "sides": {}, "done": False, "failed": False},
            "cm4": {"online": False, "ip": None, "flying": False},
            "logpull": {"busy": False, "msg": ""},
            # ground-side black box: last position we recorded to the laptop CSV
            "blackbox": {"file": None, "lat": None, "lon": None, "alt": None,
                         "t": None, "rows": 0},
            "messages": [],
        }
        self._stop = False
        # log-pull pause gate: clearing _resume asks the reader/heartbeat/poll
        # threads to stand down so MAVFTP can own the link as sole consumer.
        self._resume = threading.Event(); self._resume.set()
        self._reader_paused = threading.Event()
        if demo:
            self._fill_demo()
        else:
            self.m = mavutil.mavlink_connection(url, baud=baud)

    def _fill_demo(self):
        """Populate realistic sample telemetry so the dashboard can be
        previewed on a laptop with no FMU attached."""
        now = time.strftime("%H:%M:%S")
        self.s.update({"link": True, "last_hb": time.time(), "armed": False,
                       "mode": "STABILIZED", "in_air": False})
        self.s["att"] = {"roll": -6.0, "pitch": 3.5, "yaw": 42.0, "heading": 42.0}
        self.s["motors"] = [0.34, 0.36, 0.33, 0.35, 0.34, 0.36]
        self.s["gps"] = {"fix": 3, "fix_str": "3D", "sats": 11, "sats_max": 13,
                         "hdop": 0.93, "lat": 13.82133, "lon": 100.51311}
        self.s["gps_detect"] = {"config": 201, "config_str": "GPS 1 port (201)",
                                "last_msg": "GPS 1: u-blox ZED-F9P (demo)",
                                "busy": False}
        self.s["rc_maps"] = {"arm": 7, "kill": 8, "fltmode": 5, "busy": False}
        self.s["batt"] = {"volt": 22.1, "pct": 87}
        self.s["rc"] = {"rssi": 0, "throttle": 1494, "roll": 1494,
                        "pitch": 1494, "yaw": 1494, "count": 18}
        self.s["present"] = {k: True for k in SENS}
        self.s["health"] = {k: True for k in SENS}
        self.s["health"]["accel"] = False        # high bias -> not arm-ready
        self.s["health"]["ahrs"] = False
        self.s["accel_off"] = {"x": -0.4566, "y": 0.1314, "z": -0.1311}
        self.s["cm4"] = {"online": True, "ip": "192.168.14.41", "flying": False}
        self.s["messages"] = [
            {"t": now, "txt": "[demo] preview mode — ไม่ได้ต่อ FMU จริง"},
            {"t": now, "txt": "Preflight Fail: High Accelerometer Bias"},
        ]

    def _dmsg(self, action):
        with self.lock:
            self.s["messages"].append(
                {"t": time.strftime("%H:%M:%S"),
                 "txt": f"[demo] กดปุ่ม '{action}' (no-op ในโหมดพรีวิว)"})
            del self.s["messages"][:-40]

    # ---- background reader + heartbeat -----------------------------------
    def start(self):
        if self.demo:
            return
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        threading.Thread(target=self._poll_calparams, daemon=True).start()
        threading.Thread(target=self._blackbox_writer, daemon=True).start()

    def _heartbeat(self):
        # A GCS heartbeat is required for the FMU to allow arming.
        while not self._stop:
            if self._resume.is_set():            # paused during a log pull
                try:
                    with self.send_lock:
                        self.m.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                except Exception:
                    pass
            time.sleep(1.0)

    def _request_streams(self):
        # Keep the total message rate LOW. Two bottlenecks seen:
        #  - Pi PL011 UART @921600 overruns its RX FIFO if flooded (CM4 case).
        #  - The NOMAD/ELRS *radio* air-link is far narrower than the 460800 USB
        #    side: ~14 msg/s + an 8-command burst every 10 s saturated it and the
        #    link flapped on a ~10 s cycle ("returned no data" -> reconnect). So:
        #    rates lowered (ATTITUDE 5->2 Hz, ACTUATOR/RC 2->1 Hz, EXT_SYS ->0.5 Hz),
        #    sends spaced out, and the periodic re-request stretched to 60 s (below).
        # ids: 0=HEARTBEAT 1=SYS_STATUS 24=GPS_RAW 33=GLOBAL_POS 65=RC 245=EXT_SYS
        #      30=ATTITUDE (horizon/heading) 375=ACTUATOR_OUTPUT_STATUS (motor bars)
        #      132=DISTANCE_SENSOR 1 Hz (lidar chip): PX4's default stream is
        #      0.5 Hz — only 0.9 s under the chip's staleness limit, so any
        #      delivery hiccup (RTF dip, second GCS on the port) flapped the
        #      chip. This RAISES an existing stream, net +0.5 msg/s.
        for mid, us in ((0, 1000000), (1, 1000000), (24, 1000000),
                        (33, 1000000), (65, 1000000), (245, 2000000),
                        (30, 500000), (375, 1000000), (132, 1000000)):
            try:
                with self.send_lock:
                    self.m.mav.command_long_send(
                        1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0, mid, us, 0, 0, 0, 0, 0)
                time.sleep(0.05)   # space the 9 sends: a burst congests the narrow
                                   # ELRS uplink and can stall the whole link
            except Exception:
                pass

    def _reconnect(self):
        """Reopen the MAVLink link after a serial drop (USB re-enumerate,
        FMU power-cycle). Keeps trying until it succeeds or we're stopping."""
        try:
            self.m.close()
        except Exception:
            pass
        while not self._stop:
            try:
                self.m = mavutil.mavlink_connection(self.url, baud=self.baud)
                # clear any overrun garbage the PL011 left in the buffers so the
                # fresh link doesn't immediately choke on stale partial frames
                try:
                    self.m.port.reset_input_buffer()
                    self.m.port.reset_output_buffer()
                except Exception:
                    pass
                self._note(f"reconnecting to {self.url} …")
                return True
            except Exception:
                time.sleep(1.0)
        return False

    def _reader(self):
        last_req = 0.0
        while not self._stop:
            try:
                if self.m.wait_heartbeat(timeout=10) is None:
                    continue          # no FMU yet — keep waiting, link stays down
                self.m.target_system, self.m.target_component = 1, 1
                self._request_streams()
                last_req = time.time()
                serial_errs = 0
                while not self._stop:
                    if not self._resume.is_set():     # log-pull wants the link
                        self._reader_paused.set()
                        self._resume.wait()
                        self._reader_paused.clear()
                        continue
                    try:
                        msg = self.m.recv_match(blocking=True, timeout=1)
                    except Exception:
                        # CP210x/ELRS radio links spuriously raise "returned no data"
                        # between telemetry bursts. A full reopen (outer handler) costs
                        # multi-second dropouts + a fresh wait_heartbeat handshake — that
                        # teardown was MOST of the observed flap. So tolerate transient
                        # read errors: retry (link stays up while heartbeats are <3 s old)
                        # and only escalate to a real reconnect if they persist (~3 s).
                        serial_errs += 1
                        if serial_errs <= 60:
                            with self.lock:
                                self.s["link"] = (time.time() - self.s["last_hb"]) < 3
                            time.sleep(0.05)
                            continue
                        raise
                    serial_errs = 0
                    now = time.time()
                    if now - last_req > 60:    # re-assert streams periodically
                        self._request_streams()   # (stretched 10->60 s: the 8-cmd
                        last_req = now            #  burst was flapping the ELRS link)
                    if not msg:
                        with self.lock:
                            self.s["link"] = (now - self.s["last_hb"]) < 3
                        continue
                    self._handle(msg, now)
            except Exception as e:               # serial drop / parse error
                with self.lock:
                    self.s["link"] = False
                self._note(f"link error: {e}")
                self._reconnect()

    def _note(self, txt):
        with self.lock:
            self.s["messages"].append(
                {"t": time.strftime("%H:%M:%S"), "txt": txt})
            del self.s["messages"][:-40]

    def _handle(self, msg, now):
        t = msg.get_type()
        with self.lock:
            if t == "HEARTBEAT":
                self.s["last_hb"] = now
                self.s["link"] = True
                self.s["armed"] = bool(msg.base_mode & 0x80)
                cm = msg.custom_mode
                main = (cm >> 16) & 0xFF
                sub = (cm >> 24) & 0xFF
                name = MAIN_MODE.get(main, str(main))
                if main == 4 and sub in AUTO_SUB:
                    name = "AUTO." + AUTO_SUB[sub]
                self.s["mode"] = name
            elif t == "DISTANCE_SENSOR":
                # downward lidar (TFmini-S / SITL lidar): freshness drives the
                # Lidar chip in the sensor board (operator request 2026-08-14)
                self.s["lidar"] = {"t": time.time(),
                                   "m": round(msg.current_distance / 100.0, 2)}
            elif t == "SYS_STATUS":
                pres, hl = (msg.onboard_control_sensors_present,
                            msg.onboard_control_sensors_health)
                self.s["present"] = {k: bool(pres & b) for k, b in SENS.items()}
                self.s["health"] = {k: bool(hl & b) for k, b in SENS.items()}
                if msg.voltage_battery not in (0, 65535):
                    self.s["batt"]["volt"] = round(msg.voltage_battery / 1000, 2)
                if msg.battery_remaining != -1:
                    self.s["batt"]["pct"] = msg.battery_remaining
            elif t == "GPS_RAW_INT":
                g = self.s["gps"]
                g["fix"] = msg.fix_type
                g["fix_str"] = FIX.get(msg.fix_type, str(msg.fix_type))
                g["sats"] = msg.satellites_visible
                # high-water mark: did the antenna EVER track a satellite?
                # (0=stuck → antenna/RF dead; flickers up → weak signal)
                if msg.satellites_visible and msg.satellites_visible != 255:
                    g["sats_max"] = max(g.get("sats_max", 0),
                                        msg.satellites_visible)
                g["hdop"] = (round(msg.eph / 100, 2)
                             if msg.eph not in (0, 65535) else None)
                # lat/lon from GPS_RAW_INT too — over ELRS/low-bw links GLOBAL_
                # POSITION_INT is often not streamed, but GPS_RAW_INT is.
                if msg.lat or msg.lon:
                    g["lat"] = round(msg.lat / 1e7, 6)
                    g["lon"] = round(msg.lon / 1e7, 6)
                    # RAW GPS altitude goes to its OWN key (2026-08-21): it
                    # wanders metres while the fused (baro-ref) altitude is
                    # steady, and both writing g["alt"] made the status-bar
                    # MSL readout bounce — the pilot's in-flight sanity
                    # number must be the FUSED estimate only.
                    g["alt_raw"] = round(msg.alt / 1000, 1)   # raw GPS AMSL (m)
            elif t == "GLOBAL_POSITION_INT":
                gg = self.s["gps"]
                if msg.lat or msg.lon:
                    gg["lat"] = round(msg.lat / 1e7, 6)
                    gg["lon"] = round(msg.lon / 1e7, 6)
                gg["alt"] = round(msg.alt / 1000, 1)               # AMSL (m)
                gg["rel_alt"] = round(msg.relative_alt / 1000, 2)  # above home (m)
                vv = self.s["vel"]
                vv["vx"] = round(msg.vx / 100, 2)   # m/s north
                vv["vy"] = round(msg.vy / 100, 2)   # m/s east
                vv["vz"] = round(msg.vz / 100, 2)   # m/s down
            elif t == "GPS_GLOBAL_ORIGIN":          # AAVC: EKF origin for pad geo-referencing
                self.s["origin"]["lat"] = round(msg.latitude / 1e7, 7)
                self.s["origin"]["lon"] = round(msg.longitude / 1e7, 7)
            elif t == "LOCAL_POSITION_NED":         # AAVC: drone local NED (origin fallback)
                self.s["local"]["n"] = round(msg.x, 2)
                self.s["local"]["e"] = round(msg.y, 2)
                self.s["local"]["z"] = round(msg.z, 2)   # NED down (alt above origin = -z)
            elif t == "ATTITUDE":
                a = self.s["att"]
                a["roll"] = round(msg.roll * 57.29578, 1)
                a["pitch"] = round(msg.pitch * 57.29578, 1)
                yaw = msg.yaw * 57.29578
                a["yaw"] = round(yaw, 1)
                a["heading"] = round((yaw + 360) % 360, 1)
            elif t == "ACTUATOR_OUTPUT_STATUS":
                # normalise to 0..1: real HW gives PWM (~1000-2000), SITL gives 0..1
                n = min((getattr(msg, "active", 0) or 8), 8)
                out = []
                for v in list(msg.actuator)[:n]:
                    x = (v - 1000) / 1000.0 if v > 100 else v
                    out.append(round(max(0.0, min(1.0, x)), 3))
                self.s["motors"] = out
            elif t == "RC_CHANNELS":
                rc = self.s["rc"]
                rc["rssi"] = msg.rssi
                rc["count"] = msg.chancount
                rc["roll"] = msg.chan1_raw
                rc["pitch"] = msg.chan2_raw
                rc["throttle"] = msg.chan3_raw
                rc["yaw"] = msg.chan4_raw
            elif t == "EXTENDED_SYS_STATE":
                # landed_state: 1=ON_GROUND 2=IN_AIR 3=TAKEOFF 4=LANDING (0=unknown)
                ls = msg.landed_state
                if ls == 1:
                    self.s["in_air"] = False
                elif ls in (2, 3, 4):
                    self.s["in_air"] = True
            elif t == "STATUSTEXT":
                txt = msg.text.strip()
                if txt.startswith("AAVC "):
                    self._parse_beacon(txt)
                else:
                    buf = self.s["messages"]
                    buf.append({"t": time.strftime("%H:%M:%S"), "txt": txt})
                    del buf[:-40]
                    self._parse_cal(txt)
                    low = txt.lower()
                    # capture GPS driver boot lines ("GPS 1: u-blox …", "GPS: …")
                    if "gps" in low:
                        self.s["gps_detect"]["last_msg"] = txt
                    # PX4's OWN GPS prearm gate (e.g. "Preflight: GPS PDOP too
                    # high"): track it so the GCS GPS status MATCHES PX4's
                    # arm-readiness instead of showing 3D-fix green while PX4
                    # refuses to arm on accuracy. Stamped; JS treats it as active
                    # only while fresh (PX4 sends no explicit "GPS now OK").
                    if "pdop" in low or ("preflight" in low and "gps" in low):
                        self.s["gps_prearm"] = {"ok": False, "txt": txt,
                                                "t": time.time()}

    def _parse_beacon(self, txt):
        """cm4/status_beacon.py summaries over the RADIO (STATUSTEXT, sysid 1 /
        comp 191) — the mission + camera health that otherwise only ride WiFi:

            AAVC cam=OK 0.9s | AAVC cam=DEAD 7s stale | AAVC cam=NONE no frame file
            AAVC p=<phase> d=<got>/<want> m=<seen> ok=1,3|-  |  AAVC p=idle (…)

        Parsed into radio_* state (with the receive time) rather than appended
        to the message log: at 2 lines / 5 s the raw ticks would drown the
        40-line log. Camera state TRANSITIONS still get logged so a mid-flight
        death leaves a visible trace."""
        now = time.time()
        if txt.startswith("AAVC pads"):
            # pad coordinates (ENU m about the field origin), chunked lines:
            #   "AAVC pads 1:12.3,-8.1 3:5.0,14.2"
            pads = self.s.setdefault("radio_pads", {})
            for ent in txt[len("AAVC pads"):].split():
                try:
                    pid, en = ent.split(":", 1)
                    e, n = en.split(",", 1)
                    pads[pid] = {"t": now, "en": [float(e), float(n)]}
                except ValueError:
                    continue
            return
        m = re.match(r"AAVC stale=(\d+)", txt)
        if m:
            # The beacon re-reads and re-sends the same status file every 5 s
            # forever, so "when this line arrived" says nothing about how old
            # the mission data inside it is. This line carries the real age and
            # only appears once the data stops being current; without it a
            # finished flight kept arriving fresh and a console opened later
            # showed it at 100 % as though it were live (2026-08-18).
            self.s["radio_stale"] = {"t": now, "age": int(m.group(1))}
            return
        m = re.match(r"AAVC prg=(\d+) eta=(\d+) tp=([01]{3}) cur=(\d+|-)", txt)
        if m:
            # progress %, ETA, transit points passed, pad being served —
            # everything the awareness pack needs that plain phase+counts
            # cannot give. `progress` in particular is the switch the %-bar and
            # the milestone strip test before drawing anything at all.
            self.s["radio_prog"] = {
                "t": now, "pct": int(m.group(1)), "eta": int(m.group(2)),
                "tp": m.group(3), "cur": (None if m.group(4) == "-"
                                          else int(m.group(4)))}
            return
        m = re.match(r"AAVC why=([\w-]+)", txt)
        if m:
            # homecoming reason code — beacon repeats it every tick while the
            # reason stands; gcs_status clears it at the next FLIGHT START
            self.s["radio_why"] = {"t": now, "code": m.group(1)}
            return
        m = re.match(r"AAVC seen=([\d,]+)$", txt)
        if m:
            # identified-but-unconfirmed marker ids (2026-08-21): decoded at
            # least once, confirm votes still short. This line is what lights
            # the ORANGE pad state when WiFi is dead — G7 flight 1 was pulled
            # down while ids 4,5 were being identified behind a blank console.
            # The beacon repeats it while the tracker holds them; a confirmed
            # pad moves to the "AAVC pads" coordinate line instead.
            ids = [int(x) for x in m.group(1).split(",") if x.isdigit()]
            self.s["radio_seen"] = {"t": now, "ids": ids}
            return
        m = re.match(r"AAVC cam=(\w+)(?:\s+([\d.]+)s)?", txt)
        if m:
            prev = (self.s.get("radio_cam") or {}).get("state")
            self.s["radio_cam"] = {"t": now, "state": m.group(1),
                                   "age": float(m.group(2)) if m.group(2) else None}
            if prev is not None and prev != m.group(1):
                buf = self.s["messages"]
                buf.append({"t": time.strftime("%H:%M:%S"), "txt": txt})
                del buf[:-40]
            return
        m = re.match(r"AAVC p=(.+?) d=(\d+)/(\d+) m=(\d+) ok=(\S+)$", txt)
        if m:
            ok = [int(x) for x in m.group(5).split(",") if x.isdigit()]
            self.s["radio_mission"] = {
                "t": now, "phase": m.group(1), "delivered_n": int(m.group(2)),
                "assigned_n": int(m.group(3)), "mapped_n": int(m.group(4)),
                "ok": ok}
            return
        m = re.match(r"AAVC p=(.+)$", txt)     # "p=idle (no mission yet)"
        if m:
            self.s["radio_mission"] = {"t": now, "phase": m.group(1),
                                       "delivered_n": 0, "assigned_n": 0,
                                       "mapped_n": 0, "ok": []}

    ACC_SIDES = ["down", "up", "left", "right", "front", "back"]

    def _parse_cal(self, txt):
        """Turn PX4 '[cal] ...' statustext into structured calibration state
        (which of the 6 sides are pending / measuring / done) -- QGC-style."""
        low = txt.lower()
        if not any(w in low for w in ("[cal]", "calibration", "orientation", " side")):
            return
        c = self.s["cal"]
        if "calibration started" in low:
            c["type"] = "level" if "level" in low else "accel"
            keys = ["level"] if c["type"] == "level" else self.ACC_SIDES
            c.update({"active": True, "done": False, "failed": False,
                      "sides": {k: "pending" for k in keys},
                      "instr": "กำลังเริ่ม…"})
        elif "pending:" in low:
            names = low.split("pending:")[1].split()
            for s in c["sides"]:
                c["sides"][s] = "pending" if s in names else c["sides"].get(s, "done")
            c["instr"] = "วาง Drone ในด้านที่ยังเป็นสีเทา ค้างนิ่ง ๆ"
        elif "orientation detected" in low or ("measuring" in low and "side" in low):
            for s in c["sides"]:
                if s in low:
                    c["sides"][s] = "measuring"
                    c["instr"] = f"จับด้าน '{s}' ได้ — ถือนิ่ง ๆ กำลังวัด…"
        elif "side done" in low or "side result" in low:
            for s in c["sides"]:
                if s in low:
                    c["sides"][s] = "done"
            c["instr"] = "ดีมาก! หมุนไปด้านถัดไป (สีเทา)"
        elif "calibration done" in low:
            for s in c["sides"]:
                c["sides"][s] = "done"
            c.update({"done": True, "active": False, "instr": "เสร็จสมบูรณ์ ✓"})
        elif "calibration failed" in low or ("fail" in low and "cal" in low):
            c.update({"failed": True, "active": False, "instr": "ล้มเหลว — ลองใหม่"})

    def _poll_calparams(self):
        time.sleep(4)
        while not self._stop:
            if self._resume.is_set():            # paused during a log pull
                self.fetch_accel_offsets()
            time.sleep(6)

    def fetch_accel_offsets(self):
        names = {"CAL_ACC0_XOFF": "x", "CAL_ACC0_YOFF": "y", "CAL_ACC0_ZOFF": "z"}
        for n, key in names.items():
            if not self._resume.is_set():        # log pull grabbed the link
                return
            try:
                with self.send_lock:
                    self.m.mav.param_request_read_send(1, 1, n.encode(), -1)
                t = time.time()
                while time.time() - t < 1.5:
                    if not self._resume.is_set():
                        return
                    pm = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                           timeout=1)
                    if pm and pm.param_id.strip("\x00") == n:
                        with self.lock:
                            self.s["accel_off"][key] = round(pm.param_value, 4)
                        break
            except Exception:
                pass

    # ---- GPS hardware diagnostics ---------------------------------------
    # PX4 serial-port codes that GPS_x_CONFIG can be assigned to.
    SER_PORTS = {0: "ปิด (Disabled)", 101: "TELEM 1", 102: "TELEM 2",
                 201: "GPS 1 port", 202: "GPS 2 port", 203: "GPS 3/4 port",
                 300: "Radio Control", 301: "Wifi"}

    def check_gps_async(self):
        """On-demand read of GPS_1_CONFIG so the field operator can confirm the
        GPS serial port is enabled, without firing up QGC. Runs off the HTTP
        thread; reads PARAM_VALUE the same way fetch_accel_offsets does."""
        if self.demo:
            return
        with self.lock:
            if self.s["gps_detect"]["busy"]:
                return
            self.s["gps_detect"]["busy"] = True
        threading.Thread(target=self._check_gps_worker, daemon=True).start()

    def _check_gps_worker(self):
        try:
            val = self._read_param_int("GPS_1_CONFIG")
            with self.lock:
                gd = self.s["gps_detect"]
                if val is None:
                    gd["config_str"] = "อ่านค่าไม่ได้ (ลองใหม่)"
                else:
                    gd["config"] = val
                    gd["config_str"] = f"{self.SER_PORTS.get(val, 'port code')} ({val})"
            self._note(f"[gps] GPS_1_CONFIG = {val}"
                       + ("  ⚠️ GPS ถูกปิดอยู่!" if val == 0 else ""))
            # GPS_1_GNSS: which constellations are enabled (0 = auto/all).
            # If someone disabled GPS+GLONASS+Galileo+BeiDou the receiver tracks 0.
            gnss = self._read_param_int("GPS_1_GNSS")
            if gnss is not None:
                names = [(1, "GPS"), (2, "SBAS"), (4, "Galileo"),
                         (8, "BeiDou"), (16, "GLONASS"), (32, "QZSS")]
                on = [nm for bit, nm in names if gnss & bit]
                gstr = "auto (ทั้งหมด)" if gnss == 0 else ",".join(on) or "ไม่มีเลย!"
                with self.lock:
                    self.s["gps_detect"]["gnss"] = gnss
                    self.s["gps_detect"]["gnss_str"] = gstr
                self._note(f"[gps] GPS_1_GNSS = {gnss} → {gstr}"
                           + ("  ⚠️ ไม่ได้เปิดดาวเทียมเลย!"
                              if gnss != 0 and not on else ""))
        finally:
            with self.lock:
                self.s["gps_detect"]["busy"] = False

    # ---- RC switch-map diagnostics --------------------------------------
    def check_rcmaps_async(self):
        """Read RC_MAP_ARM_SW / KILL_SW / FLTMODE so the operator can confirm
        the arm/kill/mode switches are still mapped (a param reset would zero
        them → can't arm via RC → chirp button stays locked)."""
        if self.demo:
            return
        with self.lock:
            if self.s["rc_maps"]["busy"]:
                return
            self.s["rc_maps"]["busy"] = True
        threading.Thread(target=self._check_rcmaps_worker, daemon=True).start()

    def _check_rcmaps_worker(self):
        try:
            for key, name, exp in (("arm", "RC_MAP_ARM_SW", 7),
                                   ("kill", "RC_MAP_KILL_SW", 8),
                                   ("fltmode", "RC_MAP_FLTMODE", 5)):
                v = self._read_param_int(name)
                with self.lock:
                    self.s["rc_maps"][key] = v
                self._note(f"[rc] {name} = {v}"
                           + ("  ⚠️ ไม่ได้ map! (arm/kill ไม่ทำงาน)" if v == 0
                              else f"  (คาด ch{exp})" if v != exp else ""))
        finally:
            with self.lock:
                self.s["rc_maps"]["busy"] = False

    # ---- RC flight-mode switch mapping (COM_FLTMODE1..6) -----------------
    def read_fltmodes_async(self):
        """Read RC_MAP_FLTMODE + COM_FLTMODE1..6 so we can show the operator
        exactly what each mode-switch position is set to before changing it."""
        if self.demo:
            return
        with self.lock:
            if self.s["fltmodes"]["busy"]:
                return
            self.s["fltmodes"]["busy"] = True
        threading.Thread(target=self._read_fltmodes_worker, daemon=True).start()

    def _read_fltmodes_worker(self):
        try:
            mp = self._read_param_int("RC_MAP_FLTMODE")
            with self.lock:
                self.s["fltmodes"]["map"] = mp
            self._note(f"[mode] RC_MAP_FLTMODE = ch{mp}")
            for n in range(1, 7):
                v = self._read_param_int(f"COM_FLTMODE{n}")
                with self.lock:
                    self.s["fltmodes"]["slots"][str(n)] = v
                if v is not None:
                    self._note(f"[mode] COM_FLTMODE{n} = {v} "
                               f"({FLTMODE_NAMES.get(v, v)})")
        finally:
            with self.lock:
                self.s["fltmodes"]["busy"] = False

    def set_fltmode_async(self, slot, value):
        """Set COM_FLTMODE{slot} (INT) = value (a PX4 flight-mode number)."""
        if self.demo:
            return
        threading.Thread(target=self._set_fltmode_worker,
                         args=(int(slot), int(value)), daemon=True).start()

    def _set_fltmode_worker(self, slot, value):
        name = f"COM_FLTMODE{slot}"
        fval = struct.unpack("<f", struct.pack("<i", value))[0]
        rb, ok = None, False
        for _ in range(8):
            try:
                with self.send_lock:
                    self.m.mav.param_set_send(
                        1, 1, name.encode(), fval,
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            except Exception:
                pass
            time.sleep(0.8)
            rb = self._read_param_int(name)
            if rb == value:
                ok = True
                break
            time.sleep(0.5)
        with self.lock:
            if rb is not None:
                self.s["fltmodes"]["slots"][str(slot)] = rb
        if ok:
            self._note(f"[mode] ✅ ตั้ง {name} = {value} "
                       f"({FLTMODE_NAMES.get(value, value)}) แล้ว")
        else:
            self._note(f"[mode] ⚠️ ตั้ง {name} ไม่สำเร็จ (ลิงก์ไม่นิ่ง) ลองใหม่")

    # ---- circular geofence (GF_MAX_HOR_DIST / VER_DIST / GF_ACTION) ------
    def read_geofence_async(self):
        if self.demo:
            return
        with self.lock:
            if self.s["geofence"]["busy"]:
                return
            self.s["geofence"]["busy"] = True
        threading.Thread(target=self._read_geofence_worker, daemon=True).start()

    def _read_geofence_worker(self):
        try:
            hor = self._read_param_float("GF_MAX_HOR_DIST")
            ver = self._read_param_float("GF_MAX_VER_DIST")
            act = self._read_param_int("GF_ACTION")
            with self.lock:
                g = self.s["geofence"]
                g["hor"], g["ver"], g["action"] = hor, ver, act
            self._note(f"[fence] รัศมี={hor} m · เพดาน={ver} m · "
                       f"เกิน→{GF_ACTIONS.get(act, act)}")
        finally:
            with self.lock:
                self.s["geofence"]["busy"] = False

    def set_geofence_async(self, radius, alt, action):
        if self.demo:
            return
        threading.Thread(target=self._set_geofence_worker,
                         args=(float(radius), float(alt), int(action)),
                         daemon=True).start()

    def _set_geofence_worker(self, radius, alt, action):
        def set_float(name, val):
            rb = None
            for _ in range(6):
                try:
                    with self.send_lock:
                        self.m.mav.param_set_send(
                            1, 1, name.encode(), float(val),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                except Exception:
                    pass
                time.sleep(0.7)
                rb = self._read_param_float(name)
                if rb is not None and abs(rb - val) < 0.5:
                    return rb
                time.sleep(0.4)
            return rb

        def set_int(name, val):
            fval = struct.unpack("<f", struct.pack("<i", val))[0]
            rb = None
            for _ in range(6):
                try:
                    with self.send_lock:
                        self.m.mav.param_set_send(
                            1, 1, name.encode(), fval,
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                except Exception:
                    pass
                time.sleep(0.7)
                rb = self._read_param_int(name)
                if rb == val:
                    return rb
                time.sleep(0.4)
            return rb

        h = set_float("GF_MAX_HOR_DIST", radius)
        v = set_float("GF_MAX_VER_DIST", alt)
        a = set_int("GF_ACTION", action)
        with self.lock:
            g = self.s["geofence"]
            g["hor"], g["ver"], g["action"] = h, v, a
        self._note(f"[fence] ✅ ตั้ง geofence: รัศมี {h} m · เพดาน {v} m · "
                   f"เกิน→{GF_ACTIONS.get(a, a)}")

    # ---- AAVC: set EKF local origin = current position (local NED 0,0 = here) ----
    def set_local_origin_async(self):
        """Send SET_GPS_GLOBAL_ORIGIN at the drone's current global position so
        local NED (0,0) = 'here' (anchors the AAVC pad/mission coordinate frame).
        ADD-only: new sender, does not touch the existing Link/command paths."""
        if self.demo:
            return
        threading.Thread(target=self._set_local_origin_worker, daemon=True).start()

    def _set_local_origin_worker(self):
        with self.lock:
            g = dict(self.s.get("gps") or {})
        lat, lon, alt = g.get("lat"), g.get("lon"), g.get("alt")
        if lat is None or lon is None:
            self._note("[origin] ❌ ยังไม่มีตำแหน่ง GPS — ตั้ง origin (0,0) ที่นี่ไม่ได้")
            return
        lat_e7, lon_e7 = int(round(lat * 1e7)), int(round(lon * 1e7))
        alt_mm = int(round((alt if alt is not None else 0.0) * 1000))   # AMSL mm
        for _ in range(3):
            try:
                with self.send_lock:
                    try:
                        self.m.mav.set_gps_global_origin_send(
                            1, lat_e7, lon_e7, alt_mm, int(time.time() * 1e6))
                    except TypeError:                       # older pymavlink: no time_usec field
                        self.m.mav.set_gps_global_origin_send(1, lat_e7, lon_e7, alt_mm)
            except Exception:
                pass
            time.sleep(0.4)
        self._note(f"[origin] ✅ ตั้ง origin (0,0) = ที่นี่ ({lat:.6f}, {lon:.6f})")

    # ---- polygon (rectangle) geofence via the mission protocol ----------
    def upload_fence_async(self, pts, action, alt=None):
        if self.demo or not pts:
            return
        with self.lock:
            if self.s["fence"]["busy"]:
                raise RuntimeError("กำลังทำงานกับ fence อยู่ รอแป๊บ")
            self.s["fence"]["busy"] = True
        threading.Thread(
            target=self._upload_fence_worker,
            args=([[float(a), float(b)] for a, b in pts], int(action),
                  None if alt is None else float(alt)),
            daemon=True).start()

    def _upload_fence_worker(self, pts, action, alt=None):
        try:
            fv = struct.unpack("<f", struct.pack("<i", action))[0]   # GF_ACTION (INT)
            for _ in range(4):
                try:
                    with self.send_lock:
                        self.m.mav.param_set_send(1, 1, b"GF_ACTION", fv,
                                                  mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                except Exception:
                    pass
                time.sleep(0.5)
                if self._read_param_int("GF_ACTION") == action:
                    break
            if alt is not None:              # GF_MAX_VER_DIST = ceiling (FLOAT, m)
                for _ in range(4):
                    try:
                        with self.send_lock:
                            self.m.mav.param_set_send(
                                1, 1, b"GF_MAX_VER_DIST", float(alt),
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    except Exception:
                        pass
                    time.sleep(0.4)
                    rb = self._read_param_float("GF_MAX_VER_DIST")
                    if rb is not None and abs(rb - alt) < 0.5:
                        break
            # pause the reader so the mission handshake is the sole link consumer
            self._resume.clear()
            if not self._reader_paused.wait(timeout=3):
                self._note("[fence] ⚠️ reader ไม่ยอมหยุด ยกเลิก upload")
                return
            try:
                ok = self._mission_upload_fence(pts)
            finally:
                self._resume.set()
            if ok:
                with self.lock:
                    f = self.s["fence"]
                    f["pts"], f["action"], f["count"] = pts, action, len(pts)
                self._note(f"[fence] ✅ upload สี่เหลี่ยม {len(pts)} จุด + "
                           f"action={GF_ACTIONS.get(action, action)} สำเร็จ")
            else:
                self._note("[fence] ⚠️ upload ไม่สำเร็จ (วิทยุหลุด/ไม่ตอบ) — ลองกดใหม่")
        finally:
            with self.lock:
                self.s["fence"]["busy"] = False

    def _mission_upload_fence(self, pts):
        """Upload a polygon-inclusion fence via the mission protocol
        (MAV_MISSION_TYPE_FENCE). The reader is already paused by the caller."""
        mav, M = self.m.mav, mavutil.mavlink
        FENCE = M.MAV_MISSION_TYPE_FENCE
        n = len(pts)
        try:
            self.m.port.reset_input_buffer()
        except Exception:
            pass
        for _ in range(3):                       # retry the whole handshake
            with self.send_lock:
                mav.mission_count_send(1, 1, n, FENCE)
            t = time.time()
            while time.time() - t < 12:
                msg = self.m.recv_match(
                    type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                    blocking=True, timeout=1)
                if not msg:
                    continue
                if getattr(msg, "mission_type", FENCE) != FENCE:
                    continue
                if msg.get_type() == "MISSION_ACK":
                    return msg.type == M.MAV_MISSION_ACCEPTED
                seq = msg.seq
                if seq >= n:
                    continue
                lat, lon = pts[seq]
                with self.send_lock:
                    mav.mission_item_int_send(
                        1, 1, seq, M.MAV_FRAME_GLOBAL,
                        M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 0, 0,
                        float(n), 0.0, 0.0, 0.0,
                        int(lat * 1e7), int(lon * 1e7), 0.0, FENCE)
                t = time.time()
        return False

    def clear_fence_async(self):
        if self.demo:
            return
        threading.Thread(target=self._clear_fence_worker, daemon=True).start()

    def _clear_fence_worker(self):
        with self.lock:
            if self.s["fence"]["busy"]:
                return
            self.s["fence"]["busy"] = True
        try:
            self._resume.clear()
            if not self._reader_paused.wait(timeout=3):
                return
            try:
                with self.send_lock:
                    self.m.mav.mission_count_send(
                        1, 1, 0, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                self.m.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
            finally:
                self._resume.set()
            with self.lock:
                self.s["fence"]["pts"], self.s["fence"]["count"] = [], 0
            self._note("[fence] 🗑️ ล้าง fence แล้ว")
        finally:
            with self.lock:
                self.s["fence"]["busy"] = False

    def read_fence_async(self):
        if self.demo:
            return
        threading.Thread(target=self._read_fence_worker, daemon=True).start()

    def _read_fence_worker(self):
        act = self._read_param_int("GF_ACTION")
        cnt = None
        # ask the FMU how many fence points it has (PX4 has no GF_COUNT param;
        # query the fence via the mission protocol instead)
        self._resume.clear()
        if self._reader_paused.wait(timeout=3):
            try:
                try:
                    self.m.port.reset_input_buffer()
                except Exception:
                    pass
                with self.send_lock:
                    self.m.mav.mission_request_list_send(
                        1, 1, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                msg = self.m.recv_match(type="MISSION_COUNT",
                                        blocking=True, timeout=3)
                if msg and getattr(msg, "mission_type",
                                   mavutil.mavlink.MAV_MISSION_TYPE_FENCE) \
                        == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    cnt = msg.count
            finally:
                self._resume.set()
        with self.lock:
            self.s["fence"]["count"] = cnt
            if act is not None:
                self.s["fence"]["action"] = act
        self._note(f"[fence] อ่านค่า: fence {cnt} จุดบน FMU · "
                   f"action={GF_ACTIONS.get(act, act)}")

    # ---- ground auto-disarm timeout (COM_DISARM_PRFLT) -------------------
    def set_disarm_async(self, seconds):
        """Set COM_DISARM_PRFLT (FLOAT, seconds). Extending it gives the pilot
        time to arm via RC then press chirp before PX4 auto-disarms on the
        ground. Restore to ~10 s before real flight."""
        if self.demo:
            return
        threading.Thread(target=self._set_disarm_worker,
                         args=(float(seconds),), daemon=True).start()

    def _set_disarm_worker(self, seconds):
        ok = False
        for _ in range(6):
            try:
                with self.send_lock:
                    self.m.mav.param_set_send(
                        1, 1, b"COM_DISARM_PRFLT", float(seconds),
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            except Exception:
                pass
            time.sleep(0.7)
            rb = self._read_param_float("COM_DISARM_PRFLT")
            if rb is not None and abs(rb - seconds) < 0.5:
                ok = True
                break
            time.sleep(0.5)
        with self.lock:
            if rb is not None:
                self.s["disarm_prflt"] = rb
        if ok:
            self._note(f"[arm] ตั้ง COM_DISARM_PRFLT={seconds:.0f}s แล้ว — arm ค้างบนพื้นได้นานขึ้น")
        else:
            self._note("[arm] ⚠️ ตั้ง COM_DISARM_PRFLT ไม่สำเร็จ (ลิงก์ไม่นิ่ง) ลองใหม่")

    def _read_param_float(self, name):
        """Read one FLOAT param (no bit-reinterpret); retries for the flaky link."""
        for _ in range(4):
            try:
                with self.send_lock:
                    self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
                t = time.time()
                while time.time() - t < 1.5:
                    if not self._resume.is_set():
                        return None
                    pm = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                           timeout=1)
                    if pm and pm.param_id.strip("\x00") == name:
                        return round(pm.param_value, 2)
            except Exception:
                pass
        return None

    def _read_param_int(self, name):
        """Read one INT param and bit-reinterpret per PX4 (see bring-up notes).
        The reader thread shares recv, so a PARAM_VALUE can get stolen — retry
        the request a few times so a field read doesn't silently fail."""
        INT = (mavutil.mavlink.MAV_PARAM_TYPE_INT8,
               mavutil.mavlink.MAV_PARAM_TYPE_INT16,
               mavutil.mavlink.MAV_PARAM_TYPE_INT32,
               mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
               mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
               mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
        for _ in range(4):
            try:
                with self.send_lock:
                    self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
                t = time.time()
                while time.time() - t < 1.5:
                    if not self._resume.is_set():    # log pull grabbed the link
                        return None
                    pm = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                           timeout=1)
                    if pm and pm.param_id.strip("\x00") == name:
                        if pm.param_type in INT:
                            return f2i(pm.param_value)
                        return int(round(pm.param_value))
            except Exception:
                pass
        return None

    def enable_gps_async(self):
        """One-click fix for GPS_1_CONFIG=0 (port disabled): set it to 201
        (GPS 1 port) and reboot the FMU so the serial config takes effect.
        Field operator has no QGC, so this is the only way to flip it back."""
        if self.demo:
            return
        with self.lock:
            if self.s["gps_detect"]["busy"]:
                raise RuntimeError("กำลังทำงานกับ GPS อยู่ รอแป๊บ")
            self.s["gps_detect"]["busy"] = True
        threading.Thread(target=self._enable_gps_worker, daemon=True).start()

    def _enable_gps_worker(self):
        try:
            # INT param SET must bit-reinterpret the value into the float field.
            fval = struct.unpack("<f", struct.pack("<i", 201))[0]
            # The TELEM2 link can drop mid-write, so keep setting + reading back
            # until the FMU confirms 201 (don't reboot on an unconfirmed write).
            ok = False
            for attempt in range(8):
                try:
                    with self.send_lock:
                        self.m.mav.param_set_send(
                            1, 1, b"GPS_1_CONFIG", fval,
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                except Exception:
                    pass
                time.sleep(0.8)
                rb = self._read_param_int("GPS_1_CONFIG")
                if rb == 201:
                    ok = True
                    break
                self._note(f"[gps] ตั้งค่า… ครั้งที่ {attempt+1} (อ่านกลับได้ {rb}) ลองใหม่")
                time.sleep(0.6)
            if not ok:
                self._note("[gps] ⚠️ ตั้ง GPS_1_CONFIG ไม่สำเร็จ — ลิงก์ TELEM2 ไม่นิ่ง ลองกดใหม่")
                return
            with self.lock:
                self.s["gps_detect"]["config"] = 201
                self.s["gps_detect"]["config_str"] = "GPS 1 port (201)"
            self._note("[gps] ✅ ตั้ง GPS_1_CONFIG=201 + บันทึกแล้ว — ต้อง 'ปิด-เปิดไฟ Drone ใหม่' (power-cycle) เพื่อให้พอร์ต GPS เริ่มทำงาน")
            # best-effort reboot too; if the flaky link drops it, the power-cycle
            # the operator does anyway will apply the serial-port config.
            try:
                with self.send_lock:
                    self.m.mav.command_long_send(
                        1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                        0, 1, 0, 0, 0, 0, 0, 0)
                self._note("[gps] ส่งคำสั่งรีบูต FMU แล้ว — ถ้าไม่กลับมาใน ~30 วิ ให้ power-cycle เอง")
            except Exception:
                self._note("[gps] ส่งรีบูตไม่ได้ — power-cycle Drone เองได้เลย")
        except Exception as e:
            self._note(f"[gps] enable error: {e}")
        finally:
            with self.lock:
                self.s["gps_detect"]["busy"] = False

    # ---- log download (MAVFTP over the live link) ------------------------
    def _set_logpull(self, msg, busy=True):
        with self.lock:
            self.s["logpull"] = {"busy": busy, "msg": msg}

    def pull_log_async(self):
        """Kick off a background pull of the newest .ulg off the FMU SD card."""
        if self.demo:
            return
        with self.lock:
            if self.s["logpull"]["busy"]:
                raise RuntimeError("กำลังดึง log อยู่แล้ว")
            self.s["logpull"] = {"busy": True, "msg": "เริ่มดึง log…"}
        threading.Thread(target=self._pull_log_worker, daemon=True).start()

    def _pull_log_worker(self):
        import pull_log
        self._resume.clear()                      # ask reader/heartbeat/poll to stand down
        if not self._reader_paused.wait(timeout=5):
            self._resume.set()
            self._note("[log] ดึงไม่ได้ — ลิงก์ไม่ว่าง (FMU ออนไลน์อยู่ไหม?)")
            self._set_logpull("ดึงไม่สำเร็จ (ลิงก์ไม่ว่าง)", busy=False)
            return
        time.sleep(1.2)                           # let any in-flight heartbeat/poll settle
        try:
            path = pull_log.pull_newest_ulog(self.m, LOG_DIR, note=self._note)
            self._set_logpull(
                f"เสร็จ: {os.path.basename(path)}" if path else "ไม่พบ/ดึงไม่สำเร็จ",
                busy=False)
        except Exception as e:
            self._note(f"[log] error: {e}")
            self._set_logpull(f"error: {e}", busy=False)
        finally:
            self._resume.set()                    # resume reader/heartbeat/poll

    def release_link_for_chirp(self):
        """Fully close the serial link so a SEPARATE process (flight_runner) can
        own /dev/serial0 during the chirp. On the CM4 there is exactly ONE UART to
        the FMU (TELEM2), so the GCS reader/heartbeat/poll must stand down AND the
        fd must be closed — otherwise both openers get "multiple access on port"
        and the OFFBOARD setpoint stream never reaches the FMU (it auto-disarms).
        Returns True if the link was parked + released, False if it was busy."""
        self._resume.clear()                      # reader/heartbeat/poll stand down
        if not self._reader_paused.wait(timeout=5):
            self._resume.set()
            return False
        time.sleep(1.2)                           # let any in-flight heartbeat settle
        try:
            self.m.close()
        except Exception:
            pass
        with self.lock:
            self.s["link"] = False
        self._note("[chirp] ปล่อย serial0 ให้ flight_runner (telemetry พักชั่วคราว)")
        return True

    def reacquire_link_after_chirp(self):
        """Reopen the serial link and resume telemetry after flight_runner exits.
        The resumed reader re-asserts message streams on the first iteration."""
        try:
            self.m = mavutil.mavlink_connection(self.url, baud=self.baud)
        except Exception as e:
            self._note(f"[chirp] เปิด serial0 กลับไม่ได้: {e}")
        self._resume.set()                        # reader/heartbeat/poll resume
        self._note("[chirp] ต่อ serial0 กลับ — telemetry กลับมา")

    # ---- command senders (called from HTTP handler) ----------------------
    def _cmd(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        with self.send_lock:
            self.m.mav.command_long_send(1, 1, cmd, 0, *p)

    def arm(self):
        self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)

    def disarm(self):
        self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)

    def kill(self):
        for _ in range(5):
            self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
            time.sleep(0.05)

    def set_mode(self, main, sub=0):
        self._cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                  mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def land(self):
        for _ in range(3):
            self.set_mode(4, 6)            # AUTO.LAND
            time.sleep(0.1)

    # ---- AAVC payload-release servos (DO_SET_ACTUATOR) ---------------------
    # ADD-ONLY: a brand-new command path for the payload latches. It reuses the
    # existing _cmd() sender but does NOT touch any existing command method. The
    # "must pick pads first" interlock is enforced in do_POST before we get here.
    def release_servos(self, which=None):
        """Drive payload servos to their RELEASED pwm (drop). which=None -> all."""
        self._servo_move(which, released=True)

    def reset_servos(self, which=None):
        """Drive them back to HELD/closed so the latch is ready for another test."""
        self._servo_move(which, released=False)

    def _servo_move(self, which, released):
        servos = load_payload_servos()
        if which is not None:
            want = set(int(x) for x in which)
            servos = [s for s in servos if s["num"] in want]
        key = "released_us" if released else "held_us"
        with self.lock:                      # update state (shown in UI) — demo too
            st = self.s.setdefault("servos", {})
            for s in servos:
                st[str(s["num"])] = {"num": s["num"], "released": released,
                                     "pwm": int(s[key])}
        if not self.demo:                    # only a live link actually moves a servo
            pairs = [(s["num"], int(s[key])) for s in servos]
            threading.Thread(target=self._servo_send, args=(pairs, released),
                             daemon=True).start()
        verb = "ปล่อย" if released else "ปิดกลับ"
        nums = ", ".join(f"AUX {s['num']}"
                         + (f" ({s['label']})" if s.get("label") else "")
                         for s in servos) or "-"
        with self.lock:
            armed = bool(self.s.get("armed"))
        # say WHICH path was used: the two are mutually exclusive in PX4, so an
        # operator who sees nothing move needs to know which door was knocked on
        how = "DO_SET_ACTUATOR (armed)" if armed else "ACTUATOR_TEST (bench)"
        self._note(f"[servo] {verb} {nums} — {how} ✅")

    def _servo_send(self, pairs, released=True):
        # 2026-08-12: PX4 has NO handler for MAV_CMD_DO_SET_SERVO (183) — the old
        # send was a silent no-op on SITL AND the real Pixhawk. The only path PX4
        # implements is MAV_CMD_DO_SET_ACTUATOR (187): param7 (index) = 0,
        # param1..6 = values [-1..1] for output functions "Peripheral via
        # Actuator Set 1..6" (301..306), NaN = leave that slot untouched.
        # Latches: PWM_AUX_FUNC1..4 = 301..304 (AUX ch 9..12) on the real bird,
        # SIM_GZ_SV_FUNC1..4 in SITL. Output numbers in the field yaml may be
        # the AUX channel (9..12 → set index num-8) or the set index itself
        # (1..6). pwm→value: (pwm-1500)/500, so 1900 = +0.8 / 1100 = -0.8.
        # Send a few times each — best-effort over a possibly-narrow radio.
        #
        # BENCH vs FLIGHT (2026-08-15): PX4 MASKS every servo output while the
        # vehicle is disarmed, so DO_SET_ACTUATOR on a bench does nothing at
        # all — the button looked broken because the aircraft was, correctly,
        # refusing to move a latch on a disarmed vehicle. PX4 has a second
        # path for exactly this: MAV_CMD_ACTUATOR_TEST (310), which is the
        # MIRROR IMAGE — Commander denies it while ARMED (and with the safety
        # switch on) and requires COM_MOT_TEST_EN=1. So:
        #     disarmed → ACTUATOR_TEST  (bench: no arming, no props spinning)
        #     armed    → DO_SET_ACTUATOR (in flight / on the pad)
        # ACTUATOR_TEST params (Commander.cpp handleCommandActuatorTest):
        #   param1 = value −1..1, param2 = timeout s (≤0 releases control),
        #   param5 = function, where values ≥1000 mean "raw output function
        #   + 1000" → our latches (Peripheral via Actuator Set n = 300+n)
        #   are 1301..1304. Below 1000 it would be read as a motor/servo index.
        nan = float("nan")
        # pymavlink 2.4.49 dropped the MAV_CMD_DO_SET_ACTUATOR constant from its
        # dialects — the command id is 187 and PX4 v1.17 still handles it.
        cmd = getattr(mavutil.mavlink, "MAV_CMD_DO_SET_ACTUATOR", 187)
        cmd_test = getattr(mavutil.mavlink, "MAV_CMD_ACTUATOR_TEST", 310)
        with self.lock:
            armed = bool(self.s.get("armed"))
        for num, pwm in pairs:
            idx = int(num) - 8 if int(num) >= 9 else int(num)
            if not 1 <= idx <= 6:
                self._note(f"[servo] output {num} ไม่ map เข้า actuator set 1-6 ❌")
                continue
            val = max(-1.0, min(1.0, (float(pwm) - 1500.0) / 500.0))
            if armed:
                vals = [nan] * 6
                vals[idx - 1] = val
                for _ in range(3):
                    self._cmd(cmd, *vals, 0)
                    time.sleep(0.05)
            elif released:
                self._servo_test_hold(idx, val)
            else:
                self._servo_test_stop(idx)

    # ── bench (disarmed) latch control via ACTUATOR_TEST ──
    #
    # ACTUATOR_TEST is a WATCHDOGGED override, not a switch: PX4 only overrides
    # an output while it is "in test mode", and that mode ends the moment the
    # command's timeout expires (mixer_module/actuator_test.cpp — _next_timeout
    # is armed only when timeout_ms > 0, and Commander turns any param2 ≤ 0 into
    # RELEASE_CONTROL, so "hold forever" cannot be expressed in one command).
    # Sending it once therefore gives a latch that springs back by itself, so to
    # make the console button behave like an ON/OFF switch we re-send while the
    # latch should stay open — like QGC's actuator sliders.
    #
    # ONE supervisor owns ALL the re-sending (2026-08-19). The previous design
    # started a fresh keep-alive THREAD per press and tracked one stop-Event per
    # channel; pressing the buttons rapidly raced the pop/replace of that Event
    # and ORPHANED keep-alive loops that nothing could stop except killing the
    # process — the field-reported "latch opens/closes by itself after 5-10 min".
    # Now the buttons only edit a desired-set {idx: value}; a single long-lived
    # thread re-sends the held channels and RELEASE_CONTROLs the ones just
    # removed. Idempotent under any number of concurrent presses, unraceable
    # (no per-press threads to leak), and self-clearing once the vehicle arms.

    _TEST_PERIOD_S = 0.4        # resend interval (PX4 ignores frames > 100 ms old)
    _TEST_TIMEOUT_S = 1.5       # per-command timeout: > period, so no flicker;
                                # short enough that a dead console releases fast

    def _servo_test_hold(self, idx, value):
        """Mark actuator-set `idx` to be held open until _servo_test_stop()."""
        with self._servo_lock:
            self._servo_desired[idx] = float(value)
        self._ensure_servo_sup()

    def _servo_test_stop(self, idx):
        """Drop `idx` from the held set and hand the output straight back to PX4.
        NON-BLOCKING: one immediate RELEASE_CONTROL, then the supervisor re-sends
        RELEASE on its next tick (<=0.4s) and — since no more HOLD frames arrive —
        the ACTUATOR_TEST watchdog closes the latch within 1.5s regardless. No
        per-channel sleeps, so 'close all' is as snappy as closing one latch; the
        old 0.4s + 3x0.05s PER channel stacked into the 4-6s the operator saw."""
        cmd_test = getattr(mavutil.mavlink, "MAV_CMD_ACTUATOR_TEST", 310)
        with self._servo_lock:
            self._servo_desired.pop(idx, None)
        self._cmd(cmd_test, 0.0, 0.0, 0, 0, 1000 + 300 + idx, 0, 0)  # param2=0 → RELEASE

    def _ensure_servo_sup(self):
        """Start the single keep-alive supervisor once, lazily."""
        with self._servo_lock:
            if self._servo_sup_started:
                return
            self._servo_sup_started = True
        threading.Thread(target=self._servo_supervisor, daemon=True,
                         name="servo-keepalive").start()

    def _servo_supervisor(self):
        cmd_test = getattr(mavutil.mavlink, "MAV_CMD_ACTUATOR_TEST", 310)
        prev: set[int] = set()
        while True:
            with self._servo_lock:
                desired = dict(self._servo_desired)
            with self.lock:
                armed = bool(self.s.get("armed"))
            if armed and desired:
                # a bench hold cannot survive into armed flight (PX4 denies
                # ACTUATOR_TEST while armed anyway) — forget them so nothing
                # resumes on the next disarm, and release below.
                with self._servo_lock:
                    self._servo_desired.clear()
                desired = {}
            for i, val in desired.items():           # keep held channels open
                # param1 value, param2 timeout s, param5 = 1000 + output function
                self._cmd(cmd_test, val, self._TEST_TIMEOUT_S, 0, 0,
                          1000 + 300 + i, 0, 0)
            for i in prev - desired.keys():           # channels just released
                self._cmd(cmd_test, 0.0, 0.0, 0, 0, 1000 + 300 + i, 0, 0)
            prev = set(desired.keys())
            time.sleep(self._TEST_PERIOD_S)

    def cal_accel(self):
        # PREFLIGHT_CALIBRATION param5=1 -> accelerometer (6-orientation).
        with self.lock:
            self.s["cal_active"] = True
            self.s["cal"] = {"active": True, "type": "accel",
                             "instr": "กำลังเริ่ม… วาง Drone แต่ละด้านตามภาพ",
                             "sides": {k: "pending" for k in self.ACC_SIDES},
                             "done": False, "failed": False}
        self._cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                  0, 0, 0, 0, 1, 0, 0)
        threading.Thread(target=self._cal_watch, daemon=True).start()

    def cal_level(self):
        with self.lock:
            self.s["cal_active"] = True
            self.s["cal"] = {"active": True, "type": "level",
                             "instr": "วาง Drone ระดับบนพื้นเรียบ ค้างนิ่ง",
                             "sides": {"level": "pending"},
                             "done": False, "failed": False}
        self._cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                  0, 0, 0, 0, 2, 0, 0)
        threading.Thread(target=self._cal_watch, daemon=True).start()

    def is_cal_active(self):
        with self.lock:
            return self.s["cal"]["active"]

    def cal_cancel(self):
        # All-zero PREFLIGHT_CALIBRATION tells PX4 to abort the calibration.
        self._cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0)
        with self.lock:
            self.s["cal_active"] = False
            self.s["cal"]["active"] = False
            self.s["cal"]["instr"] = "ยกเลิกแล้ว"
            self.s["messages"].append(
                {"t": time.strftime("%H:%M:%S"), "txt": "[cal] ยกเลิก calibration"})
            del self.s["messages"][:-40]

    def _cal_watch(self):
        # Clear the cal flag once we see a done/failed statustext (or timeout).
        t = time.time()
        while time.time() - t < 90:
            with self.lock:
                msgs = self.s["messages"][-3:]
            for mm in msgs:
                low = mm["txt"].lower()
                if "calibration done" in low or "cal done" in low \
                        or "fail" in low and "cal" in low:
                    self.fetch_accel_offsets()
                    with self.lock:
                        self.s["cal_active"] = False
                        self.s["cal"]["active"] = False
                    return
            time.sleep(1)
        self.fetch_accel_offsets()
        with self.lock:
            self.s["cal_active"] = False
            self.s["cal"]["active"] = False

    def snapshot(self):
        with self.lock:
            snap = json.loads(json.dumps(self.s))   # deep copy
        snap["demo"] = self.demo
        snap["link_kind"] = self._link_kind()
        # --- AAVC additions: file-based + derived, read-only (never touches the link) ---
        snap["pads"] = PAD_IDS
        snap["assignment"] = default_assignment()
        # which servos exist + the airframe corner each one opens
        snap["servo_cfg"] = [{"num": s["num"], "label": s.get("label", "")}
                             for s in load_payload_servos()]
        snap["pads_selected"] = bool(selected_pads())                   # interlock signal
        snap["mission_cmd"] = bool(MISSION_CMD)          # 🚀 GO button available?
        snap["mission_label"] = MISSION_LABEL            # SIM/REAL badge on 🚀
        snap["mission_running"] = mission_running()      # edit-lock while flying
        snap["reset_cmd"] = bool(RESET_CMD)              # 🧹 button (SIM only)
        snap["reset_running"] = reset_running()
        snap["missions"] = mission_registry_snapshot()   # in-UI switcher list
        snap["mission_current"] = CURRENT_MISSION
        snap["cm4_ok"] = _CM4_OK                         # None = local cmd (SIM)
        snap["real_console"] = REAL_CONSOLE              # template switch locked
        snap["real_host"] = _mission_cmd_ssh_host(MISSION_CMD)   # CM4 shown on the card
        snap["real_dir"] = _mission_cmd_remote_dir(MISSION_CMD)  # repo dir on the CM4
        lid = snap.get("lidar") or {}
        snap["lidar_age"] = (round(time.time() - lid["t"], 1)
                             if lid.get("t") else None)
        # PX4's last GPS prearm-fail (PDOP too high, …) with its age, so the GPS
        # chip can go RED to MATCH PX4 refusing to arm — not just show 3D green.
        gp = snap.get("gps_prearm") or {}
        snap["gps_prearm"] = {"ok": gp.get("ok", True), "txt": gp.get("txt", ""),
                              "age": (round(time.time() - gp["t"], 1)
                                      if gp.get("t") else None)}
        # which pipe carries the telemetry this console is showing — drives
        # the "วิทยุ" chip in the sensor strip (a serial --url = the NOMAD)
        snap["link_kind"] = "radio" if str(self.url).startswith("/dev/") else "udp"
        # nadir camera feed age. In SITL the gz bridge writes this file HERE, so
        # its age IS the camera's health. On the real aircraft the file is written
        # on the CM4, and now that the WiFi puller is gone (2026-08-18, radio-pure
        # console) whatever sits at this path is a leftover from the last
        # simulator run — hours old, and about a different camera entirely.
        # Reporting its age paints the chip red with a number that LOOKS measured,
        # which is worse than admitting there is no local reading: radio mode
        # answers None, so the chip falls to a grey "n/a" unless the beacon spoke.
        try:
            snap["cam_age"] = (None if snap["link_kind"] == "radio" else round(
                time.time() - os.path.getmtime(_nadir_frame_path()), 1))
        except OSError:
            snap["cam_age"] = None
        # Camera health over the RADIO (status_beacon): measured ON the CM4,
        # right next to the camera — the ONLY camera reading a real flight gets.
        rc = self.s.get("radio_cam")
        snap["cam_radio"] = ({"state": rc["state"], "age": rc.get("age")}
                             if rc and time.time() - rc["t"] <= 15 else None)
        snap["zones"] = load_zones()
        origin = self._aavc_origin(snap)
        mission = read_mission_status()
        rm = self.s.get("radio_mission")
        if (rm and time.time() - rm["t"] <= 15          # <= 3 beacon ticks lost
                and not rm["phase"].startswith("idle")
                and (mission is None or (mission.get("age_s") or 0) > 45)):
            # WiFi feed dead but the radio beacon is alive: run the readouts on
            # the summary the beacon carries — phase + counts + delivered ids +
            # the pad coordinates from the "AAVC pads" lines (same ENU-about-
            # origin numbers mission_status carries, so the map draws the same
            # markers). A pad the beacon stops re-sending for 20 s (4 ticks)
            # drops off, matching the stale-pads rule for the WiFi feed.
            asg = snap.get("assignment") or []
            rp = self.s.get("radio_pads") or {}
            live_pads = {pid: v["en"] for pid, v in rp.items()
                         if time.time() - v["t"] <= 20}
            rw = self.s.get("radio_why")
            why = (WHY_TH.get(rw["code"], rw["code"])
                   if rw and time.time() - rw["t"] <= 15 else None)
            # Age the mission by the DATA, not by the packet. The beacon resends
            # the same file every 5 s, so packet age is always ~0 even for a
            # flight that ended half an hour ago; "AAVC stale=" carries the real
            # age and only shows up once the writer has stopped. Adding the time
            # since that line arrived keeps the number honest between beacons.
            rs = self.s.get("radio_stale")
            data_age = (rs["age"] + (time.time() - rs["t"])
                        if rs and time.time() - rs["t"] <= 20 else 0.0)
            # identified-but-unconfirmed ids from "AAVC seen=" — no coords on
            # the radio (budget), so the value is None: the sidebar ladder
            # lights orange from the id alone; the map needs a position and
            # only draws identified pads on the WiFi feed.
            rsn = self.s.get("radio_seen")
            seen_ids = (rsn["ids"] if rsn and time.time() - rsn["t"] <= 15
                        else [])
            mission = {"phase": rm["phase"], "delivered": rm["ok"],
                       "assigned": asg if len(asg) == rm["assigned_n"] else [],
                       "pads_mapped": live_pads, "mapped_n": rm["mapped_n"],
                       "pads_identified": {str(i): None for i in seen_ids},
                       "home_reason": why,
                       "age_s": round(time.time() - rm["t"] + data_age, 1),
                       "src": "radio"}
            # …and the awareness pack, rebuilt from the "AAVC prg=" line. The
            # %-bar and the milestone strip both refuse to draw unless
            # mission.progress is a number, so omitting these does not degrade
            # the display gracefully — it removes it. Two fields are rebuilt
            # rather than sent, because the console reads them out of Thai
            # strings an ascii STATUSTEXT cannot carry: progress_label (the
            # strip finds the CURRENT pad by searching it for "pad N ") and
            # events (it ticks P1·P2·P3 by searching for "ผ่านจุด Pn").
            rg = self.s.get("radio_prog")
            if rg and time.time() - rg["t"] <= 15:
                label = PHASE_TH.get(rm["phase"], rm["phase"])
                if rg["cur"] is not None:
                    label = (f"ส่งของ pad {rg['cur']} "
                             f"({len(rm['ok']) + 1}/{rm['assigned_n']})")
                mission["progress"] = rg["pct"]
                mission["eta_s"] = rg["eta"]
                mission["progress_label"] = label
                mission["events"] = [{"text": f"✅ ผ่านจุด P{i + 1}"}
                                     for i, hit in enumerate(rg["tp"]) if hit == "1"]
        if self.demo and (mission is None or (mission.get("age_s") or 0) > 45):
            mission, origin = self._demo_mission(snap, origin)   # demo: always show a live mission
        snap["origin"] = origin
        snap["mission"] = mission
        bb = snap.get("blackbox") or {}
        bb["age_s"] = (round(time.time() - bb["t"], 1)
                       if bb.get("t") else None)   # seconds since last recorded fix
        return snap

    def _aavc_origin(self, snap):
        """lat/lon of local (0,0): GPS_GLOBAL_ORIGIN if PX4 sent it, else derived from the
        drone's current GPS fix + LOCAL_POSITION_NED (no extra MAVLink request needed)."""
        o = snap.get("origin") or {}
        if o.get("lat") is not None and o.get("lon") is not None:
            return {"lat": o["lat"], "lon": o["lon"]}
        g, loc = snap.get("gps") or {}, snap.get("local") or {}
        if g.get("lat") is not None and loc.get("n") is not None:
            R = 6378137.0
            lat0 = g["lat"] - (loc["n"] / R) * 180.0 / math.pi
            lon0 = g["lon"] - (loc["e"] / (R * math.cos(math.radians(g["lat"])))) * 180.0 / math.pi
            return {"lat": round(lat0, 7), "lon": round(lon0, 7)}
        return {"lat": None, "lon": None}

    def _demo_mission(self, snap, origin):
        """Synthesize a couple of 'detected' pads near the demo GPS so --demo shows the
        pad-on-map feature with no running mission."""
        g = snap.get("gps") or {}
        if origin.get("lat") is None and g.get("lat") is not None:
            origin = {"lat": g["lat"], "lon": g["lon"]}     # demo: origin = drone position
        asg = default_assignment() or [1, 2, 3]
        found = asg[:max(1, len(asg) - 1)]           # demo: "found" all assigned but the last
        mapped = {str(pid): [10.0 + 8 * i, 14.0 + 6 * i] for i, pid in enumerate(found)}
        mission = {"phase": "deliver (drop)", "pads_mapped": mapped,
                   "assigned": asg, "delivered": asg[:1],   # demo: already "dropped" the 1st assigned
                   "mission_time": 128.0, "updated": time.time(), "age_s": 0.0,
                   # awareness-pack fields (2026-08-14) so --demo previews the
                   # %-bar + timeline exactly as a live mission renders them
                   "progress": 64,
                   "progress_label": f"ส่งของ pad {asg[1] if len(asg) > 1 else asg[0]} (2/{len(asg)})",
                   "eta_s": 260,
                   # fixed t keys — a fresh time.time() per snapshot made every
                   # tick look like a NEW event and the toasts replayed forever
                   "events": [{"t": 1, "text": "✅ ผ่านจุด P3", "warn": False},
                              {"t": 2, "text": f"🎯 เจอ pad {found[-1]}!", "warn": False},
                              {"t": 3, "text": f"📦 วางแล้ว pad {asg[0]}", "warn": False}]}
        return mission, origin

    def _link_kind(self):
        """Which physical link is this? Used to LOCK the polygon-fence upload to a
        reliable wired link (USB/CM4) — the mission-protocol handshake is flaky over
        the narrow ELRS radio, so uploading a half-formed fence there is a footgun."""
        if self.demo:
            return "demo"
        u = self.url or ""
        if "ttyACM" in u:
            return "usb"                         # FMU FC-USB (direct, fast)
        if "serial0" in u or "ttyAMA" in u:
            return "cm4"                         # onboard CM4 TELEM2 (fast)
        if "ttyUSB" in u or "CP2102" in u or "Silicon_Labs" in u:
            return "radio"                       # NOMAD/ELRS (narrow, flaky)
        return "other"

    def _blackbox_writer(self):
        """Ground-side black box. Once a second, while the link is up and we have a
        GPS fix, append the last-known position + attitude to a CSV on the LAPTOP.
        Purely a *reader* of self.s — never sends on the link, so it cannot disturb
        flight (honours the ADD-only rule). When the link drops the timestamp stops
        advancing, so the last row = the last place we heard from the aircraft: walk
        there to find it. One file per session; the file is created lazily on the
        first real fix, so bench sessions with no GPS leave no empty files."""
        fh = None
        path = None
        last_write = 0.0
        while not self._stop:
            time.sleep(1.0)
            try:
                with self.lock:
                    link = self.s.get("link")
                    g = dict(self.s.get("gps") or {})
                    a = dict(self.s.get("att") or {})
                    v = dict(self.s.get("vel") or {})
                    b = dict(self.s.get("batt") or {})
                    mode = self.s.get("mode")
                    armed = self.s.get("armed")
                    in_air = self.s.get("in_air")
                lat, lon = g.get("lat"), g.get("lon")
                # only log a fresh, real fix while the link is alive (a dropped link
                # freezes the record at the last-known point — exactly what we want)
                if not link or lat is None or lon is None or (g.get("fix") or 0) < 2:
                    continue
                now = time.time()
                if now - last_write < 1.0:
                    continue
                if fh is None:
                    os.makedirs(BLACKBOX_DIR, exist_ok=True)
                    path = os.path.join(
                        BLACKBOX_DIR, time.strftime("flight_%Y%m%d_%H%M%S.csv"))
                    fh = open(path, "a", buffering=1)
                    fh.write("iso,unix,lat,lon,alt_msl,rel_alt,roll,pitch,yaw,"
                             "heading,vx,vy,vz,vbatt,batt_pct,mode,armed,in_air\n")
                    self._note(f"[blackbox] 🛰 บันทึกกล่องดำ → {os.path.basename(path)}")
                    with self.lock:
                        self.s["blackbox"]["file"] = os.path.basename(path)
                row = [time.strftime("%Y-%m-%dT%H:%M:%S"), f"{now:.0f}",
                       lat, lon, g.get("alt"), g.get("rel_alt"),
                       a.get("roll"), a.get("pitch"), a.get("yaw"), a.get("heading"),
                       v.get("vx"), v.get("vy"), v.get("vz"),
                       b.get("volt"), b.get("pct"), mode,
                       1 if armed else 0, 1 if in_air else 0]
                fh.write(",".join("" if x is None else str(x) for x in row) + "\n")
                last_write = now
                alt = g.get("rel_alt") if g.get("rel_alt") is not None else g.get("alt")
                with self.lock:
                    bb = self.s["blackbox"]
                    bb["lat"], bb["lon"], bb["alt"], bb["t"] = lat, lon, alt, now
                    bb["rows"] = bb.get("rows", 0) + 1
            except Exception:
                pass                              # never let the black box break the GCS


class CM4:
    """Tracks reachability of the onboard Raspberry Pi CM4 and launches the
    autonomous chirp flight on it over SSH (`scripts/run_pi.sh`). The laptop GCS
    keeps its own USB link to the FMU, so LAND/KILL stay available as the primary
    abort path independent of this SSH channel."""

    def __init__(self, link: "Link", local: bool = False):
        self.link = link
        self.local = local                   # True when this GCS *runs on* the CM4
        self.ip = "localhost" if local else CM4_FIXED
        self.proc = None                     # running flight (ssh or local subprocess)
        self.lock = threading.Lock()
        self._set(online=local, ip=self.ip, flying=False)

    def _set(self, **kw):
        with self.link.lock:
            self.link.s.setdefault("cm4", {})
            self.link.s["cm4"].update(kw)

    def _note(self, txt):
        self.link._note(txt)

    # ---- discovery ---------------------------------------------------------
    @staticmethod
    def _local_ips():
        try:
            out = subprocess.run(["hostname", "-I"], capture_output=True,
                                 text=True, timeout=2).stdout
            return out.split()
        except Exception:
            return []

    @staticmethod
    def _ssh_open(ip, timeout=0.6):
        try:
            with socket.create_connection((ip, 22), timeout):
                return True
        except OSError:
            return False

    def _discover(self):
        if CM4_FIXED:
            return CM4_FIXED if self._ssh_open(CM4_FIXED) else None
        # keep the last good ip if it still answers (cheap, avoids a full sweep)
        if self.ip and self._ssh_open(self.ip):
            return self.ip
        for ip in self._local_ips():
            p = ip.split(".")
            if len(p) != 4 or p[3] == CM4_HOST_OCTET:
                continue
            cand = ".".join(p[:3] + [CM4_HOST_OCTET])
            if self._ssh_open(cand):
                return cand
        return None

    def _watch(self):
        # On the CM4 itself there is nothing to discover — this machine IS the CM4.
        if self.local:
            while not self.link._stop:
                with self.lock:
                    flying = self.proc is not None and self.proc.poll() is None
                self._set(online=True, ip="บนบอร์ด (CM4)", flying=flying)
                time.sleep(2)
            return
        while not self.link._stop:
            found = self._discover()
            was = self.ip
            self.ip = found
            with self.lock:
                flying = self.proc is not None and self.proc.poll() is None
            self._set(online=found is not None, ip=found, flying=flying)
            if found and not was:
                self._note(f"[cm4] online @ {found}")
            elif was and not found:
                self._note("[cm4] offline (หา CM4 ไม่เจอบนเครือข่าย)")
            time.sleep(4)

    def start(self):
        threading.Thread(target=self._watch, daemon=True).start()

    # ---- chirp flight ------------------------------------------------------
    def _ssh_cmd(self, remote):
        return ["ssh", "-i", CM4_KEY, "-o", "StrictHostKeyChecking=no",
                "-o", "BatchMode=yes", "-o", "ConnectTimeout=6",
                f"{CM4_USER}@{self.ip}", remote]

    def run_chirp(self):
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError("chirp กำลังบินอยู่แล้ว")
            # SAFEGUARD: pilot arms via RC ON THE GROUND, then triggers chirp; the
            # runner takes over (OFFBOARD) for takeoff+chirp. Require armed AND on the
            # ground. Refusing the airborne case is the key guard — otherwise pressing
            # it mid-flight yanks the craft into OFFBOARD toward (0,0,takeoff_alt),
            # flying it back to origin + down to 5 m.
            if not self.link.s.get("armed"):
                raise RuntimeError("ยังไม่ armed — arm ผ่าน RC ก่อน (อยู่บนพื้น) แล้วค่อยกด Chirp")
            if self.link.s.get("in_air"):
                raise RuntimeError(" Drone กำลังลอยอยู่ — กด Chirp ได้เฉพาะตอนอยู่บนพื้นเท่านั้น")
            if self.local:
                # this GCS runs on the CM4 -> launch run_pi.sh as a local subprocess.
                # The CM4 has a SINGLE UART to the FMU (/dev/serial0); the GCS holds
                # it open for telemetry, so we must release it before flight_runner
                # opens it, or they collide ("multiple access on port") and OFFBOARD
                # never streams -> FMU auto-disarms. Reacquired in _pump after exit.
                if not self.link.release_link_for_chirp():
                    raise RuntimeError("ปล่อย serial0 ไม่ได้ (ลิงก์ไม่ว่าง) — ลองใหม่")
                self.proc = subprocess.Popen(
                    ["bash", "scripts/run_pi.sh"], stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1)
            else:
                if not self.ip:
                    raise RuntimeError("CM4 ออฟไลน์ — เชื่อมต่อ CM4 ก่อน")
                remote = f"cd {CM4_REPO} && bash scripts/run_pi.sh 2>&1"
                self.proc = subprocess.Popen(
                    self._ssh_cmd(remote), stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1)
        self._set(flying=True)
        self._note("[chirp] เริ่มบิน chirp (run_pi.sh) — RC พร้อม override")
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        try:
            for line in self.proc.stdout:
                line = line.rstrip()
                if line:
                    self._note(f"[chirp] {line}")
        except Exception as e:
            self._note(f"[chirp] stream error: {e}")
        rc = self.proc.wait()
        if self.local:
            # flight_runner has released /dev/serial0 -> GCS takes it back
            self.link.reacquire_link_after_chirp()
        self._set(flying=False)
        self._note(f"[chirp] จบการบิน (exit {rc})")

    def stop_chirp(self):
        """Stop the autonomous chirp. Two independent actions:
        1) kill flight_runner -> its OFFBOARD setpoint stream stops -> PX4 enters the
           offboard-loss failsafe. (Bracket pattern '[f]light_runner' so pkill's own
           cmdline doesn't self-match.)
        2) force-disarm over the GCS's OWN MAVLink link (no second process / no serial
           contention / no venv issue, unlike the old `python3 abort.py` which failed:
           system python lacks pymavlink and abort would fight gcs_server for serial0).
        RC remains the primary override."""
        if self.local:
            subprocess.Popen(["pkill", "-9", "-f", "[f]light_runner"])
        elif self.ip:
            subprocess.Popen(self._ssh_cmd("pkill -9 -f '[f]light_runner'"))
        try:
            self.link.kill()                  # force-disarm via gcs_server's own link
        except Exception as e:
            self._note(f"[chirp] kill-via-link error: {e}")
        self._note("[chirp] หยุด chirp: kill flight_runner + force-disarm (RC = override หลัก)")


LINK: Link = None  # set in main
CM4MGR: CM4 = None  # set in main

# Minimal diagnostic page: tells us, with no dev-console, whether JS runs and
# whether fetch() works in the user's browser (the dashboard depends on both).
TESTPAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"></head>
<body style="font-family:sans-serif;font-size:22px;padding:20px;background:#111;color:#eee">
<div id=js style="padding:14px;background:#a00;border-radius:8px">✖ บรรทัดนี้แปลว่า JavaScript ไม่ทำงาน</div>
<div id=fetch style="margin-top:14px;padding:14px;background:#555;border-radius:8px">… ยังไม่ได้ลอง fetch</div>
<script>
document.getElementById('js').style.background='#0a0';
document.getElementById('js').textContent='✔ JavaScript ทำงาน (JS-OK)';
fetch('/api/status').then(r=>r.json()).then(s=>{
 var e=document.getElementById('fetch');e.style.background='#0a0';
 e.textContent='✔ FETCH-OK — link='+s.link+' mode='+s.mode+' sats='+s.gps.sats;
}).catch(e=>{
 var x=document.getElementById('fetch');x.style.background='#a00';
 x.textContent='✖ FETCH-FAIL: '+e;
});
</script></body></html>"""


# --------------------------------------------------------------------------
PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Sys_ID GCS</title>
<link rel="stylesheet" href="/leaflet.css">
<script defer src="/leaflet.js"></script>
<style>
*{box-sizing:border-box;font-family:system-ui,sans-serif}
body{margin:0;background:#0e1116;color:#e6e6e6;font-size:17px}
header{padding:10px 16px;background:#161b22;font-weight:600;font-size:23px;
 display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
.dot{height:11px;width:11px;border-radius:50%;display:inline-block;margin-right:6px}
.ok{color:#3fb950}.bad{color:#f85149}.warn{color:#d29922}
.status{display:grid;grid-template-columns:repeat(auto-fill,minmax(96px,1fr));gap:6px;max-width:404px;align-content:start}
.scbody{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap}
.scinfo{display:flex;flex-direction:column;min-width:200px;flex:1}
.scinfo .mprogwrap{border-top:none;margin-top:0;padding-top:0}
.scinfo .mprogwrap+.mprogwrap{border-top:1px solid #222b36;margin-top:6px;padding-top:6px}
/* ---- collapsible panels: click the ▾ arrow button to fold/unfold ---- */
.schead,.mapcard>h3,.instr>h3{user-select:none}
.caret{display:inline-flex;align-items:center;justify-content:center;cursor:pointer;
 width:24px;height:24px;margin-right:7px;border-radius:6px;border:1px solid #30363d;
 background:#0d1117;color:#8b98a5;font-size:13px;line-height:1;flex:none;vertical-align:middle}
.caret:hover{background:#1f6feb;border-color:#1f6feb;color:#fff}
.mapcard.collapsed>:not(h3),.instr.collapsed>:not(h3){display:none!important}
.statuscard.collapsed .scbody{display:none}
.collapse-all{width:100%;cursor:pointer;font-size:14px;font-weight:700;color:#c9d1d9;
 background:#161b22;border:1px solid #30363d;border-radius:8px;padding:8px;margin-bottom:2px}
.collapse-all:hover{background:#21262d;border-color:#3f6feb}
/* ---- REDESIGN: minimal translucent on-map status + collapsible right panel ---- */
.mapstatus{position:fixed;top:74px;right:8px;z-index:20;display:flex;flex-direction:column;gap:7px;
 padding:9px 12px;border-radius:13px;background:rgba(13,17,23,.58);backdrop-filter:blur(9px);
 -webkit-backdrop-filter:blur(9px);border:1px solid rgba(255,255,255,.08);max-width:calc(100vw - 16px)}
.msrow1{display:flex;align-items:center;gap:13px;flex-wrap:wrap}
.mstitle{font-weight:700;font-size:17px}
.msrow2{display:flex;align-items:center;gap:7px;flex-wrap:wrap}
.msi{display:inline-flex;align-items:center;gap:5px;font-size:15px;color:#c9d1d9;
 background:rgba(255,255,255,.06);padding:4px 10px;border-radius:9px;white-space:nowrap}
.msi b{font-weight:700;color:#e6e6e6}
.msi.mode b{font-size:17px}
/* on-map flight gauges (attitude + compass), bottom-right, same translucent AAVC tone */
.mapinst{position:fixed;right:12px;bottom:24px;z-index:20;display:flex;gap:16px;align-items:flex-start;
 padding:12px 16px;border-radius:14px;background:rgba(13,17,23,.58);backdrop-filter:blur(9px);
 -webkit-backdrop-filter:blur(9px);border:1px solid rgba(255,255,255,.08)}
/* 3D hexacopter attitude model — pure CSS 3D (no external lib, offline-safe) */
.stage{width:100%;height:150px;perspective:640px;display:flex;align-items:center;justify-content:center;margin:0 0 6px}
.d3cam{transform-style:preserve-3d;transform:rotateX(58deg)}
.d3att{position:relative;width:0;height:0;transform-style:preserve-3d;transition:transform .12s linear}
.d3hub{position:absolute;left:-16px;top:-16px;width:32px;height:32px;border-radius:8px;
 background:linear-gradient(145deg,#2c343f,#161b22);border:1px solid #3a4551;transform:translateZ(5px)}
.d3hub::after{content:"";position:absolute;left:50%;top:-4px;width:9px;height:13px;margin-left:-4.5px;
 background:#f0b90b;border-radius:2px;box-shadow:0 0 5px rgba(240,185,11,.6)}
.d3arm{position:absolute;left:0;top:0;height:5px;width:56px;margin-top:-2.5px;transform-origin:0 50%;
 background:linear-gradient(90deg,#3a434e,#28303a);border-radius:3px}
.d3rotor{position:absolute;left:-17px;top:-17px;width:34px;height:34px;border-radius:50%;
 border:2px solid #58a6ff;background:rgba(88,166,255,.07);transition:border-color .2s}
.d3rotor::before{content:"";position:absolute;inset:4px;border-radius:50%;
 border:1px dashed rgba(255,255,255,.30);animation:d3spin 1.05s linear infinite}
@keyframes d3spin{to{transform:rotate(360deg)}}
.attread{display:flex;justify-content:center;flex-wrap:wrap;gap:8px 14px;font-size:13px;color:#8b98a5;margin-bottom:6px}
.attread b{color:#e6e6e6;font-weight:700;font-family:ui-monospace,monospace}
.sidehead{display:flex;align-items:center;justify-content:space-between;font-size:15px;font-weight:700;
 color:#c9d1d9;padding:2px 2px 8px;border-bottom:1px solid #222b36;margin-bottom:2px}
.sidetoggle{cursor:pointer;width:28px;height:28px;border-radius:7px;border:1px solid #30363d;
 background:#0d1117;color:#8b98a5;font-size:15px;line-height:1;flex:none}
.sidetoggle:hover{background:#1f6feb;border-color:#1f6feb;color:#fff}
.colside{transition:transform .22s ease}
.sidetab{position:absolute;top:50%;left:360px;transform:translateY(-50%);z-index:26;cursor:pointer;
 width:20px;height:52px;display:flex;align-items:center;justify-content:center;
 background:#0d1117;color:#9aa7b4;border:1px solid rgba(255,255,255,.12);border-left:none;
 border-radius:0 9px 9px 0;font-size:13px;box-shadow:2px 0 7px rgba(0,0,0,.35);transition:left .2s ease}
.sidetab:hover{background:#1f6feb;color:#fff;border-color:#1f6feb}
body.side-collapsed .sidetab{left:0}
/* docked layout: map fills the left, control panel docked on the right */
html,body{height:100%}
body{display:flex;flex-direction:column;overflow:hidden}
#msgline{position:static}
.appwrap{flex:1;display:flex;min-height:0;position:relative}
#lmap{position:static;inset:auto;flex:1;width:auto;height:100%;display:block!important;z-index:0}
.colside{position:static;right:auto;top:auto;bottom:auto;width:360px;max-width:82vw;height:100%;
 overflow-y:auto;background:#0d1117;border-right:1px solid rgba(255,255,255,.09);order:-1;
 padding:12px;display:flex;flex-direction:column;gap:10px;z-index:auto;
 transition:width .2s ease,padding .2s ease}
body.side-collapsed .colside{width:0;padding:0;border-right:none;overflow:hidden;transform:none}
/* unified tone: cards + controls share the translucent on-map palette */
.mapcard,.instr{background:rgba(255,255,255,.045);border:1px solid rgba(255,255,255,.08)}
.mapcard>h3,.instr>h3{color:#9aa7b4}
.sidehead{border-bottom-color:rgba(255,255,255,.09)}
.caret{background:rgba(255,255,255,.05);border-color:rgba(255,255,255,.12)}
.padstatus{background:rgba(255,255,255,.04);border-color:rgba(255,255,255,.09)}
.pill{display:flex;flex-direction:column;background:#161b22;border:1px solid #222b36;border-radius:8px;padding:4px 9px;min-width:0;overflow:hidden}
.pill b{font-size:21px;font-weight:700;line-height:1.15;white-space:nowrap}
.pill.wide{grid-column:span 2}   /* Flight Mode: 2 cells wide so STABILIZED/OFFBOARD fit */
.pill span{font-size:12px;color:#8b98a5;text-transform:uppercase;letter-spacing:.03em}
.main{display:grid;grid-template-columns:1fr;gap:14px;padding:14px;max-width:1400px;margin:auto}
.colside{min-width:0;display:flex;flex-direction:column;gap:8px}
/* map = full-screen background; every control floats in ONE right sidebar */
.main{display:block;max-width:none;padding:0;margin:0}
.statuscard{position:fixed;top:74px;left:8px;z-index:20;background:#161b22;border:1px solid #222b36;border-radius:10px;padding:7px 10px;max-width:min(680px,calc(100vw - 16px))}
.schead{display:flex;align-items:center;gap:12px;flex-wrap:wrap;font-size:19px;font-weight:600;margin-bottom:7px}
#link{display:inline-flex;align-items:center;font-weight:700;font-size:18px}
/* (removed a stale position-fixed override on the message line — left over from the old
   fullscreen-map layout, it floated the message bar OVER the docked LEFT panel and covered
   the top of the sensor section. The message line stays static, per layoutTop() below which
   measures the real top-bar heights and positions the on-map status just under them.) */
/* (old floating right-sidebar rule removed — panel is docked LEFT now) */
.colside>.mapcard,.colside>.instr{background:rgba(20,25,32,.96)}
.leaflet-top.leaflet-left{top:56px}          /* push zoom buttons below the status bar */
.mapcard,.instr{background:#161b22;border:1px solid #222b36;border-radius:12px;padding:7px 10px}
.mapcard h3,.instr h3{margin:0 0 5px;font-size:14px;text-transform:uppercase;color:#8b98a5;letter-spacing:.05em}
.mapcard{display:flex;flex-direction:column}
#posmap{display:none}
.mapinfo{font-size:16px;color:#8b98a5;margin-top:8px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:8px}
.mapinfo b{color:#c9d1d9;font-family:ui-monospace,monospace}
.instr{text-align:center;align-self:start;padding:10px}
.instrtop{display:flex;align-items:center;justify-content:center;gap:14px;flex-wrap:wrap}
.instrread{text-align:left;min-width:76px}
.igauge{display:flex;flex-direction:column;align-items:center;gap:4px}
.glabel{font-size:12px;color:#8b98a5}.glabel b{font-size:16px;color:#e6e6e6}
/* heading compass (needle rotates to heading; N marked red) */
.compass{width:96px;height:96px;border-radius:50%;border:2px solid #4a5568;position:relative;background:#0d1117;flex:none}
.cdir{position:absolute;font-size:13px;font-weight:700;color:#8b98a5;transform:translate(-50%,-50%)}
.cn{left:50%;top:15px;color:#f85149}.cs{left:50%;top:calc(100% - 15px)}
.ce{left:calc(100% - 14px);top:50%}.cw{left:14px;top:50%}
.cneedle{position:absolute;left:50%;top:13px;bottom:13px;width:5px;margin-left:-2.5px;transform-origin:50% 50%;
 background:linear-gradient(#f85149 0 50%,#c9d1d9 50% 100%);border-radius:2px;transition:transform .15s linear}
.chub{position:absolute;left:50%;top:50%;width:13px;height:13px;margin:-6.5px 0 0 -6.5px;border-radius:50%;background:#e6e6e6;border:1px solid #0d1117}
.ai{width:96px;height:96px;border-radius:50%;overflow:hidden;position:relative;border:2px solid #4a5568;flex:none}
.ai-inner{position:absolute;left:-50%;top:-50%;width:200%;height:200%;transform-origin:50% 50%;background:linear-gradient(#3a86d6 0%,#3a86d6 50%,#8a6a3a 50%,#8a6a3a 100%)}
.ai-inner::after{content:"";position:absolute;left:0;right:0;top:50%;height:2px;background:rgba(255,255,255,.7)}
.ai-mark{position:absolute;left:50%;top:50%;width:42px;height:4px;background:#ffcc00;transform:translate(-50%,-50%);border-radius:2px}
.ai-mark-c{position:absolute;left:50%;top:50%;width:9px;height:9px;background:#ffcc00;border-radius:50%;transform:translate(-50%,-50%)}
.hdg{font-size:18px}.hdg b{font-size:25px}
.attxt{font-size:14px;color:#8b98a5;margin-top:3px}
.mlabel{font-size:13px;color:#8b98a5;margin:10px 0 8px;text-transform:uppercase;letter-spacing:.04em}
.motors{display:flex;flex-direction:column;gap:6px;padding:2px 0}
.mrow{display:flex;align-items:center;gap:9px;font-size:12px}
.mrow>span{width:26px;flex:none;color:#8b98a5;font-weight:600}
.mtrack{flex:1;height:10px;background:#21262d;border-radius:5px;overflow:hidden}
.mtrack>i{display:block;height:100%;width:0;background:#3fb950;border-radius:5px;transition:width .15s linear,background .15s}
.mrow>b{width:38px;flex:none;text-align:right;color:#c9d1d9;font-family:ui-monospace,monospace;font-weight:600}
#msgline{padding:10px 16px;font-family:ui-monospace,monospace;font-size:20px;color:#c9d1d9;
 background:#0b0e13;border-bottom:1px solid #222b36;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
/* (old fullscreen fixed map removed — map is a docked flex child now) */
.dicon{background:none;border:none}
.dm{color:#f0b90b;font-size:30px;line-height:1;text-align:center;transform-origin:50% 50%;text-shadow:0 0 3px #000}
.leaflet-container{background:#0d1117}
.sensors{display:flex;flex-wrap:wrap;gap:6px;justify-content:center;margin-top:8px}
.schip{display:flex;align-items:center;gap:5px;font-size:13px;padding:4px 8px;border-radius:8px;border:1px solid #30363d;background:#0d1117;color:#8b98a5}
.schip .sdot{height:9px;width:9px;border-radius:50%;background:#6e7681;flex:none}
.schip.sok{border-color:#238636;color:#7ee787}.schip.sok .sdot{background:#3fb950}
.schip.sbad{border-color:#8b2c22;color:#ff9a92}.schip.sbad .sdot{background:#f85149}
.schip.sna{color:#6e7681}.schip.sna .sdot{background:#6e7681}
.gfbar{display:flex;align-items:center;gap:10px;margin-top:8px;flex-wrap:wrap}
.gfbar>button{cursor:pointer;font-size:18px;font-weight:600;border-radius:8px;padding:7px 12px;border:1px solid #30363d;background:#21262d;color:#e6e6e6}
.gfnow{font-size:16px;color:#8b98a5}.gfnow b{color:#f0b90b;font-family:ui-monospace,monospace}
.gfpanel{display:none;margin-top:10px;padding:12px;background:#0d1117;border:1px solid #30363d;border-radius:10px}
.gfpanel.open{display:block}
.gfgrid{display:flex;gap:10px;flex-wrap:wrap}
.gfgrid label{display:flex;flex-direction:column;font-size:16px;color:#8b98a5;gap:4px;flex:1;min-width:130px}
.gfgrid input,.gfgrid select{background:#161b22;border:1px solid #30363d;color:#e6e6e6;border-radius:7px;padding:7px 9px;font-size:19px}
.gfrow{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap}
.gfrow>button{cursor:pointer;font-size:18px;font-weight:600;border-radius:8px;padding:8px 14px;border:1px solid #2f6feb;background:#1f6feb;color:#fff}
.gfrow>button.ghost{background:#21262d;border-color:#30363d;color:#e6e6e6}
.gfrow>button:disabled{opacity:.45;cursor:not-allowed;background:#161b22;border-color:#30363d}
.gfhint{font-size:15px;color:#6e7681;margin-top:9px;line-height:1.55}
.gfwarn{font-size:15px;color:#f0b90b;background:#2a220b;border:1px solid #6b5416;border-radius:8px;padding:8px 10px;line-height:1.6;margin-bottom:10px}
.gfwarn b{color:#f8d24a}
.gfsec{border-top:1px solid #21262d;padding-top:10px;margin-top:8px}
.gfsectitle{font-size:16px;color:#c9d1d9;font-weight:600;margin-bottom:8px}
.gfinline{flex-direction:row!important;align-items:center;gap:6px;min-width:auto!important;flex:none!important}
.fcorner{background:#3fb950;border:2px solid #fff;border-radius:3px;box-shadow:0 0 3px #000;cursor:move}
/* ---- AAVC pad selector + mission panel ---- */
.padgrid{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;margin:10px 0}
.padbtn{padding:8px 4px;border:2px solid #30363d;border-radius:9px;background:#0d1117;color:#c9d1d9;
 cursor:pointer;user-select:none;display:flex;flex-direction:column;align-items:center;gap:5px}
.padbtn.sel{border-color:#238636;background:#0f2417;color:#3fb950}
.padbtn .arucosvg{width:100%;height:auto;max-width:48px;border-radius:3px;display:block;background:#fff;padding:3px;box-sizing:border-box}
.padbtn .padid{font-weight:700;font-size:14px}
.padrow{display:flex;justify-content:space-between;align-items:center;margin-top:6px;font-size:16px;color:#8b98a5}
.savebtn{margin-top:8px;padding:8px;border:0;border-radius:8px;font-size:15px;font-weight:700;color:#fff;
 background:#1f6feb;cursor:pointer;width:100%}
.savebtn:disabled{opacity:.5;cursor:not-allowed}
/* ---- AAVC pad-picker popup ---- */
.padstatus{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:6px 10px;
 font-size:15px;color:#3fb950;font-weight:700;text-align:center;margin-bottom:5px}
.padstatus.none{color:#8b98a5;font-weight:400}
.padlive{margin:2px 0 4px;display:flex;flex-direction:column;gap:2px}
.padli{font-size:15px;font-weight:700;padding:1px 2px}
.padli.off{color:#8b98a5;font-weight:400}
.modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.62);z-index:1000;
 align-items:center;justify-content:center;padding:16px}
.modal.show{display:flex}
/* 🚀 pre-flight checklist + slide-to-confirm (operator 2026-08-19) */
.flyhint{font-size:12.5px;color:#8b98a5;margin:2px 0 10px;line-height:1.5}
.flychecks{margin-bottom:12px}
.flyrow{display:flex;align-items:center;gap:8px;padding:6px 8px;border-radius:7px;margin-bottom:4px;background:#0d1117;font-size:13.5px}
.flyrow .fmark{width:18px;height:18px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:12px;font-weight:700;flex:0 0 auto}
.flyrow.ok .fmark{background:#238636;color:#fff}
.flyrow.bad .fmark{background:#da3633;color:#fff}
.flyrow.na .fmark{background:#30363d;color:#8b98a5}
.flyrow .flabel{flex:1;color:#e6edf3}
.flyrow.bad .flabel{color:#ff7b72}
.flyrow .fdetail{color:#8b98a5;font-size:12px;text-align:right}
.slidewrap{position:relative;height:52px;border-radius:26px;background:#161b22;border:2px solid #30363d;overflow:hidden;user-select:none;touch-action:none;margin-bottom:8px}
.slidewrap.blocked{opacity:.5;pointer-events:none}
.slidefill{position:absolute;left:0;top:0;bottom:0;width:0;background:linear-gradient(90deg,#1f6feb,#238636)}
.slidelabel{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#c9d1d9;font-size:13.5px;font-weight:600;pointer-events:none}
.slidethumb{position:absolute;top:3px;left:3px;width:44px;height:44px;border-radius:50%;background:#238636;color:#fff;display:flex;align-items:center;justify-content:center;font-size:20px;cursor:grab;box-shadow:0 2px 6px rgba(0,0,0,.4);z-index:2}
.slidethumb:active{cursor:grabbing}
.flyprog{padding:10px;border-radius:8px;background:#0d1117;font-size:14px;color:#e6edf3;margin-bottom:8px}
.flyoffboard{padding:12px;border-radius:9px;font-size:14px;font-weight:600;margin-bottom:8px;text-align:center;line-height:1.55}
.flyoffboard.wait{background:#3a2d00;color:#e3b341;border:1px solid #9e6a03}
.flyoffboard.go{background:#0f5323;color:#3fb950;border:1px solid #238636;animation:stagedpulse 1.4s ease-in-out infinite}
.modalbox{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:18px;
 max-width:430px;width:100%;box-shadow:0 12px 40px rgba(0,0,0,.5)}
.modalbox h3{margin:0 0 12px;font-size:17px;color:#e6e6e6;text-transform:none;letter-spacing:0}
.modalbtns{display:flex;gap:8px;margin-top:12px}
.modalbtns>button{flex:1}
.modalcancel{padding:12px;border:1px solid #30363d;border-radius:8px;background:#21262d;
 color:#e6e6e6;font-size:17px;font-weight:700;cursor:pointer}
/* ---- AAVC payload servo release ---- */
.servobtns{display:flex;gap:8px;margin-top:4px}
.servobtns>button{flex:1;cursor:pointer;font-size:17px;font-weight:700;border-radius:8px;
 padding:11px;border:1px solid #a35a00;background:#c47500;color:#fff}
.servobtns>button.ghost{background:#21262d;border-color:#30363d;color:#e6e6e6}
.servobtns>button:disabled{opacity:.5;cursor:not-allowed}
/* small per-servo buttons — deliberately tiny so you don't fat-finger a drop */
.servomini{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:8px 0}
.servomini>button{padding:5px 2px;border:1px solid #30363d;border-radius:7px;background:#0d1117;
 color:#8b98a5;font-size:12px;line-height:1.35;cursor:pointer;text-align:center}
.servomini>button.rel{border-color:#e3a008;background:#2a1f05;color:#f0c040;font-weight:700}
.servomini>button:disabled{opacity:.45;cursor:not-allowed}
.servowarn{display:none;background:#3a1d1d;border:1px solid #f85149;color:#ffb3ae;
 padding:8px 10px;border-radius:8px;font-size:14px;margin-bottom:8px}
.servowarn.show{display:block}
.mprog .row{display:flex;justify-content:space-between;padding:3px 0;font-size:17px}
.mprog b{color:#58a6ff}
.mclock{font-family:ui-monospace,monospace;font-size:21px;font-weight:700}
.mprogwrap{border-top:1px solid #222b36;margin-top:8px;padding-top:6px}
.mproghead{font-size:12px;color:#8b98a5;text-transform:uppercase;letter-spacing:.04em;margin-bottom:3px}
.statuscard .mprog{max-width:520px}
.statuscard .mprog .row{font-size:15px;padding:2px 0}
.leg{font-size:11px;color:#8b98a5;margin-top:5px;display:flex;gap:8px;flex-wrap:wrap}
.leg i{width:11px;height:11px;border-radius:50%;display:inline-block;margin-right:4px;vertical-align:middle}
.padtip{background:transparent;border:0;box-shadow:none;color:#0d1117;font-weight:700;font-size:12px;text-shadow:0 0 2px #fff}
/* Landing-pad map marker (2026-08-12): drawn like the REAL pad — white square,
   black ring, ArUco id in the centre. Status (operator pick 2026-08-12,
   "เทาจาง → เขียว"): not-yet-dropped pads are MUTED GRAY (in-queue adds a
   dashed border); a delivered pad turns full colour — white face, green
   border, green ✓ badge. */
.padicon{background:transparent;border:0}
/* Fix 3 (2026-08-21): mission-plan waypoint numbers — where is it going NEXT */
.planicon{background:transparent;border:0}
.planseq{width:18px;height:18px;border-radius:50%;background:#a371f7;color:#fff;
 font-size:11px;font-weight:700;line-height:18px;text-align:center;
 box-shadow:0 1px 3px rgba(0,0,0,.5)}
.padbox{width:30px;height:30px;background:#fff;border:3px solid #3fb950;border-radius:5px;
 box-shadow:0 1px 5px rgba(0,0,0,.55);display:flex;align-items:center;justify-content:center;position:relative}
.padbox::before{content:'';position:absolute;inset:3px;border:2px solid #0d1117;border-radius:50%}
.padbox span{position:relative;font-weight:800;font-size:13px;color:#0d1117;line-height:1}
.padbox i{position:absolute;top:-7px;right:-7px;width:15px;height:15px;border-radius:50%;
 background:#3fb950;color:#fff;font-style:normal;font-size:10px;line-height:15px;text-align:center;
 box-shadow:0 1px 3px rgba(0,0,0,.5);display:none}
.padbox.done i{display:block}
.padbox.todo,.padbox.queue{background:#e4e6ea;border-color:#8b949e}
.padbox.todo::before,.padbox.queue::before{border-color:#8b949e}
.padbox.todo span,.padbox.queue span{color:#6e7681}
.padbox.queue{border-style:dashed}
/* identified (2026-08-21): id decoded but votes short — ORANGE middle state
   between gray todo/queue and green done (operator: ส้ม = identified) */
.padbox.ident{background:#fff7e0;border-color:#d29922}
.padbox.ident::before{border-color:#e3a008}
.padbox.ident span{color:#9a6700}
/* in-flight edit lock (2026-08-12): เลือก/แก้ไข + 🚀 go gray-out during a mission */
.savebtn.locked,.savebtn.locked:hover{background:#21262d!important;
 border-color:#30363d!important;color:#8b949e!important;cursor:not-allowed}
/* 🚀 upload feedback (2026-08-14): ⏳ waiting = amber; ✅ staged (drone
   reporting back = upload CONFIRMED end-to-end) = green pulse */
.savebtn.upwait,.savebtn.upwait:hover{background:#3a2d00!important;
 border:2px solid #d29922!important;color:#e3b341!important;cursor:progress}
.savebtn.staged,.savebtn.staged:hover{background:#0f5323!important;
 border:2px solid #3fb950!important;color:#e6ffe9!important;
 animation:stagedpulse 1.4s ease-in-out infinite}
@keyframes stagedpulse{0%,100%{box-shadow:0 0 0 0 rgba(63,185,80,.55)}
 50%{box-shadow:0 0 0 8px rgba(63,185,80,0)}}
/* A2 milestone %-bar (operator pick 2026-08-14) */
.psegrow{display:flex;gap:10px;align-items:center;margin:10px 0 2px}
.psegs{flex:1;display:flex;gap:3px}
.pseg{flex:1;height:13px;border-radius:4px;background:#21262d;position:relative}
.pseg>i{display:block;height:100%;width:0;background:#238636;border-radius:4px}
.pseg>b{position:absolute;top:16px;left:50%;transform:translateX(-50%);font-size:10px;color:#8b98a5;font-weight:400;white-space:nowrap}
.ppct{font-size:23px;font-weight:800;color:#3fb950;min-width:58px;text-align:right}
.pline{display:flex;justify-content:space-between;font-size:12.5px;margin-top:17px}
/* compact milestone chips (replaced the tall metro timeline, operator
   2026-08-14: "กินพื้นที่เยอะไป") — one wrapping row of pills */
.mchips{display:flex;flex-wrap:wrap;gap:4px;margin-top:10px}
.mchip{display:inline-flex;align-items:center;padding:2px 9px;border-radius:99px;
 font-size:11.5px;background:#21262d;color:#6e7681;border:1px solid #30363d;white-space:nowrap}
.mchip.ok{background:#12351d;color:#3fb950;border-color:#238636}
.mchip.cur{background:#0d1117;color:#58a6ff;border-color:#58a6ff;font-weight:700;
 box-shadow:0 0 6px rgba(88,166,255,.5)}
/* T1 toasts (NO sound — user request) */
#toasts{position:fixed;top:64px;right:12px;z-index:1200;width:232px}
.toast{background:#1c2129;border:1px solid #30363d;border-left:4px solid #3fb950;border-radius:8px;
 padding:8px 10px;font-size:13px;margin-bottom:7px;box-shadow:0 4px 14px rgba(0,0,0,.55);
 animation:toastin .25s ease}
.toast.warn{border-left-color:#d29922}
@keyframes toastin{from{transform:translateX(30px);opacity:0}to{transform:none;opacity:1}}
/* C1 post-flight summary modal */
#summodal{display:none;position:fixed;inset:0;background:rgba(1,4,9,.72);z-index:1300;
 align-items:center;justify-content:center}
#summodal.show{display:flex}
.sumcard{width:300px;background:#0d1117;border:2px solid #3fb950;border-radius:14px;
 padding:22px;text-align:center;box-shadow:0 10px 40px rgba(0,0,0,.7)}
.sumcard h3{margin:0 0 4px;font-size:20px;color:#3fb950}
.sumgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:14px 0}
.sumcell{background:#161b22;border-radius:8px;padding:9px 4px}
.sumcell b{display:block;font-size:19px;color:#e6edf3}
.sumcell span{font-size:11.5px;color:#8b98a5}
/* real-console mission card (operator 2026-08-14: restart-based switching is
   fine, but the console must SAY which mission is loaded, with graphics) */
.mreal{border:2px solid #d29922;background:#241c05;border-radius:10px;padding:9px 11px;cursor:pointer}
.mreal .t{font-size:14px;font-weight:800;color:#e3b341}
.mreal .s{font-size:11.5px;color:#c9b072;margin-top:2px;line-height:1.5}
.mreal .h{font-size:11px;color:#8b98a5;margin-top:5px;border-top:1px dashed #3d3212;padding-top:5px}
/* D field mode (sunlight) */
#fmbtn{position:fixed;right:12px;bottom:12px;z-index:1250;width:46px;height:46px;
 border-radius:50%;border:1px solid #30363d;background:#161b22;color:#e3b341;
 font-size:20px;cursor:pointer}
body.fieldmode{background:#000}
body.fieldmode .colside{width:430px;font-size:117%}
body.fieldmode .savebtn{font-size:19px;padding:13px;border-radius:10px}
body.fieldmode .schip{font-size:14px}
body.fieldmode #fmbtn{background:#238636;color:#fff;border-color:#3fb950}
/* ===== FLAT MINIMAL — no card boxes; one flowing panel, golden-ratio rhythm ===== */
.colside{width:360px;max-width:84vw;gap:0;padding:21px 21px 34px;
 background:#0d1117;border-right:1px solid rgba(255,255,255,.08)}
.sidehead{font-size:14px;font-weight:700;color:#c9d1d9;letter-spacing:.02em;
 padding:0 0 13px;margin:0;border-bottom:0}
/* each group becomes a frameless section, separated only by a hairline */
.colside>.mapcard,.colside>.instr{
 background:none!important;border:0!important;border-radius:0!important;
 border-top:1px solid rgba(255,255,255,.07)!important;
 padding:21px 0!important;margin:0}
.colside>.sidehead+.mapcard,.colside>.sidehead+.instr{border-top:0!important;padding-top:6px!important}
.padblock{margin-top:12px;padding-top:12px;border-top:1px solid rgba(255,255,255,.06)}
/* Mission = horizontal phase timeline (stepper) + stats */
.mtl{display:flex;align-items:flex-start;margin:2px 0}
.mtstep{flex:1;display:flex;flex-direction:column;align-items:center;position:relative;min-width:0}
.mtstep::before{content:"";position:absolute;top:9px;left:-50%;width:100%;height:2px;background:#30363d;z-index:0}
.mtstep:first-child::before{display:none}
.mtstep.done::before,.mtstep.cur::before{background:#3fb950}
.mtstep>i{width:18px;height:18px;border-radius:50%;background:#21262d;border:2px solid #30363d;z-index:1;
 display:flex;align-items:center;justify-content:center;font:700 11px/1 system-ui,sans-serif;font-style:normal;color:#8b98a5}
.mtstep.done>i{background:#3fb950;border-color:#3fb950;color:#0d1117}
.mtstep.cur>i{background:#0d1117;border-color:#58a6ff;box-shadow:0 0 0 3px rgba(88,166,255,.25)}
.mtstep>span{margin-top:6px;font-size:10px;color:#7d8b99;text-align:center;white-space:nowrap}
.mtstep.cur>span{color:#58a6ff;font-weight:700}
.mtstep.done>span{color:#c9d1d9}
.mtstats{display:flex;justify-content:center;gap:9px;flex-wrap:wrap;margin-top:10px;
 font-size:13px;color:#c9d1d9;font-family:ui-monospace,monospace}
.mtstats b{font-weight:700}
.mtstats .sep{color:#455060}
.mtwhy{margin-top:5px;padding:4px 8px;border:1px solid #9e6a03;border-radius:6px;background:rgba(187,128,9,.15);color:#e3b341;font-size:13px}
.mapcard>h3,.instr>h3{font-size:11px;letter-spacing:.10em;color:#7d8b99;
 margin:0 0 13px;font-weight:700;text-transform:uppercase;text-align:left}
.instr{text-align:center}
.instr>h3{text-align:left}
/* body rhythm */
.mprog .row{font-size:14px;padding:4px 0}
.mprog b{color:#58a6ff}
.padstatus{font-size:14px;padding:8px 11px;background:rgba(255,255,255,.04);
 border:1px solid rgba(255,255,255,.09);border-radius:8px;margin-bottom:8px}
.padli{font-size:13px}
.gfnow,.mapinfo{font-size:13px;color:#8b98a5}
.gfgrid label{font-size:13px;color:#8b98a5;gap:5px}
.gfgrid input,.gfgrid select{font-size:14px;padding:8px 10px;border-radius:8px}
.gfbar>button,.gfrow>button,.gfrow>button.ghost,.savebtn,
.servobtns>button,.servobtns>button.ghost{font-size:14px;border-radius:8px}
.gfrow>button{padding:9px 14px}
.gfhint,.gfwarn{font-size:12px;line-height:1.6}
.leg{font-size:11px}
.schip{font-size:12px}
.mlabel{font-size:11px;margin:12px 0 8px}
.caret{width:22px;height:22px;margin-right:8px;font-size:12px;
 background:rgba(255,255,255,.04);border-color:rgba(255,255,255,.10)}
.sidetab{left:360px}
</style></head><body>
<div id=msgline>—</div>
<div id=demobar style="display:none;background:#2a2140;color:#d0bcff;padding:8px 16px;text-align:center;font-size:16px;font-weight:700;border-bottom:1px solid #4a3a6a">🧪 DEMO — ข้อมูลตัวอย่าง ไม่ได้ต่อโดรนจริง · (เสียบ FMU/วิทยุ หรือรัน SITL เพื่อดูข้อมูลจริง)</div>
<div class=mapstatus>
 <div class=msrow1>
  <span class=mstitle>🛸 AAVC</span>
  <span id=link><span class=dot style=background:#6e7681></span>connecting…</span>
  <!-- mission clock REMOVED (operator 2026-08-18: "เดี๋ยวเขาจับให้เอง" —
       the committee keeps the official time; a second clock on the console
       is one more thing to argue with). The flight-side TimePolicy still
       enforces the window — this was display only. -->

 </div>
 <div class=msrow2>
  <span class="msi mode"><b id=mode>–</b></span>
  <span class=msi><b id=armstate>–</b></span>
  <span class=msi>🔋 <b id=batt>–</b> <b id=battpct>–</b></span>
  <span class=msi>📡 <b id=gpsfix>–</b> <b id=sats>–</b></span>
  <span class=msi>⬆ <b id=altv2>–</b></span>
 </div>
</div>
<div class=mapinst>
 <div class=igauge>
  <div class=ai id=ai><div class=ai-inner id=aiInner></div><div class=ai-mark></div><div class=ai-mark-c></div></div>
  <div class=glabel>ระดับ</div>
 </div>
 <div class=igauge>
  <div class=compass id=compass>
   <span class="cdir cn">N</span><span class="cdir ce">E</span><span class="cdir cs">S</span><span class="cdir cw">W</span>
   <div class=cneedle id=cneedle></div><div class=chub></div>
  </div>
  <div class=glabel>หัน <b id=hdgval>–</b>°<span id=hdgcard></span></div>
 </div>
</div>
<div class=appwrap>
 <div class=sidetab title="พับ/เปิดแผงควบคุม">◀</div>
 <div id=lmap></div>
 <div class=colside>
 <div class=sidehead><span>🛠 แผงควบคุม</span></div>
 <div class=instr>
  <h3>เครื่องวัดการบิน</h3>
  <div class=stage><div class=d3cam><div class=d3att id=d3att></div></div></div>
  <div class=attread>
   <span>roll <b id=rollv>–</b>°</span>
   <span>pitch <b id=pitchv>–</b>°</span>
   <span>หัน <b id=hdgv2>–</b>°</span>
   <span>สูง <b id=altv>–</b> m</span>
  </div>
  <div class=mlabel>ความเร็ว Motor (%)</div>
  <div class=motors id=motors></div>
  <div class=mlabel>สถานะ Sensor</div>
  <div class=sensors id=sensors></div>
 </div>
 <div class=mapcard>
  <h3>📋 Mission</h3>
  <div class=mprog id=mprog><span style=color:#8b98a5>idle — ไม่มี mission สด</span></div>
  <div id=pbarwrap style="display:none">
   <div class=psegrow>
    <div class=psegs id=psegs></div>
    <div class=ppct id=ppct>–</div>
   </div>
   <div class=pline><span id=plabel></span><span id=peta style="color:#8b98a5"></span></div>
  </div>
  <div id=tline class=mchips style="display:none"></div>
  <div class=padblock>
   <div id=miselrow style="display:none;margin-bottom:6px">
    <div style="font-size:12px;color:#8b98a5;margin-bottom:3px">🗂 Template mission — กดกล่องด้านล่างเพื่อสลับสนาม</div>
    <select id=misel onchange=selectMission() style="width:100%;background:#161b22;color:#e6edf3;border:2px solid #58a6ff;border-radius:8px;padding:8px;font-size:14px;font-weight:600;cursor:pointer"></select>
   </div>
   <div id=mrealcard style="display:none;margin-bottom:6px" onclick=realSwitchHelp()></div>
   <div class=padlive id=padlive></div>
   <div class=padstatus id=padstatus>— ยังไม่เลือก pad —</div>
   <button class=savebtn id=padopen type=button onclick=openPadModal()>✏️ เลือก / แก้ไข pad (ตั้งก่อนบิน offboard)</button>
   <button class=savebtn id=flybtn type=button onclick=startMission() style="display:none;margin-top:6px">🚀 up ขึ้นโดรน</button>
   <button class=savebtn id=resetbtn type=button onclick=resetField() style="display:none;margin-top:6px">🧹 รีเซ็ตสนาม [SIM]</button>
   <div id=savemsg style="font-size:13px;color:#8b98a5;margin-top:6px"></div>
  </div>
 </div>
 <div class=modal id=flymodal>
  <div class=modalbox>
   <h3>🚀 พร้อม up ขึ้นโดรน?</h3>
   <div class=flyhint>ตรวจความพร้อมก่อน แล้ว<b>สไลด์เพื่อยืนยัน</b>อัพโหลด mission — REAL: หลังอัพโหลด นักบิน arm RC + สลับ OFFBOARD เพื่อปล่อยบิน</div>
   <div id=flychecks class=flychecks></div>
   <div id=flyslide class=slidewrap>
    <div id=flyslidefill class=slidefill></div>
    <div class=slidelabel id=flyslidelabel>สไลด์ขวาเพื่อยืนยัน →</div>
    <div id=flyslidethumb class=slidethumb>🚀</div>
   </div>
   <div id=flyprog class=flyprog style="display:none"></div>
   <div id=flyoffboard class=flyoffboard style="display:none"></div>
   <div class=modalbtns>
    <button class=modalcancel type=button onclick=closeFlyModal()>ปิด</button>
   </div>
  </div>
 </div>
 <div class=modal id=padmodal>
  <div class=modalbox>
   <h3>🎯 เลือก Pad ที่จะส่ง (ID 1-6 · เลือกกี่อันก็ได้)</h3>
   <div class=padgrid id=padgrid></div>
   <div class=padrow><span>เลือกแล้ว: <b id=selcount style=color:#3fb950>0</b> / 6</span></div>
   <div class=modalbtns>
    <button class=savebtn id=savebtn type=button onclick=savePadModal()>💾 บันทึก</button>
    <button class=modalcancel type=button onclick=closePadModal()>ยกเลิก</button>
   </div>
  </div>
 </div>
 <div class=mapcard id=servocard>
  <h3>📦 ปล่อย Payload (Servo)</h3>
  <div id=servowarn class=servowarn></div>
  <div class=servobtns>
   <button id=servoall type=button onclick=releaseServo(null)>📦 ปล่อยทั้งหมด</button>
   <button id=servocloseall type=button class=ghost onclick=resetServo(null)>❌ ปิดทั้งหมด</button>
  </div>
  <div class=servomini id=servogrid></div>
  <div id=servomsg style="font-size:15px;color:#8b98a5;margin-top:8px"></div>
  <div class=leg><span>ปุ่มเล็ก = ปล่อยทีละตัว (กันกดผิด) · ต้องเลือก pad ก่อน</span></div>
 </div>
 <div class=mapcard>
  <h3>🛡️ Geofence · ตำแหน่ง</h3>
  <canvas id=posmap width=680 height=460></canvas>
  <div class=mapinfo><span>ตำแหน่ง: <b id=posll>–</b></span><span id=mapscale></span></div>
  <div class=gfbar>
   <button id=gfbtn type=button>🛡️ Geofence</button>
   <button id=originbtn type=button title="ส่ง SET_GPS_GLOBAL_ORIGIN — ให้ NED (0,0) = ตำแหน่งปัจจุบัน">📍 ตั้ง 0,0 ที่นี่</button>
   <span class=gfnow>ปัจจุบัน: <b id=gfnow>–</b></span>
  </div>
  <div id=gfpanel class=gfpanel>
   <div class=gfwarn>
    <b>⚠️ เลือกให้ถูกงาน:</b> <b>วงกลม</b> = chirp/ใกล้บ้าน · <b>สี่เหลี่ยม</b> = offboard/ทั่วสนาม ·
    ตั้ง <b>เผื่อ margin</b> เกินเส้นทาง (ไม่งั้น chirp drift ทริกเกอร์เอง)<br>
    💡 <b>วิธี upload ให้ชัวร์ = ผ่าน USB:</b> ① <b>ต่อสาย FMU FC-USB เข้าคอมก่อน</b> →
    ② กด <b>Upload ขึ้น FMU</b> (เร็ว+ชัวร์ ไม่ผ่านวิทยุที่ช้า/หลุด) →
    ③ fence เก็บ <b>ถาวรบน FMU</b> ถอด USB แล้วยังอยู่ตอนบิน (ทำบนโต๊ะ props off ได้ ไม่ต้อง GPS)<br>
    (จะ upload ผ่านวิทยุ NOMAD ก็ได้ แต่ <b>ช้า/อาจต้องกดซ้ำ</b>)
   </div>

   <div class=gfsectitle>⚪ วงกลม (รัศมี + เพดาน · ผ่าน params)</div>
   <div class=gfgrid>
    <label>รัศมี (m)<input id=gfrad type=number min=0 step=5 value=50></label>
    <label>เพดานสูง (m)<input id=gfalt type=number min=0 step=5 value=30></label>
    <label>เกินเขตแล้ว<select id=gfact>
     <option value=1>เตือนเฉยๆ (Warning)</option>
     <option value=2>ลอยค้าง (Hold)</option>
     <option value=3 selected>บินกลับบ้าน (RTL)</option>
     <option value=5>ลงจอด (Land)</option>
     <option value=0>ปิด geofence</option>
    </select></label>
   </div>
   <div class=gfrow>
    <button id=gfapply type=button>✅ ตั้งวงกลม</button>
    <button id=gfclrc type=button class=ghost>🗑️ ล้าง</button>
    <button id=gfread type=button class=ghost>🔄 อ่านค่า</button>
   </div>

   <div class=gfsec>
    <div class=gfsectitle>▭ สี่เหลี่ยม (polygon · upload เข้า FMU)</div>
    <div class=gfgrid>
     <label>เพดานสูง (m)<input id=gfalt2 type=number min=0 step=5 value=30></label>
     <label>เกินเขตแล้ว<select id=gfact2>
      <option value=1>เตือนเฉยๆ (Warning)</option>
      <option value=2>ลอยค้าง (Hold)</option>
      <option value=3 selected>บินกลับบ้าน (RTL)</option>
      <option value=5>ลงจอด (Land)</option>
     </select></label>
    </div>
    <div class=gfrow>
     <button id=gfdraw type=button>✏️ วาดสี่เหลี่ยม</button>
     <button id=gfup type=button>⬆️ Upload สี่เหลี่ยม</button>
     <button id=gfclr type=button class=ghost>🗑️ ล้าง</button>
    </div>
    <div class=gfhint>ลาก <b>มุมทั้ง 4</b> บนแผนที่ปรับรูป → กด Upload (เขตเขียว = อยู่ใน). <b>ต่อ FMU USB ก่อน Upload = ชัวร์สุด</b> (ผ่านวิทยุอาจต้องกดซ้ำ) · เก็บถาวรบน FMU ถอดสายได้.</div>
   </div>
  </div>
 </div>
 </div>
</div>
<div id=toasts></div>
<div id=summodal><div class=sumcard id=sumcardc></div></div>
<button id=fmbtn onclick=fmToggle() title="โหมดจอสนาม (กลางแดด)">🔆</button>
<script>
// on-screen JS error banner (debug aid 2026-08-13): a runtime error anywhere
// kills the rest of tick() silently — surface it so operator+dev see the SAME
// thing instead of "หน้าเว็บเหมือนไม่เปลี่ยน/เพี้ยน" mysteries.
window.onerror=function(m,src,l,c){try{
 var el=document.getElementById('jserr');
 if(!el){el=document.createElement('div');el.id='jserr';
  el.style.cssText='position:fixed;bottom:6px;left:6px;right:6px;background:#f85149;color:#fff;padding:6px 10px;z-index:9999;font:12px monospace;border-radius:8px;white-space:pre-wrap';
  document.body.appendChild(el);}
 el.textContent='⚠ JS ERROR: '+m+'  (บรรทัด '+l+':'+c+') — แคปหน้าจอนี้ส่งให้ Claude ได้เลย';
}catch(_){}};
function card16(h){var d=['N','NE','E','SE','S','SW','W','NW'];return d[Math.round(h/45)%8];}
// stack the fixed top bars (status -> msgline) and start the sidebar below them, measuring real heights
function layoutTop(){
 var aw=document.querySelector('.appwrap'),ms=document.querySelector('.mapstatus');
 var top=aw?Math.max(0,aw.getBoundingClientRect().top):48;
 if(ms)ms.style.top=(top+8)+'px';
}
window.addEventListener('resize',layoutTop);
window.addEventListener('load',layoutTop);
// collapsible panels: a ▾ arrow button on each header folds/unfolds it; state saved in localStorage
var _allBtnRefresh=null;
function makePanel(panel,header,key){
 var caret=document.createElement('span');caret.className='caret';caret.title='พับ/ขยาย';
 var on=false;try{on=localStorage.getItem('col_'+key)==='1';}catch(e){}
 function paint(){panel.classList.toggle('collapsed',on);caret.textContent=on?'▸':'▾';}
 function set(v){on=v;paint();try{localStorage.setItem('col_'+key,on?'1':'0');}catch(e){}
  try{layoutTop()}catch(e){}if(_allBtnRefresh)_allBtnRefresh();}
 paint();
 caret.addEventListener('click',function(e){e.stopPropagation();set(!on);});
 header.insertBefore(caret,header.firstChild);
 panel._set=set;panel._isOn=function(){return on;};
}
function initCollapse(){
 var i=0;
 document.querySelectorAll('.colside .mapcard, .colside .instr').forEach(function(card){
  var h=card.querySelector('h3');if(!h)return;makePanel(card,h,'card'+(i++));});
 // Google-Maps-style edge tab: click to collapse/expand the left panel
 var tab=document.querySelector('.sidetab');
 function setSide(on){document.body.classList.toggle('side-collapsed',on);
  if(tab)tab.textContent=on?'▶':'◀';
  try{localStorage.setItem('side_collapsed',on?'1':'0');}catch(e){}
  try{layoutTop()}catch(e){}
  setTimeout(function(){try{if(lmap)lmap.invalidateSize()}catch(e){}},240);}
 if(tab)tab.addEventListener('click',function(){setSide(!document.body.classList.contains('side-collapsed'));});
 var sv='0';try{sv=localStorage.getItem('side_collapsed')||'0';}catch(e){}
 setSide(sv==='1');
}
window.addEventListener('load',initCollapse);
var mapHome=null,mapTrack=[];
var lmap=null,lmarker=null,ltrack=null,lhome=null,lgfcircle=null,gfWant=null,mapReady=false;
var SEL=[],PADS_INIT=false,lpads={},lzones=[],lhomeMarker=null,MCLOCK={base:null,at:0,done:false};
var FT={start:null,frozen:null};   // mission budget clock: starts on ARMED+OFFBOARD (telemetry), not the mission file
var frect=null,fmarkers=[],fcorners=[],lupfence=null;
var lplan=[];   // Fix 3: mission-plan polyline + numbered stop markers
var SLAB={gyro:'Gyro',accel:'Accel',mag:'Mag',baro:'Baro',gps:'GPS',rc:'RC',ahrs:'AHRS',battery:'Batt',lidar:'Lidar',cam:'กล้องล่าง',radio:'วิทยุ',cm4:'CM4'};
var SORD=['gyro','accel','mag','baro','gps','rc','ahrs','battery','lidar','cam','radio','cm4'];
function droneIcon(){return L.divIcon({className:'dicon',html:'<div class=dm>▲</div>',iconSize:[26,26],iconAnchor:[13,13]});}
function initMap(){
 if(mapReady||typeof L==='undefined')return;
 var el=document.getElementById('lmap');if(!el)return;
 var cv=document.getElementById('posmap');if(cv)cv.style.display='none';
 el.style.display='block';
 lmap=L.map(el,{zoomControl:true,attributionControl:true}).setView([13.736,100.523],16);
 L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'© OpenStreetMap'}).addTo(lmap);
 ltrack=L.polyline([],{color:'#388bfd',weight:3}).addTo(lmap);
 [60,250,600,1200,2500].forEach(function(t){setTimeout(function(){if(lmap)lmap.invalidateSize()},t)});
 window.addEventListener('resize',function(){if(lmap)lmap.invalidateSize()});
 mapReady=true;
}
function renderSensors(s){
 var box=document.getElementById('sensors');if(!box)return;
 var pres=s.present||{},hl=s.health||{},g=s.gps||{},html='';
 for(var i=0;i<SORD.length;i++){var k=SORD[i],p=pres[k],h=hl[k],cls,txt;
  if(k==='lidar'){var la=s.lidar_age;
   /* 10 s = 10x the 1 Hz request: the NOMAD air link delivers in bursts
      (measured gaps up to ~6.3 s with the link healthy), so 5 s flapped */
   if(la!=null&&la<10){cls='sok';txt='OK';}else if(la!=null){cls='sbad';txt='ค้าง '+Math.round(la)+'s';}else{cls='sna';txt='n/a';}}
  else if(k==='cam'){var cr=s.cam_radio,ca=s.cam_age;
   if(cr&&cr.state==='OK'){cls='sok';txt='OK 📻';}
   else if(cr){cls='sbad';txt=(cr.state==='DEAD'?'ค้าง':'ไม่มีเฟรม')+' 📻';}
   else if(ca!=null&&ca<5){cls='sok';txt='OK';}else if(ca!=null){cls='sbad';txt='ค้าง '+Math.round(ca)+'s';}else{cls='sna';txt='n/a';}}
  else if(k==='radio'){
   if(s.link_kind==='radio'){if(s.link){cls='sok';txt='OK';}else{cls='sbad';txt='หลุด';}}
   else{cls='sna';txt='n/a (UDP)';}}
  else if(k==='cm4'){
   if(s.cm4_ok===true){cls='sok';txt='OK';}else if(s.cm4_ok===false){cls='sbad';txt='หลุด';}else{cls='sna';txt='n/a (SIM)';}}
  else if(k==='gps'){var fx=g.fix||0,hd=g.hdop,pa=s.gps_prearm||{};
   /* MATCH PX4 arm-readiness, not just fix: RED when PX4 has a fresh GPS
      prearm-fail ("PDOP too high") or the fix/hdop is not arm-grade, so the GCS
      GPS status stops showing green while the FC refuses to arm on GPS. */
   var preFail=(pa.ok===false&&pa.age!=null&&pa.age<15);
   if(fx<2){cls='sbad';txt='no fix';}
   else if(preFail){cls='sbad';txt='PDOP สูง';}
   else if(fx>=3&&(hd==null||hd<=2.0)){cls='sok';txt='3D'+(hd!=null?' · '+hd:'');}
   else{cls='sbad';txt=(fx>=3?'3D · hdop '+hd:'2D');}}
  else if(h){cls='sok';txt='OK';}          // health-first: AHRS is present=False/health=True on PX4
  else if(p){cls='sbad';txt='BAD';}
  else{cls='sna';txt='n/a';}
  html+='<div class="schip '+cls+'" title="'+SLAB[k]+': '+txt+'"><span class=sdot></span>'+SLAB[k]+'</div>';
 }
 // Geofence containment: is the drone INSIDE the fence right now? Uses the
 // MISSION geofence (controlled_airspace from the field yaml, always loaded —
 // s.zones.airspace) so it lights up from a GPS fix alone, with NO 🛡️ upload
 // and NO 🚀; falls back to a session-uploaded polygon if one exists. Separate
 // from the FC's own fence-breach failsafe (operator 2026-08-19).
 var fp=fenceRef(s), gg=s.gps||{};
 if(fp.length>=3&&gg.lat!=null&&gg.lon!=null&&gg.fix>=2){
  var inside=pointInPoly(gg.lat,gg.lon,fp);
  html+='<div class="schip '+(inside?'sok':'sbad')+'" title="geofence (mission): '+(inside?'โดรนอยู่ในรั้ว':'โดรนอยู่นอกรั้ว!')+'"><span class=sdot></span>'+(inside?'ในรั้ว':'นอกรั้ว!')+'</div>';
 }else if(fp.length>=3){
  html+='<div class="schip sna" title="geofence (mission): มีรั้วแล้ว รอ GPS fix"><span class=sdot></span>รั้ว · รอ GPS</div>';
 }else{
  html+='<div class="schip sna" title="geofence: ไม่มีรั้วใน mission config"><span class=sdot></span>รั้ว ?</div>';
 }
 box.innerHTML=html;
}
// fence reference polygon: prefer the MISSION geofence (field yaml
// controlled_airspace, always present), else a polygon uploaded this session.
function fenceRef(s){var z=s.zones||{};
 return (z.airspace&&z.airspace.length>=3)?z.airspace:((s.fence&&s.fence.pts)||[]);}
// point-in-polygon (ray casting); pts = [[lat,lon],…]
function pointInPoly(lat,lon,pts){var inside=false,n=pts.length;
 for(var i=0,j=n-1;i<n;j=i++){var yi=pts[i][0],xi=pts[i][1],yj=pts[j][0],xj=pts[j][1];
  if(((yi>lat)!=(yj>lat))&&(lon<(xj-xi)*(lat-yi)/(yj-yi)+xi))inside=!inside;}
 return inside;}
function updateMap(s){
 var g=s.gps||{}, ll=document.getElementById('posll');
 var msc=document.getElementById('mapscale'); if(msc) msc.textContent='แผนที่ OSM (ติดตามสด)';
 // Field data FIRST, vehicle data after the fix gate (sibling-session bug
 // report 2026-08-14, hit live: with no telemetry the zones never drew and
 // the template-switch pan silently no-op'd — zones/home/pads are FIELD
 // data and must render on a pre-connection console too).
 aavcMap(s);
 if(g.lat==null||g.lon==null||!(g.fix>=2)){if(ll)ll.textContent='ยังไม่มีตำแหน่ง GPS (sats '+(g.sats==null?0:g.sats)+')';return;}
 var pos=[g.lat,g.lon];
 if(!lhome){lhome=pos;lmap.setView(pos,18);}
 if(!lmarker)lmarker=L.marker(pos,{icon:droneIcon()}).addTo(lmap);else lmarker.setLatLng(pos);
 var hd=(s.att&&s.att.heading!=null)?s.att.heading:0;
 if(lmarker._icon){var ar=lmarker._icon.querySelector('.dm');if(ar)ar.style.transform='rotate('+hd+'deg)';}
 var pts=ltrack.getLatLngs(),lp=pts[pts.length-1];
 if(!lp||lmap.distance(lp,pos)>0.3){ltrack.addLatLng(pos);var all=ltrack.getLatLngs();if(all.length>1000)ltrack.setLatLngs(all.slice(-1000));}
 var gf=s.geofence||{};
 var _cr=(gf.hor&&gf.hor>0)?gf.hor:(gfWant?gfWant.r:0), _cc=lhome||(gfWant?gfWant.ctr:null);
 if(_cr>0&&_cc){if(!lgfcircle){lgfcircle=L.circle(_cc,{radius:_cr,color:'#f0b90b',weight:2,fill:false,dashArray:'6 6'}).addTo(lmap);}else{lgfcircle.setLatLng(_cc);lgfcircle.setRadius(_cr);}}
 else if(lgfcircle){lmap.removeLayer(lgfcircle);lgfcircle=null;}
 var fp=(s.fence&&s.fence.pts)||[];
 if(fp.length>=3){if(!lupfence){lupfence=L.polygon(fp,{color:'#3fb950',weight:2,fill:false}).addTo(lmap);}else{lupfence.setLatLngs(fp);}}
 else if(lupfence){lmap.removeLayer(lupfence);lupfence=null;}
 if(ll)ll.textContent=g.lat.toFixed(6)+', '+g.lon.toFixed(6);
}
// build the CSS-3D hexacopter once: central hub + 6 arms + 6 rotors around a hexagon
function build3d(){
 var att=document.getElementById('d3att');if(!att||att.dataset.built)return;
 var A=[0,60,120,180,240,300],h='<div class=d3hub></div>';
 for(var i=0;i<6;i++)h+='<div class=d3arm style="transform:rotate('+A[i]+'deg)"></div>';
 for(var i=0;i<6;i++)h+='<div class=d3rotor style="transform:rotate('+A[i]+'deg) translateX(52px) translateZ(7px)"></div>';
 att.innerHTML=h;att.dataset.built='1';
}
function renderInsti(s){
 var a=s.att||{};
 var roll=a.roll==null?0:a.roll, pitch=a.pitch==null?0:a.pitch;
 var inner=document.getElementById('aiInner');
 if(inner) inner.style.transform='rotate('+(-roll)+'deg) translateY('+(pitch*2.7)+'px)';
 var hv=document.getElementById('hdgval'); if(hv) hv.textContent=a.heading==null?'–':Math.round(a.heading);
 var hc=document.getElementById('hdgcard'); if(hc) hc.textContent=a.heading==null?'':(' '+card16(a.heading));
 var cnl=document.getElementById('cneedle'); if(cnl) cnl.style.transform='rotate('+(a.heading==null?0:a.heading)+'deg)';
 var rv=document.getElementById('rollv'); if(rv) rv.textContent=a.roll==null?'–':a.roll;
 var pv=document.getElementById('pitchv'); if(pv) pv.textContent=a.pitch==null?'–':a.pitch;
 var hv2=document.getElementById('hdgv2'); if(hv2) hv2.textContent=a.heading==null?'–':Math.round(a.heading);
 var alt=(s.gps&&s.gps.rel_alt!=null)?s.gps.rel_alt:((s.local&&s.local.z!=null)?-s.local.z:null);
 var av=document.getElementById('altv'); if(av) av.textContent=alt==null?'–':alt.toFixed(1);
 // main status bar: AGL big + MSL beside it — MSL is what exposes a GPS altitude
 // frame that moved after an FC reboot (2026-08-20: a stale frame flew the
 // transit 8.5 m AGL command into the ground)
 var av2=document.getElementById('altv2'); if(av2){var amsl=(s.gps&&s.gps.alt!=null)?s.gps.alt:null;av2.textContent=(alt==null?'–':alt.toFixed(1)+' m')+(amsl==null?'':' · MSL '+amsl.toFixed(0));}
 // 3D hexacopter: tilt to live attitude (roll/pitch/yaw) + tint rotors by motor throttle
 build3d();
 var d3=document.getElementById('d3att');
 if(d3) d3.style.transform='rotateY('+roll+'deg) rotateX('+(-pitch)+'deg) rotateZ('+(-(a.heading==null?0:a.heading))+'deg)';
 var _rr=document.querySelectorAll('#d3att .d3rotor'),_ms=s.motors||[];
 for(var _i=0;_i<_rr.length;_i++){var _p=Math.round((_ms[_i]||0)*100);
  _rr[_i].style.borderColor=_p>85?'#f85149':_p>60?'#d29922':'#3fb950';}
 var box=document.getElementById('motors');
 if(box){var ms=s.motors||[];
  if(box.children.length!==ms.length){box.innerHTML='';for(var i=0;i<ms.length;i++){var d=document.createElement('div');d.className='mrow';d.innerHTML='<span>M'+(i+1)+'</span><div class=mtrack><i></i></div><b>–</b>';box.appendChild(d);}}
  for(var j=0;j<ms.length;j++){var pct=Math.round(ms[j]*100);var row=box.children[j];var fill=row.querySelector('.mtrack>i');fill.style.width=pct+'%';fill.style.background=pct>85?'#f85149':pct>60?'#d29922':'#3fb950';row.querySelector('b').textContent=pct+'%';row.title='M'+(j+1)+': '+pct+'%';}}
 renderSensors(s);
 drawMap(s);
}
function drawMap(s){
 initMap();
 if(mapReady){updateMap(s);return;}
 var cv=document.getElementById('posmap'); if(!cv||!cv.getContext) return;
 var ctx=cv.getContext('2d'), W=cv.width, H=cv.height; ctx.clearRect(0,0,W,H);
 ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
 for(var x=0;x<=W;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}
 for(var y=0;y<=H;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
 var g=s.gps||{}, ll=document.getElementById('posll');
 if(g.lat==null||g.lon==null||!(g.fix>=2)){if(ll)ll.textContent='ยังไม่มีตำแหน่ง GPS (sats '+(g.sats==null?0:g.sats)+')';return;}
 if(!mapHome)mapHome={lat:g.lat,lon:g.lon};
 var dx=(g.lon-mapHome.lon)*Math.cos(mapHome.lat*Math.PI/180)*111320;
 var dy=(g.lat-mapHome.lat)*111320;
 var last=mapTrack[mapTrack.length-1];
 if(!last||Math.hypot(dx-last.x,dy-last.y)>0.3)mapTrack.push({x:dx,y:dy});
 if(mapTrack.length>800)mapTrack.shift();
 var maxr=3,i;for(i=0;i<mapTrack.length;i++){maxr=Math.max(maxr,Math.abs(mapTrack[i].x),Math.abs(mapTrack[i].y));}
 maxr=Math.max(maxr,Math.abs(dx),Math.abs(dy))*1.25;
 var sc=Math.min(W,H)/(2*maxr), cxp=W/2, cyp=H/2;
 function P(px,py){return {x:cxp+px*sc,y:cyp-py*sc};}
 var hp=P(0,0); ctx.fillStyle='#8b98a5'; ctx.beginPath(); ctx.arc(hp.x,hp.y,5,0,7); ctx.fill();
 ctx.fillStyle='#8b98a5';ctx.font='11px sans-serif';ctx.fillText('จุดเริ่ม',hp.x+8,hp.y+4);
 ctx.strokeStyle='#388bfd'; ctx.lineWidth=2; ctx.beginPath();
 for(i=0;i<mapTrack.length;i++){var p=P(mapTrack[i].x,mapTrack[i].y);if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y);}
 ctx.stroke();
 var cp=P(dx,dy), hd=a2(s)!=null?a2(s):0;
 ctx.save(); ctx.translate(cp.x,cp.y); ctx.rotate(hd*Math.PI/180);
 ctx.fillStyle='#f0b90b'; ctx.beginPath(); ctx.moveTo(0,-13); ctx.lineTo(8,9); ctx.lineTo(0,4); ctx.lineTo(-8,9); ctx.closePath(); ctx.fill(); ctx.restore();
 if(ll)ll.textContent=g.lat.toFixed(6)+', '+g.lon.toFixed(6);
 var ms=document.getElementById('mapscale'); if(ms)ms.textContent='กริดครึ่งด้าน ~'+Math.round(maxr)+' m';
}
function a2(s){return (s.att&&s.att.heading!=null)?s.att.heading:null;}
// ---------------- AAVC: pad selector + mission panel + pads on the map ----------------
// ArUco DICT_4X4_50 glyphs (6x6 modules, row-major, "1"=white "0"=black), baked
// from cv2.aruco so the operator matches the committee's assignment card
// picture-to-picture instead of translating it into a number.
var ARUCO={
 1:"000000000000011110010010010100000000",
 2:"000000000110000110000100011010000000",
 3:"000000010010010010001000001100000000",
 4:"000000001010001000010010011100000000",
 5:"000000001110010010011000011010000000",
 6:"000000010010011100000100011100000000"};
function arucoSVG(id){
 var b=ARUCO[id]||'',n=6,c=6,s=n*c,r='';
 for(var i=0;i<n*n;i++){ if(b.charAt(i)==='0'){ r+='<rect x="'+((i%n)*c)+'" y="'+(Math.floor(i/n)*c)+'" width="'+c+'" height="'+c+'"/>'; } }
 return '<svg class="arucosvg" viewBox="0 0 '+s+' '+s+'" xmlns="http://www.w3.org/2000/svg"><rect width="'+s+'" height="'+s+'" fill="#fff"/><g fill="#000">'+r+'</g></svg>';
}
function renderPads(){
 var g=document.getElementById('padgrid');if(!g)return;g.innerHTML='';
 [1,2,3,4,5,6].forEach(function(id){
  var d=document.createElement('div');
  d.className='padbtn'+(SEL.indexOf(id)>=0?' sel':'');
  d.innerHTML=arucoSVG(id)+'<div class="padid">ID '+id+'</div>';
  d.onclick=function(){var i=SEL.indexOf(id);if(i>=0)SEL.splice(i,1);else SEL.push(id);
   SEL.sort(function(a,b){return a-b});renderPads();updateMap(window.LAST||{});};
  g.appendChild(d);
 });
 var sc=document.getElementById('selcount');if(sc)sc.textContent=SEL.length;
}
function openPadModal(){
 window.SEL_BACKUP=SEL.slice();                     // remember, so ยกเลิก can revert
 var m=document.getElementById('padmodal');if(m)m.className='modal show';
 renderPads();
}
function closePadModal(){
 SEL=(window.SEL_BACKUP||SEL).slice();               // discard un-saved edits
 var m=document.getElementById('padmodal');if(m)m.className='modal';
 renderPads();updateMap(window.LAST||{});
}
async function savePadModal(){
 var b=document.getElementById('savebtn'),m=document.getElementById('savemsg');if(b)b.disabled=true;
 try{var r=await fetch('/api/assign',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ids:SEL})});
  var j=await r.json();
  if(j.ok){window.SEL_BACKUP=SEL.slice();
   var pm=document.getElementById('padmodal');if(pm)pm.className='modal';   // close popup
   if(m)m.textContent='✅ บันทึกแล้ว: ['+j.ids.join(', ')+'] — mission จะใช้ค่านี้ตอนเริ่ม';}
  else if(m){m.textContent='❌ '+(j.err||'error');}}
 catch(e){if(m)m.textContent='❌ '+e;}
 if(b)b.disabled=false;
}
function renderPadStatus(s){
 var el=document.getElementById('padstatus');if(!el)return;
 if(s.pads_selected&&(s.assignment||[]).length){
  el.className='padstatus';el.textContent='🎯 จะส่ง '+s.assignment.length+' pad: '+s.assignment.join(', ');}
 else{el.className='padstatus none';el.textContent='— ยังไม่เลือก pad —';}
}
// 🚀 GO (2026-08-12; checklist+slide 2026-08-19): open the pre-flight checklist,
// slide-to-confirm the upload, then guide the RC arm -> OFFBOARD hand-off.
var FLY={busy:false,stagedShown:false};
function startMission(){ openFlyModal(); }
function openFlyModal(){
 var s=window.LAST||{},ids=s.assignment||[];
 if(!s.pads_selected||!ids.length){alert('⚠️ เลือก + บันทึก pad ก่อนครับ');openPadModal();return;}
 FLY.busy=false;FLY.stagedShown=false;window.UPWAIT=null;
 var m=document.getElementById('flymodal');if(!m)return;
 document.getElementById('flyprog').style.display='none';
 document.getElementById('flyoffboard').style.display='none';
 document.getElementById('flyslide').style.display='block';
 slideReset();buildFlyChecks();
 m.className='modal show';
}
function closeFlyModal(){var m=document.getElementById('flymodal');if(m)m.className='modal';}
function flyRow(ok,label,detail){return '<div class="flyrow '+(ok===true?'ok':ok===false?'bad':'na')+'"><span class=fmark>'+(ok===true?'✓':ok===false?'✕':'•')+'</span><span class=flabel>'+label+'</span><span class=fdetail>'+(detail||'')+'</span></div>';}
function buildFlyChecks(){
 var s=window.LAST||{},g=s.gps||{},hl=s.health||{},pa=s.gps_prearm||{},b=s.batt||{},ids=s.assignment||[],h='',block=false;
 if(!(s.pads_selected&&ids.length))block=true;
 h+=flyRow(!!(s.pads_selected&&ids.length),'เลือก pad แล้ว',ids.length?('ID '+ids.join(', ')):'ยังไม่เลือก');
 var preFail=(pa.ok===false&&pa.age!=null&&pa.age<15);
 var gpsOk=(g.fix>=3&&!preFail&&(g.hdop==null||g.hdop<=2.0));
 h+=flyRow(gpsOk,'GPS พร้อม arm (ตรงกับ PX4)',(g.fix>=3?'3D '+(g.sats||0)+' ดวง'+(g.hdop!=null?' · hdop '+g.hdop:''):'fix '+(g.fix||0))+(preFail?' · PX4:PDOP สูง':''));
 var fp=fenceRef(s);
 if(fp.length>=3&&g.lat!=null&&g.fix>=2){var ins=pointInPoly(g.lat,g.lon,fp);if(!ins)block=true;h+=flyRow(ins,'โดรนอยู่ในรั้ว geofence (mission)',ins?'ในรั้ว':'อยู่นอกรั้ว!');}
 else h+=flyRow(null,'โดรนอยู่ในรั้ว geofence','ยังไม่มีตำแหน่ง GPS');
 h+=flyRow(hl.battery!==false&&(b.pct==null||b.pct>25),'แบตเตอรี่',(b.volt!=null?b.volt+'V':'')+(b.pct!=null?' · '+b.pct+'%':''));
 h+=flyRow(s.link!==false&&s.cm4_ok!==false,'ลิงก์ + CM4',(s.link?'link OK':'link หลุด')+(s.cm4_ok===false?' · CM4 หลุด':''));
 h+=flyRow(hl.rc!==false,'RC (safety pilot) พร้อม',hl.rc!==false?'พร้อม':'ไม่พบ RC');
 document.getElementById('flychecks').innerHTML=h;
 var sw=document.getElementById('flyslide'),lb=document.getElementById('flyslidelabel');
 if(block){sw.classList.add('blocked');lb.textContent='⛔ แก้รายการสีแดงก่อน (pad / นอกรั้ว)';}
 else{sw.classList.remove('blocked');if(!FLY.busy)lb.textContent='สไลด์ขวาเพื่อยืนยัน →';}
 return block;
}
var SL={drag:false,x0:0,max:0};
function slideReset(){var t=document.getElementById('flyslidethumb'),f=document.getElementById('flyslidefill');if(t)t.style.left='3px';if(f)f.style.width='0';}
function slideStart(e){var w=document.getElementById('flyslide');if(!w||w.classList.contains('blocked')||FLY.busy)return;SL.drag=true;SL.x0=(e.touches?e.touches[0].clientX:e.clientX);SL.max=w.clientWidth-50;e.preventDefault();}
function slideMove(e){if(!SL.drag)return;var x=(e.touches?e.touches[0].clientX:e.clientX)-SL.x0;x=Math.max(0,Math.min(SL.max,x));document.getElementById('flyslidethumb').style.left=(3+x)+'px';document.getElementById('flyslidefill').style.width=(x+47)+'px';if(x>=SL.max-2){SL.drag=false;slideDone();}}
function slideEnd(){if(!SL.drag)return;SL.drag=false;slideReset();}
function slideDone(){document.getElementById('flyslidelabel').textContent='✅ ยืนยันแล้ว — กำลัง up…';doUpload();}
function bindSlide(){var t=document.getElementById('flyslidethumb');if(!t)return;
 t.addEventListener('mousedown',slideStart);window.addEventListener('mousemove',slideMove);window.addEventListener('mouseup',slideEnd);
 t.addEventListener('touchstart',slideStart,{passive:false});window.addEventListener('touchmove',slideMove,{passive:false});window.addEventListener('touchend',slideEnd);}
async function doUpload(){
 if(FLY.busy)return;FLY.busy=true;
 var p=document.getElementById('flyprog');p.style.display='block';p.textContent='⏳ กำลัง up mission ขึ้นโดรน…';
 document.getElementById('flyslide').style.display='none';
 try{var r=await fetch('/api/mission/start',{method:'POST'});var j=await r.json();
  if(j.ok){window.UPWAIT=Date.now();p.textContent='⏳ ส่งคำสั่งแล้ว — รอโดรนยืนยันว่า mission ขึ้นเครื่อง…';}
  else{p.innerHTML='<span style=color:#ff7b72>✕ '+(j.err||'เริ่มไม่สำเร็จ')+'</span>';FLY.busy=false;document.getElementById('flyslide').style.display='block';slideReset();buildFlyChecks();}
 }catch(e){p.innerHTML='<span style=color:#ff7b72>สั่งไม่สำเร็จ: '+e+'</span>';FLY.busy=false;}
}
// called from tick(): after upload, prompt RC arm -> OFFBOARD; go green on OFFBOARD
function updateFlyOffboard(s,offb){
 var m=document.getElementById('flymodal');if(!m||!m.classList.contains('show'))return;
 if(!window.UPWAIT&&!offb)return;
 var p=document.getElementById('flyprog');
 if(p&&!FLY.stagedShown){FLY.stagedShown=true;p.style.display='block';p.innerHTML='✅ mission ขึ้นเครื่องแล้ว — พร้อมให้ RC ปล่อย';}
 var ob=document.getElementById('flyoffboard');if(!ob)return;ob.style.display='block';
 if(offb){ob.className='flyoffboard go';ob.innerHTML='✅ OFFBOARD — โดรนรับช่วงแล้ว กำลังออกบิน';}
 else{ob.className='flyoffboard wait';ob.innerHTML='🎮 นักบิน: <b>arm RC</b> → <b>สลับ OFFBOARD</b> เพื่อปล่อยบิน<br><span style="font-size:12px;font-weight:400">GCS จะแจ้งเขียวเมื่อโดรนเข้า OFFBOARD</span>';}
}
// edit-lock while flying (operator spec 2026-08-12): once the mission is
// running (or the vehicle is armed/offboard, incl. missions started from the
// CLI), เลือก/แก้ไข + 🚀 turn gray + disabled; the pad modal force-closes.
async function resetField(){
 if(!confirm('🧹 รีเซ็ตสนาม (SIM): เกิดใหม่ทั้ง SITL + pad สุ่มใหม่ + กล่องครบ 4 ใบ — ใช้เวลา ~1 นาที ทำเลยหรือไม่?'))return;
 var m=document.getElementById('savemsg');if(m)m.textContent='🧹 กำลังรีเซ็ตสนาม… (~1 นาที)';
 try{var r=await fetch('/api/mission/reset',{method:'POST'});var j=await r.json();
  if(!j.ok&&j.err){alert(j.err);if(m)m.textContent=j.err;}}
 catch(e){if(m)m.textContent='รีเซ็ตไม่สำเร็จ: '+e;}
}
// ── mission awareness pack (operator picks 2026-08-14: A2+B2+T1(no sound)+C1+D) ──
var SEGNAMES=['ขึ้นบิน','เข้าเส้นทาง','ค้นหา','ส่งของ','กลับ'];
var SEGEDGES=[2,8,18,55,90,100];   // segment i spans SEGEDGES[i]..SEGEDGES[i+1]
function fmtEta(sec){if(sec==null)return'';if(sec<60)return'เหลือ ~'+sec+' วิ';
 return'เหลือ ~'+Math.round(sec/60)+' นาที';}
function fmtClock(t){if(t==null)return'';
 return Math.floor(t/60)+':'+('0'+Math.floor(t%60)).slice(-2)}
function renderProgress(s){
 var ms=s.mission||{},w=document.getElementById('pbarwrap');if(!w)return;
 var stale=ms.age_s!=null&&ms.age_s>45;
 var pr=(typeof ms.progress==='number')?ms.progress:null;
 var active=!stale&&pr!=null&&ms.phase&&(ms.phase!=='done'||pr===100);
 // the %-bar REPLACES the old recon/deliver/done stepper (operator
 // 2026-08-14: duplicated info) — the stepper stays only for idle/stale
 // or a feed without progress fields (e.g. the sibling repo's writer)
 var mp=document.getElementById('mprog');
 if(mp)mp.style.display=active?'none':'block';
 if(!active){w.style.display='none';return}
 w.style.display='block';
 var segs=document.getElementById('psegs');
 if(!segs.childElementCount)
  segs.innerHTML=SEGNAMES.map(function(n){return'<div class=pseg><i></i><b>'+n+'</b></div>'}).join('');
 for(var i=0;i<5;i++){var lo=SEGEDGES[i],hi=SEGEDGES[i+1];
  var f=pr>=hi?1:(pr<=lo?0:(pr-lo)/(hi-lo));
  segs.children[i].firstChild.style.width=Math.round(f*100)+'%';}
 document.getElementById('ppct').textContent=pr+'%';
 document.getElementById('plabel').textContent=ms.progress_label||'';
 var right=[];
 // ⏱ mission clock removed (operator 2026-08-18) — ETA stays: it is OUR
 // progress prediction, not the official time.
 if(ms.phase!=='done'&&ms.eta_s!=null)right.push(fmtEta(ms.eta_s));
 document.getElementById('peta').textContent=right.join(' · ');
}
function renderTimeline(s){
 // compact chip strip (replaced the 9-row metro timeline — too tall)
 var ms=s.mission||{},el=document.getElementById('tline');if(!el)return;
 var stale=ms.age_s!=null&&ms.age_s>45;
 var done=(ms.phase==='done');
 var run=!stale&&ms.phase&&(typeof ms.progress==='number');
 if(!run){el.style.display='none';return}
 var pr=ms.progress||0,asg=ms.assigned||[],dlv=ms.delivered||[];
 var evs=(ms.events||[]).map(function(e){return e.text});
 function has(sub){for(var i=0;i<evs.length;i++)if(evs[i].indexOf(sub)>=0)return true;return false}
 var chips=[];
 function chip(st,txt){chips.push('<span class="mchip '+st+'">'+txt+'</span>')}
 chip(done||pr>=8?'ok':(pr>=2?'cur':'todo'),done||pr>=8?'ขึ้นบิน ✓':'ขึ้นบิน');
 var pm=['P1','P2','P3'].map(function(p){return has('ผ่านจุด '+p)?p+'✓':p}).join('·');
 chip(done||pr>=18?'ok':(pr>=8?'cur':'todo'),pm);
 var found=0,padmap=ms.pads_mapped||{};
 for(var i=0;i<asg.length;i++)if(padmap[String(asg[i])])found++;
 chip(done||pr>=55?'ok':(pr>=18?'cur':'todo'),'🔍 '+found+'/'+asg.length);
 for(var j=0;j<asg.length;j++){var a=asg[j];
  var st=dlv.indexOf(a)>=0?'ok'
        :(!done&&(ms.progress_label||'').indexOf('pad '+a+' ')>=0?'cur':'todo');
  chip(st,st==='ok'?'pad '+a+' ✓':'pad '+a);}
 chip(done?'ok':(pr>=90?'cur':'todo'),done?'ถึงบ้าน ✓':'🏠 กลับ');
 var html=chips.join('');
 if(el.dataset.h!==html){el.dataset.h=html;el.innerHTML=html;}
 el.style.display='flex';
}
// T1 toasts — NO sound (explicit user request 2026-08-14)
var EVSEEN=null;
function renderToasts(s){
 var evs=((s.mission||{}).events)||[];if(!evs.length)return;
 var box=document.getElementById('toasts');if(!box)return;
 if(EVSEEN===null){EVSEEN={};                    // first tick: absorb history,
  for(var i=0;i<evs.length;i++)EVSEEN[evs[i].t+'|'+evs[i].text]=1;return}   // never replay old toasts
 for(var j=0;j<evs.length;j++){var e=evs[j],k=e.t+'|'+e.text;
  if(EVSEEN[k])continue;EVSEEN[k]=1;
  var d=document.createElement('div');d.className='toast'+(e.warn?' warn':'');
  d.textContent=e.text;box.appendChild(d);
  while(box.childElementCount>4)box.removeChild(box.firstChild);
  (function(dd){setTimeout(function(){if(dd.parentNode)dd.parentNode.removeChild(dd)},6000)})(d);}
}
// C1 post-flight summary card
var SUMSHOWN=false,WASLIVE=false;
function renderSummary(s){
 var ms=s.mission||{},stale=ms.age_s!=null&&ms.age_s>45;
 // Early homecoming (operator 2026-08-18: "อยากให้ขึ้นตอนกลับ home เหมือน
 // กราฟฟิกตอนทำ mission เสร็จ"): a live mission that is DISARMED on the
 // ground while carrying a home_reason has ended, even though the phase
 // never reached 'done' — pop the same card, headlined by the reason.
 var endedEarly=!stale&&ms.home_reason&&s.armed===false;
 var liveNow=!stale&&ms.phase&&ms.phase!=='done'&&!endedEarly;
 if(liveNow){WASLIVE=true;SUMSHOWN=false;return}
 if((ms.phase==='done'||endedEarly)&&WASLIVE&&!SUMSHOWN&&!stale){
  SUMSHOWN=true;WASLIVE=false;
  var dlv=(ms.delivered||[]),asg=(ms.assigned||[]);
  var t=ms.mission_time!=null
    ?Math.floor(ms.mission_time/60)+':'+('0'+Math.floor(ms.mission_time%60)).slice(-2):'–';
  var batt=(s.batt&&s.batt.pct!=null)?s.batt.pct+'%':'–';
  var ok=asg.length>0&&dlv.length>=asg.length&&!ms.home_reason;
  var early=!ok&&ms.home_reason;
  document.getElementById('sumcardc').style.borderColor=early?'#9e6a03':'';
  document.getElementById('sumcardc').innerHTML=
   (early
    ?'<h3 style="color:#e3b341">🏠 กลับบ้านก่อนจบภารกิจ</h3>'
     +'<div style="font-size:14px;color:#e3b341;margin:2px 0 8px">'+ms.home_reason+'</div>'
    :'<h3>'+(ok?'✅ ภารกิจสำเร็จ':'🏁 จบภารกิจ')+'</h3>')
  +'<div style="font-size:13px;color:#8b98a5">'+(s.mission_current||'')+'</div>'
  +'<div class=sumgrid>'
  +'<div class=sumcell><b>'+dlv.length+'/'+asg.length+'</b><span>ส่งสำเร็จ</span></div>'
  +'<div class=sumcell><b>'+t+'</b><span>เวลาบิน</span></div>'
  +'<div class=sumcell><b>'+batt+'</b><span>แบตเหลือ</span></div>'
  +'<div class=sumcell><b>'+Object.keys(ms.pads_mapped||{}).length+'</b><span>pad ที่เจอ</span></div>'
  +'</div>'
  +'<div style="font-size:12.5px;color:#3fb950;margin-bottom:12px">'
  +dlv.map(function(p){return'pad '+p+' ✓'}).join(' · ')+'</div>'
  +'<button class=savebtn style="margin:0" onclick=closeSummary()>ปิด</button>';
  document.getElementById('summodal').className='show';}
}
function closeSummary(){document.getElementById('summodal').className='';
}
// D field mode (sunlight): 🔆 toggle, remembered across sessions
function fmToggle(){var on=document.body.classList.toggle('fieldmode');
 try{localStorage.setItem('fieldmode',on?'1':'0')}catch(e){}}
try{if(localStorage.getItem('fieldmode')==='1')document.body.classList.add('fieldmode')}catch(e){}
// mission switcher (2026-08-13): dropdown in the Mission card — re-points the
// console (map field, captures, 🚀 command) at another registered mission.
// Both projects fly the SAME aircraft; entries differ only in field/envelope.
function realSwitchHelp(){
 var s=window.LAST||{},host=s.real_host||'<cm4>',dir=(s.real_dir||'~/mission');
 alert('🔒 console นี้ผูกกับเครื่องจริงตัวเดียว\\n\\n'
 +'mission ที่โหลดอยู่: '+(s.mission_current||'(ตาม --mission-cmd)')+'\\n'
 +'สั่งไปที่: '+host+' → '+dir+'\\n\\n'
 +'ต้องการสลับไป mission อื่นบนเครื่องจริง ทำ 3 ขั้น:\\n'
 +'1) deploy repo ของ mission นั้นขึ้น CM4 (cm4/deploy.sh)\\n'
 +'2) ปิด console นี้ (Ctrl-C ที่ terminal ที่เปิดมัน)\\n'
 +'3) เปิดใหม่ด้วย --mission-cmd ที่ชี้ไป entry ของ repo นั้น\\n'
 +'   เช่น cm4/launch_gcs_real.sh <user>@'+host+'\\n\\n'
 +'ระหว่างบินห้ามสลับอยู่แล้ว — ที่ล็อกไว้เพื่อกันปุ่ม 🚀 กลายเป็นคำสั่ง SIM เงียบ ๆ');
}
function renderMissions(s){
 var row=document.getElementById('miselrow'),sel=document.getElementById('misel');
 var card=document.getElementById('mrealcard');
 if(!row||!sel)return;
 // REAL console: no dropdown at all — a locked card that NAMES the loaded
 // mission + its CM4, and explains switching (click for the steps)
 if(s.real_console&&card){
  row.style.display='none';card.style.display='block';card.className='mreal';
  var nm=s.mission_current||'(ตาม --mission-cmd)';
  var lbl=nm;
  for(var i=0;i<(s.missions||[]).length;i++)if(s.missions[i].name===nm)lbl=s.missions[i].label;
  var html='<div class=t>🔒 เครื่องจริง · '+lbl+'</div>'
   +'<div class=s>ปุ่ม 🚀 สั่งไปที่ <b>'+(s.real_host||'CM4')+'</b>'
   +(s.real_dir?' → <b>'+s.real_dir+'</b>':'')+'</div>'
   +'<div class=h>สลับ mission ไม่ได้จากหน้านี้ — ต้องปิด console แล้วเปิดใหม่ '
   +'(แตะเพื่อดูขั้นตอน)</div>';
  if(card.dataset.h!==html){card.dataset.h=html;card.innerHTML=html;}
  return;
 }
 if(card)card.style.display='none';
 var ms=s.missions||[];
 if(ms.length<2){row.style.display='none';return}
 row.style.display='block';
 var sig=JSON.stringify([ms,s.mission_current,!!s.real_console]);
 if(sel.dataset.sig!==sig){sel.dataset.sig=sig;
  sel.innerHTML=ms.map(function(m){
   var cur=(m.name===s.mission_current);
   var off=!m.available||(s.real_console&&!cur);
   return '<option value="'+m.name+'"'+(cur?' selected':'')
        +(off?' disabled':'')+'>'+(cur?'📌 ':'')
        +m.label+(m.available?'':' — ยังไม่พร้อม (รอ contract)')
        +((s.real_console&&!cur&&m.available)?' — ต้อง deploy ขึ้น CM4 ก่อน':'')
        +'</option>';
  }).join('');}
 // a REAL console is pinned to its ssh GO: switching would swap it for a
 // laptop-side SIM command (see apply_mission)
 sel.disabled=!!s.real_console
   ||((!s.demo)&&(!!s.mission_running||!!s.armed||!!s.reset_running));
 var lbl=row.firstElementChild;
 if(lbl)lbl.textContent=s.real_console
   ?'🗂 Template mission — 🔒 ล็อกบนเครื่องจริง (console นี้สั่ง CM4)'
   :'🗂 Template mission — กดกล่องด้านล่างเพื่อสลับสนาม';
}
async function selectMission(){
 var sel=document.getElementById('misel');var name=sel.value;
 var cur=(window.LAST||{}).mission_current;
 if(name===cur)return;
 var lbl=sel.options[sel.selectedIndex].text.replace('📌 ','');
 if(!confirm('สลับไป mission "'+lbl+'" ?\\n(แผนที่/pad/ปุ่ม 🚀 จะชี้ไป mission นั้นทันที)')){
  sel.value=cur;return;}
 try{var r=await fetch('/api/mission/select',{method:'POST',
   headers:{'Content-Type':'application/json'},body:JSON.stringify({name:name})});
  var j=await r.json();
  if(!j.ok){alert(j.err||'สลับไม่สำเร็จ');sel.value=cur;return;}
  window.ZSIG=null;window.ZFIT=true;SEL=[];PADS_INIT=false;   // re-sync + pan to the new field
 }catch(e){alert(e);sel.value=cur;}
}
function renderMissionLock(s){
 var ms=s.mission||{};
 var fresh=ms.age_s!=null&&ms.age_s<=45;
 var live=(!s.demo)&&fresh&&!!ms.phase&&ms.phase!=='done';
 var resetting=!!s.reset_running;
 var lock=(!s.demo)&&(!!s.mission_running||live||!!s.armed||resetting);
 var po=document.getElementById('padopen');
 if(po){po.disabled=lock;po.className=lock?'savebtn locked':'savebtn';
  po.textContent=lock?(resetting?'🧹 กำลังรีเซ็ตสนาม…':'🔒 ล็อกขณะบิน mission — แก้ไข pad ไม่ได้')
                     :'✏️ เลือก / แก้ไข pad (ตั้งก่อนบิน offboard)';}
 var badge=s.mission_label?' ['+s.mission_label+']':'';
 var fb=document.getElementById('flybtn');
 if(fb){fb.style.display=s.mission_cmd?'block':'none';
  var noCm4=(s.cm4_ok===false);          // ssh mission with the CM4 unreachable
  fb.disabled=lock||!s.pads_selected||noCm4;
  // 🚀 upload feedback (2026-08-14): the CONFIRMATION is the drone's own
  // status feed turning fresh — proof the mission process is alive on the
  // aircraft (REAL: via status_sync), not just that the ssh/spawn returned.
  var phase=String(ms.phase||''), freshMs=fresh&&phase&&phase!=='done';
  var upState=null;
  if(s.mission_running){
   if(freshMs&&phase.indexOf('preflight')>=0)upState='staged';
   else if(freshMs)upState='flying';
   else upState='upwait';
  }
  var cls='savebtn';
  if(upState==='staged')cls='savebtn staged';
  else if(upState==='upwait')cls='savebtn upwait';
  else if(fb.disabled)cls='savebtn locked';
  fb.className=cls;
  fb.textContent=
    upState==='upwait'?'⏳ กำลัง up ขึ้นโดรน… (รอสัญญาณยืนยันจากโดรน)'
   :upState==='staged'?'✅ up เสร็จ — mission อยู่บนโดรนแล้ว (preflight/รอ RC)'
   :upState==='flying'?'🛫 กำลังบิน — '+phase
   :(resetting?'🧹 รอรีเซ็ตสนามเสร็จ…'
   :(noCm4?'🔒 ไม่พบสัญญาณ CM4 — เช็ค WiFi/เปิดเครื่อง'
   :(live?'🛫 mission กำลังบิน (เริ่มจากภายนอก)':'🚀 up ขึ้นโดรน'+badge)));
  // one-time "upload confirmed" line the moment staged/flying first appears
  if(upState&&upState!=='upwait'&&window.UPWAIT){window.UPWAIT=null;
   var m=document.getElementById('savemsg');
   if(m)m.textContent='✅ up ขึ้นโดรนสำเร็จ — โดรนรายงานสถานะกลับมาแล้ว';}
 }
 var rb=document.getElementById('resetbtn');
 if(rb){rb.style.display=s.reset_cmd?'block':'none';
  rb.disabled=lock&&!resetting?true:resetting;
  rb.className=(rb.disabled)?'savebtn locked':'savebtn';
  rb.textContent=resetting?'🧹 กำลังรีเซ็ตสนาม…':'🧹 รีเซ็ตสนาม [SIM]';}
 if(lock){var pm=document.getElementById('padmodal');
  if(pm&&pm.className.indexOf('show')>=0){
   SEL=(window.SEL_BACKUP||SEL).slice();pm.className='modal';renderPads();}}
 window.MLOCK=lock;
}
// live per-pad status from the running mission: scanned -> "เจอแล้ว", dropped -> "drop แล้ว"
function renderPadLive(s){
 // Per-pad ladder (revived 2026-08-21 — it was dead code with no div/caller):
 // ⏳ รอสแกน → 🟠 เห็นแล้ว รอยืนยัน (identified: id decoded, votes short —
 // arrives over the RADIO alone via "AAVC seen=") → ✅ เจอแล้ว (confirmed) →
 // 📦 drop แล้ว. This is what the operator watches when in-flight WiFi dies.
 var el=document.getElementById('padlive');if(!el)return;
 var ms=s.mission||{},stale=(ms.age_s!=null&&ms.age_s>45);
 var mapped=stale?{}:(ms.pads_mapped||{}),deliv=stale?[]:(ms.delivered||[]),asg=s.assignment||[];
 var ident=stale?{}:(ms.pads_identified||{});
 var set={};asg.forEach(function(x){set[x]=1;});
 Object.keys(mapped).forEach(function(x){set[parseInt(x,10)]=1;});
 Object.keys(ident).forEach(function(x){set[parseInt(x,10)]=1;});
 deliv.forEach(function(x){set[x]=1;});
 var ids=Object.keys(set).map(Number).filter(function(x){return x>=1;}).sort(function(a,b){return a-b;});
 if(!ids.length){el.innerHTML='<div class="padli off">— รอ mission เริ่มสแกน —</div>';return;}
 el.innerHTML=ids.map(function(id){
   var found=(mapped[id]!=null)||(mapped[String(id)]!=null),dropped=deliv.indexOf(id)>=0;
   var seen=(id in ident)||(String(id) in ident);   // value may be null (radio: id only)
   if(dropped)return '<div class=padli style="color:#3fb950">📦 drop pad '+id+' แล้ว</div>';
   if(found)return '<div class=padli style="color:#8b949e">✅ เจอ pad '+id+' แล้ว</div>';
   if(seen)return '<div class=padli style="color:#e3a008">🟠 เห็น pad '+id+' รอยืนยัน</div>';
   return '<div class="padli off">⏳ pad '+id+' รอสแกน</div>';
 }).join('');
}
function renderServos(s){
 var g=document.getElementById('servogrid');if(!g)return;
 var cfg=(s.servo_cfg||[]).map(function(c){          // pre-2026-08-15 snapshots
   return (typeof c==='object')?c:{num:c,label:''};});   // sent bare numbers
 var st=s.servos||{},sel=!!s.pads_selected;
 if(g.childElementCount!==cfg.length){g.innerHTML='';cfg.forEach(function(c){
   var b=document.createElement('button');b.type='button';b.id='sv'+c.num;
   // TOGGLE (2026-08-15): each AUX button is an ON/OFF switch for that latch —
   // press to open, press again to close. Reading the CURRENT state at click
   // time (not at render time) keeps it correct even if the latch was moved by
   // "ปล่อยทั้งหมด" or by the mission between renders.
   b.onclick=(function(n){return function(){
     var st=(window.LAST||{}).servos||{};
     if((st[String(n)]||{}).released){resetServo([n]);}else{releaseServo([n]);}
   };})(c.num);
   g.appendChild(b);});}
 var mrun=!!s.mission_running;   // autonomous mission owns the latches — no manual drops
 cfg.forEach(function(c){var b=document.getElementById('sv'+c.num);if(!b)return;
   var info=st[String(c.num)]||{},rel=!!info.released;
   b.className=rel?'rel':'';
   b.innerHTML='AUX '+c.num+(c.label?'<br><b>'+c.label+'</b>':'')
              +'<br>'+(rel?'เปิดอยู่ ▸ กดเพื่อปิด':'ปิดอยู่ ▸ กดเพื่อเปิด');
   b.title='AUX '+c.num+(c.label?' — '+c.label:'')
          +(rel?' (เปิดอยู่ — กดเพื่อปิดกลับ)':' (ปิดอยู่ — กดเพื่อเปิด)');
   b.disabled=!sel||mrun;});                         // small buttons: locked until pad chosen
 var w=document.getElementById('servowarn');
 if(w){if(sel){w.className='servowarn';}
   else{w.className='servowarn show';
    w.innerHTML=s.armed?'🚨 ARMED แต่ยังไม่เลือก pad — ภารกิจไม่รู้ว่าจะ drop ที่ไหน!'
                       :'⚠️ เลือก pad ที่จะ drop ก่อน ถึงจะปล่อย servo ได้';}}
 var ab=document.getElementById('servoall');if(ab)ab.disabled=!sel||mrun;
}
async function releaseServo(which){
 var s=window.LAST||{};
 if(!s.pads_selected){alert('⚠️ เลือก pad ที่จะ drop ก่อน แล้วกดปุ่ม "บันทึก assignment"');return;}
 var lbl=which?('servo '+which.join(', ')):'servo ทั้งหมด';
 // Confirm only when it can actually drop an egg from the air. On a DISARMED
 // bench the latch is being toggled on purpose, over and over, and a modal on
 // every press just trains the operator to click through warnings.
 if((s.armed||!which)&&!confirm('ปล่อย '+lbl+' ?  payload จะหล่นทันที'))return;
 var m=document.getElementById('servomsg');if(m)m.textContent='กำลังปล่อย '+lbl+'…';
 var qs=which?('?'+which.map(function(n){return 'which='+n;}).join('&')):'';
 try{var r=await fetch('/api/servo/release'+qs,{method:'POST'});var j=await r.json();
  if(m)m.textContent=j.ok?('✅ ปล่อยแล้ว: '+lbl):('❌ '+(j.err||'error'));}
 catch(e){if(m)m.textContent='❌ '+e;}
}
async function resetServo(which){
 var lbl=which?('servo '+which.join(', ')):'servo ทั้งหมด';
 var m=document.getElementById('servomsg');if(m)m.textContent='กำลังปิด '+lbl+' กลับ…';
 var qs=which?('?'+which.map(function(n){return 'which='+n;}).join('&')):'';
 try{var r=await fetch('/api/servo/reset'+qs,{method:'POST'});var j=await r.json();
  if(m)m.textContent=j.ok?('↩️ ปิดกลับแล้ว: '+lbl+' (พร้อมเทสใหม่)'):('❌ '+(j.err||'error'));}
 catch(e){if(m)m.textContent='❌ '+e;}
}
function fmtClk(s){s=Math.max(0,Math.floor(s));var m=Math.floor(s/60),ss=s%60;return (m<10?'0':'')+m+':'+(ss<10?'0':'')+ss;}
// (updClock + its 250 ms interval removed with the #mclock element —
//  operator 2026-08-18: the committee keeps the official mission time.)
// map the real mission phase string -> step index in [recon, deliver, done]
// ("load" step removed 2026-08-12 per operator request — the eggs ride the
// whole flight so a loading step never lit in practice; an old feed's
// "load"-tagged phase still lands on recon instead of vanishing)
function phaseIdx(ph){ph=String(ph||'').toLowerCase();
 if(ph.indexOf('recon')>=0||ph.indexOf('load')>=0)return 0;
 if(ph.indexOf('deliver')>=0)return 1;
 if(ph.indexOf('done')>=0)return 2;
 return -1;}                                        // idle / unknown = not started
function renderMission(ms){
 var el=document.getElementById('mprog');if(!el)return;
 var why=ms&&ms.home_reason?'<div class="mtwhy">🏠 '+ms.home_reason+'</div>':'';
 if(!ms||(ms.age_s!=null&&ms.age_s>45)){el.innerHTML='<span style=color:#8b98a5>idle — ไม่มี mission สด</span>'+why;return;}
 var mapped=(ms.src==='radio')?(ms.mapped_n||0):Object.keys(ms.pads_mapped||{}).length,
     deliv=(ms.delivered||[]).length;
 var asg=(ms.assigned&&ms.assigned.length)?ms.assigned.length:SEL.length;
 var cur=phaseIdx(ms.phase),labels=['recon','deliver','done'],steps='';
 for(var i=0;i<labels.length;i++){
  var cls=(cur>=2)?'done':(i<cur?'done':(i===cur?'cur':''));
  steps+='<div class="mtstep '+cls+'"><i>'+(cls==='done'?'✓':'')+'</i><span>'+labels[i]+'</span></div>';
 }
 var fc=(asg>0&&mapped>=asg)?'#3fb950':'#e6e6e6',dc=(asg>0&&deliv>=asg)?'#3fb950':'#e6e6e6';
 el.innerHTML='<div class=mtl>'+steps+'</div>'+
  '<div class=mtstats>'+
   '<span>พบ <b style="color:'+fc+'">'+mapped+'/'+asg+'</b></span><span class=sep>·</span>'+
   '<span>ส่ง <b style="color:'+dc+'">'+deliv+'/'+asg+'</b></span>'+
  '</div>'+why;
}
function aavcEN(e,n,lat0,lon0){var R=6378137.0;
 return [lat0+(n/R)*180/Math.PI, lon0+(e/(R*Math.cos(lat0*Math.PI/180)))*180/Math.PI];}
function aavcMap(s){
 if(!mapReady||typeof L==='undefined')return;
 var z=s.zones||{};
 // redraw when the field CHANGES (map editor save/clear), not just once
 var zsig=JSON.stringify([z.airspace,z.search,z.transit,z.home]);
 if(window.ZSIG!==zsig&&(z.airspace||z.search)){
  window.ZSIG=zsig;
  for(var zi=0;zi<lzones.length;zi++)lmap.removeLayer(lzones[zi]);lzones=[];
  if(lhomeMarker){lmap.removeLayer(lhomeMarker);lhomeMarker=null;}
  if(z.airspace&&z.airspace.length>=3)lzones.push(L.polygon(z.airspace,{color:'#f85149',weight:2,fill:false,dashArray:'8 6'}).addTo(lmap).bindTooltip('controlled airspace'));
  if(z.search&&z.search.length>=3)lzones.push(L.polygon(z.search,{color:'#f0d000',weight:2,fill:false}).addTo(lmap).bindTooltip('search area'));
  if(z.transit&&z.transit.length>=1)lzones.push(L.polyline(z.transit,{color:'#58a6ff',weight:2,dashArray:'2 6'}).addTo(lmap).bindTooltip('transit P1→P3'));
  if(z.home)lhomeMarker=L.circleMarker(z.home,{radius:6,color:'#fff',fillColor:'#2ea043',fillOpacity:1,weight:2}).addTo(lmap).bindTooltip('HOME / L&R');
  // template switch: pan the map to the newly selected field
  if(window.ZFIT&&z.airspace&&z.airspace.length>=3){window.ZFIT=false;
   try{lmap.fitBounds(L.latLngBounds(z.airspace).pad(0.3))}catch(_){}}
 }
 // Pads = the CURRENT mission's scanned set ONLY (2026-08-12, operator
 // request): a stale feed (>45 s, e.g. mission process gone) counts as
 // empty, and any marker no longer present in pads_mapped is REMOVED — the
 // old code only ever added/recoloured, so a new mission (or a feed reset)
 // left the previous field's pads painted on the map until a page reload.
 var ms=s.mission||{},stale=(ms.age_s!=null&&ms.age_s>45);
 var pm=stale?{}:(ms.pads_mapped||{}),o=s.origin||{},deliv=stale?[]:(ms.delivered||[]);
 // identified-but-unconfirmed (2026-08-21): the ORANGE middle state. Only the
 // WiFi feed ships coordinates; the radio ships ids with null coords, which
 // light the sidebar ladder but cannot place a marker.
 var pi=stale?{}:(ms.pads_identified||{});
 // Fix 3 (2026-08-21): the LIVE mission plan — polyline + numbered stops so
 // the operator can see where the aircraft is going NEXT (the G7 takeover
 // came early precisely because the screen could not answer that). Arrives
 // over WiFi only ("plan" in mission_status.json, first written at gate
 // release while the launch-point WiFi still holds) and is deliberately
 // KEPT drawn while the feed is stale — that is exactly when it is needed.
 var plan=(ms.plan&&ms.plan.length)?ms.plan:(window.PLAN_KEEP||[]);
 if(ms.plan&&ms.plan.length)window.PLAN_KEEP=ms.plan;
 var psig=JSON.stringify(plan);
 if(window.PSIG!==psig){
  window.PSIG=psig;
  for(var lp=0;lp<lplan.length;lp++)lmap.removeLayer(lplan[lp]);lplan=[];
  if(plan.length>=1){
   var ppts=plan.map(function(r){return [r[0],r[1]];});
   lplan.push(L.polyline(ppts,{color:'#a371f7',weight:3,dashArray:'6 6',opacity:.85})
    .addTo(lmap).bindTooltip('เส้นทาง mission'));
   for(var pj=0;pj<plan.length;pj++){
    var prow=plan[pj];
    lplan.push(L.marker([prow[0],prow[1]],{icon:L.divIcon({className:'planicon',
     html:'<div class=planseq>'+prow[3]+'</div>',iconSize:[18,18],iconAnchor:[9,9]}),
     zIndexOffset:-100}).addTo(lmap).bindTooltip('#'+prow[3]+' '+prow[2],{direction:'top'}));
   }
  }
 }
 // "อยู่ในคิวส่ง" = the RUNNING mission's own assignment (mission_status
 // .assigned) when the feed is live — NOT the local editor state, which can
 // lag behind a selection saved elsewhere (bug seen 2026-08-12: pad 3 showed
 // queued from a stale page-load assignment). Editor SEL is only the
 // fallback when no live mission is feeding.
 var q=(!stale&&ms.assigned&&ms.assigned.length)?ms.assigned:SEL;
 // drawable = confirmed pads + identified pads that CARRY coordinates
 var drawable={};for(var ck in pm)drawable[ck]=1;
 for(var ik in pi){if(pi[ik]!=null)drawable[ik]=1;}
 for(var oid in lpads){if(!(oid in drawable)){lmap.removeLayer(lpads[oid]);delete lpads[oid];}}
 for(var id in drawable){
  if(o.lat==null)break;
  var conf=(id in pm);
  var en=conf?pm[id]:pi[id],pos=aavcEN(en[0],en[1],o.lat,o.lon),pid=parseInt(id);
  var done=deliv.indexOf(pid)>=0;
  var cls=done?'padbox done':(!conf?'padbox ident'
           :(q.indexOf(pid)>=0?'padbox queue':'padbox todo'));
  var st=done?'ส่งแล้ว':(!conf?'เห็นแล้ว รอยืนยัน'
          :(q.indexOf(pid)>=0?'อยู่ในคิวส่ง':'เจอแล้ว'));
  var icon=L.divIcon({className:'padicon',iconSize:[36,36],iconAnchor:[18,18],
   html:'<div class="'+cls+'"><i>✓</i><span>'+id+'</span></div>'});
  if(!lpads[id])lpads[id]=L.marker(pos,{icon:icon}).addTo(lmap).bindTooltip('Pad '+id+' — '+st,{direction:'top'});
  else{lpads[id].setLatLng(pos);lpads[id].setIcon(icon);lpads[id].setTooltipContent('Pad '+id+' — '+st);}
 }
}
async function tick(){
 var s;try{s=await(await fetch('/api/status')).json()}catch(e){
  document.getElementById('link').innerHTML='<span class=dot style=background:#f85149></span>server หลุด';return}
 window.LAST=s;
 var L=document.getElementById('link');
 var _lt=s.demo?'DEMO — ข้อมูลตัวอย่าง':(s.link?'online':'no signal'),_lc=s.demo?'#a371f7':(s.link?'#3fb950':'#f85149');
 L.innerHTML='<span class=dot style=background:'+_lc+'></span>'+_lt;
 var _db=document.getElementById('demobar');if(_db)_db.style.display=s.demo?'block':'none';
 document.getElementById('mode').textContent=s.mode||'–';
 document.getElementById('armstate').innerHTML=s.armed?'<span class=bad>ARMED</span>':'<span class=ok>DISARM</span>';
 // mission budget clock: START on ARMED+OFFBOARD, keep running while airborne, FREEZE on disarm, RESTART next flight
 var _offb=s.armed&&String(s.mode||'').toUpperCase()==='OFFBOARD';
 if(_offb){if(FT.start==null||FT.frozen!=null){FT.start=Date.now();FT.frozen=null;}}
 else if(!s.armed&&FT.start!=null&&FT.frozen==null){FT.frozen=(Date.now()-FT.start)/1000;}
 try{updateFlyOffboard(s,_offb);}catch(e){}   // 🚀 checklist modal: RC arm->OFFBOARD status
 var b=s.batt||{};
 var bc=(b.pct!=null&&b.pct<20)?'bad':(b.pct!=null&&b.pct<40)?'warn':'ok';
 document.getElementById('batt').innerHTML=b.volt==null?'–':'<span class="'+bc+'">'+b.volt+'V</span>';
 document.getElementById('battpct').innerHTML=b.pct==null?'–':'<span class="'+bc+'">'+b.pct+'%</span>';
 var g=s.gps||{};
 document.getElementById('gpsfix').innerHTML='<span class="'+(g.fix>=3?'ok':'bad')+'">'+(g.fix_str||'–')+'</span>';
 document.getElementById('sats').textContent=g.sats==null?'–':g.sats;
 var a=s.att||{};
 try{renderInsti(s)}catch(e){}
 var msgs=s.messages||[];
 if(msgs.length){var m=msgs[msgs.length-1];document.getElementById('msgline').textContent='['+m.t+'] '+m.txt;}
 var gfn=document.getElementById('gfnow'); if(gfn) gfn.textContent=gfFmt(s.geofence);
 var lk=s.link_kind||'',wired=(lk==='usb'||lk==='cm4'||lk==='other'),upB=document.getElementById('gfup');
 if(upB){upB.disabled=!wired;upB.textContent=wired?'⬆️ Upload สี่เหลี่ยม':'🔒 Upload (ต่อ USB ก่อน)';upB.title=wired?'':'เสียบ FC USB ก่อน — upload สี่เหลี่ยมบนวิทยุ ELRS ไม่ชัวร์';}
 // Keep the editor in sync with the SAVED assignment whenever the modal is
 // CLOSED (2026-08-12 rewrite of the old load-once PADS_INIT latch, which
 // went stale the moment the selection was saved from another tab/session).
 var pmod=document.getElementById('padmodal');
 if(!(pmod&&pmod.className.indexOf('show')>=0)){
  var srvSel=(s.assignment||[]).slice();
  if(srvSel.join(',')!==SEL.join(',')){SEL=srvSel;PADS_INIT=true;renderPads();}}
 // no pad chosen yet -> pop the picker automatically (once per load); manual button still edits
 // (suppressed while the edit-lock is on — never pop the picker mid-flight)
 if(!window.PAD_AUTO_SHOWN&&!s.pads_selected&&!window.MLOCK&&!s.mission_running){
  window.PAD_AUTO_SHOWN=true;try{openPadModal()}catch(e){}}
 try{renderMission(s.mission)}catch(e){}
 try{renderProgress(s)}catch(e){}
 try{renderTimeline(s)}catch(e){}
 try{renderToasts(s)}catch(e){}
 try{renderSummary(s)}catch(e){}
 try{renderMissions(s)}catch(e){}
 try{renderMissionLock(s)}catch(e){}
 try{renderServos(s)}catch(e){}
 try{renderPadStatus(s)}catch(e){}
 try{renderPadLive(s)}catch(e){}
 try{layoutTop()}catch(e){}
}
function gfActName(a){var m={0:'ปิด',1:'เตือน',2:'Hold',3:'RTL',4:'Terminate',5:'Land'};return (m[a]!=null)?m[a]:a;}
function gfFmt(g){if(!g||(g.hor==null&&g.ver==null&&g.action==null))return 'ยังไม่ได้อ่าน';
 if((!g.hor||g.hor<=0)&&(!g.ver||g.ver<=0))return 'ปิดอยู่';
 return 'R '+(g.hor||0)+'m · เพดาน '+(g.ver||0)+'m · '+gfActName(g.action);}
async function gfRead(){try{await fetch('/api/geofence/read',{method:'POST'})}catch(e){}}
async function gfApply(){
 var r=+document.getElementById('gfrad').value,a=+document.getElementById('gfalt').value,act=+document.getElementById('gfact').value;
 if(!confirm('ตั้ง geofence รอบจุดโฮม: รัศมี '+r+' m · เพดาน '+a+' m · เกินแล้ว → '+gfActName(act)+'  (เขียนลง FMU ผ่านวิทยุ ~5-10 วิ)'))return;
 var ctr=lhome||(mapReady?[lmap.getCenter().lat,lmap.getCenter().lng]:null);
 gfWant=(r>0&&ctr)?{r:r,ctr:ctr}:null;
 if(mapReady&&gfWant){
  if(!lgfcircle){lgfcircle=L.circle(gfWant.ctr,{radius:gfWant.r,color:'#f0b90b',weight:2,fill:false,dashArray:'6 6'}).addTo(lmap);}
  else{lgfcircle.setLatLng(gfWant.ctr);lgfcircle.setRadius(gfWant.r);}
  try{lmap.fitBounds(lgfcircle.getBounds().pad(0.3));}catch(e){}
 }else if(mapReady&&lgfcircle){lmap.removeLayer(lgfcircle);lgfcircle=null;}
 try{await fetch('/api/geofence/set?radius='+r+'&alt='+a+'&action='+act,{method:'POST'})}catch(e){}
 setTimeout(gfRead,3000);
}
var gfB=document.getElementById('gfbtn');
if(gfB)gfB.onclick=function(){var p=document.getElementById('gfpanel');p.classList.toggle('open');if(p.classList.contains('open'))gfRead();};
var gfR=document.getElementById('gfread');if(gfR)gfR.onclick=gfRead;
var gfA=document.getElementById('gfapply');if(gfA)gfA.onclick=gfApply;
var oB=document.getElementById('originbtn');
if(oB)oB.onclick=async function(){
 if(!confirm('ตั้ง origin พิกัด local (0,0) = ตำแหน่งปัจจุบันของโดรน?\\n(ส่ง SET_GPS_GLOBAL_ORIGIN ไป FMU — พิกัด pad/NED ของ mission จะอิงจุดนี้)'))return;
 try{var r=await fetch('/api/origin/set',{method:'POST'});await r.json();}catch(e){}
};
function cornerIcon(){return L.divIcon({className:'fcorner',html:'',iconSize:[14,14],iconAnchor:[7,7]});}
function fenceClearLocal(){if(frect){lmap.removeLayer(frect);frect=null;}for(var i=0;i<fmarkers.length;i++)lmap.removeLayer(fmarkers[i]);fmarkers=[];fcorners=[];}
function fenceDraw(){
 if(typeof L==='undefined'||!mapReady){alert('แผนที่ยังไม่พร้อม (ต้องมีเน็ต)');return;}
 fenceClearLocal();
 var c=lmap.getCenter(),dLat=0.0004,dLon=0.0004/Math.max(0.3,Math.cos(c.lat*Math.PI/180));
 fcorners=[[c.lat+dLat,c.lng-dLon],[c.lat+dLat,c.lng+dLon],[c.lat-dLat,c.lng+dLon],[c.lat-dLat,c.lng-dLon]];
 frect=L.polygon(fcorners,{color:'#3fb950',weight:2,fillOpacity:.08,dashArray:'4 4'}).addTo(lmap);
 fmarkers=fcorners.map(function(p,i){var mk=L.marker(p,{draggable:true,icon:cornerIcon()}).addTo(lmap);mk.on('drag',function(e){fcorners[i]=[e.latlng.lat,e.latlng.lng];frect.setLatLngs(fcorners);});return mk;});
 try{lmap.fitBounds(frect.getBounds().pad(0.6));}catch(e){}
}
async function fenceUpload(){
 var _u=document.getElementById('gfup');if(_u&&_u.disabled){alert('🔒 เสียบ FC USB ก่อน — upload สี่เหลี่ยมบนวิทยุ ELRS ไม่ชัวร์');return;}
 if(fcorners.length<3){alert('กด "วาดสี่เหลี่ยม" ก่อนครับ');return;}
 var act=+document.getElementById('gfact2').value,alt=+document.getElementById('gfalt2').value;
 if(!confirm('Upload สี่เหลี่ยม '+fcorners.length+' จุด · เพดาน '+alt+' m เข้า FMU? (ต้อง Drone เปิด · ผ่านวิทยุอาจช้า/ต้องกดซ้ำ)'))return;
 try{await fetch('/api/fence/upload',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pts:fcorners,action:act,alt:alt})});}catch(e){}
}
async function fenceClearFmu(){if(!confirm('ล้าง fence ออกจาก FMU?'))return;fenceClearLocal();if(lupfence){lmap.removeLayer(lupfence);lupfence=null;}try{await fetch('/api/fence/clear',{method:'POST'})}catch(e){}}
async function gfClrCircle(){
 if(!confirm('ล้างวงกลม? (ปิด geofence ระยะ+เพดาน)'))return;
 document.getElementById('gfrad').value=0;document.getElementById('gfalt').value=0;
 gfWant=null;if(lgfcircle){lmap.removeLayer(lgfcircle);lgfcircle=null;}
 var act=+document.getElementById('gfact').value;
 try{await fetch('/api/geofence/set?radius=0&alt=0&action='+act,{method:'POST'})}catch(e){}
 setTimeout(gfRead,2500);
}
var _fb;
_fb=document.getElementById('gfdraw');if(_fb)_fb.onclick=fenceDraw;
_fb=document.getElementById('gfup');if(_fb)_fb.onclick=fenceUpload;
_fb=document.getElementById('gfclr');if(_fb)_fb.onclick=fenceClearFmu;
_fb=document.getElementById('gfclrc');if(_fb)_fb.onclick=gfClrCircle;
bindSlide();setInterval(tick,600);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        # never let the browser cache the dashboard/API — a stale cached page
        # (from an older build) renders buttons but its JS can fail to update,
        # showing "no telemetry" even though the live API is fine.
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/test":
            # minimal JS-vs-fetch diagnostic (no console needed)
            self._send(200, TESTPAGE, "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self._send(200, json.dumps(LINK.snapshot()))
        elif self.path.startswith("/api/log/list"):
            files = []
            try:
                for f in os.listdir(LOG_DIR):
                    if f.endswith(".ulg"):
                        files.append({"name": f,
                                      "size": os.path.getsize(os.path.join(LOG_DIR, f))})
            except FileNotFoundError:
                pass
            files.sort(key=lambda x: x["name"], reverse=True)
            self._send(200, json.dumps(files))
        elif self.path.startswith("/api/log/get"):
            from urllib.parse import urlparse, parse_qs
            name = (parse_qs(urlparse(self.path).query).get("name") or [""])[0]
            safe = os.path.basename(name)         # block path traversal
            p = os.path.join(LOG_DIR, safe)
            if safe.endswith(".ulg") and os.path.isfile(p):
                with open(p, "rb") as fh:
                    data = fh.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{safe}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self._send(404, "{}")
        elif self.path == "/leaflet.js":
            self._send(200, VENDOR_JS, "application/javascript")
        elif self.path == "/leaflet.css":
            self._send(200, VENDOR_CSS, "text/css")
        else:
            self._send(404, "{}")

    def do_POST(self):
        action = self.path.replace("/api/", "")
        if action == "assign":            # AAVC pad-assignment: a FILE write (works in demo too,
            try:                          # and is the ONLY thing the console writes — no fly cmd)
                ln = int(self.headers.get("Content-Length", 0) or 0)
                ids = [int(x) for x in json.loads(self.rfile.read(ln) or b"{}").get("ids", [])]
                os.makedirs(AAVC_CAPTURES, exist_ok=True)
                with open(os.path.join(AAVC_CAPTURES, "pad_assignment.json"), "w") as fh:
                    json.dump({"ids": ids, "updated": time.time()}, fh)
                return self._send(200, json.dumps({"ok": True, "ids": ids}))
            except Exception as e:
                return self._send(200, json.dumps({"ok": False, "err": str(e)}))
        if action == "mission/select":    # in-UI mission switcher (2026-08-13)
            try:
                ln = int(self.headers.get("Content-Length", 0) or 0)
                name = str(json.loads(
                    self.rfile.read(ln) or b"{}").get("name") or "")
                err = apply_mission(name)
                if err:
                    return self._send(200, json.dumps({"ok": False, "err": err}))
                print(f"[aavc] mission switched -> {name}: "
                      f"field={AAVC_FIELD} captures={AAVC_CAPTURES}")
                return self._send(200, json.dumps({"ok": True, "name": name}))
            except Exception as e:
                return self._send(200, json.dumps({"ok": False, "err": str(e)}))
        if action == "mission/start":     # GCS GO (2026-08-12): spawn --mission-cmd
            if LINK.demo:
                return self._send(200, json.dumps(
                    {"ok": False, "err": "demo mode — ไม่มีเครื่องจริงให้บิน"}))
            if not MISSION_CMD:
                return self._send(200, json.dumps(
                    {"ok": False, "err": "console ไม่ได้ตั้ง --mission-cmd (เปิดผ่าน launch_stack.sh/make aavc-gcs)"}))
            ids = selected_pads()
            if not ids:                   # same interlock as the servo release
                return self._send(200, json.dumps(
                    {"ok": False, "err": "⚠️ เลือก + บันทึก pad ก่อนสั่งบิน"}))
            if _CM4_OK is False:          # ssh mission + CM4 unreachable
                return self._send(200, json.dumps(
                    {"ok": False, "err": "🔒 ไม่พบสัญญาณ CM4 — เช็ค WiFi/"
                                         "เปิดเครื่องก่อน แล้วปุ่มจะปลดเอง"}))
            if mission_running():
                return self._send(200, json.dumps(
                    {"ok": False, "err": "mission กำลังบินอยู่แล้ว"}))
            if LINK.snapshot().get("armed"):
                return self._send(200, json.dumps(
                    {"ok": False, "err": "เครื่อง ARMED อยู่ — ต้อง disarm ก่อนเริ่ม mission ใหม่"}))
            if reset_running():
                return self._send(200, json.dumps(
                    {"ok": False, "err": "🧹 กำลังรีเซ็ตสนาม — รอให้เสร็จก่อน"}))
            ok, detail = start_mission(ids)
            print(f"[aavc] mission start ids={ids}: {detail}")
            return self._send(200, json.dumps(
                {"ok": ok, "ids": ids, "err": None if ok else detail}))
        if action == "mission/reset":     # SIM-only field reset (--reset-cmd)
            if LINK.demo:
                return self._send(200, json.dumps(
                    {"ok": False, "err": "demo mode — ไม่มีสนามให้รีเซ็ต"}))
            if not RESET_CMD:
                return self._send(200, json.dumps(
                    {"ok": False, "err": "console ไม่ได้ตั้ง --reset-cmd (ปุ่มนี้มีเฉพาะ SIM)"}))
            if mission_running():
                return self._send(200, json.dumps(
                    {"ok": False, "err": "mission กำลังบิน — รีเซ็ตไม่ได้"}))
            if LINK.snapshot().get("armed"):
                return self._send(200, json.dumps(
                    {"ok": False, "err": "เครื่อง ARMED อยู่ — รีเซ็ตไม่ได้"}))
            ok, detail = start_reset()
            print(f"[aavc] field reset: {detail}")
            return self._send(200, json.dumps(
                {"ok": ok, "err": None if ok else detail}))
        if action.startswith("servo/"):      # AAVC payload servo (DO_SET_SERVO). ADD-only;
            from urllib.parse import urlparse, parse_qs   # works in demo (state only, no send)
            q = parse_qs(urlparse(self.path).query)
            which = [int(x) for x in q.get("which", [])] or None   # ?which=1&which=2 else all
            if action.startswith("servo/release"):
                if not selected_pads():       # INTERLOCK: must choose drop pads first
                    return self._send(200, json.dumps(
                        {"ok": False, "err": "⚠️ เลือก pad ที่จะ drop ก่อน แล้วค่อยปล่อย servo"}))
                if mission_running():         # INTERLOCK: the autonomous mission owns
                    return self._send(200, json.dumps(   # the latches — no manual drops
                        {"ok": False, "err": "🔒 mission กำลังบิน — ห้ามปล่อย servo มือ"}))
                LINK.release_servos(which)
            elif action.startswith("servo/reset"):
                LINK.reset_servos(which)      # closing the latch is always allowed
            else:
                return self._send(404, json.dumps({"error": "unknown servo action"}))
            return self._send(200, json.dumps({"ok": True}))
        if LINK.demo:
            LINK._dmsg(action)
            return self._send(200, json.dumps({"ok": True, "demo": True}))
        try:
            # Flight control (arm/disarm/land/kill) is RC-only by design — the web
            # exposes no manual flight commands, just calibration + the chirp trigger.
            if action == "cal/accel":
                LINK.cal_cancel() if LINK.is_cal_active() else LINK.cal_accel()
            elif action == "cal/level":
                LINK.cal_cancel() if LINK.is_cal_active() else LINK.cal_level()
            elif action == "chirp":
                CM4MGR.run_chirp()
            elif action == "chirp/stop":
                CM4MGR.stop_chirp()
            elif action == "log/pull":
                LINK.pull_log_async()
            elif action == "gps/check":
                LINK.check_gps_async()
            elif action == "gps/enable":
                LINK.enable_gps_async()
            elif action == "rc/check":
                LINK.check_rcmaps_async()
            elif action == "disarm/extend":
                LINK.set_disarm_async(60)
            elif action == "disarm/restore":
                LINK.set_disarm_async(10)
            elif action == "fltmode/read":
                LINK.read_fltmodes_async()
            elif action.startswith("fltmode/set"):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                LINK.set_fltmode_async(int(q["slot"][0]), int(q["value"][0]))
            elif action == "origin/set":          # AAVC: local NED (0,0) = current position
                LINK.set_local_origin_async()
            elif action == "geofence/read":
                LINK.read_geofence_async()
            elif action.startswith("geofence/set"):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                LINK.set_geofence_async(float(q["radius"][0]),
                                        float(q["alt"][0]), int(q["action"][0]))
            elif action == "fence/upload":
                ln = int(self.headers.get("Content-Length", 0) or 0)
                body = json.loads(self.rfile.read(ln) or b"{}")
                LINK.upload_fence_async(body.get("pts", []),
                                        int(body.get("action", 3)),
                                        body.get("alt"))
            elif action == "fence/read":
                LINK.read_fence_async()
            elif action == "fence/clear":
                LINK.clear_fence_async()
            else:
                return self._send(404, json.dumps({"error": "unknown"}))
            self._send(200, json.dumps({"ok": True}))
        except Exception as e:
            self._send(200, json.dumps({"error": str(e)}))


def main():
    global LINK, CM4MGR, AAVC_CAPTURES, AAVC_FIELD
    global MISSION_CMD, MISSION_LABEL, RESET_CMD, CURRENT_MISSION, REAL_CONSOLE
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default="config/real.yaml")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; default LOCALHOST-ONLY. The /api/* "
                         "command endpoints have NO auth, so a LAN/hotspot bind "
                         "(--host 0.0.0.0) lets any device on the network clear "
                         "the geofence, remap the RC mode switch, or fire the egg "
                         "latches while the aircraft is flying. Widen only on a "
                         "network you fully trust.")
    ap.add_argument("--demo", action="store_true",
                    help="serve sample telemetry with no FMU (preview)")
    ap.add_argument("--url", help="MAVLink endpoint override (e.g. /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, help="baud override")
    ap.add_argument("--captures",
                    help="dir for pad_assignment.json (out) + mission_status.json (in); "
                         "point at the touch-and-go mission's captures/ when testing")
    ap.add_argument("--field",
                    help="field yaml: GPS zones + pad IDs (default: bundled aavc_field.yaml)")
    ap.add_argument("--mission-cmd",
                    help="command template for the 🚀 บิน-mission button; {ids} is "
                         "replaced with the saved pad selection. SITL: "
                         "'<repo>/sitl/run_mission.sh {ids}'. Real bird: ssh into "
                         "the CM4 (the orchestrator runs there), e.g. "
                         "\"ssh aavc@cm4 'REAL=1 ~/mission/sitl/run_mission.sh {ids}'\"")
    ap.add_argument("--mission-label", default="",
                    help="badge on the 🚀 button, e.g. SIM or REAL — tells the "
                         "operator which world the configured mission-cmd flies")
    ap.add_argument("--reset-cmd",
                    help="SIM-ONLY field-reset template for the 🧹 button "
                         "(respawn SITL + pads + cargo). Omit on the real bird "
                         "— the button then stays hidden")
    args = ap.parse_args()
    if args.captures:
        AAVC_CAPTURES = os.path.abspath(os.path.expanduser(args.captures))
    if args.field:
        AAVC_FIELD = os.path.abspath(os.path.expanduser(args.field))
    if args.mission_cmd:
        MISSION_CMD = args.mission_cmd
        # an ssh GO = this console commands a REAL aircraft; pin it there
        REAL_CONSOLE = "ssh" in args.mission_cmd
    MISSION_LABEL = args.mission_label
    if args.reset_cmd:
        RESET_CMD = args.reset_cmd
    # in-UI mission switcher: load the registry and mark the entry whose
    # captures dir matches the CLI config as the active one
    load_missions()
    for _n, _m in MISSIONS.items():
        if _m and os.path.expanduser(_m.get("captures") or "") == AAVC_CAPTURES:
            CURRENT_MISSION = _n
            break
    # Desktop-icon flow (no explicit --captures): ALWAYS apply the FIRST
    # ready registry template so boot is deterministic and fully wired
    # (map field + captures + 🚀 all from ONE entry — a captures-only match
    # left a half-applied state: pinned name, no mission_cmd). The
    # captures-match above stays as a LABEL for CLI-configured consoles:
    # launch_stack / launch_gcs_real pass explicit --captures and must keep
    # their own mission_cmd (the REAL ssh command is never swapped for the
    # registry's SIM one).
    def _mission_ready(n):
        m = MISSIONS.get(n) or {}
        return bool(m.get("field")) and bool(m.get("mission_cmd"))
    if not args.captures:
        # `default: true` in missions.yaml wins; otherwise first ready entry.
        # Added 2026-08-15: registry ORDER was deciding which field yaml the
        # console booted with, so an operator on the bench kept getting the
        # other mission's servo labels/geofence after every restart and had to
        # re-pick the mission by hand each time.
        # AAVC_MISSION=<name> beats everything: it lets each operator's own
        # launcher pin the mission WITHOUT editing this shared registry, so the
        # two sessions stop flipping one `default:` flag back and forth.
        _env = os.environ.get("AAVC_MISSION", "").strip()
        _order = ([_env] if _env in MISSIONS else [])
        _order += ([n for n, m in MISSIONS.items()
                    if (m or {}).get("default") and n not in _order]
                   + [n for n, m in MISSIONS.items()
                      if not (m or {}).get("default") and n not in _order])
        if _env and _env not in MISSIONS:
            print(f"[aavc] AAVC_MISSION='{_env}' ไม่มีใน registry — ข้าม")
        for _n in _order:
            if _mission_ready(_n) and apply_mission(_n) is None:
                _why = ("AAVC_MISSION env" if _n == _env else
                        "default: true in missions.yaml"
                        if (MISSIONS.get(_n) or {}).get("default")
                        else "first ready in registry")
                print(f"[aavc] boot template: '{_n}' ({_why})")
                break
    if MISSIONS:
        print(f"[aavc] missions registry: {', '.join(MISSIONS)} "
              f"(active: {CURRENT_MISSION or 'custom CLI config'})")
    threading.Thread(target=_cm4_probe_loop, daemon=True).start()
    print(f"[aavc] captures={AAVC_CAPTURES}")
    print(f"[aavc] field={AAVC_FIELD}")
    print(f"[aavc] mission-cmd={MISSION_CMD or '(none — 🚀 button hidden)'}"
          f"{f' [{MISSION_LABEL}]' if MISSION_LABEL else ''}")
    print(f"[aavc] reset-cmd={RESET_CMD or '(none — 🧹 button hidden)'}")

    if args.demo:
        print("[gcs] DEMO mode — ไม่ต่อ FMU จริง (พรีวิวหน้าตา)")
        LINK = Link(None, None, demo=True)
    else:
        url, baud = args.url, args.baud
        if url is None:                       # fall back to config file
            with open(os.path.expanduser(args.config)) as f:
                cfg = yaml.safe_load(f)
            url = cfg["connection"]["url"]
            baud = baud or int(cfg["connection"].get("baud", 921600))
        baud = baud or 115200
        print(f"[gcs] linking to FMU on {url} @ {baud} …")
        LINK = Link(url, baud)
    LINK.start()
    # are we running ON the CM4 itself? (the onboard TELEM2 UART) -> local chirp mode
    on_cm4 = bool(os.environ.get("GCS_ON_CM4")) or (
        not args.demo and url and ("serial0" in url or "ttyAMA" in url))
    CM4MGR = CM4(LINK, local=on_cm4)
    if not args.demo:
        CM4MGR.start()                        # discover/monitor CM4 (or self if local)
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    where = "localhost" if args.host in ("127.0.0.1", "localhost") else args.host
    print(f"[gcs] dashboard ready -> http://{where}:{args.port}  (bind {args.host})")
    srv.serve_forever()


if __name__ == "__main__":
    main()
