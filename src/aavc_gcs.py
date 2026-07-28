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
AAVC_CAPTURES = os.path.join(_HERE, "..", "captures")      # overridden by --captures
AAVC_FIELD = os.path.join(_HERE, "..", "aavc_field.yaml")  # overridden by --field


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
    if gf.get("local_origin"):
        out["home"] = [float(gf["local_origin"][0]), float(gf["local_origin"][1])]
    return out


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


class Link:
    """Owns the MAVLink connection: a background reader + thread-safe senders."""

    def __init__(self, url, baud, demo=False):
        self.demo = demo
        self.url = url
        self.baud = baud
        self.send_lock = threading.Lock()
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
            "local": {"n": None, "e": None},
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
        for mid, us in ((0, 1000000), (1, 1000000), (24, 1000000),
                        (33, 1000000), (65, 1000000), (245, 2000000),
                        (30, 500000), (375, 1000000)):
            try:
                with self.send_lock:
                    self.m.mav.command_long_send(
                        1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0, mid, us, 0, 0, 0, 0, 0)
                time.sleep(0.05)   # space the 8 sends: a burst congests the narrow
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
                    g["alt"] = round(msg.alt / 1000, 1)   # AMSL (m)
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
                buf = self.s["messages"]
                buf.append({"t": time.strftime("%H:%M:%S"), "txt": txt})
                del buf[:-40]
                self._parse_cal(txt)
                # capture GPS driver boot lines ("GPS 1: u-blox …", "GPS: …")
                if "gps" in txt.lower():
                    self.s["gps_detect"]["last_msg"] = txt

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
        snap["zones"] = load_zones()
        origin = self._aavc_origin(snap)
        mission = read_mission_status()
        if self.demo and mission is None:
            mission, origin = self._demo_mission(snap, origin)
        snap["origin"] = origin
        snap["mission"] = mission
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
        mission = {"phase": "SEARCH (demo)",
                   "pads_mapped": {"3": [12.0, 20.0], "5": [-15.0, 35.0]},
                   "assigned": default_assignment(), "delivered": [],
                   "mission_time": 128.0, "updated": time.time(), "age_s": 0.0}
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
.status{display:flex;gap:8px;flex-wrap:wrap;padding:10px 16px;background:#10151c;border-bottom:1px solid #222b36}
.pill{display:flex;flex-direction:column;background:#161b22;border:1px solid #222b36;border-radius:10px;padding:8px 14px;min-width:96px}
.pill b{font-size:26px;font-weight:700;line-height:1.25}
.pill span{font-size:15px;color:#8b98a5;text-transform:uppercase;letter-spacing:.04em}
.main{display:grid;grid-template-columns:1fr;gap:14px;padding:14px;max-width:1400px;margin:auto}
@media(min-width:900px){.main{grid-template-columns:1.7fr 1fr}}
.mapcard,.instr{background:#161b22;border:1px solid #222b36;border-radius:12px;padding:12px}
.mapcard h3,.instr h3{margin:0 0 8px;font-size:16px;text-transform:uppercase;color:#8b98a5;letter-spacing:.05em}
.mapcard{display:flex;flex-direction:column}
#posmap{background:#0d1117;border:1px solid #30363d;border-radius:8px;width:100%;height:auto;display:block}
.mapinfo{font-size:16px;color:#8b98a5;margin-top:8px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:8px}
.mapinfo b{color:#c9d1d9;font-family:ui-monospace,monospace}
.instr{text-align:center}
.ai{width:190px;height:190px;border-radius:50%;overflow:hidden;position:relative;border:2px solid #4a5568;margin:0 auto}
.ai-inner{position:absolute;left:-50%;top:-50%;width:200%;height:200%;transform-origin:50% 50%;background:linear-gradient(#3a86d6 0%,#3a86d6 50%,#8a6a3a 50%,#8a6a3a 100%)}
.ai-inner::after{content:"";position:absolute;left:0;right:0;top:50%;height:2px;background:rgba(255,255,255,.7)}
.ai-mark{position:absolute;left:50%;top:50%;width:54px;height:3px;background:#ffcc00;transform:translate(-50%,-50%);border-radius:2px}
.ai-mark-c{position:absolute;left:50%;top:50%;width:7px;height:7px;background:#ffcc00;border-radius:50%;transform:translate(-50%,-50%)}
.hdg{margin-top:10px;font-size:24px}.hdg b{font-size:30px}
.attxt{font-size:16px;color:#8b98a5;margin-top:2px}
.mlabel{font-size:16px;color:#8b98a5;margin:18px 0 14px;text-transform:uppercase;letter-spacing:.04em}
.motors{display:flex;gap:9px;justify-content:center;align-items:flex-end;height:120px;padding-bottom:18px}
.mbar{width:26px;background:#21262d;border-radius:3px;position:relative;display:flex;flex-direction:column-reverse;height:100%}
.mbar>i{display:block;background:#3fb950;border-radius:3px;min-height:2px}
.mbar>b{position:absolute;bottom:-17px;left:0;right:0;font-size:14px;color:#8b98a5;font-weight:400}
#msgline{padding:8px 16px;font-family:ui-monospace,monospace;font-size:16px;color:#8b98a5;
 background:#0b0e13;border-top:1px solid #222b36;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#lmap{width:100%;height:clamp(320px,55vh,520px);border-radius:8px;display:none}
.dicon{background:none;border:none}
.dm{color:#f0b90b;font-size:30px;line-height:1;text-align:center;transform-origin:50% 50%;text-shadow:0 0 3px #000}
.leaflet-container{background:#0d1117}
.sensors{display:flex;flex-wrap:wrap;gap:7px;justify-content:center;margin-top:14px}
.schip{display:flex;align-items:center;gap:6px;font-size:16px;padding:5px 10px;border-radius:8px;border:1px solid #30363d;background:#0d1117;color:#8b98a5}
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
.padbtn{padding:12px 6px;border:2px solid #30363d;border-radius:9px;background:#0d1117;color:#c9d1d9;
 cursor:pointer;text-align:center;font-weight:700;font-size:18px;user-select:none}
.padbtn.sel{border-color:#238636;background:#0f2417;color:#3fb950}
.padrow{display:flex;justify-content:space-between;align-items:center;margin-top:6px;font-size:16px;color:#8b98a5}
.savebtn{margin-top:10px;padding:12px;border:0;border-radius:8px;font-size:18px;font-weight:700;color:#fff;
 background:#1f6feb;cursor:pointer;width:100%}
.savebtn:disabled{opacity:.5;cursor:not-allowed}
.mprog .row{display:flex;justify-content:space-between;padding:3px 0;font-size:17px}
.mprog b{color:#58a6ff}
.mclock{font-family:ui-monospace,monospace;font-size:23px;font-weight:700}
.leg{font-size:14px;color:#8b98a5;margin-top:8px;display:flex;gap:12px;flex-wrap:wrap}
.leg i{width:11px;height:11px;border-radius:50%;display:inline-block;margin-right:4px;vertical-align:middle}
.padtip{background:transparent;border:0;box-shadow:none;color:#0d1117;font-weight:700;font-size:12px;text-shadow:0 0 2px #fff}
</style></head><body>
<header>
 <span>🛸 AAVC Ground Station</span>
 <span id=mclock class=mclock title="mission time / 20:00 budget">--:--<span style="font-size:14px;color:#8b98a5"> / 20:00</span></span>
 <span id=link><span class=dot style=background:#6e7681></span>connecting…</span>
</header>
<div class=status>
 <div class=pill><b id=mode>–</b><span>Flight Mode</span></div>
 <div class=pill><b id=armstate>–</b><span>Arming</span></div>
 <div class=pill><b id=batt>–</b><span>Voltage</span></div>
 <div class=pill><b id=battpct>–</b><span>Battery %</span></div>
 <div class=pill><b id=gpsfix>–</b><span>GPS</span></div>
 <div class=pill><b id=sats>–</b><span>Sats</span></div>
 <div class=pill><b id=hdgtop>–</b><span>Heading</span></div>
</div>
<div class=main>
 <div class=mapcard>
  <h3>🗺️ แผนที่ตำแหน่ง Drone </h3>
  <div id=lmap></div>
  <canvas id=posmap width=680 height=460></canvas>
  <div class=mapinfo><span>ตำแหน่ง: <b id=posll>–</b></span><span id=mapscale></span></div>
  <div class=gfbar>
   <button id=gfbtn type=button>🛡️ Geofence</button>
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
 <div class=instr>
  <h3>เครื่องวัดการบิน</h3>
  <div class=ai id=ai><div class=ai-inner id=aiInner></div><div class=ai-mark></div><div class=ai-mark-c></div></div>
  <div class=hdg>หัน <b id=hdgval>–</b>° <span id=hdgcard></span></div>
  <div class=attxt>roll <b id=rollv>–</b>° · pitch <b id=pitchv>–</b>°</div>
  <div class=mlabel>ความเร็ว Motor (%)</div>
  <div class=motors id=motors></div>
  <div class=mlabel>สถานะ Sensor</div>
  <div class=sensors id=sensors></div>
 </div>
 <div class=mapcard>
  <h3>🎯 Pad ที่จะส่ง — ติกเลือก (ID 1-6 · เลือกกี่อันก็ได้)</h3>
  <div class=padgrid id=padgrid></div>
  <div class=padrow><span>เลือกแล้ว: <b id=selcount style=color:#3fb950>0</b> / 6</span></div>
  <button class=savebtn id=savebtn onclick=saveAssign()>💾 บันทึก assignment (ก่อนเริ่ม mission)</button>
  <div id=savemsg style="font-size:15px;color:#8b98a5;margin-top:8px"></div>
  <div class=leg>
   <span><i style=background:#3fb950></i>assigned</span>
   <span><i style=background:#58a6ff></i>mapped (เจอแล้ว)</span>
   <span><i style=background:#f85149></i>delivered</span>
   <span><i style=background:#8b98a5></i>ยังไม่เจอ</span>
  </div>
 </div>
 <div class=mapcard>
  <h3>📋 Mission progress</h3>
  <div class=mprog id=mprog><span style=color:#8b98a5>idle — mission ยังไม่รัน</span></div>
 </div>
</div>
<div id=msgline>—</div>
<script>
function card16(h){var d=['N','NE','E','SE','S','SW','W','NW'];return d[Math.round(h/45)%8];}
var mapHome=null,mapTrack=[];
var lmap=null,lmarker=null,ltrack=null,lhome=null,lgfcircle=null,gfWant=null,mapReady=false;
var SEL=[],PADS_INIT=false,lpads={},lzones=[],lhomeMarker=null,MCLOCK={base:null,at:0,done:false};
var frect=null,fmarkers=[],fcorners=[],lupfence=null;
var SLAB={gyro:'Gyro',accel:'Accel',mag:'Mag',baro:'Baro',gps:'GPS',rc:'RC',ahrs:'AHRS',battery:'Batt'};
var SORD=['gyro','accel','mag','baro','gps','rc','ahrs','battery'];
function droneIcon(){return L.divIcon({className:'dicon',html:'<div class=dm>▲</div>',iconSize:[26,26],iconAnchor:[13,13]});}
function initMap(){
 if(mapReady||typeof L==='undefined')return;
 var el=document.getElementById('lmap');if(!el)return;
 var cv=document.getElementById('posmap');if(cv)cv.style.display='none';
 el.style.display='block';
 lmap=L.map(el,{zoomControl:true,attributionControl:true}).setView([13.736,100.523],16);
 L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'© OpenStreetMap'}).addTo(lmap);
 ltrack=L.polyline([],{color:'#388bfd',weight:3}).addTo(lmap);
 setTimeout(function(){if(lmap)lmap.invalidateSize()},200);
 mapReady=true;
}
function renderSensors(s){
 var box=document.getElementById('sensors');if(!box)return;
 var pres=s.present||{},hl=s.health||{},g=s.gps||{},html='';
 for(var i=0;i<SORD.length;i++){var k=SORD[i],p=pres[k],h=hl[k],cls,txt;
  if(k==='gps'){var fx=g.fix||0;if(fx>=3){cls='sok';txt='3D';}else if(fx>=2){cls='sok';txt='2D';}else{cls='sna';txt='no fix';}}
  else if(h){cls='sok';txt='OK';}          // health-first: AHRS is present=False/health=True on PX4
  else if(p){cls='sbad';txt='BAD';}
  else{cls='sna';txt='n/a';}
  html+='<div class="schip '+cls+'" title="'+SLAB[k]+': '+txt+'"><span class=sdot></span>'+SLAB[k]+'</div>';
 }
 box.innerHTML=html;
}
function updateMap(s){
 var g=s.gps||{}, ll=document.getElementById('posll');
 var msc=document.getElementById('mapscale'); if(msc) msc.textContent='แผนที่ OSM (ติดตามสด)';
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
 aavcMap(s);
 if(ll)ll.textContent=g.lat.toFixed(6)+', '+g.lon.toFixed(6);
}
function renderInsti(s){
 var a=s.att||{};
 var roll=a.roll==null?0:a.roll, pitch=a.pitch==null?0:a.pitch;
 var inner=document.getElementById('aiInner');
 if(inner) inner.style.transform='rotate('+(-roll)+'deg) translateY('+(pitch*1.8)+'px)';
 var hv=document.getElementById('hdgval'); if(hv) hv.textContent=a.heading==null?'–':Math.round(a.heading);
 var hc=document.getElementById('hdgcard'); if(hc) hc.textContent=a.heading==null?'':card16(a.heading);
 var rv=document.getElementById('rollv'); if(rv) rv.textContent=a.roll==null?'–':a.roll;
 var pv=document.getElementById('pitchv'); if(pv) pv.textContent=a.pitch==null?'–':a.pitch;
 var box=document.getElementById('motors');
 if(box){var ms=s.motors||[];
  if(box.children.length!==ms.length){box.innerHTML='';for(var i=0;i<ms.length;i++){var d=document.createElement('div');d.className='mbar';d.innerHTML='<i></i><b>M'+(i+1)+'</b>';box.appendChild(d);}}
  for(var j=0;j<ms.length;j++){var pct=Math.round(ms[j]*100);var bar=box.children[j].firstChild;bar.style.height=pct+'%';bar.style.background=pct>85?'#f85149':pct>60?'#d29922':'#3fb950';box.children[j].title=pct+'%';}}
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
function renderPads(){
 var g=document.getElementById('padgrid');if(!g)return;g.innerHTML='';
 [1,2,3,4,5,6].forEach(function(id){
  var d=document.createElement('div');
  d.className='padbtn'+(SEL.indexOf(id)>=0?' sel':'');
  d.textContent='ID '+id;
  d.onclick=function(){var i=SEL.indexOf(id);if(i>=0)SEL.splice(i,1);else SEL.push(id);
   SEL.sort(function(a,b){return a-b});renderPads();updateMap(window.LAST||{});};
  g.appendChild(d);
 });
 var sc=document.getElementById('selcount');if(sc)sc.textContent=SEL.length;
}
async function saveAssign(){
 var b=document.getElementById('savebtn'),m=document.getElementById('savemsg');b.disabled=true;
 try{var r=await fetch('/api/assign',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ids:SEL})});
  var j=await r.json();
  m.textContent=j.ok?('✅ บันทึกแล้ว: ['+j.ids.join(', ')+'] — mission จะใช้ค่านี้ตอนเริ่ม'):('❌ '+(j.err||'error'));}
 catch(e){m.textContent='❌ '+e;}
 b.disabled=false;
}
function fmtClk(s){s=Math.max(0,Math.floor(s));var m=Math.floor(s/60),ss=s%60;return (m<10?'0':'')+m+':'+(ss<10?'0':'')+ss;}
function updClock(){
 var el=document.getElementById('mclock');if(!el)return;
 var suf='<span style="font-size:14px;color:#8b98a5"> / 20:00</span>';
 if(MCLOCK.base==null){el.innerHTML='--:--'+suf;return;}
 var t=MCLOCK.done?MCLOCK.base:MCLOCK.base+(Date.now()-MCLOCK.at)/1000;
 var col=t>=1200?'#f85149':(t>=1080?'#d29922':'#3fb950');
 el.innerHTML='<span style="color:'+col+'">'+fmtClk(t)+'</span>'+suf;
}
setInterval(updClock,250);
function renderMission(ms){
 var el=document.getElementById('mprog');if(!el)return;
 if(!ms){el.innerHTML='<span style=color:#8b98a5>idle — mission ยังไม่รัน</span>';MCLOCK.base=null;return;}
 var mapped=Object.keys(ms.pads_mapped||{}).length,deliv=(ms.delivered||[]).length;
 var asg=(ms.assigned&&ms.assigned.length)?ms.assigned.length:SEL.length;
 var mt=ms.mission_time!=null?ms.mission_time.toFixed(0)+' s':'–';
 var stale=(ms.age_s>10)?' <span class=warn>(stale '+ms.age_s+'s)</span>':'';
 el.innerHTML='<div class=row><span>Phase</span><span><b>'+(ms.phase||'–')+'</b>'+stale+'</span></div>'+
  '<div class=row><span>Pads เจอแล้ว</span><span><b>'+mapped+'</b> / 6</span></div>'+
  '<div class=row><span>ส่งแล้ว</span><span><b>'+deliv+'</b> / '+asg+'  ['+(ms.delivered||[]).join(', ')+']</span></div>'+
  '<div class=row><span>เวลา</span><span><b>'+mt+'</b></span></div>';
 if(ms.mission_time!=null){MCLOCK.base=ms.mission_time;MCLOCK.at=Date.now();MCLOCK.done=(ms.phase==='done');}else MCLOCK.base=null;
}
function aavcEN(e,n,lat0,lon0){var R=6378137.0;
 return [lat0+(n/R)*180/Math.PI, lon0+(e/(R*Math.cos(lat0*Math.PI/180)))*180/Math.PI];}
function aavcMap(s){
 if(!mapReady||typeof L==='undefined')return;
 var z=s.zones||{};
 if(!lzones.length&&(z.airspace||z.search)){
  if(z.airspace&&z.airspace.length>=3)lzones.push(L.polygon(z.airspace,{color:'#f85149',weight:2,fill:false,dashArray:'8 6'}).addTo(lmap).bindTooltip('controlled airspace'));
  if(z.search&&z.search.length>=3)lzones.push(L.polygon(z.search,{color:'#f0d000',weight:2,fill:false}).addTo(lmap).bindTooltip('search area'));
  if(z.home)lhomeMarker=L.circleMarker(z.home,{radius:6,color:'#fff',fillColor:'#2ea043',fillOpacity:1,weight:2}).addTo(lmap).bindTooltip('HOME (P1)');
 }
 var ms=s.mission||{},pm=ms.pads_mapped||{},o=s.origin||{},deliv=ms.delivered||[];
 for(var id in pm){
  if(o.lat==null)break;
  var en=pm[id],pos=aavcEN(en[0],en[1],o.lat,o.lon),pid=parseInt(id);
  var col=(deliv.indexOf(pid)>=0)?'#f85149':(SEL.indexOf(pid)>=0?'#3fb950':'#58a6ff');
  if(!lpads[id])lpads[id]=L.circleMarker(pos,{radius:11,color:'#0d1117',weight:2,fillColor:col,fillOpacity:.95}).addTo(lmap)
    .bindTooltip('Pad '+id,{permanent:true,direction:'center',className:'padtip'});
  else{lpads[id].setLatLng(pos);lpads[id].setStyle({fillColor:col});}
 }
}
async function tick(){
 var s;try{s=await(await fetch('/api/status')).json()}catch(e){
  document.getElementById('link').innerHTML='<span class=dot style=background:#f85149></span>server หลุด';return}
 window.LAST=s;
 var L=document.getElementById('link');
 L.innerHTML='<span class=dot style=background:'+(s.link?'#3fb950':'#f85149')+'></span>'+(s.link?'online':'no signal');
 document.getElementById('mode').textContent=s.mode||'–';
 document.getElementById('armstate').innerHTML=s.armed?'<span class=bad>ARMED</span>':'<span class=ok>DISARM</span>';
 var b=s.batt||{};
 var bc=(b.pct!=null&&b.pct<20)?'bad':(b.pct!=null&&b.pct<40)?'warn':'ok';
 document.getElementById('batt').innerHTML=b.volt==null?'–':'<span class="'+bc+'">'+b.volt+'V</span>';
 document.getElementById('battpct').innerHTML=b.pct==null?'–':'<span class="'+bc+'">'+b.pct+'%</span>';
 var g=s.gps||{};
 document.getElementById('gpsfix').innerHTML='<span class="'+(g.fix>=3?'ok':'bad')+'">'+(g.fix_str||'–')+'</span>';
 document.getElementById('sats').textContent=g.sats==null?'–':g.sats;
 var a=s.att||{};
 document.getElementById('hdgtop').textContent=a.heading==null?'–':Math.round(a.heading)+'° '+card16(a.heading);
 try{renderInsti(s)}catch(e){}
 var msgs=s.messages||[];
 if(msgs.length){var m=msgs[msgs.length-1];document.getElementById('msgline').textContent='['+m.t+'] '+m.txt;}
 var gfn=document.getElementById('gfnow'); if(gfn) gfn.textContent=gfFmt(s.geofence);
 var lk=s.link_kind||'',wired=(lk==='usb'||lk==='cm4'||lk==='other'),upB=document.getElementById('gfup');
 if(upB){upB.disabled=!wired;upB.textContent=wired?'⬆️ Upload สี่เหลี่ยม':'🔒 Upload (ต่อ USB ก่อน)';upB.title=wired?'':'เสียบ FC USB ก่อน — upload สี่เหลี่ยมบนวิทยุ ELRS ไม่ชัวร์';}
 if(!PADS_INIT&&s.assignment){SEL=(s.assignment||[]).slice();PADS_INIT=true;renderPads();}
 renderMission(s.mission);
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
setInterval(tick,600);tick();
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
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default="config/real.yaml")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--demo", action="store_true",
                    help="serve sample telemetry with no FMU (preview)")
    ap.add_argument("--url", help="MAVLink endpoint override (e.g. /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, help="baud override")
    ap.add_argument("--captures",
                    help="dir for pad_assignment.json (out) + mission_status.json (in); "
                         "point at the touch-and-go mission's captures/ when testing")
    ap.add_argument("--field",
                    help="field yaml: GPS zones + pad IDs (default: bundled aavc_field.yaml)")
    args = ap.parse_args()
    if args.captures:
        AAVC_CAPTURES = os.path.abspath(os.path.expanduser(args.captures))
    if args.field:
        AAVC_FIELD = os.path.abspath(os.path.expanduser(args.field))
    print(f"[aavc] captures={AAVC_CAPTURES}")
    print(f"[aavc] field={AAVC_FIELD}")

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
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    where = "localhost" if args.demo else "<host-ip>"
    print(f"[gcs] dashboard ready -> http://{where}:{args.port}")
    srv.serve_forever()


if __name__ == "__main__":
    main()
