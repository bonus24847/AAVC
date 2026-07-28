# Sys_ID Web GCS

A lightweight, self-contained **web Ground Control Station** for a PX4 drone —
pure Python (`http.server` + `pymavlink`), no Qt / no QGroundControl. Open it in
any browser (laptop or a phone on the same network) to watch live telemetry, see
the drone on an OpenStreetMap map, set geofences, and record a flight log.

Built for the EFT X6100 hexacopter project, but works with any PX4 vehicle.

## Features
- **Live telemetry** — flight mode, arming, battery (V + %), GPS (fix / sats /
  HDOP / position), RC, and colour-coded **sensor-health chips**
  (gyro / accel / mag / baro / gps / rc / ahrs / battery).
- **Instruments** — artificial-horizon disc, heading, per-motor output bars.
- **Map** — Leaflet + OSM tiles, live drone marker + track, geofence overlay
  (falls back to a canvas grid plot if there is no internet for tiles).
- **Geofence tools**
  - *Circle* — radius + ceiling + breach action via the `GF_MAX_*` params
    (reliable over any link, including the radio).
  - *Rectangle / polygon* — uploaded via the MISSION protocol
    (`MAV_MISSION_TYPE_FENCE`). **Locked to a wired link (FMU USB / CM4)** because
    the handshake is flaky over the narrow ELRS radio.
- **Log pull** — download the newest `.ulg` off the FMU SD card over MAVFTP
  (the FMU's own `.ulg` is the log of record — the GCS does not record telemetry).

## Run
```bash
pip install -r requirements.txt

# preview with fake data (no FMU needed):
python src/gcs_server.py --demo

# NOMAD / ELRS radio (Silicon Labs CP2102 USB):
python src/gcs_server.py --url /dev/ttyUSB0 --baud 460800

# FMU directly over USB:
python src/gcs_server.py --url /dev/ttyACM0 --baud 115200
```
Then open **http://localhost:8000**.

Or use the launcher, which auto-picks the link (radio → FMU-USB → demo) and opens
the browser:
```bash
bash scripts/gcs_launch.sh
```

## Notes for whoever continues this
- It's basically **one file: `src/gcs_server.py`** — a backend `Link` class
  (MAVLink parsing in `_handle`, param + geofence/fence workers) plus the entire
  web page held in a `PAGE = """..."""` string (inline HTML / CSS / JS, no build
  step). `pull_log.py` is only used by the log-pull button.
- It speaks **MAVLink 2 + the `common` dialect** (set at import) so it can tag a
  fence upload with `mission_type = FENCE`.
- After editing the inline `<script>`, always `node --check` the **served** page
  (`curl localhost:8000/`), not the source: a `\n` written inside a JS string
  literal in the Python triple-quoted string becomes a *real* newline in the
  served JS and breaks the whole script.
- Map tiles need internet; everything else works offline.

## Requirements
Python 3.8+, `pymavlink`, `pyyaml`, and a modern browser (Chromium recommended).
