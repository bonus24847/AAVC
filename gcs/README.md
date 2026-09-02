# AAVC 2026 competition GCS

The ground station for the **AAVC 2026** touch-and-go mission — a self-contained
Python web GCS (`http.server` + `pymavlink`, no Qt / QGroundControl). Watch live
telemetry, see the drone **and the scanned ArUco pads** on an OpenStreetMap map, and
pick which pads the drone should service.

> **Built on the proven [Sys_ID Web GCS](https://github.com/bonus24847/sysid-web-gcs)** —
> the MAVLink link + RC-only safety model are copied **unchanged** (the
> connection/command code is never rewritten). `src/aavc_gcs.py` is a standalone fork
> of that base with the AAVC features added on top.

It adds, over the base GCS:
- **Pad-assignment selector** — tick which ArUco pads (IDs 1–6, **any number**) the
  drone should service → writes `captures/pad_assignment.json`, which the mission reads
  at startup.
- **Detected pads on a real OSM map** — as the mission localizes each pad it publishes
  `captures/mission_status.json` (`pads_mapped: {id:[e,n]}`); the GCS converts those to
  lat/lon (via the EKF/GPS origin) and plots them colour-coded (assigned / mapped /
  delivered), alongside the rulebook zones (controlled airspace + search area) and the
  live drone.
- **Mission panel + 20:00 budget clock**.

The companion **ArUco pad scanner** (camera → marker IDs, served to the CM4) lives in
its own repo: **[`aavc-aruco`](https://github.com/bonus24847/aavc-aruco)**.

## Run
```bash
pip install -r requirements.txt

python src/aavc_gcs.py --demo                     # preview: fake telemetry + demo pads
python src/aavc_gcs.py --url /dev/ttyACM0 --baud 115200 \
    --captures ../touch_and_go_for_race/captures  # live, sharing files with the mission
```
Then open **http://localhost:8010**. Field zones + default pads come from
**`aavc_field.yaml`** (`--field` to point elsewhere).

Or use the launcher (auto-picks SITL → radio → FMU-USB → demo, opens the browser):
```bash
bash scripts/aavc_launch.sh          # AAVC_SITL=1 to attach to SITL udp:14550
```

## Aircraft bring-up
The commissioning history of the EFT X6100 hexacopter — the FMU calibration / params /
CM4 setup that made it fly, plus the operating gotchas — is in
**[`docs/FLIGHT_READINESS.md`](docs/FLIGHT_READINESS.md)**. First clean autonomous
OFFBOARD hover flew **2026-08-03**.

## Notes for whoever continues this
- One file: **`src/aavc_gcs.py`** (a fork of the base `gcs_server.py`) — a backend
  `Link` class (MAVLink parsing + param / geofence workers) plus the whole web page in
  an inline `PAGE = """..."""` string (no build step). `pull_log.py` (the MAVFTP
  log-pull helper) is bundled alongside it and imported at runtime.
- Speaks **MAVLink 2 + the `common` dialect**. After editing the inline `<script>`,
  always `node --check` the **served** page (`curl localhost:8010/`), not the source —
  a `\n` inside a JS string literal in the Python triple-quoted string becomes a real
  newline in the served JS and breaks the script.
- Map tiles need internet; the overlays (pads, zones, drone) render regardless.

## Requirements
Python 3.8+, `pymavlink`, `pyyaml`, and a modern browser (Chromium recommended).
