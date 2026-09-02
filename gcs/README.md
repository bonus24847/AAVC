# gcs/ — the AAVC 2026 ground station

The console that flew the **AAVC 2026** mission: a self-contained Python web GCS
(`http.server` + `pymavlink`, no Qt, no QGroundControl). Live telemetry, the
aircraft and the scanned ArUco pads on a map that works with **no internet**, and
the buttons that stage a mission and release the egg latches.

> **Built on the Sys_ID Web GCS** — the MAVLink link and the RC-only safety model
> were copied unchanged; the connection/command code is never rewritten.
> `src/aavc_gcs.py` is a standalone fork of that base with the AAVC features on
> top. (That base repo is private; nothing here depends on it.)

Over a plain telemetry viewer it adds:
- **Pad-assignment selector** — tick which ArUco pads (ids 0–6, any number) the
  drone should service → writes `captures/pad_assignment.json`, which the mission
  reads at startup.
- **Detected pads on a real map** — the mission publishes
  `captures/mission_status.json` (`pads_mapped: {id:[e,n]}`) as it localizes each
  pad; the console converts those to lat/lon via the EKF/GPS origin and plots
  them colour-coded (assigned / mapped / delivered), with the rulebook zones and
  the planned route.
- **Mission panel + 20:00 budget clock**, a pre-flight checklist, and a
  slide-to-confirm launch button.
- **Flight interlocks** — FC-state writes and the payload buttons are gated on
  the *aircraft's* armed/in-air state, not on this console's own ssh child.

---

## Turn this machine into a GCS

```bash
git clone https://github.com/bonus24847/AAVC.git
cd AAVC
pip install -r gcs/requirements.txt          # pymavlink + pyyaml
python3 gcs/src/aavc_gcs.py --demo
```

Then open **http://localhost:8000** — the map, the zones and fake telemetry, no
aircraft needed. That is the whole install; it was verified from a fresh clone at
an unrelated path with an empty environment.

Live, against the aircraft:

```bash
python3 gcs/src/aavc_gcs.py --url /dev/ttyACM0 --baud 115200 \
        --captures ../mission/captures
```

Or let the launcher pick the link for you — SITL → NOMAD radio → FMU-USB → demo —
and open the browser (⚠ **this one serves on 8010**, not 8000):

```bash
bash gcs/scripts/aavc_launch.sh          # AAVC_SITL=1 to attach to SITL udp:14550
```

Whole-stack entry point, for the simulator or the real bird:

```bash
bash gcs/gcs_launch.sh sim               # SITL + bridges + console (KMUTNB field)
bash gcs/gcs_launch.sh real drone@10.42.0.1
bash gcs/gcs_launch.sh gui               # click-through icon flow
bash gcs/gcs_launch.sh demo
```

### Ports

| how you start it | port |
|---|---|
| `python3 src/aavc_gcs.py` (default `--port`) | **8000** |
| `scripts/aavc_launch.sh` | **8010** |
| `mission/sitl/launch_stack.sh` | 8000, falling back to 8020 if busy |

### ⚠ It binds to localhost on purpose

`/api/*` has **no authentication**. Anything that can reach the port can clear
the geofence, remap the RC mode switch, or fire the egg latches while the
aircraft is flying. `--host 0.0.0.0` is for a network you fully trust and
nothing else.

---

## Fields

`aavc_field.yaml` (bundled, the default) is the **KMITL competition** field as
the operator redrew it on the satellite map the morning of the scored day —
digit-for-digit the same numbers as `../mission/sitl/kmitl_config.yaml`, which is
pinned by `tests/test_field_zones.py`. Two more live next to the flight code:

| field | file | `--field` / registry name |
|---|---|---|
| KMITL competition | `gcs/aavc_field.yaml` | `aavc` |
| KMUTNB rooftop practice | `../mission/gcs/kmutnb_field.yaml` | `kmutnb` |
| Bang Bo school pitch | `../mission/gcs/bangbo_field.yaml` | `bangbo` |

`missions.yaml` is the registry behind the in-UI mission switcher: each entry
wires a field file, a captures directory and the 🚀 command for that field.
Paths in it are written `{repo}/…` and expanded against the repo root at load, so
a clone works wherever it lands.

## The offline map

`tiles/` holds 1996 pre-fetched **Esri World Imagery** tiles (z9–z19) covering
the three fields. The console serves the map from there, so it draws with no
internet at all — which is the condition at the field. To add a field:

```bash
python3 gcs/scripts/prefetch_tiles.py --field ../mission/gcs/kmutnb_field.yaml
```

Run that while you still have internet. A field with no tiles still renders every
overlay (pads, zones, drone) over blank squares.

## Tests

```bash
pip install pytest        # not in requirements.txt — the console does not need it
python3 -m pytest gcs/tests
```

45 tests. `test_field_zones.py` reads `../mission/sitl/kmitl_config.yaml` and
checks the console's map against the flight config digit for digit.

## Notes for whoever continues this

- One file: **`src/aavc_gcs.py`** — a backend `Link` class (MAVLink parsing,
  param and geofence workers) plus the whole web page in an inline
  `PAGE = """..."""` string. No build step.
- Speaks **MAVLink 2 + the `common` dialect**. After editing the inline
  `<script>`, always `node --check` the **served** page (`curl localhost:8000/`),
  not the source — a `\n` inside a JS string literal in a Python triple-quoted
  string becomes a real newline in the served JS and breaks it.
- `pull_log.py` (the MAVFTP log-pull helper) sits beside it and is imported at
  runtime.
- The console writes its own telemetry blackbox to `gcs/blackbox/`. The season's
  recordings are archived under `../flight-data/`.

## Aircraft bring-up

The commissioning history of the EFT X6100 hexacopter — the FMU calibration,
params and CM4 setup that made it fly, plus the operating gotchas — is in
[`docs/FLIGHT_READINESS.md`](docs/FLIGHT_READINESS.md). First clean autonomous
OFFBOARD hover: **2026-08-03**.

## Requirements

Python 3.8+, `pymavlink`, `pyyaml`, a modern browser (Chromium recommended).
Developed and flown on Python 3.12 / Ubuntu.

## Licence

Apache License 2.0 — see [`../LICENSE`](../LICENSE) and [`../NOTICE`](../NOTICE).
