# NOTICE — third-party material in this repository

The code and documentation are MIT-licensed (see [`LICENSE`](LICENSE)). Several
non-original files travel with the repository because the system does not work
without them at the field. This is what they are and where they came from, so
anyone reusing this repo can make their own call.

## Map tiles — `gcs/tiles/` (1996 PNG, z9–z19, ~41 MB)

**Esri ArcGIS World Imagery**, fetched from
`https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}`
by `gcs/scripts/prefetch_tiles.py`. Imagery © Esri and its imagery partners
(Maxar, Earthstar Geographics, and the GIS user community).

They are committed because **the console has to draw its map with no internet**:
the competition site has no usable connectivity and the CM4's own WiFi AP has no
uplink, so a tile that is not in the repo is a blank square on the day. They
cover only the three fields this project flew.

Esri's terms of use permit caching for offline use in limited circumstances and
do not clearly permit redistributing a cached tile set. If you are reusing this
repository, the safe path is to delete `gcs/tiles/` and refetch for your own
field:

```bash
python3 gcs/scripts/prefetch_tiles.py --field <your field.yaml>
```

The console works without any tiles — every overlay (pads, zones, the aircraft,
the planned route) renders over blank squares.

## SITL ground texture — `mission/sitl/models/ground_sat/materials/textures/sat_real.png` (3.4 MB)

A satellite image of the KMUTNB rooftop pitch, used as the Gazebo ground plane
so the simulator looks like the practice field. Produced by
`mission/tools/fetch_sat.py`, whose default provider chain starts at
**Google satellite** (`mt1.google.com/vt/lyrs=s`, z20) and falls back to Esri.
`mission/README.md` records this one as Google z20 imagery.

It is **cosmetic** — the simulation flies identically without it. Regenerate it
for your own site rather than reusing this file:

```bash
cd mission && python3 tools/fetch_sat.py --provider esri --zoom 19
```

## PX4 / Gazebo model assets

`mission/sitl/models/eft_x6100_base` draws its propellers from
`model://x500_base/meshes/*.stl`, which belong to
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (BSD-3-Clause).

**They are not redistributed here.** They used to be committed as absolute
symlinks into one machine's PX4 clone; those are gone. `bash
mission/sitl/link_px4_assets.sh` links them from your own `$PX4_DIR`.

The SITL airframe, world and pad models under `mission/sitl/models/` are original
to this project, as is the `22000_gz_eft_x6100` airframe definition (which lives
in a PX4 branch, not here).

## Competition rulebook — `mission/AAVC2026_RulesAndRegulation_V1.3_140769-2.pdf`

The official AAVC 2026 Rules and Regulations, V1.3 (July 2026), published by the
competition organisers. Copyright remains theirs; it is included because every
geometry number and every envelope limit in this repository is derived from it
and a reader cannot check the work without it.
`mission/docs/RULES_AAVC2026.md` is our own digest of it, including the 28-Aug
event-briefing overrides, and is original.

## Flight recordings, photographs and videos

`flight-data/`, `mission/captures/`, `mission/docs/evidence/` and
`mission/docs/report_figures/` are this team's own recordings of its own
aircraft at KMUTNB, KMITL and Bang Bo Witthayakhom School, published under the
same MIT terms as the code.

## Presentation and report

`mission/docs/presentation/` and `mission/docs/report*` are the team's own
submissions for the competition.

## Upstream code

`gcs/src/aavc_gcs.py` is a fork of this author's own Sys_ID Web GCS
(`gcs_server.py`); `gcs/src/pull_log.py` is the MAVFTP log-pull helper from the
same base. Both are the same author's work and carry no third-party licence.
The flight code depends on, but does not vendor, MAVSDK, pymavlink, OpenCV,
NumPy, Pydantic, loguru, PyYAML, FastAPI and uvicorn — each under its own
licence, installed from PyPI.
