# AAVC 2026 — autonomous egg delivery, ground station and flight code

Everything that flew at the **AAVC 2026** competition (IAAI KMITL, 28–30 August
2026) and everything used to practise for it: the ground-station console, the
autonomous mission that runs on the aircraft, the ArUco pad scanner, the
simulator, and the flight recordings.

An EFT X6100 hexacopter (Pixhawk 6X + Raspberry Pi CM4, **no RTK**) takes off,
flies a mandatory transit corridor, sweeps a search area looking for 1 m landing
pads carrying ArUco markers, **lands on the pad whose marker id it was assigned**,
releases a fragile egg only after touchdown, and flies home — inside a 20-minute
window. The cameras own the final metre, because coarse GPS cannot.

```
AAVC/
├── gcs/          the console — run this on any laptop to become a GCS
├── mission/      the flight code that runs on the aircraft's CM4, + SITL
├── aruco/        standalone ArUco pad scanner
├── flight-data/  the season's recordings (see its README)
└── docs/         how the pieces fit together
```

---

## Make this machine a GCS

Three commands, no aircraft required:

```bash
git clone https://github.com/bonus24847/AAVC.git
cd AAVC
pip install -r gcs/requirements.txt      # pymavlink + pyyaml, that is all
python3 gcs/src/aavc_gcs.py --demo
```

Open **http://localhost:8000**. You get the console with fake telemetry, the
competition field drawn on a satellite map that needs **no internet** (the tiles
are in the repo), the pad picker and the mission panel.

<details>
<summary>ภาษาไทย — ทำเครื่องนี้ให้เป็น GCS</summary>

```bash
git clone https://github.com/bonus24847/AAVC.git
cd AAVC
pip install -r gcs/requirements.txt      # แค่ pymavlink กับ pyyaml
python3 gcs/src/aavc_gcs.py --demo       # เปิด http://localhost:8000
```

แค่นี้ครบ — ไม่ต้องมีโดรน แผนที่ดาวเทียมอยู่ใน repo แล้ว ใช้ได้แม้ไม่มีเน็ต
(สำคัญตอนอยู่สนาม) ต่อของจริงด้วย `--url /dev/ttyACM0 --baud 115200`
หรือให้มันเลือกลิงก์เองด้วย `bash gcs/scripts/aavc_launch.sh` (พอร์ต 8010)

⚠ คอนโซลผูกกับ localhost โดยตั้งใจ — `/api/*` ไม่มีการยืนยันตัวตน ใครที่ต่อ
พอร์ตนี้ได้ สั่งปล่อยไข่หรือล้าง geofence กลางอากาศได้

</details>

Against a real aircraft:

```bash
python3 gcs/src/aavc_gcs.py --url /dev/ttyACM0 --baud 115200 --captures mission/captures
bash gcs/scripts/aavc_launch.sh    # or: auto-picks SITL → radio → FMU-USB → demo (port 8010)
```

⚠ **The console binds to localhost on purpose.** `/api/*` has no authentication —
anything that reaches the port can clear the geofence, remap the RC mode switch,
or fire the egg latches in flight. Widen it only on a network you trust.

Full console documentation: [`gcs/README.md`](gcs/README.md).

---

## How the pieces fit

```
   operator laptop                              the aircraft
┌──────────────────────┐                 ┌──────────────────────────┐
│ gcs/src/aavc_gcs.py  │  MAVLink        │  Raspberry Pi CM4        │
│  console :8000       │◄───radio/USB───►│   orchestrator/          │
│  map · pads · 🚀 GO  │                 │   vision/ · mission_brain│
│                      │  ssh (GO,       │   status beacon          │
│                      │   infra, plan)  │        │ MAVLink         │
│                      │◄───────────────►│        ▼ udp 14540       │
└──────────────────────┘                 │  Pixhawk 6X (PX4 1.17)   │
        ▲                                │   nadir camera · TFmini  │
        │ shared files                   │   4 egg latches AUX4/1/2/3│
        │  captures/pad_assignment.json  └──────────────────────────┘
        │  captures/mission_status.json
        ▼
   mission/captures/
```

- The **console never flies the aircraft.** It stages a mission over ssh; the RC
  pilot arms and flips to OFFBOARD. Every command path is RC-gated by design.
- The **orchestrator** (`mission/orchestrator/`) is the autonomy: a deterministic
  async state machine — no LLM, no cloud, no network in flight (the rules ban it).
- **Vision is classical CV only** — `cv2.aruco` `DICT_4X4_50` decode fused with a
  white-pad blob cue. No neural networks anywhere in the flight path.
- The CM4 raises **its own WiFi AP** so nothing depends on a phone hotspot.

Ports: console **8000** (8010 from `aavc_launch.sh`, 8020 if 8000 is busy) ·
orchestrator MAVLink **udp 14540** · console MAVLink **udp 14550** · mission
dashboard **8765** · CM4 at **10.42.0.1** on its own AP.

---

## The mission

Rules V1.3 plus the 28-Aug event briefing (digest:
[`mission/docs/RULES_AAVC2026.md`](mission/docs/RULES_AAVC2026.md)):

1. **Transit** — take off at Launch & Recovery, fly P1→P2→P3 at strictly 20 m.
   Each coordinate passed is scored.
2. **Search** — sweep the search area, decoding every pad in view into a registry
   keyed by marker id. A pad whose id is assigned to us is served **the moment it
   is confirmed**; the sweep pauses and resumes at the leg it left.
3. **Descend** — an altitude-rung ladder over the pad on the nadir camera, judged
   against the downward **lidar** rather than the GPS/baro height frame.
4. **Land on the pad**, and release the egg **only after touchdown** — an
   id-verified gate refuses to land unless the assigned id was actually decoded
   during the approach.
5. **Egress** P3→P2→P1, land, disarm, resupply. Up to 4 flights in 20 minutes.

Ceiling 30 m AGL (raised from 20 at the 28-Aug briefing); below 10 m only for the
delivery descent.

---

## The season, from the record in this repo

| when | what | evidence here |
|---|---|---|
| 2026-08-03 | first clean autonomous OFFBOARD hover | `gcs/docs/FLIGHT_READINESS.md` |
| 2026-08-20 → 08-26 | KMUTNB rooftop practice; nadir video, first in-flight decodes | `mission/captures/real_flight_KMUTNB_*.mp4`, `decoded_2026-08-26_flight3/` |
| 2026-08-28 | KMITL trial slot; ceiling raised to 30 m, corridor moved off the building, 2/2 delivered at 17:28 | `mission/captures/ulog_2026-08-28/` |
| 2026-08-29 | **scored flight 1** — 3 eggs aboard, egg 1 delivered 0.13 m from centre; pad 5 lost at 2 m five times. Root cause: the height frame had drifted 2.4 m high | `mission/captures/flight_2026-08-29_scored1/` (1412 frames + audit + annotated stills) |
| 2026-08-29 night | Bang Bo school pitch landing tests — the **whole delivery chain flown on the real aircraft**: lidar ladder → touchdown → egg away 3 s later | `mission/captures/ulog_2026-08-29_bangbo/` |
| 2026-08-30 morning | the operator redrew the entire field on the satellite map before the scored flight — new airspace, search area, six keep-out boxes, corridor at E 116, a 29-point sweep | `mission/sitl/kmitl_config.yaml`, `gcs/aavc_field.yaml` |
| 2026-08-30 11:48 | **final scored flight** — armed 11:48:42, ten minutes airborne to 19.4 m, three land-and-climb-out cycles on the pads, battery 100 % → 60 % | `flight-data/gcs-blackbox/flight_20260830_113652.csv` |

The mission-side audit for that last flight was never pulled off the CM4, so the
console's telemetry log above is the record of it that exists.

What the failures taught, written down where it happened:
[`mission/CLAUDE.md`](mission/CLAUDE.md) §0d–§0j is a defect-by-defect account of
the height frame drift, the tip-over mechanism, the battery gauge, and the seven
pre-competition fixes.

---

## Running the simulator

Needs a PX4-Autopilot checkout with Gazebo (`PX4_DIR`, default
`~/PX4-Autopilot-v1.17`, branch `aavc/sitl-v1.17` carrying the
`22000_gz_eft_x6100` airframe).

```bash
cd mission
make install                       # .venv + the lean stack
bash sitl/link_px4_assets.sh       # borrow PX4's propeller meshes (visual only)
bash sitl/launch_stack.sh          # SITL + Gazebo + bridges + console + pads
make run                           # fly the mission on top of it
```

`make help` lists the rest — the pad spawner, the camera bridge, the post-flight
verifier, the field checks. Details in [`mission/README.md`](mission/README.md).

## Deploying to the aircraft

```bash
cd mission
bash cm4/deploy.sh drone@10.42.0.1 --install   # rsync + venv
bash cm4/deploy.sh drone@10.42.0.1 --check     # must print MD5 MATCH
bash cm4/install_icons.sh --apply              # desktop icons for this clone
```

`cm4/setup_ap.sh` turns the CM4 into its own access point. It requires you to
set `AAVC_AP_PASS` — there is deliberately no default password, because this
repo is public and that network is the ssh path to the aircraft.

---

## Fields

| field | flight config | console map |
|---|---|---|
| KMITL competition | `mission/sitl/kmitl_config.yaml` | `gcs/aavc_field.yaml` |
| KMUTNB rooftop practice | `mission/sitl/aavc_config.yaml` | `mission/gcs/kmutnb_field.yaml` |
| Bang Bo school pitch | `mission/sitl/bangbo_config.yaml` | `mission/gcs/bangbo_field.yaml` |

The console's map and the flight config must agree digit for digit — a map that
lies is worse than no map. `gcs/tests/test_field_zones.py` asserts it.

## Tests

```bash
cd mission && make test           # the flight core
pip install pytest && python3 -m pytest gcs/tests    # the console (45 tests)
```

## Safety

This is competition code for an 8.5 kg multirotor, published so others can read
and learn from it. It assumes a safety pilot on the sticks with a kill switch,
an RC-gated arm, and an operator who has read the rules of the site they are
flying at. The command API has no authentication and the console is meant for a
trusted local network. Do not point it at an aircraft you are not authorised and
prepared to fly.

## Licence

See [`LICENSE`](LICENSE). Third-party content that travels with this repo — the
map tiles, the PX4/Gazebo model assets, the competition rulebook — is listed with
its provenance in [`NOTICE.md`](NOTICE.md).
