# flight-data/ — the archive

Recorded output from the 2026 season, kept because it is the evidence behind the
numbers in the rest of the repo. **Nothing here is read at runtime.** The live
tools write elsewhere:

| tool | writes to | this archive |
|---|---|---|
| the console (`gcs/src/aavc_gcs.py`) | `gcs/blackbox/` | `gcs-blackbox/` |
| the orchestrator (`mission/orchestrator/`) | `mission/runs/<id>/` | `mission-runs/` |

A fresh clone starts with those runtime directories empty, so your own flights
never collide with what is stored here.

---

## `gcs-blackbox/` — 142 console telemetry logs, 2026-08-10 → 2026-08-30

One CSV per armed session, written by the console's blackbox thread while the
MAVLink link is up. This is the ground-station side of every flight of the
season, **including the competition day (`flight_20260830_*.csv`)**.

Columns:

```
iso, unix, lat, lon, alt_msl, rel_alt, roll, pitch, yaw, heading,
vx, vy, vz, vbatt, batt_pct, mode, armed, in_air
```

Empty attitude/battery cells mean that MAVLink stream had not arrived yet at
that sample — the writer records what it has rather than dropping the row.

## `mission-runs/` — orchestrator audit trails

| file | what it is |
|---|---|
| `audit.jsonl` (627 lines) | **SITL** run, KMUTNB sky-field map, 2026-08-29. 3 eggs, ids 4/6/3. Ends `truth audit: matched 6/6, ids correct 6/6, served 3, missed 0`. |
| `audit_20260823T212526Z.jsonl` (743 lines) | **SITL** run, KMUTNB map, 2026-08-23. 4 eggs, ids 6/3/5/1. |
| `verified_scored_run_audit.jsonl` (284 lines) | **SITL** run, 2026-08-11. The earliest full-mission audit kept. |
| `orchestrator.log` | loguru log of the 2026-08-24 → 2026-08-29 SITL sessions: geofence upload + read-back, the 30 PX4 tuning params, the vision worker, the ground-truth pad audit. |

These four are **simulator** runs — the `truth`/`matched n/n` lines only exist in
SITL, where the spawner knows where it put the pads. They are kept as the
end-to-end proof of the mission state machine, not as flight records.

---

## Where the REAL flight evidence lives

It is under `mission/captures/`, next to the code that produced it:

| path | what it is |
|---|---|
| `mission/captures/flight_2026-08-29_scored1/` | **The scored KMITL flight.** 1412 nadir frames, a 405-line audit trail, and `report/` with the annotated stills (`pad5_failure.jpg`, `egg1_descent.jpg`, contact sheets). |
| `mission/captures/ulog_2026-08-28/`, `ulog_2026-08-29/` | PX4 ULogs pulled off the FC at KMITL, with `MD5SUMS`. |
| `mission/captures/ulog_2026-08-29_bangbo/` | ULogs from the Bang Bo school-pitch landing tests (6 flights) + `parameters_backup.bson`. |
| `mission/captures/real_flight_KMUTNB_*.mp4` | Nadir camera video from the KMUTNB rooftop practice flights (2026-08-20 and 2026-08-26). |
| `mission/captures/decoded_2026-08-26_flight3/` | The frames where the ArUco id was actually decoded in flight, with the id in the filename. |
| `mission/docs/evidence/` | ULogs kept as evidence for specific fixes (e.g. the 2026-08-24 takeoff gate). |

### Reading the scored flight

`flight_2026-08-29_scored1/audit.jsonl` is the mission's own account of itself:

```
t=  0.0s  FLIGHT 1 START eggs=3 ids=1,5,6 remaining=1200s
t= 68.9s  SWEEP paused: pad=1 confirmed — delivering now
t=118.6s  DELIVERY 1 RELEASE pad=1  →  END delivered=True err=0.13m landed=True
t=223.6s  SWEEP paused: pad=5 confirmed — delivering now
t=334.2s  DELIVERY 2 END delivered=False  notes=acquired conf=0.95;
          lost@2m→climb ×5
t=346.1s  SWEEP resumed at wp 6
```

Egg 1 landed on pad 1 and released after touchdown, 0.13 m from centre. Pad 5
was acquired at 0.95 confidence but lost at 2 m on five consecutive descents —
that failure is what `report/pad5_failure.jpg` shows.

⚠ **The `ts` wall-clock in these files is wrong.** The CM4 has no RTC and boots
without network at the field, so its clock is whatever it last knew (the scored
flight's audit is stamped 2026-08-24). The `t=` seconds are the real timeline;
the directory name carries the real date.

## Licence

Apache License 2.0 — see [`../LICENSE`](../LICENSE) and [`../NOTICE`](../NOTICE).
