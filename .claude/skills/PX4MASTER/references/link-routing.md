# Link & routing — who talks to whom, and how it breaks

## Topology (2026-08-20)
FC ttyAMA0@921600 ↔ mavlink-routerd on the CM4 → udp `offboard` 127.0.0.1:14540
(orchestrator/MAVSDK) + udp server `qgc` 14550 (status beacon injects
STATUSTEXT, sysid 1/comp 191). SEPARATE physical path: NOMAD radio (CP2102
@460800 on the laptop) → aavc_gcs.py console. `MAV_1_FORWARD=1` makes PX4
carry the beacon's broadcast across to the radio. MAVLink URLs must be
`udpin://0.0.0.0:14540` — bare `udpin://:port` is rejected.

## FC mission/dataman WEDGE (2026-08-20 — three stagings eaten)
**Symptom:** every geofence upload TIMEOUT/ERROR while heartbeat, telemetry
and params answer fine — on the router AND on direct serial. **Mechanism:**
params live on FRAM; missions/fence go through dataman on the SD; MAVFTP
(also SD) timed out identically. **Fix: power-cycle the FC** (then alt-watch
before staging — see height-gps.md). The mission refusing to fly without a
verified fence is CORRECT fail-closed behaviour; don't fight it.
**Guard:** `make fence-probe` — read-only fence download with a timeout;
silence = "WEDGED, power-cycle". Diagnostic ladder that found it: broadcast
vs targeted responses → param-read control → direct-serial differential.

## ONE owner per udpin port — SO_REUSE* splits datagrams silently
Two MAVSDK binds on 14540 share delivery: sequenced protocols (mission
transfer) break first while params (retried) mostly survive — looks exactly
like a flaky FC. Symptom seen live: "Received ack for not-existing command".
**Rules:** probes (`alt_watch`, `fence_probe`, ad-hoc MAVSDK scripts) run to
completion BEFORE staging; nothing binds 14540 alongside the orchestrator;
after killing a probe over ssh, confirm the remote python actually died.

## WiFi vanishing in flight is NORMAL — the AP is on the aircraft
The CM4 hosts `AAVC-DRONE`; at takeoff the AP flies away and every laptop ssh
session dies. Do NOT diagnose, do NOT panic-message the pilot: the mission is
fully onboard, the NOMAD radio console is the flight link, RC (POSCTL) is the
intervention path. WiFi returns when the aircraft comes home.

## CM4-not-found ≠ drone dead
Twice now the LAPTOP silently roamed to another WiFi while the CM4 stayed put.
Check `iwgetid -r` FIRST; rejoin `AAVC-DRONE`; try
`ssh -i ~/.ssh/cm4_key drone@10.42.0.1` before declaring anything offline.

## 🚀 with no router = one log line and silence (fixed, know the shape)
Pre-2026-08-18 the button started only the orchestrator; on a rebooted CM4 it
died instantly with nothing on screen. `ensure_infra()` now raises
router+camera+beacon idempotently, and `sleep 3` → `0.5` so startup failures
surface in the console instead of after it stops looking. **If 🚀 doesn't
stage:** `ssh … tail -20 …/orchestrator.log` → `unmet critical checks: <name>`
(`link`/`armable`/`ekf`/`home`).

## OFFBOARD refuses without the orchestrator
PX4 only lets the pilot select OFFBOARD if a fresh >2 Hz offboard stream is
already arriving (`prime_offboard_hold` streams ~5 Hz during the RC-GO hold).
**Always 🚀 (staged) before flipping the RC switch.**

## pkill/pgrep -f self-match kills your own shell
`ssh host 'pkill -f mavlink-routerd; …'` — the pattern matches the remote
shell's OWN cmdline; it dies at that statement (exit 255, no output). Hit
AGAIN live 2026-08-20 mid-debug. **Idiom:** bracket one char —
`mavlink-route[r]d`, `orchestrator.mai[n]`. **Guard:**
`tests/test_shell_traps.py` lints every tracked *.sh in `make test`.

## Raw pymavlink on this board: use MAVSDK instead
See fc-params.md (TypeError + silent drops). MAVFTP flakiness is also why
`fence_probe` treats an FTP failure as WARN, never as the wedge verdict.

## UVC camera renumbers on a power glitch
video0 → video1 after a brownout killed the grabber silently (beacon cried
`cam=DEAD`). Fixed: grabbers open `/dev/v4l/by-id/*-video-index0`. If
`cam=DEAD` appears: check the by-id path exists before restarting anything.
