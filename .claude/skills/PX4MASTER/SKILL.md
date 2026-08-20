---
name: PX4MASTER
description: Field-proven procedures + bug catalog + check tools for the AAVC EFT X6100 (PX4 6X + CM4). Use when preparing or flying a field day, after any FC reboot or power-cycle, before staging a mission, or when debugging PX4 params, battery gauge, GPS/height, MAVLink link/routing, or SITL quirks on this aircraft. Trigger on /PX4MASTER and proactively whenever those situations appear.
---

# PX4MASTER — the AAVC bird's operating truth

One aircraft, many sharp edges, all of them already cut somebody. This skill
is the record: every documented bug/trap with its detection, and the
procedures that a real field night (2026-08-20, 3 flights, 3 live bugs)
validated end-to-end.

**Aircraft state one-liner (2026-08-20):** EFT X6100 hexa · PX4 1.17 on 6X ·
CM4 companion (WiFi AP `AAVC-DRONE` @10.42.0.1 — the AP FLIES WITH THE
AIRCRAFT) · NOMAD radio = the flight telemetry link · PM02D powers the
FC/avionics ONLY → voltage-only battery gauge (`BAT1_CAPACITY=-1`, never
positive) · 17000 mAh semi-solid pack (endpoints 4.18/3.77) · baro height
reference at the practice site (`EKF2_HGT_REF=0`; the KMITL comp config
deliberately keeps 1) · TFmini lidar aids below 7 m.

## Read the right reference

| Topic | File |
|---|---|
| FC params: motor map, GF_ACTION, INT32 pushes, readback lies, SD/crash dump | `references/fc-params.md` |
| Battery/power: gauge branches, endpoints, sag, PM02D trap, THE energy answer | `references/power-battery.md` |
| Height & GPS: frame walks, baro-vs-GPS ref, per-arm drift, TFmini | `references/height-gps.md` |
| Link & routing: wedges, pymavlink, WiFi-AP-flies-away, port rules, pkill trap | `references/link-routing.md` |
| SITL-only traps | `references/sitl.md` |
| Field operations, procedures, verifier discipline, follow-ups | `references/ops-field.md` |

## The field-day sequence (validated live 2026-08-20)

1. **Deploy + prove it**: `bash cm4/deploy.sh drone@10.42.0.1` then
   `bash cm4/deploy.sh drone@10.42.0.1 --check` → must print MD5 MATCH.
2. **`make field-check`** (or on the CM4: `make preflight` → `make fence-probe`
   → `make alt-watch`, each with `CONNECT=…`). Stops at the first failure:
   - `preflight` — BOARD params (motor map 101..106, AUX 301..304,
     BAT1_*, `SYS_HITL=0`…). Exit 2 = DO NOT FLY.
   - `fence-probe` — FC mission/dataman aliveness (the wedge that ate three
     stagings on 2026-08-20) + SD crash-dump/param-import flags.
   - `alt-watch` — altitude frame stability; MANDATORY after any FC
     reboot/power-cycle. ⚠ both probes bind the orchestrator's udpin port —
     they exit by themselves; never leave one running into staging.
3. **Stage** (🚀). Watch the log for `cached home MSL altitude: X` — X must
   match the GCS ⬆ MSL readout within ~1 m. Then wait for `RC-GO — staged`.
   Not staged? `ssh … 'tail -20 ~/mission/runs/aavc_delivery_mission/orchestrator.log'`
   → look for `unmet critical checks: <link|armable|ekf|home>`.
4. **RC**: ARM (POSCTL held) → flip OFFBOARD within a few seconds → the
   aircraft launches itself. Throttle stick to centre once airborne.
   **Takeover = flip POSCTL, any time.** Landing+disarm ends the mission.
5. **Expect the WiFi to drop at takeoff** — the AP is on the airframe. The
   radio console is the flight link; ssh monitors resume after landing.
6. **Post-flight**: `make verify RUN=runs/<id>/audit.jsonl` (on the CM4 copy),
   log the battery figures (see power-battery.md ground-truth table), UNPLUG
   the pack.

## After any FC reboot / battery re-plug

GPS altitude walks for minutes after a cold start (measured: 12.7→36.6 m on
the parked field). Baro ref removed the in-flight symptom, but the sequence
stands: wait for `make alt-watch` to print STABLE before staging, and never
trust a home-MSL cache taken while the frame was still moving.

## New-bug rule

When a new bug is found and fixed: add its entry (symptom → mechanism → fix →
tool that now catches it) to the matching `references/*.md` IN THE SAME
change that fixes it, and if a check can catch the whole class, add the check
(`tools/px4_type_audit.py` is the template: it was written for one bug and
immediately caught its sibling).
