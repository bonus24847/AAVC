# Height & GPS — the frame is the flight

## The 2026-08-20 double incident (why practice flies baro ref)
GPS vertical at the KMUTNB field WALKS: the parked field read 12.7 → 36.6 →
13.3 m MSL across one evening, and the EKF stepped +1.6/+1.3 m INSIDE a 30 s
flight. Same root, two opposite failures the same night:
- **Flight 1**: home MSL cached at 13.0 while the frame had moved ~24 m → the
  "8.5 m AGL" transit target = 15 m underground in the new frame → the
  aircraft obediently sank into the field (pilot takeover at t≈39 s).
- **Flight 2**: frame stepped mid-air → reported AGL inflated 8.7→10.3→12.0
  while the aircraft physically held ~8.5 (transit passes 1.7/1.8 m prove it)
  → the 10 m practice-ceiling watchdog RTH'd a healthy flight. The estimate
  jumps have a signature: +1.6 m in 0.9 s then FLAT — a controller overshoot
  would have corrected back down; an EKF reset carries the setpoint through.
**Fix:** `EKF2_HGT_REF=0` (baro) for the practice site — step-free over a
<20 min flight; GPS keeps horizontal; TFmini still pins the final metres.
The KMITL comp config DELIBERATELY keeps 1 (GPS) — 20 m envelope, own
decision pending (follow-ups in ops-field.md). Flight 3 after the change:
transit 3/3 at 1.4-2.0 m, zero ceiling events.

## Post-reboot protocol (MANDATORY)
After ANY FC reboot/power-cycle: `make alt-watch` until STABLE (4 samples
within 0.8 m) before staging. Baro settles in seconds, GPS position still
needs its fix; the gate covers both and costs ~1 min.

## How mission altitude is referenced (operator explainer)
Plan altitudes are AGL above the ARM point. `commands.py::goto` converts:
target MSL = cached home MSL + AGL; PX4 re-captures home at every arm and
`_refresh_home_alt` re-caches at takeoff. NOT terrain-following. The whole
chain has ONE assumption: the frame at cache time = the frame in flight
(exactly what alt-watch + baro ref protect). The GCS status bar shows
⬆ AGL · MSL — a parked MSL that disagrees with the staging log's
`cached home MSL` line is a NO-GO.

## The parked MSL reads ~40 m HIGH — and the baro is CORRECT (2026-08-26)
Operator flagged `MSL 54` parked on the KMUTNB rooftop pitch, surveyed at 15 m
(`tools/gen_geo.py GROUND_ALT_M`). Nothing is broken: PX4 reports **pressure
altitude referenced to `SENS_BARO_QNH`**, and that param has never been set off
its 1013.25 default.

| step | value |
|---|---|
| sensor RAW (`SCALED_PRESSURE` carries `sensor_baro.pressure`, PRE-calibration) | 1004.19 hPa |
| what a 15 m deck SHOULD read under the day's real QNH 1006 (METAR VTBD/VTBS 260800Z) | **1004.2 hPa** ✅ |
| `+ CAL_BARO0_OFF = -238.84 Pa` -> the pressure PX4 actually converts | 1006.58 hPa |
| `/ SENS_BARO_QNH = 1013.25` -> `ALTITUDE.altitude_monotonic` | **55.46 m** (predicted 55.68) |
| EKF (`EKF2_HGT_REF=0`) -> `GLOBAL_POSITION_INT.alt` | **53.41 m** |

The error is two terms: **QNH +60.5 m** and **`CAL_BARO0_OFF` -20.0 m**, net
**+40.7 m**. Set both right (QNH 1006, offset 0) and the same sensor reads
**15.19 m** against the 15 m surveyed deck — sensor, METAR and survey agree
three ways, so the barometer is accurate to 0.01 hPa (~10 cm). ⚠ **KMITL will
show the same kind of offset** (QNH is never 1013.25): expect a wrong-looking
MSL on the GCS at the safety inspection and do not chase it.

⚠ **Do NOT "fix" QNH alone.** 1006 with the offset still in place reads
**-4.85 m** — wrong by 20 m the other way. And moving the frame 60 m
mid-session is precisely what dropped flight 1 on 2026-08-20. Nothing flown
reads absolute MSL (plan altitudes are AGL above the arm point, above), the
bias cancels on both sides, and `altitude_relative` read -0.01 m on the deck.
If the absolute number is ever wanted, set BOTH at the START of a session and
re-run `make alt-watch`. `local.z` = -41 m parked is the same story, not a
second fault: `altitude_amsl = -z + ref_alt` with `ref_alt` = 12.35 m, the GPS
value the EKF origin was set from.

**The useful half of that session:** `EKF2_GPS_CTRL=7` means GPS **altitude**
fusion (bit 1) is ON even under baro ref — it shows as the EKF sitting 2.05 m
below the raw baro, a bounded pull the baro-bias state absorbs, NOT tracking.
Parked 179 s / 90 samples: **EKF MSL swing 0.00 m vs GPS MSL swing 6.5 m**,
GPS AMSL walking 39.9 -> 19.2 m across the session (same pathology as
2026-08-20's 12.7 -> 36.6 -> 13.3). Horizontally the same receiver gave CEP95
**4.03 / 1.76 / 2.93 m** on three 90 s `tools/gps_bench.py` runs at one spot.
This is the first time the 2026-08-20 baro-ref decision was watched PROTECTING
a flight-ready aircraft in real time.

⚠ **OPEN from that session, not chased:** the two barometers disagree by
**1.7 hPa (~14 m)** raw (1004.19 vs 1005.90) at sensor temperatures of
**51.8 / 55.0 C**, and nobody knows where `CAL_BARO0_OFF = -238.84 Pa` came
from. A stored offset encodes whatever altitude was assumed when it was
calibrated (`VehicleAirData.cpp` calibrates `delta_alt` against a TARGET
altitude). Harmless while it only shifts a frame nothing reads absolutely —
but a THERMAL component would make that frame walk in flight, which is the one
failure mode this whole file exists about. Cheapest test: log
`ALTITUDE.altitude_monotonic` against `SCALED_PRESSURE.temperature` from cold
boot through warm-up and see if the two move together.

To re-check: the console owns the radio, so read the FC over the CM4's own
router instead of fighting for the port — `ALTITUDE` (msg 141,
`altitude_monotonic` = baro) + `SCALED_PRESSURE` via
`udpout:127.0.0.1:14550` on the CM4, and today's QNH from any VTBD/VTBS METAR.

## Baro + lidar division of labour
Baro owns the absolute frame all flight (GPS altitude is still FUSED —
`EKF2_GPS_CTRL=7` bit 1 — but only as a bounded pull, measured 2.05 m
against a 21 m GPS walk, above). TFmini (0.4-12 m) joins the estimate
only below `EKF2_RNG_A_HMAX=7.0` AND under 1 m/s (`EKF2_RNG_CTRL=1`
conditional aiding) — i.e. exactly the slow pad-approach and touchdown.
HMAX=7 is deliberate: PX4's default 5.0 sits ON the 5 m descent rung, so the
height source would swap mid-aim. NOT `EKF2_HGT_REF=2` (range ref): the
origin would ride ground level — a shed box under the beam becomes "down".
⚠ Parked TFmini reads ("0.50 m" steady) are below-min-range garbage — the
sensor sits 3.5 cm off the ground; trust it only in flight.

## Per-arm altitude drift (OPEN RISK — quad-era, much smaller on the hexa)
Held transit altitude wandered ±0.9 m between sorties on the quad (constant
within a sortie, random sign across) → ceiling FAILs at 20.69/20.87 m. Hexa's
first G4: 0.18 m spread — one run, not closed. Spec:
docs/superpowers/specs/2026-07-20-altitude-frame-drift.md. Lowering the
command is NOT the fix (the corridor is checked from both sides).

## Do not "fix" the magic numbers back
Transit is commanded 0.5 m UNDER the strict altitude and touchdown threshold
is 1.5 m because the frame wanders ±~0.7 m per arm. Exact values would bust
the ceiling from the other side.

## Home re-capture eats d_home (real bird and SITL alike)
PX4 re-captures home at every arm, so after any prior flight/drift the
mission's own d_home lies (2.9 m reading while 112 m from the configured
L&R). `tools/verify_flight.py` cross-checks the final fix against CONFIG, not
d_home — do not "simplify" that check.

## PX4 1.17 land detector cannot see a landing in HOLD (2026-08-29 tip-over)
Symptom: aircraft settled on the grass at idle thrust (lidar 0.00, thrust sp
0.12, motors 1100/1292) while AUTO_LOITER held position; `ground_contact`,
`maybe_landed`, `landed` stayed 0 for 226 s; the xy integrator wound to 22° of
pitch demand and the next climb-out levered the airframe over (53° pitch).
Mechanism (flags read from the ULog): in climb-rate-controlled modes the
multicopter land detector accepts "hit ground" only while the TRAJECTORY is
commanding a descent (`in_descend`); once the trajectory setpoint reaches its
(under-ground) target its vz is 0 → `in_descend=0` → no ground contact, even
with `has_low_throttle=1` and zero movement. No `LNDMC_*` parameter changes
this. LAND mode keeps commanding a descent and latches within ~1 s.
Fix: never HOLD on the ground — the companion's ground-contact guard commands
LAND on a lidar reading ≤ 0.45 m / 0.00 after ≤ 2.5 m (`tactical_align.py`),
and every lost-pad goto at rungs ≤ 3 m is vertical from the aircraft's own
position (`vertical_climb_below_m`) so no lateral demand can build. The frame
is projected at the lidar height below 7 m (`_projection_pose`), so a biased
height frame no longer inflates the lateral corrections.
Guard: `tests/test_tactical_align.py` (ground-contact, vertical climb-back,
lidar-scaled correction); the field proof is `make lidar-check` — no stream,
no guard.
