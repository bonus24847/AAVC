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

## Baro + lidar division of labour
Baro owns the absolute frame all flight. TFmini (0.4-12 m) joins the estimate
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
