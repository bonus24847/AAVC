# SITL-only traps

## PX4_SIM_SPEED_FACTOR at 1x ⇒ ZERO GRAVITY
PX4 1.17 answers the env var with a `set_physics` call that leaves the gz
world weightless. NEVER export it, not even at 1. (Candidate future check:
grep the env + launch scripts before a scored run.)

## Mission time is FLIGHT time — RTF collapse eats the window
Every deadline reads `state.now()` (vehicle clock). Under lockstep that is
SIM time: a loaded host ran 0.20× and a "12.9-minute" run was 7.4 real
minutes; a distance-derived timeout abandoned a leg the aircraft was closing
at 8 m/s. Leg abandonment is now progress-based (`_ProgressGuard`: no 1 m
closure in 25 s). Host-side liveness (frame age, telemetry staleness)
deliberately stays on `time.monotonic()`. **RTF hygiene:** run scored SITL
headless (dashboard cost 0.95 → 0.57 RTF).

## Restart SITL before a scored run if anything else flew first
The next mission takes off from wherever the last thing landed; PX4
re-captures home at arm, so d_home lies while the aircraft is 112 m from the
configured L&R. verify_flight catches it against config — restart SITL
instead of "fixing" the check.

## Wiping rootfs parameters*.bson costs the NEXT flight
Sometimes needed so airframe `param set-default` applies (PX4 won't override
a saved value) — but the first flight after runs on un-converged estimators
(hover-thrust HTE, EKF biases): one such mission climbed through the ceiling
and got RTH'd; identical mission next boot passed 19/0. Fly one throwaway or
restart once before trusting a run. Also: SITL persisting `MPC_Z_V_AUTO_DN=0.4`
is why SITL can never catch that pin going missing (fc-params.md).

## Dirty field: shed cargo boxes hide the markers
Boxes from a previous run lie ON the pads → decode impossible → "pads
missing", eggs come home. `run_mission.sh` prechecks `/tmp/aavc_detach.log`
and refuses (exit 1). Respect the refusal; restart the stack.

## Read the box truth BEFORE tearing gz down
Shed boxes exist only inside the running simulator; killing gz destroys the
evidence (`tools/box_truth.py` reads live). A ~40-minute round was lost
exactly this way. `run_mission.sh` deliberately does not exec for this reason.

## Wind leaves no trace
`set_wind.sh` marks nothing in the world/audit/ULog — a 10 m/s flight reads
like a calm one afterwards. State is announced via `/tmp/aavc_wind_state` and
stamped into box_truth.txt; check it when comparing runs.

## Truth coordinates once had a 0.5 m bias
`spawn_targets` used the equatorial radius on the north axis — every
touchdown-vs-truth distance inflated; "0.44-0.53 m scatter" was mostly this.
Fixed (ellipsoidal scales); pinned by test_geometry_invariant/test_spawn_targets.
Lesson: before tuning against a measured error, verify the measuring stick.

## Sourced ROS env breaks pytest collection
`make test`/`make lint` run under `env -u PYTHONPATH` (launch_testing/lark
plugins leak otherwise). Mirror that when invoking pytest by hand.

## SIM_BAT_DRAIN=900 is intentional, not stale
~2/3 of the 17000 pack's real endurance so the sim reaches the reserve inside
the 20-min window and the energy gate actually gets exercised. Rescaling to
"realistic" needs its own SITL re-validation day.
