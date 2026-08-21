# Full-system review — 2026-08-21, two days before the flight test

Four independent read-only reviews (flight core · vision · GCS/ops · hardware
config), plus a live board read. Nothing here is speculation about code that
was not read; where a finding needs a runtime value to confirm, it says so and
names what to check.

**Headline:** the mission logic is sound and the guards added today work. What
the review found is a different class of problem — **numbers that were tuned
for the small practice field and never re-derived for the competition one**,
and **a handful of "the check exists but cannot fire" defects**, which is this
project's recurring failure mode. Two of the worst reproduce, bit for bit, the
two incidents that already cost flights.

Severity is ranked by (probability it bites) × (what it costs), for the NEXT
flight first.

---

## TIER 1 — fix before the next flight

### 1.1 The landing ladder's top rung IS the ceiling
`orchestrator/main.py:1064-1069` → `orchestrator/tactical_align.py:80`

`_rungs_for` computes `top = min(12.0, ceiling)`. The KMUTNB profile's ceiling
became 10.0 on 2026-08-18, so `top = 10.0` — **exactly** the ceiling. Every
other altitude in the mission carries deliberate margin (transit 8.5 against
9.0, decode floor +0.5, sweep clamped to ceiling−1); this one has none.

Each delivery hovers at 10.0 for up to 12 s of acquire plus 18 s of rung-0
align, and both defer paths climb back and hover there again. Add the ~+0.6 m
AGL→MSL frame bias the code documents elsewhere, plus climb overshoot, and the
watchdog warn line (10.5) is crossed continuously and the breach line (11.5)
is one gust away. A sustained breach → `_trigger_rth()` → terminal leaves
RUNNING → **the mission loop exits and every remaining delivery is forfeited**,
mid-descent, with eggs aboard.

Under the old 5 m ceiling the `ceil <= 6.0` branch gave 4.0 m — safe. Raising
the ceiling silently moved the calculation into the branch where
`min(12, ceil) == ceil`.

**Check it:** the `alt=` column of the TELEM lines during rung 0 of delivery 1.
**Fix:** give the top rung the same margin the rest of the mission has.

### 1.2 The transit permanently doubles the climb-speed pin
`orchestrator/mission.py:465, 489`

`_set_climb_cap(1.0)` then `_set_climb_cap(2.0)` "to restore the fast cap" —
but the pinned value is **1.5** (`MPC_Z_VEL_MAX_UP`, cut from 3.0 precisely
because "the KMUTNB ceiling sits only 1 m above transit"). Nothing writes it
back, so from ~10 s into the first ingress the board holds 2.0 for the whole
mission — including every climb to the rung-0 altitude in 1.1. The parity test
compares config against `DEFAULT_PX4_TUNING` and cannot see a runtime
override, so this is invisible to `make test`.

### 1.3 The per-delivery battery gate compares two different quantities
`orchestrator/mission.py:145` — `pct > rth_battery_pct + 8` (= 38 %)

The 8-point margin is derived as "one delivery costs 7.7 % of 17 000 mAh" —
arithmetic on **state of charge**. `pct` is not state of charge: with
`BAT1_CAPACITY=-1` and no motor-current sensing, PX4's voltage-only branch
skips load compensation, and this repo's own notes record **28 % indicated at
65-70 % resting SoC**.

So on a full pack the loaded gauge reaches 38 % at roughly two-thirds
remaining, and the gate fires `DELIVERY abort: flight 1 skipping remaining
ids=…` — **the aircraft comes home with three eggs** and an audit line that
reads like a budget decision rather than a gauge artefact.

**Check it:** where `batt=` sits in the TELEM series during delivery 1's hover.
If it is under 38, deliveries 2-4 will not start. The fix is not a smaller
margin — it is that a percentage of a sagging voltage cannot be compared
against a percentage of capacity at all.

### 1.4 The watchdog's RTH is fire-and-forget; the aircraft can end up landed and ARMED
`orchestrator/safety.py:242-246, 624-652` + `orchestrator/main.py:1010-1053`

`_spawn_action` creates a task that **nothing ever awaits or cancels**.
`watchdog.stop()` cancels only the tick loop. The mission loop exits within
~2 s of the terminal flipping, `main`'s `finally` then calls
`commander.close()` — which kills `mavsdk_server` — and `asyncio.run` cancels
everything left.

So a battery/geofence/ceiling RTH at 100 m out: PX4 flies and lands the RTL,
but `rth()`'s explicit `action.disarm()` never runs, and `COM_DISARM_LAND=-1`
means PX4 will not auto-disarm either. **A live hexacopter sits on the ground,
armed, with the companion dead and the console already showing DONE**, while
the recovery crew walks up to it. This path has already been exercised (G7
flight 3 RTH'd at 68 s).

### 1.5 The white-pad cue rejects 54 % of pad orientations
`vision/detectors/aruco.py:177-184` — **measured**

The four rim probes sit at fixed **image-axis** diagonal offsets (0.566·side
from centre), but `minAreaRect` returns a **rotated** square whose support in
that direction collapses to 0.5·side near 45°. The probes land on grass, the
rim reads 118-140 against a floor of 160, and the blob is dropped.

Measured with the real pad artwork: **passes 0-20° and 70-90°, fails 21-69°**,
identically at 6/8/12/18 m. Pads are laid at arbitrary yaw.

It takes two more things down with it: the ×4 ROI booster only runs on blobs
the cue found, and `unidentified_candidates()` — the input to "revisit
undecoded pads at the 10 m floor" — is fed only by the cue. In
`tactical_align` the cue-only positional fallback also disappears, so a rung
where the marker does not decode counts as lost → climb back → defer.

### 1.6 On real flight footage the cue fires on 0 of 457 frames
`vision/detectors/aruco.py:62` — `_PAD_V_MIN = 170`, **measured**

Running the shipping detector over the real KMUTNB video: **1 decoded frame,
0 cue frames.** The frames are pure mono (mean S = 0.0), so the `S <= 60` half
of the "white" test is a no-op, and the surviving `V >= 170` is a bare
brightness threshold sitting on the frames' own ceiling (per-frame V p99.5:
min 144 / median 176 / max 255).

Consequence for today's work: **shortening exposure to fight blur pushes the
pad under 170 and kills the cue outright.** Raising `CAM_GAIN` is the lever
that does not, and it is untested.

Together, 1.5 and 1.6 mean pad-finding currently has **one** working leg — the
raw full-frame ArUco decode — and that leg scored 1/457 in flight.

### 1.7 The landing loop never got today's two fixes
`orchestrator/tactical_align.py:190-201` and `:360-369`

`vision_worker` got a new-frame guard and a pose snapshot taken *before* the
decode. The **landing** loop — the one that decides where the egg lands — got
neither:

- `_hit_world_fix` reads `state.telemetry` live, ~55 ms after the frame was
  read. That is a **bias in the direction of travel**, not noise, so the
  median filter cannot remove it. The 12 Hz retune was bought specifically to
  cut lag; this leaves ~0.08-0.12 s of it in the same budget.
- `_detect_nadir` re-reads unconditionally. At 12 Hz against a slower camera,
  the same image is decoded repeatedly — and because the pose is read live,
  one image yields a *sequence* of fixes that translate with the aircraft, so
  the commanded goto follows the vehicle's own motion instead of correcting
  it, and `lock_cycles=9` can be satisfied from ~3 distinct frames.

### 1.8 `decode_workers: 2` moved the callbacks onto the event loop
`orchestrator/vision_worker.py:162-168` + `orchestrator/main.py:937-951`

Verified by running it: with 2 workers, `on_fix` fires on the **loop** thread,
contradicting its own docstring. One registered callback is
`gcs_status.tracker_pusher`, which writes `mission_status.json` — a
synchronous SD write now on the loop that drives `tactical_align`'s pacer and
the MAVSDK streams. Its "write only on change" guard does not hold while a pad
is identified-but-unconfirmed: the ENU is rounded to 1 cm and the fused median
moves with every vote, so it writes about once per decoded frame.

### 1.9 A 27 %-loss radio link against 15 s display gates
`src/aavc_gcs.py:1978, 2007, 3459-3464` — **measured on the live link**

12 beacon arrivals in 75 s (expected 15); gaps of 9.2 · 10.5 · 9.2 · 11.0 ·
8.9 s. Every radio readout is gated at 15 s. One more lost tick and the
console flips to `idle — ไม่มี mission สด`: the 🚀 button reverts to "waiting
for the drone", the %-bar and chip strip hide, and every pad resets to
"รอสแกน" — **with nothing on screen saying "radio gap"**. It reads as "the
mission died", which is exactly the reading that ended G7 flight 1.

Today's `AAVC seen=` line also pushed the burst to **9 packets back-to-back
with no pacing** (~411 bytes). Bandwidth is fine; the burst is not.

**Cheapest fix:** widen the gates to ~25-30 s and show an explicit "📻 ขาด N s"
badge instead of blanking; space the beacon's sends ~100 ms apart.

---

## TIER 2 — competition-only (the `aavc-comp` repo and `kmitl_config.yaml`)

These do not affect the KMUTNB practice flight. They decide the competition.

### 2.1 The competition sweep is below the rules floor AND does not fit the pack
`sitl/kmitl_config.yaml:116` — `sweep_alt_m: 8.0`

Rules: the search band is 10-20 m. Nothing clamps upward — both clamps in the
code are `min()`. Computed on the real KMITL polygon (227 × 74 m) with the
measured lens:

| sweep alt | legs | sweep | total | energy | marker px | legal |
|---|---|---|---|---|---|---|
| **8.0 (shipped)** | 8 | 644 s | **1078 s** / 1200 | **12.88 Ah / 12.75 usable** | 42 | **no** |
| 12.0 | 5 | 407 s | 841 s | 10.04 Ah | 28 | yes |
| 19.0 | 4 | 327 s | 761 s | 9.09 Ah | 17.8 | at the decode floor |

**12.0 is the answer.** The 8.0 was tuned for a 10 m practice ceiling.

### 2.2 The KMITL time budgets are KMUTNB-sized
`sitl/kmitl_config.yaml:178, 181`

- `sortie_cost_s: 240` against a real sweep of 644 s at 8 m (407 at 12 m) —
  the flight gate under-estimates by **2.7×**.
- `rth_reserve_s: 45` when the far corner is **291 m** from L&R = 97 s at
  `MPC_XY_CRUISE=3.0` — under-reserved by 2×.

So the gate approves a delivery whose way home costs twice the reserve set
aside for it, and the per-delivery abort gate computes against the same wrong
number.

### 2.3 The comp repo still has the INT32 bug and no type audit
`~/Desktop/aavc-comp/mavlink_adapter/commands.py:231-237`

Its `_INT_PARAMS` is missing `EKF2_HGT_REF` and `MAV_1_FORWARD` — both are
pushed by its own config, both are INT32, both are **rejected by PX4** and
logged as `TIMEOUT`, which reads like a link hiccup. `tools/px4_type_audit.py`
(written after this shipped twice) **does not exist in that repo**, nor does
`preflight_params.py`'s current BOARD list (missing `SYS_HITL` and
`MPC_THR_HOVER`), nor `MPC_YAW_MODE` in its tuning — so a comp flight runs
PX4's default yaw mode and the nadir camera spins at every sweep turn.

`sync_core.sh` does not cover `tools/`. The drift `ops-field.md` predicted has
materialised.

### 2.4 Two more competition-config mismatches
- `cm4/launch_flight.sh:41` has **no site awareness** (no `.aavc_site`, no
  `--profile`), so in the comp repo it flies the KMUTNB config 30 km away.
  `docs/FLIGHT.md:167` still tells the operator to use it. Fails closed
  (geofence refuses) but burns window time and reads like a broken aircraft.
- `RTL_RETURN_ALT = 9.0` in the comp config vs CLAUDE.md's "pinned to 20 so a
  failsafe RTL stays at the ceiling". Every failsafe return crosses a 280 m
  field at 9 m. Decide it deliberately; do not leave doc and code
  contradicting each other on the morning.

### 2.5 The GCS icon starts the beacon from the wrong repo
`cm4/launch_gcs_real_gui.sh:293` hardcodes `cd ~/mission` although `$M_DIR`
holds the selected mission's remote dir — and the KMITL entry is `aavc-comp`.

Chain: the icon deletes `~/mission/captures/mission_status.json` and kills the
beacon, then starts a beacon pointed at `~/mission/captures`; 🚀 then runs the
comp mission whose orchestrator writes `~/aavc-comp/captures`; `ensure_infra`
sees a beacon already running and does not start the right one. **The beacon
reads a deleted file and broadcasts `AAVC p=idle` for the entire scored
round**, which the console filters out.

The operator sees: no progress, no chips, every pad "รอสแกน", 🚀 stuck on
"waiting for the drone" — while the aircraft flies the mission perfectly.
A bit-for-bit reproduction of the abort that motivated this whole day.
**One word: `cd ~/$M_DIR`.**

---

## TIER 3 — defects introduced or left half-done by today's work

| # | Where | What |
|---|---|---|
| 3.1 | `vision_worker.py::_claim_frame`, `_detect_one` | `stat()` then read is not atomic: if the grabber replaces the file between them, the mtime of frame N is recorded for the bytes of N+1, so N+1 is decoded **twice** and votes twice into a `confirm_votes=3` scheme. Fix: one `open()`, `fstat` the same fd. |
| 3.2 | `camera_grabber.py::MjpegPassthroughBackend` | No resolution check. `V4l2Backend` resizes if the driver ignored the request; passthrough writes whatever arrives. A 1920×1080 stream makes `fx` 1.5× wrong and the principal point off by (320,180) — every projection silently scaled and offset, and no gate catches it. |
| 3.3 | `src/aavc_gcs.py:1966` | The console's camera chip still stats `/tmp/aavc_nadir.png`. In SITL there is no beacon, so this is the only camera reading: a permanent grey n/a with a healthy camera. |
| 3.4 | `dashboard/routes.py:174` | The MJPEG stream still labels each JPEG part `Content-Type: image/png`. SITL/HITL only. |
| 3.5 | `cm4/deploy.sh:69-71` | `--check` prints "the aircraft flies this working tree" while **`sitl/aavc_config.yaml`, `clear_state.sh` and the `Makefile` are unhashed** — all three are on-aircraft flight-path files, and the config is the one the flight core reads. |
| 3.6 | `orchestrator/main.py:1072-1088` | The new `align:` seam casts straight into the dataclass with no bounds check. `lock_cycles: 0` makes `final_locked = 0 >= 0` true — the centred-LAND gate added after the 2.46 m off-pad release is **bypassed**, and every rung breaks on the first fix. A YAML typo is a land-anywhere switch. |
| 3.7 | `orchestrator/gcs_status.py:411-413` | `_write()` builds the doc under the lock but does the file ops outside it, through **one shared temp name**. Two threads (vision, now ~10 Hz, and the loop) can tear the file; the beacon then reads it, gets `None`, and broadcasts a spurious `p=idle` mid-flight. |
| 3.8 | `src/aavc_gcs.py:3639` | The plan polyline is drawn **without** the staleness gate and cached in `window.PLAN_KEEP`, which is never cleared. A leftover SITL `mission_status.json` paints yesterday's simulated route over the real field, unmarked, for the life of the page. |
| 3.9 | — | **Fix 3 cannot reach a real flight at all**: `status_sync.sh` is deliberately not started by the real launchers and the beacon carries no `plan`, so the polyline is written on the CM4 and never copied to the laptop. Either start the sync (the plan is a few KB, written while WiFi is still up at L&R) or drop the feature until it is. |

---

## TIER 4 — real, lower probability, worth a pass when there is time

**Flight core:** `drop_payload` has no exception boundary anywhere up the
stack, so any failure other than a clean `ActionError` ends the whole mission
armed on a pad (and the fallback endpoint is a SITL-only port, commented out
in the shipped config) · `land()`/`rth()` are not re-guarded before their
`disarm()`, which sits behind waits of 90 and 180 s — a pilot rescue during a
watchdog LAND can be met with a companion disarm · the ACQUIRE "expanding box"
never expands (`(ring % 4) // 2 or 1` is always 1) and now re-commands the FC
~144 times per acquire at 12 Hz · one dead telemetry stream is invisible
because all 14 tasks share one `_touch()`, and 8 of them have no `try/except`
at all — a frozen `is_armed` blinds the new disarm detector permanently ·
`MPC_Z_VEL_MAX_DN` is the manual twin, so the whole per-rung descent ladder is
inert, and it is left at 3.0 (2× the pin) which *does* apply to the safety
pilot's POSCTL rescue · the takeover detectors are not actually checked first
(a stale-telemetry escalation returns above them) · `record_anomaly` dedupes by
kind, so "camera died" is recorded at most once per mission while
`altitude_ceiling_warn_{alt:.1f}m` mints a new kind every 0.1 m.

**GCS:** auto-infra latches `_INFRA_STARTED` even on failure and never re-arms
after a CM4 reboot (a battery swap silently leaves the aircraft with no
router/camera/beacon) · the progress label says "delivering pad N" while the
aircraft is landing home after an abort, and the radio's `cur=` is scraped from
that same label · `verify_flight.py` has no needle for `PILOT TAKEOVER` and no
"every FLIGHT START has an END", so a takeover flight PASSES the post-flight
check with `0/0 deliveries` — and the `mode=` field added today for exactly
this is parsed and never used.

**Vision:** the ×4 ROI booster added **zero** decodes across 32 synthetic
conditions while costing a detector construction + resize + full
`detectMarkers` per blob, uncapped · `hitl_synthetic_camera.py:50` still holds
the retired 99.7° FOV, so every HITL "validation" of the vision chain is
validating the wrong geometry by 1.57× · the size prior reduces to
`alt_reported / alt_true`, so at the 1.5 m final rung a 0.9 m altitude
under-report rejects every hit — against ±0.7 m of documented per-arm wander ·
no camera roll/yaw mount parameter exists, so a camera rotated about its
optical axis would make the align loop orbit rather than converge, undetected.

**Hardware:** `COM_MOT_TEST_EN=1` (PX4's own default) plus
`CBRK_IO_SAFETY=22027` means that **while disarmed at the resupply hold** any
MAVLink source can drive a latch or a motor — inert in flight, a crew-safety
and egg-on-the-ground risk between flights · `px4_type_audit.py` does not cover
the 13 hardcoded setters (all 13 verified correct by hand today, but the next
one will not be caught) · CLAUDE.md §7 says `connect()` pins the envelope; it
does not — `main.py` does, and believing otherwise is what hides an
"applied 0/24" failure · BOM drift: two 7500 packs listed instead of the one
17000 aboard, ESCs called DShot when they are PWM-only, and the BEC that died
yesterday is described as feeding the CM4 rather than the servo rail, so the
new "carry a spare BEC" rule has nothing to buy.

---

## Verified correct — what the review confirmed is solid

**Board reads taken live tonight closed two open questions:**
`EKF2_HGT_REF = 0` (baro, as intended). The AUX output band is **safe**:
`PWM_AUX_MIN1..4 = 1000`, `MAX1 = 2100` with `MAX2..4 = 2000` (matching the one
latch documented as needing more travel, and exactly explaining the 1990/1900
seen at the bench), `DIS1..4 = 1000` and `FAIL1..4 = -1`. Both disarmed and
failsafe values sit on the **closed** side of the hold PWM, so a disarm or a
failsafe cannot drop an egg. The docs quote 1100; the board is safer than the
docs.

**Flight core:** release idempotence is unique by construction and lock-guarded
— no double-release path exists · the touchdown gate keys on the PX4 land
detector, not altitude, and genuinely keeps the egg when telemetry reads
airborne · the id-verified LAND gate rejects wrong-id hits before they can
steer, in all three layers · no re-arm-over-the-field path survives · the flight
clock and host clock are never mixed · `_Pacer`, `_ProgressGuard`,
`TimePolicy`, `chunk_flights` and `order_by_nearest` are all correct ·
`emergency_recover` correctly stays silent after a takeover · the geofence
upload fails closed and `GF_ACTION=3` is written and read back.

**Vision:** all four projection sign conventions verified numerically against
`alt·tan(tilt)` · a wrong-id decode is effectively impossible (DICT_4X4_50 with
`errorCorrectionRate=0.6` gives **zero** correctable bits — blur can only fail
to decode, never flip an id) · `_run_parallel`'s ordering and frame ownership
are correct · the mtime guard was verified live (a static frame decodes exactly
once) · the JPEG migration is complete for every real consumer, with correct
atomicity, codec selection and passthrough guard rails.

**GCS:** the served page parses (`node --check` on the live page) · the new
drawing code cannot throw on the radio path · a pad can never be orange and
green at once · every beacon line fits one packet, worst case measured · the
audit tee cannot raise into the flight path · the `mode=` TELEM lockstep holds
across all three vintages.

**Hardware:** the **egg release chain is consistent end to end** — config →
`actuator_index` → `payload_id` → `DO_SET_ACTUATOR` → AUX pin →
`PWM_AUX_FUNC1..4` — with the diagonal CG order preserved · the motor map,
airframe and rotor count match the board exactly · the battery gauge chain is
consistent and **no coulomb count can leak in** from the avionics-only sensor ·
all four failsafe setters agree with the board · `make type-audit` is clean and
all 13 hardcoded setters were hand-resolved against the v1.17 source ·
`.aavc_site` precedence is correct in `run_mission.sh` · **570 tests pass.**

---

## Recommended order of work

1. **1.1 + 1.2** (rung-0 margin + the climb-cap restore) — one afternoon's
   worth of change, and together they are the most likely single cause of a
   forfeited flight at KMUTNB.
2. **1.3** — measure `batt=` on the next flight before changing anything; the
   fix depends on what the gauge actually reads under load.
3. **1.5 + 1.6** (the cue's rotation geometry and its brightness floor) — this
   is the answer to "why can't it find pads", and it is measurable on the
   bench with the printed marker.
4. **1.4, 1.7, 1.8, 3.1, 3.6** — contained code fixes with tests.
5. **2.x** — the competition config and the comp repo, before travelling.
6. **1.9 + 3.3/3.4/3.5** — the operator's screen and the deploy check.
