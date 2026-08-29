# AAVC 2026 — Technical Design & Analysis Report

**Autonomous PX4 Hexacopter for Precision Fragile-Cargo Delivery**
Team **AeroOptix** · King Mongkut's University of Technology North Bangkok (KMUTNB), Faculty of Engineering
Autonomous Aerial Vehicle Challenge 2026 · Rules & Regulations **V1.3** (July 2026),
as amended by the **24 July 2026** and **28 August 2026** event briefings
IAAI, KMITL Ladkrabang · 28–30 August 2026

> **Document version 2.0 · revised 28 August 2026**, after the 09:00
> competition-day briefing.
>
> Version 1.1 (17 July 2026) described a 500 mm **quadcopter** flying **one egg
> per sortie** under a 20 m ceiling, and had never flown outdoors. Every section
> below has been rebuilt against the aircraft that is actually flying, the
> envelope the committee actually briefed, and the flights that have actually
> been flown. Where something is still unproven this report says so in the same
> sentence as the claim.
>
> Structured to rules Appendix A (Executive Summary 10 pts · Team Organization
> 10 pts · Concept Development 30 pts · Engineering Design & Analysis 40 pts),
> and cross-referenced to the Appendix B DCOS airworthiness criteria (§3.8).

---

## 0. What changed since version 1.1

| Area | v1.1 (17 Jul 2026) | This document (28 Aug 2026) | Why |
|-------------|----------------------|-------------------------------|----------------------------------|
| Airframe | Holybro X500 V2 quad-X, 500 mm, ~2.6 kg | **EFT X6100 hexacopter**, 1.000 m wheelbase, 18-inch props, **7.2 kg AUW** | The quad was never procured; DRON (Defence Research Operation Network) lent the X6100 frame, ESCs and props on 22 Jul 2026 |
| Mission shape | 1 egg per sortie, up to 4 sorties | **4 eggs in ONE flight**; FLIGHT ⊃ DELIVERY, scored per delivery | 24 Jul 2026 event-briefing override |
| Ceiling | 20 m AGL | **30 m AGL**; search band 10–30 m; transit unchanged at 20 m | 28 Aug 2026 competition-day briefing |
| Transit corridor | PDF P1→P2→P3 | **P1→P2′→P3′**, moved 14–23 m west of the main building | Briefing: the PDF's north leg ended on the building roof |
| Search area | Full rules polygon | Rules polygon **cut at E 110 m** (west of the building) | Briefing declared three no-fly bands, one of them mid-polygon |
| Failsafe RTL altitude | 20 m | **25 m** | Briefing: "25 m clears everything on the field" |
| Sweep | 12 m, decode in flight | **20 m white-pad finder** + id read on **10.5 m decode visits** | Obstacle clearance over the display aircraft; 20 m puts the marker under the decode floor, the pad well above the blob floor |
| Marker set | ids 1–6 | **ids 0–6 (seven)** | The rules' Figure 7 encodes **1, 2, 0, 4, 5, 6** — id 0 is a real pad |
| Payload | One servo on AUX 9 | **Four latches on AUX 4/1/2/3**, `DO_SET_ACTUATOR`, diagonal release order | Four eggs per flight; PX4 has no `DO_SET_SERVO` handler |
| Camera | 1280 px, "~99.7°" placeholder, on a gimbal | 1280 px, **74.2° measured**, **hard-mounted**, mount yaw **180° measured** | Both numbers were assumptions; both were wrong; both are now bench measurements |
| Height reference | Optical flow + TF-Luna | **Barometer (`EKF2_HGT_REF=0`) + Benewake TFmini-S**; no optical flow | No flow module in the kit; GPS height reference diverged 10.8 m peak-to-peak in one flight |
| Battery | 6S 7500 mAh LiPo | **6S 17,000 mAh semi-solid** (+ a 15,000 mAh spare) | 7500 could not finish the mission |
| Validation | SITL only | SITL **plus 10 outdoor flights** at the KMUTNB practice field (20, 26, 27 Aug) | The system has now flown a complete mission end to end |
| Test suite | 239 tests | **771 tests** | |

---

## Executive Summary (10 pts)

We field a single **autonomous PX4 hexacopter** that carries **all four
committee-assigned eggs in one flight**, lands **on** each pad whose ArUco
marker id was assigned to us, and releases that egg **only after touchdown is
confirmed** — inside the 20-minute operation window at the IAAI KMITL field.

The design philosophy is **fast-but-safe determinism**. A classical
computer-vision pipeline — no machine learning anywhere in the flight loop, no
network of any kind (network use is an automatic disqualification) — runs on a
Raspberry Pi CM4 companion computer and commands a Pixhawk 6X flight
controller. The aircraft takes off from the Launch & Recovery point, flies the
mandatory transit corridor at a strict 20 m, surveys the search area once,
descends over each assigned pad in verified altitude rungs, lands on it,
releases the egg on the ground, and flies the corridor home to land and disarm.

**Cargo location is unknown at take-off.** The committee assigns marker ids,
not coordinates. The system therefore performs a **blind visual search with a
pad registry**: one boustrophedon sweep decodes every pad it can see into an
id-keyed registry, undecoded white-pad candidates are revisited at the legal
10 m floor to read their ids, and the flight then serves its four assigned pads
directly. The sweep stops early as soon as every id this flight owes is
confirmed.

**Envelope as amended at the 28 August 09:00 briefing:** ceiling **30 m** AGL,
transit strictly **20 m**, search band **10–30 m**, below 10 m only for the
delivery descent over a pad, failsafe RTL at **25 m**, and a transit corridor
moved west of the main building. The flown search area is the rules' polygon
cut at E 110 m so that every straight-line move the mission can make — pad
hops, decode visits, egress, even a failsafe RTL — stays clear of the three
no-fly bands the committee declared.

**Validated results.**

| Evidence | Result |
|----------------------------------|------------------------------------------------------------------|
| **Real flight, complete mission** (KMUTNB practice field, 27 Aug 2026, 15:25, 4 min 12 s) | Transit ingress **3/3** → sweep found 5 pads in ~20 s → early-stop on the assigned ids → **land-ON + release on pad 1 (0.14 m from centre) and pad 4** → egress → landed **0.7 m** from the launch point. **2/2 eggs delivered, both after touchdown.** |
| **Real flight, full survey** (27 Aug 2026, 12:44, 5 min 56 s) | 7-leg sweep, 6,044 frames, **5 of 6 pads decoded and confirmed**; the sixth was printed with marker id **0**, which the id filter then still excluded — the reason the valid set is now 0–6 |
| **SITL, competition envelope** (24 Aug 2026, transit 20 m / ceiling 20 / sweep 12 m — the hardest decode case) | **6/6 pads found, 6/6 ids correct, 4/4 eggs delivered**, releases **0.09–0.22 m** from truth, transit 6/6, landed 2.0 m from L&R, **536 s of the 1200 s window** |
| **SITL, four eggs in one flight** (25 Jul 2026) | 4/4 delivered, releases 0.15–0.21 m, max altitude 19.69 m, **457 s**, verifier **14 checks / 0 warnings** |
| Software integrity | **771 automated tests** green; lint + type-check clean; the flight core is deterministic and offline |

**The honest status.** Three things are not yet proven and are stated here
rather than buried: (1) the 28 August briefing changes — new corridor, 30 m
ceiling, 20 m sweep, cut search area — are **unflown**, and the 13:00–18:00
trial slot is their only test before scoring; (2) **battery endurance is the
binding risk**: the gauge falls 7–8 percentage points per minute under load, so
a full pack reaches the 15 % return floor at roughly minute 8.5–10 against a
plan that needs 10–13 minutes of flying — the mission is designed to give up
eggs safely rather than the aircraft, and a second charged pack plus an
automatic recovery flight is the answer; (3) the committee has not published
coordinates for the new corridor or the no-fly bands, so ours are georeferenced
from briefing photographs to **±5 m** and will be replaced by the committee's
if they are issued.

---

## 1. Team Organization (10 pts)

Team **AeroOptix**, Faculty of Engineering, KMUTNB — a **first-time AAVC
entrant**. Four months ago the team had no aircraft, no budget, and no prior
autonomous-drone build. The system described here was built from zero in that
time, on equipment donated, lent, or paid for by the members themselves.

| Role | Function |
|--------------------------|--------------------------------------------------------------------------|
| Team Leader (contact) | Overall coordination; single point of contact with AAVC staff |
| Faculty Advisor | Airworthiness sign-off, safety oversight |
| Mission software | Orchestrator, search/serve logic, time and energy policy, audit trail |
| Flight systems / **Safety Pilot** | Pixhawk 6X integration, PX4 parameters and failsafes, manual takeover and kill authority |
| Vision & GCS | ArUco pipeline, camera calibration and exposure, ground station |
| Airframe & power | Frame build, payload rack, wiring, battery and mass budget |
| C2 / GCS operators (≤4) | Per-flight GO, monitoring, resupply coordination |

Crew inside the GCS operating area complies with the rules limit: **1 safety
pilot, ≤4 command-and-control operators, ≤3 technical-support personnel**. All
flight-line crew wear PPE (protective glasses, reflective vests) per DCOS. The
aircraft frame, ESCs, propellers and both battery packs were provided by
**DRON (Defence Research Operation Network)**, whose support is acknowledged.

---

## 2. Concept Development (30 pts)

### 2.1 Capability parameters derived from the operational challenge

The operating environment fixes the design envelope. From the rules (V1.3), the
two event briefings, and the IAAI KMITL field geometry:

| Environment / rule driver | Value | Derived vehicle requirement |
|--------------------------------|--------------------------|------------------------------------------|
| Controlled airspace (geofence) | ~296 × 172 m | Endurance and speed to cross it repeatedly inside the window |
| Search area **as flown** | Rules polygon **cut at E 110 m** — the open field west of the main building, ~69 × 60 m | Cover ~0.4 ha with a decodable sensor footprint |
| No-fly bands (briefing) | Main building E 122–186 / N 25–84; east end E 215–256 / N 37–116; orange block E 219–255 / N −14…40 (ENU metres from L&R, ±5 m) | Every straight-line move must stay west of E 110 m |
| Transit altitude (mandatory, both directions, scored per point) | **strictly 20 m** | Precise altitude hold at 20 m AGL in an absolute frame that does not drift |
| Ceiling (briefing) | **30 m AGL** | Companion watchdog warns at 30.5 m, returns home above 31.5 m sustained |
| Search band | **10–30 m** AGL | Pad acquisition from 20 m, id decode from 10.5 m |
| Delivery descent | below 10 m only, over the pad | Precision terminal descent ending in a land-ON |
| Landing pad | 1 × 1 m white square, 750 mm dia. black circle, **400 mm ArUco** (`DICT_4X4_50`) | Decode a 400 mm marker from the legal band with a 1280 px sensor |
| Marker set | **ids 0–6** — six pads placed, **four assigned** per team | Reject the two distractor ids for the whole mission |
| Operation window | 5 min setup + **20 min operation**, per-minute overtime penalty | A full four-egg flight must fit with reserve |
| Cargo | Four organiser boxes (heart, ~16 × 7 × 18 cm, 300-gsm), one no. 0 raw egg each | Four independent holds; gentle touchdown; release on the ground |
| MTOW limit | ≤ 25 kg | 7.2 kg leaves large margin |
| No RTK GPS | ~0.5–1.5 m, plus a common-mode bias against the published coordinates | Vision owns the final metre; GPS never commits the egg |


![The competition field as briefed on 28 August 2026, georeferenced against the ground station's cached satellite imagery. **Purple** — the controlled airspace (geofence). **Teal** — the rules' search polygon. **Blue** — the three sweep legs actually flown, over the polygon cut at E 110 m. **Orange** — the briefed transit corridor L&R/P1 → P2′ → P3′. **Grey** — the corridor the PDF specifies (P2, P3), whose north leg ends on the main building's roof. **Red** — the three no-fly bands declared at the briefing: the main building, the east end of the search area, and the rules' orange block. Every straight-line move the mission can make stays west of the building box.](report_figures/field_geometry_2026-08-28.jpg){width=100%}

![The corridor region at 4x. **Yellow** — the PDF's P1→P2→P3 with P2 and P3 marked; the leg between them crosses the building. **Orange** — the route drawn on the committee's own briefing slide. **Green** — the operator's traced corridor, running up the grass strip between the parked display aircraft and the building's west wall; P2′ and P3′ are taken from this line and are accurate to **±5 m**. Committee-published coordinates, if issued, replace ours.](report_figures/corridor_georeference_2026-08-28.jpg){width=62%}

**Decode geometry drives the sweep plan.** With the **measured** 74.2° lens on
a 1280 px-wide sensor, the ground swath is `2·h·tan(37.1°)` and marker pixels
are `1280 × 0.4 / swath`:

| Altitude | Swath | 400 mm marker | 1 m pad | Role |
|-------------|----------|-------------------------|----------------------|------------------------------|
| 20 m (sweep) | 30.3 m | **16.9 px** — below the 18 px decode floor | **42 px** — well above the 18 px blob floor | **White-pad finder**: 3 legs, 216 m, 87 s over the cut polygon |
| 10.5 m (decode visit) | 15.9 m | **32 px** — inside the 25–35 px band proven in flight | 80 px | **Id reader**: hover and dwell until the marker decodes (~47 s) |
| 12 m (the alternative) | 18.2 m | 28.2 px | 70 px | Would decode in flight, but 5 legs / 344 m / 142 s and lower clearance over the display aircraft |

Flying the survey at 20 m and paying for a decode visit only where a white pad
was actually seen is **55 s cheaper** than sweeping at 12 m on this polygon,
and keeps the aircraft above the field obstacles the committee flagged.


![The seven landing-pad faces the mission must tell apart — 1 x 1 m white square, 750 mm diameter ring, 400 mm `DICT_4X4_50` marker. Rendered here by `vision.detectors.aruco.render_pad_bgr`, the same function that generates the simulator's pad textures and the detector's test images, so what the detector is tested against is what the simulator shows. Six of these are placed on the field and four are assigned to us; **id 0 is a real pad** — the rules' Figure 7 encodes 1, 2, 0, 4, 5, 6.](report_figures/pad_faces_ids_0_6.png){width=100%}

**Window budget drives the mission shape.** A full four-egg flight is budgeted
at **894 s of the 1200 s window** (17 s take-off + 53 s ingress + 24 s
climb/transition + 87 s sweep + 188 s decode visits + 420 s of four serves +
17 s + 53 s egress + 35 s landing). The last delivery's gate therefore runs
with **516 s remaining** against a 300 s floor. Carrying all four eggs in one
flight — rather than four one-egg flights — is what buys that margin, and the
briefing explicitly permits it.

### 2.2 Selected delivery system — configuration and architecture

**Vehicle configuration: single multirotor, hexa-X.** The mission is a confined
urban field with a vertical take-off and landing requirement, a 1 m landing
target, and a fragile payload; a multirotor is the only sensible class, and six
rotors were chosen over four for three reasons that matter operationally:

1. **Rotor-out tolerance.** With one motor lost, PX4's sequential-desaturation
   allocator mixes roll, pitch and thrust before yaw, so a hexacopter keeps
   attitude and thrust authority and gives up only yaw. That is recoverable by
   the safety pilot; on a quad it is not. (Measured in SITL: unallocated yaw
   torque rises to 0.25 with the allocator still solving for the other axes.)
2. **Payload and mass margin.** Four boxed eggs plus a four-latch rack on a
   1.0 m frame with 18-inch props hovers at ~31 % of available thrust, so the
   motors loaf and retain full authority against gusts.
3. **Availability.** The X6100 airframe, ESCs and props were lent complete,
   which converted a procurement risk into flight time.

A single aircraft — rather than a cooperative search-and-deliver pair — keeps
mass, cost and failure surface small, and the registry-based search removes the
need for a second vehicle to find pads.

**System architecture.**

```
AIR SEGMENT                                        GROUND SEGMENT
+---------------------------------------+        +-----------------------------+
| Pixhawk 6X (PX4 v1.17)                |        | Safety-pilot RC (ELRS)      |
|   attitude/position control, geofence,|  RC +  |   manual takeover, kill      |
|   FC-level failsafes, RTL             |<------>|                             |
|        ^  MAVLink (serial)            | MAVLink| Web GCS (Svelte + FastAPI)  |
|        |                              | on ONE |   live map, camera, plan,    |
| Raspberry Pi CM4 (companion)          | radio  |   4-of-6 queue editor, GO,   |
|   mission orchestrator (async Python) |        |   confirmed-pad readout, KILL|
|   vision worker (OpenCV ArUco)        |  WiFi  |                             |
|   safety watchdog (2 Hz)              |<------>| (CM4 raises its own local AP |
|   audit trail + frame recorder        |        |  — no internet, no 4G)       |
|        ^                              |        +-----------------------------+
|   OV9281 nadir camera (1280x720 mono, |
|   global shutter, hard-mounted)       |        FPV camera + 5.8 GHz VTX
|   TFmini-S lidar  ·  non-RTK GNSS     |        -> the "transmit" half of the
|   4 egg latches (AUX 4/1/2/3)         |           imaging requirement
+---------------------------------------+
```

The flight-management stack is a **non-proprietary, open-source architecture**
(PX4 on Pixhawk 6X) integrated by the team, which complies with the rules'
prohibition on complete proprietary flight packages. All higher-level autonomy
runs on the CM4 as deterministic classical code that the team wrote.

**Rationale for the key decisions:**

- *Classical CV, never ML, in the flight loop.* The target is a fiducial
  marker; an OpenCV ArUco decode is deterministic, cheap on a CM4, and
  auditable frame by frame. No torch, no YOLO, no VLM, no cloud, no network.
- *Land-ON, then release.* Scoring pays for the egg being placed on the pad
  intact. Releasing from the air risks a broken egg for zero payload score, so
  the release is hard-gated behind a confirmed touchdown and is skipped and
  audited if telemetry says the aircraft is airborne.
- *The id is the authority, not the position.* A pad that decodes to the wrong
  id can never steer a descent, and the LAND gate refuses to commit unless the
  assigned id was decoded during the approach.
- *Vision owns the final metre.* Without RTK the GNSS fix wanders by more than
  the pad is wide, so the terminal descent is closed by the camera.
- *The GCS monitors; it does not fly.* One operator click per flight starts it;
  everything after that is on board, so a WiFi dropout costs a picture, never
  the mission.

### 2.3 Concept of operations

A **FLIGHT** is one arm→disarm cycle. A **DELIVERY** is one pad served inside a
flight, and each delivery is scored independently. With four eggs aboard, the
whole mission is normally **one flight containing four deliveries**:

```
PREFLIGHT   operator loads the 4 assigned ids as an ordered queue, confirms the
            eggs are racked and the crew is clear -> ONE "GO" click
   -> TAKEOFF        arm, climb out, then close the last metres en route
   -> TRANSIT_INGRESS  P1 -> P2' -> P3' at 20 m        (scored per coordinate)
   -> SEARCH         boustrophedon sweep of the cut search polygon at 20 m,
                     registering every white pad seen; decode visits at 10.5 m
                     read the ids; the sweep stops as soon as every id this
                     flight owes is confirmed
   -> for each assigned id, in turn (one DELIVERY each):
        LOCALIZE     hop to the pad at sweep altitude, descend in verified
                     altitude rungs, re-centring on the decoded marker
        LAND         land ON the pad (final rung tolerance 0.2 m on a 1 m pad)
        DROP         after touchdown confirms + 2 s settle, open that egg's latch
   -> TRANSIT_EGRESS   P3' -> P2' -> P1 at 20 m        (scored per coordinate)
   -> LAND at L&R -> DISARM (resupply crew may approach)
```

**Search-and-identify tactic.** A flight whose assigned ids are not all in the
registry flies the sweep. Every pad the camera sees is clustered by decoded id
with a k-vote confirmation, so a single mis-decode cannot create a target. A
pad seen as a white blob but not decoded is revisited at 10.5 m and the
aircraft dwells until the marker reads; a pad decoded but short of its vote
quota gets a cheap vote top-up visit instead of a re-sweep. The registry
persists across flights, so a later flight whose whole assignment is already
registered flies **direct**, with no sweep at all.

**Delivery tactic (land-ON, touchdown-gated).** Over the assigned pad the
aircraft descends through altitude rungs, re-centring on the decoded marker at
each rung with a tightening tolerance down to **0.2 m** before the final
descent. Three gates protect the egg:

- the **id-verified LAND gate** refuses to land unless the delivery's own
  assigned id was decoded during the approach — a wrong-id pad or an undecoded
  blob can never commit the egg;
- the **centred-final-rung gate** refuses to land if the re-centre never
  converges, climbing back and deferring instead, with one re-approach if the
  window allows;
- the **touchdown gate** releases only after PX4 reports the vehicle landed,
  plus a 2 s settle; if telemetry reads airborne at the release point the
  release is skipped and audited.

Between deliveries inside a flight the aircraft stays **armed** on the pad
(`COM_DISARM_LAND = -1`), so no re-arm happens over the field and PX4's home
point stays pinned to the launch site.

**Repeat-delivery and resupply tactic.** Between flights the aircraft lands at
L&R and disarms so the crew can approach. The 20-minute window clock starts at
the **first** GO. A per-flight gate refuses a launch that cannot finish inside
the window unless the operator explicitly accepts the overtime penalty, and a
**per-delivery gate** re-checks time and battery before **every** egg: failing
it skips the remaining eggs of that flight and brings them home rather than
starting a descent the budget cannot finish.

**Planned battery egress — the mission gives up eggs, not the aircraft.** Below
**30 %** indicated the flight starts nothing new: the sweep stops, decode visits
are skipped, the delivery gate refuses, and the aircraft flies the **normal
egress through the corridor**, lands and disarms. The crew swaps to the second
pack and the **recovery flight** serves what is still owed — flying direct off
the registry, and firing only the latches that still hold eggs (the crew does
not re-rack eggs at a battery swap, so the recovery flight continues the wiring
order through the unfired holds). The 15 % return-to-home and 10 % land-in-place
thresholds sit **underneath** this as failsafes; the point of the planned
egress is never to reach them.

---

## 3. Engineering Design & Analysis (40 pts)

### 3.1 Airframe structure and integrity

- **Frame:** **EFT X6100** hexa-X, **1.000 m wheelbase** (0.500 m arm radius),
  folding carbon arms and carbon plates, frame mass ~2.75 kg, manufacturer MTOW
  ~12 kg. Provided by DRON.
- **Rotor clearance:** adjacent motors sit 500 mm apart on the hexagon; with
  **18 × 6.5 in** propellers (457 mm diameter) the tip-to-tip gap is **~43 mm**,
  so there is no rotor overlap and no interaction between discs (DCOS
  "clearance between moving parts").
- **Payload rack:** four independent holds arranged front-left, front-right,
  rear-left and rear-right about the centre of gravity, each sized to accept the
  organiser's heart-shaped box (~16 × 7 × 18 cm, 300-gsm) rather than a bare
  egg. The release order is **diagonal** so the net CG moment is zero when full
  and zero again after the first two eggs leave.
- **Ground clearance:** the camera and the lidar are carried 0.25 m below the
  CG on the underside; the landing gear keeps both, and the loaded boxes, clear
  of the pad at touchdown.
- **Structural load cases:** flight manoeuvre load is envelope-limited to
  **30° tilt** (`MPC_TILTMAX_AIR`) and **2 m/s² horizontal acceleration**
  (`MPC_ACC_HOR`); the land-ON touchdown is absorbed at
  **`MPC_LAND_SPEED` = 0.3 m/s** with the final autonomous descent pinned to
  **0.4 m/s** (`MPC_Z_V_AUTO_DN`); ground handling and transport use the folded
  arms.
- **Power path:** the **motors are fed from a separate distribution board that
  the flight controller cannot sense**. A Holybro **PM02D** supplies the
  Pixhawk 6X and avionics only, and the CM4 runs from its own power-bank rail
  so companion current cannot brown out the flight controller. This split is
  deliberate and has one important consequence, carried through §3.2: **any
  current the FC reports is avionics draw, never flight current**, so the
  battery gauge must never be current-fused.
- **Mass, as weighed 19 August 2026:** **7.2 kg all-up, ready to fly**, with one
  17,000 mAh pack (1.5 kg) and four boxed eggs aboard — **29 % of the 25 kg
  rules limit**. Rule adopted after three conflicting weighings in one
  afternoon: *never record a mass without recording what was on the aircraft*
  (pack count, power module, companion, eggs).
- **Open items (honest status):** a formal **CG placement record** and a
  **quantified structural load-factor margin** are not documented; the aircraft
  is trimmed by flight behaviour, not by a calculated CG envelope. The
  organiser's boxes are heavier than the 50 g dummies in the weighed figure, so
  the aircraft is re-weighed with the real cargo at the field committee's scale.

### 3.2 Propulsion and flight-performance analysis

| Parameter | Value | Basis |
|------------------------------------|------------------------------------------|----------------------|
| Airframe | EFT X6100 hexa-X, 1.000 m wheelbase, 18 × 6.5 in props | as built |
| Motors | **6 × EFT E5 5008, 335 KV** | as built |
| ESC | 6 × PWM-only ESC (**no telemetry lead** — see §3.3) | as built |
| Max thrust per motor | **37.65 N** (3,838 gf at 100 % throttle, 24 V, 18 × 6.5 in) | manufacturer bench table (Power-System-Guide) |
| Max thrust, 6 rotors | **226 N ≈ 23 kgf** | 6 × above |
| All-up weight (flight-ready, 4 boxed eggs) | **7.2 kg** | weighed 2026-08-19 |
| Thrust-to-weight | **≈ 3.2 : 1** | 23 kgf / 7.2 kg |
| Hover thrust fraction | **≈ 31 %** of maximum (11.8 N per motor) | as above |
| Hover current | **≈ 30 A** at 6S (~670 W) | E5 bench anchor scaled by mass^1.5 |
| Hover throttle, measured in flight | **0.59–0.64** | ULog, 2026-08-27 |
| Battery (mission) | **6S 17,000 mAh semi-solid**, 10 C, 1.5 kg, 25.1 V full (4.18 V/cell), ~22.6 V empty (3.77 V/cell) | as fitted 2026-08-19 |
| Battery (spare, for the swap) | 6S 15,000 mAh | as held |
| Usable energy | **12,750 mAh** (75 % of nameplate) | policy |
| Planned cost of a four-delivery flight | **4,700 mAh** (1,750 first delivery + 3 × 900 + 250 margin) | `battery:` seeds, deliberately conservative |
| Cruise / sweep speed | 3.0 m/s (`MPC_XY_CRUISE`), max 5.0 m/s (`MPC_XY_VEL_MAX`) | config |
| Climb / take-off | 1.5 m/s (`MPC_Z_VEL_MAX_UP`, `MPC_TKO_SPEED`) | config |
| Autonomous descent / touchdown | 0.4 m/s (`MPC_Z_V_AUTO_DN`) / 0.3 m/s (`MPC_LAND_SPEED`) | config |
| Auto yaw rate | 25 °/s (`MPC_YAWRAUTO_MAX`) | config |

**Excess-thrust margin (DCOS).** Hovering at ~31 % of available thrust leaves
roughly a two-thirds throttle headroom, so the motors run cool, the allocator
never saturates (confirmed in the 27 August logs), and full control authority
remains for gust rejection. The margin holds across the loaded and empty cases
because four boxed eggs are a small fraction of a 7.2 kg aircraft.

**Energy — the binding constraint, stated plainly.** On paper the budget is
comfortable: 4,700 mAh planned against 12,750 mAh usable. In flight it is not
that simple, because this aircraft **cannot measure current**. With
`BAT1_CAPACITY = -1` the state of charge is derived from voltage alone, and PX4
only load-compensates when it has a current reading — which it does not — so
the indicated percentage sags under thrust and rebounds when the aircraft
settles. Measured across four flights on 27 August:

| Observation | Value |
|------------------------------------------------------------------------|----------------------------|
| Continuous sweep flight, full pack | 93 % → 26 % indicated in **5.8 min** |
| Three flights with pauses, full pack | 96 % → 25 % indicated over **8.6 min** of flying |
| Indicated drain under load | **~7–8 percentage points per minute** |
| Extrapolated time to the 15 % return floor | **minute 8.5–10** of powered flight |
| KMITL plan | **894 s budgeted**, ~10–13 min realistic |
| Cell voltage at 25 % indicated, rested | 3.87–3.92 V/cell ⇒ true state of charge ~55–65 % |

The gauge is therefore **pessimistic** — the pack almost certainly holds the
energy (roughly 10 Ah of 17 Ah for a 15-minute flight at ~40 A) — but the
**true capacity has never been measured**, so the flight is planned against the
gauge, not against the optimistic interpretation. Consequences, all already
implemented: the per-delivery gate gives up eggs 3 and 4 safely rather than the
aircraft; the planned 30 % egress flies the scored corridor home instead of
falling into a straight-line failsafe RTL; and a **second charged pack with an
automatic recovery flight** is the primary mitigation. The one measurement that
closes the question is the **mAh returned by the charger** after a flight, which
is recorded from now on. The battery percentage floors were **not** lowered
further: the ESCs' low-voltage cut-off is unknown, and a floor is worth more
than an egg.

**Flight-performance verification against §2.1.** Requirements met: the survey
covers the cut polygon in 3 legs at 3 m/s; transit holds 20 m in the
barometric frame (commanded 19.5 m — see §3.3 — measured maximum 19.69 m in the
four-egg SITL run, no ceiling flag); and a complete real mission — transit,
survey, two land-ONs with releases, egress and landing 0.7 m from the launch
point — was flown in **4 min 12 s** on 27 August.

### 3.3 Avionics and sensor subsystem

| Subsystem | Component | Function and status |
|----------------|----------------------------|--------------------------------------------------------|
| Flight controller | **Holybro Pixhawk 6X**, PX4 **v1.17.0** | `SYS_AUTOSTART = 6001` (generic hexarotor X), `CA_ROTOR_COUNT = 6`, `PWM_MAIN_FUNC1..6 = 101..106` — verified pin by pin with `ACTUATOR_TEST`, all six motors observed spinning in the correct order and direction |
| Companion computer | **Raspberry Pi CM4** | Mission orchestrator, vision, watchdog, GCS server; own power rail |
| Nadir camera | **Meige OV9281** USB UVC, 1280 × 720 **mono global shutter** | Sole control-authority sensor: ArUco decode + white-pad cue. **FOV measured 74.2°** (fx ≈ 847 px). **Hard-mounted, no gimbal.** |
| Camera orientation | **Mount yaw measured 180°** | The camera is bolted **upside down**; image row 0 looks at the tail. Measured from four independent placements of a floor target around the levelled airframe (180.3 / 187.4 / 183.4 / 186.1°, snapped to the bolted 180) |
| Height (primary) | **Barometer**, `EKF2_HGT_REF = 0` | GPS height reference was measured diverging **10.8 m peak-to-peak** inside one flight and returned an aircraft that was tracking transit correctly — the barometer is the reference at both fields |
| Height (terminal) | **Benewake TFmini-S** downward lidar, TELEM3 (`SENS_TFMINI_CFG = 103`) | `EKF2_RNG_CTRL = 1`, conditional aiding below `EKF2_RNG_A_HMAX = 7.0` m — pins the final metres of the descent |
| Optical flow | **None** (`EKF2_OF_CTRL = 0`) | No flow module in the kit; the decision is pinned so it cannot silently re-enable |
| GNSS | Non-RTK GPS/compass | Measured 14–16 satellites, horizontal accuracy ≤ 2 m at the practice field |
| Power sensing | Holybro **PM02D**, avionics only | `BAT1_CAPACITY = -1` forces the **voltage-only** gauge; `BAT1_V_DIV` closed by multimeter (24.9 V at the connector vs 24.89 V reported — 0.04 %) |
| ESC telemetry | **None — physically absent** | The ESCs are PWM-only with no telemetry lead; `DSHOT_TEL_CFG = 0` and zero `ESC_STATUS` even after the FC accepted a message-interval request. Per-motor current is the only signal a motor-out detector could use, so both detector layers were **deleted rather than left inert**; the safety pilot is the mitigation, and restoring the feature starts with buying telemetry-capable ESCs |
| Imaging downlink | FPV camera + 5.8 GHz VTX | The "transmit" half of the imaging rule, independent of WiFi |

**Why the camera pose is load-bearing.** With no gimbal the camera pitches with
the body, so `vision/projection.py` composes roll and pitch into every
pixel→ground fix; a translating multirotor holds 10–15° of tilt, which
uncompensated is ~2 m of error at 12 m altitude. Both the mount yaw and the
lens FOV had shipped as **assumptions** and both were wrong — the yaw by ~180°,
which point-mirrored every fix about the aircraft and is exactly the signature
of a sweep that sees pads and never re-acquires them. They are now bench
measurements, and the simulator carries the same 180° mount so the same error
would fail in simulation instead of at the field.

**Exposure — the fix that made in-flight decoding work.** The camera's own
auto-exposure meters the whole frame, which at this field is sunlit grass at
~130 counts; a white pad is four to five times that albedo and clips at 255,
and the marker's black modules bleed into 1–3 px lines. That is why 2,788
frames of one flight decoded nothing with a pad plainly in view. The grabber
now runs a **highlight-priority auto-exposure**: it meters the brightest 0.2 %
of the frame every 0.5 s and steps the manual exposure to hold that between
**190 and 225**, caps the ground mean at 55 when nothing bright is in view, and
never exceeds a 4 ms exposure (about 1.3 px of smear at 3 m/s from 8 m). A
marker carried at running speed across 8–10 m decoded **56–65 % of frames** at
25–35 px on the bench — the sweep's own pixel size at twice its angular rate —
and the 26 August flight decoded ids in the air for the first time.


![Real flight, 26 August 2026: twelve nadir frames in which the detector decoded a pad **in the air**, with the marker outlined and the id it read. Markers 1, 5, 2 and 4 appear here. Every frame is labelled `exposure: highlight-AE` — this is the flight immediately after the exposure fix, on the same camera and the same field that had produced 2,788 frames and zero decodes two flights earlier.](report_figures/decode_real_flight_2026-08-26.jpg){width=100%}

### 3.4 Software system architecture — operating modes and safety features

The autonomy stack is a **deterministic, offline, classical** system on the CM4
(Python 3.12, async throughout, MAVSDK to the flight controller). There is no
LLM, no cloud and no network in flight — network use is an automatic
disqualification. Layers:

- **Mission orchestrator** — the state machine of §2.3; one long-running
  process flies every flight of the window, holding in PREFLIGHT before each.
- **Live plan builder** — rebuilds the plan per flight and re-renders it at
  every gate release and every serve, so the ground station always shows what
  the aircraft is actually going to do next.
- **Vision worker** — ArUco decode plus the white-pad blob cue, with a
  frame-age gate and multi-pad fixes per frame.
- **Pad registry** — id-keyed clusters with k-vote confirmation, median-fused
  geolocation, and a maximum fix distance so a wild projection cannot create a
  target.
- **Terminal controller** — the rung descent, the id-verified LAND gate, the
  centred-final-rung gate and the touchdown-gated release.
- **Safety watchdog** — a 2 Hz phase-aware monitor (below).
- **Time and energy policies** — the window reserve, the per-flight and
  per-delivery gates, and the pack budget with swap detection.
- **Audit trail and verifier** — every flight writes a machine-readable
  `audit.jsonl` (1 Hz telemetry samples, transit pass/miss per point per
  direction, flight and delivery start/release/end lines); a **fail-closed**
  post-flight verifier re-scores the flight against it.
- **Ground station** — a Svelte + FastAPI web GCS: live map with the corridor
  drawn, camera view, the live plan, the ordered 4-of-6 mission-queue editor,
  the per-flight GO, an emergency KILL, and a confirmed-pad readout that shows
  **each pad id as the ArUco glyph itself** so the operator matches the
  committee's card picture-to-picture instead of translating it to a number.

**Operating modes:** PREFLIGHT · TAKEOFF · TRANSIT_INGRESS · SEARCH · LOCALIZE
· LAND · DROP · TRANSIT_EGRESS · RTH · ABORT.

**Companion-side watchdog (2 Hz, phase-aware).** Every check below is active in
flight; none can be switched off:

| Condition | Response |
|---------------------------------------------|-------------------------------------------------------|
| Geofence breach | Return to home (proximity inside the margin = warning) |
| No-fly-zone entry | Return to home |
| Altitude ceiling | Warning above **30.5 m**; sustained above **31.5 m** → return to home |
| Below the 10 m search floor outside the delivery descent | Advisory anomaly |
| Terrain proximity / unknown terrain (lidar) | Advisory, escalating to return to home |
| Battery below **30 %** (planned) | Stop starting anything new; fly the corridor egress home, land, disarm |
| Battery below **15 %** / **10 %** | Return to home / land in place |
| Battery telemetry NaN, sustained | Loud operator anomaly — never a silent loss of battery protection |
| GPS 3D-fix loss, debounced 5 s | Return to home |
| Datalink loss, debounced 5 s | Return to home |
| Telemetry stale > 10 s | Return to home |
| Camera frame older than 2 s | Treated as **no detection** — a frozen image can never satisfy the LAND gate |
| Time budget exhausted | Return to home |
| **Pilot takeover** (manual mode from the RC) | Orchestrator **stands down**; no companion command may fight the pilot |
| **FC-commanded RTL or LAND** we did not ask for | Orchestrator **stands down** (`FC FAILSAFE`) on the first tick |

The last two are recent and were both bought with real flights. On 26 August
PX4's own vertical fence returned an aircraft that was flying correctly, and
the mission never noticed: it kept sweeping, and its next waypoint went out
while the RTL was 0.46 m from touchdown — PX4 obeyed and climbed back out. The
watchdog now records which AUTO mode *we* commanded and treats any other
FC-initiated RTL or LAND as a stand-down. On 27 August the first version of
that rule fired on **our own** landing 0.3 s after the first real egg was
released, so two further rules were added: a LAND while on the ground is never
a failsafe, and an expectation is **consumed** when the flight controller
leaves the mode by itself, never cleared ahead of time by the next command.

**Flight-controller-level failsafes** (these fire even if the companion dies;
all are written at mission start and read back):

- `NAV_DLL_ACT` = Return (datalink loss) · `NAV_RCL_ACT` = Return (RC loss,
  with `COM_RCL_EXCEPT` so an autonomous no-RC flight is not spuriously
  returned) · `GF_ACTION` = **3, Return**.
- Battery: `BAT_LOW_THR` 25 % · `BAT_CRIT_THR` 15 % · `BAT_EMERGEN_THR` 7 %
  with `COM_LOW_BAT_ACT` = 3, arranged so the FC's critical threshold coincides
  with the companion's return-home floor and its emergency threshold sits under
  the companion's land floor — the two layers never race.
- `RTL_RETURN_ALT` = **25 m**: any failsafe RTL is a straight line home over the
  building, the display aircraft and the tree line, and 25 m clears them while
  leaving 5 m under the ceiling for PX4's in-flight home-altitude drift. PX4's
  60 m default would bust the ceiling.
- `GF_MAX_VER_DIST` = 50 m as a gross-runaway net only: the FC's vertical fence
  is home-relative, and PX4 1.17 rewrites the home altitude in flight, so it
  cannot be trusted to hold the rules' ceiling. The ceiling is enforced by the
  companion against a latched home altitude.

**Three defects found in flight and fixed — worth stating because they are the
reason the numbers in this report can be trusted:**

1. **The mission clock ran at 1/20 speed on the real aircraft** (26 August).
   Mission time is read from the vehicle's own clock, and a re-fed identical
   timestamp was resetting the wall-clock anchor, so every deadline — the
   20-minute window, the delivery gates, the leg-progress guard, the touchdown
   timeouts — stretched twentyfold and none of them could have fired at KMITL.
   Flight time is now accumulated per epoch as `min(vehicle, wall)` and a
   re-fed sample moves nothing.
2. **PX4 rewrites `home.alt` in flight** (upstream issue in 1.17), which moves
   the reported relative altitude while the aircraft physically holds station:
   measured shifts of +3.29 / −4.65 / +0.91 / +1.17 / −3.18 m across five
   flights, and one of them returned a correctly-flying aircraft through the
   ceiling watchdog. The height used by the mission is now
   `altitude − home altitude latched at arming`, and home updates are refused
   once airborne.
3. **The geofence action had been "Hold", not "Return", for its whole life** —
   the setter wrote the wrong enum value while its own name, comment and error
   text said RTL. The breach response was therefore "stop and loiter outside
   the fence". The lesson kept in the code: a read-back gate proves a value was
   stored, never that the value means what the caller thinks.

**Software integrity evidence.** **771 automated tests** import the real
modules — the mission loop, the terminal controller, the watchdog, the
detector, the projection maths, the flight clock, the home latch — and all
pass, with linting and static type-checking clean. Behavioural changes are
test-driven; the SITL gate exercises the whole sequence end to end; and every
real flight is re-scored afterwards by the fail-closed verifier.

### 3.5 Payload handling mechanism

Four independent latches carry four boxed eggs. Each is a metal-gear servo on a
Pixhawk **AUX** output configured as a peripheral actuator
(`PWM_AUX_FUNC n = 300 + n`), commanded with **`MAV_CMD_DO_SET_ACTUATOR`
(187)** — PX4 has **no handler for `DO_SET_SERVO`**, so that command is never
used. PWM **1900 µs = release**, **1100 µs = hold**; one latch needs
`PWM_AUX_MAX = 2100` to reach its release angle.

| `payload_id` (release order) | Rack position | AUX pin / actuator set |
|----------------------------------------|------------------------------|------------------------------|
| 0 | Front-left | **4** |
| 1 | Rear-right | **1** |
| 2 | Front-right | **2** |
| 3 | Rear-left | **3** |

The wiring was verified **pin by pin on the aircraft** with `ACTUATOR_TEST`
while disarmed, watching which corner moved — the paper table written before
that test had the order wrong. The same session found the four AUX outputs
sitting on **RC passthrough** functions, i.e. two egg latches wired to the roll
and yaw sticks, which would have released eggs whenever the aircraft banked.

Safety properties of the release path:

- **Never airborne.** The release runs only after the flight controller reports
  a landing, plus a 2 s settle. If telemetry reads airborne the release is
  skipped and audited.
- **Never twice.** Releases are idempotent against a ledger keyed by the
  **mission-global delivery index**, so two deliveries inside one flight can
  never collide and a retry cannot double-fire.
- **Never the wrong hold on a recovery flight.** The crew does not re-rack eggs
  at a battery swap, so a recovery flight continues the wiring-order
  progression through the latches that **still hold an egg**, rather than
  restarting at slot 0 and firing two empty holds.
- **Manual release is gated too.** The ground station's manual DROP is refused
  while telemetry reads airborne unless the operator explicitly forces it.

Per DCOS the payload is securely retained in flight and the mechanism has no
unsafe state: a latch failure leaves an egg aboard, which costs a delivery and
nothing else. The rules' prohibitions on simultaneous multi-cargo release and
on winching are both respected — eggs are released one at a time, on the
ground, one per landing.

### 3.6 Communication system

- **RC and telemetry on one link:** a **RadioMaster NOMAD** dual-band system
  carries both the safety pilot's ELRS control link and the MAVLink telemetry
  the ground station reads (receiver: DBR4 diversity). This replaced a
  telemetry radio that failed at the field, and it means the GCS panels are fed
  by the radio rather than by WiFi — a WiFi dropout never freezes the display.
- **Imagery:** an FPV camera on its own 5.8 GHz VTX satisfies the "transmit"
  half of the rules' imaging requirement, and the on-board frame recorder
  satisfies "record" by writing a JPEG trail to the CM4's own disk. Neither
  half depends on the WiFi link, so the live dashboard image is a debug
  convenience that can be switched off on competition day at no cost.
- **Frequencies:** ELRS/NOMAD and the VTX operate inside the recommended
  920–925 / 2400–2500 / 5725–5850 MHz bands. **No 4G/LTE, no SIM, no internet**
  — their use is a disqualification, and the field network is the CM4's own
  local access point.
- **On-board routing:** a MAVLink router fans the flight link to the
  orchestrator, the ground station, and an optional QGroundControl endpoint
  that **defaults to loopback**, so the armed aircraft's control plane is never
  exposed unauthenticated on a field network.
- **Standby-area discipline (procedural).** The rules forbid activating radio
  control and telemetry while in the standby area, and this aircraft raises its
  own access point at boot. The procedure is therefore: **do not connect the
  battery until the launch point**, and keep the ground console closed while
  waiting.

### 3.7 Operating procedures (normal and emergency)

**Before the field day.** A parameter tool reads back the flight controller's
own state and splits it into a **STOP block** — the values that live on the
board and must be correct before flying (airframe, motor map, battery gauge
endpoints, height reference, sensor ports, fence) — and an informational block
of values the orchestrator pins at mission start. This exists because the motor
map was once found reading zero, i.e. the aircraft was unflyable and looked
normal; the cause was never identified, so it is treated as a regression that
can recur and is re-read every field day rather than trusted.

**Normal operation.** The operator enters the committee's four assigned ids
once, as an ordered queue, using the ArUco glyphs shown on screen. Then, per
flight: the readiness board goes green (link, arm-ready, EKF, home, sensors,
GPS fix, battery ≥ 25 % resting, on-ground, geofence, fresh camera frame) → the
operator confirms the eggs are racked and the crew is clear → **one GO click**
→ the aircraft flies the whole flight autonomously → it lands and disarms at
L&R. The window clock starts at the first GO; the gates manage the schedule and
refuse a launch that cannot finish, unless the operator explicitly accepts the
overtime penalty.

**Battery swap and recovery flight.** If the planned 30 % egress fires, the
aircraft comes home through the scored corridor and disarms. The crew swaps to
the second charged pack; the between-flights gate waits as long as the swap
takes. The next GO launches a **recovery flight** which flies direct to the
pads still owed — they are already in the registry — and fires only the latches
that still hold eggs.

**Emergency operation.** The safety pilot can take manual control at any
instant, and the orchestrator stands down the moment it detects the takeover.
The ground station's KILL cuts motors, guarded by an armed command session so
that an accidental click cannot fire it. Automatic failsafes — the companion
watchdog and the FC-level RTL on geofence, datalink, RC and battery — recover
the aircraft without operator action. On any unhandled mission error the
orchestrator commands an emergency return and land, and the FC failsafe sits
underneath that. Retries within the window are free under the rules, and the
mission's own registry makes a restart cheap: the pads are already known.

**Brief for the safety pilot.** A commanded mode change may need to be repeated
once: the orchestrator stands down on the first tick, but a command already in
the MAVLink pipeline (~0.5 s) can still reach the aircraft.

### 3.8 Airworthiness — DCOS compliance summary (rules Appendix B)

| DCOS criterion | Compliance |
|--------------------------------------|--------------------------------------------------------------|
| Flight-performance margin across all load configurations | T/W ≈ 3.2 : 1, hover at ~31 % thrust, loaded and empty (§3.2) |
| Envelope protection (tilt, rates, speed) | PX4 limits: 30° tilt, 25 °/s auto yaw, 5 m/s max horizontal, 1.5 m/s climb (§3.2) |
| Propulsion excess-thrust margin | ~69 % throttle headroom at hover (§3.2) |
| Power reserve (no energy starvation) | Planned 4,700 mAh of 12,750 usable; planned 30 % corridor egress; FC battery failsafes below it; second pack + recovery flight (§3.2, §3.7) |
| Airframe load (flight / ground / transport) | §3.1 — envelope-limited manoeuvre load, 0.3 m/s touchdown, folding arms |
| Redundancy | **Six rotors** (rotor-out keeps attitude and thrust); **two battery packs**; **three height sources** (lidar, barometer, GPS); FC and companion are independent computers, and the FC's failsafes fire without the companion |
| Hardware compatibility (protocols, power, comms) | MAVLink throughout; separate avionics and motor power paths; §3.6 band compliance |
| Secure fastening / payload retention | Four independent latches, hold at 1100 µs, retained on failure (§3.5) |
| Clearance between moving parts | ~43 mm prop-tip gap, no rotor overlap (§3.1) |
| Secure wiring and environmental protection | Strain-relieved looms; separate avionics rail; the companion on its own supply (§3.1) |
| Envelope protection and fail-safe modes activatable at any time | Companion watchdog (2 Hz, non-optional) + FC failsafes + pilot takeover + GCS KILL (§3.4) |
| Normal and emergency procedures | §3.7 |
| PPE per role | §1 |

---

## Appendix A — Bill of materials, as built

The aircraft is not the one this report's version 1.1 costed. The frame, ESCs,
propellers and both battery packs were **lent or donated by DRON (Defence
Research Operation Network)**; the flight controller, companion computer, radio
chain and ground equipment were already owned or were paid for by the team
members themselves. The list below is what actually flies.

| Group | Item | Part | Qty | Source |
|------------|----------------|------------------------------------------|----------|--------------------|
| Airframe | Frame | **EFT X6100** hexa-X, 1.000 m wheelbase, folding carbon arms | 1 | DRON |
| Propulsion | Motors | **EFT E5 5008, 335 KV** | 6 | DRON |
| Propulsion | Propellers | **18 × 6.5 in** | 6 (+ spares) | DRON |
| Propulsion | ESCs | 6 × PWM ESC (no telemetry lead) | 6 | DRON |
| Power | Mission pack | **6S 17,000 mAh semi-solid**, 10 C, 1.5 kg | 1 | DRON |
| Power | Spare pack (swap / recovery flight) | 6S 15,000 mAh | 1 | DRON |
| Power | Power module | Holybro **PM02D** — flight controller and avionics only | 1 | owned |
| Power | Companion supply | Dedicated power-bank rail for the CM4 | 1 | owned |
| Avionics | Flight controller | **Holybro Pixhawk 6X**, PX4 v1.17.0 | 1 | owned |
| Avionics | Companion computer | **Raspberry Pi CM4** + carrier | 1 | owned |
| Avionics | GNSS / compass | Non-RTK GPS module (cable replaced after an intermittent fault) | 1 | owned |
| Sensing | Nadir camera | **Meige OV9281** USB UVC, 1280 × 720 mono global shutter, 74.2° measured | 1 | owned |
| Sensing | Rangefinder | **Benewake TFmini-S**, TELEM3 | 1 | owned |
| Imaging | FPV camera + 5.8 GHz VTX | "transmit" half of the imaging requirement | 1 | owned |
| Radio | RC + telemetry | **RadioMaster NOMAD** (dual-band) + **DBR4** receiver + TX16S | 1 set | owned |
| Payload | Egg latches | 4 × metal-gear servo on AUX 4/1/2/3 + rack | 4 (+1 spare) | team-built |
| Ground | GCS | Laptop running the team's web GCS; CM4 local access point | 1 | owned |

Retired from the version-1.1 BOM and **not** carried: the X500 V2 quad frame,
the 10-inch propulsion set, the camera gimbal servo, the ARK Flow optical-flow
module and the TF-Luna rangefinder. Two of those are worth noting as deliberate
subtractions rather than omissions — the gimbal (the camera is hard-mounted and
the projection composes attitude instead) and optical flow (not in the kit, and
pinned off so it cannot silently re-enable).

---

## Appendix B — Validation evidence

### B.1 Simulation (PX4 SITL + Gazebo, KMITL field geometry)

The full mission is flown in simulation against an independent, **fail-closed**
post-flight verifier. Ground truth is used only for the audit — never for
planning.

| Run | Configuration | Result |
|------------|------------------------------|----------------------------------------------------------|
| 22 Jul 2026 | First hexacopter model, one egg per flight, 4 flights | **4/4** delivered id-correct, releases **0.13–0.25 m**, transit **8/8** in order, 14.8 min, verifier **19 checks / 0 warnings** |
| 25 Jul 2026 | **Four eggs in one flight**, 6-pad field | **4/4** delivered id-correct, releases **0.15–0.21 m**, transit 6/6, max altitude 19.69 m, landed 2.6 m from L&R, **457 s**, verifier **14 / 0** |
| 24 Aug 2026 | **Competition envelope** (transit 20 m, ceiling 20, sweep 12 m — the 28 px decode case), 4 eggs | **6/6 pads found, 6/6 ids correct, 4/4 eggs**, releases **0.09–0.22 m**, transit 6/6, landed 2.0 m from L&R, **536 s of 1200** |

The 24 August pair was flown head-to-head to settle the sweep overlap with
numbers rather than argument: 0.30 and 0.44 overlap on the same pad layout gave
**identical coverage and identical delivery success**, with 0.30 **111 s
faster** there and a computed 158 s faster on the KMITL polygon. Decode was
separately measured at every across-track offset out to 94 % of the half-swath,
so the wider spacing never puts a pad where the detector fails.

The same session produced the most instructive failure in the project: with the
simulator's camera still mounted at yaw 0 against a configuration that had just
been set to the **measured** 180°, the aircraft **decoded all six pads with
every id correct and delivered 0 of 4**, flying to empty grass 3.7–20.2 m from
each pad. That is the signature of a point-mirrored projection, and it is now
asserted by a test so the two mounts can never disagree again.


![Simulation replay of a full four-egg mission. The main image is the aircraft's own nadir camera over a pad at 8.4 m; the overlay carries mission time, AGL, battery and phase, and the inset shows the flown track — the transit corridor in green and the completed survey sweep in orange. Ground truth from the simulator is used only for the post-flight audit, never for planning.](report_figures/sitl_mission_replay.png){width=78%}

### B.2 Real flights (KMUTNB practice field, standing in for KMITL)

| Date / time | Duration | What was flown | Outcome |
|------------|------------|-------------------------|---------------------------------------------------|
| 20 Aug | 3 flights | First outdoor autonomy: transit + sweep | **Found:** every waypoint was sent with an undefined yaw, so PX4 turned the nose at each one — **867° of yaw in 122 s**, and the body-fixed camera spun with it (1 of 457 frames decoded). **Found:** the GPS height reference inflated the reported AGL and returned a flight that was tracking transit to 1.7 m. Both fixed (single sweep heading; barometric height reference) |
| 21 Aug | bench, props off | **Pilot-takeover drill** on the real flight controller and companion | Stand-down at the 1 s debounce verified; no companion command can fight the pilot |
| 24 Aug | 2 flights (from the FC's own log) | Take-off gate investigation | **Found:** PX4 leaves AUTO_TAKEOFF as soon as it is within `NAV_MC_ALT_RAD` of the target, levelling off 0.8 m low and aborting an otherwise perfect mission at take-off |
| 26 Aug | 3 flights | Sweep with the new envelope | **Found:** PX4's own vertical fence returned a correctly-flying aircraft (its home altitude had walked 14 m in 93 s), the mission did not notice, and its next waypoint reached the aircraft while the return was **0.46 m from touchdown**. **Found:** 2,788 frames, **0 decodes**, with a pad plainly in view — the white pad was clipped at 255 by an exposure metered on grass. Both fixed (FC-failsafe stand-down, home-altitude latch, highlight-priority exposure); ids decoded in the air later the same day |
| 27 Aug 12:44 | 5 min 56 s | **Full 7-leg survey**, 6,044 frames | **5 of 6 pads decoded and confirmed** (votes 13–14, confidence 0.95). The sixth pad is **printed with marker id 0** and was being filtered out — the reason the valid set is now 0–6 |
| 27 Aug 14:13 | — | Survey + serve | Confirmed all three of its assigned ids at **t = 52 s** and kept sweeping to leg 5 of 7. The sweep now stops as soon as every id the flight owes is confirmed |
| 27 Aug 14:51 | — | First delivery attempt | **First real egg placed on a pad: 0.14 m from centre**, released after touchdown. Exposed a false failsafe on our own landing, fixed the same day |
| **27 Aug 15:25** | **4 min 12 s** | **Complete mission, end to end** | Transit ingress **3/3** → survey found 5 pads in ~20 s → early stop on the assigned ids → **land-ON pad 1, release; land-ON pad 4, release** → egress → landed **0.7 m** from the launch point → disarm. **2/2 delivered.** Aircraft health: 14–16 satellites, horizontal accuracy ≤ 2 m, companion CPU 41 %, hover throttle 0.59–0.64, allocator never saturated, and the home-altitude latch held silent through five home rewrites |

### B.3 What the flight record establishes

- The **full sequence works on real hardware**: gate, transit, survey, registry,
  id-verified descent, land-ON, touchdown-gated release, egress, landing,
  disarm.
- **Release accuracy on the real aircraft (0.14 m)** is consistent with the
  simulated 0.09–0.30 m band, so the terminal controller is not
  simulator-flattered.
- Every safety layer added since 20 August has been **exercised in flight**, not
  only in tests: pilot takeover, FC-failsafe stand-down, the home-altitude
  latch, the early-stop sweep, and the release idempotence.
- The **remaining unknown is endurance**, not capability (§3.2, Appendix C).

---

## Appendix C — Open items and risk register

Stated in the order they matter on competition day.

| # | Item | Status and mitigation |
|----|---------------------|---------------------------------------------------------------------------|
| 1 | **Battery endurance vs the KMITL plan** | The indicated gauge reaches the 15 % return floor at roughly minute 8.5–10; the plan needs 10–13 minutes. **Mitigation, all implemented:** per-delivery time-and-battery gate (gives up eggs, not the aircraft), planned 30 % egress through the scored corridor, second charged pack, automatic recovery flight from the unfired latches. **Action:** record the mAh returned by the charger after every flight — it is the one measurement that closes the question |
| 2 | **The 28 Aug briefing changes are unflown** | 30 m ceiling, new corridor P2′/P3′, cut search area, 20 m survey with 10.5 m decode visits, 25 m failsafe RTL. The **13:00–18:00 trial slot is the only test** before scoring |
| 3 | **Committee coordinates not published** | Our corridor and no-fly polygons are georeferenced from briefing photographs to **±5 m**. Committee-published coordinates replace ours in both the flight configuration and the ground station, same digits in both |
| 4 | **Where the pads are placed** | The flown search area is cut at E 110 m, west of the main building. **Pads placed east of the building would not be searched.** Question to ask the committee before the scored flights; routing around the building box is a day-2 change |
| 5 | **RC-loss drill** | The parameters are pinned and read back, but the net itself has never been exercised in the air. The diagnosis of an earlier "no pulses" indication is closed (a held kill switch, not the link) |
| 6 | **CG record and structural margin** | Not formally documented (§3.1). The aircraft is trimmed by flight behaviour; the cargo boxes are heavier than the dummies in the weighed figure, so it is re-weighed with real cargo at the field |
| 7 | **Motor-map regression** | The motor map was once found reading zero with no identified cause. It is re-read from the board before every field day rather than trusted |
| 8 | **Marker id 0 in the field configuration** | The flight detector accepts ids 0–6 (correct). The `marker.valid_ids` list in the field configuration still reads 1–6; it is consumed only by the simulator's pad spawner, so it cannot affect a real flight, but it should be widened for consistency |
| 9 | **Sim-only descent detail** | The per-rung descent ladder writes the manual-mode descent parameter rather than its AUTO twin; the effective pad approach is the pinned 0.4 m/s that every validated landing has flown. Do not "unpin" it before the competition |

---

*Report structured to AAVC 2026 Rules & Regulations V1.3, Appendix A, as
amended by the 24 July and 28 August 2026 event briefings. The flight system is
deterministic, classical-CV and offline: no machine learning in the flight loop,
no cloud, and no network — network use is an automatic disqualification.
Supporting documents in this repository: `docs/RULES_AAVC2026.md` (rules
digest), `docs/FLIGHT.md` (field procedure), `docs/SERVO_AUX_MAPPING.md`
(payload wiring), `docs/evidence/` (flight and bench evidence).*
