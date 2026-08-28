# AAVC 2026 — Rules & Regulations digest (V1.3, July 2026)

Source of truth: `AAVC2026_RulesAndRegulation_V1.3_140769.pdf` (repo root;
18 pages, read in full 2026-07-15) — **overridden where noted below by the
2026-07-24 event briefing**, since every PDF page carries a "subject to be
changed" watermark (see "Event-briefing override" below). This digest maps
every rule the software must honour to the config/module that implements it.
On any conflict, the PDF wins UNLESS a briefing override below says
otherwise — update this file and the config together.

## V1.3 changes (editorial-only over V1.1 — verified by full-text + figure diff)

**No flight-rule changed.** Every coordinate (geofence, search area, P1/P2/P3),
altitude (20 m transit / 10 m floor / 20 m ceiling), pad geometry (1000 mm pad,
⌀750 mm circle, 400 mm marker, 25 mm stroke), marker set (DICT_4X4_50 ids 1–6),
sortie mechanics (≤4 pads — **superseded 2026-07-24, see "Event-briefing
override" above**; 5+20-min window, per-minute overtime penalty, free
retries), and scoring criterion is **identical** to V1.1. What V1.3 actually
changed:

- **Coordinates now also printed in DMS** alongside the decimals (the decimal
  values are unchanged — config keeps the decimals).
- **Scoring table transit labels fixed.** V1.1 mislabelled BOTH transit rows
  "(outbound)"; V1.3 labels the ingress row "(inbound)" and the egress row
  "(outbound)". The substance is unchanged — points per coordinate passed in
  ingress order, then in egress order (our audit uses ingress/egress terms).
- **Cargo package specified** (Fig 5): a heart-shaped box, **~16 × 7 × 18 cm**,
  **300-gsm art card**, with a handle loop — organiser-provided, one raw egg
  (no.0) inside. The egg-hold bay must physically accept this box (→ G5/G6
  hardware; `docs/FLIGHT.md`, `docs/BOM_REPORT.md`).
- **Imaging requirement wording** now explicitly says the imaging system must
  **record AND transmit** (record was implicit in V1.1). We satisfy both, and
  **neither half needs WiFi** (2026-08-16, once the aircraft gained a separate
  FPV camera): **record** = the mission frame recorder writing to the CM4's own
  disk (`orchestrator/frame_recorder.py` → `runs/<id>/frames/`, config
  `recording:`) — local, no link at all; **transmit** = the **FPV video
  downlink** on its own VTX. The live dashboard image over WiFi was the previous
  answer to "transmit" and is now a **debug convenience, not a scored
  requirement** — it can be dropped on competition day at no cost. Note the two
  halves are covered by two different things: FPV alone would not satisfy
  "record" (a VTX transmits, it does not store), and the frame recorder alone
  would not satisfy "transmit".
- **"subject to be changed" watermark** on every page — treat all figure-derived
  coordinates (no-fly zone, L&R) as provisional; keep them config-tunable and
  **re-measure at the event briefing**.
- Ops-schedule wording (tech-exchange session moved to the 28 Aug morning,
  accommodation clause reworded) — no software impact.

## Event-briefing override (2026-07-24)

At the 28 Aug event briefing the committee's stated field differs from the
PDF numbers elsewhere in this file, in three linked ways. This is not a
contradiction to resolve — **every page of the PDF carries a "subject to be
changed" watermark** (see "V1.3 changes" above), so the briefing, being the
committee's most recent statement, governs. This section records the
override; the PDF text below is left in place and marked **superseded**,
not deleted, so the original source stays visible.

| PDF said | Briefing says | Config key / module |
|---|---|---|
| §3/§5: **"Up to four (4) pads"** placed on the field | **SIX** pads are physically placed, ids 1–6 — the whole of `DICT_4X4_50` ids 1–6; "four" was always how many get **assigned**, not how many are **placed** | `sitl.n_pads: 6`, `search.max_pads: 6` (`sitl/spawn_targets.py` places all 6); the 2 unassigned ids are permanent distractors the id-verified LAND gate must keep rejecting for the whole mission |
| (the same "four", read as an assignment count) | **FOUR** pads assigned per team, out of the six placed — unchanged in substance, now stated explicitly alongside the six | `mission.max_deliveries: 4`, `mission.assigned_marker_ids` / `state.assigned_id_queue` (the 4-of-6 mission queue, GCS queue editor or `--assigned-ids`) |
| §3: "a new payload–pad pair is assigned per **flight**", read (V1.1/V1.3 digest below) as one egg per sortie | **All four assigned eggs are carried in ONE flight** (one arm→disarm cycle); a flight now serves up to `eggs_aboard` pads, each its own **DELIVERY** | `mission.eggs_aboard: 4`, `connection.drop_payload_count: 4` (`payload_id` 0–3 → actuator set / AUX 4/1/2/3 via `drop_servo_channels`, docs/SERVO_AUX_MAPPING.md); loop shape `orchestrator/mission.py::run_delivery_mission` (`for flight: for delivery`) |
| §8: scoring matrix scored **"per sortie, accumulated"** | Scoring is **per DELIVERY**: each pad served — not each flight — earns its own transit/identification/landing/cargo points, independent of how many other pads share its flight | Audit grammar `DELIVERY k START\|END\|RELEASE` (`k` numbered 1-based across the WHOLE mission, one block per pad); `tools/verify_flight.py` scores every delivered `DELIVERY` on its own evidence |

`eggs_aboard=1` is the one-integer rollback to the pre-briefing
one-egg-per-flight model (still fully supported and regression-tested —
`mission_brain/flights.py`, `tests/test_flights.py`). See `CLAUDE.md` §2 for
the FLIGHT ⊃ DELIVERY model and §5 for the resulting audit grammar.

## Event-briefing override (2026-08-28, competition-day briefing)

At the 09:00 briefing on 28 Aug the committee amended the envelope and the
corridor. The operator relayed the amendments from the field the same
morning, with the new corridor sketched by hand on the committee's slide
(page 3 of 18, "transit route (yellow) and emergency egress"); the slide
carries the corridor as a picture, so the P2'/P3' coordinates below are
OURS — georeferenced from that sketch, ±5 m — not the committee's. Same
standing as the 2026-07-24 section: the briefing governs over the PDF and
the PDF text below is marked superseded, not deleted.

| PDF said | Briefing says | Config key / module |
|---|---|---|
| §6: ceiling **20 m** AGL; search-area band 10–20 m | Ceiling **30 m** AGL; search band **10–30 m**. The 10 m floor and the pad-only descent below it are unchanged | `mission_brain/profile.py` COMPETITION `altitude_ceiling_m=30` — the companion watchdog rides on it (warn 30.5 / RTH 31.5, `orchestrator/constants.py`); `sitl/kmitl_config.yaml` `mission.altitude_ceiling_m: 30` (the line `tools/verify_flight.py` reads) |
| Table 1: P2 13.730397, 100.788694 · P3 13.730712, 100.788755 — the north leg ends ON the main building's west wing | The north leg moves west, off the building: **due north at E 123.7 m from L&R** — the operator's own line on the gridded map — **P2′ 13.730390, 100.788590** (ENU 123.7 E, 7.6 N) · **P3′ 13.730715, 100.788590** (ENU 123.7 E, 43.8 N). P1 = L&R unchanged; still mandatory both ways, still 20 m | `transit_route` in `sitl/kmitl_config.yaml`; `aavc-gcs/aavc_field.yaml` `transit_waypoints` (same digits). ⚠ Committee-published coordinates, if any, replace ours in BOTH files |
| §3: pads anywhere in the search area | Pads are placed **in the open only** — never on the building roof, never beside an obstacle | No key. It is what makes a decode visit at 10.5 m (`orchestrator/mission.py::_decode_visits`) over a white-pad candidate safe, and why the sweep can fly at 20 m (`search.sweep_alt_m`) and leave the reading of ids to the visit |
| (nothing) | "**25 m clears everything** on the field" — the obstacle-height statement | `px4_tuning.RTL_RETURN_ALT: 25.0` (was 19.5): a failsafe RTL is a straight line home over the building, the display aircraft and the tree line; 25 clears them and leaves 5 m under the ceiling for PX4's in-flight home-alt drift |
| Figure 1: ONE no-fly block (orange, south of the search area's east end, no coordinates) | **Two no-fly bands** marked by the operator on the gridded map (±3 m): the **building band** — the main building and the courtyard east of it, ENU E 125.6–266 / N 42–75.3 — and the **east band**, E 219.8–266 / N 42–116 (the orange block, E 219–255 / N −14..40, still stands south of the search area) | The flown `search_area` is the rules' polygon minus the bands — an **L** (west field + the strip north of the building); the sweep is hand-laid (`search.sweep_waypoints_enu`) and every goto is routed around the bands through one gateway (`routing.keepout_zones` / `routing.gateway`, `orchestrator/mission.py::_goto_routed`); candidates or pads inside a band are refused. `no_fly_zones` stays `[]` (no approximate polygon in the watchdog — operator 2026-08-27; the corridor runs 2 m from the building band). ⚠ PX4's own RTL cannot be routed |

Unchanged by the briefing as relayed: transit altitude **20 m** (Table 1
was not restated — `profile.transit_alt_m` is the one line to change if it
was), the 10 m search floor, P1 = L&R, the corridor being mandatory in both
directions and scored per point. **Still open from the same briefing:** the
committee's OWN coordinates for the corridor and the no-fly bands (ours are
±5 m from photos), and **where the pads are placed** — the plan searches
only west of the building.

## 1. Event

- **28–30 Aug 2026**, International Academy of Aviation Industry (IAAI),
  KMITL Ladkrabang, Bangkok.
- 28 Aug: arrival 08:00–09:00, briefing 09:00, safety inspection, then
  **flight trials + technical exchange 13:00–18:00** (15-min trial slots,
  first-come-first-served; ~15-min team presentation + 5-min Q&A).
- 29–30 Aug: **flight operational challenge days** (gather before 07:30).
- Team ≤15 members incl. 1 faculty advisor, ≤1 graduate student.

## 2. Operation slot

- **25 minutes total = 5 min setup + 20 min operation window.** Ready early →
  the 20-min countdown starts immediately; not ready at 5 min → it starts
  anyway.
- Cargo + **the delivery assignment (pad marker id)** are handed over while the
  vehicle is weighed by the field committee.
- Crew inside the GCS operating area: **1 safety pilot per vehicle, ≤4 C2
  operators, ≤3 technical support**.
- Retries (return mid-operation on malfunction, restart) are free **within**
  the window. **Once the window is exceeded: negative points per minute.**
- The safety committee can abort operations at any time.

→ software: `profile.operation_window_s=1200`; the clock starts at the FIRST
operator GO (`state.start_window()`); `TimePolicy.can_start_sortie` gates
starting another FLIGHT and `TimePolicy.can_start_delivery` additionally
gates starting each individual DELIVERY inside a multi-egg flight (skips the
remaining eggs and comes home rather than start one it can't finish) +
`/api/cmd/preflight/go` `force` flag; per-minute penalty is the operator's
decision to accept.

## 3. Mission (fig. 4 profile)

1. Load cargo → 2. Launch → 3. Outbound transit → 4. Search for the landing
pad → 5. **Land ON the pad & release cargo** → 6. Egress → 7. Inbound transit
→ 8. Landing at launch & recovery.

- **Cargo:** one large (no.0) raw egg in a heart-shaped paper package,
  organiser-provided. Points for intact condition → release only after
  touchdown, gently (`settle_after_land_s`, `MPC_LAND_SPEED=0.3`).
- **Repeat deliveries are highly encouraged**: resupply (cargo and/or battery)
  between flights; a new payload–pad pair is assigned per flight; repeat until
  all pads are served or the window ends. **Up to four (4) pads** are placed
  *(superseded 2026-07-24 — see "Event-briefing override" above: SIX pads
  are placed; four are ASSIGNED per team, all in one flight)*.
- Single or multiple cooperative vehicles allowed (we fly ONE hexacopter).

→ software: `run_delivery_mission`'s `for flight: for delivery` loop
(`orchestrator/mission.py`); `mission.max_deliveries=4` pads assigned,
`mission.eggs_aboard=4` carried per flight (briefing override, above);
per-flight assignment (a chunk of up to `eggs_aboard` ids) via the dashboard
GO / `--assigned-ids`.

## 4. Landing pad (fig. 6/7)

- **1000×1000 mm white pad**, black **circle ⌀750 mm** (~25 mm stroke),
  central **ArUco marker 400×400 mm**.
- Dictionary: **"standard 4×4"** generated at chev.me/arucogen ⇒ OpenCV
  **`DICT_4X4_50`**; ids used: **1 through 6**.

→ software: `vision/detectors/aruco.py` (`find_landing_pads`,
`render_pad_bgr`), config `marker:` block, `tools/gen_pads.py` textures.

## 5. Field geometry (WGS-84, from the PDF)

| What | Coordinates | Config key |
|---|---|---|
| Controlled airspace (geofence, ~296×167 m) | 13.731312,100.787175 · 13.731359,100.789916 · 13.729994,100.789841 · 13.729806,100.787228 | `controlled_airspace` |
| Search area (~210–227 × 57–74 m) | 13.731239,100.787824 · 13.731359,100.789916 · 13.730703,100.789776 · 13.730723,100.787840 | `search_area` |
| Transit P1 (initial/final + RTH hold) | 13.730322, 100.787446 | `transit_route[0]` |
| Transit P2 (intermediate) | 13.730397, 100.788694 — **superseded 2026-08-28 → P2′ 13.730389, 100.788567** (see the 2026-08-28 override) | `transit_route[1]` |
| Transit P3 (search ingress/egress) | 13.730712, 100.788755 — **superseded 2026-08-28 → P3′ 13.730716, 100.788544** | `transit_route[2]` |
| Launch & Recovery (**APPROXIMATE** — purple area in fig. 1/2, not published as coordinates) | ≈ 13.730250, 100.787300 | `ground_operation.launch_recovery`, `site.center` |
| No-fly zone (**APPROXIMATE** — red block in fig. 1, figure-only) | ≈ lat 13.7302–13.7307, lon 100.7891–100.7897 | `no_fly_zones[0]` |

**Re-measure the two APPROXIMATE entries at the event briefing** and update
`sitl/aavc_config.yaml` (the ENU comments + world/spawner all key off
`site.center`).

## 6. Flight restrictions

- All flights take off + land at the designated L&R site only.
- **The transit route is mandatory** to enter and leave the search area.
- **Transit altitude strictly 20 m.** In the search area: **minimum 10 m
  AGL**; **descending below 10 m is allowed ONLY for the delivery on the
  landing pad**; maximum in the search area **20 m AGL** — **superseded
  2026-08-28: the ceiling is 30 m AGL** (search band 10–30; see the
  2026-08-28 override above).
- Leaving the controlled airspace (geofence) or entering a no-fly area is
  strictly prohibited.
- No RC/telemetry transmission while in the standby area. PPE for flight-line
  crew.

→ software: `profile.transit_alt_m=20`, `search_floor_m=10`,
`altitude_ceiling_m=30` (20 until 2026-08-28); `SafetyWatchdog`
ceiling/no-fly/floor + geofence; `RTL_RETURN_ALT=25` at KMITL (19.5 under
the old ceiling) keeps failsafe returns legal AND above the obstacles; the
align descent (LOCALIZE/DROP/LAND phases) and the 10.5 m decode visits are
the only flight under 20 m inside the search area.

## 7. System restrictions

- Combined MTOW of all vehicles **≤ 25 kg**.
- Team assembles/integrates the airframe + subsystems themselves (third-party
  supervision OK). Open-source flight stack into a commercial "empty airframe"
  OK; **complete proprietary flight packages are prohibited** (PX4 ✓).
- **No winching** mechanism; **no simultaneous multi-cargo release**; multiple
  cargo allowed only with independent release mechanisms.
- Telemetry ≥500 m LOS over the operation area. Recommended bands:
  **920–925 MHz / 2400–2500 MHz / 5725–5850 MHz**. **4G LTE (SIM) is banned.**
- GCS: primary terminal + safety-pilot RC + comm tower + backup power; must
  relay flight + imaging data to **≥1 external display** for evaluation
  (→ the Svelte dashboard).
- Imaging system that records/captures and transmits to the GCS (EO/IR);
  a separate landing camera is allowed; no restriction on the localisation
  technique except the minimum operating altitude.

→ software: `connection.drop_payload_count=4` — four INDEPENDENT release
mechanisms (`payload_id` 0–3 → actuator set / AUX **4/1/2/3** as wired,
`connection.drop_servo_channels` →
`mavlink_adapter/commands.py::DroneCommander.drop_payload`); each DELIVERY
releases exactly one `payload_id`, at most one per touchdown — never
simultaneous.

## 8. Scoring matrix (per sortie, accumulated over the day)

| Criterion | Points for | Bonus |
|---|---|---|
| Take-off | successful takeoff | autonomous |
| Transit (inbound) | **each transit coordinate passed in ingress order** | — |
| Pad identification + localization | identifying the pad matching the assignment; **obtaining the pad's coordinate** | significant, if autonomous |
| Landing + release | cargo placed onto the pad; **landed on the pad BEFORE releasing** | significant, if fully autonomous |
| Transit (outbound) | each transit coordinate in egress order | — |
| Landing | landing within the L&R area | autonomous |
| Condition after landing | vehicle returned undamaged | — |
| Cargo condition | egg intact | — |
| Repeating delivery | every sortie scored with all criteria above, accumulated | — |
| Operation time penalty | **negative points per minute** past the window | — |

→ software traceability: transit passes audited per point
(`TRANSIT_PASS/MISS ... flight=n`); the confirmed pad's id + lat/lon shown on
the GCS ("Confirmed pads" readout); touchdown-gated release; per-flight
(`FLIGHT n START|END`) AND per-DELIVERY (`DELIVERY k START|END|RELEASE`,
scored independently — see "Event-briefing override" above) summaries in
`runs/<mission_id>/audit.jsonl`; `tools/verify_flight.py` asserts all of it
post-flight.

## 9. Technical documentation (for reference)

- Preliminary report deadline **20 June 2026**; technical document (≤30 pages)
  deadline **20 July 2026** — scored (executive summary 10, team org 10,
  concept development 30, engineering design & analysis 40) and used to order
  the operating time slots.
- Appendix B (DCOS) = airworthiness guidelines: envelope protection, fail-safe
  modes activatable at any time, redundancy, secured wiring/components, PPE —
  the safety inspection checks these.
