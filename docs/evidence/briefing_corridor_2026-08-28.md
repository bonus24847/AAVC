# Briefing 2026-08-28 — corridor, no-fly bands and the flown search polygon (georeferenced)

Evidence for the `rules/2026-08-28-briefing-ceiling-30` change set. Two operator
photos taken at the 09:00 competition-day briefing were georeferenced against
the Esri World Imagery z19 tiles the GCS caches (`aavc-gcs/tiles/19`,
0.290 m/px at this latitude):

* `IMG_20260828_104353.jpg` — the committee's slide (page 3 of 18, "transit
  route (yellow) and emergency egress"), a north-up Google-Maps screenshot,
  with the NEW ingress corridor sketched on it by the operator in green.
* `IMG_0550.jpg` — the rules' field figure (yellow search area, orange
  no-fly block, red dashed airspace) with the operator's red boxes marking the
  areas the committee declared un-flyable: the main building and the east end
  of the search area.

## Method

1. Overlays (yellow/red/green/orange hues) masked out; SIFT on CLAHE'd
   greyscale; BFMatcher ratio test; `cv2.estimateAffinePartial2D` (similarity,
   RANSAC 4 px). Slide: 10-15 inliers, scale 0.326-0.345 mosaic-px per image
   px, rotation 0.3-1.1 deg; refined locally with `cv2.findTransformECC`
   (Euclidean, high-passed, cc 0.33). IMG_0550: 14 inliers, scale 0.7295,
   rotation 0.02 deg, median residual 0.7-0.9 px — stable across ratio
   thresholds to 0.4 px.
2. Cross-checks on the slide: the drawn yellow corner sits 0.2 m from the
   PDF's P2 easting and 4.4 m south of the P1-P2 line (the yellow route is
   schematic — drawn axis-aligned while the PDF's P2-P3 leg bears 11 deg);
   the drawn search band and red dashed lines land within 3-5 m of the
   config polygons.
3. The green stroke was traced row by row (G-dominance); its three anchor
   points map to mosaic (766.0, 653.2) / (760.9, 545.1) / (756.4, 528.5) —
   a line 3.8 deg west of north. P2' = its intersection with the PDF's
   P1->P2 line; P3' = its crossing of the search-area south edge.
4. Precision: the fits agree to 1-3 m in the corridor region; the finger
   stroke itself is the limit. **±5 m.** Committee-published coordinates,
   if any, replace these.

## Results (WGS-84; ENU metres from L&R = P1 13.730322, 100.787446)

| point | lat, lon | ENU E, N | vs PDF |
|---|---|---|---|
| P2' | 13.730389, 100.788567 | 121.2, 7.5 | 13.8 m W of P2 |
| P3' | 13.730716, 100.788544 | 118.7, 43.9 | 22.8 m W of P3 (which was ON the building) |

Ingress P1-P2'-P3' = 157.9 m (was 170.9). The north leg runs
2.3-3.6 m west of the building box's drawn west edge (E 122.3).

No-fly bands (corner order NW, NE, SE, SW):

* **building** — E 122.3–185.7 · N 25.2–84.3
  13.731079, 100.788577
  13.731078, 100.789163
  13.730548, 100.789162
  13.730548, 100.788577
* **east band** — E 215.4–256.1 · N 37.4–116.3
  13.731367, 100.789439
  13.731367, 100.789814
  13.730658, 100.789814
  13.730658, 100.789438
* **orange block (rules Fig. 1)** — E 218.7–254.6 · N -14.0–39.7
  13.730679, 100.789468
  13.730679, 100.789800
  13.730196, 100.789800
  13.730196, 100.789468

Flown `search_area` = the rules' polygon cut at E = 110 m (west of the
building): [[13.731239, 100.787824], [13.731276, 100.788463], [13.730717, 100.788463], [13.730723, 100.78784]]. Sweep at 20 m: 3 legs / 216 m /
87 s, marker 16.9 px, pad 42 px, camera footprint from the legs' east
ends reaches E 122.6; at 12 m it would be 5 legs / 344 m /
142 s with the marker at 28.2 px.

Time budget of a full 4-egg flight on this plan: 894 s; egg 4's gate
runs with 516 s left (floor 300).

Images: `briefing_corridor_2026-08-28_overview.jpg` (whole airspace: orange =
new corridor, grey = PDF corridor, green = sweep legs, red = no-fly bands),
`briefing_corridor_2026-08-28_zoom.jpg` (the slide's sketch over the Esri
imagery, 4x: green = sketch, yellow = PDF P1-P2-P3, orange = the slide's own
drawn route).

## Update, 28-Aug noon — the operator's own layout supersedes the slide georeference

After seeing the plan the operator drew the corridor and the no-fly areas
himself on the 20 m-gridded Esri map (`~/Desktop/KMITL_map_rules_geometry.jpg`
→ `KMITL_map_rules_geometrygooo.jpg`), which reads back with no fit at all
(the grid IS the georeference; ±3 m from the stroke width):

* corridor = a due-north green line at **E 123.7 m** → P2′ 13.730390,
  100.788590 (ENU 123.7 / 7.6, on the P1-P2 line) · P3′ 13.730715, 100.788590
  (ENU 123.7 / 43.8, on the search-area south edge). Replaces the slide-derived
  P2′/P3′ above (121.2/7.5 · 118.7/43.9 — 2-5 m west of his line).
* no-fly **building band** E 125.6-266 / N 42-75.3 (the building AND the
  courtyard east of it, down to the search-area south edge) and **east band**
  E 219.8-266 / N 42-116. These replace the IMG_0550 boxes above for planning.

Consequences (all in `sitl/kmitl_config.yaml`, tests in
`tests/test_keepout_routing.py`): the flown search area is the L left over
(west field + the strip north of the building, N ≥ 75.3); the sweep is laid
by hand (`search.sweep_waypoints_enu`: C (112,58)→(46,58), A (46,88)→(205,88),
B (205,107)→(46,100); 433 m, ~159 s; 2 m-grid coverage check: only a few
metres in the corner against the corridor gate are uncovered); every goto is
routed around the bands through the gateway ENU (115, 83) = 13.731068,
100.788509 (`routing:`); candidates and pads inside a band (+3 m) are refused.
Image of the plan: `~/Desktop/KMITL_flight_path_L_2026-08-28.jpg`.

### 4 legs, not 3 (operator, right after seeing the plan)

The three-leg cut had leg 1 at N 58 and leg 2 at N 88 — 30 m apart, exactly
one 20 m swath, zero overlap — because the strip leg had to clear the band's
top at N 75.3. Grid check (2 m, the whole L, points within 3 m of a band
excluded): unseen 0.7 % at the full 15.1 m half-swath, **5.9 % at a realistic
13 m** (a pad must sit wholly in the frame), **15.3 % at 11 m** (GPS bias +
tilt) — all along the N ≈ 73 seam. Four legs — 1 (112,52)→(46,52),
2 (46,68)→(112,68), 3 (46,86)→(205,86), 4 (205,107)→(46,100); spacings
16 / 18 / 21 m — bring that to 0 % / 0.6 % / 2.6 %; cost ~50 s of sweep
(556 m incl. transitions, ~206 s). Pinned by
`tests/test_keepout_routing.py::test_the_kmitl_sweep_covers_the_L_with_overlap`.
