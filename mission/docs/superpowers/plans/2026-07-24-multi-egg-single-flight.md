# Multi-egg single flight (FLIGHT ⊃ DELIVERY) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Carry all four eggs in one flight and serve every assigned pad in sequence before returning, controlled by a single `eggs_aboard` config knob, with a per-delivery mid-flight abort gate and a 6-pad SITL field whose payload boxes actually detach on release.

**Architecture:** Split the "sortie" concept into FLIGHT (one arm→disarm cycle: gate, transit, land+disarm) and DELIVERY (one pad: align, land-ON, release). The mission loop becomes `for flight: for delivery in flight`. `eggs_aboard=N` chunks the assigned-id queue into flights of ≤N deliveries; `eggs_aboard=1` collapses the inner loop to one iteration and reproduces today's behaviour. There is ONE code path — no `if multi_egg:` branch.

**Tech Stack:** Python 3.12 (async orchestrator, MAVSDK), pydantic, loguru, pytest; Gazebo Harmonic SDF + gz-transport (system python3) for SITL; PX4 v1.17.

## Global Constraints

- **Classical/deterministic only** — no LLM, no torch, no new flight-core deps. Flight core stays as-is except where a task explicitly modifies it.
- **`eggs_aboard=1` ⇒ today's behaviour, STRUCTURALLY** — exactly one delivery per flight, one flight per assigned id, identical plan geometry, identical find-before-serve logic (pre-sweep top-up → sweep → decode visits → opportunistic registry completion). This is the rollback path and a regression gate. It is NOT byte-identical audit text: the audit grammar changes to FLIGHT/DELIVERY for ALL `eggs_aboard` values (below). The regression assertions check flight/delivery counts and plan structure, not verbatim audit lines.
- **The audit grammar changes for everyone** (SORTIE→FLIGHT/DELIVERY, `sortie=`→`flight=`). The writers (`orchestrator/mission.py`, `orchestrator/tactical_align.py`) and the sole reader (`tools/verify_flight.py`) change together. `test_realtime.py`'s literal `sortie=`/`SORTIE` audit strings are inputs to an anomaly-feed test that does not parse them — leave them or modernize them, they do not gate behaviour.
- **"sortie" == "flight"** internally: `state.sortie_index` is kept as the FLIGHT counter (one arm→disarm cycle) to limit churn across the 10 dashboard/test files that read it. New fields carry the delivery-level state. The operator-facing audit grammar uses the words FLIGHT and DELIVERY.
- **Release channels: AUX 9/10/11/12** — `drop_servo_channel=9`, `drop_payload_count=4`; `payload_id` (0..N-1, per-flight) indexes the channel, `stop_index` (queue position, mission-global) keys the idempotence ledger.
- **Headless must keep working** (`--no-dashboard --assigned-ids …`) and degrade gracefully if the dashboard seam or the SITL detach bridge is absent.
- **Tests run under `env -u PYTHONPATH`** (mirror `make test`). Touch real modules, no mocks of the unit under test.
- **SITL invariant:** the aircraft spawns at world origin `(0,0)` (`PX4_GZ_MODEL_POSE="0,0,0.35,0,0,0"`); detachable boxes are pre-placed relative to that fixed pose. `cargo_box` (static ground dressing) is NOT modified — a new dynamic `cargo_payload` model is added.

---

### Task 1: `chunk_flights` + `eggs_aboard` in profile & config

**Files:**
- Create: `mission_brain/flights.py`
- Modify: `mission_brain/profile.py` (add ONE field `eggs_aboard` to `MissionProfile`, `COMPETITION`, `PRODUCTION`)
- Modify: `tests/test_config.py` (add the new field to the locked assertions)
- Test: `tests/test_flights.py`

**Interfaces:**
- Produces: `chunk_flights(ids: list[int], eggs_aboard: int) -> list[list[int]]` — splits `ids` into consecutive chunks of ≤`eggs_aboard`, order preserved. `eggs_aboard<1` is treated as 1. `max_flights_for(n_ids, eggs_aboard) -> int`.
- Produces: `MissionProfile.eggs_aboard: int` (default 1). NOTE: `max_deliveries` is NOT a profile field — it lives on `OrchestratorState` (Task 4), seeded in `main.py` from `mission.max_deliveries` config or `profile.max_sorties` (which has always meant "≤4 pads" = deliveries). This keeps `test_config`'s `profile.max_sorties == 4` lock valid and avoids a dangling profile field.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_flights.py
from mission_brain.flights import chunk_flights, max_flights_for


def test_eggs_aboard_1_is_one_delivery_per_flight():
    assert chunk_flights([3, 1, 4, 6], 1) == [[3], [1], [4], [6]]


def test_eggs_aboard_4_is_a_single_flight():
    assert chunk_flights([3, 1, 4, 6], 4) == [[3, 1, 4, 6]]


def test_eggs_aboard_2_pairs_in_order():
    assert chunk_flights([3, 1, 4, 6], 2) == [[3, 1], [4, 6]]


def test_ragged_last_chunk():
    assert chunk_flights([3, 1, 4], 2) == [[3, 1], [4]]


def test_empty_queue_is_no_flights():
    assert chunk_flights([], 4) == []


def test_eggs_aboard_below_one_is_treated_as_one():
    assert chunk_flights([3, 1], 0) == [[3], [1]]


def test_max_flights_for():
    assert max_flights_for(4, 1) == 4
    assert max_flights_for(4, 4) == 1
    assert max_flights_for(4, 2) == 2
    assert max_flights_for(3, 2) == 2
    assert max_flights_for(0, 4) == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_flights.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'mission_brain.flights'`

- [ ] **Step 3: Write minimal implementation**

```python
# mission_brain/flights.py
"""Split the committee's assigned-id queue into flights.

A FLIGHT is one arm→disarm cycle carrying up to ``eggs_aboard`` eggs; a
DELIVERY is one pad served within a flight. ``eggs_aboard=1`` yields one
delivery per flight — the original per-sortie behaviour. Pure: no I/O.
"""

from __future__ import annotations

import math


def chunk_flights(ids: list[int], eggs_aboard: int) -> list[list[int]]:
    """Consecutive chunks of ``ids`` of at most ``eggs_aboard`` (order kept).
    ``eggs_aboard < 1`` is treated as 1 (one delivery per flight)."""
    n = max(1, int(eggs_aboard))
    return [ids[i:i + n] for i in range(0, len(ids), n)]


def max_flights_for(n_ids: int, eggs_aboard: int) -> int:
    """Number of flights ``n_ids`` deliveries need at ``eggs_aboard`` per flight."""
    n = max(1, int(eggs_aboard))
    return math.ceil(n_ids / n) if n_ids > 0 else 0
```

- [ ] **Step 4: Add the profile field**

In `mission_brain/profile.py`, inside `class MissionProfile`, after the `max_sorties` field (line ~50) add:

```python
    # ── V1.3+briefing: eggs carried per FLIGHT (arm→disarm). 1 = the original
    # one-egg-per-sortie model; 4 = the briefing's carry-all-in-one-flight. ──
    eggs_aboard: int = Field(1, ge=1)
```

In `COMPETITION` add `eggs_aboard=4,` (the briefing model is the competition default). In `PRODUCTION` add `eggs_aboard=1,`.

- [ ] **Step 5: Lock the new field in test_config**

`tests/test_config.py::test_competition_profile_is_locked` pins COMPETITION fields. Add one assertion beside the existing `max_sorties` one:

```python
    assert p.eggs_aboard == 4                    # briefing: carry all 4 in one flight
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_flights.py tests/test_config.py -q`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add mission_brain/flights.py mission_brain/profile.py tests/test_flights.py tests/test_config.py
git commit -m "feat(flights): chunk the assigned-id queue into flights (eggs_aboard)"
```

---

### Task 2: `ServedStop.payload_id` + live plan renders one GOTO+DROP per delivery

**Files:**
- Modify: `mission_brain/live_plan.py:47-56` (add `payload_id` to `ServedStop`), `:114-127` (use `d.payload_id`)
- Test: `tests/test_live_plan.py`

**Interfaces:**
- Consumes: `ServedStop(stop_index, lat, lon, name)` from Task 0 (existing).
- Produces: `ServedStop(stop_index, lat, lon, name="", payload_id=0)` — new field, default 0 (back-compat). `render_live_plan` emits `DROP_PAYLOAD` commands with `payload_id=d.payload_id` (was hard-coded `0`).

- [ ] **Step 1: Write the failing test**

The file already has `_spec()` (calls `build_search_pattern(SEARCH_AREA, HOME, sweep_alt_m=12.0)`), `HOME`, and imports `render_live_plan`, `ServedStop`, `CommandKind`, `Coordinate`. Reuse them — add only the new test:

```python
# add to tests/test_live_plan.py (uses the file's existing _spec(), HOME, load_profile)
def test_multi_delivery_flight_renders_distinct_payload_and_stop_ids():
    prof = load_profile("competition")
    discovered = [
        ServedStop(stop_index=0, lat=13.7307, lon=100.7880, name="PAD3", payload_id=0),
        ServedStop(stop_index=1, lat=13.7306, lon=100.7883, name="PAD1", payload_id=1),
        ServedStop(stop_index=2, lat=13.7308, lon=100.7885, name="PAD4", payload_id=2),
        ServedStop(stop_index=3, lat=13.7309, lon=100.7887, name="PAD6", payload_id=3),
    ]
    plan = render_live_plan(HOME, _spec(), discovered=discovered, profile=prof,
                            include_search=False, sortie=1)
    drops = [c for c in plan.commands if c.kind is CommandKind.DROP_PAYLOAD]
    assert [d.payload_id for d in drops] == [0, 1, 2, 3]
    assert [d.stop_index for d in drops] == [0, 1, 2, 3]
```

> If `load_profile` isn't already imported in the file, add `from mission_brain.profile import load_profile`. Use the file's own `HOME`/`_spec()` names verbatim (confirm them at the top of the file).

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_live_plan.py -k multi_delivery -q`
Expected: FAIL — `TypeError: ServedStop.__init__() got an unexpected keyword argument 'payload_id'`

- [ ] **Step 3: Implement**

In `mission_brain/live_plan.py`, extend `ServedStop`:

```python
@dataclass(frozen=True)
class ServedStop:
    """A pad the mission has committed a delivery to (serving or served). The
    ledger of these — spanning every flight — renders the GOTO+DROP section of
    the live plan."""

    stop_index: int
    lat: float
    lon: float
    name: str = ""
    # Per-FLIGHT release-mechanism index (0..eggs_aboard-1) → servo channel
    # drop_servo_channel + payload_id. Distinct from stop_index, which is the
    # mission-global ledger key. Defaults to 0 (single-egg flights).
    payload_id: int = 0
```

In `render_live_plan`, change the `DROP_PAYLOAD` command (line ~122-127):

```python
        _add(MissionCommand(
            seq=seq, kind=CommandKind.DROP_PAYLOAD, phase=MissionPhase.DROP,
            coord=Coordinate(lat=d.lat, lon=d.lon),
            payload_id=d.payload_id, stop_index=d.stop_index, confirmed=True,
            notes=f"land ON pad {d.name or d.stop_index} + release the egg "
                  "after touchdown",
        ))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_live_plan.py -q`
Expected: PASS (all existing tests + the new one; existing tests pass because `payload_id` defaults to 0).

- [ ] **Step 5: Commit**

```bash
git add mission_brain/live_plan.py tests/test_live_plan.py
git commit -m "feat(live_plan): render one GOTO+DROP per delivery with its payload_id"
```

---

### Task 3: Thread `payload_id` through the release + DELIVERY audit grammar

**Files:**
- Modify: `orchestrator/tactical_align.py:224-234` (`acquire_and_land_drop` signature), `:505-513` (drop call), `:516-554` (`_drop_once`)
- Test: `tests/test_tactical_align.py`

**Interfaces:**
- Consumes: `commander.drop_payload(payload_id: int)` (existing, bounds-checked against `drop_payload_count`).
- Produces: `acquire_and_land_drop(..., stop_index, *, payload_id: int = 0, delivery_index: int = 0, marker_id_for_audit=..., ...)` and `_drop_once(commander, state, stop_index, *, payload_id=0, delivery_index=0, marker_id=None, on_drop_prediction=None)`. New RELEASE audit line: `DELIVERY {delivery_index} RELEASE pad={marker_id} payload={payload_id} lat=… lon=…`.

- [ ] **Step 1: Write the failing test**

```python
# add to tests/test_tactical_align.py
import asyncio
from orchestrator.tactical_align import _drop_once


def test_drop_once_uses_payload_id_and_delivery_grammar(make_state, fake_commander):
    # make_state / fake_commander: reuse this file's existing fixtures.
    state = make_state()
    cmd = fake_commander()  # records drop_payload(payload_id=...) calls
    ok = asyncio.run(_drop_once(cmd, state, stop_index=2, payload_id=2,
                                delivery_index=3, marker_id=4))
    assert ok is True
    assert cmd.dropped_payload_ids == [2]                 # channel 9+2 = 11
    line = next(a for a in state.anomalies if "DELIVERY 3 RELEASE" in a)
    assert "pad=4" in line and "payload=2" in line


def test_drop_once_is_idempotent_per_stop_index(make_state, fake_commander):
    state = make_state()
    cmd = fake_commander()
    assert asyncio.run(_drop_once(cmd, state, stop_index=0, payload_id=0,
                                  delivery_index=1, marker_id=3)) is True
    assert asyncio.run(_drop_once(cmd, state, stop_index=0, payload_id=0,
                                  delivery_index=1, marker_id=3)) is False
    assert cmd.dropped_payload_ids == [0]
```

> Reuse the file's existing state/commander fixtures. If `fake_commander` doesn't record `payload_id`, extend it minimally to append to `dropped_payload_ids` in its `drop_payload`.

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_tactical_align.py -k drop_once -q`
Expected: FAIL — `_drop_once()` got an unexpected keyword argument `payload_id` / `delivery_index`.

- [ ] **Step 3: Implement `_drop_once`**

Replace `_drop_once` (`orchestrator/tactical_align.py:516-554`):

```python
async def _drop_once(
    commander: DroneCommander,
    state: OrchestratorState,
    stop_index: int,
    *,
    payload_id: int = 0,
    delivery_index: int = 0,
    marker_id: int | None = None,
    on_drop_prediction: DropPredCb | None = None,
) -> bool:
    """Release exactly once for ``stop_index`` (guards the shared drop lock).

    ``payload_id`` selects the release mechanism for THIS flight (0..N-1 →
    servo channel drop_servo_channel + payload_id); ``stop_index`` (the id's
    position in the mission queue) keys the idempotence ledger so a retried
    serve can't double-open the same hold. ``delivery_index`` is the 1-based
    delivery number across the mission, for the audit line only."""
    async with state.drop_lock:
        if stop_index in state.dropped_stops:
            return False
        if on_drop_prediction is not None:
            try:
                t = state.telemetry
                pred = drop_trajectory.predict(
                    release_lat=t.lat, release_lon=t.lon,
                    release_alt_agl_m=max(0.0, t.relative_alt_m)
                    if not math.isnan(t.relative_alt_m) else 0.0,
                    vehicle_ground_speed_mps=t.ground_speed_mps
                    if not math.isnan(t.ground_speed_mps) else 0.0,
                    vehicle_heading_deg=t.heading_deg
                    if not math.isnan(t.heading_deg) else 0.0,
                )
                on_drop_prediction(pred)
            except Exception:
                logger.exception("[align] drop prediction failed (non-fatal)")
        await commander.drop_payload(payload_id=payload_id)
        state.dropped_stops.add(stop_index)
        state.record_audit(
            f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} RELEASE "
            f"pad={marker_id} payload={payload_id} "
            f"lat={state.telemetry.lat:.7f} lon={state.telemetry.lon:.7f}")
        return True
```

- [ ] **Step 4: Thread through `acquire_and_land_drop`**

Change the signature (`:224-234`) to add two keyword args after `stop_index`:

```python
async def acquire_and_land_drop(
    commander: DroneCommander,
    state: OrchestratorState,
    target: Coordinate,
    stop_index: int,
    *,
    payload_id: int = 0,
    delivery_index: int = 0,
    nadir_frame: Path = DEFAULT_NADIR_FRAME,
    params: AlignParams = AlignParams(),
    on_phase: PhaseCb | None = None,
    on_drop_prediction: DropPredCb | None = None,
) -> AlignResult:
```

At the drop call (`:505-509`) pass them through:

```python
    dropped = await _drop_once(
        commander, state, stop_index,
        payload_id=payload_id, delivery_index=delivery_index,
        marker_id=params.assigned_marker_id,
        on_drop_prediction=on_drop_prediction,
    )
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_tactical_align.py -q`
Expected: PASS. If other tests in the file call `acquire_and_land_drop` positionally past `stop_index`, they still pass (new args are keyword-only with defaults).

- [ ] **Step 6: Commit**

```bash
git add orchestrator/tactical_align.py tests/test_tactical_align.py
git commit -m "feat(align): release the flight's payload_id, log DELIVERY RELEASE"
```

---

### Task 4: State fields for the flight/delivery model

**Files:**
- Modify: `orchestrator/state.py:56-70` (add fields near the multi-sortie block)
- Test: `tests/test_state.py` (create if absent, else add)

**Interfaces:**
- Produces on `OrchestratorState`: `eggs_aboard: int = 1`, `max_deliveries: int = 4`, `delivery_index: int = 0` (1-based current delivery across the mission; 0 = none yet), `flight_ids: list[int] = []` (the ids of the current flight). `sortie_index` keeps its meaning (the FLIGHT counter). `max_sorties` keeps its name but now holds the number of FLIGHTS.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_state.py (add)
from orchestrator.state import OrchestratorState


def test_state_has_flight_delivery_fields(make_min_state):
    s = make_min_state()  # reuse existing factory if present; else construct minimally
    assert s.eggs_aboard == 1
    assert s.max_deliveries == 4
    assert s.delivery_index == 0
    assert s.flight_ids == []
```

> If there is no state factory, construct `OrchestratorState(mode=..., plan=..., telemetry=...)` the way `tests/test_delivery_mission.py` already does and copy that setup.

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_state.py -k flight_delivery -q`
Expected: FAIL — `AttributeError: 'OrchestratorState' object has no attribute 'eggs_aboard'`

- [ ] **Step 3: Implement**

In `orchestrator/state.py`, in the `# ── V1.3 multi-sortie delivery ──` block (after `max_sorties`, line ~58) add:

```python
    # ── FLIGHT ⊃ DELIVERY (2026-07-24 briefing) ──
    # A FLIGHT is one arm→disarm cycle; sortie_index above IS the flight
    # counter and max_sorties above IS the number of flights. eggs_aboard eggs
    # are carried per flight; the flight serves flight_ids in order.
    eggs_aboard: int = 1
    max_deliveries: int = 4                    # pads to serve in the window (≤ placed)
    delivery_index: int = 0                    # 1-based delivery across the mission
    flight_ids: list[int] = field(default_factory=list, compare=False, repr=False)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_state.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add orchestrator/state.py tests/test_state.py
git commit -m "feat(state): flight/delivery fields (eggs_aboard, delivery_index, flight_ids)"
```

---

### Task 5: Mid-flight per-delivery time gate

**Files:**
- Modify: `orchestrator/time_policy.py` (add `can_start_delivery`)
- Test: `tests/test_time_policy.py`

**Interfaces:**
- Produces: `TimePolicy.can_start_delivery(remaining_s: float) -> bool` — True if there is time for ONE more land-ON delivery AND still fly the egress+land reserve. Uses `rth_reserve_s` (already airborne and committed to the flight) rather than the full end-of-mission `reserve_s`.

- [ ] **Step 1: Write the failing test**

```python
# add to tests/test_time_policy.py
from orchestrator.time_policy import TimePolicy


def test_can_start_delivery_reserves_egress_plus_one_serve():
    p = TimePolicy(serve_cost_s=110.0, rth_reserve_s=60.0, margin_s=30.0)
    # need >= 60 + 110 + 30 = 200 s
    assert p.can_start_delivery(200.0) is True
    assert p.can_start_delivery(199.0) is False


def test_can_start_delivery_is_looser_than_can_start_serve():
    # mid-flight uses rth_reserve, not the full end-of-mission reserve_s
    p = TimePolicy(serve_cost_s=110.0, watchdog_floor_s=180.0,
                   rth_reserve_s=60.0, margin_s=30.0)
    r = 250.0
    assert p.can_start_delivery(r) is True
    assert p.can_start_serve(r) is False   # reserve_s=max(180,60)=180 → needs 320
```

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_time_policy.py -k can_start_delivery -q`
Expected: FAIL — `AttributeError: 'TimePolicy' object has no attribute 'can_start_delivery'`

- [ ] **Step 3: Implement**

In `orchestrator/time_policy.py`, add after `can_start_serve`:

```python
    def can_start_delivery(self, remaining_s: float) -> bool:
        """Mid-flight gate before each delivery in a multi-egg flight.

        The aircraft is already airborne and committed to THIS flight, so it
        reserves only the egress-transit + L&R-landing cost (``rth_reserve_s``),
        not the full end-of-mission ``reserve_s``. It must simply never START a
        delivery it cannot finish before it has to head home. The per-delivery
        battery guard lives alongside this in the mission loop."""
        return remaining_s >= self.rth_reserve_s + self.serve_cost_s + self.margin_s
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_time_policy.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add orchestrator/time_policy.py tests/test_time_policy.py
git commit -m "feat(time_policy): mid-flight can_start_delivery gate"
```

---

### Task 6: Mission loop — FLIGHT ⊃ DELIVERY + per-delivery abort + FLIGHT/DELIVERY audit

**Files:**
- Modify: `orchestrator/mission.py` (the loop body `:436-604`, the helpers `_fly_transit`/`_sweep_for`/`_serve`/`_telemetry_sampler`, the `SortieGate` type `:92-95`)
- Test: `tests/test_delivery_mission.py`

**Interfaces:**
- Consumes: `chunk_flights` (Task 1), `ServedStop(..., payload_id=)` (Task 2), `acquire_and_land_drop(..., payload_id=, delivery_index=)` (Task 3), state fields (Task 4), `TimePolicy.can_start_delivery` (Task 5).
- Produces: `FlightGate = Callable[[int], Awaitable[list[int] | None]]` (replaces `SortieGate`). Audit grammar: `FLIGHT n START eggs=E ids=a,b,c remaining=Rs` · `TRANSIT_PASS Pn ingress flight=n d=Dm` · `DELIVERY k START pad=P payload=Y stop_index=S` · `DELIVERY k END delivered=BOOL pad=P` · `FLIGHT n START/END`, `FLIGHT n ENERGY …`, `FLIGHT n END delivered=X/Y d_home=Dm remaining=Rs`. `TELEM … flight=n …` (token renamed from `sortie=`).

This is the largest task. Work in sub-steps, each with its own test run.

- [ ] **Step 1: Write the failing integration tests**

Add to `tests/test_delivery_mission.py` (reuse this file's existing harness — the fake `DroneCommander`, `TargetTracker` seeding, and `run_delivery_mission` driver it already uses; mirror an existing passing test's setup exactly and only change the assertions/gate):

```python
def test_single_flight_serves_all_ids_in_order(delivery_harness):
    # eggs_aboard=4 → ONE flight, four deliveries. Registry pre-seeded so no sweep.
    h = delivery_harness(eggs_aboard=4, queue=[3, 1, 4, 6], seed_registry=[3, 1, 4, 6])
    asyncio.run(h.run())
    # one arm/takeoff, one land+disarm
    assert h.commander.takeoff_count == 1
    assert h.commander.land_disarm_count == 1
    # four releases, correct channels, correct order
    assert h.commander.dropped_payload_ids == [0, 1, 2, 3]
    starts = [a for a in h.state.anomalies if "DELIVERY" in a and "START" in a]
    assert [s for s in starts].__len__() == 4
    assert "FLIGHT 1 END delivered=4/4" in "\n".join(h.state.anomalies)


def test_eggs_aboard_1_is_one_delivery_per_flight(delivery_harness):
    h = delivery_harness(eggs_aboard=1, queue=[3, 1], seed_registry=[3, 1])
    asyncio.run(h.run())
    assert h.commander.takeoff_count == 2          # two flights
    assert h.commander.land_disarm_count == 2
    assert h.commander.dropped_payload_ids == [0, 0]   # each flight reloads → payload 0
    assert "FLIGHT 1 END delivered=1/1" in "\n".join(h.state.anomalies)
    assert "FLIGHT 2 END delivered=1/1" in "\n".join(h.state.anomalies)


def test_partial_find_serves_confirmed_skips_missing(delivery_harness):
    # 4 assigned, only 3 confirmable in the registry
    h = delivery_harness(eggs_aboard=4, queue=[3, 1, 4, 6], seed_registry=[3, 1, 4])
    asyncio.run(h.run())
    assert h.commander.dropped_payload_ids == [0, 1, 2]     # pad 6 skipped
    joined = "\n".join(h.state.anomalies)
    assert "FLIGHT 1 END delivered=3/4" in joined


def test_per_delivery_abort_on_low_time(delivery_harness):
    # window nearly gone after 2 deliveries → 3rd/4th aborted, aircraft returns
    h = delivery_harness(eggs_aboard=4, queue=[3, 1, 4, 6], seed_registry=[3, 1, 4, 6],
                         time_remaining_after=lambda n: 500.0 if n < 2 else 120.0)
    asyncio.run(h.run())
    assert len(h.commander.dropped_payload_ids) < 4
    assert h.commander.land_disarm_count == 1              # still returned + disarmed
```

> `delivery_harness` almost certainly does not exist yet with these kwargs. Extend the file's EXISTING fixture/helper to accept `eggs_aboard`, `queue`, `seed_registry`, and optional `time_remaining_after`. If the file drives `run_delivery_mission` inline in each test rather than via a fixture, factor that setup into a local helper first (no behaviour change), commit it, then add these tests.

- [ ] **Step 2: Run to verify they fail**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_delivery_mission.py -k "single_flight or eggs_aboard_1 or partial_find or per_delivery_abort" -q`
Expected: FAIL (gate returns int not list; no DELIVERY grammar).

- [ ] **Step 3: Change the gate type + telemetry token**

In `orchestrator/mission.py`:

Replace the `SortieGate` type alias (`:93-95`) with:

```python
PlanUpdateCb = Callable[[MissionPlan, int], None]
# Per-flight gate: awaits operator GO (or headless auto-GO) and returns the
# committee-assigned marker ids for flight i (its ≤eggs_aboard chunk of the
# queue), or None to end the mission.
FlightGate = Callable[[int], Awaitable[list[int] | None]]
```

Update the import in `orchestrator/main.py` and `run_delivery_mission`'s parameter name `sortie_gate: SortieGate` → `flight_gate: FlightGate` (keep the keyword name change consistent; update `main.py`'s call site in Task 7).

Add near the other module constants (`:71`):

```python
# Mid-flight per-delivery battery guard: don't START a new descent if the pack
# is already at/near the FC's low-battery RTL threshold — the FC failsafe would
# otherwise fire mid-delivery with the egg aboard. Percentage, above rth_battery_pct.
_DELIVERY_BATT_MARGIN_PCT = 5.0
```

In `_telemetry_sampler` change the TELEM line token `sortie={state.sortie_index}` → `flight={state.sortie_index}`.

- [ ] **Step 4: Rework `_fly_transit` and `_sweep_for` signatures for the flight/find-all model**

`_fly_transit(sortie, *, egress)` → keep the parameter but rename to `flight` and change its audit token:

```python
    async def _fly_transit(flight: int, *, egress: bool) -> None:
        ...
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s {tag} P{n} "
                f"{'egress' if egress else 'ingress'} flight={flight} d={d:.1f}m")
```

`_sweep_for` early-stops when the whole FLIGHT is satisfied, not only when `max_pads` distinct ids are confirmed. Change its signature to take the flight's id list:

```python
    async def _sweep_for(flight: int, wanted: list[int]) -> None:
        """Full boustrophedon sweep (finish-sweep-then-serve): discovery only,
        feeding every decoded pad into the cross-sortie registry. Early-stop
        once every ``wanted`` id is confirmed OR max_pads distinct ids are."""
        def _done() -> bool:
            return (all(tracker.confirmed_by_marker(a) is not None for a in wanted)
                    or len(tracker.distinct_confirmed_ids()) >= max_pads)
        for orig_i, wp in enumerate(spec.waypoints):
            if not _running() or _done():
                return
            _phase(MissionPhase.SEARCH)
            state.command_pointer = pointer_for(state.plan, wp_index=orig_i)
            await commander.goto(wp.lat, wp.lon, sweep_alt)
            cur = _cur_latlon()
            leg_len = _latlon_dist_m(cur[0], cur[1], wp.lat, wp.lon) if cur else 0.0
            leg_timeout = 2.0 * leg_len / max(spec.speed_mps, 0.1) + _WAIT_PAD_S
            t_leg = time.monotonic()
            while _running():
                _drain_tracker()
                if _done():
                    return
                cur = _cur_latlon()
                if cur is not None and _latlon_dist_m(
                        cur[0], cur[1], wp.lat, wp.lon) <= _ARRIVAL_RADIUS_M:
                    break
                if (time.monotonic() - t_leg) > leg_timeout:
                    state.record_anomaly(f"sweep_leg_timeout_wp{orig_i}")
                    break
                await asyncio.sleep(_LOOKOUT_POLL_S)
```

`_decode_visits(sortie, assigned, …)` → rename param `sortie`→`flight`; its logic is unchanged (it already accepts a single `assigned` or `-1`). Keep it; callers pass a single id to hunt or `-1`.

- [ ] **Step 5: Rework `_serve` to take stop_index + payload_id + delivery_index**

```python
    async def _serve(flight: int, assigned: int, *, stop_index: int,
                     payload_id: int, delivery_index: int) -> bool:
        """Land ON the assigned pad + release (with ONE retry if time allows).
        Leaves the aircraft landed on the pad; the caller climbs out."""
        for attempt in (1, 2):
            if not _running():
                return False
            if tracker.confirmed_by_marker(assigned) is None:
                return False
            claimed = tracker.claim_by_marker(assigned)
            _drain_tracker()
            if claimed is None:
                return False
            ledger = ServedStop(stop_index=stop_index, lat=claimed.lat,
                                lon=claimed.lon, name=f"PAD{assigned}",
                                payload_id=payload_id)
            discovered.append(ledger)
            state.command_pointer = pointer_for(
                state.plan, stop_index=stop_index, kind="goto")
            _rebuild_plan(flight)
            _phase(MissionPhase.SEARCH)
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} START "
                f"pad={assigned} payload={payload_id} stop_index={stop_index}")
            logger.info(
                f"[mission] flight {flight} delivery {delivery_index}: serving pad "
                f"{assigned} ({claimed.lat:.7f},{claimed.lon:.7f}) attempt={attempt}")
            cur0 = _cur_latlon()
            d0 = (_latlon_dist_m(cur0[0], cur0[1], claimed.lat, claimed.lon)
                  if cur0 else 200.0)
            await commander.goto(claimed.lat, claimed.lon, sweep_alt)
            await _wait_arrival(
                (claimed.lat, claimed.lon),
                timeout_s=2.0 * d0 / max(spec.speed_mps, 0.1) + _WAIT_PAD_S)
            serve_params = replace(
                align_p, accept_radius_m=_SERVE_ACCEPT_RADIUS_M,
                assigned_marker_id=assigned)
            res = await acquire_and_land_drop(
                commander, state, Coordinate(lat=claimed.lat, lon=claimed.lon),
                stop_index=stop_index, payload_id=payload_id,
                delivery_index=delivery_index, params=serve_params,
                on_phase=on_phase, on_drop_prediction=on_drop_prediction)
            if res.dropped:
                tracker.mark_served(claimed.target_id)
                _drain_tracker()
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} END "
                    f"delivered=True pad={assigned} err={res.final_error_m:.2f}m "
                    f"landed={res.landed}")
                return True
            discovered.pop()
            tracker.defer(claimed.target_id)
            _drain_tracker()
            _rebuild_plan(flight)
            if attempt == 1 and pol.can_start_serve(state.time_remaining_s()):
                logger.warning(f"[mission] flight {flight} delivery {delivery_index}: "
                               f"deferred ({'; '.join(res.notes)}) — one retry")
                continue
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_index} END "
                f"delivered=False pad={assigned} notes={'; '.join(res.notes)}")
            return False
        return False
```

- [ ] **Step 6: Rewrite the outer loop body**

Replace the `for sortie in range(1, max_sorties + 1):` block (`:437-604`) with the flight loop. `max_sorties` now holds the flight count (Task 7 sets it); `delivery_no` counts deliveries across the whole mission.

```python
    # state.max_sorties IS the flight count (main.py seeds it via max_flights_for;
    # the test harness sets it directly). It is authoritative — do NOT fall back
    # to prof.max_sorties, which counts deliveries (≤4 pads), not flights.
    max_flights = max(1, state.max_sorties)
    logger.info(
        f"[mission] delivery mission: ≤{max_flights} flights "
        f"(eggs_aboard={state.eggs_aboard}), transit {len(transit_route)} pts @ "
        f"{transit_alt:.0f} m, sweep {spec.leg_count} legs @ {sweep_alt:.0f} m, "
        f"window {state.operation_window_s:.0f} s")
    sampler = asyncio.create_task(_telemetry_sampler())
    delivery_no = 0
    try:
        for flight in range(1, max_flights + 1):
            if not _running():
                break
            flight_ids = await flight_gate(flight)
            if not flight_ids:
                logger.info(f"[mission] no assignment for flight {flight} — ends")
                break
            state.start_window()
            state.sortie_index = flight
            state.flight_ids = list(flight_ids)
            _entry_mah, _ = energy_consumed_mah(
                state.telemetry, state.energy_capacity_mah)
            _entry_pct = state.telemetry.battery_percent
            if flight > 1 and detect_battery_swap(
                    state.energy_exit_mah, _entry_mah,
                    state.energy_exit_pct, _entry_pct):
                state.energy_baseline_mah = baseline_for_pack(
                    _entry_mah, _entry_pct, state.energy_capacity_mah)
                state.record_audit(
                    f"t={state.time_elapsed_s():.1f}s BATTERY SWAP before flight "
                    f"{flight} baseline={state.energy_baseline_mah:.0f}mAh")
            state.assigned_marker_id = flight_ids[0]
            missing = [a for a in flight_ids
                       if tracker.confirmed_by_marker(a) is None]
            include_search = bool(missing)
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} START "
                f"eggs={len(flight_ids)} ids={','.join(map(str, flight_ids))} "
                f"remaining={state.time_remaining_s():.0f}s")
            _rebuild_plan(flight)

            _phase(MissionPhase.TAKEOFF)
            await commander.arm_and_takeoff(climb_alt)
            await _fly_transit(flight, egress=False)

            # Find every missing id before serving (finish-sweep-then-serve).
            # Mirrors the original per-sortie logic, generalised over the flight's
            # ids so eggs_aboard=1 is behaviourally identical (top-up → sweep →
            # decode visits → opportunistic registry completion).
            if _running() and missing:
                for a in missing:
                    if tracker.identified_unconfirmed(a):
                        state.record_audit(
                            f"t={state.time_elapsed_s():.1f}s registry top-up: "
                            f"pad={a} identified-unconfirmed — decode visit "
                            f"before sweep")
                        await _decode_visits(flight, a, identified_only=True)
                if any(tracker.confirmed_by_marker(a) is None for a in flight_ids):
                    await _sweep_for(flight, flight_ids)
                for a in flight_ids:
                    if _running() and tracker.confirmed_by_marker(a) is None:
                        await _decode_visits(flight, a)
                # Opportunistic registry completion: decode leftover candidates
                # now (cheap) rather than paying a whole re-sweep on a later flight.
                if (_running() and (tracker.unidentified_candidates()
                                    or tracker.identified_unconfirmed())
                        and len(tracker.distinct_confirmed_ids()) < max_pads
                        and pol.can_start_sortie(state.time_remaining_s())):
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s registry completion: "
                        f"decoding leftover candidates")
                    await _decode_visits(flight, assigned=-1)

            # Serve each confirmed id in queue order; abort-budget between each.
            n_delivered = 0
            for slot, assigned in enumerate(flight_ids):
                if not _running():
                    break
                pct = state.telemetry.battery_percent
                batt_ok = (math.isnan(pct)
                           or pct > prof.rth_battery_pct + _DELIVERY_BATT_MARGIN_PCT)
                if not (pol.can_start_delivery(state.time_remaining_s()) and batt_ok):
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s DELIVERY abort: flight "
                        f"{flight} skipping remaining ids {flight_ids[slot:]} "
                        f"(remaining={state.time_remaining_s():.0f}s "
                        f"batt={pct:.0f}%) — returning with the egg(s)")
                    break
                if tracker.confirmed_by_marker(assigned) is None:
                    delivery_no += 1
                    state.delivery_index = delivery_no
                    state.record_anomaly(f"flight{flight}_pad{assigned}_not_found")
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s DELIVERY {delivery_no} END "
                        f"delivered=False pad={assigned} reason=not_found")
                    continue
                delivery_no += 1
                state.delivery_index = delivery_no
                stop_index = (state.assigned_id_queue.index(assigned)
                              if assigned in state.assigned_id_queue else delivery_no - 1)
                if await _serve(flight, assigned, stop_index=stop_index,
                                payload_id=slot, delivery_index=delivery_no):
                    n_delivered += 1
                    delivered += 1

            if not _running():
                break
            # Climb out, egress, land + DISARM (unchanged geometry).
            t = state.telemetry
            if (not t.is_armed) or (not math.isnan(t.relative_alt_m)
                                    and t.relative_alt_m < 2.0):
                await commander.arm_and_takeoff(climb_alt)
            else:
                await commander.goto(t.lat, t.lon, climb_alt)
            await _fly_transit(flight, egress=True)
            if not _running():
                break

            _phase(MissionPhase.LAND)
            state.command_pointer = len(state.plan.commands) - 1
            cur = _cur_latlon()
            dist = _latlon_dist_m(cur[0], cur[1], home.lat, home.lon) if cur else 300.0
            await commander.goto(home.lat, home.lon, transit_alt)
            home_timeout = 2.0 * dist / max(spec.speed_mps, 0.1) + _WAIT_PAD_S + 10.0
            await _wait_arrival((home.lat, home.lon), timeout_s=home_timeout)
            cur = _cur_latlon()
            d_home = (_latlon_dist_m(cur[0], cur[1], home.lat, home.lon)
                      if cur else float("nan"))
            await _set_descent_speed(_LAND_STAGE_MPS)
            stage_alt = min(_LAND_STAGE_ALT_M, transit_alt)
            await commander.goto(home.lat, home.lon, stage_alt)
            drop_m = max(0.0, transit_alt - stage_alt)
            staged = await _wait_descent(
                stage_alt, timeout_s=drop_m / _LAND_STAGE_MIN_MPS + _WAIT_PAD_S)
            if not staged:
                logger.warning(
                    f"[mission] flight {flight}: staged descent timed out at "
                    f"{state.telemetry.relative_alt_m:.1f} m — landing anyway")
            await commander.land(disarm=True)
            await _set_descent_speed(_PAD_DESCENT_MPS)
            state.record_audit(
                f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} END "
                f"delivered={n_delivered}/{len(flight_ids)} d_home={d_home:.1f}m "
                f"remaining={state.time_remaining_s():.0f}s")
            state.sortie_time_ok = (
                pol.can_start_known_sortie(state.time_remaining_s())
                if tracker.distinct_confirmed_ids()
                else pol.can_start_sortie(state.time_remaining_s()))
            _exit_mah, _ = energy_consumed_mah(
                state.telemetry, state.energy_capacity_mah)
            _exit_pct = state.telemetry.battery_percent
            if not math.isnan(_exit_mah) and not math.isnan(_entry_mah):
                _cost = _exit_mah - _entry_mah
                if _cost > 0:
                    state.sortie_energy_mah.append(_cost)
                    state.record_audit(
                        f"t={state.time_elapsed_s():.1f}s FLIGHT {flight} ENERGY "
                        f"{_cost:.0f}mAh total={_exit_mah:.0f}mAh")
            state.energy_exit_mah = _exit_mah
            state.energy_exit_pct = _exit_pct
    finally:
        sampler.cancel()
        await asyncio.gather(sampler, return_exceptions=True)

    if _running():
        state.set_terminal(TerminalState.COMPLETED, MissionPhase.LAND)
        logger.info(f"[mission] complete: {delivered} delivered over "
                    f"{state.sortie_index} flights → completed")
    else:
        logger.warning(f"[mission] ended under watchdog control: {state.terminal.value}")
```

Delete the now-unused `_rebuild_plan(sortie)` local naming mismatch by renaming its param to `flight` (cosmetic). Remove the old single-`served`/`_serve(sortie, assigned)` remnants.

- [ ] **Step 7: Run the integration tests**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_delivery_mission.py -q`
Expected: PASS for the four new tests and any pre-existing ones (update pre-existing tests that asserted the old `SORTIE`/`sortie=` grammar to the new `FLIGHT`/`DELIVERY` grammar — this is expected churn, do it here).

- [ ] **Step 8: Commit**

```bash
git add orchestrator/mission.py tests/test_delivery_mission.py
git commit -m "feat(mission): FLIGHT ⊃ DELIVERY loop, per-delivery abort, new audit grammar"
```

---

### Task 7: Gate factory returns a flight's id chunk + dashboard GO per flight

**Files:**
- Modify: `orchestrator/main.py:183-320` (`_sortie_gate_factory` → returns `list[int] | None`), `:420-435` (seed `eggs_aboard`, `max_deliveries`, `state.max_sorties = max_flights`), `:685-695` (call site keyword `flight_gate=`)
- Modify: `dashboard/commands.py` / `dashboard/routes` GO endpoint (resolve the flight's chunk), `dashboard/web/src/lib/types.ts` + a status widget (delivery chip)
- Test: `tests/test_dashboard_commands.py`, a headless gate unit test in `tests/test_main_gate.py` (create)

**Interfaces:**
- Consumes: `chunk_flights`, `max_flights_for` (Task 1); state fields (Task 4).
- Produces: gate `Callable[[int], Awaitable[list[int] | None]]` returning `chunk_flights(state.assigned_id_queue, state.eggs_aboard)[flight-1]` (interactive: after the operator GO; headless: auto), or None when that chunk index is out of range / window refuses. `state.max_sorties` seeded to `max_flights_for(len(queue) or max_deliveries, eggs_aboard)`.

- [ ] **Step 1: Write the failing headless-gate test**

```python
# tests/test_main_gate.py
import asyncio
from orchestrator.main import _sortie_gate_factory   # keep the name or rename per Step 3


def test_headless_gate_returns_flight_chunks(gate_harness):
    # gate_harness: minimal state+tracker+policy per the factory's kwargs.
    g = gate_harness(queue=[3, 1, 4, 6], eggs_aboard=4, skip_preflight=True)
    assert asyncio.run(g(1)) == [3, 1, 4, 6]
    assert asyncio.run(g(2)) is None            # only one flight


def test_headless_gate_eggs_aboard_1(gate_harness):
    g = gate_harness(queue=[3, 1], eggs_aboard=1, skip_preflight=True)
    assert asyncio.run(g(1)) == [3]
    assert asyncio.run(g(2)) == [1]
    assert asyncio.run(g(3)) is None
```

> Build `gate_harness` from the same objects `orchestrator.main` passes into `_sortie_gate_factory` (state, tracker, profile, policy, cfg). Set `state.eggs_aboard` and `state.assigned_id_queue`.

- [ ] **Step 2: Run to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_main_gate.py -q`
Expected: FAIL (gate returns int, not list).

- [ ] **Step 3: Implement the gate chunking**

In `orchestrator/main.py`, `_sortie_gate_factory._gate`: resolve the flight's chunk. Replace the queue reads:

```python
        from mission_brain.flights import chunk_flights
        flights = chunk_flights(state.assigned_id_queue, state.eggs_aboard)
        chunk = flights[sortie - 1] if 1 <= sortie <= len(flights) else None
```

Headless branch: `return chunk` (was `return nxt`); the time/energy refusals stay but gate on the chunk's first id's known/unknown status (use `all(...known...)` for the whole chunk):

```python
        if chunk is None:
            return None
        known = all(tracker.confirmed_by_marker(a) is not None for a in chunk)
        ok = (policy.can_start_known_sortie(state.time_remaining_s()) if known
              else policy.can_start_sortie(state.time_remaining_s()))
        ...
        return chunk
```

Interactive branch: after the operator GO event fires, `state.flight_ids`/return `chunk`. The GO endpoint's manual-id override applies only when `state.eggs_aboard == 1` (documented): if a manual id is set and eggs_aboard==1, return `[state.assigned_marker_id]`; else return `chunk`:

```python
        # interactive, after preflight_resume_event:
        if state.eggs_aboard == 1 and state.assigned_marker_id is not None:
            return [state.assigned_marker_id]
        return chunk
```

Rename the factory `_sortie_gate_factory` and its returned callable's usage to `flight` semantics where touched (keep the public function name to limit churn, or rename to `_flight_gate_factory` and update the one call site). Update the type import to `FlightGate`.

- [ ] **Step 4: Seed eggs_aboard + max flights at startup**

In `orchestrator/main.py` where the queue is seeded (`:420-435`), add:

```python
    state.eggs_aboard = int(mc.get("eggs_aboard", profile.eggs_aboard))
    # profile.max_sorties has always meant "≤4 pads" = max deliveries.
    state.max_deliveries = int(mc.get("max_deliveries", profile.max_sorties))
    from mission_brain.flights import max_flights_for
    n_ids = len(state.assigned_id_queue) or state.max_deliveries
    state.max_sorties = max(1, max_flights_for(n_ids, state.eggs_aboard))  # = flights
```

Update the `run_delivery_mission(...)` call (`:685-695`) keyword `sortie_gate=` → `flight_gate=`.

- [ ] **Step 5: Dashboard GO endpoint + mission-ids validation + delivery chip**

In the GO route handler (`dashboard/commands.py` / `dashboard/server` where `/api/cmd/preflight/go` sets `assigned_marker_id`): the body still carries an optional manual `assigned_marker_id`; the resolution above uses it only at eggs_aboard==1. No signature change needed if it already writes `state.assigned_marker_id` then sets `preflight_resume_event`.

**Fix the mission-ids queue-length validation** (`dashboard/commands.py:475-479`). It currently rejects a queue longer than `state.max_sorties` — but `max_sorties` now counts FLIGHTS (1 for eggs_aboard=4), so a valid 4-id queue would be refused. Validate against deliveries instead:

```python
        if len(req.ids) > state.max_deliveries:
            raise HTTPException(
                status_code=422,
                detail=f"queue of {len(req.ids)} exceeds max_deliveries="
                       f"{state.max_deliveries}",
            )
```

The readiness payload at `commands.py:454` may also expose `max_sorties`; add `max_deliveries` and `eggs_aboard` beside it. Update any `test_dashboard_commands.py` assertion that pinned the old `max_sorties` rejection message.

Add `delivery_index`, `flight_ids`, `eggs_aboard` to the telemetry frame / preflight payload:
- `dashboard/realtime.py` (or wherever `TelemetryFrame` is assembled): include `state.delivery_index`, `state.flight_ids`, `state.eggs_aboard`, `state.max_deliveries`.
- `dashboard/web/src/lib/types.ts`: add `delivery_index: number; flight_ids: number[]; eggs_aboard: number; max_deliveries: number;`.
- `dashboard/web/src/widgets/MissionStatus.svelte`: render `flight {sortie_index}/{max_sorties} · delivery {delivery_index}/{max_deliveries}` (guard for 0).

- [ ] **Step 6: Run tests + build web**

Run:
```
env -u PYTHONPATH .venv/bin/pytest tests/test_main_gate.py tests/test_dashboard_commands.py -q
make web-build
```
Expected: pytest PASS; web build succeeds. Update `test_dashboard_commands.py` assertions that referenced the single-id GO if they break.

- [ ] **Step 7: Commit**

```bash
git add orchestrator/main.py dashboard/ tests/test_main_gate.py tests/test_dashboard_commands.py
git commit -m "feat(gate,dashboard): GO resolves a flight's id chunk; delivery chip"
```

---

### Task 8: `verify_flight.py` — FLIGHT/DELIVERY grammar

**Files:**
- Modify: `tools/verify_flight.py:52-80` (regexes), `:169-176` (anchor), `:324-426` (transit/terminal/L&R/timing sections)
- Modify: `tests/test_verify_flight.py` — **this file EXISTS and its current tests build SORTIE/`sortie=`-grammar audit fixtures.** Migrate every existing fixture and assertion to the FLIGHT/DELIVERY grammar (the two new tests below plus the file's existing scenarios — off-pad, ceiling breach, L&R, disarm, NaN). This is expected churn; do not leave a mix of grammars.

**Interfaces:**
- Consumes: the audit grammar produced in Tasks 3 & 6.
- Produces: a verifier that parses `FLIGHT n START/END`, `DELIVERY k START/END`, `DELIVERY k RELEASE pad= payload= lat= lon=`, `TRANSIT_* Pn dir flight=n`, `TELEM … flight=n …`; matches each `DELIVERY k END delivered=True` to its `DELIVERY k RELEASE` by the **k in the line** (no `stop_index == sortie-1` inference); checks transit per FLIGHT; every FLIGHT ends near L&R and disarmed.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_verify_flight.py (add)
from tools.verify_flight import verify

_TRUTH = [{"marker_id": 3, "lat": 13.7307, "lon": 100.7880},
          {"marker_id": 1, "lat": 13.7306, "lon": 100.7883}]
_CFG = {"mission": {"altitude_ceiling_m": 20, "transit_alt_m": 20, "search_floor_m": 10,
                    "landing_accuracy_threshold_m": 0.5},
        "ground_operation": {"launch_recovery": [13.73025, 100.7873],
                             "launch_recovery_zone_radius_m": 25.0},
        "controlled_airspace": []}


def _one_flight_two_deliveries():
    # minimal happy-path audit lines for a single flight, two deliveries
    lines = ["t=1.0s FLIGHT 1 START eggs=2 ids=3,1 remaining=1200s"]
    for n, d in [("1", "ingress"), ("2", "ingress"), ("3", "ingress"),
                 ("3", "egress"), ("2", "egress"), ("1", "egress")]:
        lines.append(f"t=2.0s TRANSIT_PASS P{n} {d} flight=1 d=1.0m")
    # two deliveries with on-ground telem + release lines
    lines += [
        "t=3.0s TELEM phase=drop flight=1 lat=13.7307000 lon=100.7880000 alt=0.20 armed=1",
        "t=3.1s DELIVERY 1 START pad=3 payload=0 stop_index=0",
        "t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 lat=13.7307000 lon=100.7880000",
        "t=3.3s DELIVERY 1 END delivered=True pad=3 err=0.10m landed=True",
        "t=4.0s TELEM phase=drop flight=1 lat=13.7306000 lon=100.7883000 alt=0.20 armed=1",
        "t=4.1s DELIVERY 2 START pad=1 payload=1 stop_index=1",
        "t=4.2s DELIVERY 2 RELEASE pad=1 payload=1 lat=13.7306000 lon=100.7883000",
        "t=4.3s DELIVERY 2 END delivered=True pad=1 err=0.12m landed=True",
        "t=5.0s TELEM phase=land flight=1 lat=13.7302500 lon=100.7873000 alt=0.10 armed=0",
        "t=5.1s FLIGHT 1 END delivered=2/2 d_home=1.0m remaining=900s",
    ]
    return lines


def test_verify_passes_single_flight_two_deliveries():
    rep = verify(_one_flight_two_deliveries(), _TRUTH, _CFG,
                 land_acc_m=0.5, window_s=1200.0)
    assert rep.fails == [], rep.fails


def test_verify_fails_release_off_pad():
    lines = _one_flight_two_deliveries()
    lines[lines.index(
        "t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 lat=13.7307000 lon=100.7880000"
    )] = "t=3.2s DELIVERY 1 RELEASE pad=3 payload=0 lat=13.7320000 lon=100.7880000"
    rep = verify(lines, _TRUTH, _CFG, land_acc_m=0.5, window_s=1200.0)
    assert any("from truth pad 3" in f for f in rep.fails)
```

- [ ] **Step 2: Run to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_verify_flight.py -q`
Expected: FAIL (regexes still expect `SORTIE …`).

- [ ] **Step 3: Implement the new grammar**

Replace the regexes (`:52-69`):

```python
_TELEM = re.compile(
    r"t=(?P<t>[\d.]+)s TELEM phase=(?P<phase>\S+) flight=(?P<flight>\d+) "
    r"lat=(?P<lat>[-\d.nan]+) lon=(?P<lon>[-\d.nan]+) alt=(?P<alt>[-\d.nan]+) "
    r"armed=(?P<armed>[01])")
_TRANSIT = re.compile(
    r"t=(?P<t>[\d.]+)s TRANSIT_(?P<kind>PASS|MISS) P(?P<n>\d) "
    r"(?P<dir>ingress|egress) flight=(?P<flight>\d+) d=(?P<d>[-\d.nan]+)m")
_FLIGHT_START = re.compile(
    r"t=(?P<t>[\d.]+)s FLIGHT (?P<flight>\d+) START eggs=(?P<eggs>\d+) "
    r"ids=(?P<ids>[\d,]+)")
_FLIGHT_END = re.compile(
    r"t=(?P<t>[\d.]+)s FLIGHT (?P<flight>\d+) END delivered=(?P<n>\d+)/(?P<of>\d+) "
    r"d_home=(?P<d>[-\d.nan]+)m")
_DELIV_END = re.compile(
    r"t=(?P<t>[\d.]+)s DELIVERY (?P<k>\d+) END delivered=(?P<ok>True|False) "
    r"pad=(?P<pad>\d+)(?: err=(?P<err>[-\d.nan]+)m landed=(?P<landed>True|False))?")
_RELEASE = re.compile(
    r"t=(?P<t>[\d.]+)s DELIVERY (?P<k>\d+) RELEASE pad=(?P<pad>\d+) "
    r"payload=(?P<payload>\d+) lat=(?P<lat>[-\d.nan]+) lon=(?P<lon>[-\d.nan]+)")
```

Update `_FATAL_ANOMALIES`: keep the `transit_ingress_P`/`transit_egress_P`/`sweep_leg_timeout` etc. (anomaly kinds unchanged — mission still records those). Add nothing new is required.

Anchor (`:169-172`): first `_FLIGHT_START` instead of `_SORTIE_START`; TELEM filter uses `flight` groups. Rename `starts`→ flights, `ends`→ flight_ends, and derive releases from `_RELEASE`, delivery-ends from `_DELIV_END`.

Transit section (`:324-339`): iterate FLIGHTS; `seq` filtered by `tr["flight"] == i`; same `want` order (one ingress + one egress per flight). 

Terminal section (`:341-380`): iterate the `DELIVERY … END delivered=True` lines; for each, find the `_RELEASE` line with the **same `k`** (exact match, no `-1` inference), score its lat/lon vs `truth_by_id[pad]`, and check the nearest pre-release TELEM reads on-ground.

L&R section (`:382-410`): iterate `_FLIGHT_END`; same distance + config cross-check + disarm-after check.

Timing/coverage (`:412-425`): `t_last <= window_s`; warn if `sum(delivered) < sum(of)` across flights.

- [ ] **Step 4: Run tests**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_verify_flight.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add tools/verify_flight.py tests/test_verify_flight.py
git commit -m "feat(verify): FLIGHT/DELIVERY grammar; match release by delivery index"
```

---

### Task 9: SITL — spawn 6 pads, assign 4

**Files:**
- Modify: `sitl/spawn_targets.py:78` (`N_PADS`), `:195-206` (`_layout`), `:209-241` (config-driven `n_pads`, `_ALL_PAD_NAMES`)
- Modify: `sitl/aavc_config.yaml` (add `sitl.n_pads: 6`, `mission.assigned_marker_ids` = 4-of-6)
- Test: `tests/test_spawn_targets.py` (create if absent, else extend)

**Interfaces:**
- Produces: `_layout(seed, poly, valid_ids, n_pads)` places `n_pads` distinct-id pads (up to `len(valid_ids)`); truth JSON holds all placed pads. The mission's `assigned_marker_ids` is a 4-subset, so ≥2 pads are permanent distractors.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_spawn_targets.py (add)
from sitl.spawn_targets import _layout, _latlon_to_local_enu  # import what exists


def test_layout_places_six_distinct_ids_when_asked():
    poly = [(0.0, 0.0), (200.0, 0.0), (200.0, 60.0), (0.0, 60.0)]
    pads = _layout(seed=42, search_poly_enu=poly, valid_ids=[1, 2, 3, 4, 5, 6],
                   n_pads=6)
    ids = [p[0] for p in pads]
    assert len(pads) == 6
    assert sorted(ids) == [1, 2, 3, 4, 5, 6]
```

- [ ] **Step 2: Run to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_spawn_targets.py -k six_distinct -q`
Expected: FAIL — `_layout()` takes no `n_pads` argument.

- [ ] **Step 3: Implement**

In `sitl/spawn_targets.py`: change `N_PADS = 4` to `DEFAULT_N_PADS = 6`. `_ALL_PAD_NAMES` must cover up to 6 — derive it dynamically in `main()` from the chosen count instead of the module constant, or set `_ALL_PAD_NAMES = tuple(f"pad_{i}" for i in range(1, 7))`. Change `_layout`:

```python
def _layout(seed, search_poly_enu, valid_ids, n_pads=DEFAULT_N_PADS):
    """(marker_id, east, north, yaw_deg) per pad. No seed → the world baseline;
    a seed picks ``n_pads`` distinct ids of the valid set + fresh positions."""
    if seed is None:
        return list(BASELINE_PADS)
    rng = random.Random(seed)
    k = min(n_pads, len(valid_ids))
    ids = rng.sample(sorted(valid_ids), k=k)
    pts = _sample_positions(search_poly_enu, len(ids), rng)
    yaw_rng = random.Random(seed + 104729)
    return [(mid, x, y, yaw_rng.uniform(0.0, 360.0))
            for mid, (x, y) in zip(ids, pts, strict=True)]
```

In `main()`: read `n_pads = int(cfg.get("sitl", {}).get("n_pads", DEFAULT_N_PADS))`, pass to `_layout`, and add `--n-pads` arg overriding it. Update `BASELINE_PADS` to include 6 ids (add ids 2 and 5 at two more well-separated ENU positions) so a no-seed spawn also places 6.

- [ ] **Step 4: Config**

`sitl/aavc_config.yaml`: add under a `sitl:` block `n_pads: 6`. Ensure `mission.assigned_marker_ids` (or the run's `--assigned-ids`) names 4 ids that are a subset of the 6.

- [ ] **Step 5: Run tests + a spawn smoke (optional, needs SITL)**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_spawn_targets.py tests/test_world_assets.py -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add sitl/spawn_targets.py sitl/aavc_config.yaml tests/test_spawn_targets.py
git commit -m "feat(sitl): spawn 6 pads; assign 4 (2 permanent distractors)"
```

---

### Task 10: SITL — dynamic `cargo_payload` model + belly attach + detachable joints

**Files:**
- Create: `sitl/models/cargo_payload/model.sdf`, `sitl/models/cargo_payload/model.config`
- Modify: `sitl/models/eft_x6100/model.sdf` (4 DetachableJoint plugins; base_link ↔ cargo_payload_N)
- Modify: `sitl/worlds/aavc_field.sdf` (include 4 `cargo_payload_N` at the belly of the world-origin spawn)
- Test: `tests/test_world_assets.py` (extend)

**Interfaces:**
- Produces: dynamic model `cargo_payload` (~0.09 kg, box 0.16×0.07×0.18 m) with link `payload`. Aircraft carries 4 via `DetachableJoint` whose `<detach_topic>` is `/model/eft_x6100/detach_payload_{0..3}`. While attached, each adds mass to the flight dynamics (Tier 1); publishing on the topic sheds it onto the pad (Tier 2).

- [ ] **Step 1: Write the failing structural test**

> Reuse the file's EXISTING `import xml.etree.ElementTree as ET` and `_REPO` — do not add new imports. `test_world_assets.py:12-16` already documents why stdlib ElementTree is correct here (trusted repo-local SDF; lean-deps doctrine §4 rules out `defusedxml` for a test-only parse). A `PostToolUse` security hook will warn about ElementTree; that warning is already adjudicated by that comment — ignore it for this file only.

```python
# tests/test_world_assets.py (add — ET and _REPO already exist in the file)
_REPO = Path(__file__).resolve().parent.parent


def test_cargo_payload_model_is_dynamic_with_mass():
    root = ET.parse(_REPO / "sitl/models/cargo_payload/model.sdf").getroot()
    model = root.find("model")
    assert (model.findtext("static") or "false").strip().lower() == "false"
    mass = float(model.find(".//inertial/mass").text)
    assert 0.05 <= mass <= 0.15


def test_aircraft_has_four_detach_topics():
    txt = (_REPO / "sitl/models/eft_x6100/model.sdf").read_text()
    for i in range(4):
        assert f"detach_payload_{i}" in txt


def test_world_includes_four_cargo_payloads():
    root = ET.parse(_REPO / "sitl/worlds/aavc_field.sdf").getroot()
    world = root.find("world")
    names = [inc.findtext("name") for inc in world.iter("include")
             if (inc.findtext("uri") or "").strip() == "model://cargo_payload"]
    assert sorted(n for n in names if n) == [
        "cargo_payload_0", "cargo_payload_1", "cargo_payload_2", "cargo_payload_3"]
```

- [ ] **Step 2: Run to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_world_assets.py -k "cargo_payload or detach" -q`
Expected: FAIL (files/plugins absent).

- [ ] **Step 3: Create `cargo_payload` model**

`sitl/models/cargo_payload/model.config` — copy `cargo_box/model.config`, set `<name>cargo_payload</name>`.

`sitl/models/cargo_payload/model.sdf` — dynamic box with mass/inertia (thin-box inertia for m=0.09, w=0.16, d=0.07, h=0.18):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  AAVC 2026 dynamic payload dummy (rules-V1.3 organiser package, Fig. 5): a
  ~16x7x18 cm heart box + one no.0 egg, ~90 g. Unlike the static cargo_box
  (ground dressing at L&R), this is a DYNAMIC model carried in the aircraft
  belly via a DetachableJoint and shed onto the pad on release — so its mass
  loads the flight dynamics while aboard and leaves when dropped.
-->
<sdf version="1.9">
  <model name="cargo_payload">
    <link name="payload">
      <inertial>
        <mass>0.090</mass>
        <!-- solid box: Ix=m/12(d^2+h^2), Iy=m/12(w^2+h^2), Iz=m/12(w^2+d^2) -->
        <inertia>
          <ixx>0.000280</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000435</iyy><iyz>0</iyz>
          <izz>0.000229</izz>
        </inertia>
      </inertial>
      <visual name="body">
        <geometry><box><size>0.16 0.07 0.18</size></box></geometry>
        <material>
          <ambient>0.92 0.92 0.90 1</ambient>
          <diffuse>0.96 0.96 0.94 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.16 0.07 0.18</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```

- [ ] **Step 4: Pre-place 4 payloads in the world at the belly**

In `sitl/worlds/aavc_field.sdf`, near the aircraft-spawn origin (world 0,0), the aircraft base_link sits ~0.35 m up. Place 4 payloads just under it (z so the box top is a few cm below base_link, boxes spread along Y within the ≈0.40×0.64 m belly, centres of the 0.07 m-deep boxes ~0.09 m apart):

```xml
    <!-- Four dynamic cargo payloads carried in the X6100 belly, attached to
         base_link by DetachableJoints (see sitl/models/eft_x6100/model.sdf).
         Pre-placed at the world-origin spawn belly so the joints bind on
         aircraft spawn; shed onto the pad on release by the detach bridge. -->
    <include><name>cargo_payload_0</name><uri>model://cargo_payload</uri>
      <pose>0.06 0.14 0.14 0 0 0</pose></include>
    <include><name>cargo_payload_1</name><uri>model://cargo_payload</uri>
      <pose>0.06 0.05 0.14 0 0 0</pose></include>
    <include><name>cargo_payload_2</name><uri>model://cargo_payload</uri>
      <pose>0.06 -0.05 0.14 0 0 0</pose></include>
    <include><name>cargo_payload_3</name><uri>model://cargo_payload</uri>
      <pose>0.06 -0.14 0.14 0 0 0</pose></include>
```

> The exact z/x may need a small nudge so the boxes don't intersect the gear/legs at spawn — verify visually in Step 6 and adjust the pose, not the joint.

- [ ] **Step 5: Add 4 DetachableJoint plugins to the aircraft**

In `sitl/models/eft_x6100/model.sdf`, before `</model>` add four plugins:

```xml
    <!-- Cargo payloads: base_link ↔ cargo_payload_N, shed on release. Publishing
         an Empty on <detach_topic> breaks the joint (SITL Tier-2 drop). While
         attached each ~90 g box loads the flight dynamics (Tier-1). -->
    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>cargo_payload_0</child_model>
      <child_link>payload</child_link>
      <detach_topic>/model/eft_x6100/detach_payload_0</detach_topic>
    </plugin>
    <!-- repeat for cargo_payload_1 → detach_payload_1, _2, _3 -->
```

Add all four (indices 0..3), each naming its own `child_model` and `detach_topic`.

- [ ] **Step 6: Verify structurally + (if SITL available) visually**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_world_assets.py -q`
Expected: PASS.
If SITL is up: `make sitl`, confirm the aircraft spawns with 4 boxes attached (`gz model --list` shows `cargo_payload_0..3`; they ride with the aircraft on takeoff). Nudge the belly poses if any box clips the gear.

- [ ] **Step 7: Commit**

```bash
git add sitl/models/cargo_payload/ sitl/models/eft_x6100/model.sdf sitl/worlds/aavc_field.sdf tests/test_world_assets.py
git commit -m "feat(sitl): dynamic belly payloads on detachable joints (Tier 1+2)"
```

---

### Task 11: SITL — `payload_detach_bridge.py` (audit-tail → gz detach) + launcher wiring

**Files:**
- Create: `sitl/payload_detach_bridge.py`
- Modify: `sitl/launch_sitl.sh` (or `Makefile`) — a `payload-bridge` target/launch
- Test: `tests/test_payload_detach_bridge.py` (parser unit; the gz publish is manual/SITL)

**Interfaces:**
- Consumes: the run's `audit.jsonl` (RELEASE lines from Task 3), the gz detach topics from Task 10.
- Produces: a system-python3 process that tails the audit file from EOF and, on each `DELIVERY k RELEASE pad=P payload=Y …`, publishes an `Empty` on `/model/eft_x6100/detach_payload_{Y}` exactly once per payload index. Pure-parse helper `parse_release(line) -> tuple[int, int] | None` (returns `(delivery_k, payload_id)`), unit-tested without gz.

Why audit-tail, not "watch the actuator": the SITL airframe maps only 6 motor outputs to gz (`SIM_GZ_EC_FUNC1..6`) — no servo output is published, so there is nothing in gz to observe. The audit trail is the flight core's own release record and needs no flight-core change to consume.

- [ ] **Step 1: Write the failing parser test**

```python
# tests/test_payload_detach_bridge.py
from sitl.payload_detach_bridge import parse_release


def test_parse_release_extracts_delivery_and_payload():
    line = ("t=3.2s DELIVERY 3 RELEASE pad=4 payload=2 "
            "lat=13.7307000 lon=100.7880000")
    assert parse_release(line) == (3, 2)


def test_parse_release_ignores_other_lines():
    assert parse_release("t=1.0s FLIGHT 1 START eggs=4 ids=3,1,4,6 remaining=1200s") is None
    assert parse_release("t=3.1s DELIVERY 3 START pad=4 payload=2 stop_index=2") is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_payload_detach_bridge.py -q`
Expected: FAIL — module missing.

- [ ] **Step 3: Implement the bridge**

```python
#!/usr/bin/python3
"""SITL payload-detach bridge — tails a mission run's audit trail and sheds the
matching cargo box in Gazebo on each release.

The flight core writes `DELIVERY k RELEASE pad=P payload=Y …` to
runs/<id>/audit.jsonl when it opens a hold. This SITL-only helper watches that
file (from EOF, so cross-run appends don't fire stale drops) and publishes an
Empty on /model/eft_x6100/detach_payload_<Y>, which breaks that box's
DetachableJoint so it falls onto the pad — the same physical effect the real
servo has, with zero coupling to the flight core.

No gz servo output exists to observe (the airframe maps only motor outputs),
so the audit trail is the trigger. Degrades to a no-op if gz-transport isn't
importable.

INVOKE WITH /usr/bin/python3 (gz-transport is an apt package, not in the venv).
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

_RELEASE = re.compile(
    r"DELIVERY (?P<k>\d+) RELEASE pad=(?P<pad>\d+) payload=(?P<payload>\d+) ")


def parse_release(line: str) -> tuple[int, int] | None:
    """(delivery_k, payload_id) for a RELEASE line, else None. Pure — no gz."""
    m = _RELEASE.search(line)
    return (int(m.group("k")), int(m.group("payload"))) if m else None


def _run(audit: Path, model: str, poll_s: float) -> int:
    try:
        from gz.msgs10.empty_pb2 import Empty
        from gz.transport13 import Node
    except Exception as e:  # gz not present → no-op, Tier-1 mass still valid
        print(f"[detach] gz-transport unavailable ({e}) — bridge disabled")
        return 0
    node = Node()
    pubs = {i: node.advertise(f"/model/{model}/detach_payload_{i}", Empty)
            for i in range(4)}
    fired: set[int] = set()
    # Start at EOF so a re-used audit.jsonl doesn't replay old releases.
    while not audit.exists():
        time.sleep(poll_s)
    with audit.open() as fh:
        fh.seek(0, 2)
        while True:
            line = fh.readline()
            if not line:
                time.sleep(poll_s)
                continue
            r = parse_release(line)
            if r is None:
                continue
            _, payload = r
            if payload in fired or payload not in pubs:
                continue
            pubs[payload].publish(Empty())
            fired.add(payload)
            print(f"[detach] shed payload {payload} (detach_payload_{payload})")


def main() -> int:
    ap = argparse.ArgumentParser(description="SITL cargo-detach bridge")
    ap.add_argument("audit", type=Path, help="runs/<id>/audit.jsonl to tail")
    ap.add_argument("--model", default="eft_x6100")
    ap.add_argument("--poll-s", type=float, default=0.2)
    args = ap.parse_args()
    try:
        return _run(args.audit, args.model, args.poll_s)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Launcher wiring**

Add a Makefile target so the operator can start it against the active run:

```make
payload-bridge:  ## SITL: shed cargo boxes on release (pass RUN=runs/<id>/audit.jsonl)
	/usr/bin/python3 sitl/payload_detach_bridge.py $(RUN) --model eft_x6100
```

Document in the SITL section that it's optional (Tier-1 mass works without it; it adds the visible drop + mass-shed).

- [ ] **Step 5: Run parser tests**

Run: `env -u PYTHONPATH .venv/bin/pytest tests/test_payload_detach_bridge.py -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add sitl/payload_detach_bridge.py tests/test_payload_detach_bridge.py Makefile
git commit -m "feat(sitl): audit-tail payload-detach bridge (Tier-2 drop)"
```

---

### Task 12: Config, docs, and the G4′ verification run

**Files:**
- Modify: `sitl/aavc_config.yaml` (`mission.eggs_aboard: 4`, `mission.max_deliveries: 4`, `drop_payload_count: 4`, keep `drop_servo_channel: 9`)
- Modify: `mavlink_adapter/commands.py:43` default `drop_payload_count` note (leave default 1; config drives 4) — confirm bounds-check allows 0..3
- Modify: `docs/RULES_AAVC2026.md`, `CLAUDE.md` (record the briefing override + the FLIGHT/DELIVERY model)
- Test: full suite + lint; SITL G4′
- Create: `docs/evidence/G4_multiegg_<date>.txt` (the passing run)

**Interfaces:** none (integration + documentation).

- [ ] **Step 1: Config**

`sitl/aavc_config.yaml` `mission:` block: add `eggs_aboard: 4`, `max_deliveries: 4`; set `drop_payload_count: 4` (the `drop:`/servo block). Leave `drop_servo_channel: 9` (→ 9/10/11/12).

- [ ] **Step 2: Docs**

`docs/RULES_AAVC2026.md`: add a subsection under the V1.3 changes noting the **event-briefing override** (6 pads placed, 4 assigned, all eggs in one flight, per-delivery scoring) beats the "subject to be changed" PDF, mapping to `mission.eggs_aboard`.

`CLAUDE.md`: update §2 "One egg per sortie, ≤4 sorties" → the FLIGHT ⊃ DELIVERY model with `eggs_aboard`; update §5 audit-trail grammar (`FLIGHT`/`DELIVERY`); note AUX 9–12 in §2/§8 hardware; note `sortie==flight` internally.

- [ ] **Step 3: Full test + lint**

Run:
```
make test
make lint
```
Expected: all green. Fix any stragglers referencing the old grammar/`sortie=` token.

- [ ] **Step 4: G4′ SITL run (single flight, 4 deliveries, 6-pad field)**

Follow the CLAUDE.md §6 flow (detach gz/PX4/bridge with nohup per the task-cap note). Sequence:
```
make sitl            # KMITL field
make spawn-targets SEED=42          # 6 pads, ids 1-6; note the 4 assigned ⊂ 6
make camera-bridge   # nadir frames
# start the detach bridge against the run once it exists:
#   /usr/bin/python3 sitl/payload_detach_bridge.py runs/<id>/audit.jsonl &
.venv/bin/python -m orchestrator.main --assigned-ids "3,1,4,6"   # eggs_aboard=4 → 1 flight
.venv/bin/python tools/verify_flight.py runs/<id>/audit.jsonl --truth /tmp/aavc_targets.json
```
Expected: one FLIGHT, four DELIVERY releases id-correct within `landing_accuracy_threshold_m`, transit 6/6 in order, window < 20 min, `verify_flight` PASS (0 fails). Capture output to `docs/evidence/G4_multiegg_<date>.txt`.

- [ ] **Step 5: Commit**

```bash
git add sitl/aavc_config.yaml mavlink_adapter/commands.py docs/ CLAUDE.md
git commit -m "feat(config,docs): eggs_aboard=4, AUX 9-12, briefing override; G4′ evidence"
```

---

## Self-Review

**Spec coverage:**
- FLIGHT ⊃ DELIVERY vocab + `eggs_aboard` knob → Tasks 1, 4, 6, 7 ✓
- Two indices (`payload_id` per-flight, `stop_index` mission-global) → Tasks 2, 3, 6 ✓
- Mission loop `for flight: for delivery` → Task 6 ✓
- Queue-order serve, partial-find skip, no-retry-of-skipped → Task 6 (tests) ✓
- Per-delivery abort gate (time + battery) → Tasks 5, 6 ✓
- Audit grammar break + verify_flight in lockstep → Tasks 3, 6, 8 ✓
- 6-pad SITL / 4 assigned / 2 distractors → Task 9 (+ registry-never-serves-distractor is covered by Task 6's partial-find test and existing `test_target_tracker`) ✓
- Payload dummies Tier 1 (loaded mass) + Tier 2 (detach on release) → Tasks 10, 11 ✓
- AUX 9-12 release channels → Tasks 3, 12 ✓
- Hardware notes (4 independent mechanisms, MTOW/CG, energy, eggs_aboard=2 fallback) → docs in Task 12; no code beyond `drop_payload_count` ✓
- Rollback = one integer → `eggs_aboard=1` regression tests in Tasks 1, 6, 7 ✓

**Deviations from the spec (intentional, noted):**
1. `state.sortie_index` is **kept** (not renamed to `flight_index`) — "sortie" already denotes one arm→disarm cycle, and a rename churns 10 dashboard/test files for no behavioural gain. `sortie_index` = the flight counter; new `delivery_index`/`flight_ids` carry delivery state. The operator-facing **audit grammar** still uses FLIGHT/DELIVERY per the spec.
2. Tier-2 trigger is the **audit tail**, not "watch the actuator output PX4 publishes into gz" — the airframe maps no servo output to gz, so there is nothing to observe. Same effect, still zero flight-core coupling.

**Placeholder scan:** none — every code step carries real code; SITL pose nudges and fixture reuse are called out explicitly with how to resolve.

**Type consistency:** `chunk_flights(list[int], int) -> list[list[int]]` used identically in Tasks 1/6/7. `ServedStop.payload_id` (Task 2) consumed in Task 6's `_serve`. `acquire_and_land_drop(..., payload_id=, delivery_index=)` (Task 3) called with those kwargs in Task 6. `FlightGate = Callable[[int], Awaitable[list[int] | None]]` defined in Task 6, produced in Task 7. Audit tokens `flight=`/`DELIVERY k`/`payload=` written in Tasks 3/6 and parsed in Task 8. Detach topic `/model/eft_x6100/detach_payload_{i}` defined in Task 10, published in Task 11.

## Open questions (carry to the tech exchange / G5)
- Does per-delivery scoring also re-score the transit corridor per delivery? Plan assumes one ingress + one egress per flight.
- Whether the 7.17 kg AUW already includes cargo (affects MTOW headroom for 4 boxes + 4 mechanisms).
