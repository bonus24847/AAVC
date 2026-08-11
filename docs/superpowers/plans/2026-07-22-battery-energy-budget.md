# Battery Energy Budget Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tell the operator, before each per-sortie GO, whether the pack can cover the next sortie or must be swapped — and block GO when it cannot, with a FORCE escape.

**Architecture:** A pure-logic `EnergyPolicy` mirroring `orchestrator/time_policy.py`, fed by a two-tier energy reading (FC coulomb count when available, percent × capacity otherwise), surfaced as an advisory preflight row plus a `state.sortie_energy_ok` flag the GO endpoint honours. SITL gets PX4's battery simulator enabled so the whole chain is exercisable.

**Tech Stack:** Python 3.12, pydantic, pytest, MAVSDK/pymavlink, Svelte 5 (runes), FastAPI.

## Global Constraints

- Pack: DXF **6S 7500 mAh** through a Holybro PM03D. Reserve fraction = the existing `failsafes.bat_low_thr` (**0.25**) → **5,625 mAh usable**.
- `seed_sortie_mah` = **1700** (29 A hover × 3.5 min, EFT bench table via `Power-System-Guide-1.pdf`).
- Config and `mavlink_adapter/commands.py::DEFAULT_PX4_TUNING` must stay byte-identical — `tests/test_px4_tuning_parity.py` enforces it. **SITL-only `SIM_BAT_*` params must NOT go in that block.**
- `make test` and `make lint` must pass at every commit (`env -u PYTHONPATH` — a sourced ROS env breaks collection).
- No `print()` in flight paths; `loguru` only. Type hints throughout.

## ⚠ Design correction to the spec (read before Task 4)

The spec says the energy row should be a **critical** preflight item. **Do not do that.** `dashboard/commands.py::preflight_go` checks `state.preflight_can_go` (= `all_critical_pass`) with **no force escape**, so a critical row makes GO impossible even with `force: true` — the exact dead-path the "time" row hit (fixed 2026-07-15, a138aaf; see the comment at `orchestrator/preflight.py:190-195`).

Mirror the proven pattern instead:
- the preflight row is **advisory** (`critical=False`), so it shows on the card without freezing the board, and
- the blocking lives in `state.sortie_energy_ok`, checked at the GO endpoint as `if not state.sortie_energy_ok and not req.force`.

Behaviour is what the operator asked for — GO is blocked, FORCE overrides — without recreating a known bug.

## File Structure

| File | Responsibility |
|---|---|
| `orchestrator/energy_policy.py` (new) | Pure arithmetic: usable energy, per-sortie cost, can-start predicate + reason |
| `tests/test_energy_policy.py` (new) | Unit tests for the above, incl. the boundary |
| `orchestrator/state.py` | Energy fields: capacity, consumed baseline, per-sortie history, `sortie_energy_ok` |
| `mavlink_adapter/telemetry.py` | Expose `battery_current_a` (already streamed by MAVSDK, currently dropped) |
| `dashboard/payloads.py` | Wire fields to the GCS |
| `dashboard/realtime.py` | Fill them per frame |
| `orchestrator/preflight.py` | Advisory `energy` row |
| `orchestrator/main.py` | Build the policy from config; set `sortie_energy_ok` at each gate |
| `orchestrator/mission.py` | Record per-sortie consumption; detect a swap |
| `dashboard/commands.py` | GO honours `sortie_energy_ok` unless forced |
| `dashboard/web/src/widgets/TelemetrySidebar.svelte` | Battery panel: capacity, used, remaining, power, sorties-left, swap prompt |
| `sitl/aavc_config.yaml` | `battery:` block + `sim_battery:` SITL-only block |

---

### Task 1: EnergyPolicy — pure logic

**Files:**
- Create: `orchestrator/energy_policy.py`
- Test: `tests/test_energy_policy.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `EnergyPolicy(capacity_mah: float, reserve_frac: float, seed_sortie_mah: float, margin_mah: float)` with `usable_mah() -> float`, `sortie_cost_mah(history: Sequence[float]) -> float`, `can_start_sortie(consumed_mah: float, history: Sequence[float]) -> tuple[bool, str]`.

- [ ] **Step 1: Write the failing test**

```python
"""Unit tests for the per-sortie energy budget (mirrors test_time_policy)."""
from orchestrator.energy_policy import EnergyPolicy

POLICY = EnergyPolicy(capacity_mah=7500.0, reserve_frac=0.25,
                      seed_sortie_mah=1700.0, margin_mah=150.0)


def test_usable_excludes_the_fc_reserve():
    assert POLICY.usable_mah() == 5625.0


def test_cost_uses_the_seed_until_a_sortie_has_been_measured():
    assert POLICY.sortie_cost_mah([]) == 1700.0


def test_cost_is_the_median_of_measured_sorties():
    assert POLICY.sortie_cost_mah([1500.0, 1600.0, 2000.0]) == 1600.0


def test_allows_a_sortie_when_the_pack_can_cover_it():
    ok, reason = POLICY.can_start_sortie(consumed_mah=1000.0, history=[])
    assert ok
    assert "1700" in reason


def test_blocks_when_the_remaining_charge_cannot_cover_the_next_sortie():
    # 5625 usable - 4200 used = 1425 left, under 1700 + 150 margin
    ok, reason = POLICY.can_start_sortie(consumed_mah=4200.0, history=[])
    assert not ok
    assert "swap" in reason.lower()


def test_boundary_is_cost_plus_margin():
    exact = POLICY.usable_mah() - (1700.0 + 150.0)
    assert POLICY.can_start_sortie(exact, [])[0]
    assert not POLICY.can_start_sortie(exact + 1.0, [])[0]


def test_unknown_consumption_is_not_treated_as_empty():
    ok, reason = POLICY.can_start_sortie(float("nan"), [])
    assert ok
    assert "unknown" in reason.lower()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/python -m pytest tests/test_energy_policy.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'orchestrator.energy_policy'`

- [ ] **Step 3: Write the implementation**

```python
"""Energy-budget policy for the multi-sortie delivery window.

The twin of time_policy.py: that one refuses to start a sortie the clock cannot
finish, this one refuses to start a sortie the PACK cannot finish. A 7,500 mAh
pack yields 5,625 mAh once the flight controller's own low-battery reserve is
set aside, and the four-sortie mission needs about 6,800 mAh, so the aircraft
lands at L&R and the crew swaps the pack — the rules already have them approach
between sorties. The point of this module is that the operator learns that
BEFORE spending window time on a sortie that cannot finish.

Like time_policy this only refuses to start new work; it is never a flight
action. The safety watchdog and the FC's own battery failsafe remain the
hard stops.

Pure arithmetic — no telemetry, no clock — so it is trivially unit-testable.
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class EnergyPolicy:
    # Pack label capacity. Declared in config and cross-checked against the FC's
    # BAT1_CAPACITY, because a swapped pack nobody told the config about and an
    # uncalibrated power module look identical from here.
    capacity_mah: float = 7500.0
    # Charge the companion may not plan against: the flight controller triggers
    # its own low-battery RTL at this fraction, so planning below it would be
    # planning through a failsafe. Construct from failsafes.bat_low_thr.
    reserve_frac: float = 0.25
    # First-sortie estimate, replaced by measurement as soon as one sortie flies.
    # 29 A hover x 3.5 min from the EFT E5 bench table (Power-System-Guide-1.pdf).
    seed_sortie_mah: float = 1700.0
    # Slack so a sortie approved at the boundary still lands with charge in hand.
    margin_mah: float = 150.0

    def usable_mah(self) -> float:
        """Charge available for planning, i.e. above the FC's own reserve."""
        return self.capacity_mah * (1.0 - self.reserve_frac)

    def sortie_cost_mah(self, history: Sequence[float]) -> float:
        """Expected cost of the next sortie: the median of what sorties have
        actually cost, or the seed estimate before any have flown. Median, not
        mean, so one sweep-heavy sortie does not distort the rest."""
        measured = [v for v in history if v > 0 and not math.isnan(v)]
        return statistics.median(measured) if measured else self.seed_sortie_mah

    def can_start_sortie(
        self, consumed_mah: float, history: Sequence[float],
    ) -> tuple[bool, str]:
        """Whether the pack can cover another sortie, and why — the reason is
        written for the operator and is shown verbatim on the pre-flight card.

        An unknown consumption (NaN — no calibrated power module yet) allows the
        sortie: refusing to fly because we cannot measure would ground a
        perfectly good aircraft, and the FC's own failsafe still protects it.
        """
        cost = self.sortie_cost_mah(history)
        if math.isnan(consumed_mah):
            return True, (f"energy unknown (no calibrated power module) — "
                          f"next sortie needs ~{cost:.0f} mAh")
        left = self.usable_mah() - consumed_mah
        if left >= cost + self.margin_mah:
            return True, (f"{left:.0f} mAh usable left, next sortie "
                          f"needs ~{cost:.0f}")
        return False, (f"{left:.0f} mAh usable left, next sortie needs "
                       f"~{cost:.0f} — swap the battery")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `env -u PYTHONPATH .venv/bin/python -m pytest tests/test_energy_policy.py -q`
Expected: PASS, 7 passed

- [ ] **Step 5: Commit**

```bash
git add orchestrator/energy_policy.py tests/test_energy_policy.py
git commit -m "feat(energy): per-sortie energy budget policy"
```

---

### Task 2: Two-tier energy reading + state

**Files:**
- Modify: `orchestrator/state.py`
- Modify: `mavlink_adapter/telemetry.py`
- Test: `tests/test_energy_policy.py` (append)

**Interfaces:**
- Consumes: `EnergyPolicy` from Task 1.
- Produces: `energy_consumed_mah(telemetry, capacity_mah) -> tuple[float, str]` in `orchestrator/energy_policy.py` returning `(mah, tier)` where tier is `"A"` (coulomb count) or `"B"` (percent estimate) or `"none"`. New `OrchestratorState` fields: `energy_capacity_mah: float`, `energy_baseline_mah: float`, `sortie_energy_mah: list[float]`, `sortie_energy_ok: bool`, `energy_tier: str`.

- [ ] **Step 1: Write the failing test (append to `tests/test_energy_policy.py`)**

```python
from types import SimpleNamespace

from orchestrator.energy_policy import energy_consumed_mah


def _telem(consumed=float("nan"), percent=float("nan")):
    return SimpleNamespace(battery_consumed_mah=consumed, battery_percent=percent)


def test_tier_a_prefers_the_flight_controller_coulomb_count():
    mah, tier = energy_consumed_mah(_telem(consumed=1234.0, percent=80.0), 7500.0)
    assert (round(mah), tier) == (1234, "A")


def test_tier_b_falls_back_to_percent_times_capacity():
    mah, tier = energy_consumed_mah(_telem(percent=80.0), 7500.0)
    assert (round(mah), tier) == (1500, "B")


def test_no_tier_when_neither_signal_is_available():
    mah, tier = energy_consumed_mah(_telem(), 7500.0)
    assert math.isnan(mah) and tier == "none"


def test_a_negative_coulomb_count_is_not_trusted():
    # PX4 reports -1 (SITL sends -383) when the module cannot measure
    mah, tier = energy_consumed_mah(_telem(consumed=-383.0, percent=90.0), 7500.0)
    assert (round(mah), tier) == (750, "B")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `env -u PYTHONPATH .venv/bin/python -m pytest tests/test_energy_policy.py -q`
Expected: FAIL — `ImportError: cannot import name 'energy_consumed_mah'`

- [ ] **Step 3: Implement — append to `orchestrator/energy_policy.py`**

```python
def energy_consumed_mah(telemetry: object, capacity_mah: float) -> tuple[float, str]:
    """Charge drawn from the pack so far, and which tier the number came from.

    Tier A is the flight controller's coulomb count — trustworthy, but only
    present once the power module is calibrated. Tier B derives it from the
    reported percentage, which PX4 itself estimates; it is much coarser and the
    GCS says so, because an estimate shown as a measurement is worse than no
    number. A negative coulomb count is PX4's "not available" sentinel.
    """
    consumed = float(getattr(telemetry, "battery_consumed_mah", float("nan")))
    if not math.isnan(consumed) and consumed >= 0.0:
        return consumed, "A"
    percent = float(getattr(telemetry, "battery_percent", float("nan")))
    if not math.isnan(percent) and 0.0 <= percent <= 100.0:
        return capacity_mah * (1.0 - percent / 100.0), "B"
    return float("nan"), "none"
```

- [ ] **Step 4: Add the state fields — `orchestrator/state.py`, beside `sortie_time_ok`**

```python
    # ── energy budget (orchestrator/energy_policy.py) ──
    energy_capacity_mah: float = field(default=0.0, compare=False, repr=False)
    # Consumption reading at the start of the current pack; a battery swap
    # rebases it so per-sortie costs stay comparable across packs.
    energy_baseline_mah: float = field(default=0.0, compare=False, repr=False)
    sortie_energy_mah: list[float] = field(default_factory=list, compare=False, repr=False)
    sortie_energy_ok: bool = field(default=True, compare=False, repr=False)
    energy_tier: str = field(default="none", compare=False, repr=False)
```

- [ ] **Step 5: Expose pack current — `mavlink_adapter/telemetry.py`**

Add to `CurrentTelemetry` beside `battery_voltage_v`:

```python
    battery_current_a: float = math.nan
```

and in `_sub_battery`, after `self.state.battery_voltage_v = bat.voltage_v`:

```python
            # MAVSDK has carried this since v2; it drives the GCS power readout.
            self.state.battery_current_a = float(
                getattr(bat, "current_battery_a", math.nan))
```

- [ ] **Step 6: Run the tests**

Run: `make test`
Expected: PASS, 265 passed

- [ ] **Step 7: Commit**

```bash
git add orchestrator/energy_policy.py orchestrator/state.py mavlink_adapter/telemetry.py tests/test_energy_policy.py
git commit -m "feat(energy): two-tier consumption reading + pack current"
```

---

### Task 3: Config block + FC capacity cross-check

**Files:**
- Modify: `sitl/aavc_config.yaml`
- Modify: `orchestrator/main.py`
- Test: `tests/test_config.py`

**Interfaces:**
- Consumes: `EnergyPolicy` (Task 1), state fields (Task 2).
- Produces: `state.energy_capacity_mah` populated; an `EnergyPolicy` instance built in `main.run()` and passed to the gate closure.

- [ ] **Step 1: Write the failing test (append to `tests/test_config.py`)**

```python
def test_battery_block_matches_the_energy_policy_defaults():
    cfg = yaml.safe_load(Path("sitl/aavc_config.yaml").read_text())
    bat = cfg["battery"]
    assert bat["capacity_mah"] == 7500
    assert bat["cells"] == 6
    assert bat["seed_sortie_mah"] == 1700
    # the reserve is the FC's own low-battery threshold, not a second number
    assert cfg["failsafes"]["bat_low_thr"] == 0.25
```

- [ ] **Step 2: Run it**

Run: `env -u PYTHONPATH .venv/bin/python -m pytest tests/test_config.py -q`
Expected: FAIL — `KeyError: 'battery'`

- [ ] **Step 3: Add the config block** — `sitl/aavc_config.yaml`, immediately after the `failsafes:` block

```yaml
# Pack + energy budget (orchestrator/energy_policy.py). The reserve is NOT set
# here: it is failsafes.bat_low_thr above, because below that the FC flies its
# own low-battery RTL and planning past it would be planning through a failsafe.
battery:
  capacity_mah: 7500        # DXF 6S 7500 mAh 140C
  cells: 6
  # 29 A hover x 3.5 min, from the EFT E5 bench table in Power-System-Guide-1.pdf.
  # Used ONLY for sortie 1; measurement replaces it from sortie 2 on.
  seed_sortie_mah: 1700
  margin_mah: 150

# SITL ONLY — PX4's battery simulator. Without it the simulated pack never
# drains (measured 2026-07-22: 100 % and 16.2 V flat through a whole flight) and
# none of the energy logic can be exercised before real hardware. These params
# do not exist on the real board; they are applied on a separate path and must
# NEVER be merged into px4_tuning.
sim_battery:
  SIM_BAT_ENABLE: 1
  SIM_BAT_DRAIN: 1500.0     # seconds from full to empty ~ one 20-min window + margin
  SIM_BAT_MIN_PCT: 0.0      # default 50 stops the drain half way
```

- [ ] **Step 4: Build the policy — `orchestrator/main.py`, next to the `TimePolicy` construction**

```python
    bat_cfg = cfg.get("battery") or {}
    energy_policy = EnergyPolicy(
        capacity_mah=float(bat_cfg.get("capacity_mah", 7500.0)),
        reserve_frac=float((cfg.get("failsafes") or {}).get("bat_low_thr", 0.25)),
        seed_sortie_mah=float(bat_cfg.get("seed_sortie_mah", 1700.0)),
        margin_mah=float(bat_cfg.get("margin_mah", 150.0)),
    )
    state.energy_capacity_mah = energy_policy.capacity_mah
```

with `from orchestrator.energy_policy import EnergyPolicy` at the top.

- [ ] **Step 5: Cross-check the FC — `orchestrator/main.py`, after `apply_param_overrides`**

```python
        # A pack swapped without updating the config and an uncalibrated power
        # module look the same from here, and both make every mAh number
        # fiction. Say so once, loudly, rather than silently trusting it.
        try:
            fc_capacity = float(await commander.get_param_float("BAT1_CAPACITY"))
        except Exception:  # noqa: BLE001 — never block a launch on a param read
            fc_capacity = -1.0
        if fc_capacity <= 0:
            state.record_anomaly(
                "BAT1_CAPACITY not set on the FC — battery percentage and the "
                "energy budget are estimates until the power module is calibrated")
        elif abs(fc_capacity - energy_policy.capacity_mah) > 100.0:
            state.record_anomaly(
                f"battery capacity mismatch: FC says {fc_capacity:.0f} mAh, "
                f"config says {energy_policy.capacity_mah:.0f} mAh")
```

- [ ] **Step 6: Apply the SITL battery params — `orchestrator/main.py`, right after the `px4_tuning` apply**

```python
        # SITL only; harmless no-op on hardware where these params do not exist.
        sim_bat = cfg.get("sim_battery") or {}
        if sim_bat:
            await commander.apply_param_overrides(
                {k: float(v) for k, v in sim_bat.items()})
```

- [ ] **Step 7: Run the tests**

Run: `make test && make lint`
Expected: PASS, 266 passed; lint clean

- [ ] **Step 8: Commit**

```bash
git add sitl/aavc_config.yaml orchestrator/main.py tests/test_config.py
git commit -m "feat(energy): pack config, FC capacity cross-check, SITL battery sim"
```

---

### Task 4: Preflight row + GO gate with a working FORCE

**Files:**
- Modify: `orchestrator/preflight.py`
- Modify: `orchestrator/main.py` (the `_evaluate` closure)
- Modify: `dashboard/commands.py:527` area
- Modify: `dashboard/payloads.py`
- Test: `tests/test_preflight.py`, `tests/test_sortie_gate.py`

**Interfaces:**
- Consumes: `EnergyPolicy`, `energy_consumed_mah`, `state.sortie_energy_mah`.
- Produces: an advisory `energy` row in the preflight report; `state.sortie_energy_ok` honoured by `POST /api/cmd/preflight/go`.

- [ ] **Step 1: Write the failing tests**

In `tests/test_preflight.py`:

```python
def test_energy_row_is_advisory_so_force_is_never_a_dead_path():
    # A critical row would make preflight_can_go false, and the GO endpoint
    # checks that WITHOUT a force escape — the 2026-07-15 dead-path bug.
    state = _state_ready()
    report = preflight.run_preflight(state, **_kwargs())
    row = next(i for i in report.items if i.id == "energy")
    assert row.critical is False
```

In `tests/test_sortie_gate.py`:

```python
def test_go_is_blocked_when_the_pack_cannot_cover_the_sortie():
    state = _gate_ready_state()
    state.sortie_energy_ok = False
    resp = _post_go(state, force=False)
    assert resp.status_code == 409
    assert "swap" in resp.json()["detail"].lower()


def test_force_releases_a_gate_blocked_only_on_energy():
    state = _gate_ready_state()
    state.sortie_energy_ok = False
    resp = _post_go(state, force=True)
    assert resp.status_code == 200
    assert resp.json()["go"] is True
```

- [ ] **Step 2: Run them**

Run: `env -u PYTHONPATH .venv/bin/python -m pytest tests/test_preflight.py tests/test_sortie_gate.py -q`
Expected: FAIL — no `energy` row; GO ignores `sortie_energy_ok`

- [ ] **Step 3: Add the advisory row — `orchestrator/preflight.py`, after the `time` row**

```python
    # ADVISORY for the same reason the window row is: the refusal lives in
    # `state.sortie_energy_ok` and the GO endpoint's `sortie_energy_ok || force`.
    # A critical row here would make FORCE a dead path — the board could never be
    # green exactly when force is needed.
    add("energy", "Battery energy budget",
        PASS if state.sortie_energy_ok else WARN, False,
        state.energy_detail or "not evaluated")
```

with `energy_detail: str = field(default="", compare=False, repr=False)` added to `OrchestratorState`.

- [ ] **Step 4: Evaluate it — `orchestrator/main.py` `_evaluate()`, beside the `sortie_time_ok` block**

```python
        consumed, tier = energy_consumed_mah(state.telemetry,
                                             energy_policy.capacity_mah)
        state.energy_tier = tier
        state.sortie_energy_ok, state.energy_detail = (
            energy_policy.can_start_sortie(consumed - state.energy_baseline_mah
                                           if not math.isnan(consumed) else consumed,
                                           state.sortie_energy_mah))
```

- [ ] **Step 5: Gate GO — `dashboard/commands.py`, immediately after the `sortie_time_ok` check**

```python
        if not state.sortie_energy_ok and not req.force:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="the pack can't cover another sortie — swap the battery, "
                       "or tick FORCE to launch on what is left",
            )
```

and add `"sortie_energy_ok": state.sortie_energy_ok,` to the preflight GET payload beside `sortie_time_ok`, plus `sortie_energy_ok: bool = True` to `dashboard/payloads.py::TelemetryFrame`.

- [ ] **Step 6: Run the tests**

Run: `make test && make lint`
Expected: PASS; lint clean

- [ ] **Step 7: Commit**

```bash
git add orchestrator/preflight.py orchestrator/main.py orchestrator/state.py dashboard/commands.py dashboard/payloads.py tests/
git commit -m "feat(energy): advisory preflight row + GO gate with a working FORCE"
```

---

### Task 5: Per-sortie accounting + battery-swap detection

**Files:**
- Modify: `orchestrator/mission.py`
- Test: `tests/test_energy_policy.py` (append)

**Interfaces:**
- Consumes: `energy_consumed_mah`, state fields.
- Produces: `state.sortie_energy_mah` grows one entry per completed sortie; `BATTERY SWAP sortie=n` audit lines.

- [ ] **Step 1: Write the failing test**

```python
from orchestrator.energy_policy import detect_battery_swap


def test_swap_detected_when_the_coulomb_count_resets():
    assert detect_battery_swap(prev_mah=4200.0, now_mah=12.0,
                               prev_pct=30.0, now_pct=98.0)


def test_swap_detected_on_a_large_percent_jump_alone():
    assert detect_battery_swap(prev_mah=float("nan"), now_mah=float("nan"),
                               prev_pct=28.0, now_pct=95.0)


def test_normal_discharge_is_not_a_swap():
    assert not detect_battery_swap(prev_mah=1000.0, now_mah=1400.0,
                                   prev_pct=80.0, now_pct=74.0)
```

- [ ] **Step 2: Run it** — FAIL, `cannot import name 'detect_battery_swap'`

- [ ] **Step 3: Implement — append to `orchestrator/energy_policy.py`**

```python
#: A pack swap shows up as the charge going UP. Normal flight never does that,
#: so the threshold only has to clear sensor noise, not a real discharge.
SWAP_PERCENT_JUMP = 15.0


def detect_battery_swap(prev_mah: float, now_mah: float,
                        prev_pct: float, now_pct: float) -> bool:
    """Whether the pack was changed between two readings.

    Only meaningful while landed and disarmed at L&R — the one moment the crew
    touches the aircraft. Either signal is enough: the coulomb count restarting
    near zero, or the charge jumping up by more than noise.
    """
    if not math.isnan(prev_mah) and not math.isnan(now_mah) and prev_mah > 100.0:
        if now_mah < prev_mah * 0.5:
            return True
    if not math.isnan(prev_pct) and not math.isnan(now_pct):
        if now_pct - prev_pct > SWAP_PERCENT_JUMP:
            return True
    return False
```

- [ ] **Step 4: Wire it into the sortie loop — `orchestrator/mission.py`**

At sortie start, record the entry reading; at `SORTIE n END`, append the delta and check for a swap during the L&R hold:

```python
        consumed, _tier = energy_consumed_mah(state.telemetry,
                                              state.energy_capacity_mah)
        if not math.isnan(consumed) and not math.isnan(sortie_entry_mah):
            state.sortie_energy_mah.append(max(0.0, consumed - sortie_entry_mah))
        # The crew only reaches the aircraft while it is down and disarmed here.
        if detect_battery_swap(prev_mah=sortie_entry_mah, now_mah=consumed,
                               prev_pct=entry_pct,
                               now_pct=state.telemetry.battery_percent):
            state.energy_baseline_mah = 0.0
            audit(f"BATTERY SWAP sortie={sortie}")
```

- [ ] **Step 5: Run the tests** — `make test` → PASS

- [ ] **Step 6: Commit**

```bash
git add orchestrator/energy_policy.py orchestrator/mission.py tests/test_energy_policy.py
git commit -m "feat(energy): per-sortie accounting and battery-swap detection"
```

---

### Task 6: GCS battery panel

**Files:**
- Modify: `dashboard/realtime.py`
- Modify: `dashboard/payloads.py`
- Modify: `dashboard/web/src/lib/types.ts`
- Modify: `dashboard/web/src/widgets/TelemetrySidebar.svelte`

**Interfaces:**
- Consumes: `sortie_energy_ok`, `energy_tier`, `battery_current_a`, `energy_capacity_mah`.
- Produces: the operator-facing panel. No further consumers.

- [ ] **Step 1: Add the wire fields — `dashboard/payloads.py::TelemetryFrame`**

```python
    battery_current_a: float | None = None
    battery_capacity_mah: float | None = None
    energy_tier: str = "none"          # "A" coulomb count | "B" percent estimate
    energy_sorties_left: float | None = None
```

and fill them in `dashboard/realtime.py::_telemetry_frame`:

```python
            battery_current_a=_nf(t.battery_current_a),
            battery_capacity_mah=self.state.energy_capacity_mah or None,
            energy_tier=self.state.energy_tier,
            energy_sorties_left=self.state.energy_sorties_left,
```

- [ ] **Step 2: Mirror them in `dashboard/web/src/lib/types.ts`**

```typescript
  battery_current_a: number | null;
  battery_capacity_mah: number | null;
  energy_tier: string;            // "A" measured | "B" estimated | "none"
  energy_sorties_left: number | null;
```

- [ ] **Step 3: Extend the battery block — `TelemetrySidebar.svelte`**

```svelte
  <div class="py-2">
    <div class="flex items-center justify-between">
      <span class="aavc-readout-label">battery</span>
      <span class="font-mono text-[10px]"
            style="color: var(--color-aavc-ink-dim);"
            title={t?.energy_tier === 'A'
              ? 'measured by the power module'
              : 'estimated from percentage — coarse'}>
        {t?.energy_tier === 'A' ? '● measured' : t?.energy_tier === 'B' ? '○ estimated' : '— no data'}
      </span>
    </div>
    {#if t?.battery_capacity_mah}
      <div class="font-mono text-xs" style="color: var(--color-aavc-ink-2);">
        {fmtInt(t.battery_consumed_mah ?? 0)} / {fmtInt(t.battery_capacity_mah)} mAh
      </div>
    {/if}
    {#if t?.battery_current_a != null && t?.battery_voltage_v != null}
      <div class="font-mono text-xs" style="color: var(--color-aavc-ink-2);">
        {fmtNum(t.battery_current_a * t.battery_voltage_v, 0)} W
      </div>
    {/if}
    {#if t?.energy_sorties_left != null}
      <div class="font-mono text-xs"
           class:aavc-chip-critical={t.energy_sorties_left < 1}
           style="color: var(--color-aavc-ink);">
        {t.energy_sorties_left < 1
          ? '⚡ SWAP BATTERY BEFORE NEXT GO'
          : `energy for ${fmtNum(t.energy_sorties_left, 1)} more sorties`}
      </div>
    {/if}
  </div>
```

- [ ] **Step 4: Build and verify**

Run: `make web-build && make test && make lint`
Expected: build succeeds; 268 passed; lint clean

- [ ] **Step 5: Commit**

```bash
git add dashboard/ tests/
git commit -m "feat(gcs): battery capacity, power draw and sorties-left panel"
```

---

### Task 7: End-to-end SITL validation

**Files:** none modified — this task proves the chain.

- [ ] **Step 1: Launch SITL and confirm the pack now drains**

```bash
pkill -9 -f 'gz[ ]sim'; rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
HEADLESS=1 bash sitl/launch_sitl.sh > /tmp/aavc_sitl.log 2>&1 &
env -u PYTHONPATH .venv/bin/python sitl/wait_sitl_ready.py --timeout 180
```

Then fly the mission and watch `battery_remaining` fall — before this change it sat at 100 % forever.

- [ ] **Step 2: Run a 4-sortie mission**

```bash
make spawn-targets
make camera-bridge &
env -u PYTHONPATH .venv/bin/python -m orchestrator.main --config sitl/aavc_config.yaml \
    --truth-json /tmp/aavc_targets.json --assigned-ids "3,1,4,6" --no-dashboard --skip-preflight
```

Expected: the run completes; `runs/aavc_delivery_mission/audit.jsonl` contains per-sortie energy and the mission log shows the energy detail changing as charge drops.

- [ ] **Step 3: Verify the gate actually blocks**

With the dashboard running, set `SIM_BAT_MIN_PCT` low enough that the pack empties mid-mission and confirm the sortie GO returns 409 with the swap message, then that `force: true` releases it.

- [ ] **Step 4: Commit the evidence**

```bash
cp /tmp/aavc_sitl.log docs/evidence/energy-budget-sitl-$(date +%F).log
git add docs/evidence/
git commit -m "test(energy): SITL evidence for the energy budget chain"
```

---

## Task 7 result — SITL validation (2026-07-22)

**What the chain proved.** With `sim_battery` applied, the simulated pack drains
for the first time (99 % → 98 % in 12 s; before this change it sat at 100 % and
16.2 V for an entire flight). `current_consumed` stays negative in SITL, so
tier **B** is the live path — exactly the fallback the design exists for. The
mission recorded `SORTIE 1 ENERGY 1500mAh total=1500mAh` in the audit trail, and
the four-sortie run passed `verify_flight` 19/0 with the energy code in place.

**What SITL cannot prove: cumulative discharge across sorties.** PX4's battery
simulator **recharges on disarm** — the pack read 100 % immediately after a
15-minute four-sortie mission. Sorties 2 and 3 therefore measured a *negative*
cost and were correctly skipped by the `_cost > 0` guard rather than polluting
the history with nonsense.

So the pack-running-empty path — the gate refusing a sortie, the swap prompt
appearing — cannot be produced by flying in SITL. It is covered by
`tests/test_energy_policy.py` (the boundary, the refusal text, and that FORCE
still releases a gate blocked only on energy), and it will be exercisable for
real at G5 once the PM03D is calibrated and tier A goes live.

Do not "fix" this by making the simulator drain harder: it recharges regardless,
so the only thing a faster drain buys is a bigger jump at each disarm.
