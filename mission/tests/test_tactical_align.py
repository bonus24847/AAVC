"""Land-ON-pad terminal controller (orchestrator.tactical_align) — V1.3.

The egg is committed the moment the vehicle lands, so these lock the guards
around that commitment: the per-frame detector filter (assigned id wins,
wrong id NEVER steers, cue blobs only as positional fallback), the id-verified
LAND gate (no decode → no landing → defer), and the touchdown-gated release
(payload_id always 0; keep the egg when telemetry reads airborne).

The full descend-gate behaviour needs camera frames + a flying vehicle — SITL
(G4) validates that end-to-end; here a fake commander + patched detector lock
the decision logic.
"""

from __future__ import annotations

import asyncio
import itertools
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from mavlink_adapter.telemetry import CurrentTelemetry
from mission_brain.live_plan import render_live_plan
from mission_brain.profile import COMPETITION
from mission_brain.schemas import Coordinate
from mission_brain.search_pattern import build_search_pattern
from orchestrator import tactical_align as ta
from orchestrator.state import OrchestratorMode, OrchestratorState
from orchestrator.tactical_align import AlignParams, _drop_once, acquire_and_land_drop
from vision.detectors.aruco import PadHit, render_pad_bgr

_LAT, _LON = 13.7308, 100.7886
_AREA = [[_LAT - 3e-4, _LON - 3e-4], [_LAT - 3e-4, _LON + 3e-4],
         [_LAT + 3e-4, _LON + 3e-4], [_LAT + 3e-4, _LON - 3e-4]]


# ── _detect_nadir: the per-frame acceptance filter ──

def _frame_with_pads(tmp_path, *pads):
    """Write a nadir frame containing the given (marker_id, cx, cy, pad_px)."""
    img = np.full((960, 1280, 3), (60, 120, 70), np.uint8)
    for mid, cx, cy, px in pads:
        pad = cv2.resize(render_pad_bgr(mid, 512), (px, px),
                         interpolation=cv2.INTER_AREA)
        img[cy - px // 2:cy - px // 2 + px, cx - px // 2:cx - px // 2 + px] = pad
    p = tmp_path / "nadir.png"
    cv2.imwrite(str(p), img)
    return p


def test_detect_nadir_prefers_assigned_id(tmp_path) -> None:
    # Assigned pad 3 + neighbour pad 5, both decodable.
    f = _frame_with_pads(tmp_path, (5, 300, 300, 90), (3, 800, 600, 90))
    hit = ta._detect_nadir(f, 0.45, assigned_id=3)
    assert hit is not None and hit.marker_id == 3
    assert abs(hit.cx - 800) <= 3 and abs(hit.cy - 600) <= 3


def test_detect_nadir_never_steers_by_wrong_id(tmp_path) -> None:
    # ONLY the wrong pad is visible — must return nothing, not "close enough".
    f = _frame_with_pads(tmp_path, (5, 640, 480, 90))
    assert ta._detect_nadir(f, 0.45, assigned_id=3) is None


def test_detect_nadir_accepts_cue_blob_as_positional_fallback(tmp_path) -> None:
    # A pad too blurred to decode still centres the descent (id gate handles
    # the commitment later).
    img = np.full((960, 1280, 3), (60, 120, 70), np.uint8)
    pad = cv2.resize(render_pad_bgr(2, 512), (45, 45), interpolation=cv2.INTER_AREA)
    img[458:503, 618:663] = pad
    img = cv2.GaussianBlur(img, (7, 7), 0)
    p = tmp_path / "nadir.png"
    cv2.imwrite(str(p), img)
    hit = ta._detect_nadir(p, 0.45, assigned_id=3)
    assert hit is not None and hit.marker_id is None


# ── acquire_and_land_drop: the commitment logic (fake commander + patched detector) ──

class FakeCommander:
    def __init__(self, state: OrchestratorState) -> None:
        self.state = state
        self.landed_calls: list[bool] = []
        self.released: list[int] = []
        self.params: dict[str, float] = {}

    async def goto(self, lat, lon, alt_m, yaw_deg=float("nan")) -> None:
        t = self.state.telemetry
        t.lat, t.lon, t.relative_alt_m = lat, lon, alt_m

    async def land(self, *, disarm: bool = True) -> None:
        self.landed_calls.append(disarm)
        self.state.telemetry.relative_alt_m = 0.0
        # the release gate keys on PX4's land detector, not altitude
        # (2026-08-13 mid-air release fix) — a normal landing reports it
        self.state.telemetry.landed_state = "ON_GROUND"

    async def drop_payload(self, payload_id: int = 0) -> None:
        self.released.append(payload_id)

    async def set_param_float(self, name: str, value: float) -> None:
        self.params[name] = value


def _state() -> OrchestratorState:
    home = Coordinate(lat=_LAT, lon=_LON)
    spec = build_search_pattern(_AREA, home, sweep_alt_m=12.0)
    plan = render_live_plan(home, spec, discovered=[], profile=COMPETITION)
    telem = CurrentTelemetry()
    telem.lat, telem.lon, telem.relative_alt_m = _LAT, _LON, 12.0
    telem.heading_deg = telem.roll_deg = telem.pitch_deg = 0.0
    telem.ground_speed_mps = 0.0
    return OrchestratorState(mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=telem)


def _fast_params(**overrides) -> AlignParams:
    kw = dict(
        rungs=(12.0, 5.0), rung_tol_m=(1.5, 0.4), rung_descent_mps=(3.0, 1.0),
        lock_cycles=1, cycle_hz=200.0, acquire_timeout_s=0.2, rung_timeout_s=0.3,
        median_window=1, settle_after_land_s=0.0, assigned_marker_id=3,
        # These tests patch _detect_nadir and rely on the default /tmp frame path
        # (which may hold a stale frame from a prior SITL run); disable the S2
        # freshness gate so they exercise align logic deterministically.
        frame_max_age_s=0.0,
    )
    kw.update(overrides)
    return AlignParams(**kw)


def _centred_hit(marker_id) -> PadHit:
    # Centred in the DEFAULT 640x480 projection model; radius ≈ expected for
    # the 0.2 m marker so the size prior passes at any rung altitude.
    return PadHit(cx=320, cy=240, marker_id=marker_id, radius_px=0.0,
                  confidence=0.9, corners=(), pad_side_px=0.0)


def _patch_live_camera(monkeypatch) -> None:
    """A faked detector is a faked camera — make it a LIVE one.

    Since 2026-08-21 the loop only decodes frames it has not seen, so a frozen
    mtime means one decode and then nothing: that is the frozen-camera case,
    not the flying one. Every test that fakes _detect_nadir must fake this too."""
    ticks = itertools.count(1.0, 0.05)
    monkeypatch.setattr(ta, "_frame_mtime", lambda _p: next(ticks))


def _patch_detector(monkeypatch, marker_id) -> None:
    def fake(frame_path, min_conf, assigned_id):
        alt = max(state_ref.telemetry.relative_alt_m, 0.5)
        exp = 423.1 * 0.2 / alt          # fx(640px,1.295rad) * R / slant
        hit = _centred_hit(marker_id)
        return PadHit(cx=hit.cx, cy=hit.cy, marker_id=hit.marker_id,
                      radius_px=exp, confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)


state_ref: OrchestratorState


def test_lands_on_pad_and_releases_after_touchdown(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    _patch_detector(monkeypatch, marker_id=3)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))

    assert res.acquired and res.landed and res.dropped
    assert cmd.landed_calls == [False]        # touch down ARMED (no disarm)
    assert cmd.released == [0]                # the egg servo is ALWAYS payload 0
    assert state.dropped_stops == {1}         # ledger keyed by sortie index
    assert not math.isnan(res.final_error_m)


def test_id_gate_refuses_to_land_without_a_decode(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    _patch_detector(monkeypatch, marker_id=None)   # cue blobs only, never decoded

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))

    assert res.acquired                        # the blob did centre the descent
    assert not res.landed and not res.dropped  # ...but the egg was NOT committed
    assert cmd.landed_calls == []              # LAND never commanded
    assert "id-not-confirmed → defer" in res.notes
    assert any("land_gate_id_not_confirmed" in a for a in state.anomalies)
    assert state.telemetry.relative_alt_m == 12.0   # climbed back to the top rung


def test_wrong_pad_never_acquires_and_defers(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    # _detect_nadir itself rejects wrong ids → the loop sees nothing at all.
    monkeypatch.setattr(ta, "_detect_nadir", lambda f, c, a: None)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 2, params=_fast_params()))

    assert not res.acquired and not res.landed and not res.dropped
    assert cmd.landed_calls == [] and cmd.released == []
    assert "acquire-timeout: deferred" in res.notes   # gps_fallback=False default


def test_uncentred_final_rung_defers_instead_of_landing(monkeypatch) -> None:
    """Centred-LAND gate (found by the 2026-07-15 GCS run): a biased fix streak
    kept the vehicle ~0.9 m off the pad — every frame decodes the ASSIGNED id
    (so the id gate passes) but the fix always projects 0.9 m away, so the
    final rung can never lock. The old code fell through the rung timeouts and
    LANDED anyway (2.46 m off-pad in flight — forfeits the landed-on-pad-
    before-release scoring line). It must defer like the id gate instead."""
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)

    def biased(frame_path, min_conf, assigned_id):
        # Offset the hit so the projected fix is ~0.9 m from the vehicle at any
        # altitude: locks the 1.5 m top rung, can NEVER lock the 0.4 m final.
        alt = max(state_ref.telemetry.relative_alt_m, 0.5)
        dx = 271.4 * 0.9 / alt
        exp = 271.4 * 0.2 / alt
        return PadHit(cx=int(320 + dx), cy=240, marker_id=3, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)

    monkeypatch.setattr(ta, "_detect_nadir", biased)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))

    assert res.acquired                        # the pad WAS seen + id decoded
    assert not res.landed and not res.dropped  # ...but the egg was NOT committed
    assert cmd.landed_calls == []              # LAND never commanded off-centre
    assert any("not-centred" in n for n in res.notes)
    assert any("land_gate_not_centred" in a for a in state.anomalies)
    assert state.telemetry.relative_alt_m == 12.0   # climbed back to defer


def test_release_skipped_when_telemetry_reads_airborne(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    _patch_detector(monkeypatch, marker_id=3)

    async def land_but_stay_high(*, disarm: bool = True) -> None:
        cmd.landed_calls.append(disarm)
        state.telemetry.relative_alt_m = 4.0   # "landed" never confirms

    async def no_touchdown(st, pred, timeout_s, poll_s=0.25) -> bool:
        return False                           # skip the real 40 s wait

    monkeypatch.setattr(cmd, "land", land_but_stay_high)
    monkeypatch.setattr(ta, "_wait_until", no_touchdown)
    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1,
        params=_fast_params()))

    assert not res.landed and not res.dropped
    assert cmd.released == []                  # egg kept — not dropped from 4 m
    assert any("release_skipped_touchdown_unconfirmed" in a for a in state.anomalies)


# ── _drop_once: per-flight payload_id + the DELIVERY audit grammar ──

def test_drop_once_uses_payload_id_and_delivery_grammar() -> None:
    state = _state()
    cmd = FakeCommander(state)
    ok = asyncio.run(_drop_once(cmd, state, stop_index=2, payload_id=2,
                                delivery_index=3, marker_id=4))
    assert ok is True
    assert cmd.released == [2]                 # channel 9+2 = 11
    line = next(a for a in state.anomalies if "DELIVERY 3 RELEASE" in a)
    assert "pad=4" in line and "payload=2" in line


def test_drop_once_is_idempotent_per_stop_index() -> None:
    state = _state()
    cmd = FakeCommander(state)
    assert asyncio.run(_drop_once(cmd, state, stop_index=0, payload_id=0,
                                  delivery_index=1, marker_id=3)) is True
    assert asyncio.run(_drop_once(cmd, state, stop_index=0, payload_id=0,
                                  delivery_index=1, marker_id=3)) is False
    assert cmd.released == [0]


class FakeCommanderNoLandDetect(FakeCommander):
    """land() drops the altitude but the landed_state stream NEVER reports —
    the vehicle that motivated the 2026-08-13 fix (alt<=threshold fired while
    still sinking ~1 m above the pad; the box fell mid-air on camera)."""

    async def land(self, *, disarm: bool = True) -> None:
        self.landed_calls.append(disarm)
        self.state.telemetry.relative_alt_m = 0.9   # below the 1.5 m threshold
        # landed_state stays "UNKNOWN" — no detector verdict


def test_release_gates_on_the_land_detector_not_altitude(monkeypatch) -> None:
    # Without ON_GROUND the release may only happen via the audited
    # last-resort alt fallback — never straight from the alt threshold.
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommanderNoLandDetect(state)
    _patch_detector(monkeypatch, marker_id=3)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1,
        params=_fast_params(touchdown_timeout_s=0.3)))

    assert res.dropped and res.landed              # fallback still delivers…
    assert any("landed_state_timeout_alt_fallback" in a
               for a in state.anomalies)           # …but ALWAYS audited


def test_release_fires_once_the_detector_reports_on_ground(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)                     # land() reports ON_GROUND
    _patch_detector(monkeypatch, marker_id=3)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))

    assert res.landed and res.dropped
    assert not any("landed_state_timeout" in a for a in state.anomalies)


# ── the landing loop's rate trio is ONE setting (2026-08-21) ────────────────
# lock_cycles and max_lost_cycles are counted in CYCLES, so raising cycle_hz
# alone silently shortens how long the loop confirms a lock and how long it
# tolerates a lost pad — it gets twitchy exactly when the camera is struggling,
# which is the opposite of what a faster loop is for. The rate went 5 -> 10 Hz
# when JPEG frames cut a cycle to ~56 ms of CM4 CPU; these wall-clock constants
# are what the validated 5 Hz/3/8 era flew and must survive any retune.


def _wall_clock(p: AlignParams) -> tuple[float, float, float]:
    return (p.lock_cycles / p.cycle_hz,
            p.max_lost_cycles / p.cycle_hz,
            p.median_window / p.cycle_hz)


def test_align_rate_change_preserves_the_wall_clock_constants() -> None:
    lock_s, lost_s, median_lag_s = _wall_clock(AlignParams())
    assert lock_s == pytest.approx(0.75, abs=0.1)
    assert lost_s == pytest.approx(2.0, abs=0.2)
    # the median filter's own lag rides into the commanded setpoint, so it IS
    # landing error — a faster loop must SHRINK it, never grow it
    assert median_lag_s <= 0.3


def test_align_loop_is_not_slower_than_the_camera() -> None:
    """A loop slower than the camera throws frames away, and one faster than
    it can actually run just slips. The camera writes 20-25 Hz with MJPEG
    passthrough; a cycle costs ~60 ms on the CM4, so the honest ceiling is
    ~16 Hz and the setting must leave headroom under it."""
    assert 10.0 <= AlignParams().cycle_hz <= 15.0


def test_align_config_block_moves_the_trio_together() -> None:
    """The config seam exists so landing precision can be A/B'd with
    tools/landing_trial.py without editing the flight core — but a config that
    changes the rate alone would break the invariant above, so the shipped
    block sets all three."""
    import yaml

    from orchestrator.main import _align_for, _align_tuning

    cfg = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "sitl" / "aavc_config.yaml").read_text())
    tuning = _align_tuning(cfg)
    assert {"cycle_hz", "lock_cycles", "max_lost_cycles"} <= set(tuning)
    lock_s, lost_s, _ = _wall_clock(_align_for(COMPETITION, 2.0, tuning))
    assert lock_s == pytest.approx(0.75, abs=0.1)
    assert lost_s == pytest.approx(2.0, abs=0.2)


# ── the landing loop must not re-decode a frame it has already seen ─────────
# (2026-08-21 review) The pose is sampled per read, so re-decoding one image
# yields a SEQUENCE of world fixes that translate WITH the aircraft: the
# commanded goto then chases the vehicle's own motion instead of correcting
# it, and lock_cycles can be satisfied from a handful of distinct frames.


def test_a_frozen_frame_is_decoded_once_not_every_cycle(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    calls = []

    def counting(frame_path, min_conf, assigned_id):
        calls.append(1)
        return _centred_hit(3)

    monkeypatch.setattr(ta, "_detect_nadir", counting)
    monkeypatch.setattr(ta, "_frame_mtime", lambda _p: 7.0)   # camera frozen
    asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))

    assert len(calls) == 1, f"decoded a frozen frame {len(calls)} times"


def test_a_missing_frame_never_decodes(monkeypatch) -> None:
    """No camera at all must not spin the decode path — the staleness gate and
    the lost-detection counter own that case."""
    global state_ref
    state = state_ref = _state()
    cmd = FakeCommander(state)
    calls = []
    monkeypatch.setattr(ta, "_detect_nadir",
                        lambda f, c, a: (calls.append(1), None)[1])
    monkeypatch.setattr(ta, "_frame_mtime", lambda _p: None)
    asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1, params=_fast_params()))
    assert not calls


def test_the_pose_belongs_to_the_frame_not_to_the_decode(monkeypatch) -> None:
    """_hit_world_fix must project with the pose passed IN, so a pose captured
    before a ~55 ms decode is the one used — a live read afterwards is a bias
    in the direction of travel, which the median filter cannot remove."""
    hit = _centred_hit(3)
    pose_a = (_LAT, _LON, 10.0, 0.0, 0.0, 0.0)
    pose_b = (_LAT + 0.001, _LON, 10.0, 0.0, 0.0, 0.0)   # ~111 m north
    fix_a = ta._hit_world_fix(hit, pose_a)
    fix_b = ta._hit_world_fix(hit, pose_b)
    assert fix_a is not None and fix_b is not None
    assert abs(fix_b.lat - fix_a.lat) > 0.0009        # it really used the pose
    assert ta._hit_world_fix(hit, None) is None


# ── rung ALTITUDE gate (2026-08-28, KMITL 17:28 flight) ──────────────────────

class LaggingCommander(FakeCommander):
    """Records the commanded altitude; the airframe sinks toward it 1 m per
    loop cycle (see _patch_detector_lagging) — the real bird's 0.4-3 m/s
    against a 12 Hz loop, not the instant arrival the base fake models. The
    align issues ONE goto per rung and lets PX4 fly to it, so the lag has to
    live in the per-cycle telemetry, not in goto()."""

    def __init__(self, state: OrchestratorState) -> None:
        super().__init__(state)
        self.target_alt: float | None = None
        self.alt_at_land: float | None = None

    async def goto(self, lat, lon, alt_m, yaw_deg=float("nan")) -> None:
        t = self.state.telemetry
        t.lat, t.lon = lat, lon
        self.target_alt = alt_m

    async def land(self, *, disarm: bool = True) -> None:
        self.alt_at_land = self.state.telemetry.relative_alt_m
        await super().land(disarm=disarm)


def _patch_detector_lagging(monkeypatch, cmd: LaggingCommander, marker_id,
                            step_m: float = 1.0) -> list[float]:
    """Centred pad every cycle; the altitude moves ≤ step_m toward the last
    commanded altitude per cycle. Returns the per-cycle altitude trace."""
    trace: list[float] = []

    def fake(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - step_m)
        alt = max(t.relative_alt_m, 0.5)
        trace.append(alt)
        exp = 423.1 * 0.2 / alt          # fx(640px,1.295rad) * R / slant
        return PadHit(cx=320, cy=240, marker_id=marker_id, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)
    return trace


def test_a_rung_is_not_done_until_the_aircraft_is_at_its_altitude(monkeypatch) -> None:
    """Pad centred from the first frame (as after a sweep fix) — the OLD loop
    locked the 5 m rung on the first cycle at 11 m and commanded LAND from
    there (the 17:28 flight: LAND from 4.8 / 8.5 m, eggs 0.5-0.7 m off).
    Now the rung waits for the aircraft to actually arrive."""
    global state_ref
    state = state_ref = _state()
    cmd = LaggingCommander(state)
    trace = _patch_detector_lagging(monkeypatch, cmd, 3)
    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=2.0)))
    assert res.landed and cmd.landed_calls
    assert cmd.alt_at_land is not None and cmd.alt_at_land <= 5.0 + 0.6, (
        f"LAND commanded at {cmd.alt_at_land} m — the 5 m rung was not reached")
    # Seven 1 m steps from 12 to 5 m: the rung took several cycles, not one.
    assert sum(1 for a in trace if 5.0 < a < 12.0) >= 6, trace
    assert not any("alt_unverified" in a for a in state.anomalies)


class StuckCommander(FakeCommander):
    """Position holds but the altitude never changes (a frozen baro/EKF or a
    goto PX4 will not follow) — the gate must not turn that into a deferral."""

    async def goto(self, lat, lon, alt_m, yaw_deg=float("nan")) -> None:
        t = self.state.telemetry
        t.lat, t.lon = lat, lon            # altitude stays where it was


def test_an_unverifiable_rung_altitude_falls_back_to_the_old_rule(monkeypatch) -> None:
    global state_ref
    state = state_ref = _state()                # parked at 12 m for good
    cmd = StuckCommander(state)
    _patch_detector(monkeypatch, 3)
    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=0.3)))
    assert res.landed and cmd.landed_calls, (
        "centred pad + unverifiable altitude must still land (old rule)")
    assert any("rung5m_alt_unverified_fallback" in a for a in state.anomalies), state.anomalies


def test_a_low_reading_height_frame_is_corrected_to_the_true_rung(monkeypatch) -> None:
    """17:28 flight: the aircraft's AGL read 0.4-1.4 m LOW at the pads, so
    "descend to 2 m" parked it at a true ~3 m. The rung goto must be
    corrected by the measured (AGL − marker altitude) so the TRUE height
    reaches the rung — quickly, without the 18 s fallback."""
    global state_ref
    state = state_ref = _state()
    cmd = LaggingCommander(state)
    BIAS = 1.0                                  # true height = AGL + 1.0
    trace: list[float] = []

    def fake(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - 1.0)
        true_alt = max(t.relative_alt_m + BIAS, 0.5)
        trace.append(true_alt)
        exp = 423.1 * 0.2 / true_alt            # the marker's size is the TRUTH
        return PadHit(cx=320, cy=240, marker_id=3, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=2.0)))
    assert res.landed and cmd.landed_calls
    # LAND was commanded with the TRUE height at the 5 m rung — i.e. the
    # aircraft's own frame read ~4 m — not parked at a true 6 m.
    assert cmd.alt_at_land is not None and cmd.alt_at_land + BIAS <= 5.0 + 0.6, (
        f"LAND at a true {cmd.alt_at_land + BIAS:.1f} m — the frame bias was not corrected")
    assert not any("alt_unverified" in a for a in state.anomalies), state.anomalies


def test_a_high_reading_height_frame_is_corrected_to_the_true_rung(monkeypatch) -> None:
    """2026-08-29, KMITL scored flight 1 (ULog ``2026-08-29/05_56_59``): the
    aircraft's AGL frame read 2.4 m HIGH over pad 5 — GPS altitude drifted
    +8.96 m ground-to-ground while the baro moved +0.34, and the height blend
    followed it. The bias correction saw all 2.4 m but the old ±1.5 m clamp
    threw a metre of it away, so the 3 m rung flew at a true 2.1 m, the gate
    (correctly) refused it, and the ladder's fallback pushed on to a true
    ~1.1 m where the 400 mm marker leaves the frame: pad lost, no LAND, no
    release, pilot kill. The clamp must pass a bias this large."""
    global state_ref
    state = state_ref = _state()
    cmd = LaggingCommander(state)
    BIAS = -2.4                                 # true height = AGL - 2.4

    def fake(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        # A frame that reads HIGH means the first goto is UPWARD (rung + bias),
        # so the airframe has to track the setpoint in both directions here —
        # the descend-only lag of the low-bias twin would strand it.
        if cmd.target_alt is not None:
            step = cmd.target_alt - t.relative_alt_m
            t.relative_alt_m += max(-1.0, min(1.0, step))
        true_alt = max(t.relative_alt_m + BIAS, 0.5)
        exp = 423.1 * 0.2 / true_alt            # the marker's size is the TRUTH
        return PadHit(cx=320, cy=240, marker_id=3, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)
    # Assert on what was COMMANDED, not on the telemetry at the LAND call: the
    # fake sinks 1 m per cycle, so the aircraft passes THROUGH the right height
    # on its way to a wrong setpoint and a telemetry-sampled check cannot tell
    # the two apart (it read 5.1 m "true" with the bias clamped to 1.5).
    gotos: list[float] = []
    real_goto = cmd.goto

    async def spy_goto(lat, lon, alt_m, yaw_deg=float("nan")):
        gotos.append(alt_m)
        await real_goto(lat, lon, alt_m, yaw_deg)
    monkeypatch.setattr(cmd, "goto", spy_goto)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=2.0)))
    assert res.landed and cmd.landed_calls
    # The 5 m rung must be commanded at ~7.4 m in the aircraft's drifted frame
    # (5 + the full 2.4 m bias). A ±1.5 m clamp caps it at 6.5 = a true 4.1 m.
    rung5 = [g for g in gotos if g < 12.0]
    assert rung5, gotos
    assert max(rung5) >= 5.0 + 2.0, (
        f"the 5 m rung was commanded no higher than {max(rung5):.1f} m in a "
        f"frame reading 2.4 m HIGH — a true {max(rung5) + BIAS:.1f} m. The "
        f"frame bias was clamped away (gotos={gotos})")
    assert not any("alt_unverified" in a for a in state.anomalies), state.anomalies


def test_the_lidar_flies_the_rung_when_the_height_frame_is_further_out_than_the_clamp(
        monkeypatch) -> None:
    """The 2026-08-29 failure, made worse than it really was, with the lidar on.

    That flight's height frame read 2.4 m HIGH and the ±1.5 m clamp threw a
    metre of it away. Raising the clamp to 3.0 covers what was measured, but an
    open-loop correction can only ever apply its clamp once — so here the frame
    reads **4 m** high, past even the new clamp, and the ladder still has to
    reach the true rung. It can, because below rangefinder_max_alt_m it flies
    the rung CLOSED LOOP on the downward lidar: each command moves by the error
    the lidar measures, so an arbitrarily large frame bias is walked off over a
    few cycles instead of being clipped.
    """
    global state_ref
    state = state_ref = _state()
    cmd = LaggingCommander(state)
    BIAS = -4.0                                 # true height = AGL - 4.0
    gotos: list[float] = []
    real_goto = cmd.goto

    async def spy_goto(lat, lon, alt_m, yaw_deg=float("nan")):
        gotos.append(alt_m)
        await real_goto(lat, lon, alt_m, yaw_deg)
    monkeypatch.setattr(cmd, "goto", spy_goto)

    def fake(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None:
            step = cmd.target_alt - t.relative_alt_m
            t.relative_alt_m += max(-1.0, min(1.0, step))
        true_alt = max(t.relative_alt_m + BIAS, 0.5)
        # the lidar sees the TRUE height, in its own beam range
        t.rangefinder_m = true_alt
        t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0
        t.rangefinder_ts = time.monotonic()
        exp = 423.1 * 0.2 / true_alt
        return PadHit(cx=320, cy=240, marker_id=3, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=3.0)))
    assert res.landed and cmd.landed_calls
    # The 5 m rung has to be commanded at ~9 m in the aircraft's own frame —
    # 5 + the full 4 m bias. A clamp of any kind would stop short of that.
    rung5 = [g for g in gotos if g < 12.0]
    assert rung5, gotos
    assert max(rung5) >= 5.0 + 3.5, (
        f"the 5 m rung was commanded no higher than {max(rung5):.1f} m in a "
        f"frame reading 4 m HIGH — a true {max(rung5) + BIAS:.1f} m; the "
        f"closed loop did not walk off a bias bigger than the clamp "
        f"(gotos={gotos})")
    assert not any("alt_unverified" in a for a in state.anomalies), state.anomalies


def test_a_below_minimum_lidar_reading_is_not_treated_as_the_ground(monkeypatch) -> None:
    """The TFmini-S reports 0.00 m under its 0.4 m floor, and 0.00 is exactly
    what it read for the last 25 s of the 2026-08-29 flight with the aircraft
    sitting on the grass. Read literally that is "0 m above the ground" — which
    would tell the ladder it had arrived at every rung at once. It must be
    refused, leaving the marker-size path in charge."""
    t = CurrentTelemetry()
    t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0
    t.rangefinder_ts = time.monotonic()
    for reading, why in ((0.0, "below the beam minimum"),
                         (0.2, "below the beam minimum"),
                         (13.0, "past the beam maximum")):
        t.rangefinder_m = reading
        assert math.isnan(t.rangefinder_agl_m()), f"{reading} m accepted ({why})"
    t.rangefinder_m = 2.14
    assert t.rangefinder_agl_m() == 2.14
    t.rangefinder_ts = time.monotonic() - 5.0
    assert math.isnan(t.rangefinder_agl_m()), "a 5 s old reading is not current"


def _sized(marker_id, alt_m):
    """A centred hit whose marker size matches ``alt_m`` — the size prior gates
    on it, so a fixed radius is rejected at every altitude but one."""
    return PadHit(cx=320, cy=240, marker_id=marker_id,
                  radius_px=423.1 * 0.2 / max(alt_m, 0.5),
                  confidence=0.9, corners=(), pad_side_px=0.0)


class GroundedCommander(LaggingCommander):
    """Tracks land(); optionally makes PX4's land detector refuse forever."""

    def __init__(self, state: OrchestratorState) -> None:
        super().__init__(state)
        self.land_calls: list[float] = []
        #: when True the fake PX4 land detector NEVER latches — which is what
        #: the real one did for 226 s on 2026-08-29 with the aircraft on the
        #: grass, because it will not latch while the position controller is
        #: still demanding movement.
        self.refuse_land_detector = False

    async def land(self, *, disarm: bool = True) -> None:
        self.land_calls.append(self.state.telemetry.relative_alt_m)
        await super().land(disarm=disarm)
        if self.refuse_land_detector:
            t = self.state.telemetry
            t.landed_state = "IN_AIR"
            t.rangefinder_m, t.rangefinder_ts = 0.0, time.monotonic()


def test_ground_contact_stops_the_ladder_and_lands_instead_of_climbing(monkeypatch) -> None:
    """2026-08-29, the tip-over. The ladder walked the aircraft onto the grass
    while the height frame still read 2.4 m; PX4 stayed in AUTO.LOITER holding
    position, its land detector never latched, the commanded pitch wound up
    +1.4° → +20.3° over ~10 s at idle thrust, and the next climb-out levered
    the airframe onto its tail. The guard must see the ground on the lidar and
    command LAND — which ramps thrust down, and is the only state PX4's own
    detector will latch in — instead of leaving it pressed into the grass."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedCommander(state)
    state.telemetry.rangefinder_min_m, state.telemetry.rangefinder_max_m = 0.4, 12.0
    state.telemetry.rangefinder_m = 12.0
    state.telemetry.rangefinder_ts = time.monotonic()
    # the real sequence: the beam walks down through its range, then the
    # aircraft is ON the grass — 0.00 m (below the minimum) and no pad to see
    touchdown = iter([4.0, 3.0, 2.0, 1.2, 0.8, 0.5] + [0.0] * 200)

    def detect(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and cmd.target_alt < 8.0:
            v = next(touchdown)
            t.rangefinder_m, t.rangefinder_ts = v, time.monotonic()
            if v == 0.0:
                return None
        else:
            t.rangefinder_m, t.rangefinder_ts = 12.0, time.monotonic()
        return _sized(3, t.relative_alt_m)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=2.0, ground_contact_cycles=3)))
    assert any("ground_contact_at_rung" in a for a in state.anomalies), state.anomalies
    assert cmd.land_calls, "the guard must command LAND, not leave it in position hold"


def test_the_rangefinder_confirms_touchdown_when_px4s_land_detector_will_not(
        monkeypatch) -> None:
    """On 2026-08-29 `vehicle_land_detected.landed` stayed False for 226 s with
    the aircraft on the grass. The lidar had the answer the whole time
    (0.11 m). It must be believed BEFORE the altitude-threshold fallback, which
    reads the very frame that was 2.4 m wrong."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedCommander(state)
    cmd.refuse_land_detector = True
    state.telemetry.landed_state = "IN_AIR"
    state.telemetry.rangefinder_min_m, state.telemetry.rangefinder_max_m = 0.4, 12.0
    state.telemetry.rangefinder_m = 12.0
    state.telemetry.rangefinder_ts = time.monotonic()

    def detect(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None:
            step = cmd.target_alt - t.relative_alt_m
            t.relative_alt_m += max(-1.0, min(1.0, step))
        alt = max(t.relative_alt_m, 0.5)
        t.rangefinder_m, t.rangefinder_ts = alt, time.monotonic()
        return _sized(3, alt)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rungs=(12.0, 5.0, 2.0), rung_tol_m=(1.5, 0.6, 0.3),
                            rung_descent_mps=(3.0, 1.0, 0.5),
                            rung_timeout_s=3.0, touchdown_timeout_s=0.2,
                            ground_contact_cycles=3,
                            rangefinder_max_age_s=3600.0)))
    assert res.landed, "the rangefinder's ground contact must count as touchdown"
    assert any("touchdown_from_rangefinder" in a for a in state.anomalies), state.anomalies
    assert not any("alt_fallback" in a for a in state.anomalies), (
        "the lidar must be tried BEFORE the altitude threshold", state.anomalies)
    assert res.dropped, "and the egg goes out"


def test_a_0_00_reading_from_ABOVE_the_beam_is_not_ground_contact(monkeypatch) -> None:
    """The TFmini-S reports 0.00 past its 12 m maximum as well as under its
    0.4 m minimum. At sweep altitude that must never read as "on the ground" —
    the guard requires the beam to have just measured something within
    ground_contact_from_m."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedCommander(state)
    state.telemetry.rangefinder_min_m, state.telemetry.rangefinder_max_m = 0.4, 12.0

    def detect(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        t.rangefinder_m, t.rangefinder_ts = 0.0, time.monotonic()   # out of range: HIGH
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - 1.0)
        return _sized(3, t.relative_alt_m)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=0.4, ground_contact_cycles=3)))
    assert not any("ground_contact_at_rung" in a for a in state.anomalies), (
        "a 0.00 m never preceded by a close reading is out-of-range, not the ground",
        state.anomalies)


def test_undecoded_blob_hits_never_feed_the_frame_bias(monkeypatch) -> None:
    """A white-pad blob's marker-equivalent size is an inference; if it read
    2.5x too small the bias would steer the aircraft LOWER than it thinks.
    Blob-only frames must leave the bias at zero (gotos at the plain rung)."""
    global state_ref
    state = state_ref = _state()
    cmd = LaggingCommander(state)
    gotos: list[float] = []
    real_goto = cmd.goto

    async def spy_goto(lat, lon, alt_m, yaw_deg=float("nan")):
        gotos.append(alt_m)
        await real_goto(lat, lon, alt_m, yaw_deg)
    monkeypatch.setattr(cmd, "goto", spy_goto)

    def blob(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - 1.0)
        alt = max(t.relative_alt_m, 0.5)
        exp = 423.1 * 0.2 / alt * 2.0          # a blob "seen" 2x too small
        return PadHit(cx=320, cy=240, marker_id=None, radius_px=exp,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", blob)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rung_timeout_s=0.5, assigned_marker_id=None,
                            require_id_votes=0)))
    rung_gotos = [a for a in gotos if a < 12.0]
    assert rung_gotos and all(abs(a - 5.0) < 1e-6 for a in rung_gotos), gotos



# ── 2026-08-29 tip-over, the two mechanisms behind the wind-up ──────────────

class GroundedLateralCommander(GroundedCommander):
    """Records every goto; the airframe does NOT follow laterally (it is on
    the grass at idle thrust, or simply has not moved yet) — only the
    commanded altitude is tracked, and the detector fake sinks toward it."""

    def __init__(self, state: OrchestratorState) -> None:
        super().__init__(state)
        self.gotos: list[tuple[float, float, float]] = []

    async def goto(self, lat, lon, alt_m, yaw_deg=float("nan")) -> None:
        self.gotos.append((lat, lon, alt_m))
        self.target_alt = alt_m


def test_a_climb_back_from_the_bottom_rung_is_vertical_from_where_the_aircraft_is(
        monkeypatch) -> None:
    """2026-08-29, pad 5 (ULog 05_56_59, t=325-338): with the pad lost at the
    2 m rung the ladder's climb-back goto targeted the pad's ESTIMATED xy,
    1.4 m from where the aircraft actually sat on the grass. It could not move
    at idle thrust, the velocity integrator wound the demand to 22° of pitch in
    ten seconds, and the climb-out levered the airframe onto its tail. At the
    bottom rungs every lost-pad goto — the rung re-command and the climb-back
    — must be VERTICAL from the aircraft's own position; centring resumes once
    the pad is seen again."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedLateralCommander(state)
    t = state.telemetry
    t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0
    t.rangefinder_m, t.rangefinder_ts = 12.0, time.monotonic()
    off_centre_frames = iter([True, True])
    lost_at: list[int] = []            # len(cmd.gotos) when the pad was first lost

    def detect(frame_path, min_conf, assigned_id):
        tt = state_ref.telemetry
        if cmd.target_alt is not None and tt.relative_alt_m > cmd.target_alt:
            tt.relative_alt_m = max(cmd.target_alt, tt.relative_alt_m - 1.0)
        alt = max(tt.relative_alt_m, 0.5)
        tt.rangefinder_m, tt.rangefinder_ts = alt, time.monotonic()
        radius = 423.1 * 0.2 / alt
        if alt <= 2.6:
            # at the bottom rung: the pad is seen 300 px (≈1.4 m at 2 m) off
            # centre twice, then lost for good
            if next(off_centre_frames, False):
                return PadHit(cx=620, cy=240, marker_id=3, radius_px=radius,
                              confidence=0.9, corners=(), pad_side_px=0.0)
            if not lost_at:
                lost_at.append(len(cmd.gotos))
            return None
        return PadHit(cx=320, cy=240, marker_id=3, radius_px=radius,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rungs=(12.0, 5.0, 2.0), rung_tol_m=(1.5, 0.6, 0.3),
                            rung_descent_mps=(3.0, 1.0, 0.5),
                            rung_timeout_s=1.0, max_lost_cycles=3)))
    assert lost_at, "the fake never reached the lost-pad path"
    after_loss = cmd.gotos[lost_at[0]:]
    assert any(g[2] >= 4.0 for g in after_loss), (
        "the lost pad must trigger a climb-back", cmd.gotos)
    offsets = [ta._latlon_dist_m(g[0], g[1], t.lat, t.lon) for g in after_loss]
    assert all(d < 0.05 for d in offsets), (
        f"gotos after the pad was lost carry lateral offsets of {offsets} m — "
        "setpoints an airframe on the ground cannot follow", after_loss)


def test_a_lateral_correction_at_the_bottom_rung_is_scaled_by_the_lidar_height(
        monkeypatch) -> None:
    """A fix is a pixel offset scaled by the height the projection is given.
    With the mission's height frame reading 2.4 m HIGH (2026-08-29) a pad
    150 px off centre at a true 2 m projects 1.56 m away instead of 0.71 —
    every centring correction overshoots 2.2x. Below the lidar's range the
    projection must use the MEASURED height."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedLateralCommander(state)
    t = state.telemetry
    t.relative_alt_m = 4.4                      # the frame reads 2.4 m high
    t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0

    def detect(frame_path, min_conf, assigned_id):
        tt = state_ref.telemetry
        tt.rangefinder_m, tt.rangefinder_ts = 2.0, time.monotonic()   # the truth
        return PadHit(cx=470, cy=240, marker_id=3, radius_px=423.1 * 0.2 / 2.0,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rungs=(2.0,), rung_tol_m=(0.3,), rung_descent_mps=(0.5,),
                            rung_timeout_s=0.3)))
    corrections = cmd.gotos[1:]                  # gotos[0] is the acquire goto
    assert corrections, cmd.gotos
    lat, lon, _alt = corrections[0]
    d = ta._latlon_dist_m(lat, lon, t.lat, t.lon)
    true_offset = 150 * 2.0 / 423.1               # px · height / fx
    assert abs(d - true_offset) < 0.08, (
        f"correction of {d:.2f} m for a true offset of {true_offset:.2f} m — "
        "the projection is still scaled by the biased frame", cmd.gotos)


# ── "วางไม่ตรง ดีกว่าไม่วาง" (operator 2026-08-29): the centring gate ────────

def _off_centre_detector(monkeypatch, cmd, px_off: int):
    """Centred at the upper rungs; at the bottom rung the pad sits px_off
    pixels off centre every frame, so the 0.25 m lock is never met."""
    def detect(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - 1.0)
        alt = max(t.relative_alt_m, 0.5)
        t.rangefinder_m, t.rangefinder_ts = alt, time.monotonic()
        cx = 320 + px_off if alt <= 2.6 else 320
        return PadHit(cx=cx, cy=240, marker_id=3, radius_px=423.1 * 0.2 / alt,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)


def _run_off_centre(monkeypatch, px_off: int, **params):
    global state_ref
    state = state_ref = _state()
    cmd = GroundedLateralCommander(state)   # the airframe does not teleport onto each fix
    state.telemetry.rangefinder_min_m, state.telemetry.rangefinder_max_m = 0.4, 12.0
    _off_centre_detector(monkeypatch, cmd, px_off)
    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rungs=(12.0, 5.0, 2.0), rung_tol_m=(1.5, 0.6, 0.25),
                            rung_descent_mps=(3.0, 1.0, 0.5), rung_timeout_s=0.5,
                            **params)))
    return res, state, cmd


def test_an_off_centre_but_visible_pad_is_landed_on_not_deferred(monkeypatch) -> None:
    """Operator 2026-08-29, before the last flight: an egg placed 30 cm off the
    centre of a 1 m pad scores; an egg flown home because the 0.25 m lock never
    held scores nothing (the 29-Aug audit deferred at err 0.17 m). At the bottom
    rung a pad still in view within land_ok_err_m is landed on, audited."""
    res, state, cmd = _run_off_centre(monkeypatch, px_off=65)      # ≈0.31 m at 2 m
    assert res.landed and res.dropped, (res.notes, state.anomalies)
    assert any("land_gate_relaxed" in a for a in state.anomalies), state.anomalies
    assert not any("land_gate_not_centred" in a for a in state.anomalies)


def test_a_pad_further_off_than_the_relaxed_limit_still_defers_on_attempt_one(
        monkeypatch) -> None:
    res, state, cmd = _run_off_centre(monkeypatch, px_off=170)     # ≈0.80 m at 2 m
    assert not res.landed and not res.dropped, res.notes
    assert any("land_gate_not_centred" in a for a in state.anomalies), state.anomalies


def test_the_last_attempt_lands_on_any_pad_still_in_view(monkeypatch) -> None:
    """The mission retries a deferred pad once. On that retry the choice is
    between an egg placed at the pad's edge and an egg brought home."""
    res, state, cmd = _run_off_centre(monkeypatch, px_off=170, last_attempt=True)
    assert res.landed and res.dropped, (res.notes, state.anomalies)
    assert any("land_gate_relaxed" in a for a in state.anomalies), state.anomalies


def test_the_last_attempt_still_defers_when_the_pad_was_not_seen_at_the_bottom_rung(
        monkeypatch) -> None:
    """A 400 mm marker that has LEFT the frame at 2 m is more than ~1.3 m off —
    that is not "off-centre on the pad", that is next to it. An error carried
    over from a higher rung must not license the landing."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedCommander(state)
    state.telemetry.rangefinder_min_m, state.telemetry.rangefinder_max_m = 0.4, 12.0

    def detect(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None and t.relative_alt_m > cmd.target_alt:
            t.relative_alt_m = max(cmd.target_alt, t.relative_alt_m - 1.0)
        alt = max(t.relative_alt_m, 0.5)
        t.rangefinder_m, t.rangefinder_ts = alt, time.monotonic()
        if alt <= 2.6:
            return None                              # lost at the bottom rung
        return PadHit(cx=320, cy=240, marker_id=3, radius_px=423.1 * 0.2 / alt,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)
    res = asyncio.run(acquire_and_land_drop(
        cmd, state, target=Coordinate(lat=_LAT, lon=_LON), stop_index=0,
        params=_fast_params(rungs=(12.0, 5.0, 2.0), rung_tol_m=(1.5, 0.6, 0.25),
                            rung_descent_mps=(3.0, 1.0, 0.5), rung_timeout_s=0.5,
                            max_lost_cycles=3, last_attempt=True)))
    assert not res.landed and not res.dropped, res.notes


# ── review 2026-08-30 (pre-competition): four defects in the ladder found by an
# adversarial read of the code that had flown exactly one successful flight ────

_PROD_RUNGS = dict(rungs=(12.0, 8.0, 5.0, 3.0, 2.0),
                   rung_tol_m=(1.5, 1.0, 0.6, 0.35, 0.2),
                   rung_descent_mps=(3.0, 2.0, 1.0, 0.6, 0.4))


class LandHeightCommander(LaggingCommander):
    """Records the TRUE height (what the lidar reads) at the LAND command."""

    def __init__(self, state: OrchestratorState) -> None:
        super().__init__(state)
        self.land_true_alt = float("nan")

    async def land(self, *, disarm: bool = True) -> None:
        self.land_true_alt = self.state.telemetry.rangefinder_m
        await super().land(disarm=disarm)


def _biased_frame_detector(monkeypatch, cmd, bias_m: float, *, step_m: float = 1.0):
    """Pad centred every cycle. The aircraft's own AGL frame is wrong by
    ``bias_m``: TRUE height = AGL + bias_m (so a POSITIVE bias is a frame that
    reads LOW — the sign measured on five of the last seven flights). The
    lidar and the marker size both report the TRUE height."""
    def fake(frame_path, min_conf, assigned_id):
        t = state_ref.telemetry
        if cmd.target_alt is not None:
            step = cmd.target_alt - t.relative_alt_m
            t.relative_alt_m += max(-step_m, min(step_m, step))
        true_alt = max(t.relative_alt_m + bias_m, 0.3)
        t.rangefinder_m = true_alt
        t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0
        t.rangefinder_ts = time.monotonic()
        return PadHit(cx=320, cy=240, marker_id=3,
                      radius_px=423.1 * 0.2 / true_alt,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", fake)
    _patch_live_camera(monkeypatch)


def test_a_frame_that_reads_LOW_still_reaches_the_true_bottom_rung(monkeypatch) -> None:
    """The closed-loop rung command was floored in the UNTRUSTED frame —
    ``max(rungs[-1] * 0.5, agl - step)``. With the frame reading low (−0.93,
    −1.02, −1.03, −1.33, −1.36, −1.62 m on the recent flights) that floor
    clips the descent before the aircraft reaches the rung: it parks at a true
    ``1.0 + |bias|`` m, the altitude gate can never pass, the rung burns its
    whole budget and PX4 is handed a blind LAND from 2.6-3.5 m — the mechanism
    behind the 0.5-0.7 m placement errors. The step is already clamped to
    ``lidar − rung_alt``, so the floor must be expressed in the MEASURED
    frame, where it protects the same metre without lying about it."""
    global state_ref
    state = state_ref = _state()
    cmd = LandHeightCommander(state)
    _biased_frame_detector(monkeypatch, cmd, bias_m=+1.6)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 0,
        params=_fast_params(rung_timeout_s=3.0, **_PROD_RUNGS)))

    assert res.landed and cmd.landed_calls
    assert abs(cmd.land_true_alt - 2.0) <= 0.35, (
        f"LAND was commanded from a TRUE {cmd.land_true_alt:.2f} m instead of "
        "the 2 m rung — the frame-space floor clipped the descent")
    assert not any("alt_unverified" in a for a in state.anomalies), state.anomalies


def test_a_lost_pad_goto_carries_the_frame_bias_like_every_other_goto(
        monkeypatch) -> None:
    """Every in-view rung goto is commanded at ``rung + frame_bias``; the two
    lost-pad gotos (the rung re-command and the climb-back) were commanded at
    the RAW rung. With the 2026-08-29 frame reading +2.4 m high that is a true
    −0.4 m for the re-command and a true 0.6 m for the "climb" — the aircraft
    is pressed into the grass, which is exactly the ``lost@2m→climb`` ×5
    sequence that ended in the tip-over. The ground-contact guard only masks
    it while the lidar is alive."""
    global state_ref
    state = state_ref = _state()
    cmd = GroundedLateralCommander(state)
    t = state.telemetry
    t.rangefinder_min_m, t.rangefinder_max_m = 0.4, 12.0
    t.rangefinder_m, t.rangefinder_ts = 12.0, time.monotonic()
    BIAS = 2.4                     # the frame reads 2.4 m HIGH: true = agl - 2.4
    lost_from = {"i": None}

    def detect(frame_path, min_conf, assigned_id):
        tt = state_ref.telemetry
        if cmd.target_alt is not None and tt.relative_alt_m > cmd.target_alt:
            tt.relative_alt_m = max(cmd.target_alt, tt.relative_alt_m - 1.0)
        true_alt = max(tt.relative_alt_m - BIAS, 0.3)
        tt.rangefinder_m, tt.rangefinder_ts = true_alt, time.monotonic()
        if true_alt <= 2.5:
            # the 400 mm marker leaves the frame at the bottom rung (measured
            # on the 2026-08-28 trial) — from here the ladder is blind
            if lost_from["i"] is None:
                lost_from["i"] = len(cmd.gotos)
            return None
        return PadHit(cx=320, cy=240, marker_id=3,
                      radius_px=423.1 * 0.2 / true_alt,
                      confidence=0.9, corners=(), pad_side_px=0.0)
    monkeypatch.setattr(ta, "_detect_nadir", detect)
    _patch_live_camera(monkeypatch)

    asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 0,
        params=_fast_params(rung_timeout_s=1.0, lock_cycles=2, max_lost_cycles=3,
                            ground_contact_cycles=10_000, **_PROD_RUNGS)))

    assert lost_from["i"] is not None, "the pad was never lost — test is inert"
    after = [alt for _, _, alt in cmd.gotos[lost_from["i"]:]]
    assert after, cmd.gotos
    assert min(after) >= 3.5, (
        f"a lost-pad goto commanded {min(after):.2f} m in a frame reading "
        f"{BIAS:.1f} m HIGH — a TRUE {min(after) - BIAS:.2f} m, i.e. into the "
        f"ground (gotos after the loss: {after})")


class NoLandDetectAtHeight(FakeCommander):
    """PX4's land detector never reports (measured: 226 s on the grass on
    2026-08-29) and the height frame reads ~2 m LOW, while the lidar — alive
    and in range — says the aircraft is still 3.5 m up."""

    async def land(self, *, disarm: bool = True) -> None:
        self.landed_calls.append(disarm)
        t = self.state.telemetry
        t.relative_alt_m = 1.4                      # under land_alt_threshold_m
        # The lidar STREAMS at 10 Hz on the aircraft, so it is still answering
        # while the touchdown waits run — model that, not a single stale
        # sample (a stale reading is NaN and deliberately vetoes nothing).
        t.rangefinder_agl_m = lambda max_age_s=1.0: 3.5   # type: ignore[method-assign]
        # landed_state stays UNKNOWN


def test_the_altitude_fallback_cannot_release_while_the_lidar_says_airborne(
        monkeypatch) -> None:
    """The last-resort touchdown fallback reads ``relative_alt_m`` — the one
    frame known to be up to 3.4 m wrong — and sets ``landed = True`` with no
    further check, so the egg goes out. With the frame reading 2 m low the
    aircraft is at a TRUE 3.5 m when the frame shows 1.4: a broken egg and the
    forfeited "landed on the pad BEFORE releasing" line. A live rangefinder
    that says otherwise must veto it; when the lidar is dead (NaN) the old
    behaviour stands."""
    global state_ref
    state = state_ref = _state()
    cmd = NoLandDetectAtHeight(state)
    _patch_detector(monkeypatch, marker_id=3)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1,
        params=_fast_params(touchdown_timeout_s=0.3)))

    assert not res.dropped and not cmd.released, (
        "the egg was released from a TRUE 3.5 m on a height frame reading 1.4")
    assert any("release_skipped_touchdown_unconfirmed" in a
               for a in state.anomalies), state.anomalies


class PinnedDescentCommander(FakeCommander):
    """Knows its pinned MPC_Z_V_AUTO_DN, like the real board."""

    async def get_param_float(self, name: str) -> float:
        return 0.4


def test_an_acquire_timeout_hands_the_descent_cap_back(monkeypatch) -> None:
    """The ladder caps MPC_Z_V_AUTO_DN at 3.0 m/s before ACQUIRE and restores
    the pin after the rungs — but the acquire-timeout return skipped the
    restore, leaving 3.0 on the board (7.5x the validated 0.4). The leak is
    sticky: the next delivery's align reads 3.0 as its own "pin", so every
    AUTO descent that does not set its own cap — the 10.5 m decode visits, an
    RTL's descend phase — runs at 3 m/s until the flight ends."""
    global state_ref
    state = state_ref = _state()
    cmd = PinnedDescentCommander(state)
    monkeypatch.setattr(ta, "_detect_nadir",
                        lambda frame_path, min_conf, assigned_id: None)
    _patch_live_camera(monkeypatch)

    res = asyncio.run(acquire_and_land_drop(
        cmd, state, Coordinate(lat=_LAT, lon=_LON), 1,
        params=_fast_params(acquire_timeout_s=0.2)))

    assert not res.acquired
    assert cmd.params.get("MPC_Z_V_AUTO_DN") == 0.4, (
        f"the descent cap was left at {cmd.params.get('MPC_Z_V_AUTO_DN')} m/s "
        "after an acquire timeout")
