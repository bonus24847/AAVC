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

