"""Unit tests for the GCS RealtimeBroadcaster — the fan-out that feeds the
dashboard. These guard the JSON-safety scrub, the once-only anomaly cursor, and
the drop ring buffer's bound + trajectory shape (a CLAUDE.md §5 contract seam).

The broadcaster only stores `state` and reads `state.anomalies` in the paths
tested here, so a minimal fake state suffices (no full OrchestratorState).
`_push_event` is a no-op until `start()` captures a loop, so these stay sync.
"""
from dashboard.realtime import DEFAULT_RING_MAXLEN, RealtimeBroadcaster


class _FakeState:
    def __init__(self) -> None:
        self.anomalies: list[str] = []


# ── _sanitize: command args must be JSON-serialisable ──

def test_sanitize_makes_args_json_safe():
    out = RealtimeBroadcaster._sanitize({"a": float("nan"), "b": 3, "c": object()})
    assert out["a"] is None          # NaN → None (NaN is invalid JSON)
    assert out["b"] == 3
    assert isinstance(out["c"], str)  # arbitrary object → str


# ── _drain_new_anomalies: each anomaly broadcast exactly once ──

def test_drain_new_anomalies_dedups_via_cursor():
    st = _FakeState()
    b = RealtimeBroadcaster(st)  # type: ignore[arg-type]
    st.anomalies.extend(["lost gps", "low batt"])
    assert len(list(b._drain_new_anomalies())) == 2
    assert list(b._drain_new_anomalies()) == []      # nothing new → no repeats
    st.anomalies.append("geofence")
    assert len(list(b._drain_new_anomalies())) == 1   # only the new one
    assert len(b._recent_anomalies) == 3              # all retained for late joiners


# ── record_drop: ring bound + trajectory shape ──

class _FakePoint:
    def __init__(self, t: float, lat: float, lon: float, alt: float) -> None:
        self.t_s, self.lat, self.lon, self.alt_agl_m = t, lat, lon, alt


class _FakeDrop:
    def __init__(self) -> None:
        self.points = [_FakePoint(0.0, 14.65, 101.18, 5.0)]
        self.impact_lat, self.impact_lon = 14.65, 101.18
        self.impact_t_s, self.horizontal_drift_m = 0.5, 0.2


def test_record_drop_ring_is_bounded_and_keeps_latest():
    b = RealtimeBroadcaster(_FakeState())  # type: ignore[arg-type]
    for _ in range(DEFAULT_RING_MAXLEN + 12):
        b.record_drop(_FakeDrop())  # type: ignore[arg-type]
    assert len(b._recent_drops) == DEFAULT_RING_MAXLEN     # FIFO eviction, not unbounded
    ev = b._recent_drops[-1]
    assert ev.trajectory[0] == (0.0, 14.65, 101.18, 5.0)   # (t_s, lat, lon, alt) order held
    assert ev.impact_lat == 14.65 and ev.impact_lon == 101.18


# ── _telemetry_frame: the mission-id queue reaches the GCS (W3 contract) ──

def test_telemetry_frame_carries_assigned_id_queue():
    from mavlink_adapter.telemetry import CurrentTelemetry
    from mission_brain.live_plan import render_live_plan
    from mission_brain.profile import COMPETITION
    from mission_brain.schemas import Coordinate
    from mission_brain.search_pattern import build_search_pattern
    from orchestrator.state import OrchestratorMode, OrchestratorState

    area = [[13.7307, 100.7878], [13.7307, 100.7898],
            [13.7314, 100.7899], [13.7312, 100.7878]]
    home = Coordinate(lat=13.730250, lon=100.787300)
    spec = build_search_pattern(area, home, sweep_alt_m=12.0)
    plan = render_live_plan(home, spec, discovered=[], profile=COMPETITION)
    st = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=plan, telemetry=CurrentTelemetry()
    )
    st.assigned_id_queue = [3, 1, 4, 6]
    frame = RealtimeBroadcaster(st)._telemetry_frame()
    assert frame.assigned_id_queue == [3, 1, 4, 6]
    assert frame.assigned_id_queue is not st.assigned_id_queue  # copied, not aliased


# ── anomaly channel: routine audit entries must NOT reach the operator feed ──

def test_anomaly_feed_excludes_routine_audit_entries():
    """2026-07-17 GCS defect: state.anomalies doubles as the full audit trail
    (1 Hz TELEM samples, SORTIE/TRANSIT events, operator commands), and the
    broadcaster streamed ALL of it as `anomaly` events — the red ANOMALY banner
    showed routine TELEM lines at 1 Hz and drowned real anomalies. Only
    record_anomaly() entries belong on the anomaly channel."""
    from mavlink_adapter.telemetry import CurrentTelemetry
    from orchestrator.state import OrchestratorMode, OrchestratorState

    st = OrchestratorState(
        mode=OrchestratorMode.OFFLINE, plan=None, telemetry=CurrentTelemetry()
    )
    b = RealtimeBroadcaster(st)  # type: ignore[arg-type]
    st.record_audit(
        "t=38.2s TELEM phase=transit_ingress sortie=1 lat=13.73 lon=100.78 alt=19.4 armed=1")
    st.record_audit("t=39.0s SORTIE 1 START pad=1 registry=unknown remaining=1200s")
    st.record_anomaly("battery_telemetry_nan")
    msgs = [ev.message for ev in b._drain_new_anomalies()]
    assert msgs == ["t=0.0s battery_telemetry_nan"]
    # the audit trail itself keeps EVERYTHING (verify_flight.py contract)
    assert len(st.anomalies) == 3
