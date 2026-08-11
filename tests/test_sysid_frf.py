"""Unit tests for the numpy-only frequency-response system-ID (tuning.sysid).

The FRF + plant fit are exercised on SYNTHETIC signals run through a known plant
``ω̇ = b·u`` (integrator, optionally + a first-order actuator lag), so the
recovered transfer-function parameters can be checked against ground truth with
no ULog. This is the offline guarantee behind "identify the transfer function".
"""

import numpy as np

from tuning.sysid import fit_plant, frf_from_arrays

FS = 250.0
DUR = 25.0


def _logchirp(t: np.ndarray, f0: float, f1: float, dur: float) -> np.ndarray:
    """Unit-amplitude logarithmic chirp f0→f1 over `dur` (same as sysid_sweep)."""
    k = f1 / f0
    phase = 2.0 * np.pi * f0 * dur / np.log(k) * (k ** (t / dur) - 1.0)
    return np.sin(phase)


def _excitation() -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(0.0, DUR, 1.0 / FS)
    u = _logchirp(t, 0.6, 13.0, DUR)
    return t, u


def _integrate(u: np.ndarray, dt: float) -> np.ndarray:
    """Trapezoidal cumulative integral (the integrator plant ω = b·∫u dt)."""
    out = np.zeros_like(u)
    out[1:] = np.cumsum((u[1:] + u[:-1]) * 0.5) * dt
    return out


def _first_order_lag(x: np.ndarray, tau: float, dt: float) -> np.ndarray:
    """Discrete 1/(1+τs) low-pass."""
    a = dt / (dt + tau)
    y = np.zeros_like(x)
    for k in range(1, len(x)):
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def test_recovers_integrator_gain():
    t, u = _excitation()
    dt = 1.0 / FS
    b_true = 120.0
    y = b_true * _integrate(u, dt)
    frf = frf_from_arrays(t, u, y, axis="roll")
    assert frf.ok()
    fit = fit_plant(frf)
    assert fit.b is not None
    # within ~15% of truth, and a clean (noise-free) board fits tightly.
    assert abs(fit.b - b_true) / b_true < 0.15
    assert fit.r2 is not None and fit.r2 > 0.9
    assert fit.coherence_med is not None and fit.coherence_med > 0.9


def test_recovers_integrator_plus_lag():
    t, u = _excitation()
    dt = 1.0 / FS
    b_true, tau_true = 200.0, 0.03
    y = b_true * _first_order_lag(_integrate(u, dt), tau_true, dt)
    fit = fit_plant(frf_from_arrays(t, u, y, axis="pitch"))
    assert fit.fit_kind == "integrator_lag"
    assert fit.tau_eff_s is not None
    # τ recovered within a factor of ~2 (grid resolution + discretisation).
    assert 0.5 * tau_true < fit.tau_eff_s < 2.0 * tau_true
    assert fit.b is not None and abs(fit.b - b_true) / b_true < 0.25


def test_noise_lowers_coherence_but_keeps_gain():
    t, u = _excitation()
    dt = 1.0 / FS
    b_true = 90.0
    rng = np.random.default_rng(0)
    clean = b_true * _integrate(u, dt)
    y = clean + 0.15 * np.std(clean) * rng.standard_normal(len(clean))
    fit = fit_plant(frf_from_arrays(t, u, y, axis="yaw"))
    assert fit.b is not None and abs(fit.b - b_true) / b_true < 0.20
    # independent measurement noise pulls median coherence below the clean case.
    assert fit.coherence_med is not None and fit.coherence_med < 1.0


def test_garbage_is_rejected():
    rng = np.random.default_rng(1)
    t = np.arange(0.0, 5.0, 1.0 / FS)
    u = rng.standard_normal(len(t))
    y = rng.standard_normal(len(t))   # output independent of input → no coherent plant
    fit = fit_plant(frf_from_arrays(t, u, y, axis="roll"))
    # Either too few coherent bins, or a meaningless low-coherence fit.
    assert (not fit.fit_kind == "integrator_lag") or (fit.coherence_med or 0) < 0.6


# ── OFFBOARD window extraction: a shared log can hold SEVERAL sweeps ──


class _DS:
    def __init__(self, data: dict) -> None:
        self.data = data


class _FakeUlog:
    """Duck-typed pyulog.ULog: get_dataset(name) → dataset with .data arrays."""

    def __init__(self, ds_map: dict) -> None:
        self._m = ds_map

    def get_dataset(self, name: str):
        if name not in self._m:
            raise KeyError(name)
        return self._m[name]


def test_offboard_window_uses_last_contiguous_segment():
    """With COM_DISARM_LAND=-1 the vehicle can stay armed across sweeps, so ONE
    ULog holds several OFFBOARD windows (roll chirp, then pitch chirp…). The
    fit that runs right after a sweep must window on THAT sweep — the LAST
    contiguous OFFBOARD segment — not the first→last span, which would mix the
    axes' excitations (the 2026-07-04 stale/mixed-log failure)."""
    from tuning.sysid import _offboard_window_us

    hz = 5.0
    def seg(t0: float, t1: float) -> np.ndarray:
        return np.arange(t0, t1, 1.0 / hz) * 1e6          # µs

    # roll sweep 10–35 s OFFBOARD, HOLD gap, pitch sweep 60–85 s OFFBOARD.
    ts = np.concatenate([seg(10, 35), seg(35, 60), seg(60, 85)])
    nav = np.concatenate([
        np.full(len(seg(10, 35)), 14),                    # OFFBOARD
        np.full(len(seg(35, 60)), 4),                     # HOLD between sweeps
        np.full(len(seg(60, 85)), 14),                    # OFFBOARD again
    ])
    ulog = _FakeUlog({"vehicle_status": _DS({"timestamp": ts, "nav_state": nav})})
    span = _offboard_window_us(ulog)
    assert span is not None
    lo, hi = span
    assert lo >= 60e6 - 1e5                               # starts at the LAST segment
    assert hi <= 85e6 + 1e5


def test_offboard_window_single_segment_unchanged():
    from tuning.sysid import _offboard_window_us

    ts = np.arange(10.0, 35.0, 0.2) * 1e6
    nav = np.full(len(ts), 14)
    ulog = _FakeUlog({"vehicle_status": _DS({"timestamp": ts, "nav_state": nav})})
    span = _offboard_window_us(ulog)
    assert span is not None
    assert span[0] == ts[0] and span[1] == ts[-1]
