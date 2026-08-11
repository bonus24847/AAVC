"""Frequency-sweep system identification — fit the rate-loop PLANT from a ULog.

**numpy-only** (the reference build used scipy.signal welch/csd/coherence +
scipy.optimize; this reimplements the load-bearing pieces in numpy so the lean
stack keeps its dependency list). ``pyulog`` is imported lazily in the ULog
adapter only — the FRF + fit core is pure numpy and unit-testable on synthetic
signals with no log.

The open-loop rate plant is ``ω̇ = b·u`` where ``u`` is PX4's normalized torque
command (``vehicle_torque_setpoint`` ∈ [-1, 1]) and ``ω`` is the measured body
rate (``vehicle_angular_velocity``). Its transfer function is an integrator
``H(jω) = ω/u = b/(jω)``; a frequency sweep excites it and we recover ``b`` (and,
when the actuator/aero lag rolls magnitude off faster than -20 dB/dec, a
first-order lag ``τ``). This ``b`` is unit-identical to ``tuning.plant.mc_plant_gain``
(``τ_max/I``) so it can drop straight into the pole-placement gain synthesis
(``P = 2ζωₙ/b``).

IMPORTANT — the input is the **torque setpoint** (the controller's plant
command), NOT the rate setpoint: ``rate_sp → rate`` is the CLOSED-loop response
(→ 1 at low frequency), which would not recover the open-loop plant. The torque
command is not on the MAVSDK wire, hence the ULog.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

# PX4 nav_state for OFFBOARD (the sweep window). vehicle_status.nav_state == 14.
_NAV_STATE_OFFBOARD = 14
_AXIS_IDX = {"roll": 0, "pitch": 1, "yaw": 2}


def _round_opt(v: float | None, n: int) -> float | None:
    return round(v, n) if v is not None else None


# ──────────────────────────── data ────────────────────────────

@dataclass
class FrequencyResponse:
    """Empirical transfer function ``H = ω/u`` over the coherent frequency band."""

    axis: str = ""
    f_hz: np.ndarray = field(default_factory=lambda: np.array([]))
    H: np.ndarray = field(default_factory=lambda: np.array([], dtype=complex))
    coherence: np.ndarray = field(default_factory=lambda: np.array([]))
    fs_hz: float = 0.0
    nperseg: int = 0
    n_samples: int = 0
    source_ulog: str | None = None
    note: str = ""

    def ok(self) -> bool:
        return self.f_hz is not None and len(self.f_hz) >= 3

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis,
            "f_hz": [round(float(x), 4) for x in self.f_hz],
            "mag": [round(float(abs(h)), 6) for h in self.H],
            "phase_deg": [round(float(np.angle(h, deg=True)), 2) for h in self.H],
            "coherence": [round(float(c), 4) for c in self.coherence],
            "fs_hz": round(self.fs_hz, 2),
            "nperseg": self.nperseg,
            "n_samples": self.n_samples,
            "source_ulog": self.source_ulog,
            "note": self.note,
        }


@dataclass
class PlantFit:
    """Fitted rate-loop plant for one axis (the identified transfer function)."""

    axis: str = ""
    b: float | None = None              # integrator gain (rad/s² per unit cmd)
    tau_eff_s: float | None = None      # actuator/aero lag time-constant (if fit)
    omega_n_rad_s: float | None = None  # lag corner = 1/τ (if fit)
    fit_kind: str = "none"              # integrator | integrator_lag | none
    r2: float | None = None             # log-magnitude fit quality
    n_freq: int = 0
    coherence_med: float | None = None
    f_band_hz: tuple[float, float] | None = None
    source_ulog: str | None = None
    note: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis,
            "b": _round_opt(self.b, 5),
            "tau_eff_s": _round_opt(self.tau_eff_s, 5),
            "omega_n_rad_s": _round_opt(self.omega_n_rad_s, 3),
            "fit_kind": self.fit_kind,
            "r2": _round_opt(self.r2, 4),
            "n_freq": self.n_freq,
            "coherence_med": _round_opt(self.coherence_med, 4),
            "f_band_hz": [round(float(x), 3) for x in self.f_band_hz] if self.f_band_hz else None,
            "source_ulog": self.source_ulog,
            "note": self.note,
        }


# ──────────────────────────── FRF core (pure numpy) ────────────────────────────

def _to_uniform(
    t_s: np.ndarray, u: np.ndarray, y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float] | None:
    """Resample (possibly non-uniform) (u, y) onto a uniform grid at the median rate."""
    if len(t_s) < 8:
        return None
    order = np.argsort(t_s)
    t_s, u, y = t_s[order], u[order], y[order]
    dt = float(np.median(np.diff(t_s)))
    if not math.isfinite(dt) or dt <= 0:
        return None
    fs = 1.0 / dt
    tu = np.arange(t_s[0], t_s[-1], dt)
    if len(tu) < 8:
        return None
    return tu, np.interp(tu, t_s, u), np.interp(tu, t_s, y), fs


def _welch_h1(
    u: np.ndarray, y: np.ndarray, nperseg: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Welch-averaged cross/auto spectra → (Puu, Pyy, Puy) one-sided, 50% overlap,
    Hann window. The constant scale (fs · window-norm · n_seg) cancels in both the
    H1 estimator ``Puy/Puu`` and the coherence ``|Puy|²/(Puu·Pyy)``, so we keep raw
    accumulated periodograms — averaging across ≥3 segments is what makes the
    coherence meaningful (a single segment gives coherence ≡ 1)."""
    n = len(u)
    nperseg = int(min(nperseg, n))
    step = max(1, nperseg // 2)
    win = np.hanning(nperseg)
    starts = list(range(0, n - nperseg + 1, step)) or [0]
    nbins = nperseg // 2 + 1
    puu = np.zeros(nbins)
    pyy = np.zeros(nbins)
    puy = np.zeros(nbins, dtype=complex)
    for s in starts:
        su = (u[s:s + nperseg]) * win
        sy = (y[s:s + nperseg]) * win
        uf = np.fft.rfft(su)
        yf = np.fft.rfft(sy)
        puu += (uf * np.conj(uf)).real
        pyy += (yf * np.conj(yf)).real
        puy += np.conj(uf) * yf
    return puu, pyy, puy


def frf_from_arrays(
    t_s: np.ndarray, u: np.ndarray, y: np.ndarray, axis: str = "",
    *, coh_thresh: float = 0.5, f_band: tuple[float, float] | None = None,
    nperseg: int | None = None,
) -> FrequencyResponse:
    """Empirical transfer function ``H(f) = Y/U`` (H1 estimator) over the coherent
    band. ``u`` = input (torque command), ``y`` = output (body rate); ``t_s`` in
    seconds (need not be uniform). Pure — the offline-testable core."""
    res = FrequencyResponse(axis=axis)
    t_s = np.asarray(t_s, dtype=np.float64)
    u = np.asarray(u, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    m = np.isfinite(t_s) & np.isfinite(u) & np.isfinite(y)
    t_s, u, y = t_s[m], u[m], y[m]
    uni = _to_uniform(t_s, u, y)
    if uni is None:
        res.note = "insufficient/invalid samples for FRF"
        return res
    tu, uu, yu, fs = uni
    n = len(tu)
    uu = uu - float(np.mean(uu))    # de-mean (the plant integrator carries no DC)
    yu = yu - float(np.mean(yu))
    if nperseg is None:
        nperseg = int(2 ** math.floor(math.log2(max(n // 6, 64))))   # ≳ 5 averaging segments
    nperseg = int(min(nperseg, n))
    f = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    puu, pyy, puy = _welch_h1(uu, yu, nperseg)
    with np.errstate(divide="ignore", invalid="ignore"):
        h = puy / puu                                  # H1 estimator
        coh = (np.abs(puy) ** 2) / (puu * pyy)         # magnitude-squared coherence
    coh = np.clip(np.nan_to_num(coh, nan=0.0), 0.0, 1.0)
    mask = (f > 0) & np.isfinite(h) & (coh >= coh_thresh)
    if f_band is not None:
        mask &= (f >= f_band[0]) & (f <= f_band[1])
    pos = f > 0
    if np.any(pos) and np.max(puu[pos]) > 0:
        mask &= puu > 0.01 * np.max(puu[pos])          # drop bins with negligible excitation
    res.f_hz = f[mask]
    res.H = h[mask]
    res.coherence = coh[mask]
    res.fs_hz = float(fs)
    res.nperseg = int(nperseg)
    res.n_samples = int(n)
    if len(res.f_hz) < 3:
        res.note = f"too few coherent bins ({len(res.f_hz)}) — weak excitation or low coherence"
    return res


def _weighted_median(x: np.ndarray, w: np.ndarray) -> float:
    order = np.argsort(x)
    xs, ws = x[order], w[order]
    cum = np.cumsum(ws)
    if cum[-1] <= 0:
        return float(np.median(xs))
    return float(xs[int(np.searchsorted(cum, 0.5 * cum[-1]))])


def _r2_logmag(
    w: np.ndarray, h: np.ndarray, model_mag: Callable[[np.ndarray], np.ndarray]
) -> float:
    meas = np.log10(np.abs(h) + 1e-12)
    pred = np.log10(np.asarray(model_mag(w), dtype=np.float64) + 1e-12)
    ss_res = float(np.sum((meas - pred) ** 2))
    ss_tot = float(np.sum((meas - np.mean(meas)) ** 2))
    return (1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0


def _integrator_mag(b: float) -> Callable[[np.ndarray], np.ndarray]:
    """|H| = b/ω for a pure integrator."""
    def mag(w: np.ndarray) -> np.ndarray:
        return b / w
    return mag


def _integrator_lag_mag(b: float, tau: float) -> Callable[[np.ndarray], np.ndarray]:
    """|H| = b / (ω·√(1+(ωτ)²)) for an integrator + first-order lag. A factory
    (not an in-loop lambda) so each (b, τ) is captured by value — no late-binding."""
    def mag(w: np.ndarray) -> np.ndarray:
        return b / (w * np.sqrt(1.0 + (w * tau) ** 2))
    return mag


def fit_plant(frf: FrequencyResponse) -> PlantFit:
    """Fit ``b`` (integrator) and, if it improves the fit, an integrator+lag
    ``H = b/(jω(1+jωτ))``. Pure numpy — the scipy.optimize least-squares of the
    reference is replaced by a log-spaced τ grid with the closed-form best ``b``
    at each τ (``|H| = b/(ω√(1+(ωτ)²))`` ⇒ ``b = wmedian(|H|·ω·√(1+(ωτ)²))``)."""
    fit = PlantFit(axis=frf.axis, source_ulog=frf.source_ulog)
    if not frf.ok():
        fit.note = frf.note or "no usable FRF"
        return fit
    w = 2.0 * np.pi * frf.f_hz
    h = frf.H
    coh = frf.coherence
    fit.n_freq = int(len(frf.f_hz))
    fit.coherence_med = float(np.median(coh))
    fit.f_band_hz = (float(frf.f_hz[0]), float(frf.f_hz[-1]))

    # Integrator: |H| = b/ω  ⇒  b = |H|·ω (coherence-weighted median over the band).
    b_int = _weighted_median(np.abs(h) * w, coh)
    fit.b = b_int
    fit.fit_kind = "integrator"
    fit.r2 = _r2_logmag(w, h, _integrator_mag(b_int))

    # Integrator + first-order lag — grid the corner frequency between the band
    # edges (τ = 1/(2πf_c)); closed-form b at each τ; keep the best if it clearly
    # beats the pure integrator.
    f_lo, f_hi = float(frf.f_hz[0]), float(frf.f_hz[-1])
    if f_hi > f_lo > 0:
        taus = 1.0 / (2.0 * np.pi * np.logspace(np.log10(f_lo), np.log10(2.0 * f_hi), 40))
        best_b2, best_tau, best_r2 = b_int, None, fit.r2
        for tau in taus:
            tau = float(tau)
            if not (1e-4 <= tau <= 1.0):
                continue
            roll = np.sqrt(1.0 + (w * tau) ** 2)
            b2 = _weighted_median(np.abs(h) * w * roll, coh)
            r2 = _r2_logmag(w, h, _integrator_lag_mag(b2, tau))
            if r2 > best_r2:
                best_b2, best_tau, best_r2 = b2, tau, r2
        if best_tau is not None and best_r2 > (fit.r2 or -1.0) + 0.02:
            fit.b = best_b2
            fit.tau_eff_s = best_tau
            fit.omega_n_rad_s = 1.0 / best_tau
            fit.fit_kind = "integrator_lag"
            fit.r2 = best_r2
    return fit


# ──────────────────────────── ULog adapter (lazy pyulog) ────────────────────────────

def _topic(ulog: Any, name: str) -> Any | None:
    try:
        return ulog.get_dataset(name)
    except Exception:
        return None


def _ts(ds: Any) -> np.ndarray:
    return np.asarray(ds.data["timestamp"], dtype=np.float64)   # microseconds


def _offboard_window_us(ulog: Any) -> tuple[float, float] | None:
    """Span (µs) of the LAST contiguous OFFBOARD segment — the just-flown sweep.
    Reads ``vehicle_status.nav_state``; None when the topic/state is absent.

    One ULog can hold SEVERAL offboard windows: with the mission's
    ``COM_DISARM_LAND=-1`` the vehicle stays armed across sweeps, so PX4 keeps
    one log spanning the roll AND pitch AND yaw chirps. The fit runs right
    after each sweep, so its window is the last segment; the old first→last
    span mixed the axes' excitations (2026-07-04 stale/mixed-log failure)."""
    vs = _topic(ulog, "vehicle_status")
    if vs is None or "nav_state" not in vs.data:
        return None
    ts = _ts(vs)
    nav = np.asarray(vs.data["nav_state"])
    off = ts[nav == _NAV_STATE_OFFBOARD]
    if len(off) < 2:
        return None
    gaps = np.where(np.diff(off) > 2e6)[0]        # >2 s out of OFFBOARD = new segment
    start = off[gaps[-1] + 1] if len(gaps) else off[0]
    return float(start), float(off[-1])


def estimate_frf(
    ulog_path: str | Path, axis: str,
    window_rel_s: tuple[float, float] | None = None,
    *, coh_thresh: float = 0.5, f_band: tuple[float, float] | None = None,
) -> FrequencyResponse:
    """FRF from a ULog: input ``vehicle_torque_setpoint[axis]``, output
    ``vehicle_angular_velocity[axis]``, over the OFFBOARD (sweep) window."""
    res = FrequencyResponse(axis=axis, source_ulog=str(ulog_path))
    try:
        from pyulog import ULog
    except Exception as e:  # pragma: no cover — optional dep
        res.note = f"pyulog not installed ({e}); pip install -e .[tuning]"
        return res
    try:
        ulog = ULog(str(ulog_path))
    except Exception as e:
        res.note = f"ULog open failed: {e!s:.80}"
        return res
    tq = _topic(ulog, "vehicle_torque_setpoint")
    av = _topic(ulog, "vehicle_angular_velocity")
    if tq is None or av is None:
        res.note = "missing vehicle_torque_setpoint / vehicle_angular_velocity"
        return res
    key = f"xyz[{_AXIS_IDX[axis]}]"
    if key not in tq.data or key not in av.data:
        res.note = f"missing {key} in torque/rate topic"
        return res
    u_ts, u = _ts(tq), np.asarray(tq.data[key], dtype=np.float64)
    y_ts, y = _ts(av), np.asarray(av.data[key], dtype=np.float64)
    log_t0 = y_ts[0]
    span = _offboard_window_us(ulog)
    lo, hi = span if span is not None else (max(u_ts[0], y_ts[0]), min(u_ts[-1], y_ts[-1]))
    if window_rel_s is not None:
        lo = max(lo, log_t0 + window_rel_s[0] * 1e6)
        hi = min(hi, log_t0 + window_rel_s[1] * 1e6)
    ym = (y_ts >= lo) & (y_ts <= hi)
    if int(ym.sum()) < 16:
        res.note = "too few samples in window"
        return res
    t_us = y_ts[ym]
    out = frf_from_arrays(
        (t_us - t_us[0]) / 1e6, np.interp(t_us, u_ts, u), y[ym],
        axis=axis, coh_thresh=coh_thresh, f_band=f_band,
    )
    out.source_ulog = str(ulog_path)
    return out


def sysid_from_ulog(
    ulog_path: str | Path, axis: str,
    window_rel_s: tuple[float, float] | None = None,
    *, coh_thresh: float = 0.5, f_band: tuple[float, float] | None = None,
) -> PlantFit:
    """Convenience: estimate the FRF then fit the plant for one axis."""
    fit = fit_plant(estimate_frf(ulog_path, axis, window_rel_s,
                                 coh_thresh=coh_thresh, f_band=f_band))
    fit.source_ulog = str(ulog_path)
    return fit
