#!/usr/bin/env python3
"""
fc_noise.py -- Allan deviation and a GMWM-style noise fit for an FC-V4 IMU capture.

    python fc_noise.py logs/imucap/20261104-010000          # .imu + .json from fc_imucap.py
    python fc_noise.py CAPTURE --fig                        # also write PNG figures (matplotlib)
    python fc_noise.py --selftest                           # recover known synthetic parameters

Output: per-axis overlapping Allan deviation, the white-noise level read from
the tau^-1/2 region with the rate-register quantisation share separated
(Sheppard: sigma^2_phys = sigma^2_meas - Delta^2/12), a WN + GM + RW model
fitted to the Allan variance, and the IEKF constants for Core/Inc/board.h.

Model and conventions (rates in rad/s or m/s^2, sample rate fs, tau = m/fs)
  white noise   AVAR = N^2 / tau            N  = ARW [rad/s/sqrt(Hz)] / VRW
  rate RW       AVAR = K^2 tau / 3          K  = RRW [rad/s/sqrt(s)]
  Gauss-Markov  exact AVAR of a stationary AR(1) (variance s^2, time const T);
                its peak stands in for bias instability, B ~= peak ADEV / 0.664
  Rate-register quantisation is WHITE in the rate output (ADEV slope -1/2,
  not the -1 of angle quantisation): it adds eta * Delta^2/12 per sample to
  the white level. eta = 1 for rho = sigma/Delta >~ 1 (checked and reported);
  below that the quantiser can suppress variance (eta < 1, see the Sheppard
  repo's eta(rho)), and the correction is flagged as unreliable.

The fit is the GMWM idea (Guerrier et al. 2013) applied to the Haar wavelet
variance (= AVAR / 2): minimise a weighted distance between empirical and
model variances over log-spaced scales, weights from the chi-square variance
of each estimate. Diagonal weighting, not the full GMWM Omega -- adequate for
setting filter noise, not for publication-grade inference.

The filter uses what it actually receives: IEKF_Q_ANGLE = the MEASURED white
level (quantisation included); the physical (corrected) ARW is reported for
comparison with the datasheet. IEKF_Q_BIAS = max(K, sqrt(2 s^2 / T)): the GM
behaves as a random walk of that strength for times << T, and the filter only
models a random walk. Requires numpy; matplotlib optional; scipy optional.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys

import numpy as np

D2R = math.pi / 180.0
G0 = 9.80665
REC = struct.Struct("<Ih3h3h")             # telemetry.h MSG_IMU payload (18 B)
# Below ~0.1 s the sensor's own LPF (13.37 % ODR, ~30 Hz) correlates successive
# samples: the ADEV rises to a hump at ~20 ms instead of following tau^-1/2.
# That region is the filter, not the noise model, so it is excluded from the
# white-level read-out and from the fit; and a Gauss-Markov term is only
# allowed time constants >= GM_T_MIN, so it cannot absorb the LPF hump (which
# would turn into an absurd bias random walk).
TAU_MIN_FIT = 0.1          # s
TAU_WHITE_MAX = 1.0        # s, upper end of the white-noise read-out
GM_T_MIN = 1.0             # s
TAU_ENV_MIN = 100.0        # s, start of the long-tau envelope for Q_BIAS


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def load(stem: str):
    """-> meta dict, ts (int64, unwrapped ticks), temp (degC), acc (counts Nx3), gyr (counts Nx3)."""
    stem = stem[:-4] if stem.endswith((".imu", ".json")) else stem
    with open(stem + ".json", encoding="utf-8") as fh:
        meta = json.load(fh)
    raw = np.fromfile(stem + ".imu", dtype=np.uint8)
    n = raw.size // REC.size
    rec = np.frombuffer(raw[: n * REC.size].tobytes(),
                        dtype=np.dtype([("ts", "<u4"), ("t", "<i2"), ("a", "<i2", 3), ("g", "<i2", 3)]))
    ts = rec["ts"].astype(np.int64) & 0xFFFFFF
    d = np.diff(ts) & 0xFFFFFF                      # 24-bit counter
    ts = np.concatenate(([0], np.cumsum(d))) + ts[0]
    return meta, ts, rec["t"] / 256.0, rec["a"].astype(np.float64), rec["g"].astype(np.float64)


def regularise(ts: np.ndarray, x: np.ndarray):
    """Put samples on the uniform tick grid. Duplicates (dts=0) dropped;
    missing ticks filled by linear interpolation. Returns x_uniform, n_missing."""
    keep = np.concatenate(([True], np.diff(ts) > 0))
    ts, x = ts[keep], x[keep]
    grid = np.arange(ts[0], ts[-1] + 1)
    missing = grid.size - ts.size
    if missing == 0:
        return x, 0
    out = np.empty((grid.size, x.shape[1]))
    for k in range(x.shape[1]):
        out[:, k] = np.interp(grid, ts, x[:, k])
    return out, int(missing)


# ---------------------------------------------------------------------------
# Allan variance (overlapping) and model AVARs
# ---------------------------------------------------------------------------
def avar(x: np.ndarray, fs: float, n_tau: int = 60):
    """Overlapping AVAR of a rate series via the integrated signal.
    Returns tau, avar, n_eff (number of independent-ish terms, for weights)."""
    x = np.asarray(x, dtype=np.float64) - np.mean(x)
    th = np.concatenate(([0.0], np.cumsum(x))) / fs
    n = x.size
    m_max = max(1, (n - 1) // 3)
    ms = np.unique(np.round(np.logspace(0, math.log10(m_max), n_tau)).astype(np.int64))
    taus, av, ne = [], [], []
    for m in ms:
        d = th[2 * m:] - 2.0 * th[m:-m] + th[:-2 * m]
        if d.size < 2:
            continue
        tau = m / fs
        taus.append(tau)
        av.append(float(np.dot(d, d)) / (2.0 * tau * tau * d.size))
        ne.append(max(1.0, n / m - 1.0))
    return np.asarray(taus), np.asarray(av), np.asarray(ne)


def avar_gm(m: np.ndarray, s2: float, phi: float) -> np.ndarray:
    """Exact AVAR (units of s2) of block means of a stationary AR(1) with lag-1
    correlation phi, block length m samples (derivation: Var(mean) - Cov(adjacent means))."""
    m = m.astype(np.float64)
    if phi <= 0.0:
        return s2 / m
    pm = phi ** m
    pm1 = phi ** (m - 1)
    a = phi * (1.0 - pm1) / (1.0 - phi)                                   # sum_{k=1}^{m-1} phi^k
    b = phi * (1.0 - m * pm1 + (m - 1.0) * pm) / (1.0 - phi) ** 2         # sum k phi^k
    tri = m * a - b                                                       # sum (m-k) phi^k
    s1 = m + 2.0 * tri
    s2sum = (b + m * pm) + pm * tri
    return s2 * (s1 - s2sum) / (m * m)


def model_avar(theta, tau, fs, terms):
    """theta: log-parameters per term. WN: log N^2. RW: log K^2. GM: log s^2, log T."""
    out = np.zeros_like(tau)
    i = 0
    m = np.maximum(1.0, np.round(tau * fs))
    for t in terms:
        if t == "WN":
            out += math.exp(theta[i]) / tau; i += 1
        elif t == "RW":
            out += math.exp(theta[i]) * tau / 3.0; i += 1
        elif t == "GM":
            s2, T = math.exp(theta[i]), GM_T_MIN + math.exp(theta[i + 1]); i += 2
            out += avar_gm(m, s2, math.exp(-1.0 / (T * fs)))
    return out


def _nelder_mead(f, x0, iters=4000, step=1.0):
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + step * np.eye(n)[i] for i in range(n)]
    val = [f(p) for p in pts]
    for _ in range(iters):
        order = np.argsort(val)
        pts, val = [pts[i] for i in order], [val[i] for i in order]
        if abs(val[-1] - val[0]) < 1e-12 * (1 + abs(val[0])):
            break
        c = np.mean(pts[:-1], axis=0)
        xr = c + (c - pts[-1]); fr = f(xr)
        if fr < val[0]:
            xe = c + 2 * (c - pts[-1]); fe = f(xe)
            pts[-1], val[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < val[-2]:
            pts[-1], val[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c); fc = f(xc)
            if fc < val[-1]:
                pts[-1], val[-1] = xc, fc
            else:
                pts = [pts[0] + 0.5 * (p - pts[0]) for p in pts]
                val = [f(p) for p in pts]
    i = int(np.argmin(val))
    return pts[i], val[i]


def fit(tau, av, ne, fs, terms):
    """Weighted log-space fit: residual r = log(avar_hat/avar_model), Var(r) ~ 2/n_eff."""
    w = ne / 2.0
    la = np.log(av)

    def cost(th):
        mod = model_avar(th, tau, fs, terms)
        if np.any(mod <= 0) or not np.all(np.isfinite(mod)):
            return 1e30
        r = la - np.log(mod)
        return float(np.sum(w * r * r))

    # Starting values from the curve itself.
    th0 = []
    for t in terms:
        if t == "WN":
            th0.append(math.log(av[0] * tau[0]))
        elif t == "RW":
            th0.append(math.log(max(av[-1] * 3.0 / tau[-1], 1e-30)))
        elif t == "GM":
            k = int(np.argmin(av * 0 + np.abs(np.log(tau) - np.log(tau[len(tau) // 2]))))
            th0 += [math.log(av[k] * 2.0), math.log(max(tau[k] - GM_T_MIN, 1.0))]
    try:
        from scipy.optimize import minimize          # optional
        best = None
        for start in (th0, [v + 1.0 for v in th0], [v - 1.0 for v in th0]):
            r = minimize(cost, start, method="Nelder-Mead", options={"maxiter": 20000, "xatol": 1e-6, "fatol": 1e-9})
            if best is None or r.fun < best[1]:
                best = (r.x, r.fun)
    except ImportError:
        best = None
        for start in (th0, [v + 1.0 for v in th0], [v - 1.0 for v in th0]):
            r = _nelder_mead(cost, start)
            if best is None or r[1] < best[1]:
                best = r
    return np.asarray(best[0]), float(best[1])


def describe(theta, terms):
    out, i = {}, 0
    for t in terms:
        if t == "WN":
            out["N"] = math.sqrt(math.exp(theta[i])); i += 1
        elif t == "RW":
            out["K"] = math.sqrt(math.exp(theta[i])); i += 1
        elif t == "GM":
            out["gm_sigma"] = math.sqrt(math.exp(theta[i])); out["gm_T"] = GM_T_MIN + math.exp(theta[i + 1]); i += 2
    return out


# ---------------------------------------------------------------------------
# One channel
# ---------------------------------------------------------------------------
def analyse_channel(x_units: np.ndarray, fs: float, lsb_units: float):
    """x_units: rate series in physical units (rad/s or m/s^2); lsb_units: one LSB in the same units."""
    tau, av, ne = avar(x_units, fs)
    # Per-sample noise: total std about a 1 s moving mean (the LPF makes
    # successive samples correlated, so a difference-based estimate is low).
    w = max(1, int(fs))
    k = np.ones(w) / w
    hp = x_units[w:-w] - np.convolve(x_units, k, mode="same")[w:-w]
    sigma_sample = float(np.std(hp))
    rho = sigma_sample / lsb_units
    q_var = lsb_units ** 2 / 12.0
    sel = (tau >= TAU_MIN_FIT) & (tau <= TAU_WHITE_MAX)
    n_meas = float(np.sqrt(np.median(av[sel] * tau[sel])))
    n_phys = math.sqrt(max(n_meas ** 2 - q_var / fs, 0.0))
    fsel = tau >= TAU_MIN_FIT
    fits = {}
    for terms in (("WN", "RW"), ("WN", "GM", "RW")):
        th, c = fit(tau[fsel], av[fsel], ne[fsel], fs, terms)
        fits["+".join(terms)] = {"params": describe(th, terms), "cost": c, "aic": c + 2 * len(th),
                                 "theta": th.tolist(), "terms": list(terms)}
    best = min(fits.values(), key=lambda f: f["aic"])
    adev = np.sqrt(av)
    i_min = int(np.argmin(adev[fsel])) + int(np.argmax(fsel))
    env = tau >= TAU_ENV_MIN
    k_env = float(np.max(adev[env] * np.sqrt(3.0 / tau[env]))) if np.any(env) else float("nan")
    return {
        "tau": tau.tolist(), "adev": adev.tolist(),
        "sigma_sample": sigma_sample, "lsb": lsb_units, "rho": rho, "eta_ok": rho >= 0.8,
        "N_meas": n_meas, "N_phys": n_phys, "quant_share": (q_var / fs) / n_meas ** 2 if n_meas > 0 else float("nan"),
        "adev_min": float(adev[i_min]), "tau_at_min": float(tau[i_min]),
        "B_flicker": float(adev[i_min] / 0.664), "K_env": k_env,
        "fits": fits, "best": "+".join(best["terms"]),
    }


def iekf_constants(gyro: list[dict], acc: list[dict]) -> dict:
    """Worst axis of each quantity: the IEKF has one Q/R for all axes.
    Q_BIAS uses the long-tau envelope K_env = max_tau>=100s sigma(tau) sqrt(3/tau):
    the random walk that bounds the observed bias wander INCLUDING any
    temperature-driven drift in the run (conservative; the fitted RRW alone
    would understate a bias the filter must track on a cooling pad)."""
    n = max(c["N_meas"] for c in gyro)
    k_env = max(c["K_env"] for c in gyro)
    sig_a = max(c["sigma_sample"] for c in acc)
    return {
        "IEKF_Q_ANGLE": n,                        # rad/s/sqrt(Hz), measured (quantisation included)
        "IEKF_Q_BIAS": k_env,                     # rad/s/sqrt(s)
        "IEKF_R_ACCEL_floor": (sig_a / G0) ** 2,  # normalised direction variance, sensor noise alone
        "accel_sigma_sample_mps2": sig_a,
    }


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def run(stem: str, figs: bool) -> dict:
    meta, ts, temp, acc, gyr = load(stem)
    fs = meta["odr_hz"]
    data, missing = regularise(ts, np.hstack([gyr, acc, temp[:, None]]))
    g_lsb_dps = 1.0 / meta["g_lsb_per_dps"]
    a_lsb_g = 1.0 / meta["a_lsb_per_g"]
    gyro_rad = data[:, 0:3] * g_lsb_dps * D2R
    acc_ms2 = data[:, 3:6] * a_lsb_g * G0
    hours = data.shape[0] / fs / 3600.0
    print(f"{os.path.basename(stem)}: {data.shape[0]:,} samples, {hours:.2f} h at {fs:.2f} Hz, "
          f"{missing} missing ({100.0 * missing / data.shape[0]:.3f} %), "
          f"temp {data[:, 6].min():.2f}..{data[:, 6].max():.2f} degC")
    res = {"meta": meta, "fs": fs, "samples": int(data.shape[0]), "missing": missing,
           "temp_range": [float(data[:, 6].min()), float(data[:, 6].max())], "gyro": [], "accel": []}
    for k in range(3):
        res["gyro"].append(analyse_channel(gyro_rad[:, k], fs, g_lsb_dps * D2R))
    for k in range(3):
        res["accel"].append(analyse_channel(acc_ms2[:, k], fs, a_lsb_g * G0))
    # Temperature sensitivity of the gyro bias (1-minute means), for interpreting the long-tau end.
    blk = int(60 * fs)
    if data.shape[0] >= 3 * blk:
        nb = data.shape[0] // blk
        tb = data[: nb * blk, 6].reshape(nb, blk).mean(1)
        res["gyro_tempco_dps_per_C"] = []
        for k in range(3):
            gb = (data[: nb * blk, k] * g_lsb_dps).reshape(nb, blk).mean(1)
            slope = float(np.polyfit(tb, gb, 1)[0]) if np.ptp(tb) > 0.2 else float("nan")
            res["gyro_tempco_dps_per_C"].append(slope)

    ax = "xyz"
    print("\nGYRO  ARW (white, 0.1-1 s)      quant  rho   BI (min/0.664)        RRW fit        K_env (>=100 s)   model")
    for k, c in enumerate(res["gyro"]):
        p = c["fits"][c["best"]]["params"]
        print(f"  {ax[k]}  {c['N_meas'] / D2R * 1e3:6.2f} mdps/rtHz {c['N_meas'] / D2R * 60:5.3f} deg/rth"
              f"  {100 * c['quant_share']:4.1f}%  {c['rho']:4.2f}  {c['B_flicker'] / D2R * 3600:5.1f} deg/h @{c['tau_at_min']:5.0f} s"
              f"  {p.get('K', 0.0) / D2R * 3600 * 60:6.1f} deg/h/rth  {c['K_env'] / D2R * 3600 * 60:6.1f} deg/h/rth  {c['best']}"
              + ("" if c["eta_ok"] else "  (rho<0.8: Sheppard correction unreliable)"))
    print("\nACCEL VRW (white, 0.1-1 s)      sigma/sample   BI (min/0.664)")
    for k, c in enumerate(res["accel"]):
        print(f"  {ax[k]}  {c['N_meas'] / G0 * 1e6:6.1f} ug/rtHz ({c['N_meas'] * 1e3:5.3f} mm/s/rts)  "
              f"{c['sigma_sample'] / G0 * 1e3:6.3f} mg   {c['B_flicker'] / G0 * 1e6:6.1f} ug @{c['tau_at_min']:5.0f} s")
    if "gyro_tempco_dps_per_C" in res:
        print("\ngyro bias tempco (dps/degC, 1-min means): "
              + "  ".join(f"{ax[k]} {v:+.4f}" for k, v in enumerate(res["gyro_tempco_dps_per_C"])))
    ic = iekf_constants(res["gyro"], res["accel"])
    res["iekf"] = ic
    print("\nboard.h (IEKF, worst axis):")
    print(f"  #define IEKF_Q_ANGLE  {ic['IEKF_Q_ANGLE']:.4e}f  /* rad/s/sqrt(Hz), measured ARW incl. quantisation */")
    print(f"  #define IEKF_Q_BIAS   {ic['IEKF_Q_BIAS']:.4e}f  /* rad/s/sqrt(s), long-tau envelope (thermal drift included) */")
    print(f"  sensor-noise floor for IEKF_R_ACCEL: {ic['IEKF_R_ACCEL_floor']:.3e} "
          f"(sigma {ic['accel_sigma_sample_mps2'] / G0 * 1e3:.2f} mg per sample); R must also cover "
          f"non-gravitational acceleration (handling, vibration) -- see README")
    out = os.path.splitext(stem)[0] + "_noise.json"
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1)
    print(f"\nresults -> {out}")
    if figs:
        plot(res, os.path.splitext(stem)[0])
    return res


def plot(res: dict, stem: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: no figures")
        return
    fs = res["fs"]
    for name, unit, scale in (("gyro", "deg/h", 3600 / D2R), ("accel", "ug", 1e6 / G0)):
        fig, axs = plt.subplots(1, 3, figsize=(14, 4.2), sharey=True)
        for k, a in enumerate(axs):
            c = res[name][k]
            tau = np.asarray(c["tau"])
            a.loglog(tau, np.asarray(c["adev"]) * scale, "k.", ms=3, label="overlapping ADEV")
            f = c["fits"][c["best"]]
            tf = tau[tau >= TAU_MIN_FIT]
            a.loglog(tf, np.sqrt(model_avar(np.asarray(f["theta"]), tf, fs, f["terms"])) * scale, "C0-",
                     label=f"fit {c['best']} (tau >= {TAU_MIN_FIT:g} s)")
            a.axvspan(tau[0], TAU_MIN_FIT, color="0.9", zorder=0, label="sensor LPF region (not fitted)")
            a.loglog(tau, np.sqrt(c["lsb"] ** 2 / 12 / fs / tau) * scale, "C3--", lw=1,
                     label="quantisation share (white, slope -1/2)")
            a.set_title(f"{name} {'xyz'[k]}")
            a.set_xlabel("tau (s)")
            a.grid(True, which="both", alpha=0.3)
        axs[0].set_ylabel(f"Allan deviation ({unit})")
        axs[0].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{stem}_{name}_adev.png", dpi=130)
        plt.close(fig)
        print(f"figure -> {stem}_{name}_adev.png")


# ---------------------------------------------------------------------------
# Self-test: synthetic WN + RW (+ GM) through a 16-bit quantiser
# ---------------------------------------------------------------------------
def selftest(seed: int = 1) -> bool:
    rng = np.random.default_rng(seed)
    fs, n = 224.2, 224 * 3600                  # 1 h
    N, K = 2.3e-4, 2.0e-6                      # rad/s/sqrt(Hz), rad/s/sqrt(s)
    wn = rng.standard_normal(n) * N * math.sqrt(fs)
    rw = np.cumsum(rng.standard_normal(n)) * K / math.sqrt(fs)
    lsb = 1.0 / 16.0 * D2R                     # +/-2048 dps
    x = np.round((wn + rw + 0.37 * lsb) / lsb) * lsb
    c = analyse_channel(x, fs, lsb)
    p = c["fits"]["WN+RW"]["params"]
    n_true_meas = math.sqrt(N ** 2 + lsb ** 2 / 12 / fs)
    ok = (abs(c["N_meas"] / n_true_meas - 1) < 0.05 and abs(c["N_phys"] / N - 1) < 0.08
          and abs(p["K"] / K - 1) < 0.5)
    print(f"selftest: N_meas {c['N_meas']:.3e} (true {n_true_meas:.3e}), N_phys {c['N_phys']:.3e} (true {N:.3e}), "
          f"K {p['K']:.3e} (true {K:.3e}), rho {c['rho']:.2f} -> {'OK' if ok else 'FAIL'}")
    # GM AVAR formula against brute force on a short AR(1).
    phi, s2 = 0.98, 1.0
    e = rng.standard_normal(400000) * math.sqrt(s2 * (1 - phi * phi))
    y = np.empty_like(e); y[0] = 0.0
    for i in range(1, y.size):
        y[i] = phi * y[i - 1] + e[i]
    tau, av, _ = avar(y, 1.0, 20)
    th = avar_gm(np.round(tau), s2, phi)
    err = float(np.max(np.abs(av[:12] / th[:12] - 1)))
    print(f"selftest: AR(1) AVAR closed form vs data, max rel err {err:.3f} -> {'OK' if err < 0.1 else 'FAIL'}")
    return ok and err < 0.1


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", help="capture stem (path without .imu/.json)")
    ap.add_argument("--fig", action="store_true", help="write Allan deviation PNGs (needs matplotlib)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(0 if selftest() else 1)
    if not a.capture:
        ap.error("capture stem required")
    run(a.capture, a.fig)


if __name__ == "__main__":
    main()
