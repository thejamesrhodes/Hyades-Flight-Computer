"""BENCH-02 step 3: fit the SITL servo model to every analysed run at once.

    python fit_servo.py DIR [--runs steps,sine1,sine3,creep,flightn] [--out fit.json]

Reads DIR/*.runN.npz (analyse.py) and replays each run's command through the servo model the SITL uses
(Tools/sitl/sitl.c servo_step): an input threshold (a new target only once the command has moved sod from
the last accepted one), a dead time, a second-order position loop (omega_n, zeta) with a rate limit, and
lash at the output. The model runs at 2 kHz; frame k's pulse starts at k/200 s. The fit minimises a robust
(soft-L1) misfit between model and measured angle over all chosen runs together.

Not in the fitted model: the landing error (each new target lands a random 0.1-0.2 deg away; it is left as
residual and its size is reported from the settled levels), and the rolling-shutter offset between the dot
and the LEDs (it adds to the fitted dead time; analyse.py reports its bound for each run).
"""
import argparse, ctypes, glob, json, os, subprocess, tempfile
import numpy as np
from scipy.optimize import least_squares
import profiles as P

C_SRC = r"""
#include <math.h>
/* cmd: per-frame command (deg); n frames at 200 Hz from frame -pre. Output y at times t (s from RUN frame 0). */
void servo_sim(const double *cmd, int n, int pre, const double *p, const double *t, int nt, double *y)
{
    const double sod = p[0], tau = p[1], wn = p[2], zeta = p[3], rate = p[4], lash = p[5];
    const int sub = 10; const double dt = 1.0 / (200.0 * sub);
    double dd = tau / dt; if (dd < 0.0) dd = 0.0; if (dd > 19998.0) dd = 19998.0;
    const int nd = (int)floor(dd); const double fr = dd - nd;      /* fractional delay: linear interpolation */
    static double hist[20002]; int w = 0; const int L = nd + 2;
    for (int i = 0; i < L; i++) hist[i] = 0.0;
    double acc = 0.0, tgt = 0.0, th = 0.0, thd = 0.0, out = 0.0;
    const double b = 0.5 * lash;
    int j = 0;
    for (int k = 0; k < n * sub && j < nt; k++) {
        const double c = cmd[k / sub];
        if (fabs(c - acc) >= sod - 1e-12) { acc = c; tgt = c; }
        hist[w] = tgt;
        const double u0 = hist[(w - nd + 2 * L) % L], u1 = hist[(w - nd - 1 + 2 * L) % L];
        const double u = (1.0 - fr) * u0 + fr * u1;
        w = (w + 1) % L;
        double thdd = wn * wn * (u - th) - 2.0 * zeta * wn * thd;
        thd += thdd * dt;
        if (thd > rate) thd = rate; if (thd < -rate) thd = -rate;
        th += thd * dt;
        if (th - out > b) out = th - b; else if (out - th > b) out = th + b;
        const double tk = (k + 1) * dt - pre / 200.0;
        while (j < nt && t[j] <= tk) { y[j] = out; j++; }
    }
    while (j < nt) { y[j] = out; j++; }
}
"""
_lib = None


def lib():
    global _lib
    if _lib is None:
        d = tempfile.mkdtemp()
        src, so = os.path.join(d, "servo.c"), os.path.join(d, "servo.so")
        open(src, "w").write(C_SRC)
        subprocess.run(["gcc", "-O2", "-shared", "-fPIC", "-o", so, src, "-lm"], check=True)
        _lib = ctypes.CDLL(so)
        dp = ctypes.POINTER(ctypes.c_double)
        _lib.servo_sim.argtypes = [dp, ctypes.c_int, ctypes.c_int, dp, dp, ctypes.c_int, dp]
    return _lib


def simulate(cmd, p, t):
    """cmd: per-frame command from RUN frame 0 (deg); prepended with PRE zeros. t: sorted times (s)."""
    full = np.ascontiguousarray(np.r_[np.zeros(P.PRE), cmd, np.zeros(P.POST)], dtype=np.float64)
    pp = np.ascontiguousarray(p, dtype=np.float64)
    tt = np.ascontiguousarray(t, dtype=np.float64)
    y = np.zeros(len(tt))
    dp = ctypes.POINTER(ctypes.c_double)
    lib().servo_sim(full.ctypes.data_as(dp), len(full), P.PRE, pp.ctypes.data_as(dp), tt.ctypes.data_as(dp), len(tt), y.ctypes.data_as(dp))
    return y


NAMES = ["sod_deg", "tau_s", "wn_rad_s", "zeta", "rate_deg_s", "lash_deg"]
X0 = np.array([0.25, 0.010, 130.0, 0.8, 800.0, 0.15])
LO = np.array([0.0, 0.0, 30.0, 0.2, 100.0, 0.0])
HI = np.array([1.0, 0.040, 400.0, 2.0, 3000.0, 1.0])


def load(dirpath, names):
    runs = []
    for f in sorted(glob.glob(os.path.join(dirpath, "*.run*.npz"))):
        js = json.load(open(f[:-4] + ".json"))
        if js["profile"] not in names:
            continue
        d = np.load(f)
        t, a = d["t"], d["angle"]
        m = np.isfinite(a) & (t > -0.3) & (t < len(d["cmd"]) / P.FRAME_HZ + 0.3)
        # leave out the large moves that leave the picture (> 25 deg)
        m &= np.abs(np.interp(t, np.arange(len(d["cmd"])) / P.FRAME_HZ, d["cmd"])) < 25
        runs.append(dict(name=js["profile"], file=os.path.basename(f), t=t[m], a=a[m], cmd=d["cmd"],
                         shutter_ms=1e3 * js.get("rolling_shutter_rows", 0) / js.get("camera_fps", 240)))
    return runs


def residuals(p, runs):
    out = []
    for r in runs:
        y = simulate(r["cmd"], p, r["t"])
        e = r["a"] - y
        out.append(e / np.sqrt(len(e)))          # each run counts equally, whatever its length
    return np.concatenate(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir")
    ap.add_argument("--runs", default="steps,sine1,sine3,sine03,creep,flightn")
    ap.add_argument("--out")
    a = ap.parse_args()
    runs = load(a.dir, a.runs.split(","))
    if not runs:
        raise SystemExit("no analysed runs found (run analyse.py first)")
    print("runs:", ", ".join(f"{r['name']} ({len(r['t'])} frames)" for r in runs))
    # The threshold is a hard switch (no useful gradient): search it on a grid, fitting the rest each time.
    def inner(sod, x):
        f = lambda q: residuals(np.r_[sod, q], runs)
        return least_squares(f, x, bounds=(LO[1:], HI[1:]), loss="soft_l1", f_scale=0.02,
                             x_scale=np.array([0.002, 20, 0.1, 100, 0.05]), diff_step=1e-2, max_nfev=200)
    x, best = X0[1:].copy(), None
    for sod in np.arange(0.15, 0.351, 0.01):
        r = inner(sod, x if best is None else best[1].x)
        if best is None or r.cost < best[1].cost:
            best = (sod, r)
    sod0 = best[0]
    for sod in np.arange(sod0 - 0.009, sod0 + 0.0091, 0.002):
        r = inner(sod, best[1].x)
        if r.cost < best[1].cost:
            best = (sod, r)
    fit = best[1]
    p = np.r_[best[0], fit.x]
    print(f"threshold search: best {best[0]:.3f} deg")
    # rough 1-sigma from the Jacobian (robust loss: indicative only)
    J = fit.jac
    try:
        cov = np.linalg.inv(J.T @ J) * (2 * fit.cost / max(len(fit.fun) - len(p) + 1, 1))
        sd = np.r_[0.01, np.sqrt(np.diag(cov))]          # threshold: the grid step
    except np.linalg.LinAlgError:
        sd = np.full(len(p), np.nan)
    res = dict(params={k: float(v) for k, v in zip(NAMES, p)}, sd={k: float(v) for k, v in zip(NAMES, sd)}, runs={})
    print("fitted:", ", ".join(f"{k} {v:.4g} (+/-{s:.2g})" for k, v, s in zip(NAMES, p, sd)))
    for r in runs:
        y = simulate(r["cmd"], p, r["t"])
        e = r["a"] - y
        c = np.interp(r["t"] * P.FRAME_HZ, np.arange(len(r["cmd"])) + 0.5, r["cmd"])
        res["runs"][r["file"]] = dict(profile=r["name"], model_error_rms_deg=float(np.sqrt(np.mean(e ** 2))),
                                      model_error_p95_deg=float(np.percentile(np.abs(e), 95)),
                                      command_error_rms_deg=float(np.sqrt(np.mean((r["a"] - c) ** 2))),
                                      rolling_shutter_bound_ms=r["shutter_ms"])
        print(f"  {r['name']:8s} measured - model {res['runs'][r['file']]['model_error_rms_deg']:.3f} deg rms "
              f"(measured - command {res['runs'][r['file']]['command_error_rms_deg']:.3f})")
    json.dump(res, open(a.out or os.path.join(a.dir, "servo_fit.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
