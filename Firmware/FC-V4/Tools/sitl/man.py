"""In-flight roll test manoeuvres: tracking and identification quality over the build-tolerance Monte Carlo.

    python man.py --exe ./sitl_law1 --kind 2 --amp 114.6 --T 0.8 --n 20 --out runs/M2

Each flight is drawn exactly as mc.py draws it (BENCH-02 servo, build tolerance), with a test manoeuvre timed
from COAST entry (sitl.c man_*). The 200 Hz trace is reduced to what the flight log would hold (100 Hz, gyro
0.1 dps, roll command at the log's 30/127 deg or at full resolution) and the roll plant is identified by
equation error:   pdot = beta * qhat * delta + kp * qhat * p / V + phi * tanh(p / 0.3) + g * qhat + b0
beta is the number the roll law rests on (canard effectiveness per q, S d Cl_delta / Ixx).
"""
import argparse, json, os, subprocess, sys
import numpy as np, pandas as pd
from concurrent.futures import ProcessPoolExecutor
from scipy.signal import savgol_filter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from man_one import params

MOTORS = ["G40W", "G53FJ", "G79W", "G80T", "G77R", "G104T"]
BETA_NOM = 0.559          # board.h CTRL_ESO_BETA: S d Cl_delta / Ixx at the nominal values


def act_model(cmd, n_delay=1, ta=0.012, dt=0.01):
    """Canard from the logged command: n_delay samples of dead time, then a first-order lag (bench servo)."""
    u = np.concatenate([np.full(n_delay, cmd[0]), cmd[:-n_delay]]) if n_delay else cmd.copy()
    y = np.empty_like(u); a = dt / (ta + dt); y[0] = u[0]
    for i in range(1, len(u)):
        y[i] = y[i - 1] + a * (u[i] - y[i - 1])
    return y


def act_from_video(cmd, tv, dl_v, t):
    """Fit the servo map to the video canard angle (gain, offset, dead time, lag) and return the fitted,
    noise-free canard at the log rate. Regressing on the raw video angle instead biases beta low
    (errors in variables: 0.2 deg of frame noise against a 0.3-1 deg excitation)."""
    best = None
    for n in range(0, 5):
        for ta in (0.005, 0.010, 0.015, 0.020, 0.030, 0.040):
            pr = act_model(cmd, n, ta); x = np.interp(tv, t, pr)
            A = np.column_stack([x, np.ones_like(x)]); th, *_ = np.linalg.lstsq(A, dl_v, rcond=None)
            r = float(np.sum((dl_v - A @ th) ** 2))
            if best is None or r < best[0]:
                best = (r, th[0] * pr + th[1], th[0], n, ta)
    return best[1], best[2], best[3], best[4]


def identify(c, q_cmd, slip=None, act=True):
    """c: 100 Hz coast rows. Returns beta-hat (1/(Pa s^2)), friction term (rad/s^2), residual sd."""
    p = np.radians(c.gyro_p_q.values)
    pd_ = savgol_filter(p, 9, 2, deriv=1, delta=0.01)
    ps = savgol_filter(p, 9, 2)
    d = np.radians(act_model(q_cmd) if act else q_cmd)          # video: the canard itself, no servo model
    qh = c.qbar_est.values; V = np.maximum(c.vz_est.values, 5.0)
    sl = ps if slip is None else savgol_filter(slip, 9, 2)
    X = np.column_stack([qh * d, qh * ps / V, np.tanh(sl / 0.5), qh, np.ones_like(qh)])
    m = (qh > 800)
    if m.sum() < 50:
        return dict(beta=np.nan, phi=np.nan, res=np.nan, n=int(m.sum()))
    th, *_ = np.linalg.lstsq(X[m], pd_[m], rcond=None)
    res = pd_[m] - X[m] @ th
    # parameter standard error from the residual (white-noise assumption: optimistic)
    cov = np.linalg.pinv(X[m].T @ X[m]) * res.var()
    return dict(beta=th[0], beta_se=np.sqrt(cov[0, 0]), phi=th[2], res=res.std(), n=int(m.sum()))


def one(job):
    exe, motor, seed, extra, keep = job
    tr = f"/tmp/man_{os.getpid()}_{seed}.csv"
    d = params(motor, seed, dict(extra, trace=tr, trace_dec=1))
    r = subprocess.run([exe, f"seed={seed}"] + [f"{k}={v}" for k, v in d.items()], capture_output=True, text=True, timeout=180)
    kv = {}
    for t in r.stdout.split():
        k, v = t.split("=", 1)
        try: kv[k] = float(v)
        except ValueError: kv[k] = v
    t = pd.read_csv(tr)
    if not keep:
        os.remove(tr)
    out = dict(motor=motor, seed=seed, p_max=max(kv["p_max_boost"], kv["p_max_coast"]), p_rms_coast=kv["p_rms_coast"],
               body_ratio=kv["body_pitch_ratio"], can_ratio=kv["can_pitch_ratio"], sat=kv["sat_frac"],
               beta_true=BETA_NOM * d["Cld_scale"] / d["ixx_scale"], bear=d["bear_coul"], ixx_scale=d["ixx_scale"], cld=d["Cld_scale"])
    co = t[(t.phase == 5) & (t.vz > 0)].reset_index(drop=True)
    if len(co) < 100:
        return out
    # the manoeuvre
    ref = co.p_ref_dps.values; ue = co.u_exc_deg.values; p = co.p_dps.values; tt = co.t.values
    on = np.nonzero((ref != 0) | (ue != 0))[0]
    if len(on):
        i0, i1 = on[0], on[-1]
        out.update(q_start=co.qbar.values[i0], q_end=co.qbar.values[i1], t_man=tt[i1] - tt[i0],
                   defl_max_man=np.abs(co.delta0.values[i0:i1 + 1]).max(), p_max_man=np.abs(p[i0:i1 + 1]).max(),
                   pcan_max_man=np.abs(co.pcan_dps.values[i0:i1 + 300]).max())
        step = np.nonzero(ref > 0)[0]
        if len(step) > 20:
            s0, s1 = step[0], step[-1]; A = ref[s0]
            k90 = np.nonzero(p[s0:s1] >= 0.9 * A)[0]
            out.update(t90=(k90[0] / 200.0) if len(k90) else np.nan, ovs=(p[s0:s1 + 40].max() / A - 1.0),
                       err_hold=np.mean(A - p[s0 + int(0.6 * (s1 - s0)):s1]) / A)
        post = slice(i1 + 1, min(i1 + 1 + 200, len(p)))
        out.update(p_after_1s=np.abs(p[min(i1 + 200, len(p) - 1)]), p_rms_after=np.sqrt(np.mean(p[i1 + 1:] ** 2)) if i1 + 1 < len(p) else np.nan)
    # truth: the same regression on noise-free plant data (true canard, true slip, true q): what the plant does
    tq = co[co.qbar > 800]
    if len(tq) > 100:
        pt = np.radians(tq.p_dps.values); pdt = savgol_filter(pt, 9, 2, deriv=1, delta=0.005)
        dl = np.radians(tq[["delta0", "delta1", "delta2", "delta3"]].mean(axis=1).values)
        Xt = np.column_stack([tq.qbar.values * dl, tq.qbar.values * pt / tq.V.values, np.tanh((pt - np.radians(tq.pcan_dps.values)) / 0.5),
                              tq.qbar.values, np.ones(len(tq))])
        tht, *_ = np.linalg.lstsq(Xt, pdt, rcond=None)
        out.update(beta_true=tht[0], phi_true=tht[2])
    # the log: 100 Hz, gyro 0.1 dps, command quantised (30/127 deg) or full resolution
    lg = co.iloc[::2].reset_index(drop=True)
    lg["gyro_p_q"] = np.round(lg.gyro_p_dps * 10) / 10
    cmd = lg.roll_cmd.values * 30.0
    slip = np.radians(lg.gyro_p_q.values - np.round(lg.pcan_dps.values))     # with a can-spin sensor, 1 dps
    # onboard video (offline): fin-can roll relative to the body and one canard's angle, 60 fps,
    # 1 deg and 0.2 deg rms noise per frame, synchronised to the log (SYSID-01 section 9)
    rng = np.random.default_rng(int(seed) % 2**32)
    tv = np.arange(lg.t.values[0], lg.t.values[-1], 1 / 60.0)
    rel = np.cumsum(np.radians(lg.p_dps.values - lg.pcan_dps.values)) * 0.01          # body-can angle, rad
    rel_v = np.interp(tv, lg.t.values, rel) + np.radians(1.0) * rng.standard_normal(len(tv))
    slip_v = np.interp(lg.t.values, tv, np.gradient(savgol_filter(rel_v, 7, 2), 1 / 60.0))
    dl_true = lg[["delta0", "delta1", "delta2", "delta3"]].mean(axis=1).values
    sync = 0.008 * rng.standard_normal()                     # video-to-log sync by the horizon, about +/-1 frame
    dl_f = np.interp(tv + sync, lg.t.values, dl_true) + 0.2 * rng.standard_normal(len(tv))   # video frames
    dl_raw = np.interp(lg.t.values, tv, dl_f)
    cmd_q = np.round(cmd * 127 / 30) * 30 / 127
    dl_fit, out["video_act_gain"], out["video_act_n"], out["video_act_ta"] = act_from_video(cmd_q, tv, dl_f, lg.t.values)
    dl_fit_f = act_from_video(cmd, tv, dl_f, lg.t.values)[0]                # with the command logged finely (S14)
    for tag, q, sv, act in (("log", cmd_q, None, True), ("fine", cmd, None, True), ("fine_can", cmd, slip, True),
                            ("video_slip", cmd_q, slip_v, True), ("video_raw", dl_raw, slip_v, False), ("video", dl_fit, slip_v, False), ("video_fine", dl_fit_f, slip_v, False)):
        idn = identify(lg, q, sv, act)
        out.update({f"{tag}_{k}": v for k, v in idn.items()})
        out[f"{tag}_ratio"] = idn["beta"] / out["beta_true"]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default="./sitl_law1"); ap.add_argument("--kind", type=int, default=0)
    ap.add_argument("--amp", type=float, default=114.6); ap.add_argument("--T", type=float, default=0.8)
    ap.add_argument("--ua", type=float, default=2.0); ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--t0", type=float, default=0.25); ap.add_argument("--gap", type=float, default=0.2)
    ap.add_argument("--n", type=int, default=20); ap.add_argument("--out", required=True)
    ap.add_argument("--keep", type=int, default=0, help="keep the traces of the first k flights per motor")
    a = ap.parse_args()
    extra = dict(man_kind=a.kind, man_amp=a.amp, man_T=a.T, man_ua=a.ua, man_dt=a.dt, man_t0=a.t0, man_gap=a.gap)
    jobs = [(a.exe, m, 1000 + mi * 100000 + i, extra, i < a.keep) for mi, m in enumerate(MOTORS) for i in range(a.n)]
    with ProcessPoolExecutor(2) as ex:
        rows = list(ex.map(one, jobs))
    os.makedirs(a.out, exist_ok=True)
    df = pd.DataFrame(rows); df.to_csv(os.path.join(a.out, "man.csv"), index=False)
    def q(col, f=np.nanmedian):
        return float(f(df[col])) if col in df else float("nan")
    s = dict(args=vars(a), n=len(df),
             p_max_p95=float(np.nanpercentile(df.p_max, 95)), p_max_max=float(df.p_max.max()),
             body_ratio_max=float(df.body_ratio.max()), can_ratio_max=float(df.can_ratio.max()),
             p_rms_coast_p95=float(np.nanpercentile(df.p_rms_coast, 95)))
    for c in ("t90", "ovs", "err_hold", "p_after_1s", "p_rms_after", "q_start", "q_end", "defl_max_man", "pcan_max_man", "t_man"):
        if c in df:
            s[c] = [round(float(np.nanpercentile(df[c], x)), 3) for x in (5, 50, 95)]
    for tag in ("log", "fine", "fine_can", "video_slip", "video_raw", "video", "video_fine"):
        r = df[f"{tag}_ratio"]
        s[f"{tag}_beta_ratio_p5_50_95"] = [round(float(np.nanpercentile(r, x)), 3) for x in (5, 50, 95)]
        s[f"{tag}_beta_err_rms"] = round(float(np.sqrt(np.nanmean((r - 1) ** 2))), 3)
        s[f"{tag}_beta_se_rel_med"] = round(float(np.nanmedian(df[f"{tag}_beta_se"] / df[f"{tag}_beta"])), 3)
    json.dump(s, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print(json.dumps(s))


if __name__ == "__main__":
    main()
