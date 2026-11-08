"""FIG-01: roll-rate / roll-angle time histories across SITL Monte Carlo campaigns and hardware HITL runs.

Re-runs a sample of seeds from each campaign's runs.csv with a full trace (50 Hz), then plots every
trace with the p5-p50-p95 band and the worst cases highlighted, so the scalar statistics in MC-01 /
LD-01 can be checked against the behaviour that produced them.

    python trace_figs.py [--hitl DIR ...]      -> Documentation/Images/fig01/*.png
"""
import argparse, csv, glob, json, os, subprocess
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "fig01"))
TR = os.path.join(HERE, "runs", "traces")
TG = np.arange(0.0, 13.0, 0.02)                     # common time grid, s from ignition
I_BOOST, I_COAST, I_DESC = 4, 5, 6
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3})
CAMPS = [("L_active", "Active, build tolerance"), ("L_wind12", "Active, wind 0-12 m/s"),
         ("L_wide", "Active, wide build tolerance"), ("L_shadow", "Shadow (canards held at 0)"),
         ("L_fault_hardover", "One canard hard-over")]
FAULTS = [("V_fault_stuck", "stuck"), ("V_fault_slow", "slow (1/4 rate)"), ("V_fault_offset", "offset"),
          ("V_fault_float", "floating (unpowered)"), ("L_fault_hardover", "hard-over")]


def rows(c):
    return list(csv.DictReader(open(os.path.join(HERE, "runs", c, "runs.csv"))))


def failed(c):
    try:
        fs = json.load(open(os.path.join(HERE, "runs", c, "summary.json"))).get("fail_seeds", {})
        return {int(x) for v in fs.values() for x in v}
    except Exception:
        return set()


def pick(rr, n_rand, rng, fails=frozenset()):
    """random sample + worst by peak, by coast RMS, by lock-in ratio, and every failed run"""
    idx = set(rng.choice(len(rr), size=min(n_rand, len(rr)), replace=False).tolist())
    worst = set()
    for k in ("p_max_coast", "p_rms_coast", "p_max_boost", "body_pitch_ratio"):
        if k in rr[0]:
            worst |= set(np.argsort([-float(r[k]) for r in rr])[:3].tolist())
    worst |= {i for i, r in enumerate(rr) if int(float(r["seed"])) in fails}
    return sorted(idx | worst), worst


def run_trace(r, out):
    if os.path.exists(out):
        return out
    args = [os.path.join(HERE, "sitl"), f"seed={int(float(r['seed']))}", f"trace={out}"]
    args += [f"{k[3:]}={v}" for k, v in r.items() if k.startswith("in_") and v != ""]
    subprocess.run(args, capture_output=True, text=True, cwd=HERE)
    return out


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    i0 = np.argmax(d["phase"] >= I_BOOST)               # first frame of BOOST (ignition-relative clock is valid)
    i0 = max(0, np.argmax(d["t"] >= 0.0) if d["t"][i0] < 0 else i0)
    d = d[np.argmax(d["t"] >= 0.0):]
    t = d["t"]; p = d["p_dps"]
    phi = np.concatenate([[0.0], np.cumsum(0.5 * (p[1:] + p[:-1]) * np.diff(t))])
    tb = t[np.argmax(d["phase"] >= I_COAST)] if (d["phase"] >= I_COAST).any() else np.nan
    ta = t[np.argmax(d["phase"] >= I_DESC)] if (d["phase"] >= I_DESC).any() else np.nan
    defl = np.mean(np.abs(np.vstack([d[f"delta{k}"] for k in range(4)])), axis=0)          # trace is in degrees
    g = lambda y: np.interp(TG, t, y, right=np.nan)
    return dict(p=g(p), phi=g(phi), pcan=g(d["pcan_dps"]), defl=g(defl), q=g(d["qbar"]), tb=tb, ta=ta, t=t, praw=p,
                phase=d["phase"])


def gen(campaign, n_rand, seed=7):
    rr = rows(campaign)
    fails = failed(campaign) if campaign.startswith("L_") and "fault" not in campaign else set()
    sel, worst = pick(rr, n_rand, np.random.default_rng(seed), fails)
    os.makedirs(os.path.join(TR, campaign), exist_ok=True)
    jobs = [(rr[i], os.path.join(TR, campaign, f"{int(float(rr[i]['seed']))}.csv")) for i in sel]
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
        list(ex.map(lambda j: run_trace(*j), jobs))
    out = []
    for i, (r, path) in zip(sel, jobs):
        try:
            out.append(dict(row=r, worst=i in worst, failed=int(float(r["seed"])) in fails, **load(path)))
        except Exception:
            pass
    return out


def band(ax, tr, key, color="C0", label=None):
    Y = np.vstack([x[key] for x in tr])
    for x in tr:
        if not x["worst"]:
            ax.plot(TG, x[key], color="0.6", lw=0.4, alpha=0.5, zorder=1)
    lo, med, hi = (np.nanpercentile(Y, q, axis=0) for q in (5, 50, 95))
    ax.fill_between(TG, lo, hi, color=color, alpha=0.18, zorder=2, label="p5-p95 of traced runs")
    ax.plot(TG, med, color=color, lw=1.6, zorder=3, label="median")
    first = True
    for x in tr:
        if x["worst"]:
            ax.plot(TG, x[key], color="C3", lw=0.9, alpha=0.9, zorder=4,
                    label="worst-case runs (peak / RMS / lock-in / failed)" if first else None)
            first = False
    tb = np.nanmedian([x["tb"] for x in tr]); ta = np.nanmedian([x["ta"] for x in tr])
    for tv, s in ((tb, "burnout"), (ta, "apogee")):
        if np.isfinite(tv):
            ax.axvline(tv, color="k", ls=":", lw=0.8); ax.text(tv, 1.0, f" {s}", transform=ax.get_xaxis_transform(),
                                                                fontsize=7.5, va="top")


def fig_campaign(name, title, tr, rlim):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8.4), sharex=True)
    band(ax[0], tr, "p"); ax[0].set_ylabel("body roll rate (°/s)"); ax[0].set_ylim(-rlim, rlim)
    for lim in (240,):
        if lim < rlim:
            for s in (1, -1):
                ax[0].axhline(s * lim, color="C3", ls="--", lw=0.8)
            ax[0].text(0.1, lim, "R5a 240 °/s", color="C3", fontsize=7.5, va="bottom")
    band(ax[1], tr, "phi"); ax[1].set_ylabel("roll angle from liftoff (°)")
    band(ax[2], tr, "defl"); ax[2].set_ylabel("mean |canard deflection| (°)"); ax[2].set_xlabel("time from ignition (s)")
    ax[0].legend(loc="lower right", fontsize=7.5)
    n_w = sum(x["worst"] for x in tr)
    fig.suptitle(f"{title}: {len(tr)} traced SITL flights ({len(tr) - n_w} random + {n_w} worst-case)", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"sitl_roll_{name}.png"), dpi=150); plt.close(fig)


def fig_overview(all_tr):
    fig, axs = plt.subplots(2, 3, figsize=(15, 7.5))
    for ax, (c, title) in zip(axs.flat, CAMPS + [("_motors", "")]):
        if c not in all_tr:
            ax.axis("off"); continue
        band(ax, all_tr[c], "p"); ax.set_title(title); ax.set_ylim(-700, 700) if "hard" in c or "shadow" in c else ax.set_ylim(-260, 260)
        ax.set_ylabel("roll rate (°/s)"); ax.set_xlabel("time from ignition (s)"); ax.set_xlim(0, 13)
    ax = axs.flat[5]; ax.axis("on")
    for c, title in CAMPS:
        if c in all_tr:
            rms = [np.sqrt(np.nanmean(np.where((TG > x["tb"]) & (TG < x["ta"]), x["p"], np.nan) ** 2)) for x in all_tr[c]]
            ax.hist(rms, bins=np.linspace(0, 200, 41), histtype="step", lw=1.4, label=title)
    ax.set_xlabel("coast roll RMS from traces (°/s)"); ax.set_ylabel("runs"); ax.set_title("Coast RMS distribution (traced runs)")
    ax.legend(fontsize=7)
    fig.suptitle("Roll-rate time histories by campaign (y-axis wider for shadow and hard-over)", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_roll_overview.png"), dpi=150); plt.close(fig)


def fig_motors(tr):
    motors = sorted({os.path.basename(x["row"]["motor"]).replace(".csv", "") for x in tr})
    fig, axs = plt.subplots(2, 3, figsize=(15, 7), sharey=True)
    for ax, m in zip(axs.flat, motors):
        sub = [x for x in tr if os.path.basename(x["row"]["motor"]).replace(".csv", "") == m]
        band(ax, sub, "p"); ax.set_title(f"{m} ({len(sub)} runs)"); ax.set_ylim(-200, 200)
        ax.set_xlabel("time from ignition (s)"); ax.set_ylabel("roll rate (°/s)")
    fig.suptitle("Active, build tolerance: roll rate by motor", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_roll_by_motor.png"), dpi=150); plt.close(fig)


def fig_faults(ftr):
    fig, axs = plt.subplots(2, 3, figsize=(15, 7.5), sharex=True)
    for ax, (c, lab) in zip(axs.flat, FAULTS):
        if c not in ftr:
            ax.axis("off"); continue
        band(ax, ftr[c], "p"); ax.set_title(f"Servo fault: {lab} ({len(ftr[c])} runs)")
        ax.set_ylabel("roll rate (°/s)"); ax.set_xlabel("time from ignition (s)")
        ax.set_ylim(-1100, 1100) if "hard" in lab else ax.set_ylim(-300, 300)
    ax = axs.flat[5]
    for c, lab in FAULTS:
        if c in ftr:
            for x in ftr[c]:
                ax.plot(TG, x["q"] / 1000, color="0.7", lw=0.3)
    ax.set_title("Dynamic pressure (all fault runs)"); ax.set_ylabel("q (kPa)"); ax.set_xlabel("time from ignition (s)")
    fig.suptitle("Single-servo faults injected at a random time 0-6 s (re-run on the corrected plant)", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_roll_faults.png"), dpi=150); plt.close(fig)


def fig_validate(all_tr):
    """Recompute metrics from the traces. The Monte Carlo metrics use TRUTH windows (boost = thrust > 0,
    coast = thrust off and climbing). The board's own phases differ: COAST runs on past true apogee (fins
    still active) until apogee is detected, and nothing covers DESCENT before the chute opens."""
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    post = {}
    for c, title in CAMPS:
        if c not in all_tr:
            continue
        a, b, e, f, g = [], [], [], [], []
        for x in all_tr[c]:
            t, p, ph = x["t"], np.abs(x["praw"]), x["phase"]
            act = (ph == I_BOOST) | (ph == I_COAST)
            co = ph == I_COAST
            a.append(max(float(x["row"]["p_max_boost"]), float(x["row"]["p_max_coast"]))); b.append(p[act].max())
            e.append(float(x["row"]["p_rms_coast"])); f.append(np.sqrt(np.mean(p[co] ** 2)) if co.any() else np.nan)
            ds = (ph == I_DESC) & (t <= x["ta"] + 3.0)
            g.append(p[ds].max() if ds.any() else np.nan)
        ax[0].scatter(a, b, s=9, label=title); ax[1].scatter(e, f, s=9, label=title)
        post[title] = g
    for k, lab in enumerate(("peak |p|, active flight", "coast roll RMS")):
        lim = max(ax[k].get_xlim()[1], ax[k].get_ylim()[1]); ax[k].plot([0, lim], [0, lim], "k--", lw=0.8)
    ax[0].set_xlabel("Monte Carlo metric: max(p_max_boost, p_max_coast), truth windows (°/s)")
    ax[0].set_ylabel("trace: max |p| over board BOOST + COAST (°/s)")
    ax[0].set_title("Points above the line: roll the metric window misses\n(between true apogee and detected apogee, fins still active)", fontsize=8.5)
    ax[1].set_xlabel("Monte Carlo metric p_rms_coast, truth window (°/s)"); ax[1].set_ylabel("trace: RMS over board COAST (°/s)")
    ax[1].set_title("Coast RMS: the two windows agree", fontsize=8.5)
    ax[0].legend(fontsize=7)
    bins = np.linspace(0, 400, 41)
    for title, g in post.items():
        if "shadow" in title.lower() or "hard" in title.lower():
            continue
        ax[2].hist(np.array(g)[np.isfinite(g)], bins=bins, histtype="step", lw=1.4, label=title)
    ax[2].set_xlabel("max |p| in the first 3 s of DESCENT (fins OFF, before/at deployment) (°/s)"); ax[2].set_ylabel("runs")
    ax[2].set_title("Not covered by any requirement: post-apogee spin-up", fontsize=8.5); ax[2].legend(fontsize=7)
    fig.suptitle("Cross-check of the summary statistics against the traced time histories", weight="bold", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_metric_crosscheck.png"), dpi=150); plt.close(fig)


def fail_reason(c):
    try:
        fs = json.load(open(os.path.join(HERE, "runs", c, "summary.json"))).get("fail_seeds", {})
    except Exception:
        return {}
    out = {}
    for k, v in fs.items():
        for x in v:
            out.setdefault(int(x), []).append(k.split("_")[0])
    return out


def fig_worst(all_tr):
    sel, seen = [], set()
    def add(lab, x):
        sd = int(float(x["row"]["seed"]))
        if sd not in seen:
            seen.add(sd); sel.append((lab, x))
    for c, title in CAMPS[:3]:
        why = fail_reason(c)
        tr = all_tr.get(c, [])
        roll_fail = [x for x in tr if any(r.startswith("R5") or r.startswith("R6") for r in why.get(int(float(x["row"]["seed"])), []))]
        for x in roll_fail[:2]:
            add(f"{title}, seed {int(float(x['row']['seed']))}: FAILED {'+'.join(why[int(float(x['row']['seed']))])}", x)
        if tr:
            w = max(tr, key=lambda x: float(x["row"]["p_rms_coast"]))
            add(f"{title}, seed {int(float(w['row']['seed']))}: worst coast RMS", w)
            w = max(tr, key=lambda x: max(float(x["row"]["p_max_boost"]), float(x["row"]["p_max_coast"])))
            add(f"{title}, seed {int(float(w['row']['seed']))}: worst peak", w)
    sel = sel[:6]
    fig, axs = plt.subplots(len(sel), 1, figsize=(10, 2.1 * len(sel) + 0.6), sharex=True)
    for ax, (lab, x) in zip(np.atleast_1d(axs), sel):
        ax.plot(TG, x["p"], "C0", lw=1, label="body roll rate (°/s)")
        ax.plot(TG, x["pcan"] / 5, "C2", lw=0.8, label="fin-can spin / 5 (°/s)")
        a2 = ax.twinx(); a2.plot(TG, x["defl"], "C3", lw=0.8, label="mean |δ|"); a2.set_ylabel("mean |δ| (°)", color="C3"); a2.grid(False)
        r = x["row"]
        ax.set_title(f"{lab}: {os.path.basename(r['motor'])[:-4]}, Cl_δ×{float(r['in_Cld_scale']):.2f}, Ixx×{float(r['in_ixx_scale']):.2f}, "
                     f"τ_d {float(r['in_sv_tau']) * 1000:.0f} ms, cant {float(r['in_can_cant_deg']):+.2f}°, wind {float(r['in_wind']):.1f} m/s",
                     fontsize=8)
        ax.set_ylabel("°/s")
        for v in (x["tb"], x["ta"]):
            if np.isfinite(v):
                ax.axvline(v, color="k", ls=":", lw=0.7)
    np.atleast_1d(axs)[0].legend(fontsize=7, loc="upper right"); np.atleast_1d(axs)[-1].set_xlabel("time from ignition (s)  (dotted: burnout, apogee)")
    fig.suptitle("Worst and failed flights in detail (R9 q-hat failures in the wind campaign are not roll failures)", weight="bold", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_roll_worst_cases.png"), dpi=150); plt.close(fig)


def hitl(dirs):
    runs = []
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, "*_flight_*.csv"))):
            x = np.genfromtxt(f, delimiter=",", names=True, dtype=None, encoding="utf-8")
            ph = np.array(x["phase"]); t = x["t"].astype(float)
            k = (t >= 0)
            t = t[k]; p = x["p_true_dps"][k]; ph = ph[k]
            phi = np.concatenate([[0.0], np.cumsum(0.5 * (p[1:] + p[:-1]) * np.diff(t))])
            name = os.path.basename(f).split("_flight_")[1][:-4]
            build = "dither build" if "103621" in d else "pre-dither build"
            runs.append(dict(name=name, build=build, t=t, p=p, phi=phi, d=x["delta_true_deg"][k], ph=ph,
                             pm=x["gz_meas_dps"][k]))
    return runs


def fig_hitl(runs, sitl_nom):
    def tmark(ax, r):
        for s, c in (("COAST", "burnout"), ("DESCENT", "apogee")):
            i = np.argmax(r["ph"] == s)
            if (r["ph"] == s).any():
                ax.axvline(r["t"][i], color="k", ls=":", lw=0.7)
    names = []
    for r in runs:
        if r["name"] not in names:
            names.append(r["name"])
    fig, axs = plt.subplots(len(names), 2, figsize=(13, 1.75 * len(names) + 0.8), sharex="col")
    for i, n in enumerate(names):
        for r in runs:
            if r["name"] != n:
                continue
            ls = "-" if r["build"] == "dither build" else "--"
            lim = 13 if "i205w" not in n else 20
            k = r["t"] < lim
            axs[i, 0].plot(r["t"][k], r["p"][k], ls, lw=0.9, label=r["build"])
            axs[i, 1].plot(r["t"][k], r["phi"][k], ls, lw=0.9)
            tmark(axs[i, 0], r)
        axs[i, 0].set_ylabel(n.replace("_", " "), fontsize=8, rotation=0, ha="right", va="center")
        axs[i, 0].legend(fontsize=6.5, loc="upper right")
    axs[0, 0].set_title("true body roll rate (°/s), ascent"); axs[0, 1].set_title("roll angle from liftoff (°)")
    axs[-1, 0].set_xlabel("time from ignition (s)"); axs[-1, 1].set_xlabel("time from ignition (s)")
    fig.suptitle("Hardware HITL (FC-V4 in the loop), 8 Nov: all cases, both builds", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "hitl_roll_all_cases.png"), dpi=150); plt.close(fig)

    nom = [r for r in runs if r["name"] == "G79W_nominal" and r["build"] == "dither build"]
    if nom:
        r = nom[0]
        fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
        if sitl_nom:
            band(ax[0], sitl_nom, "p", color="C2"); band(ax[1], sitl_nom, "phi", color="C2")
        k = r["t"] < 13
        ax[0].plot(r["t"][k], r["p"][k], "k", lw=1.2, label="HITL (hardware), G79W nominal, dither build")
        ax[0].plot(r["t"][k], r["pm"][k], color="C1", lw=0.5, alpha=0.7, label="gyro measured by the FC")
        ax[1].plot(r["t"][k], r["phi"][k], "k", lw=1.2)
        ax[2].plot(r["t"][k], r["d"][k], "k", lw=1); ax[2].set_ylabel("true canard deflection (°)")
        ax[0].set_ylabel("roll rate (°/s)"); ax[1].set_ylabel("roll angle (°)"); ax[2].set_xlabel("time from ignition (s)")
        ax[0].legend(fontsize=7, loc="lower right"); ax[0].set_ylim(-200, 200)
        fig.suptitle("HITL nominal flight against the SITL G79W dispersion band (green)", weight="bold")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "hitl_vs_sitl_nominal.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 3.4))
    for r in runs:
        if r["build"] == "dither build":
            ax.plot(r["t"], r["p"], lw=0.8, label=r["name"])
            tmark(ax, r)
    ax.set_xlabel("time from ignition (s)"); ax.set_ylabel("true roll rate (°/s)")
    ax.text(0.5, 0.92, "Descent: fins OFF and the HITL plant has no parachute, so the canards' last offset spins the body. "
            "Rig artefact, not flight behaviour.", transform=ax.transAxes, ha="center", fontsize=7.5,
            bbox=dict(fc="white", ec="0.7"))
    ax.legend(fontsize=7, loc="lower right"); ax.set_title("HITL whole flight (dither build)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "hitl_roll_whole_flight.png"), dpi=150); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--hitl", nargs="*", default=[])
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    all_tr = {c: gen(c, a.n) for c, _ in CAMPS}
    ftr = {c: gen(c, 25) for c, _ in FAULTS if os.path.exists(os.path.join(HERE, "runs", c, "runs.csv"))}
    for c, title in CAMPS:
        fig_campaign(c, title, all_tr[c], 1100 if "hard" in c else (900 if "shadow" in c else 300))
    fig_overview(all_tr)
    mot = gen("L_active", 150, seed=11)
    fig_motors(mot)
    fig_faults(ftr)
    fig_validate(all_tr)
    fig_worst(all_tr)
    if a.hitl:
        nom = [x for x in mot if "G79W" in x["row"]["motor"]]
        fig_hitl(hitl(a.hitl), nom)
    print("figures ->", OUT)


if __name__ == "__main__":
    main()
