import json, numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
R = "runs/"
OI = dict(blue="#0072B2", verm="#D55E00", orange="#E69F00", sky="#56B4E9", green="#009E73", grey="#7F7F7F", black="#000000", purple="#CC79A7")
def S(d): return json.load(open(R + d + "/summary.json"))
# ---------------- Fig 1: what limits roll performance
rows = [("Current law (c = 15), BENCH-02 servo", "NV5a_bench02_116", "law1"),
        ("  same, perfect actuator (no delay, threshold, lash)", "O1_ideal_act", "law1"),
        ("  same, no bearing friction", "O2_nofric", "law1"),
        ("  c = 22, BENCH-02 servo", "O3_c22", "law1"),
        ("  c = 22, perfect actuator", "O4_ideal_c22", "law1"),
        ("Observer law (ESO), beta model nominal", "E1_eso", "eso"),
        ("  ESO, beta model 2x too high", "E2_eso_b2x", "eso"),
        ("  ESO, beta model 2x too low (4/600 unstable)", "E3_eso_bhalf", "eso")]
fig, ax = plt.subplots(1, 2, figsize=(13, 5.2), sharey=True)
y = np.arange(len(rows))[::-1]
for (lab, d, kind), yy in zip(rows, y):
    p = S(d)["pct"]; col = OI["blue"] if kind == "law1" else OI["verm"]; hatch = None if kind == "law1" else "//"
    ax[0].barh(yy, p["p_rms_coast"]["p95"], color=col, hatch=hatch, edgecolor="white")
    ax[0].plot(p["p_rms_coast"]["max"], yy, "k|", ms=12, mew=2)
    ax[0].text(p["p_rms_coast"]["p95"] + 0.4, yy + 0.18, f'{p["p_rms_coast"]["p95"]:.1f}', fontsize=8, va="center")
    ax[1].barh(yy, p["p_max_boost"]["p95"], color=col, hatch=hatch, edgecolor="white")
    ax[1].plot(min(p["p_max_boost"]["max"], 330), yy, "k|", ms=12, mew=2)
    ax[1].text(p["p_max_boost"]["p95"] + 3, yy + 0.18, f'{p["p_max_boost"]["p95"]:.0f}', fontsize=8, va="center")
ax[0].set_yticks(y); ax[0].set_yticklabels([r[0] for r in rows], fontsize=9)
ax[0].axvline(25, color=OI["grey"], ls="--", lw=1); ax[0].text(25.3, y[-1] - 0.6, "goal G2", fontsize=8, color="#444444")
ax[1].axvline(120, color=OI["grey"], ls="--", lw=1); ax[1].text(122, y[-1] - 0.6, "goal G1", fontsize=8, color="#444444")
ax[1].axvline(240, color=OI["black"], ls=":", lw=1); ax[1].text(242, y[-1] - 0.6, "R5a", fontsize=8)
ax[0].set_xlabel("coast roll-rate RMS, °/s: bar p95, tick max"); ax[1].set_xlabel("peak roll rate, °/s: bar p95, tick max (clipped at 330)")
ax[0].set_xlim(0, 40); ax[1].set_xlim(0, 340)
fig.suptitle("Where the roll error comes from: 600 flights each, build tolerance, BENCH-02 servo at the corrected scale.\n"
             "A perfect actuator gains 1 %; cancelling the disturbance torque (observer law) gains 4-5x", fontsize=11)
fig.tight_layout(); fig.savefig("../../../../Documentation/Images/sysid01/sysid_bounds.png", dpi=110); plt.close(fig)

# ---------------- Fig 2: the 0 -> 2 -> 0 rad/s step on one flight, both laws
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for f, lab, col, ls in (("runs/sysid_traces/k1_201000.csv", "current law (c/q + trim)", OI["blue"], "-"), ("runs/sysid_traces/k1eso_201000.csv", "observer law (ESO)", OI["verm"], "--")):
    t = pd.read_csv(f); c = t[(t.phase == 5) & (t.vz > 0)]; t0 = c.t.iloc[0]; c = c[c.t < t0 + 4.5]; tt = c.t - t0
    if "k1_" in f:
        ax[0].plot(tt, c.p_ref_dps, color=OI["black"], lw=1.2, label="reference")
        ax[3].plot(tt, c.qbar_est / 1000, color=OI["black"], lw=1.2, label="q estimate")
    ax[0].plot(tt, c.p_dps, color=col, ls=ls, lw=1.6, label=lab)
    ax[1].plot(tt, c[["delta0", "delta1", "delta2", "delta3"]].mean(axis=1), color=col, ls=ls, lw=1.4, label=lab)
    ax[2].plot(tt, c.pcan_dps, color=col, ls=ls, lw=1.4, label=lab)
ax[0].set_ylabel("body roll rate, °/s"); ax[0].legend(fontsize=8, loc="upper right")
ax[1].set_ylabel("canard (roll mode), °"); ax[2].set_ylabel("fin-can spin, °/s"); ax[3].set_ylabel("q, kPa"); ax[3].set_xlabel("time from coast entry (detected burnout), s")
ax[2].text(0.02, 0.85, "the can is dragged along by bearing friction, then drags the body back", transform=ax[2].transAxes, fontsize=8)
for a in ax: a.grid(alpha=0.3)
fig.suptitle("Step test 0 -> 2 rad/s (1 s) -> 0, G79W, build-tolerance draw (seed 201000)", fontsize=11)
fig.tight_layout(); fig.savefig("../../../../Documentation/Images/sysid01/sysid_step.png", dpi=110); plt.close(fig)

# ---------------- Fig 3: schedules: identification quality and cost
sch = [("none (natural excitation)", "K0_none"), ("rate step +2 rad/s, 1 s", "K1_step2_v2"), ("rate doublet ±2 rad/s, 0.8 s each", "K2_doublet2_v2"),
       ("doublet, started 0.55 s later", "K2b_doublet2_late"), ("canard 3-2-1-1, 0.5°", "K3_3211"), ("3-2-1-1 then doublet", "K4_3211_doublet"),
       ("canard multisine, 2 s", "K5_multisine"), ("combined, torque-scaled, guarded", "K7_combined")]
fig, ax = plt.subplots(1, 2, figsize=(13, 5.4), sharey=True, gridspec_kw=dict(width_ratios=[1.6, 1]))
y = np.arange(len(sch))[::-1]
for (lab, d), yy in zip(sch, y):
    df = pd.read_csv(R + d + "/man.csv"); s = S(d)
    for k, (tag, col, mk, off) in enumerate((("log", OI["sky"], "o", 0.22), ("fine", OI["blue"], "s", 0.0), ("fine_can", OI["green"], "D", -0.22))):
        if f"{tag}_beta" not in df: continue
        r = df[f"{tag}_beta"] / df.beta_true
        lo, med, hi = np.nanpercentile(r, [5, 50, 95])
        ax[0].plot([lo, hi], [yy + off] * 2, color=col, lw=2.5)
        ax[0].plot(med, yy + off, mk, color=col, ms=6, label={"log": "log as now (100 Hz, 0.24° command)", "fine": "command logged at full resolution",
                                                               "fine_can": "plus a fin-can spin sensor"}[tag] if yy == y[2] else None)
    ax[1].barh(yy, s["p_max_p95"], color=OI["orange"]); ax[1].plot(s["p_max_max"], yy, "k|", ms=12, mew=2)
ax[0].axvline(1.0, color=OI["black"], lw=1); ax[0].set_xlim(0.3, 1.4)
ax[0].set_yticks(y); ax[0].set_yticklabels([s[0] for s in sch], fontsize=9)
ax[0].set_xlabel("estimated / true canard effectiveness beta (5-95 %, marker median)"); ax[0].legend(fontsize=8, loc="upper right")
ax[1].axvline(240, color=OI["black"], ls=":", lw=1); ax[1].set_xlabel("peak roll rate, °/s (bar p95, tick max)")
fig.suptitle("In-flight test schedules, current roll law, 120 flights each: what one test tells you about beta, and what it costs", fontsize=11)
fig.tight_layout(); fig.savefig("../../../../Documentation/Images/sysid01/sysid_schedules.png", dpi=110); plt.close(fig)
print("ok")
