"""FIG-02 F7: how much servo degradation each roll law tolerates (SITL 6DOF, 600 flights per bar, build tolerance).

Run from Tools/sitl: python fig02_servo.py OUTDIR   (reads runs/*/runs.csv and summary.json)
"""
import json, os, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, VERM = "#0072B2", "#D55E00"
CASES = [  # label, law-2 run, law-1 run (or None)
    ("BENCH-02 as measured\n9.3 ms, 130 rad/s, ζ 0.76; 0.15° / 0.12°", "G1_fl1_final", None),
    ("slow dynamics only\n30 ms, 50 rad/s, ζ 0.5", "W6_fl1_slowdyn_only", None),
    ("dead band + lash only\n0.40° / 0.29°", "W7_fl1_dblash_only", None),
    ("21 ms, 60 rad/s, ζ 0.6\n+ 0.20° / 0.20°", "X1_fl1_slow_db02", None),
    ("21 ms, 60 rad/s, ζ 0.6\n+ 0.40° / 0.29°", "X3_fl1_slow_db04", "X3L_law1_slow_db04"),
    ("HITL 'servo_worst'\n30 ms, 50 rad/s, ζ 0.5 + 0.40° / 0.29°", "W1_fl1_servoworst", "W2_law1_servoworst"),
]


def stats(run):
    s = json.load(open(os.path.join("runs", run, "summary.json")))
    d = pd.read_csv(os.path.join("runs", run, "runs.csv"))
    pk = np.maximum(d.p_max_boost, d.p_max_coast)
    return s["all_pass_pct"], float(pk.max()), int((pk > 400).sum()), float(np.percentile(d.p_rms_coast, 95))


def main(out):
    fig, axs = plt.subplots(1, 3, figsize=(14, 5.6), sharey=True)
    y = np.arange(len(CASES))[::-1]; h = 0.36
    for i, (lab, r2, r1) in enumerate(CASES):
        a2 = stats(r2)
        for k, ax in enumerate(axs):
            v = (a2[0], a2[3], a2[1])[k]
            ax.barh(y[i] + (h / 2 if r1 else 0), v, height=h, color=VERM)
            ax.text(v * (1.03 if k == 2 else 1) + (0.3 if k < 2 else 0), y[i] + (h / 2 if r1 else 0),
                    f"{v:.1f}" if k < 2 else f"{v:.0f}" + (f"  ({a2[2]} diverged)" if a2[2] else ""), va="center", fontsize=7.5)
            if r1:
                a1 = stats(r1); v1 = (a1[0], a1[3], a1[1])[k]
                ax.barh(y[i] - h / 2, v1, height=h, color=BLUE)
                ax.text(v1 * (1.03 if k == 2 else 1) + (0.3 if k < 2 else 0), y[i] - h / 2, f"{v1:.1f}" if k < 2 else f"{v1:.0f}", va="center", fontsize=7.5)
    axs[0].set_yticks(y, [c[0] for c in CASES], fontsize=8)
    axs[0].set_xlabel("all requirements pass (%)"); axs[0].set_xlim(75, 104)
    axs[1].set_xlabel("coast roll RMS, p95 (°/s)"); axs[1].axvline(45, color="#444", ls="--", lw=0.8); axs[1].text(45.5, len(CASES) - 0.6, "R5b 45", fontsize=7)
    axs[2].set_xscale("log"); axs[2].set_xlim(100, 20000); axs[2].set_xlabel("worst peak roll in 600 flights (°/s, log)")
    axs[2].axvline(240, color="#444", ls="--", lw=0.8); axs[2].text(250, len(CASES) - 0.6, "R5a 240", fontsize=7)
    for ax in axs:
        [ax.spines[s].set_visible(False) for s in ("top", "right")]; ax.grid(axis="x", alpha=0.25)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=VERM, label="flight-1 build (law 2, observer)"), Patch(color=BLUE, label="law 1 (trim integrator)")],
               loc="lower center", ncol=2, frameon=False, fontsize=8.5, bbox_to_anchor=(0.6, -0.04))
    fig.suptitle("Servo degradation the roll laws tolerate (SITL 6DOF, 600 flights each; servo: dead time, ω_n, ζ; dead band / lash)",
                 fontsize=10.5, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_servo_robustness.png"), dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(sys.argv[-1] if len(sys.argv) > 1 else ".", exist_ok=True)
    main(sys.argv[1])
