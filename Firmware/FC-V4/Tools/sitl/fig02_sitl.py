"""FIG-02 F3, F4: one G79W flight (SITL seed 201029, build tolerance) on the flight-1 build and on law 1.

Traces come from sitl.c (trace=..., trace_dec=1, 200 Hz). fin0 is the allocated command (normalised,
x30 = deg) and shows the null-axis dither; delta0..3 are the canards after the servo model (deg).
Run from Tools/sitl: python fig02_sitl.py DATADIR OUTDIR   (makes the two traces in DATADIR if missing)
"""
import os, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, VERM, SKY, PURPLE, YEL, GREY = "#0072B2", "#E69F00", "#009E73", "#D55E00", "#56B4E9", "#CC79A7", "#F0E442", "#8C8C8C"
R2D = 57.29577951308232


def load(d, tag):
    t = pd.read_csv(os.path.join(d, f"sitl_201029_{tag}.csv"))
    t["dmean"] = t[["delta0", "delta1", "delta2", "delta3"]].mean(axis=1)
    return t


def rt_window(t):
    a = t[(t.p_ref_dps != 0) | (t.u_exc_deg != 0)]
    return (a.t.min(), a.t.max()) if len(a) else (None, None)


def phase_marks(ax, t, txt=False):
    tb = t[t.phase == 5].t.min(); ta = t[t.phase == 6].t.min()
    for x, lab in ((0.0, "liftoff"), (tb, "burnout\n(COAST)"), (ta, "apogee\n(DESCENT)")):
        ax.axvline(x, color="#999", lw=0.8, ls=":")
        if txt:
            ax.text(x + 0.05, 1.0, lab, transform=ax.get_xaxis_transform(), fontsize=7.5, va="top", color="#555")


def make_traces(d):
    import subprocess, mc
    os.makedirs(d, exist_ok=True)
    runs = pd.read_csv(os.path.join("runs", "G1_fl1_final", "runs.csv")); r = runs[runs.seed == 201029].iloc[0]
    args = ["seed=201029"] + [f"{c[3:]}={r[c]}" for c in runs.columns if c.startswith("in_") and pd.notna(r[c])] + ["trace_dec=1"]
    for tag, var in (("fl1", ""), ("law1", "-DCTRL_ROLL_LAW=1 -DCFG_ROLL_TEST=0")):
        p = os.path.join(d, f"sitl_201029_{tag}.csv")
        if not os.path.exists(p):
            subprocess.run([mc.build(var)] + args + [f"trace={p}"], check=True, capture_output=True)


def main(d, out):
    make_traces(d)
    f1, l1 = load(d, "fl1"), load(d, "law1")
    r0, r1 = rt_window(f1)
    fig, axs = plt.subplots(5, 1, figsize=(11, 12.5), sharex=True, gridspec_kw=dict(height_ratios=[1.3, 1.1, 1.0, 0.9, 0.9]))
    xl = (-0.6, 10.2)
    for ax in axs:
        ax.set_xlim(*xl); ax.grid(alpha=0.25); [ax.spines[s].set_visible(False) for s in ("top", "right")]
        ax.axvspan(r0, r1, color=PURPLE, alpha=0.10, lw=0)
        phase_marks(ax, f1)
    phase_marks(axs[0], f1, txt=True)
    a = axs[0]
    a.plot(l1.t, l1.p_dps, color=GREY, lw=1.0, label="law 1 (trim integrator), no test")
    a.plot(f1.t, f1.p_dps, color=BLUE, lw=1.2, label="flight-1 build (observer + roll test)")
    a.plot(f1.t, f1.p_ref_dps, color="#222", lw=1.0, ls="--", label="p_ref (roll test doublet)")
    a.set_ylabel("body roll rate (°/s)"); a.legend(loc="lower left", fontsize=8, frameon=False, ncol=1)
    a.text(r0 + 0.02, 0.02, "roll test", transform=a.get_xaxis_transform(), color=PURPLE, fontsize=8, va="bottom")
    a.set_title("One G79W flight (SITL 6DOF, build tolerance, seed 201029): law 1 vs the flight-1 build", fontsize=10.5, loc="left")

    a = axs[1]
    a.plot(f1.t, f1.fin0 * 30.0, color=SKY, lw=0.6, label="fin 0 command (incl. ±0.17° dither)")
    a.plot(l1.t, l1.dmean, color=GREY, lw=1.0, label="law 1: mean canard")
    a.plot(f1.t, f1.dmean, color=BLUE, lw=1.2, label="flight-1: mean canard (after servo)")
    a.set_ylabel("canard (°)"); a.legend(loc="upper right", fontsize=8, frameon=False)

    a = axs[2]
    a.plot(l1.t, l1.trim * R2D, color=GREY, lw=1.0, label="law 1: trim integrator")
    on = f1.phase.isin([4, 5]) & (f1.qbar_est >= 1000.0)
    off = f1.phase.isin([4, 5]) & (f1.qbar_est < 1000.0) & (f1.t > 1.0)
    a.plot(f1.t, np.where(on, f1.trim * R2D, np.nan), color=ORANGE, lw=1.3, label="flight-1: observer −f̂/(β̂q̂) (q̂ ≥ 1 kPa)")
    a.plot(f1.t, np.where(off, f1.trim * R2D, np.nan), color=ORANGE, lw=1.1, ls="--", label="flight-1: law-1 trim after the hand-over")
    a.set_ylabel("disturbance\ncancellation (°)"); a.legend(loc="upper right", fontsize=8, frameon=False)

    a = axs[3]
    a.plot(l1.t, l1.pcan_dps, color=GREY, lw=1.0, label="law 1")
    a.plot(f1.t, f1.pcan_dps, color=GREEN, lw=1.1, label="flight-1")
    a.set_ylabel("fin-can spin\n(°/s)"); a.legend(loc="lower right", fontsize=8, frameon=False)

    a = axs[4]
    a.plot(f1.t, f1.qbar / 1000, color="#222", lw=1.1, label="true q")
    a.plot(f1.t, f1.qbar_est / 1000, color=VERM, lw=1.0, ls="--", label="q̂ (NAV-01)")
    for y, lab, va, dy in ((3.0, "test start ≥ 3 kPa", "bottom", 0.08), (1.2, "test abort 1.2 kPa", "bottom", 0.08), (1.0, "observer gate 1 kPa", "top", -0.08)):
        a.axhline(y, color="#888", lw=0.7, ls="-.")
        a.text(9.9, y + dy, lab, fontsize=7, ha="right", va=va, color="#555")
    a.set_ylabel("q (kPa)"); a.set_xlabel("time from ignition (s)"); a.legend(loc="upper right", fontsize=8, frameon=False)
    a.set_ylim(0, 6.5)
    rms = lambda t: np.sqrt(np.mean(t[(t.phase == 5) & ((t.t < r0 - 0.01) | (t.t > r1 + 1.0))].p_dps ** 2))
    axs[0].text(0.99, 0.97, f"coast roll RMS outside the test: law 1 {rms(l1):.1f} °/s, flight-1 {rms(f1):.1f} °/s",
                transform=axs[0].transAxes, ha="right", va="top", fontsize=8.5,
                bbox=dict(fc="white", ec="#ccc", pad=3))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_flight1_time_history.png"), dpi=160, bbox_inches="tight")

    # zoom on the test
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True, gridspec_kw=dict(height_ratios=[1.3, 1.0, 0.8]))
    z = f1[(f1.t > r0 - 0.4) & (f1.t < r1 + 1.2)]
    for ax in axs:
        ax.grid(alpha=0.25); [ax.spines[s].set_visible(False) for s in ("top", "right")]
        ax.axvspan(r0, r1, color=PURPLE, alpha=0.08, lw=0)
    exc_end = z[z.u_exc_deg != 0].t.max()
    for ax in axs:
        ax.axvline(exc_end, color="#aaa", lw=0.8, ls=":")
    axs[0].plot(z.t, z.p_ref_dps, color="#222", lw=1.0, ls="--", label="p_ref")
    axs[0].plot(z.t, z.p_dps, color=BLUE, lw=1.3, label="body roll rate p")
    axs[0].plot(z.t, z.pcan_dps, color=GREEN, lw=1.0, label="fin-can spin")
    axs[0].set_ylabel("°/s"); axs[0].legend(loc="lower left", fontsize=8, frameon=False)
    axs[0].text(r0 + 0.02, 0.97, "3-2-1-1 at the canard", transform=axs[0].get_xaxis_transform(), fontsize=8, va="top", color=PURPLE)
    axs[0].text(r1 + 0.06, 0.97, "← rate doublet\n±2 rad/s, 0.6 s each", transform=axs[0].get_xaxis_transform(), fontsize=8, va="top", color=PURPLE)
    axs[0].set_title(f"Flight-1 roll test, same flight (q̂ at start {z.qbar_est[z.u_exc_deg != 0].iloc[0]:.0f} Pa)", fontsize=10.5, loc="left")
    axs[1].plot(z.t, z.u_exc_deg, color=PURPLE, lw=1.2, label="u_exc (added after the law)")
    axs[1].plot(z.t, z.fin0 * 30.0, color=SKY, lw=0.6, label="fin 0 command (incl. dither)")
    axs[1].plot(z.t, z.dmean, color=BLUE, lw=1.2, label="mean canard (after servo)")
    axs[1].set_ylabel("canard (°)"); axs[1].legend(loc="lower left", fontsize=8, frameon=False, ncol=3)
    axs[2].plot(z.t, z.qbar_est / 1000, color=VERM, lw=1.1, label="q̂")
    axs[2].axhline(1.2, color="#888", lw=0.7, ls="-.")
    axs[2].text(z.t.max(), 1.25, "abort 1.2 kPa", ha="right", fontsize=7, color="#555")
    axs[2].set_ylabel("q̂ (kPa)"); axs[2].set_xlabel("time from ignition (s)"); axs[2].set_ylim(0, None)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_roll_test_zoom.png"), dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(sys.argv[-1] if len(sys.argv) > 1 else ".", exist_ok=True)
    main(sys.argv[1], sys.argv[2])
