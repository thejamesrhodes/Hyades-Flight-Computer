"""FIG-02 F5, F6: hardware-in-the-loop (FC-V4 board, PC plant hitl_model.py), G79W nominal case.

Compares the 8 Nov law-1 build with dither (campaign-20261108-103621) and the 15 Nov flight-1 build
with the dither gate (campaign-20261115-113108). fin* are the board's commands (normalised, x30 = deg);
p_true_dps, delta_true_deg, qbar_true are the plant's truth; qbar is the board's q-hat.
Run: python fig02_hitl.py HITLDIR OUTDIR
"""
import os, sys, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, VERM, SKY, PURPLE, GREY = "#0072B2", "#E69F00", "#009E73", "#D55E00", "#56B4E9", "#CC79A7", "#8C8C8C"
OLD, NEW = "campaign-20261108-103621", "campaign-20261115-113108"


def load(d, camp, case="nominal"):
    f = glob.glob(os.path.join(d, camp, f"*_flight_G79W_{case}.csv"))[0]
    t = pd.read_csv(f).sort_values("frame").reset_index(drop=True)
    i0 = t.index[t.phase == "BOOST"][0]
    t["tt"] = (t.frame - t.frame[i0]) * 0.005          # s from the liftoff frame (board frames, 200 Hz)
    return t


def dither_fig(d, out):
    o, n = load(d, OLD), load(d, NEW)
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.2), sharey="row", gridspec_kw=dict(width_ratios=[1, 1]))
    for col, (t, lab) in enumerate(((o, "8 Nov build (law 1, dither always on)"), (n, "15 Nov build (dither gated to BOOST/COAST)"))):
        for row, (a0, a1, ttl) in enumerate(((-0.30, -0.10, "on the pad (READY, bench actuation)"), (0.02, 0.22, "after liftoff (BOOST)"))):
            ax = axs[row, col]
            z = t[(t.tt >= a0) & (t.tt <= a1)]
            for k, c, ls in ((0, BLUE, "-"), (1, VERM, "--"), (2, GREEN, ":"), (3, PURPLE, "-.")):
                ax.step(z.tt, z[f"fin{k}"] * 30.0, where="post", color=c, lw=1.1, ls=ls, label=f"fin {k}")
            ax.set_title(f"{lab}\n{ttl}", fontsize=9, loc="left")
            ax.grid(alpha=0.25); [ax.spines[s].set_visible(False) for s in ("top", "right")]
            if col == 0:
                ax.set_ylabel("fin command (°)")
            if row == 1:
                ax.set_xlabel("time from liftoff (s)")
            pad = t[t.phase == "READY"]
            if row == 0:
                steps = int((np.abs(np.diff(pad.fin0.values * 30.0)) > 0.1).sum())   # above the servo's 0.15 deg input threshold
                dur = len(pad) * 0.005
                ax.text(0.99, 0.04, f"whole READY: |fin| max {pad[[f'fin{i}' for i in range(4)]].abs().values.max() * 30:.3f}°, "
                        f"{steps / dur:.0f} command steps > 0.1° per second", transform=ax.transAxes, ha="right", fontsize=8,
                        bbox=dict(fc="white", ec="#ccc", pad=2))
    axs[0, 0].legend(loc="upper left", fontsize=7.5, ncol=4, frameon=False)
    axs[0, 0].set_ylim(-0.3, 0.3)
    fig.suptitle("Canard dither on the board (hardware-in-the-loop, G79W nominal): ±0.17° at 50 Hz", fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_dither_gate_hitl.png"), dpi=160, bbox_inches="tight")


def flight_fig(d, out):
    o, n = load(d, OLD), load(d, NEW)
    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True, gridspec_kw=dict(height_ratios=[1.3, 1.0, 0.8, 0.8]))
    xl = (-0.5, 10.5)
    rt = n[(n.phase == "COAST")]
    for ax in axs:
        ax.set_xlim(*xl); ax.grid(alpha=0.25); [ax.spines[s].set_visible(False) for s in ("top", "right")]
        for t, ls in ((n, ":"),):
            for ph in ("COAST", "DESCENT"):
                ax.axvline(t[t.phase == ph].tt.min(), color="#999", lw=0.8, ls=ls)
    a = axs[0]
    a.plot(o.tt, o.p_true_dps, color=GREY, lw=1.0, label="8 Nov build: law 1 + trim")
    a.plot(n.tt, n.p_true_dps, color=BLUE, lw=1.2, label="15 Nov flight-1 build: observer + roll test")
    a.set_ylabel("roll rate, plant truth (°/s)"); a.legend(loc="lower left", fontsize=8, frameon=False)
    a.set_title("Hardware-in-the-loop, G79W nominal: the real board flying the PC plant (HITL-01)", fontsize=10.5, loc="left")
    a.text(n[n.phase == "COAST"].tt.min() + 0.05, 0.97, "COAST", transform=a.get_xaxis_transform(), fontsize=7.5, va="top", color="#555")
    a.text(n[n.phase == "DESCENT"].tt.min() + 0.05, 0.97, "DESCENT\n(fins off)", transform=a.get_xaxis_transform(), fontsize=7.5, va="top", color="#555")
    a = axs[1]
    a.plot(o.tt, o.delta_true_deg, color=GREY, lw=1.0, label="8 Nov: canard (plant servo)")
    a.plot(n.tt, n.fin0 * 30, color=SKY, lw=0.6, label="15 Nov: fin 0 command")
    a.plot(n.tt, n.delta_true_deg, color=BLUE, lw=1.2, label="15 Nov: canard (plant servo)")
    a.set_ylabel("canard (°)"); a.legend(loc="lower left", fontsize=8, frameon=False, ncol=3)
    a = axs[2]
    a.plot(n.tt, n.qbar_true / 1000, color="#222", lw=1.1, label="true q")
    a.plot(n.tt, n.qbar / 1000, color=VERM, lw=1.0, ls="--", label="board q̂")
    a.set_ylabel("q (kPa)"); a.legend(loc="upper right", fontsize=8, frameon=False)
    a = axs[3]
    a.plot(o.tt, o.loop_us, color=GREY, lw=0.8, label=f"8 Nov (p99 {o.loop_us.quantile(.99):.0f} µs)")
    a.plot(n.tt, n.loop_us, color=BLUE, lw=0.8, label=f"15 Nov (p99 {n.loop_us.quantile(.99):.0f} µs, max {n.loop_us.max()} µs)")
    a.set_ylabel("frame compute (µs)"); a.set_xlabel("time from liftoff (s)")
    a.set_ylim(0, 1000); a.legend(loc="upper right", fontsize=8, frameon=False)
    a.text(0.01, 0.95, "frame 5000 µs, budget 4000 µs", transform=a.transAxes, fontsize=7.5, va="top", color="#555")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_hitl_flight1.png"), dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(sys.argv[-1] if len(sys.argv) > 1 else ".", exist_ok=True)
    dither_fig(sys.argv[1], sys.argv[2]); flight_fig(sys.argv[1], sys.argv[2])
