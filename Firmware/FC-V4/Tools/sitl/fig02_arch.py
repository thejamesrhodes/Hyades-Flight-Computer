"""FIG-02 F1: the flight-1 roll-control chain, from sensors to canards and back.

Every block is taken from the code as flashed on 15 Nov: flight.c (Flight_Step: estimate ->
monitor_and_arbitrate -> Ctrl_SetQbar -> roll test -> Ctrl_Update -> Alloc_Apply), control.c
(rate law, ESO law 2, law-1 trim), rolltest.c, alloc.c (null-axis dither, priority saturation,
quantisation), board.h constants, BENCH-02 servo, sitl.c plant (body + free fin can).
Same palette as sw01_flight_dataflow.py (Okabe-Ito tints).
Run: python fig02_arch.py OUTDIR  ->  fig02_roll_chain.png / .svg
"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

C = {  # fill, edge (Okabe-Ito tints)
    "hw":   ("#cfcfcf", "#303030"),
    "est":  ("#9fdcc6", "#00664a"),
    "fdir": ("#f6c4a4", "#a34700"),
    "fly":  ("#bcd7ee", "#00507d"),
    "eso":  ("#f3d38f", "#8a5a00"),
    "io":   ("#e6c3d8", "#8e4573"),
    "gate": ("#ffffff", "#555555"),
}
BLK = "#222222"


def box(ax, x, y, w, h, st, title, lines=(), fs=7.4):
    fc, ec = C[st]
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.5, zorder=3))
    n = len(lines)
    ty = y + (0.16 * n if n else 0.0)
    ax.text(x, ty, title, ha="center", va="center", fontsize=fs + 1.0, weight="bold", color=BLK, zorder=4)
    for i, s in enumerate(lines):
        ax.text(x, ty - 0.36 * (i + 1), s, ha="center", va="center", fontsize=fs, color=BLK, zorder=4)


def arrow(ax, p, q, label=None, lpos=0.5, dx=0.0, dy=0.18, col=BLK, ls="-", rad=0.0, fs=7.0, ha="center"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=11, lw=1.2, color=col, ls=ls,
                                 connectionstyle=f"arc3,rad={rad}", zorder=2, shrinkA=2, shrinkB=2))
    if label:
        x = p[0] + (q[0] - p[0]) * lpos + dx; y = p[1] + (q[1] - p[1]) * lpos + dy
        ax.text(x, y, label, ha=ha, va="bottom", fontsize=fs, color=col, zorder=5,
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.85))


def poly(ax, pts, col=BLK, ls="-", label=None, lxy=None, fs=7.0):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1], ys[:-1], color=col, lw=1.2, ls=ls, zorder=2)
    arrow(ax, pts[-2], pts[-1], col=col, ls=ls)
    if label:
        ax.text(*lxy, label, fontsize=fs, color=col, ha="center", va="bottom", zorder=5,
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.85))


def main(out):
    fig, ax = plt.subplots(figsize=(15.5, 8.6))
    ax.set_xlim(-0.6, 21.2); ax.set_ylim(-3.3, 9.6); ax.axis("off")

    # --- sensing and state (top row)
    box(ax, 1.6, 8.3, 2.7, 1.5, "hw", "IMU (QMI8658A)", ["gyro p, accel", "200 Hz frame"])
    box(ax, 5.6, 8.3, 3.2, 1.5, "est", "IEKF + NAV-01", ["attitude, v_up, h", "q̂ = ½ρV², q̂ valid"])
    box(ax, 10.0, 8.3, 3.3, 1.5, "fdir", "Detectors, phase, FDIR", ["PAD / BOOST / COAST / DESCENT", "control mode, faults"])
    arrow(ax, (2.95, 8.3), (4.0, 8.3))
    arrow(ax, (7.2, 8.3), (8.35, 8.3))
    # gates
    gx = 15.6
    box(ax, gx, 8.3, 5.6, 1.9, "gate", "Gates (set every frame)", [
        "observer: BOOST or COAST, q̂ valid, q̂ ≥ 1 kPa",
        "roll test: COAST + 0.15 s, q̂ ≥ 3 kPa, FULL, no fault",
        "dither: BOOST or COAST only (new 15 Nov)"], fs=7.2)
    arrow(ax, (11.65, 8.3), (12.8, 8.3))

    # --- control row
    yc = 4.5
    box(ax, 1.6, yc, 2.9, 1.9, "io", "Roll test", ["3-2-1-1 at the canard", "u_exc = 0.5°·3 kPa/q̂", "doublet p_ref ±2 rad/s",
                                                  "abort q̂<1.2 kPa, |p|>200°/s"], fs=6.9)
    box(ax, 5.6, yc, 2.9, 1.5, "fly", "Rate law", ["δ_p = (c/q̂)(p_ref − p)", "c = 15 Pa·s"])
    ax.add_patch(Circle((8.4, yc), 0.33, fc="white", ec=BLK, lw=1.3, zorder=3))
    ax.text(8.4, yc, "Σ", ha="center", va="center", fontsize=12, zorder=4)
    box(ax, 10.7, yc, 2.6, 1.5, "fly", "Limit", ["±12° at the canard", "(travel clamp)"])
    box(ax, 14.6, yc, 3.6, 1.9, "fly", "Allocation (ALLOC-01)", ["roll r = mean of 4 fins", "null-axis dither ±0.17°, 50 Hz",
                                                               "priority saturation", "sum-preserving quantisation"], fs=6.9)
    box(ax, 19.3, yc, 3.0, 1.5, "hw", "PWM, 8 MHz", ["per-fin zero (CAL-01)", "10.34 µs per degree"])
    arrow(ax, (3.05, yc + 0.15), (4.15, yc + 0.15), "p_ref", dy=0.05)
    poly(ax, [(1.6, yc - 0.95), (1.6, 2.95), (7.95, 2.95), (8.17, yc - 0.23)], label="u_exc (added after the law)", lxy=(4.4, 3.0))
    arrow(ax, (7.05, yc), (8.07, yc), "δ_p", dy=0.05)
    arrow(ax, (8.73, yc), (9.4, yc))
    arrow(ax, (12.0, yc), (12.8, yc), "δ", dy=0.05)
    arrow(ax, (16.4, yc), (17.8, yc), "4 fins", dy=0.05)

    # observer and trim (below the sum)
    box(ax, 8.4, 1.25, 4.4, 1.9, "eso", "Observer (law 2, ESO)", ["ṗ = β̂ q̂ δ + f;  ω_o = 20 rad/s", "β̂ = 0.727 (1.3 × nominal)",
                                                              "servo model: 4 frames + 20 ms lag", "δ_d = −f̂ / (β̂ q̂)"], fs=6.9)
    arrow(ax, (8.4, 2.2), (8.4, yc - 0.33), "δ_d", lpos=0.62, dx=0.35, dy=-0.1)
    ax.text(8.4, -0.15, "off (pad, q̂ < 1 kPa, after apogee): law-1 trim integrator, bumpless hand-over",
            ha="center", va="top", fontsize=6.6, color="#555", style="italic")
    poly(ax, [(12.4, yc), (12.4, 1.25), (10.6, 1.25)], label=None)
    ax.text(12.5, 1.6, "command δ\n(incl. u_exc)", fontsize=6.6, ha="left", va="center", color=BLK)

    # gate wires (dashed)
    G = "#777777"
    poly(ax, [(13.4, 7.35), (13.4, 6.6), (1.6, 6.6), (1.6, yc + 0.95)], col=G, ls="--")
    poly(ax, [(7.55, 6.6), (7.55, 2.2)], col=G, ls="--")
    poly(ax, [(15.6, 7.35), (15.6, yc + 0.95)], col=G, ls="--")
    poly(ax, [(5.6, 7.55), (5.6, yc + 0.75)], label="q̂", lxy=(5.95, 6.95))

    # --- plant (bottom right)
    yp = -1.6
    box(ax, 19.3, 1.25, 3.0, 1.5, "hw", "4 × BMS-127WV+", ["dead time 9.3 ms", "ω_n 130 rad/s, ζ 0.76"])
    box(ax, 15.0, yp, 3.6, 1.6, "hw", "Canards → forward body", ["L = q S d Cl_δ δ, β = S d Cl_δ / Ixx", "roll damping Cl_p, Cl₀"], fs=6.9)
    box(ax, 10.3, yp, 3.4, 1.6, "hw", "Fin can on bearing", ["cant spins it; friction", "drags the body (≤ 6 mN m)"], fs=6.9)
    arrow(ax, (19.3, yc - 0.75), (19.3, 2.0))
    poly(ax, [(19.3, 0.5), (19.3, yp), (16.8, yp)], label="canard δ", lxy=(18.1, yp + 0.05))
    ax.add_patch(FancyArrowPatch((13.2, yp + 0.25), (12.0, yp + 0.25), arrowstyle="<|-|>", mutation_scale=10, lw=1.2, color=BLK, zorder=2))
    ax.text(12.6, yp + 0.4, "slip", fontsize=6.8, ha="center", va="bottom")
    poly(ax, [(15.0, yp - 0.8), (15.0, -2.55), (-0.3, -2.55), (-0.3, 8.3), (0.25, 8.3)], label="body roll rate p (gyro)", lxy=(5.0, -2.5))
    poly(ax, [(1.6, 7.55), (1.6, 6.95), (4.3, 6.95), (4.3, yc + 0.7)], label=None)
    ax.text(3.0, 7.02, "p", fontsize=7.2, ha="center", va="bottom")
    poly(ax, [(-0.3, 1.25), (6.2, 1.25)], label=None)
    ax.text(3.0, 1.3, "p", fontsize=7.2, ha="center", va="bottom")

    ax.text(-0.5, 9.45, "Flight-1 roll control: sensors to canards (as flashed 15 Nov 2026, FC-V4, 200 Hz)",
            fontsize=12, weight="bold", ha="left", va="top")
    for st, lab, x in (("hw", "hardware / plant", 0.2), ("est", "estimation", 3.3), ("fdir", "phase & FDIR", 5.9),
                       ("fly", "control & allocation", 8.7), ("eso", "observer", 12.2), ("io", "roll test", 14.6)):
        fc, ec = C[st]
        ax.add_patch(FancyBboxPatch((x, -3.16), 0.45, 0.22, boxstyle="round,pad=0.01", fc=fc, ec=ec, lw=1))
    for st, lab, x in (("hw", "hardware / plant", 0.2), ("est", "estimation", 3.3), ("fdir", "phase & FDIR", 5.9),
                       ("fly", "control & allocation", 8.7), ("eso", "observer", 12.2), ("io", "roll test", 14.6)):
        ax.text(x + 0.55, -3.05, lab, fontsize=7, va="center")
    ax.plot([16.6, 17.2], [-3.05, -3.05], color="#777", ls="--", lw=1.2); ax.text(17.3, -3.05, "gate", fontsize=7, va="center")
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(out, f"fig02_roll_chain.{ext}"), dpi=170, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(sys.argv[-1] if len(sys.argv) > 1 else ".", exist_ok=True)
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
