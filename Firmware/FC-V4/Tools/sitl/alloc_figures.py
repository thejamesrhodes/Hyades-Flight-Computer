"""ALLOC-01 figures -> Documentation/Images/alloc01/

  alloc01_mechanism.png/.svg   what the allocator does, from alloc.c itself (alloc_demo.c)
  alloc01_results.png/.svg     paired SITL Monte Carlo changes per flight

    python alloc_figures.py
"""
import csv, os, subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CORE = os.path.join(HERE, "..", "..", "Core")
OUT = os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "alloc01")
# Okabe-Ito
BLUE, VERM, GREEN, ORANGE, SKY, PURPLE, BLK, GRY = "#0072B2", "#D55E00", "#009E73", "#E69F00", "#56B4E9", "#CC79A7", "#222222", "#777777"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})


def demo(flags, mode):
    exe = os.path.join("/tmp", "alloc_demo_" + str(abs(hash(flags + str(mode)))))
    subprocess.run(["gcc", "-std=c11", "-O2", *flags.split(), f"-I{CORE}/hyades/gnc", f"-I{CORE}/Inc",
                    os.path.join(HERE, "alloc_demo.c"), f"{CORE}/hyades/gnc/alloc.c", "-lm", "-o", exe], check=True)
    r = subprocess.run([exe, str(mode)], capture_output=True, text=True, check=True).stdout.split("\n")
    return np.array([[float(v) for v in l.split()] for l in r if l.strip()])


def mechanism():
    tpn = 200.0                                                   # ticks per unit at 1 MHz
    leg = demo("-DPWM_TICK_HZ=1000000u -DCTRL_DITHER_ON=0", 0)
    stg = demo("-DPWM_TICK_HZ=1000000u -DCTRL_DITHER_ON=0 -DCTRL_ALLOC_STAGGER=1", 0)
    nul = demo("-DPWM_TICK_HZ=8000000u -DCTRL_DITHER_NULL=1", 1)
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw=dict(wspace=0.32))

    a = ax[0]
    k = leg[:, 0]
    a.plot(k, 4 * leg[:, 1] * tpn, color=BLK, lw=1.4, label="Requested")
    a.step(k, leg[:, 2:6].sum(1), where="post", color=VERM, lw=1.6, label="Each fin rounded")
    a.step(k, stg[:, 2:6].sum(1), where="post", color=BLUE, lw=1.6, label="Sum-preserving")
    a.set_xlabel("Frame")
    a.set_ylabel("Total roll command, ticks (sum of 4 fins)")
    a.set_title("(a) Roll command resolution, 1 MHz tick", loc="left", fontsize=9.5, weight="bold")
    a.legend(frameon=False, loc="upper left")

    a = ax[1]
    w = slice(0, 24)
    kk = stg[w, 0]
    a.step(kk, stg[w, 2:6].sum(1) - 4 * np.floor(stg[w, 1] * tpn), where="post", color=BLUE, lw=1.6, label="Roll, extra ticks")
    a.step(kk, stg[w, 2] - stg[w, 4], where="post", color=ORANGE, lw=1.6, label="Pitch pair difference")
    a.step(kk, stg[w, 3] - stg[w, 5], where="post", color=PURPLE, lw=1.6, label="Yaw pair difference")
    a.axhline(0, color=GRY, lw=0.8)
    a.set_xlabel("Frame")
    a.set_ylabel("Ticks")
    a.set_ylim(-1.6, 5.4)
    a.set_title("(b) Odd tick alternates between fins", loc="left", fontsize=9.5, weight="bold")
    a.legend(frameon=False, loc="upper right")

    a = ax[2]
    w = slice(0, 16)
    kk = nul[w, 0]
    a.step(kk, nul[w, 2], where="post", color=BLUE, lw=1.6, label="Top and bottom fins")
    a.step(kk, nul[w, 3], where="post", color=VERM, lw=1.6, label="Left and right fins")
    a.step(kk, nul[w, 2:6].sum(1), where="post", color=BLK, lw=1.6, label="Roll sum")
    a.set_xlabel("Frame")
    a.set_ylabel("Ticks, 8 MHz")
    a.set_ylim(-14, 24)
    a.set_title("(c) Null-space dither, zero command", loc="left", fontsize=9.5, weight="bold")
    a.legend(frameon=False, loc="upper right", ncol=1)
    from matplotlib.ticker import MaxNLocator
    for a in ax:
        a.xaxis.set_major_locator(MaxNLocator(integer=True))
    for e in ("png", "svg"):
        fig.savefig(os.path.join(OUT, "alloc01_mechanism." + e), dpi=160, bbox_inches="tight")


def load(path):
    return {int(float(r["seed"])): r for r in csv.DictReader(open(os.path.join(HERE, path, "runs.csv")))}


def results():
    base = load("runs/alloc/A1_1MHz")
    cases = [  # label, run, baseline, colour
        ("8 MHz tick", "runs/alloc/B1_8MHz", base, BLUE),
        ("Sum-preserving, 1 MHz", "runs/alloc/C1_1MHz_stag", base, BLUE),
        ("8 MHz + sum-preserving\n+ null dither", "runs/alloc/E1_8MHz_stag_null", base, BLUE),
        ("Null-space dither", "runs/alloc/D1_1MHz_null", base, PURPLE),
        ("Adopted: 8 MHz\n+ null dither", "runs/alloc/H1_8MHz_null", base, GREEN),
        ("Dither removed", "runs/alloc/G1_1MHz_nodither", base, VERM),
        ("Canard zero error\n0.5 to 3 deg, uncalibrated", "runs/alloc_zero/H1_uncal", load("runs/alloc_zero/H1_cal"), VERM),
    ]
    mets = [("p_rms_coast", "Coast roll-rate RMS, change per flight (deg/s)"),
            ("p_peak", "Peak roll rate, change per flight (deg/s)")]
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw=dict(wspace=0.08), sharey=True)
    for j, (m, lab) in enumerate(mets):
        a = ax[j]
        for i, (name, path, b, col) in enumerate(cases):
            d = load(path)
            seeds = sorted(set(d) & set(b))

            def val(r):
                return max(float(r["p_max_boost"]), float(r["p_max_coast"])) if m == "p_peak" else float(r[m])
            diff = np.array([val(d[s]) - val(b[s]) for s in seeds])
            y = len(cases) - 1 - i
            lo, q1, med, q3, hi = np.percentile(diff, [5, 25, 50, 75, 95])
            a.plot([lo, hi], [y, y], color=col, lw=1.4, solid_capstyle="butt")
            a.add_patch(plt.Rectangle((q1, y - 0.22), q3 - q1, 0.44, fc="white", ec=col, lw=1.6, zorder=3))
            a.plot([med, med], [y - 0.22, y + 0.22], color=col, lw=2.4, zorder=4)
            a.plot([diff.mean()], [y], marker="D", ms=4.5, color=BLK, zorder=5)
        a.axvline(0, color=GRY, lw=0.9)
        a.set_xlabel(lab)
        a.set_yticks(range(len(cases)))
        a.set_yticklabels([c[0] for c in cases][::-1])
        a.grid(axis="x", color="#e6e6e6", lw=0.7)
    ax[0].set_title("Paired SITL Monte Carlo against the firmware before ALLOC-01, 360 flights over 6 motors (zero-error case 240)",
                    loc="left", fontsize=10, weight="bold")
    h = [plt.Line2D([], [], color=BLUE, lw=2, label="Command resolution"),
         plt.Line2D([], [], color=PURPLE, lw=2, label="Dither placement"),
         plt.Line2D([], [], color=GREEN, lw=2, label="Adopted configuration"),
         plt.Line2D([], [], color=VERM, lw=2, label="Large effect"),
         plt.Line2D([], [], color=BLK, marker="D", ls="none", ms=4.5, label="Mean"),
         plt.Rectangle((0, 0), 1, 1, fc="white", ec=GRY, label="Box: 25 to 75 %, whiskers 5 to 95 %")]
    ax[0].legend(handles=h, frameon=False, loc="upper right", fontsize=8.5)
    for e in ("png", "svg"):
        fig.savefig(os.path.join(OUT, "alloc01_results." + e), dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    mechanism()
    results()
    print("ok")
