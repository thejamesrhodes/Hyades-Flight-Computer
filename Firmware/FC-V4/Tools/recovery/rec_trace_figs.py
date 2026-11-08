"""FIG-01 recovery: time histories from the REC-01 ejection / cord / canopy sub-simulation.

    python rec_trace_figs.py     -> Documentation/Images/fig01/rec_*.png
"""
import argparse, csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rec

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "fig01"))
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3})


def run(n, seed, **kw):
    a = argparse.Namespace(v_dep=None, bp=[0.3, 1.0], hold=[10.0, 50.0], chute_cds=0.7 * np.pi * 0.09, chute_d=0.6,
                           cord_len=4.0, cord="kevlar", ea=None, zeta=[0.05, 0.15])
    for k, v in kw.items():
        setattr(a, k, v)
    rows = list(csv.DictReader(open(os.path.join(HERE, "..", "sitl", "runs", "L_active", "runs.csv"))))
    dep = {k: np.array([float(r[k]) for r in rows if float(r.get("dep_V", 0)) > 0]) for k in ("dep_V", "dep_gam", "dep_aoa", "dep_h")}
    s = rec.sample(np.random.default_rng(seed), n, a, dep)
    R = {}
    out = rec.simulate(s, rec=R)
    R = {k: np.array(v) for k, v in R.items()}
    return s, out, R


def spaghetti(ax, t, Y, color, label, lw=0.35):
    for j in range(Y.shape[1]):
        ax.plot(t, Y[:, j], color=color, lw=lw, alpha=0.35)
    ax.plot(t, np.percentile(Y, 50, axis=1), color=color, lw=1.8, label=f"{label}: median")
    ax.plot(t, np.percentile(Y, 95, axis=1), color=color, lw=1.0, ls="--", label=f"{label}: p95")


def main():
    os.makedirs(OUT, exist_ok=True)
    N = 60
    sK, oK, RK = run(N, 5)                                                     # as built: 0.6 m, 4 m Kevlar
    sR, oR, RR = run(N, 5, ea=300.0, cord_len=6.0, chute_d=0.9, chute_cds=0.477)   # recommended

    # --- ejection pressure and stroke ---
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    t1 = RK["t1"] * 1e3
    for j in range(N):
        ax[0].plot(t1, RK["P"][:, j] / 1e3, color="C0", lw=0.5, alpha=0.5)
        ax[1].plot(t1, RK["s"][:, j] * 1e3, color="C0", lw=0.5, alpha=0.5)
    ax[0].axhspan(69, 103, color="g", alpha=0.08); ax[0].text(1, 105, "10-15 psi hobby design band", fontsize=7.5)
    a2 = ax[0].twinx(); a2.set_ylim(np.array(ax[0].get_ylim()) * rec.A_T * 1e3); a2.set_ylabel("bulkhead / motor-retention force (N)"); a2.grid(False)
    ax[0].set(xlabel="time from charge ignition (ms)", ylabel="gauge pressure (kPa)", title=f"Ejection pressure, {N} runs (charge 0.3-1.0 g)")
    ax[1].axhline(60, color="k", ls=":"); ax[1].text(1, 62, "coupler leaves the tube (60 mm)", fontsize=7.5)
    ax[1].set(xlabel="time from charge ignition (ms)", ylabel="coupler stroke (mm)", title="Separation stroke"); ax[1].set_ylim(0, 80); ax[0].set_xlim(0, 40); ax[1].set_xlim(0, 40)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_ejection_pressure.png"), dpi=150); plt.close(fig)

    # --- cord / canopy / accelerations, as built vs recommended ---
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for R, col, lab in ((RK, "C3", "as built (0.6 m, 4 m Kevlar)"), (RR, "C0", "recommended (0.9 m, 6 m, EA 300 N)")):
        t = R["t2"]
        spaghetti(axs[0], t, R["d"], col, lab)
        spaghetti(axs[1], t, np.maximum(R["T1"], R["T2"]), col, lab)
        spaghetti(axs[2], t, R["T3"], col, lab)
        spaghetti(axs[3], t, np.maximum(R["gF"], R["gA"]), col, lab)
    axs[0].axhline(0.4, color="k", ls=":"); axs[0].text(3.0, 0.5, "joined spacing (closer = sections pass / touch)", fontsize=7.5)
    axs[0].set_ylabel("section separation (m)")
    axs[1].set_ylabel("cord tension, max of segments (N)")
    axs[2].set_ylabel("shroud / canopy load (N)")
    axs[3].set_ylabel("max section acceleration (g)"); axs[3].axhline(16, color="k", ls=":"); axs[3].text(3.0, 18, "FC-V4 IMU range ±16 g", fontsize=7.5)
    axs[3].set_yscale("symlog", linthresh=10)
    axs[0].legend(fontsize=7, loc="upper left", ncol=2); axs[-1].set_xlabel("time from separation (s)")
    fig.suptitle(f"Recovery sub-sim: {N} dispersed deployments each, same seeds", weight="bold")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_cord_timeseries.png"), dpi=150); plt.close(fig)

    # --- one annotated deployment ---
    j = int(np.argsort(oK["T_max"])[N // 2])                        # median-tension Kevlar run
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for R, col, lab in ((RK, "C3", "as built"), (RR, "C0", "recommended")):
        t = R["t2"]
        ax[0].plot(t, R["d"][:, j], col, label=lab); ax[1].plot(t, R["T1"][:, j], col, lw=1, label=f"{lab}: forward segment F-K")
        ax[1].plot(t, R["T2"][:, j], col, lw=1, ls="--", label=f"{lab}: aft segment K-A"); ax[1].plot(t, R["T3"][:, j], col, lw=0.8, ls=":", label=f"{lab}: shroud")
        ax[2].plot(t, R["cds"][:, j], col, label=lab)
    ax[0].set_ylabel("separation (m)"); ax[1].set_ylabel("tension (N)"); ax[2].set_ylabel("canopy CdS (m²)")
    ax[0].legend(fontsize=7); ax[1].legend(fontsize=6.5, ncol=2); ax[2].set_xlabel("time from separation (s)")
    ax[0].set_title(f"One deployment (median-tension seed): V_dep {sK['V0'][j]:.1f} m/s, charge {sK['m_bp'][j]:.2f} g, "
                    f"v_sep {oK['v_sep'][j]:.1f} m/s. Sequence: separation, cord stretch (snatch), canopy fill, aft-section overrun, second snatch",
                    fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_single_deployment.png"), dpi=150); plt.close(fig)
    print("figures ->", OUT)


if __name__ == "__main__":
    main()
