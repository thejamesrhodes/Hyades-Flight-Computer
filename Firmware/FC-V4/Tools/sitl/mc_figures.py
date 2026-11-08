"""Figures for ADR-004 / MC-01 (writes into Documentation/Images/adr004)."""
import csv, json, os, sys, subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "adr004")
RUNS = os.path.join(HERE, "runs")
MOT = ["G40W", "G53FJ", "G79W", "G80T", "G77R", "G104T"]
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 110})


def rows(name):
    return list(csv.DictReader(open(os.path.join(RUNS, name, "runs.csv"))))


def col(R, k):
    return np.array([float(r[k]) for r in R])


def fig_roll_by_motor():
    R = rows("V_active")
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    for i, (k, lim, t) in enumerate([("p_rms_coast", 60, "Coast roll-rate RMS (deg/s)"),
                                     ("pmax", 360, "Peak |roll rate|, boost+coast (deg/s)")]):
        data = []
        for m in MOT:
            rr = [r for r in R if r["motor"].endswith(m + ".csv")]
            v = (np.maximum(col(rr, "p_max_boost"), col(rr, "p_max_coast")) if k == "pmax" else col(rr, k))
            data.append(v)
        ax[i].boxplot(data, tick_labels=MOT, whis=(1, 99), showfliers=True, flierprops={"markersize": 2})
        ax[i].axhline(lim, color="r", ls="--", lw=1, label=f"requirement {lim}")
        ax[i].set_title(t); ax[i].legend(loc="upper left")
    fig.suptitle("MC-01 active roll control, 1200 dispersed flights (build tolerances met), c = 15, 200 Hz synchronous", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "mc_roll_by_motor.png")); plt.close(fig)


def fig_matrix():
    camps = [("V_active", "Active, build tol."), ("V_active_widetol", "Active, wide tol."), ("V_wind12", "Active, wind 0-12 m/s"),
             ("V_shadow", "Flight 1 shadow (canards 0)"), ("V_damponly", "Flight 1 damping only"),
             ("V_fault_stuck", "Servo stuck"), ("V_fault_slow", "Servo slow (1/4 rate)"), ("V_fault_offset", "Servo offset 0.5-3 deg"),
             ("V_fault_float", "Servo unpowered (float)"), ("V_fault_hardover", "Servo hard-over")]
    req = ["R1_sequence", "R2_liftoff_le_0.35s", "R3_apogee_-0.5_+1.5s", "R4a_landing_never_early", "R4b_landing_latency",
           "R6_lockin_ratio_le_0.7", "R9_qhat_rms_le_25pct", "R5a_roll_max_le_360dps", "R5b_coast_rms_le_60dps", "R8_sat_le_5pct"]
    M = np.full((len(camps), len(req)), np.nan)
    for i, (c, _) in enumerate(camps):
        s = json.load(open(os.path.join(RUNS, c, "summary.json")))
        for j, k in enumerate(req):
            if k in s["pass"]:
                M[i, j] = s["pass"][k]
    fig, ax = plt.subplots(figsize=(10, 4.6))
    im = ax.imshow(M, cmap="RdYlGn", vmin=50, vmax=100, aspect="auto")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            ax.text(j, i, "n/a" if np.isnan(M[i, j]) else f"{M[i, j]:.0f}", ha="center", va="center", fontsize=8)
    ax.set_xticks(range(len(req))); ax.set_xticklabels([r.split("_", 1)[0] for r in req])
    ax.set_yticks(range(len(camps))); ax.set_yticklabels([f"{b}  (n={json.load(open(os.path.join(RUNS, a, 'summary.json')))['n']})" for a, b in camps])
    ax.grid(False); fig.colorbar(im, label="% of runs passing")
    ax.set_title("MC-01 requirement pass rates by campaign (R-numbers: MC-01 §3; roll requirements n/a when the canards are held)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "mc_campaign_matrix.png")); plt.close(fig)


def fig_lockin():
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    for c, lab in [("V_active", "active"), ("V_damponly", "damping only"), ("V_shadow", "shadow (canards held)")]:
        v = np.sort(col(rows(c), "body_pitch_ratio"))
        ax[0].plot(v, 1 - np.arange(len(v)) / len(v), label=lab)
    ax[0].axvline(0.7, color="r", ls="--", lw=1, label="R6 limit 0.7"); ax[0].axvline(1.0, color="k", ls=":", lw=1, label="resonance")
    ax[0].set_yscale("log"); ax[0].set_xlabel("max |body roll rate| / pitch natural frequency"); ax[0].set_ylabel("fraction of flights exceeding")
    ax[0].legend(fontsize=8); ax[0].set_title("Body roll vs pitch resonance (lock-in)")
    R = rows("V_active_widetol")
    ax[1].scatter(np.abs(col(R, "in_can_cant_deg")), col(R, "can_pitch_ratio"), s=4, alpha=0.5)
    x = np.linspace(0, 1, 10); ax[1].plot(x, 1.13 * x, "k--", lw=1, label="fit 1.13 x |cant|")
    ax[1].axhline(0.7, color="r", ls="--", lw=1, label="R6 limit"); ax[1].axvline(0.25, color="g", ls=":", lw=1.2, label="build tolerance 0.25 deg")
    ax[1].set_xlabel("|net rear-fin cant| (deg)"); ax[1].set_ylabel("max can spin / pitch frequency")
    ax[1].set_title("Free fin can spin vs pitch frequency (1200 flights, wide tolerance)"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "mc_lockin.png")); plt.close(fig)


def fig_faults():
    kinds = ["stuck", "slow", "offset", "float", "hardover"]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    data = [np.maximum(col(rows(f"V_fault_{k}"), "p_max_boost"), col(rows(f"V_fault_{k}"), "p_max_coast")) for k in kinds]
    base = np.maximum(col(rows("V_active"), "p_max_boost"), col(rows("V_active"), "p_max_coast"))
    ax[0].boxplot([base] + data, tick_labels=["none"] + kinds, whis=(1, 99), flierprops={"markersize": 2})
    ax[0].axhline(360, color="r", ls="--", lw=1); ax[0].set_ylabel("peak |roll rate| (deg/s)"); ax[0].set_title("Single-servo failure at 0-6 s: roll")
    data = [col(rows(f"V_fault_{k}"), "aoa_max") for k in kinds]
    ax[1].boxplot([col(rows("V_active"), "aoa_max")] + data, tick_labels=["none"] + kinds, whis=(1, 99), flierprops={"markersize": 2})
    ax[1].set_ylabel("max angle of attack, q > 500 Pa (deg)"); ax[1].set_title("Pitch-over check: no failure mode raises AoA")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "mc_faults.png")); plt.close(fig)


def fig_margins():
    sys.path.insert(0, "/home/claude/an")
    from rollopt import at_C, chain
    Cs = np.arange(6, 41, 1.0)
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for lab, ex, lpf, st in [("7 Nov: 100 Hz frame, 50 Hz async PWM", chain(100, 50, 224.2, 30, False)[0], 30, "--"),
                             ("ADR-004: 200 Hz, PWM synced, 448 Hz IMU", chain(200, 200, 448.4, 60, True)[0], 60, "-")]:
        pm = [at_C(C, Ixx=2.2e-4, extra=ex, lpf=lpf)[0] for C in Cs]
        gm = [20 * np.log10(max(at_C(C, Ixx=2.2e-4, extra=ex, lpf=lpf)[1], 1e-3)) for C in Cs]
        ax[0].plot(Cs, pm, st, label=lab); ax[1].plot(Cs, gm, st, label=lab)
    for a, lim, yl in [(ax[0], 45, "worst phase margin over envelope (deg)"), (ax[1], 6, "worst gain margin (dB)")]:
        a.axhline(lim, color="r", ls=":", lw=1, label=f"SAE AS94900 {lim}"); a.axvline(15, color="g", lw=1, label="c = 15 (chosen)")
        a.axvline(70, color="k", lw=1, ls="-.", label="c = 70 (old)"); a.set_xlabel("roll gain constant c (Pa s)"); a.set_ylabel(yl); a.legend(fontsize=7)
    ax[0].set_ylim(-90, 90); ax[1].set_ylim(-10, 20)
    fig.suptitle("Roll-loop margins on the forward-body inertia 2.2e-4 kg m^2 (aero +/-30 %, nominal servo)", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "loop_margins.png")); plt.close(fig)
    # delay budget
    fig, ax = plt.subplots(figsize=(7, 3))
    items_old = [("IMU LPF (30 Hz, group)", 7.5), ("sample age", 2.2), ("compute", 1.5), ("frame ZOH T/2", 5.0), ("PWM resample (50 Hz)", 10.0), ("pulse read", 1.5), ("servo tau_d", 14)]
    items_new = [("IMU LPF (60 Hz, group)", 3.75), ("sample age", 1.1), ("compute", 1.5), ("frame ZOH T/2", 2.5), ("PWM resample", 0.0), ("pulse read", 1.5), ("servo tau_d", 14)]
    for y, items in [(1, items_old), (0, items_new)]:
        x0 = 0
        for i, (n, v) in enumerate(items):
            ax.barh(y, v, left=x0, color=plt.cm.tab10(i), edgecolor="w", label=n if y == 1 else None); x0 += v
        ax.text(x0 + 0.5, y, f"{x0:.1f} ms", va="center")
    ax.set_yticks([0, 1]); ax.set_yticklabels(["ADR-004 (200 Hz synced)", "7 Nov (100/50 Hz)"]); ax.set_xlabel("equivalent loop delay (ms)")
    ax.legend(fontsize=7, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.3)); ax.set_xlim(0, 48)
    ax.set_title("Roll-loop delay budget (ADR-004 §2)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "delay_budget.png")); plt.close(fig)


def fig_flight():
    tr = "/tmp/fig_trace.csv"
    subprocess.run([os.path.join(HERE, "sitl"), "seed=42", f"trace={tr}", "cl0_body=0.003", "can_cant_deg=0.15", "fin1_mis_deg=0.08",
                    "bear_coul=0.005", "wind=4", "motor=motors/G79W.csv"], cwd=HERE, check=True, capture_output=True)
    d = np.genfromtxt(tr, delimiter=",", names=True)
    m = (d["t"] > -0.3) & (d["t"] < 12)
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(d["t"][m], d["p_dps"][m], label="body roll rate"); ax[0].plot(d["t"][m], d["pcan_dps"][m] / 5, label="fin can / 5")
    ax[0].set_ylabel("deg/s"); ax[0].legend(fontsize=8)
    for i in range(4):
        ax[1].plot(d["t"][m], d[f"delta{i}"][m], lw=0.8, label=f"canard {i}")
    ax[1].plot(d["t"][m], d["trim"][m] * 57.2958, "k--", lw=1, label="trim"); ax[1].set_ylabel("deg"); ax[1].legend(fontsize=7, ncol=5)
    ax[2].plot(d["t"][m], d["qbar"][m], label="q true"); ax[2].plot(d["t"][m], d["qbar_est"][m], "--", label="q_hat (NAV-01)")
    ax[2].set_ylabel("Pa"); ax[2].legend(fontsize=8)
    ax[3].plot(d["t"][m], d["alpha_deg"][m], label="angle of attack"); ax3b = ax[3].twinx()
    ax3b.step(d["t"][m], d["phase"][m], "k", lw=0.8, where="post"); ax3b.set_yticks([3, 4, 5, 6]); ax3b.set_yticklabels(["READY", "BOOST", "COAST", "DESCENT"])
    ax[3].set_ylabel("deg"); ax[3].set_xlabel("time from ignition (s)")
    fig.suptitle("SITL 6DOF, G79W, wind 4 m/s, realistic asymmetries: the flight code (flight.c) in the loop", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sitl_flight_example.png")); plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for f in (fig_margins, fig_roll_by_motor, fig_matrix, fig_lockin, fig_faults, fig_flight):
        f(); print("ok", f.__name__)
