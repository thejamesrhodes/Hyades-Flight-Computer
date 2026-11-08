"""Figures for LD-01 (flight loads) and REC-01 (recovery). Writes Documentation/Images/ld01/."""
import csv, json, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "ld01")
SITL = os.path.join(HERE, "..", "sitl", "runs")
os.makedirs(OUT, exist_ok=True)
ST = ["nose_joint", "avbay_canard", "canard_body", "coupler_aft", "body_mid", "body_fincan"]
XS = [0.10, 0.18, 0.26, 0.32, 0.51, 0.76]
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3})


def rows(p):
    return list(csv.DictReader(open(p)))


def col(rr, k):
    return np.array([float(r[k]) for r in rr if r.get(k, "") != ""])


# ---- LD-01 envelope ----
camps = [("L_active", "active, build tol."), ("L_wind12", "wind 0-12 m/s"), ("L_fault_hardover", "canard hard-over")]
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
for name, lab in camps:
    p = os.path.join(SITL, name, "runs.csv")
    if not os.path.exists(p):
        continue
    rr = rows(p)
    M = [np.percentile(col(rr, f"ldM_{s}"), 99) for s in ST]
    P = [np.percentile(col(rr, f"ldP_{s}"), 99) for s in ST]
    ax[0].plot(XS, M, "o-", label=lab); ax[1].plot(XS, P, "o-", label=lab)
ax[0].set(xlabel="station from nose tip (m)", ylabel="bending moment p99 (N m)", title="Ascent bending moment (p99)")
ax[1].set(xlabel="station from nose tip (m)", ylabel="axial compression p99 (N)", title="Ascent axial compression (p99)")
ax[0].legend(); fig.tight_layout(); fig.savefig(os.path.join(OUT, "ld_envelope.png"), dpi=150); plt.close(fig)

# ---- REC-01 ----
R = os.path.join(HERE, "runs")
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
for c in ("kevlar", "nylon", "elastic"):
    p = os.path.join(R, f"R_{c}", "runs.csv")
    if not os.path.exists(p):
        continue
    rr = rows(p); t = np.sort(col(rr, "T_max")); g = np.sort(col(rr, "g_A"))
    y = np.arange(1, len(t) + 1) / len(t)
    ax[0].plot(t, y, label=c); ax[1].plot(g, y, label=c)
ax[0].set(xlabel="peak shock-cord tension (N)", ylabel="cumulative fraction", title="Shock-cord peak tension")
ax[1].set(xlabel="peak aft-section acceleration (g)", title="Aft-section peak acceleration")
ax[0].legend(); fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_cord_cdf.png"), dpi=150); plt.close(fig)

ea, t50, t99, g99 = [], [], [], []
for e in (50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000):
    p = os.path.join(R, f"S_ea{e}", "summary.json")
    if os.path.exists(p):
        s = json.load(open(p))["pct"]; ea.append(e); t50.append(s["T_max"]["p50"]); t99.append(s["T_max"]["p99"]); g99.append(s["g_A"]["p99"])
if ea:
    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    ax.semilogx(ea, t50, "o-", label="tension p50"); ax.semilogx(ea, t99, "s-", label="tension p99")
    ax2 = ax.twinx(); ax2.semilogx(ea, g99, "^--", color="C3", label="aft accel p99 (g)"); ax2.set_ylabel("aft accel p99 (g)")
    for lab, (lo, hi) in (("elastic", (40, 150)), ("nylon", (1.5e3, 5e3)), ("Kevlar", (6e3, 2.5e4))):
        ax.axvspan(lo, hi, alpha=0.08); ax.text(np.sqrt(lo * hi), max(t99) * 0.95, lab, ha="center", fontsize=8)
    ax.set(xlabel="cord axial stiffness EA (N)", ylabel="peak tension (N)", title="Cord stiffness trade (4 m cord, 0.6 m chute)")
    ax.legend(loc="center left"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_ea_sweep.png"), dpi=150); plt.close(fig)

p = os.path.join(R, "R_kevlar", "runs.csv")
if os.path.exists(p):
    rr = rows(p)
    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    sc = ax.scatter(col(rr, "m_bp"), col(rr, "P_max_kPa"), c=col(rr, "eta"), s=4, cmap="viridis")
    plt.colorbar(sc, label="combustion efficiency eta")
    ax.axhline(69, ls="--", c="k", lw=0.8); ax.axhline(103, ls="--", c="k", lw=0.8)
    ax.text(0.31, 106, "10-15 psi hobby design band", fontsize=8)
    ax.set(xlabel="motor ejection charge (g BP)", ylabel="peak gauge pressure (kPa)", title="Ejection pressure (bulkhead force = P x 28.3 cm^2)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_pressure.png"), dpi=150); plt.close(fig)

# descent rate vs canopy
m = 0.7736 + 0.064; rho = 1.2
fig, ax = plt.subplots(figsize=(5.5, 3.4))
D = np.linspace(0.5, 1.4, 50)
for cd, lab in ((0.7, "flat, Cd 0.7"), (0.8, "Cd 0.8"), (1.5, "high-Cd (elliptical/toroidal) Cd 1.5")):
    ax.plot(D, np.sqrt(2 * m * 9.81 / (rho * cd * np.pi * D ** 2 / 4)), label=lab)
ax.axhspan(4.5, 6.0, alpha=0.1, color="g"); ax.text(1.0, 5.2, "4.5-6 m/s target (inference)", fontsize=8)
ax.axvline(0.6, c="k", ls=":"); ax.text(0.61, 11, "current 0.6 m", fontsize=8)
ax.set(xlabel="canopy nominal diameter (m)", ylabel="descent rate at sea level (m/s)", title=f"Descent rate, {m * 1000:.0f} g recovered")
ax.legend(); fig.tight_layout(); fig.savefig(os.path.join(OUT, "rec_descent.png"), dpi=150); plt.close(fig)
print("figures ->", os.path.abspath(OUT))
