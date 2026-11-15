import numpy as np, json, cv2, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Arc
OI = dict(blue="#0072B2", verm="#D55E00", orange="#E69F00", sky="#56B4E9", green="#009E73", grey="#7F7F7F", black="#000000")
D = np.load("theta.npy"); E = np.load("theta_estimators.npy"); c = D[:, 0]
th = D[:, 3]                                   # adopted: screw-rectified template estimate
dirn = np.array([-1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1], float)
up = np.zeros(len(c), bool); up[1:7] = True
def fit(y, lim=40):
    m = np.abs(c) <= lim; X = np.c_[c, np.ones_like(c), dirn]
    b, *_ = np.linalg.lstsq(X[m], y[m], rcond=None)
    ks = []
    for j in np.where(m)[0]:
        mm = m.copy(); mm[j] = False; bb, *_ = np.linalg.lstsq(X[mm], y[mm], rcond=None); ks.append(bb[0])
    ks = np.array(ks); n = len(ks)
    return b, np.sqrt((n - 1) / n * np.sum((ks - ks.mean()) ** 2))
b, ksd = fit(th); k = b[0]
SIG = 0.4                                      # per-level 1-sigma: pose drift + feature location + noise

# ---------------- Figure 1: the hub at five commands, body-stabilised averages
A = np.load("plateau_avg.npy"); S = np.array(json.load(open("screws_tm.json"))["full"])
sel = [18, 15, 12, 9, 6]
fig, ax = plt.subplots(1, 5, figsize=(15, 3.6))
cen = np.array([297.0, 397.0])
v0 = S[12, 1] - S[12, 0]; a0 = np.degrees(np.arctan2(v0[1], v0[0]))
for a_, k_ in zip(ax, sel):
    im = np.clip(A[k_], 0, 255)
    a_.imshow(im[cen[1].astype(int)-150:cen[1].astype(int)+150, cen[0].astype(int)-150:cen[0].astype(int)+150], cmap="gray", vmin=30, vmax=220)
    off = cen - 150
    p = S[k_] - off
    m = 0.5 * (S[12, 0] + S[12, 1]) - off
    u0 = v0 / np.linalg.norm(v0)
    a_.plot([m[0] - 75*u0[0], m[0] + 75*u0[0]], [m[1] - 75*u0[1], m[1] + 75*u0[1]], color=OI["sky"], ls="--", lw=1.4, label="screw line at 0° command")
    a_.plot(p[:, 0], p[:, 1], color=OI["orange"], lw=2.0, marker="o", ms=7, mfc="none", mew=2, label="screw line, this level")
    a_.set_title(f"command {c[k_]:+.0f}°  →  canard {th[k_] - th[12]:+.1f}°", fontsize=11)
    a_.set_xticks([]); a_.set_yticks([])
ax[0].legend(loc="lower left", fontsize=8, framealpha=0.8)
fig.suptitle("Fin 2 hub seen along the hinge axis (each image: one held level, ~50 frames averaged after removing camera shake). "
             "Command = cal jog zero at 8.889 µs per commanded degree", fontsize=10)
fig.tight_layout(); fig.savefig("bench02_hub_views.png", dpi=110); plt.close(fig)

# ---------------- Figure 2: angle map, residuals, estimators, consequence
fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
a1 = ax[0, 0]
a1.axvspan(-62, -45, color="#EEEEEE"); a1.axvspan(45, 62, color="#EEEEEE")
a1.text(53.5, -45, "outside the\n1100–1900 µs\ndata-sheet range", ha="center", fontsize=8, color="#555555")
xx = np.linspace(-60, 60, 2)
a1.plot(xx, xx, color=OI["grey"], ls=":", lw=1.5, label="assumed: 8.889 µs/° (data sheet; BENCH-01)")
a1.plot(xx, k*xx + b[1], color=OI["black"], lw=1.4, label=f"fit |cmd| ≤ 40°: canard = {k:.3f} × command")
a1.errorbar(c[up], th[up], yerr=SIG, fmt="^", color=OI["blue"], ms=7, capsize=2, label="up pass (0 → +60)")
a1.errorbar(c[~up], th[~up], yerr=SIG, fmt="v", color=OI["verm"], ms=7, capsize=2, label="down pass (+60 → −60)")
a1.set_xlabel("commanded angle, deg (at the firmware's 8.889 µs/°)"); a1.set_ylabel("measured canard angle, deg")
a1.set_xlim(-62, 62); a1.set_ylim(-62, 62); a1.legend(fontsize=8, loc="upper left")
sec = a1.secondary_xaxis("top", functions=(lambda x: 1520 + 8.889*x, lambda p: (p - 1520) / 8.889)); sec.set_xlabel("pulse width, µs")
a1.set_title("Canard angle against command", fontsize=10)
a2 = ax[0, 1]
r = th - (k*c + b[1] + b[2]*dirn)
a2.axhspan(-2*0.15, 2*0.15, color="#E8F1F8", label="±2σ servo landing scatter (BENCH-01/02)")
a2.errorbar(c[up], r[up], yerr=SIG, fmt="^", color=OI["blue"], ms=7, capsize=2, label="up pass")
a2.errorbar(c[~up], r[~up], yerr=SIG, fmt="v", color=OI["verm"], ms=7, capsize=2, label="down pass")
a2.axhline(0, color=OI["black"], lw=0.8)
for xv in (-45, 45): a2.axvline(xv, color=OI["grey"], ls=":", lw=1)
a2.set_xlabel("commanded angle, deg"); a2.set_ylabel("measured − fit, deg"); a2.legend(fontsize=8, loc="upper center")
a2.set_title("Residual from the linear fit: straight to ±40°, ends possibly steeper", fontsize=10)
a3 = ax[1, 0]
est = [("template, image plane", E[:, 1]), ("template, screw-rectified (adopted)", th), ("template, bearing-rectified", D[:, 2]),
       ("screw-head circle fit", E[:, 2]), ("counterbore circle fit", E[:, 3])]
for i, (nm, y) in enumerate(est):
    bb, sd = fit(y)
    a3.errorbar(bb[0], i, xerr=max(sd, 0.004), fmt="o", color=OI["green"] if "adopted" in nm else OI["blue"], ms=8, capsize=3)
    a3.text(bb[0], i - 0.32, f"{bb[0]:.3f}", ha="center", va="bottom", fontsize=8)
a3.errorbar(0.86, len(est), xerr=0.086, fmt="s", color=OI["orange"], ms=8, capsize=3)
a3.text(0.86, len(est) - 0.32, "0.84–0.88 at the nominal 24 mm; ±10 % from the focal length", ha="center", va="bottom", fontsize=8)
a3.axvline(1.0, color=OI["grey"], ls=":", lw=1.5); a3.text(0.997, 2.0, "1.0 = data sheet scale", fontsize=8, color="#555555", rotation=90, ha="right", va="center")
a3.axvspan(k - 0.015, k + 0.03, color="#E5F4EF")
a3.set_yticks(range(len(est) + 1)); a3.set_yticklabels([e[0] for e in est] + ["ceiling camera (laser dot)"], fontsize=8)
a3.set_xlim(0.72, 1.05); a3.set_xlabel("canard / command (fit |cmd| ≤ 40°, jackknife 1σ)"); a3.set_ylim(len(est) + 0.6, -0.95)
a3.set_title("All estimators 0.82–0.88 (circle fits are the noisy ones); shaded: adopted 0.86 +0.03/−0.015", fontsize=10)
a4 = ax[1, 1]
mot = ["G40W", "G53FJ", "G79W", "G80T", "G77R", "G104T"]
k1 = {"G40W": [18.4, 84.9], "G53FJ": [16.4, 77.8], "G79W": [19.3, 101.1], "G80T": [19.2, 108.4], "G77R": [20.0, 111.4], "G104T": [22.9, 115.2]}   # NV5a, SERVO_RANGE_DEG 116
k86 = {"G40W": [20.6, 95.2], "G53FJ": [18.5, 86.2], "G79W": [21.3, 118.9], "G80T": [21.8, 119.2], "G77R": [22.7, 122.6], "G104T": [25.9, 124.1]}
xi = np.arange(len(mot)); w = 0.38
a4.bar(xi - w/2, [k1[m][0] for m in mot], w, color=OI["sky"], label="corrected, SERVO_RANGE_DEG 116 (canard 0.98-1.04 x command)")
a4.bar(xi + w/2, [k86[m][0] for m in mot], w, color=OI["verm"], hatch="//", edgecolor="white", label="as built 10-15 Nov, 135 (canard 0.845-0.89 x command)")
a4.axhline(25, color=OI["black"], ls="--", lw=1); a4.text(5.45, 25.4, "goal G2 ≤ 25 °/s", fontsize=8, ha="right")
a4.set_xticks(xi); a4.set_xticklabels(mot); a4.set_ylabel("coast roll RMS p95, °/s"); a4.set_ylim(0, 34); a4.legend(fontsize=8, loc="upper left", ncol=1, framealpha=0.9)
a4.set_title("SITL, 600 flights per case: 600/600 corrected, 599/600 as built", fontsize=10)
fig.suptitle(f"BENCH-02 §11, 15 Nov 2026: fin 2 canard turns {k:.2f} of the command (servo ≈ {8.889/k:.1f} µs/°, not 8.889). "
             "Per-level error bars ±0.4° (camera pose drift, feature location)", fontsize=11)
fig.tight_layout(); fig.savefig("bench02_hub_scale.png", dpi=110); plt.close(fig)
print(k, ksd, b)
