"""FIG-02 F2: roll performance through the project, SITL 6DOF Monte Carlo at build tolerance.

Values are the campaign results recorded in the docs (MC-01, LD-01, REVIEW-01, MC-01 §2 BENCH-02 notes,
SYSID-01 §2, §4, §9). p95 over the campaign; 'pass' is R1-R9 all-pass.
Run: python fig02_progress.py OUTDIR
"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

BLUE, VERM, PURPLE, GREY = "#0072B2", "#D55E00", "#CC79A7", "#8C8C8C"
# label, date, law, coast RMS p95, peak p95, pass %, n, source, kind
ROWS = [
    ("Law 1, assumed servo", "8 Nov", 1, 22.0, 112, 100.0, 1200, "MC-01 §4", "law1"),
    ("… CG corrected (LD-01)", "8 Nov", 1, 21.7, 111, 99.8, 1200, "LD-01 §3", "law1"),
    ("… measured servo, BENCH-01", "10 Nov", 1, 19.9, 111, 100.0, 1200, "REVIEW-01 §3", "law1"),
    ("… BENCH-02 servo, as built (scale 135)", "15 Nov", 1, 21.5, 118, 99.8, 600, "MC-01 §2", "law1"),
    ("… pulse scale corrected (116)", "15 Nov", 1, 19.1, 106, 100.0, 600, "MC-01 §2", "law1"),
    ("Observer, ω_o 30, β̂ nominal (study)", "15 Nov", 2, 4.2, 41, 100.0, 600, "SYSID-01 §4", "law2"),
    ("Observer, ω_o 20, slow servo model", "15 Nov", 2, 6.4, 55, 100.0, 600, "SYSID-01 §4", "law2"),
    ("Flight-1 build: + β̂ 1.3×, 1 kPa gate", "15 Nov", 2, 8.3, 69, 100.0, 600, "SYSID-01 §9.4", "fl1"),
    ("Flight-1 build with the roll test", "15 Nov", 2, 9.1, 136, 100.0, 600, "SYSID-01 §9.3", "fl1test"),
    ("Candidate: gate 300 Pa", "15 Nov", 2, 6.7, 59, 100.0, 600, "SYSID-01 §9.4", "cand"),
]
STY = {"law1": dict(color=BLUE), "law2": dict(color=VERM), "fl1": dict(color=VERM, ec="#222", lw=1.6),
       "fl1test": dict(color=PURPLE, ec="#222", lw=1.6), "cand": dict(color="white", ec=VERM, hatch="///", lw=1.2)}


def main(out):
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 6.2), sharey=True, gridspec_kw=dict(width_ratios=[1, 1]))
    n = len(ROWS); y = list(range(n))[::-1]
    for i, r in enumerate(ROWS):
        st = STY[r[8]]
        a.barh(y[i], r[3], height=0.62, **st)
        b.barh(y[i], r[4], height=0.62, **st)
        a.text(r[3] + 0.4, y[i], f"{r[3]:.1f}", va="center", fontsize=8)
        b.text(r[4] + 2, y[i], f"{r[4]:.0f}" + ("  (incl. the commanded 115 °/s)" if r[8] == "fl1test" else ""), va="center", fontsize=8)
    a.set_yticks(y, [f"{r[0]}  ({r[1]}, {r[6]}; {r[5]:g} %)" for r in ROWS], fontsize=8.6)
    a.axvline(25, color="#444", ls="--", lw=0.9); a.text(25.3, n - 0.45, "goal G2: p95 ≤ 25", fontsize=7.5, va="top")
    b.axvline(120, color="#444", ls="--", lw=0.9); b.text(122, n - 0.45, "goal G1: p95 ≤ 120", fontsize=7.5, va="top")
    a.set_xlabel("coast roll-rate RMS, p95 over the campaign (°/s)"); b.set_xlabel("peak roll rate, p95 (°/s)")
    a.set_xlim(0, 28); b.set_xlim(0, 175)
    for ax in (a, b):
        [ax.spines[s].set_visible(False) for s in ("top", "right")]; ax.grid(axis="x", alpha=0.25)
    a.set_title("Roll control through the project (SITL 6DOF, build tolerance; label: date, flights, all-pass)", fontsize=10.5, loc="left")
    fig.legend(handles=[Patch(color=BLUE, label="law 1: rate + trim integrator"), Patch(color=VERM, label="law 2: rate + observer (ESO)"),
                        Patch(fc=VERM, ec="#222", lw=1.6, label="as flashed for flight 1"),
                        Patch(fc=PURPLE, ec="#222", lw=1.6, label="as flashed, with the roll test (coast RMS excludes the test)"),
                        Patch(fc="white", ec=VERM, hatch="///", label="candidate, not flashed")],
               loc="lower center", ncol=3, fontsize=8, frameon=False, bbox_to_anchor=(0.55, -0.06))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig02_roll_progress.png"), dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    os.makedirs(sys.argv[-1] if len(sys.argv) > 1 else ".", exist_ok=True)
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
