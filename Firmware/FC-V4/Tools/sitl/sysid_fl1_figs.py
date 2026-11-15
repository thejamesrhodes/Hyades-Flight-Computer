"""SYSID-01 rev 2 figure: the flight-1 roll test (completion by motor) and how well beta is known from each source.

    python sysid_fl1_figs.py      # reads runs/F1_flight1, runs/G1_fl1_final, runs/M_fl1_final; writes ../../Documentation/Images/sysid01/
"""
import json, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "..", "..", "Documentation", "Images", "sysid01")
if not os.path.isdir(os.path.join(HERE, "..", "..", "..", "..", "Documentation")):
    OUT = os.path.join(HERE, "figs")
os.makedirs(OUT, exist_ok=True)
# Okabe-Ito
BLUE, ORANGE, GREEN, VERM, SKY, GREY, PURPLE = "#0072B2", "#E69F00", "#009E73", "#D55E00", "#56B4E9", "#7F7F7F", "#CC79A7"
MOTORS = ["G53FJ", "G40W", "G104T", "G79W", "G77R", "G80T"]          # by q at COAST entry, lowest first


def done_pct(run):
    d = pd.read_csv(os.path.join(HERE, "runs", run, "runs.csv"))
    d["m"] = d.motor.str.extract(r"(G\w+)\.csv")[0]
    g = d.groupby("m").rt_state
    return {m: (100 * (g.get_group(m) == 5).mean(), 100 * (g.get_group(m) == 7).mean(), 100 * (g.get_group(m) == 6).mean()) for m in MOTORS}


def main():
    first, final = done_pct("F1_flight1"), done_pct("G1_fl1_final")
    s = json.load(open(os.path.join(HERE, "runs", "M_fl1_final", "summary.json")))
    rng = np.random.default_rng(1)
    cld = rng.uniform(0.7, 1.3, 200000); ixx = rng.uniform(0.68, 1.6, 200000)
    prior = np.percentile(ixx / cld, [5, 50, 95])                       # nominal / true
    tri = np.percentile(rng.normal(1, 0.02, 200000) / cld, [5, 50, 95])
    g = 0.11; ground = [np.exp(-1.645 * g), 1.0, np.exp(1.645 * g)]   # [I] budget, section 10

    fig, (a, b) = plt.subplots(1, 2, figsize=(12.5, 4.8), gridspec_kw=dict(width_ratios=[1, 1.25]))
    x = np.arange(len(MOTORS)); w = 0.38
    a.bar(x - w / 2, [first[m][0] for m in MOTORS], w, color=GREY, label="first schedule: +0.25 s, hold 0.8 s, abort 1.5 kPa")
    a.bar(x + w / 2, [final[m][0] for m in MOTORS], w, color=BLUE, label="flight 1: +0.15 s, hold 0.6 s, abort 1.2 kPa")
    for i, m in enumerate(MOTORS):
        a.text(x[i] - w / 2, first[m][0] + 1.5, f"{first[m][0]:.0f}", ha="center", fontsize=7, color="#333")
        a.text(x[i] + w / 2, final[m][0] + 1.5, f"{final[m][0]:.0f}", ha="center", fontsize=7, color="#333")
    a.set_xticks(x, MOTORS); a.set_ylim(0, 128); a.set_ylabel("roll test completed (% of 100 flights)")
    a.set_title("(a) Roll test completed in coast, by motor (SITL, build tolerance)", fontsize=10, loc="left")
    a.legend(loc="upper left", fontsize=8, frameon=False, ncol=1)
    sk = final["G53FJ"][2]; ab = final["G53FJ"][1]
    a.text(0.0, -0.13, "Motors ordered by q at burnout, lowest first. An unfinished test ends in the doublet's second step\n"
           f"(q below the abort level), after the 3-2-1-1 and the first step. Flight 1 on G53FJ: {ab:.0f} % cut short, {sk:.0f} % not started (q < 3 kPa).",
           transform=a.transAxes, fontsize=7.5, color="#444", va="top")
    for sp in ("top", "right"):
        a.spines[sp].set_visible(False); b.spines[sp].set_visible(False)

    rows = [("No measurement (Barrowman Cl_δ, Ixx guess)", prior, GREY, "prior"),
            ("Trifilar Ixx only (G2)", tri, SKY, "ground"),
            ("Trifilar + car-roof roll rig (G2 + G13)", ground, BLUE, "estimate"),
            ("Flight 1, log as recorded", s["log_beta_ratio_p5_50_95"], ORANGE, "flight"),
            ("Flight 1 + onboard video", s["video_beta_ratio_p5_50_95"], VERM, "flight"),
            ("Flight 1 + video + fine command log (S14)", s["video_fine_beta_ratio_p5_50_95"], PURPLE, "flight")]
    b.axvspan(0.2, 1 / 3, color=VERM, alpha=0.12, lw=0)
    b.text(0.205, -0.45, "observer unstable:\ntrue β > 3× estimate", fontsize=7.5, color=VERM, va="bottom")
    b.axvline(1, color="#333", lw=0.8); b.axvline(2, color=GREEN, lw=1, ls="--")
    b.text(2.03, -0.45, "2× high:\ntested, no loss", fontsize=7.5, color=GREEN, va="bottom")
    for i, (lab, (lo, med, hi), col, kind) in enumerate(rows[::-1]):
        b.plot([lo, hi], [i, i], color=col, lw=6, solid_capstyle="butt", alpha=0.55 if kind == "estimate" else 1.0)
        b.plot(med, i, "o", color="white", mec=col, mew=2, ms=7)
        b.text(hi * 1.04, i, f"{lo:.2f}–{hi:.2f}", va="center", fontsize=7.5, color="#333")
    b.set_yticks(range(len(rows)), [r[0] for r in rows[::-1]], fontsize=8.5)
    b.set_xscale("log"); b.set_xlim(0.2, 3.0); b.set_ylim(-0.6, len(rows) - 0.5)
    b.xaxis.set_minor_locator(matplotlib.ticker.NullLocator()); b.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    b.set_xticks([0.25, 0.33, 0.5, 0.75, 1, 1.5, 2, 3], ["0.25", "1/3", "0.5", "0.75", "1", "1.5", "2", "3"])
    b.set_xlabel("estimate ÷ true β  (bar 5–95 %, dot median)")
    b.set_title("(b) How well β = S d Cl_δ / Ixx is known, by source", fontsize=10, loc="left")
    b.text(0.0, -0.13, "Ground rig row is a budget estimate [I], not simulated (lighter bar). Flight rows: SITL, 120 flights,\n"
           "equation error; their low median is estimator bias, removable by the SITL calibration.",
           transform=b.transAxes, fontsize=7.5, color="#444", va="top")
    fig.tight_layout()
    p = os.path.join(OUT, "sysid_flight1_beta.png"); fig.savefig(p, dpi=150, bbox_inches="tight"); print(p)


if __name__ == "__main__":
    main()
