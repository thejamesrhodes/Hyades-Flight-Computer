#!/usr/bin/env python3
"""
hitl_figures.py -- figures from a fc_hitl_campaign run (HITL-01 / NAV-01 report).

    python hitl_figures.py logs/hitl/campaign-<time> [--out ../../../Documentation/Images/hitl]

Writes PNGs: flight profile (nominal), q_hat accuracy, roll control (closed vs
open loop), vibration effect, worst-case roll envelope, event-timing errors,
campaign matrix, I205W saturation. Truth comes from the plant; 'board' is what
the flight computer reported over the HITL link.
"""
from __future__ import annotations

import argparse
import csv
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

C_TRUE, C_BOARD, C_ALT = "#000000", "#0072B2", "#D55E00"      # Okabe-Ito
PH_COL = {"BOOST": "#E69F00", "COAST": "#56B4E9", "DESCENT": "#009E73", "LANDED": "#999999",
          "CATO": "#CC0000", "SETTLE": "#F0E442", "READY": "#FFFFFF"}


def load_csv(path: str) -> dict:
    with open(path) as f:
        r = list(csv.DictReader(f))
    out = {}
    for k in r[0].keys():
        try:
            out[k] = np.array([float(x[k]) for x in r])
        except ValueError:
            out[k] = np.array([x[k] for x in r])
    return out


def phase_bands(ax, d, tmax=None):
    t, ph = d["t"], d["phase"]
    start = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or ph[i] != ph[start]:
            if t[start] >= 0 and ph[start] in PH_COL and ph[start] != "READY":
                ax.axvspan(t[start], t[i - 1], color=PH_COL[ph[start]], alpha=0.12, lw=0)
            start = i
    if tmax:
        ax.set_xlim(0, tmax)


def case(res, name):
    for r in res:
        if r.get("name") == name and "csv" in r:
            return r, load_csv(r["csv"])
    return None, None


def fig_profile(res, out):
    r, d = case(res, "nominal")
    if d is None:
        return
    fl = d["t"] >= 0
    tl = float(d["t"][fl].max())
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    for a in ax:
        phase_bands(a, d, tl)
    ax[0].plot(d["t"][fl], d["h_true"][fl], C_TRUE, lw=1.6, label="truth")
    ax[0].plot(d["t"][fl], d["h"][fl], C_BOARD, lw=1, ls="--", label="board (NAV-01)")
    ax[0].set_ylabel("height above pad (m)")
    ax[1].plot(d["t"][fl], d["vz_true"][fl], C_TRUE, lw=1.6)
    ax[1].plot(d["t"][fl], d["v_up"][fl], C_BOARD, lw=1, ls="--")
    ax[1].set_ylabel("v_up (m/s)")
    ax[2].plot(d["t"][fl], d["qbar_true"][fl] / 1e3, C_TRUE, lw=1.6)
    ax[2].plot(d["t"][fl], d["qbar"][fl] / 1e3, C_BOARD, lw=1, ls="--")
    ax[2].set_ylabel("dynamic pressure (kPa)")
    ax[3].plot(d["t"][fl], d["p_true_dps"][fl], C_TRUE, lw=1.2, label="roll rate (truth)")
    ax3 = ax[3].twinx()
    ax3.plot(d["t"][fl], d["fin0"][fl] * 30.0, C_ALT, lw=0.8, label="canard command")
    ax3.set_ylabel("canard cmd (deg)", color=C_ALT)
    ax[3].set_ylabel("roll rate (deg/s)")
    ax[3].set_xlabel("time from ignition (s)")
    ax[0].legend(loc="upper right")
    ev = r.get("events", {})
    ttl = "HITL nominal flight, G79W, closed loop (q-scheduled roll law)\n" + "  ".join(
        f"{k}: board {v.get('board_s')} / truth {v.get('truth_s', '-')} s" for k, v in ev.items())
    fig.suptitle(ttl, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_flight_profile_nominal.png"), dpi=150)
    # ascent zoom
    for a in ax:
        a.set_xlim(0, (r.get("events", {}).get("DESCENT", {}).get("truth_s") or 10) + 1)
    fig.savefig(os.path.join(out, "hitl_flight_profile_ascent.png"), dpi=150)
    plt.close(fig)


def fig_qhat(res, out):
    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    for name, col in (("nominal", C_BOARD), ("accsf_p3", C_ALT), ("accsf_m3", "#009E73"),
                      ("vib_nominal", "#CC79A7"), ("vib_worst", "#E69F00")):
        r, d = case(res, name)
        if d is None:
            continue
        ta = r.get("events", {}).get("DESCENT", {}).get("truth_s") or 10
        m = (d["t"] > 0.1) & (d["t"] < ta) & (d["qbar_true"] > 2000)
        ax.plot(d["t"][m], (d["qbar"][m] - d["qbar_true"][m]) / d["qbar_true"][m] * 100, color=col, lw=1, label=name)
    ax.axhspan(-30, 30, color="#009E73", alpha=0.07, label="schedule tolerance ±30 %")
    ax.set_ylim(-60, 60)
    ax.set_xlabel("time from ignition (s)")
    ax.set_ylabel("q̂ error (%)")
    ax.set_title("Dynamic-pressure estimate error, rail exit to apogee (q > 2 kPa)", fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_qhat_error.png"), dpi=150)
    plt.close(fig)


def fig_roll(res, out):
    fig, ax = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for name, col, lab in (("open_loop_cl0_03", C_TRUE, "open loop, Cl0 = 0.03"),
                           ("cl0_03_1sig", C_BOARD, "closed loop, Cl0 = 0.03"),
                           ("cl0_09_3sig", C_ALT, "closed loop, Cl0 = 0.09 (3σ)"),
                           ("servo_worst", "#CC79A7", "closed, worst servo"),
                           ("combined_worst", "#E69F00", "closed, combined worst")):
        r, d = case(res, name)
        if d is None:
            continue
        ta = (r.get("events", {}).get("DESCENT", {}).get("truth_s") or 10)
        m = (d["t"] >= 0) & (d["t"] <= ta)
        ax[0].plot(d["t"][m], d["p_true_dps"][m], color=col, lw=1, label=lab)
        ax[1].plot(d["t"][m], d["delta_true_deg"][m], color=col, lw=1)
    ax[0].set_ylabel("roll rate (deg/s)")
    ax[1].set_ylabel("canard deflection, actual (deg)")
    ax[1].axhline(12, color="#999999", ls=":"); ax[1].axhline(-12, color="#999999", ls=":")
    ax[1].set_xlabel("time from ignition (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Roll control in HITL (G79W): flight firmware closing the loop on the simulated vehicle", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_roll_control.png"), dpi=150)
    plt.close(fig)


def fig_vib(res, out):
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for name, col in (("nominal", C_BOARD), ("vib_nominal", "#CC79A7"), ("vib_worst", C_ALT)):
        r, d = case(res, name)
        if d is None:
            continue
        m = (d["t"] >= 0) & (d["t"] <= 4)
        ax[0].plot(d["t"][m], d["az_meas"][m] / 9.80665, color=col, lw=0.7, label=f"{name} (clips {r.get('imu_internal_clips', 0)})")
        ax[1].plot(d["t"][m], d["v_up"][m] - d["vz_true"][m], color=col, lw=1)
    ax[0].axhline(16, color="#999999", ls=":")
    ax[0].set_ylabel("axial accel seen by board (g)")
    ax[1].set_ylabel("v_up error (m/s)")
    ax[1].set_xlabel("time from ignition (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Boost vibration: clipping inside the sensor rectifies into a velocity error", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_vibration.png"), dpi=150)
    plt.close(fig)


def fig_i205(res, out):
    r, d = case(res, "i205w_nominal")
    if d is None:
        return
    m = (d["t"] >= 0) & (d["t"] <= 20)
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for a in ax:
        phase_bands(a, d, 20)
    ax[0].plot(d["t"][m], d["az_meas"][m] / 9.80665, C_BOARD, lw=0.8, label="measured axial (g)")
    ax[0].axhline(16, color=C_ALT, ls=":", label="±16 g full scale")
    ax[0].legend(fontsize=8)
    ax[1].plot(d["t"][m], d["vz_true"][m], C_TRUE, lw=1.5, label="truth")
    ax[1].plot(d["t"][m], d["v_up"][m], C_BOARD, lw=1, ls="--", label="board")
    ax[1].set_ylabel("v_up (m/s)"); ax[1].legend(fontsize=8)
    ax[2].plot(d["t"][m], d["p_true_dps"][m], C_TRUE, lw=1)
    ax[2].set_ylabel("roll rate (deg/s)"); ax[2].set_xlabel("time from ignition (s)")
    ax[0].set_title("I205W (30 g peak): accelerometer saturation -> q̂ invalid, schedule at fallback gain", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_i205w_saturation.png"), dpi=150)
    plt.close(fig)


def fig_matrix(res, out):
    names = [r["name"] for r in res]
    prms = [r.get("roll", {}).get("p_rms_dps", np.nan) for r in res]
    pmax = [r.get("roll", {}).get("p_max_abs_dps", np.nan) for r in res]
    qerr = [r.get("qhat_err_pct", {}).get("max_abs", np.nan) for r in res]
    bad = [bool(r.get("issues")) for r in res]
    fig, ax = plt.subplots(1, 3, figsize=(13, 0.32 * len(names) + 1.5), sharey=True)
    y = np.arange(len(names))
    cols = ["#CC0000" if b else C_BOARD for b in bad]
    ax[0].barh(y, prms, color=cols); ax[0].set_xlabel("roll rate RMS, ascent (deg/s, log)")
    ax[1].barh(y, pmax, color=cols); ax[1].set_xlabel("max |roll rate| (deg/s, log)")
    for a in ax[:2]:
        a.set_xscale("log"); a.set_xlim(1, 1e4)
    ax[2].barh(y, np.minimum(qerr, 100), color=cols); ax[2].set_xlabel("max |q̂ error| (%, rail exit to apogee)")
    ax[2].axvline(30, color=C_ALT, ls=":")
    ax[0].set_yticks(y); ax[0].set_yticklabels(names, fontsize=8); ax[0].invert_yaxis()
    fig.suptitle("HITL campaign (red = flight software did not do what was expected)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_campaign_matrix.png"), dpi=150)
    plt.close(fig)


def fig_events(res, out):
    rows = []
    for r in res:
        ev = r.get("events") or {}
        e = []
        for k in ("BOOST", "COAST", "DESCENT"):
            b, t = (ev.get(k) or {}).get("board_s"), (ev.get(k) or {}).get("truth_s")
            e.append(np.nan if b is None or t is None else b - t)
        rows.append((r["name"], e))
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(9, 0.3 * len(rows) + 1.5))
    y = np.arange(len(rows))
    for j, (k, col) in enumerate((("liftoff", C_BOARD), ("burnout", C_ALT), ("apogee", "#009E73"))):
        ax.scatter([e[j] for _n, e in rows], y + (j - 1) * 0.2, color=col, s=14, label=k)
    ax.axvline(0, color="#999999")
    ax.set_yticks(y); ax.set_yticklabels([n for n, _e in rows], fontsize=8); ax.invert_yaxis()
    ax.set_xlabel("board detection - truth (s)")
    ax.legend(fontsize=8)
    ax.set_title("Flight-event detection timing across the campaign", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "hitl_event_timing.png"), dpi=150)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("campaign")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
                                                  "Documentation", "Images", "hitl"))
    a = ap.parse_args()
    with open(os.path.join(a.campaign, "campaign.json")) as f:
        res = json.load(f)
    for r in res:                                # csv paths were written by Windows
        if "csv" in r:
            r["csv"] = os.path.join(a.campaign, os.path.basename(r["csv"].replace("\\", "/")))
    os.makedirs(a.out, exist_ok=True)
    for fn in (fig_profile, fig_qhat, fig_roll, fig_vib, fig_i205, fig_matrix, fig_events):
        try:
            fn(res, a.out)
        except Exception as e:                   # one bad case must not stop the rest
            print(f"{fn.__name__}: {e!r}")
    print("figures ->", os.path.abspath(a.out))


if __name__ == "__main__":
    main()
