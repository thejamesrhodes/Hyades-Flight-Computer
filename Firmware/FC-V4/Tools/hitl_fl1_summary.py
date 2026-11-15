#!/usr/bin/env python3
"""Summary table of a HITL campaign on the flight-1 build (SYSID-01 rev 2 / FIG-02).

    python hitl_fl1_summary.py logs/hitl/campaign-<time> [--old logs/hitl/campaign-<time>]

Per case: verdict, roll-test outcome from the board's EV_ROLLTEST events, peak and coast roll
(plant truth), canard command max, frame compute p99/max, and the dither seen on the pad
(max |fin| in READY: 0.17 deg with the old always-on dither, ~0 with the 15 Nov gate).
Writes summary.csv and summary.md into the campaign folder.
"""
import argparse, csv, json, os
import numpy as np


def coast_stats(csv_path):
    ph, p, f, lu = [], [], [], []
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            ph.append(r["phase"]); p.append(float(r["p_true_dps"])); lu.append(int(r["loop_us"]))
            f.append(max(abs(float(r[f"fin{k}"])) for k in range(4)))
    ph = np.array(ph); p = np.array(p); f = np.array(f) * 30.0; lu = np.array(lu)
    co = ph == "COAST"; ready = ph == "READY"; fl = (ph == "BOOST") | co
    return dict(coast_rms=float(np.sqrt(np.mean(p[co] ** 2))) if co.any() else float("nan"),
                peak=float(np.abs(p[fl]).max()) if fl.any() else float("nan"),
                pad_fin=float(f[ready].max()) if ready.any() else float("nan"),
                loop_fl_max=int(lu[fl].max()) if fl.any() else -1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    a = ap.parse_args()
    res = json.load(open(os.path.join(a.campaign, "campaign.json")))
    rows = []
    for r in res:
        name = r.get("name", "?")
        d = r                                     # campaign.json holds each case's full summary
        ev = [e.get("text", "") for e in d.get("board_events", d.get("events", [])) if isinstance(e, dict)]
        rt = [t for t in ev if t.startswith("rt ") and not t.startswith("rt start")]
        start = [t for t in ev if t.startswith("rt start")]
        csvp = os.path.join(a.campaign, os.path.basename(str(d.get("csv", "")).replace("\\", "/")))
        st = coast_stats(csvp) if os.path.exists(csvp) else {}
        roll = d.get("roll", {})
        rows.append(dict(case=name, verdict="OK" if not r.get("issues") else "; ".join(r["issues"]),
                         rolltest=(rt[-1] if rt else ("started, no end" if start else "-")),
                         peak_dps=round(st.get("peak", float("nan")), 0), coast_rms_dps=round(st.get("coast_rms", float("nan")), 1),
                         fin_max_deg=roll.get("fin_cmd_max_deg"), pad_fin_max_deg=round(st.get("pad_fin", float("nan")), 3),
                         loop_p99_us=d.get("loop_us_p99"), loop_max_us=d.get("loop_us_max"),
                         phases=" ".join(d.get("phases_seen", []))))
    keys = list(rows[0].keys())
    with open(os.path.join(a.campaign, "summary.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, keys); w.writeheader(); w.writerows(rows)
    with open(os.path.join(a.campaign, "summary.md"), "w") as fh:
        fh.write("| " + " | ".join(keys) + " |\n|" + "---|" * len(keys) + "\n")
        for r in rows:
            fh.write("| " + " | ".join(str(r[k]) for k in keys) + " |\n")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
