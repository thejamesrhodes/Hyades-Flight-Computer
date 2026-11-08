"""Re-judge existing runs against the current mc.judge() without re-running: python rejudge.py runs/*"""
import csv, json, sys, os
import numpy as np
from mc import judge
for d in sys.argv[1:]:
    if not os.path.exists(os.path.join(d, "runs.csv")):
        continue
    R = []
    for r in csv.DictReader(open(os.path.join(d, "runs.csv"))):
        R.append({k: (float(v) if v.replace(".", "", 1).replace("-", "", 1).replace("e", "", 1).isdigit() else v) for k, v in r.items()})
    s = json.load(open(os.path.join(d, "summary.json")))
    C = [judge(r, s.get("shadow", False)) for r in R]
    s["pass"] = {k: round(100.0 * np.mean([c[k] for c in C]), 1) for k in C[0]}
    s["all_pass_pct"] = round(100.0 * np.mean([all(c.values()) for c in C]), 1)
    pk = np.array([max(r["p_max_boost"], r["p_max_coast"]) for r in R]); rm = np.array([r["p_rms_coast"] for r in R])
    s["goals"] = {"p95_peak_le_120": bool(np.percentile(pk, 95) <= 120), "p95_rms_le_25": bool(np.percentile(rm, 95) <= 25)}
    json.dump(s, open(os.path.join(d, "summary.json"), "w"), indent=1)
    print(os.path.basename(d).ljust(18), s["n"], "all", s["all_pass_pct"], {k: v for k, v in s["pass"].items() if v < 100}, s["goals"] if not s.get("shadow") else "")
