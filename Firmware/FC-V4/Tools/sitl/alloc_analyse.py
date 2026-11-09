"""ALLOC-01: paired comparison of the allocation / PWM-tick variants.

Every variant ran the same seeds (same dispersions), so each flight is compared
with itself under the baseline (A1: the firmware before ALLOC-01, 1 MHz tick,
common-mode dither). Prints a table and writes runs/alloc/<set>/summary_alloc.json.

    python alloc_analyse.py runs/alloc
"""
import csv, json, os, sys
import numpy as np
from scipy.stats import wilcoxon

BASE = "A1_1MHz"
ORDER = ["A0_continuous", "A1_1MHz", "B1_8MHz", "C1_1MHz_stag", "D1_1MHz_null", "E1_8MHz_stag_null",
         "F1_8MHz_stag_nodither", "G1_1MHz_nodither", "H1_8MHz_null"]
METRICS = [("p_rms_coast", "deg/s"), ("p_rms_boost", "deg/s"), ("p_max", "deg/s"), ("wl_rms_coast", "deg/s"),
           ("pyd_rms", "deg"), ("sv_act", "deg/s"), ("sat_frac", "-")]


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, "runs.csv"))))
    out = {}
    for r in rows:
        v = {k: float(r[k]) for k in ("p_rms_coast", "p_rms_boost", "p_max_boost", "p_max_coast", "wl_rms_coast",
                                       "pyd_rms", "sv_act", "sat_frac")}
        v["p_max"] = max(v["p_max_boost"], v["p_max_coast"])
        out[int(float(r["seed"]))] = v
    return out


def main(root):
    data = {n: load(os.path.join(root, n)) for n in ORDER if os.path.exists(os.path.join(root, n, "runs.csv"))}
    seeds = sorted(set.intersection(*[set(d) for d in data.values()]))
    res = {"n": len(seeds), "variants": {}}
    print(f"n = {len(seeds)} paired flights per variant\n")
    hdr = f"{'variant':24s}" + "".join(f"{m:>22s}" for m, _ in METRICS)
    print(hdr)
    for name, d in data.items():
        row = {}
        line = f"{name:24s}"
        for m, _ in METRICS:
            v = np.array([d[s][m] for s in seeds])
            b = np.array([data[BASE][s][m] for s in seeds])
            p50, p95 = np.percentile(v, 50), np.percentile(v, 95)
            diff = v - b
            p = float(wilcoxon(diff).pvalue) if (name != BASE and np.any(diff != 0)) else 1.0
            # bootstrap 95 % interval of the change in p95 (paired resample)
            rng = np.random.default_rng(1)
            idx = rng.integers(0, len(seeds), (2000, len(seeds)))
            dp95 = np.percentile(v[idx], 95, axis=1) - np.percentile(b[idx], 95, axis=1)
            row[m] = {"p50": round(float(p50), 4), "p95": round(float(p95), 4), "mean": round(float(v.mean()), 4),
                      "d_median": round(float(np.median(diff)), 4), "wilcoxon_p": p,
                      "d_p95_ci": [round(float(np.percentile(dp95, 2.5)), 4), round(float(np.percentile(dp95, 97.5)), 4)]}
            line += f"{p50:9.3f} /{p95:9.3f}  "
        res["variants"][name] = row
        print(line)
    print("\npaired change vs", BASE, "(median of per-flight difference; Wilcoxon p)")
    for name, row in res["variants"].items():
        if name == BASE:
            continue
        print(f"{name:24s}" + "".join(f"{row[m]['d_median']:+10.3f} (p={row[m]['wilcoxon_p']:.1e})" for m, _ in METRICS))
    json.dump(res, open(os.path.join(root, "summary_alloc.json"), "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "runs/alloc")
