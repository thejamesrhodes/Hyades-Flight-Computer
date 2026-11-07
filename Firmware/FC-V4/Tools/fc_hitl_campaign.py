#!/usr/bin/env python3
"""
fc_hitl_campaign.py -- HITL worst-case / failure-hunting campaign (HITL-01 H1-H4).

    python fc_hitl_campaign.py                 # every case, ~40 min
    python fc_hitl_campaign.py --only vib      # cases whose name contains 'vib'
    python fc_hitl_campaign.py --list

Each case is one fc_hitl flight on the real board (fresh reboot each time) with
plant dispersions, environment extremes or injected faults. The aim is to FIND
failure modes, so many cases are deliberately beyond the expected envelope;
each states what the flight software should do ('expect'), and the report
marks where it did something else. Results: logs/hitl/campaign-<time>/ (one
csv/json per case + campaign.json); figures: hitl_figures.py.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fc_hitl  # noqa: E402

VIB_NOM = {"boost_g": 2.0, "res_g": 1.0, "res_hz": 180.0, "coast_g": 0.3, "chute_g": 0.1, "gyro_dps": 10.0}
VIB_WORST = {"boost_g": 6.0, "res_g": 4.0, "res_hz": 180.0, "coast_g": 1.5, "chute_g": 0.3, "gyro_dps": 40.0}
SERVO_WORST = {"f_N": 50.0, "Zeta": 0.5, "Tau_d": 0.030, "Deadband": 0.0070, "Lash": 0.0050}
FLIGHT = ["READY", "BOOST", "COAST", "DESCENT", "LANDED"]

# name, overrides, expect: {"phases": [...must appear in order], "no": [...must not appear], "note"}
CASES = [
    ("nominal",          {"cl0": 0.01},                                   {"phases": FLIGHT}),
    ("open_loop_cl0_03", {"cl0": 0.03, "closed": False},                  {"phases": FLIGHT, "note": "reference: no control"}),
    ("cl0_03_1sig",      {"cl0": 0.03},                                   {"phases": FLIGHT}),
    ("cl0_09_3sig",      {"cl0": 0.09},                                   {"phases": FLIGHT, "note": "beyond trim clamp: standing roll expected"}),
    ("wind9_gust6",      {"wind": 9.0, "cl0": 0.01, "sim": {"gust": (1.2, 1.0, 6.0, 90.0)}}, {"phases": FLIGHT}),
    ("aero_cld_m30",     {"cl0": 0.03, "sim": {"cld_scale": 0.7, "clp_scale": 0.7}}, {"phases": FLIGHT}),
    ("aero_cld_p30",     {"cl0": 0.03, "sim": {"cld_scale": 1.3, "clp_scale": 1.3}}, {"phases": FLIGHT}),
    ("ixx_m30",          {"cl0": 0.03, "sim": {"ixx_scale": 0.7}},       {"phases": FLIGHT}),
    ("servo_worst",      {"cl0": 0.03, "sim": {"servo": SERVO_WORST}},    {"phases": FLIGHT}),
    ("thrust_p10",       {"cl0": 0.03, "sim": {"thrust_scale": 1.10}},    {"phases": FLIGHT}),
    ("accsf_p3",         {"cl0": 0.01, "acc_sf": 0.03},                   {"phases": FLIGHT}),
    ("accsf_m3",         {"cl0": 0.01, "acc_sf": -0.03},                  {"phases": FLIGHT}),
    ("vib_nominal",      {"cl0": 0.01, "sim": {"vib": VIB_NOM}},          {"phases": FLIGHT}),
    ("vib_worst",        {"cl0": 0.01, "sim": {"vib": VIB_WORST}},        {"phases": FLIGHT}),
    ("combined_worst",   {"cl0": 0.06, "wind": 9.0, "acc_sf": 0.03,
                          "sim": {"cld_scale": 1.3, "clp_scale": 0.7, "servo": SERVO_WORST, "vib": VIB_WORST,
                                  "gust": (1.2, 1.0, 6.0, 90.0)}},       {"phases": FLIGHT}),
    ("drop_5fr_boost",   {"cl0": 0.01, "inject": [{"kind": "fail", "t0": 0.5, "t1": 0.55}]},
                         {"phases": FLIGHT, "note": "below the 10-frame imu_read persistence: ride through"}),
    ("drop_15fr_boost",  {"cl0": 0.01, "inject": [{"kind": "fail", "t0": 0.5, "t1": 0.65}], "run_through": True},
                         {"phases": ["READY", "BOOST", "CATO"], "note": "HARD in flight -> CATO_SAFE, fins centred"}),
    ("nan_sample_coast", {"cl0": 0.01, "inject": [{"kind": "nan", "t0": 3.0, "t1": 3.01}]}, {"phases": FLIGHT}),
    ("sat_10fr_coast",   {"cl0": 0.01, "inject": [{"kind": "sat", "t0": 3.0, "t1": 3.10}]}, {"phases": FLIGHT}),
    ("gyro_spike_coast", {"cl0": 0.01, "inject": [{"kind": "gyro_spike", "t0": 3.0, "t1": 3.01}]}, {"phases": FLIGHT}),
    ("gps_none",         {"cl0": 0.01, "gps": False},                     {"phases": FLIGHT, "note": "landing by the 300 s timer"}),
    ("gps_garbage_desc", {"cl0": 0.01, "gps_fault": {"garbage": (12.0, 40.0)}}, {"phases": FLIGHT}),
    ("gps_altbias_60",   {"cl0": 0.01, "gps_cfg": {"alt_bias": 60.0}},   {"phases": FLIGHT}),
    ("pad_knock_short",  {"cl0": 0.01, "pad_min": 20.0, "sim": {"pad_knocks": ((12.0, 6.0, 0.06, 2), (15.0, 8.0, 0.04, 0))}},
                         {"phases": FLIGHT, "no_before_ignition": ["BOOST"], "note": "60 ms 6 g knock must not be liftoff"}),
    ("pad_knock_long",   {"cl0": 0.01, "pad_min": 20.0, "sim": {"pad_knocks": ((12.0, 5.0, 0.12, 2),)}},
                         {"phases": FLIGHT, "no_before_ignition": ["BOOST"], "note": "120 ms 5 g axial drop/bump"}),
    ("pad_knock_extreme", {"cl0": 0.01, "pad_min": 20.0, "sim": {"pad_knocks": ((12.0, 10.0, 0.20, 2),)}},
                         {"phases": FLIGHT, "no_before_ignition": ["BOOST"],
                          "note": "probe: 10 g / 200 ms axial (~12 m/s) is beyond what a velocity gate can reject"}),
    ("pad_sway",         {"cl0": 0.01, "pad_min": 20.0, "sim": {"pad_sway_deg": 3.0}}, {"phases": FLIGHT}),
    ("i205w_nominal",    {"motor": "I205W", "cl0": 0.01},                 {"phases": FLIGHT, "note": "30 g: nav saturates, q fallback"}),
    ("i205w_vib",        {"motor": "I205W", "cl0": 0.03, "sim": {"vib": VIB_NOM}}, {"phases": FLIGHT}),
]

DEFAULTS = dict(mode="flight", port=None, frames=0, motor="G79W", closed=True, elev=110.0, wind=2.2, cl0=0.0,
                acc_sf=0.0, seed=1, gps=True, pad_min=10.0, pad_max=60.0, post_land=15.0, params=None,
                sim=None, inject=None, gps_fault=None, gps_cfg=None, run_through=False, ignite_regardless=False)


def judge(summ: dict, expect: dict) -> list[str]:
    issues = []
    seen = summ.get("phases_seen", [])
    want = expect.get("phases", [])
    it = iter(seen)
    if not all(any(p == q for q in it) for p in want):
        issues.append(f"phase sequence {seen} lacks {want}")
    for p in expect.get("no", []):
        if p in seen:
            issues.append(f"unexpected {p}")
    pre = summ.get("pre_ignition_phases", [])
    for p in expect.get("no_before_ignition", []):
        if p in pre:
            issues.append(f"{p} BEFORE ignition (false detection)")
    for k in ("reply_timeouts", "board_missing_samples"):
        if summ.get(k, 0):
            issues.append(f"{k}={summ[k]}")
    return issues


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", default="", help="comma-separated substrings")
    ap.add_argument("--append", help="existing campaign dir: add/replace these cases in its campaign.json")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--port")
    a = ap.parse_args()
    keys = [k for k in a.only.split(",") if k] or [""]
    exact = a.only.startswith("=")
    if exact:
        keys = [k.lstrip("=") for k in keys]
    cases = [c for c in CASES if any((k == c[0]) if exact else (k in c[0]) for k in keys)]
    if a.list:
        for n, o, e in cases:
            print(f"{n:20s} {e.get('note', '')}")
        return
    out = a.append or os.path.join(HERE, "logs", "hitl", "campaign-" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out, exist_ok=True)
    results = []
    if a.append and os.path.exists(os.path.join(out, "campaign.json")):
        names = {c[0] for c in cases}
        with open(os.path.join(out, "campaign.json")) as f:
            results = [r for r in json.load(f) if r.get("name") not in names]
    for i, (name, over, expect) in enumerate(cases):
        print(f"\n=== [{i + 1}/{len(cases)}] {name}: {expect.get('note', '')}")
        ns = types.SimpleNamespace(**{**DEFAULTS, **over, "name": name, "out_dir": out, "port": a.port})
        try:
            summ = fc_hitl.run(ns)
        except SystemExit as e:
            summ = {"name": name, "error": f"exit {e.code}"}
        except Exception as e:  # keep the campaign going; record the failure
            summ = {"name": name, "error": repr(e)}
        summ["case"] = {"overrides": over, "expect": expect}
        summ["issues"] = judge(summ, expect) if "error" not in summ else [summ["error"]]
        results.append(summ)
        with open(os.path.join(out, "campaign.json"), "w") as f:
            json.dump(results, f, indent=2, default=str)
        print(f"=== {name}: {'OK' if not summ['issues'] else 'ISSUES: ' + '; '.join(summ['issues'])}")
    print("\nSUMMARY")
    for r in results:
        print(f"  {r['name']:20s} {'OK' if not r['issues'] else '; '.join(r['issues'])}")
    print(f"results: {out}")


if __name__ == "__main__":
    main()
