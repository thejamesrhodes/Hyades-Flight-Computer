#!/usr/bin/env python3
"""
hitl_crosscheck.py -- HITL-01 H0b: does hitl_model.py reproduce the Matlab
roll model? Repeats hitl_crosscheck.m (canard_servo_reduced + the Simulink
'Plant' block, fixed step P.Servo.dt) with the Python port, reading the same
sim_params.json, and compares against hitl_crosscheck_matlab.csv.

    python hitl_crosscheck.py          # exit 1 if any trace is outside 2 % RMS
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hitl_model as hm                                     # noqa: E402

TOL_PCT = 2.0


def main() -> int:
    P, src = hm.load_params()
    ref_path = os.path.join(HERE, "hitl_crosscheck_matlab.csv")
    if not os.path.exists(ref_path):
        print("no hitl_crosscheck_matlab.csv: run hitl_crosscheck.m in Matlab first")
        return 2
    ref = np.loadtxt(ref_path, delimiter=",")
    t, cmd = ref[:, 0], ref[:, 1]
    dt = P["Servo"]["dt"]
    sv = hm.Servo(P)
    V, q = P["Vehicle"]["V"], P["Aero"]["q"]
    Cld, Clp = hm.roll_coeffs(P, V / 343.0)
    d, ixx = P["Vehicle"]["d"], P["Vehicle"]["Ixx"]
    A = math.pi * (d / 2) ** 2
    delta, p, pk = np.zeros_like(t), np.zeros_like(t), 0.0
    for i, u in enumerate(cmd):
        delta[i] = sv.step(float(u))
        p[i] = pk
        pk += (q * A * d * (Cld * delta[i] + P["Aero"]["Cl_0"]) + q * A * d * d * Clp * pk / (2 * V)) / ixx * dt
    ok = True
    print(f"params: {src}; {len(t)} samples at dt={dt}")
    for name, a, b in (("servo delta", delta, ref[:, 2]), ("roll rate p", p, ref[:, 3])):
        rms = float(np.sqrt(np.mean((a - b) ** 2)))
        scale = float(np.sqrt(np.mean(b ** 2)))
        pct = 100 * rms / scale if scale > 0 else 0.0
        verdict = "PASS" if pct <= TOL_PCT else "FAIL"
        ok &= pct <= TOL_PCT
        print(f"  {name:12s} rms diff {rms:.3e} ({pct:.3f} % of rms signal)  max {np.abs(a - b).max():.3e}  {verdict}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
