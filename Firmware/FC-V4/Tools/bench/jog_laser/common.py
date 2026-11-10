"""Pixel-to-degree calibration from the 10 deg jog levels (res.json, written by analyse.py)."""
import json, numpy as np
from scipy.interpolate import PchipInterpolator
from tracklib import load

S = json.load(open("steps2.json")); R0 = json.load(open("res.json"))
CAL = {}
for f in ("83", "84"):
    lv = R0[f]["levels"]; A = np.array(sorted(map(float, lv)))
    Y = np.array([np.mean(lv[next(k for k in lv if float(k) == a)]) for a in A])
    if f == "83":    # projective map in tan(theta): residual < 0.06 deg at every level
        a, b, c = R0[f]["cal"]
        CAL[f] = lambda yy, a=a, b=b, c=c: np.degrees(np.arctan((np.asarray(yy) - b)/(a - c*np.asarray(yy))))
    else:            # setup B's level scatter (+/-0.7 deg) defeats the projective fit: monotone interpolation
        o = np.argsort(Y); pc = PchipInterpolator(Y[o], A[o], extrapolate=True)
        CAL[f] = lambda yy, pc=pc: pc(np.asarray(yy))
