import numpy as np, cv2, json
exec(open("circfit.py").read().split("out = []")[0])       # reuse circle() and the data
dirn = np.array([-1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1], float)
la = lambda v: np.degrees(np.unwrap(np.arctan2(v[:, 1], v[:, 0])))
res = {}
for name, rr in (("head r 16.5-19.5", np.arange(16.5, 19.51, 0.25)), ("counterbore r 23.5-26.5", np.arange(23.5, 26.51, 0.25))):
    C = np.array([[circle(A[k], S[k, s], rr=rr)[0] for s in range(2)] for k in range(len(A))])
    a = la(C[:, 1] - C[:, 0]); a -= a[0]; res[name] = a
Vt = S[:, 1] - S[:, 0]; a_t = la(Vt); a_t -= a_t[0]; res["template"] = a_t
np.save("theta_estimators.npy", np.c_[CMD, res["template"], res["head r 16.5-19.5"], res["counterbore r 23.5-26.5"]])
print(" cmd  template  head   counterbore")
for k in range(len(CMD)): print(f"{CMD[k]:+4.0f} {res['template'][k]:+8.2f} {res['head r 16.5-19.5'][k]:+8.2f} {res['counterbore r 23.5-26.5'][k]:+8.2f}")
for nm, a in res.items():
    for lim in (30, 60):
        m = np.abs(CMD) <= lim; X = np.c_[CMD, np.ones_like(CMD), dirn][m]
        b, *_ = np.linalg.lstsq(X, a[m], rcond=None); r = a[m] - X @ b
        print(f"{nm:24s} |cmd|<={lim}: k {b[0]:.4f}  resid sd {r.std():.3f}")
