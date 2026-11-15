import numpy as np, json
from scipy.optimize import least_squares
P = json.load(open("plateaus.json")); CMD = np.array(P["CMD"], float)
T = json.load(open("screws_tm.json")); S = np.array(T["full"]); Hs = np.array(T["half"])
RJ = json.load(open("ring_joint.json")); a = np.array(RJ["a"])
def rect(v, q, phd):
    ph = np.radians(phd); c, s = np.cos(ph), np.sin(ph)
    u = c*v[..., 0] + s*v[..., 1]; w = -s*v[..., 0] + c*v[..., 1]
    w = w / q
    return np.stack([c*u - s*w, s*u + c*w], -1)
V = S[:, 1] - S[:, 0]
def angles(q, phd, Vv=V):
    r = rect(Vv, q, phd); ang = np.degrees(np.unwrap(np.arctan2(r[..., 1], r[..., 0]), axis=0))
    return ang - ang[0], np.hypot(r[..., 0], r[..., 1])
# 1) ring rectification
th_ring, sep = angles(RJ["q"], RJ["phi_deg"])
rs = sep / a
print("ring-rectified screw separation / ring scale: mean %.4f, sd %.4f (%.2f %%)" % (rs.mean(), rs.std(), 100*rs.std()/rs.mean()))
raw = sep / a; th_raw = np.degrees(np.unwrap(np.arctan2(V[:, 1], V[:, 0]))); th_raw -= th_raw[0]
print("unrectified sep / scale sd %.2f %%" % (100 * (np.hypot(V[:,0], V[:,1]) / a).std() / (np.hypot(V[:,0], V[:,1]) / a).mean()))
# 2) screws-only shape, with the ring's per-level scale: the separation/scale must be constant after rectification
def res(x):
    q, phd, rho = x
    _, sp = angles(q, phd)
    return sp / a - rho
f = least_squares(res, [0.95, 120.0, 0.96], bounds=([0.8, 0, 0.5], [1.0, 180, 1.5]))
qs, ps, rho = f.x
J = f.jac; cov = np.linalg.inv(J.T @ J) * (f.fun @ f.fun) / (len(f.fun) - 3)
print(f"screws-only shape: q {qs:.4f} +/- {np.sqrt(cov[0,0]):.4f}, major axis {ps:.1f} +/- {np.sqrt(cov[1,1]):.1f} deg, residual sd {f.fun.std()*100/rho:.2f} %")
th_scr, _ = angles(qs, ps)
print("\n cmd   image   ring-rect  screw-rect")
for k in range(len(CMD)):
    print(f"{CMD[k]:+5.0f} {th_raw[k]:+8.2f} {th_ring[k]:+9.2f} {th_scr[k]:+10.2f}")
np.save("theta.npy", np.c_[CMD, th_raw, th_ring, th_scr])
