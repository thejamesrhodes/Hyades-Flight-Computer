import numpy as np, cv2, json
from scipy.optimize import least_squares
A = np.load("plateau_avg.npy"); R = json.load(open("ring.json"))
K = len(A)
def ray_points(img, c, rnom=125.0, win=12):
    img = cv2.GaussianBlur(img, (0, 0), 1.0); pts = []
    for a in np.radians(np.arange(0, 360, 0.5)):
        rr = np.arange(rnom - win, rnom + win, 0.25)
        xs = (c[0] + rr*np.cos(a)).astype(np.float32); ys = (c[1] + rr*np.sin(a)).astype(np.float32)
        v = cv2.remap(img, xs[None, :], ys[None, :], cv2.INTER_LINEAR)[0]
        j = int(np.argmin(v))
        if 3 <= j < len(v) - 3:
            den = v[j-1] - 2*v[j] + v[j+1]; dj = 0.5*(v[j-1] - v[j+1])/den if den > 0 else 0.0
            depth = 0.5*(v[:j+1].max() + v[j:].max()) - v[j]
            pts.append((xs[j] + 0.25*dj*np.cos(a), ys[j] + 0.25*dj*np.sin(a), depth))
    return np.array(pts)
PTS = []
for k in range(K):
    c = R[k]["e"][:2]
    p = ray_points(A[k], c); PTS.append(p[p[:, 2] > np.percentile(p[:, 2], 30)])
def unpack(x):
    q, ph = x[0], x[1]; cs = x[2:2+2*K].reshape(K, 2); a = x[2+2*K:]
    return q, ph, cs, a
def resid(x):
    q, ph, cs, a = unpack(x)
    c, s = np.cos(ph), np.sin(ph)
    out = []
    for k in range(K):
        d = PTS[k][:, :2] - cs[k]
        u = c*d[:, 0] + s*d[:, 1]; w = -s*d[:, 0] + c*d[:, 1]           # major-axis frame
        out.append(np.hypot(u, w / q) - a[k])
    return np.concatenate(out)
x0 = np.r_[0.94, np.radians(25.0), np.array([r["e"][:2] for r in R]).ravel(), np.full(K, 125.0)]
fit = least_squares(resid, x0, loss="soft_l1", f_scale=0.8, bounds=(np.r_[0.7, -np.pi, np.full(2*K, -1e4), np.full(K, 50)], np.r_[1.0, np.pi, np.full(2*K, 1e4), np.full(K, 200)]))
q, ph, cs, a = unpack(fit.x)
r = resid(fit.x); inl = np.abs(r) < 2.5
print(f"joint ring: axis ratio q {q:.4f} (tilt {np.degrees(np.arccos(q)):.1f} deg), major axis at {np.degrees(ph) % 180:.1f} deg, inliers {inl.mean():.2f}, inlier rms {np.sqrt(np.mean(r[inl]**2)):.2f} px")
# bootstrap over plateaus
rng = np.random.default_rng(1); bq, bp = [], []
allP = PTS
for b in range(40):
    idx = rng.integers(0, K, K)
    PTS = [allP[i] for i in idx]
    xb = np.r_[q, ph, cs[idx].ravel(), a[idx]]
    fb = least_squares(resid, xb, loss="soft_l1", f_scale=0.8, max_nfev=60)
    bq.append(fb.x[0]); bp.append(np.degrees(fb.x[1]) % 180)
PTS = allP
print(f"bootstrap over levels: q sd {np.std(bq):.4f}, major-axis angle sd {np.std(bp):.2f} deg")
json.dump(dict(q=q, phi_deg=float(np.degrees(ph) % 180), q_sd=float(np.std(bq)), phi_sd=float(np.std(bp)), centres=cs.tolist(), a=a.tolist()), open("ring_joint.json", "w"))
print("per-level scale a_k:", np.round(a, 2))
