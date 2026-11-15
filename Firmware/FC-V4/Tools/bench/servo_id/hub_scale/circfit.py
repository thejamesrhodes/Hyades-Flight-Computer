import numpy as np, cv2, json
A = np.load("plateau_avg.npy"); Hh = np.load("plateau_half.npy")
S = np.array(json.load(open("screws_tm.json"))["full"]); CMD = np.array(json.load(open("plateaus.json"))["CMD"], float)
th = np.radians(np.arange(0, 360, 4))
def circle(img, c0, rr=np.arange(15.0, 25.01, 0.5), span=7.0, step=0.5):
    g = cv2.GaussianBlur(img, (0, 0), 1.2).astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    best = (-1e9, None, None)
    for st, sp in ((step, span), (step / 4, step * 2)):
        cxs = np.arange(-sp, sp + 1e-9, st)
        base = c0 if best[1] is None else best[1]
        for dx in cxs:
            for dy in cxs:
                cx, cy = base[0] + dx, base[1] + dy
                for r in (rr if best[2] is None else np.arange(best[2] - 1, best[2] + 1.01, 0.25)):
                    xs = (cx + r * np.cos(th)).astype(np.float32); ys = (cy + r * np.sin(th)).astype(np.float32)
                    gxs = cv2.remap(gx, xs[None], ys[None], cv2.INTER_LINEAR)[0]; gys = cv2.remap(gy, xs[None], ys[None], cv2.INTER_LINEAR)[0]
                    radial = gxs * np.cos(th) + gys * np.sin(th)          # dark head inside, brighter outside
                    sc = np.mean(np.clip(radial, 0, None)) - 0.5 * np.std(radial)
                    if sc > best[0]: best = (sc, np.array([cx, cy]), r)
    return best[1], best[2]
out = []
for k in range(len(A)):
    row = []
    for s in range(2):
        p, r = circle(A[k], S[k, s]); row.append((p[0], p[1], r))
    out.append(row)
C = np.array(out); np.save("screws_circ.npy", C)
V = C[:, 1, :2] - C[:, 0, :2]; Vt = S[:, 1] - S[:, 0]
la = lambda v: np.degrees(np.unwrap(np.arctan2(v[:, 1], v[:, 0])))
a_c = la(V); a_c -= a_c[0]; a_t = la(Vt); a_t -= a_t[0]
for k in range(len(A)):
    print(f"{CMD[k]:+4.0f} circle-fit line {a_c[k]:+7.2f} (r {C[k,0,2]:.1f}, {C[k,1,2]:.1f})  template {a_t[k]:+7.2f}  diff {a_c[k]-a_t[k]:+5.2f}  centre shift s1 {np.hypot(*(C[k,0,:2]-S[k,0])):.1f} s2 {np.hypot(*(C[k,1,:2]-S[k,1])):.1f} px")
