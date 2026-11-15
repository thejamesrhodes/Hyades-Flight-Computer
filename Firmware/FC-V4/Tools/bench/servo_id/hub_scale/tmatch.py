"""Screw positions by rotated-template matching (sub-pixel), on full averages and on the odd/even halves."""
import numpy as np, cv2, json
A = np.load("plateau_avg.npy"); Hh = np.load("plateau_half.npy"); P = json.load(open("plateaus.json"))
CMD = np.array(P["CMD"], float); S0 = np.load("screws_full.npy")        # centroid estimates (start points)
RT = 22                                                                  # template half-size; head radius ~20 px
blur = lambda im: cv2.GaussianBlur(im, (0, 0), 1.0)
ref = blur(A[0])
yy, xx = np.mgrid[-RT:RT+1, -RT:RT+1]
mask = (np.hypot(xx, yy) <= 21).astype(np.float32)
def patch(img, c, ang):
    """Patch of img around c, rotated by -ang (deg, image plane) so it lines up with the reference orientation."""
    M = cv2.getRotationMatrix2D((float(c[0]), float(c[1])), ang, 1.0)
    M[:, 2] += [RT - c[0], RT - c[1]]
    return cv2.warpAffine(img, M, (2*RT+1, 2*RT+1), flags=cv2.INTER_CUBIC)
def match(img, tpl, c0, search=7):
    x0, y0 = int(round(c0[0])), int(round(c0[1])); hw = RT + search
    win = img[y0-hw:y0+hw+1, x0-hw:x0+hw+1].astype(np.float32)
    res = cv2.matchTemplate(win, tpl.astype(np.float32), cv2.TM_CCOEFF_NORMED, mask=mask)
    res = np.nan_to_num(res, nan=-1)
    _, q, _, (bx, by) = cv2.minMaxLoc(res)
    def sub(a, b, c):  # parabolic peak
        den = a - 2*b + c
        return 0.0 if abs(den) < 1e-9 else 0.5 * (a - c) / den
    dx = sub(res[by, bx-1], res[by, bx], res[by, bx+1]) if 0 < bx < res.shape[1]-1 else 0
    dy = sub(res[by-1, bx], res[by, bx], res[by+1, bx]) if 0 < by < res.shape[0]-1 else 0
    return np.array([x0 - search + bx + dx, y0 - search + by + dy]), q
def measure(img, k, it=3):
    pos = S0[k].copy()
    v0 = S0[0, 1] - S0[0, 0]
    for _ in range(it):
        v = pos[1] - pos[0]
        ang = np.degrees(np.arctan2(v[1], v[0]) - np.arctan2(v0[1], v0[0]))
        qs = []
        for s in range(2):
            tpl = patch(ref, S0[0, s], 0.0)
            # rotate the template to the current image-plane orientation
            M = cv2.getRotationMatrix2D((RT, RT), -ang, 1.0)
            tplr = cv2.warpAffine(tpl, M, (2*RT+1, 2*RT+1), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
            pos[s], q = match(blur(img), tplr, pos[s]); qs.append(q)
    return pos, qs
out = dict(full=[], half=[], q=[])
for k in range(len(CMD)):
    p, q = measure(A[k], k); out["full"].append(p.tolist()); out["q"].append(q)
    out["half"].append([measure(Hh[k, h], k)[0].tolist() for h in range(2)])
json.dump(out, open("screws_tm.json", "w"))
F = np.array(out["full"]); Hs = np.array(out["half"])
for k in range(len(CMD)):
    v = F[k, 1] - F[k, 0]; va = Hs[k, 0, 1] - Hs[k, 0, 0]; vb = Hs[k, 1, 1] - Hs[k, 1, 0]
    la = lambda u: np.degrees(np.arctan2(u[1], u[0]))
    print(f"{CMD[k]:+4.0f} sep {np.hypot(*v):6.2f} line {la(v):7.2f}  halves differ {la(va)-la(vb):+.3f} deg  ncc {out['q'][k][0]:.3f} {out['q'][k][1]:.3f}")
