import numpy as np, cv2
F = np.load("roi_small.npy").astype(np.float32)
t = np.loadtxt("pts.txt")
ref = cv2.GaussianBlur(F[0], (0, 0), 1.0)
h, w = ref.shape
yy, xx = np.mgrid[0:h, 0:w]
cx, cy = 137.5, 200.0
r = np.hypot(xx - cx, yy - cy)
body = ((r > 75) & (xx > 20) & ~((xx < 120) & (yy > 150) & (yy < 260))).astype(np.uint8)
crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-7)
# screw templates from the reference (blurred)
S0 = [np.array([145.0, 165.0]), np.array([130.0, 227.5])]
def refine(img, p, R=9):
    """Sub-pixel centre of the dark disc near p: NCC with a dark-disc template, then a dark-weighted centroid."""
    x0, y0 = int(round(p[0])), int(round(p[1]))
    win = img[y0-22:y0+23, x0-22:x0+23]
    tpl = np.full((2*R+5, 2*R+5), 1.0, np.float32); cv2.circle(tpl, (R+2, R+2), R, 0.0, -1)
    res = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
    _, mx, _, (bx, by) = cv2.minMaxLoc(res)
    px, py = x0 - 22 + bx + R + 2, y0 - 22 + by + R + 2
    for _ in range(3):
        sub = img[int(py)-R-2:int(py)+R+3, int(px)-R-2:int(px)+R+3]
        sy, sx = np.mgrid[int(py)-R-2:int(py)+R+3, int(px)-R-2:int(px)+R+3]
        bg = np.percentile(sub, 90)
        wgt = np.clip(bg - sub, 0, None) * (np.hypot(sx - px, sy - py) <= R + 1)
        px, py = (wgt * sx).sum() / wgt.sum(), (wgt * sy).sum() / wgt.sum()
    return np.array([px, py]), mx
W = np.eye(2, 3, dtype=np.float32)
rows, pos = [], [p.copy() for p in S0]
for i in range(0, len(F), 2):
    img = cv2.GaussianBlur(F[i], (0, 0), 1.0)
    try:
        _, W = cv2.findTransformECC(ref, img, W, cv2.MOTION_EUCLIDEAN, crit, body, 5)
    except cv2.error:
        rows.append([t[i]] + [np.nan]*7); continue
    st = cv2.warpAffine(F[i], W, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)   # into reference coordinates
    st = cv2.GaussianBlur(st, (0, 0), 1.0)
    out = [t[i]]
    for k in range(2):
        p, q = refine(st, pos[k]); pos[k] = p; out += [p[0], p[1], q]
    out += [np.degrees(np.arctan2(W[1, 0], W[0, 0]))]
    rows.append(out)
np.save("screws_s2.npy", np.array(rows, dtype=float))
print(len(rows))
