import numpy as np, cv2, subprocess
F = np.load("roi_small.npy").astype(np.float32)
t = np.loadtxt("pts.txt")
ref = cv2.GaussianBlur(F[0], (0, 0), 1.0)
h, w = ref.shape
yy, xx = np.mgrid[0:h, 0:w]
r = np.hypot(xx - 137.5, yy - 200.0)
body = ((r > 75) & (xx > 20) & ~((xx < 120) & (yy > 150) & (yy < 260))).astype(np.uint8)
crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-7)
W = np.eye(2, 3, dtype=np.float32); Ws, cc = [], []
for i in range(len(F)):
    img = cv2.GaussianBlur(F[i], (0, 0), 1.0)
    c, W = cv2.findTransformECC(ref, img, W, cv2.MOTION_EUCLIDEAN, crit, body, 5)
    Ws.append(W.copy()); cc.append(c)
np.save("Wbody.npy", np.array(Ws)); np.save("ccbody.npy", np.array(cc))
print(len(Ws), min(cc))
