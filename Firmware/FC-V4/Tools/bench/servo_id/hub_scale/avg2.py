import numpy as np, cv2, subprocess, json
t = np.loadtxt("pts.txt"); Wb = np.load("Wbody.npy"); P = json.load(open("plateaus.json"))
W, H, X0, Y0, N = 1080, 1920, 250, 600, 700
p = subprocess.Popen(["ffmpeg", "-v", "error", "-i", "/mnt/user-data/uploads/8145.mp4", "-vf", "format=gray", "-fps_mode", "passthrough", "-f", "rawvideo", "-"], stdout=subprocess.PIPE, bufsize=1 << 24, stderr=subprocess.DEVNULL)
K = len(P["CMD"]); acc = np.zeros((K, 2, N, N)); cnt = np.zeros((K, 2))
where = {}
for k, fr in enumerate(P["frames"]):
    for n, i in enumerate(fr): where[i] = (k, n % 2)
i = 0
while True:
    b = p.stdout.read(W * H)
    if len(b) < W * H: break
    if i in where:
        k, half = where[i]
        f = np.frombuffer(b, np.uint8).reshape(H, W)[Y0:Y0 + N, X0:X0 + N].astype(np.float32)
        M = Wb[i].astype(np.float64).copy(); M[:, 2] *= 2.0
        acc[k, half] += cv2.warpAffine(f, M, (N, N), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP); cnt[k, half] += 1
    i += 1
np.save("plateau_half.npy", (acc / cnt[:, :, None, None]).astype(np.float32))
print(cnt.T)
