import numpy as np, cv2, subprocess, json
t = np.loadtxt("pts.txt"); Wb = np.load("Wbody.npy")
T0 = 4.07
cons = [5.425, 5.985, 6.606, 7.126, 7.725, 8.289, 10.436, 11.032, 11.645, 12.211, 12.778, 13.298, 13.911, 14.525, 15.170, 15.736, 16.301, 16.914]
CMD = [0, 10, 20, 30, 40, 50, 60, 50, 40, 30, 20, 10, 0, -10, -20, -30, -40, -50, -60]
edges = [0.0] + [c - T0 for c in cons] + [t[-1] + 0.01]
win = [(edges[k] + (0.15 if k else 0.05), edges[k + 1] - 0.03) for k in range(len(CMD))]
W, H, X0, Y0, N = 1080, 1920, 250, 600, 700
p = subprocess.Popen(["ffmpeg", "-v", "error", "-i", "/mnt/user-data/uploads/8145.mp4", "-vf", "format=gray", "-fps_mode", "passthrough", "-f", "rawvideo", "-"], stdout=subprocess.PIPE, bufsize=1 << 24)
acc = [np.zeros((N, N), np.float64) for _ in CMD]; cnt = [0] * len(CMD)
frames_in = [[] for _ in CMD]
i = 0
while True:
    b = p.stdout.read(W * H)
    if len(b) < W * H: break
    k = next((j for j, (a, e) in enumerate(win) if a <= t[i] < e), None)
    if k is not None:
        f = np.frombuffer(b, np.uint8).reshape(H, W)[Y0:Y0 + N, X0:X0 + N].astype(np.float32)
        M = Wb[i].astype(np.float64).copy(); M[:, 2] *= 2.0                 # half-res -> full-res
        st = cv2.warpAffine(f, M, (N, N), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        acc[k] += st; cnt[k] += 1; frames_in[k].append(i)
    i += 1
avg = np.array([a / max(c, 1) for a, c in zip(acc, cnt)], np.float32)
np.save("plateau_avg.npy", avg)
json.dump(dict(CMD=CMD, win=win, cnt=cnt, frames=frames_in), open("plateaus.json", "w"))
print(cnt)
cv2.imwrite("avg0.png", avg[0].astype(np.uint8)); cv2.imwrite("avg6.png", avg[6].astype(np.uint8))
