"""BENCH-02 self-test: make a synthetic bench video with a known servo, for checking track.py + analyse.py.

    python synth.py OUT.mp4 stair [steps ...] [--skew-rows 0.0] [--ppm 80] [--seed 1]

Renders what the phone would see: a red laser dot moving along a slightly tilted line on a dark wall, the
board LEDs blinking the run code, a fixed red light (to be rejected) and sensor noise, at 240 fps with a
1 ms exposure. The servo is a simple model with known numbers (TRUTH below); the camera clock is off by
--ppm against the flight computer's, and starts at an arbitrary phase. The truth series is written to
OUT.truth.npz so the analysis can be compared with it.
"""
import argparse, subprocess, sys
import numpy as np
import profiles as P

TRUTH = dict(sod_deg=0.225, land_sd_deg=0.15, delay_ms=8.0, wn=130.0, zeta=0.8, rate_dps=600.0, lash_deg=0.135)
W, H, FPS = 960, 540, 240.0
FOCAL = 600.0          # px: dot x = X0 + FOCAL tan(angle - wall tilt) ...
X0, Y0, TILT = 500.0, 300.0, 0.02
LED_XY, JUNK_XY = (60.0, 300.0), (800.0, 120.0)


def servo(cmd_deg, seed):
    """True angle at 2 kHz for a 200 Hz command (deg). Time 0 = pulse start of RUN frame 0; includes PRE/POST at 0."""
    rng = np.random.default_rng(seed)
    T = TRUTH
    sub = 10                                                     # 2 kHz
    full = np.r_[np.zeros(P.PRE), cmd_deg, np.zeros(P.POST)]
    n = len(full) * sub
    dt = 1.0 / (P.FRAME_HZ * sub)
    nd = int(round(T["delay_ms"] * 1e-3 / dt))
    acc_cmd, tgt = 0.0, 0.0
    th = thd = out = 0.0
    hist = np.zeros(n)
    y = np.zeros(n)
    half = 0.5 * T["lash_deg"]
    wn, z, rate = T["wn"], T["zeta"], T["rate_dps"]
    for k in range(n):
        c = full[k // sub]
        if abs(c - acc_cmd) >= T["sod_deg"] - 1e-9:
            acc_cmd = c
            tgt = c + rng.uniform(-1, 1) * T["land_sd_deg"] * np.sqrt(3.0)
        hist[k] = tgt
        u = hist[k - nd] if k >= nd else 0.0
        thdd = wn * wn * (u - th) - 2 * z * wn * thd
        thd = max(min(thd + thdd * dt, rate), -rate)
        th += thd * dt
        if th - out > half:
            out = th - half
        elif out - th > half:
            out = th + half
        y[k] = out
    return np.arange(n) * dt - P.PRE / P.FRAME_HZ, y


def blob(img, x, y, sigma, rgb, gain=1.0):
    r = int(4 * sigma) + 2
    x0, y0 = int(round(x)), int(round(y))
    xs = np.arange(max(x0 - r, 0), min(x0 + r + 1, W)); ys = np.arange(max(y0 - r, 0), min(y0 + r + 1, H))
    if len(xs) == 0 or len(ys) == 0:
        return
    g = gain * np.exp(-((xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2) / (2 * sigma * sigma))
    for c in range(3):
        img[ys[0]:ys[-1] + 1, xs[0]:xs[-1] + 1, c] += rgb[c] * g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out"); ap.add_argument("profiles", nargs="+")
    ap.add_argument("--skew-rows", type=float, default=0.0, help="move the LEDs this fraction of the picture height off the dot's row")
    ap.add_argument("--ppm", type=float, default=80.0); ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--readout-ms", type=float, default=3.5, help="rolling-shutter readout time, top to bottom")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    # timeline in FC seconds: 2 s idle (normal 1 Hz blip), then each run, 2 s idle between
    segs, t_at = [], 2.0
    for name in a.profiles:
        prof = P.BY_NAME[name]
        cmd = np.array(P.command_deg(name))
        tt, yy = servo(cmd, a.seed + len(segs))
        start = t_at + P.PRE / P.FRAME_HZ                        # FC time of RUN frame 0
        segs.append(dict(name=name, start=start, n=prof["frames"], tt=tt + start, yy=yy))
        t_at = start + (prof["frames"] + P.POST) / P.FRAME_HZ + 2.0
    total = t_at
    nfr = int(total * FPS)
    led_y = Y0 + a.skew_rows * H
    enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(int(FPS)),
                            "-i", "-", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "16", "-pix_fmt", "yuv420p", a.out], stdin=subprocess.PIPE)
    scale, phase0 = 1.0 + a.ppm * 1e-6, 0.00137
    truth_t, truth_y, truth_run = [], [], []
    noise = [rng.normal(0, 2.0, (H, W, 3)).astype(np.float32) for _ in range(12)]     # a bank of noise frames
    for n in range(nfr):
        tc = n / FPS                                             # camera timestamp
        def fc_time(row):                                        # FC time at which this row is exposed (mid exposure)
            return (tc + a.readout_ms * 1e-3 * row / H + 0.0005) * scale + phase0
        img = np.full((H, W, 3), 14.0, np.float32)
        t_dot, t_led = fc_time(Y0), fc_time(led_y)
        ang, led_on, run = 0.0, (t_led % 1.0) < 0.1, -1          # idle: the READY blip
        for i, s in enumerate(segs):
            if s["start"] - P.PRE / P.FRAME_HZ <= t_led < s["start"] + (s["n"] + P.POST) / P.FRAME_HZ:
                fr = (t_led - s["start"]) * P.FRAME_HZ
                led_on = (0 <= fr < s["n"]) and (int(fr // P.LED_HALF) % 2 == 0)
            if s["tt"][0] <= t_dot <= s["tt"][-1]:
                ang, run = float(np.interp(t_dot, s["tt"], s["yy"])), i
        x = X0 + FOCAL * np.tan(np.radians(ang))
        y = Y0 + TILT * (x - X0)
        blob(img, x, y, 2.2, (900.0, 260.0, 240.0))              # laser: saturated core, red rim
        blob(img, x, y, 6.0, (120.0, 8.0, 8.0))
        blob(img, JUNK_XY[0], JUNK_XY[1], 2.5, (400.0, 30.0, 30.0))
        if led_on:
            blob(img, LED_XY[0], led_y, 3.0, (300.0, 500.0, 300.0)); blob(img, LED_XY[0] + 14, led_y, 3.0, (500.0, 300.0, 100.0))
        img += noise[rng.integers(0, len(noise))]
        enc.stdin.write(np.clip(img, 0, 255).astype(np.uint8).tobytes())
        truth_t.append(t_dot); truth_y.append(ang); truth_run.append(run)
    enc.stdin.close(); enc.wait()
    np.savez_compressed(a.out.rsplit(".", 1)[0] + ".truth.npz", t=np.array(truth_t), angle=np.array(truth_y), run=np.array(truth_run),
                        starts=np.array([s["start"] for s in segs]), names=np.array([s["name"] for s in segs]))
    print(f"{a.out}: {nfr} frames, {total:.1f} s, runs {', '.join(a.profiles)}")


if __name__ == "__main__":
    main()
