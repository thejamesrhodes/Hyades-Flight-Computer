"""BENCH-02 step 1: video -> laser-dot candidates and LED brightness, one row per camera frame.

    python track.py VIDEO [--out FILE.npz] [--budget 150]

One pass through the video at a reduced size (long side 640 px). For every frame it keeps
  - the eight strongest laser-like spots (sub-pixel centre, strength), so the laser dot can be told from any
    other light afterwards (analyse.py picks the one that lies on the dot's line), and
  - the brightness of the picture on a 16 px grid (mean and brightest pixel of each cell), from which
    analyse.py finds the blinking board LEDs.
Frame times are the file's own timestamps (ffprobe), not frame number / 240.

Long videos: the pass stops after --budget seconds of work and saves what it has; run the same
command again and it carries on from there. Needs ffmpeg, ffprobe, numpy and OpenCV.
"""
import argparse, json, os, subprocess, sys, time
import numpy as np
import cv2

LONG_SIDE, CELL, K = 640, 16, 8
WIN, SUPPRESS = 10, 16          # centroid window half-size and candidate separation, px at the reduced size
MIN_PEAK = 20.0                 # weakest score (0-510) kept as a candidate


def probe(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_streams", "-of", "json", path],
                       capture_output=True, text=True, check=True)
    st = json.loads(r.stdout)["streams"][0]
    rot = int(round(float(st.get("tags", {}).get("rotate", 0))))           # older ffprobe: a tag
    for sd in st.get("side_data_list", []):                                # newer: display-matrix side data
        rot = int(round(float(sd.get("rotation", rot))))
    w, h = int(st["width"]), int(st["height"])
    if abs(rot) % 180 == 90:
        w, h = h, w                                  # ffmpeg turns the picture upright before the filters
    num, den = st["r_frame_rate"].split("/")
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
                        "-of", "csv=p=0", path], capture_output=True, text=True, check=True)
    pts = np.sort(np.array([float(x) for x in r.stdout.split() if x not in ("", "N/A")]))
    k = LONG_SIDE / max(w, h)
    ow, oh = int(round(w * k / CELL)) * CELL, int(round(h * k / CELL)) * CELL
    return dict(w=w, h=h, ow=ow, oh=oh, fps=float(num) / float(den), pts=pts, rotation=rot)


def _passthrough():
    """Every decoded frame once, none dropped or repeated: the option changed name in ffmpeg 5.1."""
    v = subprocess.run(["ffmpeg", "-version"], capture_output=True, text=True).stdout.split()
    try:
        major, minor = (int(q) for q in v[2].lstrip("n").split("-")[0].split(".")[:2])
    except (IndexError, ValueError):
        major, minor = 6, 0
    return ["-fps_mode", "passthrough"] if (major, minor) >= (5, 1) else ["-vsync", "0"]


def frames(path, info, i0):
    """Yield reduced RGB frames from frame index i0 (seek to just before its timestamp)."""
    pts = info["pts"]
    cmd = ["ffmpeg", "-v", "error", "-nostdin"]
    if i0 > 0:
        cmd += ["-ss", f"{0.5 * (pts[i0 - 1] + pts[i0]):.6f}"]
    cmd += ["-i", path, "-vf", f"scale={info['ow']}:{info['oh']}:flags=area", "-pix_fmt", "rgb24",
            *_passthrough(), "-f", "rawvideo", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=1 << 22)
    n = info["ow"] * info["oh"] * 3
    try:
        while True:
            buf = p.stdout.read(n)
            if len(buf) < n:
                return
            yield np.frombuffer(buf, np.uint8).reshape(info["oh"], info["ow"], 3)
    finally:
        p.kill()


_KH, _KV = np.ones((1, 25), np.uint8), np.ones((25, 1), np.uint8)
_IDX = np.arange(2 * WIN + 1, dtype=np.float32)
_KC = np.ones((CELL, CELL), np.uint8)


def _compact(a):
    """Keep what is narrower than 25 px both across and down: a spot stays, an edge or a long highlight goes."""
    return np.minimum(cv2.morphologyEx(a, cv2.MORPH_TOPHAT, _KH), cv2.morphologyEx(a, cv2.MORPH_TOPHAT, _KV))


def spots(f):
    """Up to K laser-like spots: x, y (reduced px), peak score, summed weight.

    A laser dot shows as a red rim (red well above green and blue) and, where the sensor clips, a white core
    (all three channels high). The score is the sum of both, each kept only where it is a compact spot, so it
    works in a dim room (red blob) and in a lit one (white core on a bright wall)."""
    r, g, b = f[:, :, 0], f[:, :, 1], f[:, :, 2]
    ex = cv2.subtract(r, cv2.addWeighted(g, 0.5, b, 0.5, 0))          # red excess, clipped at 0
    sc = _compact(ex).astype(np.float32) + _compact(cv2.min(cv2.min(r, g), b)).astype(np.float32)
    sm = cv2.blur(sc, (3, 3))
    out = np.zeros((K, 4), np.float32)
    h, w = sc.shape
    for k in range(K):
        _, peak, _, (px, py) = cv2.minMaxLoc(sm)
        if peak < MIN_PEAK:
            break
        x0, x1, y0, y1 = max(px - WIN, 0), min(px + WIN + 1, w), max(py - WIN, 0), min(py + WIN + 1, h)
        wt = np.clip(sc[y0:y1, x0:x1] - 0.35 * peak, 0, None)
        m = float(wt.sum())
        if m > 0:
            out[k] = (x0 + float(wt.sum(axis=0) @ _IDX[:x1 - x0]) / m, y0 + float(wt.sum(axis=1) @ _IDX[:y1 - y0]) / m, peak, m)
        sm[max(py - SUPPRESS, 0):py + SUPPRESS + 1, max(px - SUPPRESS, 0):px + SUPPRESS + 1] = 0
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--out")
    ap.add_argument("--budget", type=float, default=150.0, help="seconds of work per call (the bridge shell stops at 180)")
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.video)[0] + ".track.npz"
    info = probe(a.video)
    n = len(info["pts"])
    gh, gw = info["oh"] // CELL, info["ow"] // CELL
    if os.path.exists(out):
        d = np.load(out)
        done, cand, cells, cmax = int(d["done"]), d["cand"].copy(), d["cells"].copy(), d["cmax"].copy()
        if len(cand) != n:
            sys.exit(f"{out} belongs to a different video: delete it")
    else:
        done, cand = 0, np.zeros((n, K, 4), np.float32)
        cells, cmax = np.zeros((n, gh, gw), np.uint8), np.zeros((n, gh, gw), np.uint8)
    if done >= n:
        print(f"{out}: already complete ({n} frames)")
        return
    t0, i = time.monotonic(), done
    for f in frames(a.video, info, done):
        if i >= n:
            break
        cand[i] = spots(f)
        v = cv2.max(cv2.max(f[:, :, 0], f[:, :, 1]), f[:, :, 2])
        cells[i] = cv2.resize(v, (gw, gh), interpolation=cv2.INTER_AREA)      # mean brightness of each 16 px cell
        cmax[i] = cv2.dilate(v, _KC)[CELL // 2::CELL, CELL // 2::CELL]            # and the brightest pixel in it (a small LED)
        i += 1
        if time.monotonic() - t0 > a.budget:
            break
    dt = time.monotonic() - t0
    complete = i >= n - 2                             # some files hold a packet or two that do not decode
    np.savez_compressed(out, done=(n if complete else i), cand=cand, cells=cells, cmax=cmax, pts=info["pts"], fps=info["fps"],
                        size=np.array([info["ow"], info["oh"], info["w"], info["h"]]), cell=CELL)
    print(f"{os.path.basename(a.video)}: {info['w']}x{info['h']} at {info['fps']:.2f} fps, {n} frames, "
          f"{info['pts'][-1] - info['pts'][0]:.1f} s; frames {done}..{i} in {dt:.0f} s ({(i - done) / max(dt, 1e-3):.0f} fps)")
    print(("complete: " if complete else "NOT FINISHED, run the same command again: ") + out)


if __name__ == "__main__":
    main()
