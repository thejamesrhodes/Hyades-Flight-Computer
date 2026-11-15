"""BENCH-02 step 2: track file -> runs, timing, angle, per-profile results and a picture.

    python analyse.py FILE.track.npz [...] [--cal cal.json] [--no-fig]

For each track file (track.py) this
  1. finds the blinking board LEDs and reads every run in the video from them: which profile (the count of
     LED half periods), and the camera time of every flight-computer frame (a straight-line fit to the LED
     edges, so the two clocks need not agree);
  2. picks the laser dot out of the red spots (the one on the dot's line, not a fixed light, not the LEDs);
  3. turns position along the line into angle. A 'stair' run fits the picture-to-angle map from its 88
     commanded levels and writes it to cal.json beside the video; later runs in the same folder use it
     (camera and rocket not moved), and check it against their own +/-10 deg header. Without cal.json the
     header alone is used (three levels: fine near zero, 1-2 % out at 30 deg);
  4. replays the command from profiles.py (same ticks as the firmware, checked by CRC there) and writes
     the per-profile numbers to FILE.runN.json, the aligned series to FILE.runN.npz and a figure.

Angles are true degrees at the canard. Times are flight-computer time from RUN frame 0, whose servo pulse
starts with the first LED edge. A servo reads a pulse at its end, 1.5 ms later: 'delay from pulse start'
includes that.
"""
import argparse, json, os, sys
import numpy as np
import profiles as P

HALF_S = P.LED_HALF / P.FRAME_HZ          # 0.485 s
TOL = 0.07                                # LED interval tolerance (clock ratio + one camera frame)
C_CMD, C_MEAS, C_ERR = "#0072B2", "#D55E00", "#000000"     # Okabe-Ito; command is also dashed


# ------------------------------------------------------------------------------------------------ LED
def _edges(v, t, thr):
    b = v > thr
    idx = np.nonzero(b[1:] != b[:-1])[0] + 1                  # frame idx is the first frame after the change
    return [(int(i), bool(b[i])) for i in idx]


def find_led(cells, t, cmax=None):
    """Grid cells that blink with the 0.485 s half period. Returns (series, cell list, score).

    Tried on the mean brightness of each cell and, if given, on its brightest pixel (a small LED in a lit
    room hardly moves the mean); the one with the longer clean run of edges is used."""
    if cmax is not None:
        a, b = find_led(cells, t), find_led(cmax, t)
        return b if (b[2], b[0] is not None) > (a[2], a[0] is not None) else a
    n, gh, gw = cells.shape
    lo, hi = np.percentile(cells, 5, axis=0), np.percentile(cells, 95, axis=0)
    score = np.zeros((gh, gw), int)
    for y, x in zip(*np.nonzero(hi - lo >= 8)):
        v = cells[:, y, x].astype(np.float32)
        e = _edges(v, t, 0.5 * (lo[y, x] + hi[y, x]))
        if len(e) < 6:
            continue
        dt = np.diff([t[i] for i, _ in e])
        ok = np.abs(dt / HALF_S - 1.0) < TOL
        run = best = 0
        for o in ok:
            run = run + 1 if o else 0
            best = max(best, run)
        score[y, x] = best
    if score.max() < 5:
        return None, [], 0
    best = score.max()
    blk = [(int(y), int(x)) for y, x in zip(*np.nonzero(score >= 0.8 * best))]
    y0, x0 = np.unravel_index(np.argmax(score * 1000 + (hi - lo)), score.shape)
    blk = [(y, x) for y, x in blk if abs(y - y0) <= 2 and abs(x - x0) <= 2]
    w = np.array([hi[y, x] - lo[y, x] for y, x in blk], np.float32)
    s = sum(wi * cells[:, y, x].astype(np.float32) for wi, (y, x) in zip(w, blk)) / w.sum()
    return s, blk, int(best)


def led_runs(s, t):
    """Runs in the LED series: a rising edge after >= 1.2 s dark, then edges every 0.485 s."""
    lo, hi = np.percentile(s, 5), np.percentile(s, 95)
    e = _edges(s, t, 0.5 * (lo + hi))
    runs, k = [], 0
    while k < len(e):
        i, up = e[k]
        dark = t[i] - (t[e[k - 1][0]] if k > 0 else t[0] - 10.0)
        if not up or dark < 1.2:
            k += 1
            continue
        seq, j = [k], k
        while j + 1 < len(e) and abs((t[e[j + 1][0]] - t[e[j][0]]) / HALF_S - 1.0) < TOL:
            j += 1
            seq.append(j)
        k = j + 1
        if len(seq) < 4:
            continue
        ei = np.array([e[q][0] for q in seq])
        halves = len(seq) - 1
        fi = P.LED_HALF * np.arange(len(seq))                                  # FC frame of each edge
        tmid = 0.5 * (t[ei - 1] + t[ei])
        # Straight line through the edges plus a rise/fall offset: a frame lit for part of its exposure
        # reads above the half-way grey level (camera gamma), so rising edges are seen early and falling
        # edges late by the same amount. Edges more than 3 sd off the fit are dropped (twice at most).
        rise = np.array([1.0 if e[q][1] else -1.0 for q in seq])
        keep = np.ones(len(fi), bool)
        for _ in range(3):
            A_ = np.c_[np.ones(len(fi)), fi, rise]
            coef = np.linalg.lstsq(A_[keep], tmid[keep], rcond=None)[0]
            res = tmid - A_ @ coef
            sd = max(float(np.std(res[keep])), 1e-5)
            new_keep = np.abs(res) < 3 * sd + 0.5 / 240
            if (new_keep == keep).all():
                break
            keep = new_keep
        alpha, beta, rf = (float(c) for c in coef)
        a_err = float(np.std(res[keep]) / np.sqrt(max(keep.sum() - 3, 1)))      # standard error of alpha
        complete = (not e[seq[-1]][1]) and (halves % 2 == 1)                   # ends on a falling edge
        runs.append(dict(halves=halves, alpha=float(alpha), beta=float(beta), alpha_err=float(a_err), complete=bool(complete),
                         rise_fall_bias_ms=float(-1e3 * rf), edges_used=int(keep.sum()), edge_residual_sd_ms=float(1e3 * np.std(res[keep])),
                         profile=P.BY_HALVES[halves]["name"] if (complete and halves in P.BY_HALVES) else None,
                         edge_frames=ei.tolist(), contrast=float(hi - lo)))
    return runs


# ------------------------------------------------------------------------------------------------ dot
def pick_dot(cand, led_cells, cell, line=None):
    """Position of the laser dot along its line, per frame (NaN where it is not found).

    Fixed red lights are dropped (a spot within 3 px of the same place in more than 99.5 % of frames; the dot
    leaves its rest position for the +/-10 deg header, so it never is), and anything inside the LED cells. The
    line is a robust fit through the spots that move, unless one is given."""
    n, K, _ = cand.shape
    x, y, pk, m = cand[:, :, 0], cand[:, :, 1], cand[:, :, 2], cand[:, :, 3]
    ok = pk > 0
    # how often is there a spot within 3 px of each place? (2 px bins, 3 x 3 box)
    bx, by = np.clip((x // 2).astype(int), 0, 511), np.clip((y // 2).astype(int), 0, 511)
    hist = np.zeros((512, 512), np.float32)
    np.add.at(hist, (by[ok], bx[ok]), 1.0)
    box = sum(np.roll(np.roll(hist, dy, 0), dx, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1))
    occ = box[by, bx] / n
    ok &= occ <= 0.995                                         # a fixed light: there in every frame
    for cy, cx in led_cells:
        ok &= ~((x >= (cx - 0.5) * cell) & (x < (cx + 1.5) * cell) & (y >= (cy - 0.5) * cell) & (y < (cy + 1.5) * cell))
    if line is None:
        # The dot's path is the longest straight run of places a spot has been: take the line through two
        # occupied places that passes the most occupied places (each counted once, so a big bright patch that
        # hardly moves - the laser module itself, an LED - cannot outvote it), then refine on the spots near it.
        occd = np.zeros((512, 512), bool)
        occd[by[ok], bx[ok]] = True
        qy, qx = np.nonzero(occd)
        if len(qx) < 12:
            return None
        pts = np.c_[qx * 2.0 + 1.0, qy * 2.0 + 1.0]
        rng = np.random.default_rng(0)
        best, best_n = None, -1
        for _ in range(600):
            i, j = rng.integers(0, len(pts), 2)
            v = pts[j] - pts[i]
            L = np.hypot(v[0], v[1])
            if L < 20:
                continue
            v = v / L
            nin = int(np.sum(np.abs((pts[:, 0] - pts[i, 0]) * (-v[1]) + (pts[:, 1] - pts[i, 1]) * v[0]) < 3.0))
            if nin > best_n:
                best, best_n = (pts[i].copy(), v), nin
        if best is None:
            return None
        c, d = best
        for _ in range(4):                                    # refine on the spots themselves
            perp = np.abs((x - c[0]) * (-d[1]) + (y - c[1]) * d[0])
            near = ok & (perp < 3.0)
            if near.sum() < 20:
                return None
            px, py = x[near], y[near]
            c = np.array([px.mean(), py.mean()])
            _, _, vt = np.linalg.svd(np.c_[px - c[0], py - c[1]], full_matrices=False)
            d = vt[0] if np.dot(vt[0], d) >= 0 else -vt[0]
        line = dict(cx=float(c[0]), cy=float(c[1]), dx=float(d[0]), dy=float(d[1]))
    c, d = np.array([line["cx"], line["cy"]]), np.array([line["dx"], line["dy"]])
    perp = np.abs((x - c[0]) * (-d[1]) + (y - c[1]) * d[0])
    along = (x - c[0]) * d[0] + (y - c[1]) * d[1]
    good = ok & (perp < 3.0)
    score = np.where(good, m, -1.0)
    k = np.argmax(score, axis=1)
    rows = np.arange(n)
    s = np.where(score[rows, k] > 0, along[rows, k], np.nan)
    return dict(s=s, line=line, row=float(c[1]), found=float(np.mean(np.isfinite(s))))


def fill(s, max_gap=6):
    """Interpolate over gaps of up to max_gap frames; longer gaps stay NaN."""
    s = s.copy()
    bad = ~np.isfinite(s)
    if bad.all():
        return s
    i = np.arange(len(s))
    s[bad] = np.interp(i[bad], i[~bad], s[~bad])
    edge = np.diff(np.r_[0, bad.astype(int), 0])
    for a, b in zip(np.nonzero(edge == 1)[0], np.nonzero(edge == -1)[0]):
        if b - a > max_gap:
            s[a:b] = np.nan
    return s


# ------------------------------------------------------------------------------------------------ angle
def holds(prof, min_frames=60):
    """(first frame, last frame + 1, commanded deg) of every steady level in a profile (no dither)."""
    out, f = [], 0
    for kind, flags, frames, a, b in prof["segs"]:
        if kind == P.MOVE and b <= 1 and frames >= min_frames and not (flags & P.F_DITHER):
            out.append((f, f + frames, a / P.TICKS_PER_DEG))
        f += frames
    return out


def level_means(prof, fc, s, frac=0.5, levels=None):
    """Median dot position over the settled part of each steady level."""
    res = []
    for f0, f1, deg in (levels or holds(prof)):
        m = (fc >= f0 + frac * (f1 - f0)) & (fc < f1 - 1) & np.isfinite(s)
        if m.sum() >= 5:
            res.append((deg, float(np.median(s[m])), f0, f1))
    return res


def fit_map(deg, s):
    """Projective map s = (p0 + p1 u) / (1 + p2 u), u = tan(angle): exact for a flat wall and a pinhole camera."""
    u = np.tan(np.radians(deg))
    s = np.asarray(s, float)
    A = np.c_[np.ones_like(u), u, -u * s]
    p = np.linalg.lstsq(A, s, rcond=None)[0]
    for _ in range(5):                                        # Gauss-Newton on the true residual
        den = 1 + p[2] * u
        r = s - (p[0] + p[1] * u) / den
        J = np.c_[1 / den, u / den, -(p[0] + p[1] * u) * u / den ** 2]
        p = p + np.linalg.lstsq(J, r, rcond=None)[0]
    return p


def to_deg(p, s):
    return np.degrees(np.arctan((s - p[0]) / (p[1] - p[2] * s)))


# ------------------------------------------------------------------------------------------------ results
def _interp_cross(t, y, level):
    """First time y crosses level (y starts on the other side), linear interpolation; NaN if never."""
    sgn = np.sign(level - y[0]) or 1.0
    k = np.nonzero((y - level) * sgn >= 0)[0]
    if len(k) == 0:
        return np.nan
    k = k[0]
    if k == 0:
        return float(t[0])
    return float(t[k - 1] + (level - y[k - 1]) * (t[k] - t[k - 1]) / (y[k] - y[k - 1]))


def r_stair(prof, tf, ang, noise):
    lv = level_means(prof, tf * P.FRAME_HZ, ang)
    deg = np.array([l[0] for l in lv]); meas = np.array([l[1] for l in lv])
    err = meas - deg
    fine = np.abs(deg) <= 12.01
    g, o = np.polyfit(deg[fine], meas[fine], 1)
    # direction of approach: previous level lower (up) or higher (down)
    prev = np.r_[0.0, deg[:-1]]
    up, dn = (deg > prev), (deg < prev)
    hyst = []
    for d in np.unique(deg[fine]):
        a, b = err[fine & up & (deg == d)], err[fine & dn & (deg == d)]
        if len(a) and len(b):
            hyst.append(float(np.mean(a) - np.mean(b)))
    return dict(levels=len(lv), gain_fine=float(g), offset_deg=float(o),
                level_error_sd_deg=float(np.std(err[fine] - (g - 1) * deg[fine] - o)),
                level_error_max_deg=float(np.max(np.abs(err))),
                up_minus_down_deg=float(np.mean(hyst)) if hyst else None, up_minus_down_sd_deg=float(np.std(hyst)) if hyst else None,
                table=[(float(d), float(m)) for d, m in zip(deg, meas)])


def r_steps(prof, tf, ang, noise):
    cmd = np.array([v for v, _ in P.play(prof["segs"])]) / P.TICKS_PER_DEG
    k = np.nonzero(np.diff(cmd) != 0)[0] + 1                 # frame of each step
    k = k[k >= 360]                                           # after the header
    rows = []
    for i, f in enumerate(k):
        f_end = k[i + 1] if i + 1 < len(k) else len(cmd)
        size = cmd[f] - cmd[f - 1]
        t0 = f / P.FRAME_HZ
        w = (tf >= t0 - 0.06) & (tf < f_end / P.FRAME_HZ - 0.005) & np.isfinite(ang)
        t, y = tf[w] - t0, ang[w]
        if len(t) < 20:
            continue
        base = np.median(y[t < 0]); final = np.median(y[t > 0.6 * t[-1]])
        move = final - base
        row = dict(t=float(t0), size=float(size), moved=float(move))
        if abs(move) > max(4 * noise, 0.04):
            yy = (y - base) / move
            tt, yy = t[t >= -0.004], yy[t >= -0.004]
            t10, t50, t90 = (_interp_cross(tt, yy, q) for q in (0.1, 0.5, 0.9))
            rate = np.max(np.abs(np.gradient(y, t)[(t > 0) & (t < t90 + 0.01)])) if np.isfinite(t90) else np.nan
            row.update(t10_ms=1e3 * t10, t50_ms=1e3 * t50, t90_ms=1e3 * t90, overshoot=float(np.max(yy) - 1.0), peak_rate_dps=float(rate))
        rows.append(row)
    by = {}
    for r in rows:
        by.setdefault(round(abs(r["size"]), 2), []).append(r)
    summ = []
    for sz in sorted(by):
        g = by[sz]; mv = [r for r in g if "t10_ms" in r]
        e = dict(size_deg=sz, n=len(g), responded=len(mv), moved_mean_deg=float(np.mean([abs(r["moved"]) for r in g])),
                 moved_sd_deg=float(np.std([abs(r["moved"]) for r in g])))
        if mv:
            for key in ("t10_ms", "t50_ms", "t90_ms", "overshoot", "peak_rate_dps"):
                v = np.array([r[key] for r in mv]); v = v[np.isfinite(v)]
                if len(v):
                    e[key] = float(np.median(v)); e[key + "_sd"] = float(np.std(v))
        summ.append(e)
    return dict(by_size=summ, steps=rows)


def r_sine(prof, tf, ang, noise):
    out, f = [], 0
    for kind, flags, frames, a, b in prof["segs"]:
        if kind == P.SINE:
            cyc = frames // b
            skip = 2 if cyc >= 6 else 1
            t0, t1 = (f + skip * b) / P.FRAME_HZ, (f + frames) / P.FRAME_HZ
            w = (tf >= t0) & (tf < t1) & np.isfinite(ang)
            if w.sum() > 20:
                om = 2 * np.pi * P.FRAME_HZ / b
                ph = om * (tf[w] - f / P.FRAME_HZ)
                A = np.c_[np.sin(ph), np.cos(ph), np.ones(w.sum())]
                c = np.linalg.lstsq(A, ang[w], rcond=None)[0]
                res = ang[w] - A @ c
                amp = float(np.hypot(c[0], c[1]))
                phase = float(np.degrees(np.arctan2(c[1], c[0])))
                out.append(dict(hz=P.FRAME_HZ / b, amp_cmd_deg=a / P.TICKS_PER_DEG, amp_deg=amp, gain=amp / (a / P.TICKS_PER_DEG),
                                phase_deg=phase, delay_ms=float(-phase / 360.0 * 1000.0 * b / P.FRAME_HZ),
                                residual_deg=float(np.std(res)), cycles=cyc - skip))
        f += frames
    if out:                                                   # phase falls with frequency: undo the 360 deg wraps
        ph = np.degrees(np.unwrap(np.radians([q["phase_deg"] for q in out])))
        for q, v in zip(out, ph):
            q["phase_deg"] = float(v)
            q["delay_ms"] = float(-v / 360.0 * 1000.0 / q["hz"])
    return dict(points=out, note="phase and gain are against the ideal sine; the 200 Hz hold adds 2.5 ms and the pulse read 1.5 ms")


def _ripple50(tf, ang, w):
    ph = 2 * np.pi * 50.0 * tf[w]
    A = np.c_[np.sin(ph), np.cos(ph), np.ones(w.sum()), tf[w] - tf[w].mean()]
    c = np.linalg.lstsq(A, ang[w], rcond=None)[0]
    return float(np.hypot(c[0], c[1]))


def r_track(prof, tf, ang, noise):
    """creep / flight: how the angle follows a slow command; jumps (no dither) or ripple (dither)."""
    seq = P.play(prof["segs"])
    cmd = np.array([v for v, _ in seq]) / P.TICKS_PER_DEG
    dith = any(d for _, d in seq)
    f0 = 360 + 20
    w = (tf >= f0 / P.FRAME_HZ) & (tf < len(cmd) / P.FRAME_HZ) & np.isfinite(ang)
    c = np.interp(tf[w] * P.FRAME_HZ, np.arange(len(cmd)) + 0.5, cmd)
    e = ang[w] - c
    lags = np.arange(0, 0.0605, 0.0005)                       # lag that minimises the error
    rms = [np.sqrt(np.mean((ang[w] - np.interp((tf[w] - L) * P.FRAME_HZ, np.arange(len(cmd)) + 0.5, cmd)) ** 2)) for L in lags]
    res = dict(dither=bool(dith), error_rms_deg=float(np.sqrt(np.mean(e ** 2))), error_mean_deg=float(np.mean(e)),
               error_max_deg=float(np.max(np.abs(e))), best_lag_ms=float(1e3 * lags[int(np.argmin(rms))]), error_rms_at_best_lag_deg=float(np.min(rms)),
               ripple_50hz_deg=_ripple50(tf, ang, w))
    if not dith:
        y = ang[w]; t = tf[w]
        ys = np.convolve(y, np.ones(5) / 5, mode="same")
        d = ys[8:] - ys[:-8]                                  # change over 8 camera frames (33 ms)
        thr = max(5 * noise, 0.06)
        big = np.abs(d) > thr
        jumps, i = [], 0
        while i < len(d):
            if big[i]:
                j = i
                while j < len(d) and big[j]:
                    j += 1
                mid = (i + j) // 2 + 4
                a0, a1 = np.median(y[max(i - 12, 0):i + 1]), np.median(y[min(j + 8, len(y) - 1):min(j + 24, len(y))])
                jumps.append((float(t[mid]), float(a1 - a0), float(a1 - np.interp(t[mid] * P.FRAME_HZ, np.arange(len(cmd)) + 0.5, cmd))))
                i = j + 8
            else:
                i += 1
        if jumps:
            tj = np.array([q[0] for q in jumps])
            cj = np.interp(tj * P.FRAME_HZ, np.arange(len(cmd)) + 0.5, cmd)
            dc = np.abs(np.diff(cj))                          # command travelled between consecutive jumps
            same = np.sign(np.diff(cj)) == np.sign([q[1] for q in jumps[1:]])
            sz = np.abs([q[1] for q in jumps])
            res.update(jumps=len(jumps), jump_size_median_deg=float(np.median(sz)), jump_size_iqr_deg=[float(np.percentile(sz, 25)), float(np.percentile(sz, 75))],
                       command_between_jumps_median_deg=float(np.median(dc[same])) if same.any() else None,
                       landing_minus_command_sd_deg=float(np.std([q[2] for q in jumps])))
        else:
            res.update(jumps=0)
    return res


RESULTS = dict(stair=r_stair, steps=r_steps, sine03=r_sine, sine1=r_sine, sine1d=r_sine, sine3=r_sine,
               creep=r_track, creepd=r_track, flight=r_track, flightn=r_track)


# ------------------------------------------------------------------------------------------------ driver
def figure(path, title, tf, ang, cmd_t, cmd, led_t, led):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), gridspec_kw=dict(height_ratios=[3, 2, 1]))
    ax[0].step(cmd_t, cmd, where="post", color=C_CMD, ls="--", lw=1.0, label="command")
    ax[0].plot(tf, ang, color=C_MEAS, lw=0.9, label="measured (laser)")
    ax[0].set_ylabel("angle, deg"); ax[0].legend(loc="upper right", ncol=2, frameon=False); ax[0].set_title(title, fontsize=10)
    c = np.interp(tf * P.FRAME_HZ, np.arange(len(cmd)) + 0.5, cmd)
    ax[1].plot(tf, ang - c, color=C_ERR, lw=0.6)
    lim = np.nanpercentile(np.abs(ang - c), 99) if np.isfinite(ang).any() else 1.0
    ax[1].set_ylim(-1.3 * lim - 0.05, 1.3 * lim + 0.05); ax[1].set_ylabel("measured - command, deg")
    ax[2].plot(led_t, led, color="#555555", lw=0.7); ax[2].set_ylabel("LED"); ax[2].set_xlabel("flight-computer time from RUN frame 0, s")
    for a_ in ax:
        a_.set_xlim(-1.6, cmd_t[-1] + 1.6); a_.grid(alpha=0.25, lw=0.5)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def analyse(npz, cal_path=None, make_fig=True):
    d = np.load(npz)
    if int(d["done"]) < len(d["pts"]) - 2:
        print(f"{npz}: tracking not finished ({int(d['done'])} of {len(d['pts'])} frames): run track.py again")
        return []
    t, cand, cells, cell = d["pts"], d["cand"], d["cells"], int(d["cell"])
    stem = npz[:-len(".track.npz")] if npz.endswith(".track.npz") else os.path.splitext(npz)[0]
    name = os.path.basename(stem)
    fps_file = (len(t) - 1) / (t[-1] - t[0])
    print(f"== {name}: {len(t)} frames, {t[-1] - t[0]:.1f} s, {fps_file:.2f} fps by the file's timestamps")
    led, blk, score = find_led(cells, t, d["cmax"] if "cmax" in d else None)
    if led is None:
        print("   NO LED FOUND: nothing in the picture blinks at 0.485 s. Are the board LEDs in frame and was a profile running?")
        return []
    runs = led_runs(led, t)
    ly = np.mean([y for y, _ in blk]) * cell + cell / 2
    print(f"   LEDs at grid cells {blk} (row {ly:.0f} of {cells.shape[1] * cell} px), contrast {np.percentile(led, 95) - np.percentile(led, 5):.0f}/255, {len(runs)} run(s)")
    cal_path = cal_path or os.path.join(os.path.dirname(os.path.abspath(npz)), "cal.json")
    out = []
    for ri, r in enumerate(runs):
        tag = f"{stem}.run{ri + 1}"
        if r["profile"] is None:
            print(f"   run {ri + 1}: {r['halves']} LED half periods, {'cut off by the end of the video' if not r['complete'] else 'not a known profile'}: skipped")
            continue
        prof = P.BY_NAME[r["profile"]]
        fc = (t - r["alpha"]) / r["beta"]                    # FC frame number of every camera frame
        tf = fc / P.FRAME_HZ
        sel = (fc >= -P.PRE) & (fc < prof["frames"] + P.POST)
        rate = 1.0 / (r["beta"] * P.FRAME_HZ)                # camera seconds per FC second, inverted
        res = dict(video=name, run=ri + 1, profile=prof["name"], crc=f"{prof['crc']:08X}", frames=prof["frames"],
                   clock_ratio_fc_over_camera=rate, sync_uncertainty_ms=1e3 * r["alpha_err"], led_edges=len(r["edge_frames"]),
                   camera_fps=float(fps_file))
        print(f"   run {ri + 1}: {prof['name']} ({r['halves']} half periods), sync +/-{1e3 * r['alpha_err']:.2f} ms "
              f"({r['edges_used']} of {len(r['edge_frames'])} edges, sd {r['edge_residual_sd_ms']:.2f} ms, rise/fall bias {r['rise_fall_bias_ms']:.2f} ms), "
              f"FC clock / camera clock = {rate:.6f}")
        if prof["name"] == "sync":
            res["note"] = "LED check only"
            json.dump(res, open(tag + ".json", "w"), indent=1)
            out.append(res)
            continue
        cal = json.load(open(cal_path)) if os.path.exists(cal_path) and prof["name"] != "stair" else None
        dot = pick_dot(cand[sel], blk, cell, cal["line"] if cal else None)
        if dot is None or dot["found"] < 0.5:
            print(f"   run {ri + 1}: laser dot found in only {0 if dot is None else 100 * dot['found']:.0f} % of frames: skipped")
            continue
        s = fill(dot["s"])
        fcr, tfr = fc[sel], tf[sel]
        hdr = level_means(prof, fcr, s, levels=[(0, 100, 0.0), (100, 180, 10.0), (180, 260, -10.0), (260, 360, 0.0)])
        if len(hdr) < 4:
            print(f"   run {ri + 1}: the +/-10 deg header was not seen: skipped")
            continue
        if cal:                                               # stair calibration of this session
            p = np.array(cal["p"])
            if (to_deg(p, hdr[1][1]) - to_deg(p, hdr[2][1])) < 0:
                print("   the dot moves the other way to cal.json: camera or rocket moved, record 'stair' again"); continue
            res["calibration"] = "cal.json (stair run)"
        else:
            if prof["name"] == "stair":
                lv = level_means(prof, fcr, s)
                p = fit_map([l[0] for l in lv], [l[1] for l in lv])
                json.dump(dict(p=p.tolist(), line=dot["line"], from_video=name, levels=len(lv)), open(cal_path, "w"), indent=1)
                res["calibration"] = "this run's 88 levels (written to cal.json)"
            else:
                p = fit_map([h[0] for h in hdr[:3]], [h[1] for h in hdr[:3]])
                res["calibration"] = "this run's header only (no cal.json): 1-2 % at large angles"
        ang = to_deg(p, s)
        hres = [float(to_deg(p, h[1]) - h[0]) for h in hdr]
        pre = (fcr >= -200) & (fcr < -20) & np.isfinite(ang)
        noise = float(np.std(ang[pre] - np.median(ang[pre]))) if pre.sum() > 20 else float("nan")
        px_per_deg = float(abs(p[1]) * np.pi / 180)
        # The sensor reads out along the rows of the STORED picture. A video stored landscape and shown
        # portrait (rotation tag 90/270) reads out across the displayed x axis instead.
        rot = int(d["rotation"]) if "rotation" in d else 0
        if abs(rot) % 180 == 90:
            lx = np.mean([x for _, x in blk]) * cell + cell / 2
            sep = abs(dot["line"]["cx"] - lx) / (cells.shape[2] * cell)
        else:
            sep = abs(dot["row"] - ly) / (cells.shape[1] * cell)
        res.update(dot_found=dot["found"], header_residual_deg=hres, noise_sd_deg=noise, px_per_deg_at_zero=px_per_deg,
                   rolling_shutter_rows=float(sep), rotation=rot)
        print(f"   dot found in {100 * dot['found']:.1f} % of frames, noise {noise:.4f} deg rms, {px_per_deg:.1f} px/deg (640 px picture), "
              f"header residuals {', '.join(f'{h:+.2f}' for h in hres)} deg ({res['calibration']})")
        if cal and max(abs(h) for h in hres) > 0.5:
            print("   WARNING: header levels are more than 0.5 deg from cal.json: camera or rocket moved since the stair run")
        if res["rolling_shutter_rows"] > 0.1:
            print(f"   note: LEDs and dot are {100 * res['rolling_shutter_rows']:.0f} % of the picture apart along the sensor readout: "
                  f"timing may be out by up to {res['rolling_shutter_rows'] * 1000 / fps_file:.1f} ms (rolling shutter)")
        res["result"] = RESULTS[prof["name"]](prof, tfr, ang, noise)
        cmd = np.array(P.command_deg(prof["name"]))
        np.savez_compressed(tag + ".npz", t=tfr, angle=ang, cmd=cmd, s=s, p=p)
        json.dump(res, open(tag + ".json", "w"), indent=1)
        if make_fig:
            figure(tag + ".png", f"{name} run {ri + 1}: {prof['name']} - {prof['what']}", tfr, ang, np.arange(len(cmd)) / P.FRAME_HZ, cmd, tfr, led[sel])
        out.append(res)
    return out


def brief(res):
    r, k = res.get("result"), res["profile"]
    if r is None:
        return
    if k == "stair":
        print(f"   stair: gain {r['gain_fine']:.4f} over +/-12 deg, level scatter {r['level_error_sd_deg']:.3f} deg sd, up minus down {r['up_minus_down_deg']:+.3f} deg, worst level {r['level_error_max_deg']:.2f} deg")
    elif k == "steps":
        print("   steps: size  responded  moved   t10    t90   overshoot  peak rate")
        for e in r["by_size"]:
            if "t10_ms" in e:
                print(f"         {e['size_deg']:5.1f}  {e['responded']}/{e['n']}      {e['moved_mean_deg']:6.2f}  {e['t10_ms']:5.1f}  {e['t90_ms']:5.1f} ms  {100 * e['overshoot']:5.1f} %  {e['peak_rate_dps']:6.0f} deg/s")
            else:
                print(f"         {e['size_deg']:5.1f}  {e['responded']}/{e['n']}      {e['moved_mean_deg']:6.2f}")
    elif k.startswith("sine"):
        print("   sines:   Hz   gain   phase   delay")
        for q in r["points"]:
            print(f"          {q['hz']:5.1f}  {q['gain']:5.2f}  {q['phase_deg']:6.1f}  {q['delay_ms']:5.1f} ms")
    else:
        msg = f"   {k}: error {r['error_rms_deg']:.3f} deg rms (mean {r['error_mean_deg']:+.3f}), best lag {r['best_lag_ms']:.1f} ms, 50 Hz ripple {r['ripple_50hz_deg']:.3f} deg"
        if r.get("jumps"):
            msg += f"; {r['jumps']} jumps, median {r['jump_size_median_deg']:.3f} deg, command travelled between jumps {r['command_between_jumps_median_deg']:.3f} deg"
        print(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tracks", nargs="+")
    ap.add_argument("--cal")
    ap.add_argument("--no-fig", action="store_true")
    ap.add_argument("--range-deg", type=int, default=P.RANGE_NOW, choices=sorted(P.RANGES),
                    help="servo scale the recording was played at (SERVO_RANGE_DEG): 135 for the 14 Nov 2026 recordings")
    a = ap.parse_args()
    P.use_range(a.range_deg)
    # a stair run first, so that its cal.json serves the others
    def is_stair(f):
        d = np.load(f); led, _, _ = find_led(d["cells"], d["pts"], d["cmax"] if "cmax" in d else None)
        return led is not None and any(r["profile"] == "stair" for r in led_runs(led, d["pts"]))
    order = sorted(a.tracks, key=lambda f: (not is_stair(f), f)) if len(a.tracks) > 1 else a.tracks
    for f in order:
        for res in analyse(f, a.cal, not a.no_fig):
            brief(res)


if __name__ == "__main__":
    main()
