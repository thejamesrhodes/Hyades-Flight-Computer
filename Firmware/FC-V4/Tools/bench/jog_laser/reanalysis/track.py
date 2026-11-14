import subprocess, numpy as np, sys
src, tag = sys.argv[1], sys.argv[2]
y0, y1, x0, x1 = [int(v) for v in sys.argv[3:7]]
refs = [tuple(int(v) for v in r.split(',')) for r in sys.argv[7:]]   # y0,y1,x0,x1 patches
W, H = 720, 1280
FS = W*H*3
def reader(extra=[]):
    cmd = ['ffmpeg','-v','error','-i',src]+extra+['-vsync','0','-f','rawvideo','-pix_fmt','rgb24','-']
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=FS*8)
# pass 1: background (median of sampled frames) for band
p = reader(['-vf','select=not(mod(n\\,40))'])
samp = []
while True:
    b = p.stdout.read(FS)
    if len(b) < FS: break
    f = np.frombuffer(b, np.uint8).reshape(H, W, 3)
    samp.append(f[y0:y1, x0:x1].copy())
samp = np.array(samp)
bg = np.median(samp, axis=0).astype(np.int16)
print('bg from', len(samp), 'frames')
# pass 2
p = reader()
out = []; kyB = []; kyC = []; refp = [[] for _ in refs]
n = 0
yy, xx = np.mgrid[0:(y1-y0), 0:(x1-x0)]
import cv2
while True:
    b = p.stdout.read(FS)
    if len(b) < FS: break
    f = np.frombuffer(b, np.uint8).reshape(H, W, 3)
    band = f[y0:y1, x0:x1].astype(np.int16)
    G = band[:,:,1]; B = band[:,:,2]; R = band[:,:,0]
    dB = B - bg[:,:,2]
    core = np.minimum(G, B)
    kyB.append(np.clip(dB.max(axis=0), 0, 255).astype(np.uint8))
    kyC.append(core.max(axis=0).astype(np.uint8))
    rec = [np.nan]*14
    # core detection
    cb = cv2.blur(core.astype(np.float32), (5,5))
    iy, ix = np.unravel_index(np.argmax(cb), cb.shape)
    pk = cb[iy, ix]
    if pk > 205:
        m = (core > 200).astype(np.uint8)
        nl, lab = cv2.connectedComponents(m)
        l = lab[iy, ix]
        if l == 0:
            # peak pixel itself below thr; take nearest
            l = lab[max(0,iy-2):iy+3, max(0,ix-2):ix+3].max()
        if l > 0:
            comp = lab == l
            ys, xs = np.nonzero(comp)
            bx0, bx1, by0, by1 = xs.min(), xs.max(), ys.min(), ys.max()
            a0, a1 = max(0, by0-4), min(band.shape[0], by1+5)
            c0, c1 = max(0, bx0-4), min(band.shape[1], bx1+5)
            w = np.clip(core[a0:a1, c0:c1] - 185, 0, 60).astype(np.float64)
            # restrict to dilated comp to avoid neighbours
            cm = cv2.dilate(comp[a0:a1, c0:c1].astype(np.uint8), np.ones((5,5),np.uint8)) > 0
            w = w * cm
            sw = w.sum()
            cxg = (w * xx[a0:a1, c0:c1]).sum()/sw + x0
            cyg = (w * yy[a0:a1, c0:c1]).sum()/sw + y0
            # sub-pixel extents along x: column sums of w normalised
            col = w.sum(axis=0)
            rec[0:7] = [cxg, cyg, comp.sum(), bx0 + x0, bx1 + x0, by0 + y0, by1 + y0]
            rec[7] = pk
    # B-diff detection (halo + streaks)
    mB = (dB > 35)
    if mB.sum() >= 12:
        mB8 = cv2.morphologyEx(mB.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        nl, lab, stats, cent = cv2.connectedComponentsWithStats(mB8)
        if nl > 1:
            k = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            comp = lab == k
            w = np.clip(dB, 0, 80).astype(np.float64) * comp
            sw = w.sum()
            rec[8] = (w*xx).sum()/sw + x0
            rec[9] = (w*yy).sum()/sw + y0
            rec[10] = stats[k, cv2.CC_STAT_AREA]
            rec[11] = stats[k, cv2.CC_STAT_LEFT] + x0
            rec[12] = stats[k, cv2.CC_STAT_LEFT] + stats[k, cv2.CC_STAT_WIDTH] - 1 + x0
            rec[13] = nl - 1
    out.append(rec)
    for i, (ry0, ry1, rx0, rx1) in enumerate(refs):
        if n % 4 == 0:
            refp[i].append(f[ry0:ry1, rx0:rx1, 1].copy())
    n += 1
    if n % 3000 == 0: print(n, flush=True)
np.savez_compressed(f'trk_{tag}.npz', out=np.array(out), roi=np.array([y0,y1,x0,x1]))   # kymographs and reference patches dropped: 40 MB
print('done', n)
