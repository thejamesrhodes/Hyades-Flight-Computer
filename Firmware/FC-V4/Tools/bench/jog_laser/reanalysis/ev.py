import numpy as np, scipy.signal as sg, scipy.io.wavfile as wf, scipy.ndimage as ni
A0 = {'7983':0.181104,'7984':0.053792}
def load(tag):
    d = np.load(f'trk_{tag}.npz'); o = d['out']; t = np.sort(np.loadtxt(f'pts_{tag}.txt'))
    x = o[:,0].copy(); y = o[:,1].copy(); bx = o[:,8]; by = o[:,9]
    # use B-diff centroid where core missing or core area abnormal (streaks)
    bad = ~np.isfinite(x)
    x[bad] = bx[bad]; y[bad] = by[bad]
    return t, x, y, o, d
def click_onsets(tag, lo=5000, hi=16000, thr=5.0, hold=0.045):
    fs, a = wf.read(f'a_{tag}.wav'); a = a.astype(float)/32768
    sos = sg.butter(4, [lo,hi], btype='band', fs=fs, output='sos')
    h = sg.sosfilt(sos, a)                       # causal to keep onsets sharp
    e2 = h*h
    # short energy (1 ms) vs noise floor (median over 0.4 s)
    k = int(0.001*fs); es = np.convolve(e2, np.ones(k)/k, 'same')
    dec = 12; ed = es[::dec]; fsd = fs/dec
    floor = ni.median_filter(ed[::20], size=int(0.5*fsd/20)|1); floor = np.repeat(floor, 20)[:len(ed)] + 1e-12
    r = ed/floor
    above = r > thr**2
    on = np.flatnonzero(above[1:] & ~above[:-1]) + 1
    keep = []; last = -1e9
    for i in on:
        ti = i/fsd
        if ti - last > hold:
            keep.append(i); 
        last = ti if (ti - last > hold) else last
    keep = np.array(keep)
    # peak ratio in the following 20 ms
    pk = np.array([np.sqrt(r[i:i+int(0.02*fsd)].max()) for i in keep])
    return keep/fsd + A0[tag], pk
def plateaus(t, x, win=5, thr=0.35, minlen=12):
    """segment into static plateaus: frames where |x[i+win]-x[i-win]| < thr"""
    n = len(x); xm = ni.median_filter(np.nan_to_num(x, nan=0), 3)
    d = np.abs(np.r_[xm[win:], [xm[-1]]*win] - np.r_[[xm[0]]*win, xm[:-win]])
    stat = (d < thr) & np.isfinite(x)
    segs = []; i = 0
    while i < n:
        if stat[i]:
            j = i
            while j < n and stat[j]: j += 1
            if j - i >= minlen: segs.append((i, j))
            i = j
        else: i += 1
    return segs
if __name__ == '__main__':
    for tag in ['7983','7984']:
        t, x, y, o, d = load(tag)
        on, pk = click_onsets(tag)
        print(tag, 'click onsets:', len(on))
        gaps = np.diff(on)
        print('   gap histogram (ms):', np.histogram(gaps*1e3, bins=[0,60,90,120,150,180,220,280,350,500,1000,1e5])[0])
        # noise on quiet windows
        for (a0,a1) in [(26,27),(5,7)] if tag=='7983' else [(11,13),(4,5)]:
            m = (t>a0)&(t<a1); xx = x[m]
            print('   static %g-%g s: x mean %.3f std %.4f, detrended std %.4f, y std %.4f' % (a0,a1,np.nanmean(xx), np.nanstd(xx), np.nanstd(sg.detrend(xx[np.isfinite(xx)])), np.nanstd(y[m])))
        np.save(f'on_{tag}.npy', np.c_[on, pk])
