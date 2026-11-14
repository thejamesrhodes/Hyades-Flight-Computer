import numpy as np, pickle, scipy.io.wavfile as wf, scipy.signal as sg
from ev import load
A0 = {'7983':0.181104,'7984':0.053792}
res = {}
for tag, pxdeg in [('7983',16.8),('7984',15.7)]:
    fs, a = wf.read(f'a_{tag}.wav'); a = a.astype(float)/32768
    t, x, y, o, d = load(tag)
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); tc = D['tc'].copy()
    if tag == '7983': tc[3] = 7.906
    b = np.load(f'burst_{tag}.npy')[:,0]
    xs = D['xs']; xprev = np.r_[D['x0'], xs[:-1]]; dx = np.abs(xs - xprev)
    rel = np.full(len(tc), np.nan)
    for k in range(len(tc)):
        nxt = b[(b > tc[k] + 0.06) & (b < tc[k] + 0.27)]
        if len(nxt): rel[k] = nxt[0]
    sos = sg.butter(4, [3000, 9000], btype='band', fs=fs, output='sos')
    h = sg.sosfiltfilt(sos, a); e2 = h*h
    w = int(0.002*fs)                                   # 2 ms bins
    c = np.cumsum(np.r_[0, e2])
    tt = np.arange(-0.02, 0.12, 0.002)
    def seg(t0):
        i0 = int(round((t0 - A0[tag] - 0.02)*fs)); idx = np.clip(i0 + np.arange(len(tt)+1)*w, 0, len(c)-1)
        return (c[idx[1:]] - c[idx[:-1]])/w
    ten = [k for k in range(len(tc)) if abs(D['step'][k]) >= 10 and np.isfinite(rel[k])]
    if tag == '7984': ten = [k for k in ten if k in (0,1,2,6,7)]      # free moves only
    nomv = [k for k in range(len(tc)) if abs(D['step'][k]) < 0.5 and dx[k] < 0.3 and np.isfinite(rel[k])]
    E0 = np.array([seg(rel[k]) for k in nomv])
    # normalise each click by its energy in first 6 ms after click onset
    i_on = np.searchsorted(tt, 0.0)
    def norm(E): return E / E[:, i_on:i_on+3].mean(axis=1, keepdims=True)
    tmpl = np.median(norm(E0), axis=0)
    lo, hi = np.percentile(norm(E0), [10, 90], axis=0)
    E1 = norm(np.array([seg(rel[k]) for k in ten]))
    print('=====', tag, 'n ten', len(ten), 'n no-move', len(nomv))
    print(' t ms      ', ' '.join('%4d' % round(v*1e3) for v in tt[8:45]))
    print(' tmpl dB   ', ' '.join('%4.0f' % v for v in 10*np.log10(tmpl[8:45])))
    print(' p90  dB   ', ' '.join('%4.0f' % v for v in 10*np.log10(hi[8:45])))
    on = []
    for k, e in zip(ten, E1):
        ex = 10*np.log10(e/ hi)                           # excess over the 90th percentile click-only decay
        print(' jog %2d ex ' % k, ' '.join('%4.0f' % v for v in ex[8:45]))
        # onset: first bin >= 0 ms where excess > 3 dB for 3 consecutive bins
        j = i_on
        while j < len(ex)-3 and not (ex[j] > 3 and ex[j+1] > 3 and ex[j+2] > 3): j += 1
        # video motion start relative to release (video time, container offset applied)
        i = np.searchsorted(t, tc[k] - 0.015); pre = np.nanmedian(x[i-10:i]); jj = i
        while abs(x[jj] - pre) < 0.3: jj += 1
        # sub-frame start by linear extrapolation of the first two moving frames back to pre-level
        v = (x[jj+1] - x[jj])/(t[jj+1] - t[jj]); tstart = t[jj] - (x[jj] - pre)/v if abs(v) > 0 else t[jj]
        on.append((tt[j]*1e3, (tstart - rel[k])*1e3))
    on = np.array(on)
    print(' sound-excess onset after release click (ms):', np.round(on[:,0]), ' median %.0f' % np.median(on[:,0]))
    print(' video motion start minus release click, container A/V offset applied (ms):', np.round(on[:,1]), ' median %.0f' % np.median(on[:,1]))
    print(' implied audio-late-vs-video skew = sound onset - video start (ms):', np.round(on[:,0]-on[:,1]), ' median %.0f' % np.median(on[:,0]-on[:,1]))
