import numpy as np, pickle
from scipy.optimize import minimize
from ev import load
TEXP = 0.0035
def sim(A, wn, zeta, R=np.inf, T=0.14, dt=1e-4, kind='so'):
    n = int(T/dt); th = np.zeros(n); v = 0.0; p = 0.0
    for i in range(1, n):
        if kind == 'so':
            a = wn*wn*(A - p) - 2*zeta*wn*v
            v = np.clip(v + a*dt, -R, R)
        else:  # first order lag (tau = 1/wn) with rate limit
            v = np.clip((A - p)*wn, -R, R)
        p += v*dt; th[i] = p
    # exposure boxcar (trailing window TEXP ending at t)
    k = int(TEXP/dt); c = np.cumsum(np.r_[np.zeros(k), th]); thb = (c[k:] - c[:-k])/k
    return np.arange(n)*dt, thb/A
def collect(tag, sel, pre=0.03, post=0.11, final_win=(0.07, 0.11)):
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); t, x, y, o, d = load(tag)
    xs = D['xs']; xprev = np.r_[D['x0'], xs[:-1]]
    out = []
    for k in sel(D):
        # onset: first frame after click-10ms where |x - xprev| > 0.25 px
        i = np.searchsorted(t, D['tc'][k] - 0.01)
        j = i
        while j < len(t)-1 and abs(x[j] - xprev[k]) < 0.25 and t[j] < D['tc'][k] + 0.5: j += 1
        if t[j] >= D['tc'][k] + 0.5: continue
        ton = t[j]
        m = (t >= ton - pre) & (t <= ton + post) & np.isfinite(x)
        mf = (t >= ton + final_win[0]) & (t <= ton + final_win[1]) & np.isfinite(x)
        xf = np.median(x[mf]); amp = xf - xprev[k]
        out.append(dict(k=k, ton=ton, tt=t[m]-ton, yy=(x[m]-xprev[k])/amp, amp_px=amp, amp_px_final=xs[k]-xprev[k], step=D['step'][k], lat=ton - D['tc'][k]))
    return out
def cost(params, steps, pxdeg, kind, use_R, ret=False):
    wn, zeta = abs(params[0]), abs(params[1]); R = abs(params[2]) if use_R else np.inf
    tot = 0; n = 0; fits = []
    cache = {}
    for s in steps:
        A = abs(s['amp_px'])/pxdeg
        key = round(A, 1) if use_R else 1.0
        if key not in cache: cache[key] = sim(key, wn, zeta, R, kind=kind)
        tm, ym = cache[key]
        best = (1e18, 0)
        for t0 in np.arange(-0.012, 0.0121, 0.0002):
            r = np.interp(s['tt'] - t0, tm, ym, left=0, right=ym[-1]) - s['yy']
            c = (r*r).sum()
            if c < best[0]: best = (c, t0)
        tot += best[0]*(1 if not use_R else 1); n += len(s['tt']); fits.append(best[1])
    if ret: return np.sqrt(tot/n), fits
    return tot/n
if __name__ == '__main__':
    import sys
    res = {}
    for tag, pxdeg in [('7983', 16.8), ('7984', 15.5)]:
        ten = collect(tag, lambda D: np.flatnonzero(np.abs(D['step']) >= 10))
        one = collect(tag, lambda D: np.flatnonzero(np.abs(np.abs(D['step']) - 1) < 1e-6))
        fine = collect(tag, lambda D: [k for k in np.flatnonzero(np.abs(D['step']) < 0.5) if abs(D['xs'][k] - np.r_[D['x0'], D['xs'][:-1]][k]) > 2.0])
        if tag == '7984':
            ten = [s for s in ten if s['k'] in (0, 1, 2, 6, 7)]     # free-moving ones only
        print(tag, 'n 10deg', len(ten), 'n 1deg', len(one), 'n fine moves', len(fine))
        print('   latency click->onset ms: 10deg', np.round([s['lat']*1e3 for s in ten]), '\n      1deg median %.0f min %.0f max %.0f' % tuple(np.percentile([s['lat']*1e3 for s in one], [50, 0, 100])))
        for name, steps in [('10deg', ten), ('1deg', one), ('fine(~0.3deg)', fine)]:
            # immediate-vs-final completeness
            comp = np.array([s['amp_px']/s['amp_px_final'] for s in steps])
            for kind in ['so', 'fo']:
                x0 = [180, 0.8] if kind == 'so' else [110, 1.0]
                r = minimize(cost, x0, args=(steps, pxdeg, kind, False), method='Nelder-Mead', options=dict(xatol=0.5, fatol=1e-7))
                rms, _ = cost(r.x, steps, pxdeg, kind, False, ret=True)
                print('   %-14s %s linear: wn=%.0f rad/s zeta=%.2f  rms resid %.4f (frac of step)' % (name, kind, abs(r.x[0]), abs(r.x[1]), rms))
                res[(tag, name, kind)] = r.x
            if name == '10deg':
                r = minimize(cost, [250, 0.8, 480], args=(steps, pxdeg, 'so', True), method='Nelder-Mead', options=dict(xatol=0.5, fatol=1e-7))
                rms, _ = cost(r.x, steps, pxdeg, 'so', True, ret=True)
                print('   %-14s so + rate limit: wn=%.0f zeta=%.2f R=%.0f deg/s  rms %.4f' % (name, *np.abs(r.x), rms))
                res[(tag, name, 'soR')] = r.x
        pickle.dump(dict(ten=ten, one=one, fine=fine), open(f'steps_{tag}.pkl','wb'))
    pickle.dump(res, open('dynfit.pkl','wb'))
