import numpy as np, pickle
from scipy.optimize import minimize
from ev import load
from dyn import sim, cost
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def collect(tag, ks, pre=0.03, post=0.12, final_win=(0.06, 0.10)):
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); t, x, y, o, d = load(tag)
    out = []
    for k in ks(D):
        tc = D['tc'][k]
        # robust onset: search window tc-15ms .. next cmd; pre-level tracked as running median of previous 8 frames
        i = np.searchsorted(t, tc - 0.015); j = i
        tnext = D['tc'][k+1] if k+1 < len(D['tc']) else tc + 0.6
        pre_lvl = np.nanmedian(x[i-10:i])
        while t[j] < min(tc + 0.25, tnext - 0.03) and abs(x[j] - pre_lvl) < 0.3:
            j += 1
        if abs(x[j] - pre_lvl) < 0.3: continue
        ton = t[j]
        pre_lvl = np.nanmedian(x[j-9:j-1])
        m = (t >= ton - pre) & (t <= min(ton + post, tnext - 0.004)) & np.isfinite(x)
        mf = (t >= ton + final_win[0]) & (t <= min(ton + final_win[1], tnext - 0.004)) & np.isfinite(x)
        if mf.sum() < 3: continue
        xf = np.median(x[mf]); amp = xf - pre_lvl
        if abs(amp) < 1.5: continue
        # late creep: level at 0.2-0.3 s after onset if available
        ml = (t >= ton + 0.2) & (t <= min(ton + 0.3, tnext - 0.004))
        out.append(dict(k=k, ton=ton, tt=t[m]-ton, yy=(x[m]-pre_lvl)/amp, amp_px=amp, step=D['step'][k], lat=ton - tc, late=(np.median(x[ml]) - pre_lvl) if ml.sum() > 5 else np.nan,
                        final_px=D['xs'][k] - pre_lvl))
    return out
if __name__ == '__main__':
    res = {}; allsteps = {}
    fig, axs = plt.subplots(2, 3, figsize=(18, 9))
    for r_, (tag, pxdeg) in enumerate([('7983', 16.8), ('7984', 15.5)]):
        ten = collect(tag, lambda D: np.flatnonzero(np.abs(D['step']) >= 10))
        one = collect(tag, lambda D: np.flatnonzero(np.abs(np.abs(D['step']) - 1) < 1e-6))
        fine = collect(tag, lambda D: np.flatnonzero(np.abs(D['step']) < 0.5))
        if tag == '7984': ten = [s for s in ten if s['k'] in (0, 1, 2, 6, 7)]
        allsteps[tag] = dict(ten=ten, one=one, fine=fine)
        print(tag, 'n 10deg', len(ten), 'n 1deg', len(one), 'n fine moves', len(fine))
        for c_, (name, steps) in enumerate([('10deg', ten), ('1deg', one), ('fine', fine)]):
            r = minimize(cost, [150, 0.8], args=(steps, pxdeg, 'so', False), method='Nelder-Mead', options=dict(xatol=0.5, fatol=1e-8))
            rms, t0s = cost(r.x, steps, pxdeg, 'so', False, ret=True)
            print('   %-6s linear 2nd order: wn=%.0f rad/s (%.1f Hz) zeta=%.2f  rms %.4f' % (name, abs(r.x[0]), abs(r.x[0])/2/np.pi, abs(r.x[1]), rms))
            res[(tag, name, 'so')] = np.abs(r.x)
            if name == '10deg':
                r2 = minimize(cost, [200, 0.9, 450], args=(steps, pxdeg, 'so', True), method='Nelder-Mead', options=dict(xatol=0.5, fatol=1e-8))
                rms2, _ = cost(r2.x, steps, pxdeg, 'so', True, ret=True)
                print('          + rate limit: wn=%.0f zeta=%.2f R=%.0f deg/s  rms %.4f' % (*np.abs(r2.x), rms2))
                res[(tag, name, 'soR')] = np.abs(r2.x)
            # individual fits
            ind = []
            for s in steps:
                ri = minimize(cost, np.abs(r.x), args=([s], pxdeg, 'so', False), method='Nelder-Mead', options=dict(xatol=0.5, fatol=1e-8))
                rmsi, _ = cost(ri.x, [s], pxdeg, 'so', False, ret=True)
                ind.append([abs(ri.x[0]), abs(ri.x[1]), rmsi, abs(s['amp_px'])])
            ind = np.array(ind)
            good = ind[:,2] < 0.03
            print('          per-step: n=%d (good fits %d) wn median %.0f IQR %.0f-%.0f | zeta median %.2f IQR %.2f-%.2f' % (len(ind), good.sum(), np.median(ind[good,0]), *np.percentile(ind[good,0],[25,75]), np.median(ind[good,1]), *np.percentile(ind[good,1],[25,75])))
            res[(tag, name, 'ind')] = ind
            ax = axs[r_, c_]
            tm, ym = sim(1.0, abs(r.x[0]), abs(r.x[1]))
            for s, t0 in zip(steps, t0s): ax.plot((s['tt'] - t0)*1e3, s['yy'], '.', ms=3, alpha=.6)
            ax.plot(tm*1e3, ym, 'k-', lw=1.2); ax.set_xlim(-10, 100); ax.set_ylim(-0.1, 1.25); ax.grid(alpha=.3)
            ax.set_title(f'{tag} {name}: wn={abs(r.x[0]):.0f} zeta={abs(r.x[1]):.2f} n={len(steps)}')
        print('   latency click->onset ms: 10deg', np.round([s['lat']*1e3 for s in ten]), '| 1deg: median %.0f min %.0f max %.0f' % tuple(np.percentile([s['lat']*1e3 for s in one], [50, 0, 100])),
              '| fine: median %.0f min %.0f max %.0f' % tuple(np.percentile([s['lat']*1e3 for s in fine], [50, 0, 100])))
    plt.tight_layout(); plt.savefig('dynfit.png', dpi=60)
    pickle.dump((res, allsteps), open('dynfit2.pkl','wb'))
