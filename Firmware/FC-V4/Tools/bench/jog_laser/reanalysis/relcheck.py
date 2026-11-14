import pickle, numpy as np
from dyn import cost
res, allsteps = pickle.load(open('dynfit2.pkl','rb'))
out = {}
for tag, pxdeg in [('7983',16.8),('7984',15.7)]:
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); tc = D['tc'].copy()
    if tag == '7983': tc[3] = 7.906
    b = np.load(f'burst_{tag}.npy')[:,0]
    rows = []
    for name, wn in [('ten',112),('one',132),('fine',175)]:
        st = allsteps[tag][name]
        _, t0s = cost([wn, 0.81], st, pxdeg, 'so', False, ret=True)
        for s, t0 in zip(st, t0s):
            k = s['k']; press = tc[k]
            start = s['ton'] + t0                      # model-based start of motion (video time)
            nxt = b[(b > press + 0.06) & (b < press + 0.27)]
            if len(nxt) == 0: continue
            rel = nxt[0]
            rows.append((name, k, press, rel, start, abs(s['amp_px'])/pxdeg))
    out[tag] = rows
    for name in ['ten','one','fine']:
        r = np.array([(x[2], x[3], x[4], x[5]) for x in rows if x[0] == name])
        if len(r) < 3: continue
        hold = r[:,1] - r[:,0]; dp = r[:,2] - r[:,0]; dr = r[:,2] - r[:,1]
        sl = np.polyfit(hold, dp, 1)[0] if hold.std() > 1e-4 else np.nan
        print(tag, name, 'n=%2d' % len(r), 'hold ms: median %.0f sd %.0f | start-press: median %+.0f sd %.0f | start-release: median %+.0f sd %.0f IQR %+.0f..%+.0f | slope d(start-press)/d(hold) = %.2f'
              % (np.median(hold)*1e3, hold.std()*1e3, np.median(dp)*1e3, dp.std()*1e3, np.median(dr)*1e3, dr.std()*1e3, *(np.percentile(dr,[25,75])*1e3), sl))
    r = np.array([(x[2], x[3], x[4]) for x in rows if x[0] in ('ten','one')])
    hold = r[:,1]-r[:,0]; dp = r[:,2]-r[:,0]
    A = np.c_[hold, np.ones(len(hold))]; coef, *_ = np.linalg.lstsq(A, dp, rcond=None)
    resid = dp - A@coef
    print(tag, 'pooled 10+1 deg: start-press = %.2f*hold %+.0f ms ; resid sd %.0f ms ; corr %.2f' % (coef[0], coef[1]*1e3, resid.std()*1e3, np.corrcoef(hold, dp)[0,1]))
pickle.dump(out, open('relcheck.pkl','wb'))
