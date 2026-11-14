import numpy as np
from ev import *
from ev3 import motion_events
SEQ = {'7983': [(-10,3),(+10,3),(-1,22),(+0.1,43)], '7984': [(-10,2),(+10,3),(-10,3),(+1,24),(-0.1,102)]}
START = {'7983': 29.4, '7984': -20.0}
FINE83 = [28.550,29.053,29.523,30.110,30.639,31.229,31.663,32.326,32.661,33.181,33.683,34.232,34.746,35.160,35.755,36.206,36.673,37.176,37.716,38.193,38.726,39.201,39.68,40.166,40.656,41.195,41.697,42.142,42.596,43.136,43.592,44.081,44.631,45.039,45.589,46.128,46.585,47.034,47.476,47.849,48.341,48.822,49.202]
def build(tag):
    t, x, y, o, d = load(tag)
    ev, segs = motion_events(t, x, thr=0.5)
    b = np.load(f'burst_{tag}.npy'); tb, pb = b[:,0], b[:,1]
    seq = SEQ[tag]
    big_signs = [np.sign(s) for s, n in seq[:-1] for _ in range(n)]
    big_steps = [s for s, n in seq[:-1] for _ in range(n)]
    cand = [e for e in ev if abs(e['dx']) >= 2.0]
    cmds = []  # (t_cmd, step, t_onset)
    j = 0
    for s in big_steps:
        while np.sign(cand[j]['dx']) != np.sign(s): j += 1
        e = cand[j]; prev = tb[(tb < e['t_on'] + 0.005) & (tb > e['t_on'] - 0.12)]
        cmds.append([prev[-1] if len(prev) else e['t_on'] - 0.015, s, e['t_on']]); j += 1
    if tag == '7983':
        fine = FINE83
    else:
        fb = tb[(tb > 28.5) & (tb < 57.6)]
        assert len(fb) == 204, len(fb)
        fine = fb[0::2]
    for tf in fine: cmds.append([tf, seq[-1][0], np.nan])
    cmds = np.array(cmds)
    th = START[tag] + np.cumsum(cmds[:,1])
    # settled level after each command
    xs = []; ys = []; xsd = []
    tc = cmds[:,0]
    for k in range(len(tc)):
        t1 = tc[k+1] - 0.008 if k+1 < len(tc) else tc[k] + 0.6
        t0 = max(tc[k] + 0.12, t1 - 0.25) if t1 - tc[k] > 0.2 else tc[k] + 0.6*(t1 - tc[k])
        m = (t >= t0) & (t <= t1) & np.isfinite(x)
        xs.append(np.median(x[m]) if m.sum() >= 3 else np.nan); ys.append(np.median(y[m]) if m.sum() >= 3 else np.nan); xsd.append(np.std(x[m]) if m.sum() >= 3 else np.nan)
    m0 = (t < tc[0] - 0.02) & (t > tc[0] - 0.5) & np.isfinite(x)
    x0 = np.median(x[m0]); y0 = np.median(y[m0])
    return dict(t=t, x=x, y=y, tc=tc, step=cmds[:,1], ton=cmds[:,2], th=th, xs=np.array(xs), ys=np.array(ys), xsd=np.array(xsd), x0=x0, y0=y0, th0=START[tag], ev=ev)
if __name__ == '__main__':
    import pickle
    LOG0 = {'7983': 15*60+21, '7984': 18*60+17}
    for tag in ['7983','7984']:
        D = build(tag)
        print(tag, 'n cmds', len(D['tc']), 'start x %.2f y %.2f' % (D['x0'], D['y0']))
        xprev = np.r_[D['x0'], D['xs'][:-1]]; dx = D['xs'] - xprev
        for k in range(len(D['tc'])):
            if abs(D['step'][k]) >= 1 or True:
                print('  %3d t=%7.3f step %+6.2f th=%+7.2f x=%8.2f dx=%+8.2f y=%7.2f sd=%.3f' % (k, D['tc'][k], D['step'][k], D['th'][k], D['xs'][k], dx[k], D['ys'][k], D['xsd'][k]))
        pickle.dump(D, open(f'tab_{tag}.pkl','wb'))
