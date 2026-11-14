import re, numpy as np, pickle
rows = []
import sys
LOG = sys.argv[1] if len(sys.argv) > 1 else '../data/console_2026-11-09.txt'
for ln in open(LOG, encoding='utf-8', errors='replace'):
    m = re.match(r'(\d\d):(\d\d):(\d\d) › cal jog (\d) ([+-][\d.]+)', ln)
    if m: rows.append((int(m[1])*3600+int(m[2])*60+int(m[3]), int(m[4]), float(m[5])))
rows = np.array(rows)
def sess(t0, t1):
    m = (rows[:,0] >= t0) & (rows[:,0] <= t1) & (rows[:,1] == 2)
    return rows[m]
L = {'7983': sess(15*60+21, 16*60+7), '7984': sess(18*60+17, 19*60+12)}
for tag in ['7983','7984']:
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); lg = L[tag]
    tc = D['tc'].copy()
    if tag == '7983': tc[3] = 7.906
    assert len(lg) == len(tc), (len(lg), len(tc))
    assert np.allclose(lg[:,2], D['step']), 'sequence mismatch'
    S = lg[:,0]
    lo = (S - tc).max(); hi = (S + 1 - tc).min()
    print(tag, 'n', len(tc), 'sequence matches log: True | offset window: [%.3f, %.3f) s -> width %.0f ms' % (lo, hi, (hi-lo)*1e3))
    if hi <= lo:
        O = 0.5*(lo+hi); bad = np.flatnonzero(np.floor(tc + O) != S)
        print('   inconsistent commands:', len(bad), bad[:20], 'residual (s):', np.round((tc + O - S)[bad], 3)[:20])
    else:
        O = 0.5*(lo+hi)
        hh = int(O + 0)  # seconds since midnight of video t=0
        print('   video t=0 at PC clock 00:%02d:%06.3f (+-%.0f ms); file creation_time tag minus duration gives 00:%s' % (int(O//60), O%60, (hi-lo)*500, {'7983':'15:19.37','7984':'18:15.96'}[tag]))
