import numpy as np, pickle
np.set_printoptions(linewidth=220, precision=3, suppress=True)
OUT = {}
for tag, kr in [('7983', (5, 27)), ('7984', (7, 26))]:
    D = pickle.load(open(f'tab_{tag}.pkl','rb'))
    th, xs, step = D['th'], D['xs'], D['step']
    k = np.arange(kr[0], kr[1]+1)
    for deg in [1, 2, 3]:
        p = np.polyfit(th[k], xs[k], deg); r = xs[k] - np.polyval(p, th[k])
        s_mid = np.polyval(np.polyder(p), th[k].mean())
        print(tag, 'poly deg', deg, 'sens at mid %.2f px/deg' % s_mid, 'resid std %.2f px = %.3f deg, max |r| %.2f px = %.3f deg' % (r.std(ddof=deg+1), r.std(ddof=deg+1)/abs(s_mid), abs(r).max(), abs(r).max()/abs(s_mid)))
    p = np.polyfit(th[k], xs[k], 2); dp = np.polyder(p)
    sens = lambda t: np.polyval(dp, t)
    r = (xs[k] - np.polyval(p, th[k]))/sens(th[k])
    # 1 deg step sizes in deg
    ks = k[1:]; d1 = (xs[ks] - xs[ks-1])/sens(th[ks] - 0.5*step[ks])
    print('   1deg steps (n=%d): realised mean %.3f sd %.3f min %.2f max %.2f deg | lag-1 autocorr of step sizes %.2f' % (len(d1), abs(d1).mean(), d1.std(ddof=1), abs(d1).min(), abs(d1).max(), np.corrcoef(d1[:-1], d1[1:])[0,1]))
    print('   landing residual vs smooth fit: sd %.3f deg, range %.2f..%.2f' % (r.std(ddof=3), r.min(), r.max()))
    OUT[tag] = dict(p=p, sens_mid=sens(th[k].mean()), resid=r, d1=d1, k=k)
    # fine section
    fine = np.flatnonzero(np.abs(step) < 0.5); k0 = fine[0]
    xprev = np.r_[D['x0'], xs[:-1]]; dx = xs - xprev
    s_f = sens(th[fine].mean()) if tag == '7983' else sens(-21.0)   # local sensitivity (7984: use edge of valid cal range)
    mv = [kk for kk in fine if abs(dx[kk]) > 0.45]
    print('   fine: local sens %.2f px/deg; moves:' % s_f, [(int(kk-k0+1), round(dx[kk]/abs(s_f), 3)) for kk in mv])
    OUT[tag].update(s_f=s_f, mv=[(int(kk-k0+1), dx[kk]/abs(s_f)) for kk in mv], k0=k0)
pickle.dump(OUT, open('statics.pkl','wb'))
# 7983 details
D = pickle.load(open('tab_7983.pkl','rb')); O = OUT['7983']; th, xs = D['th'], D['xs']; s = O['s_f']
print('\n7983 fine section (bare): 43 cmds = 4.30 deg commanded; realised %.2f px = %.3f deg (local sens %.2f)' % (xs[70]-xs[27], (xs[70]-xs[27])/s, s))
mv = O['mv']; sizes = np.array([m[1] for m in mv]); idx = np.array([m[0] for m in mv])
print('   move cmd indices', idx, 'intervals', np.diff(idx))
print('   move sizes deg', np.round(sizes, 3), '| after first two: mean %.3f sd %.3f min %.3f max %.3f' % (sizes[2:].mean(), sizes[2:].std(ddof=1), sizes[2:].min(), sizes[2:].max()))
# landing error relative to commanded at each accepted update (using calibration poly from the downward staircase)
p = O['p']; acc_k = O['k0'] - 1 + idx
err = (xs[acc_k] - np.polyval(p, th[acc_k]))/s
print('   position minus downward-staircase calibration at accepted targets (deg):', np.round(err, 3), ' mean after take-up %.3f sd %.3f' % (err[2:].mean(), err[2:].std(ddof=1)))
print('   repeat visits: th=29.4:', xs[5]-D['x0'], 'px;  19.4:', xs[[0,4,15]], ' 9.4:', xs[[1,3,25]], ' -> spreads %.2f, %.2f deg' % (np.ptp(xs[[0,4,15]])/16.8, np.ptp(xs[[1,3,25]])/16.8))
print('   11.4 from above %.2f vs from below %.2f; 8.4: %.2f vs %.2f' % (xs[23], xs[67], xs[26], xs[37]))
# 7984
D = pickle.load(open('tab_7984.pkl','rb')); O = OUT['7984']; th, xs = D['th'], D['xs']
mv = O['mv']; idx = np.array([m[0] for m in mv]); sizes = np.array([m[1] for m in mv])
print('\n7984 fine: move cmd indices', idx)
lat = np.arange(3, 103, 3)
moved = np.isin(lat, idx)
print('   lattice points (multiples of 3):', len(lat), 'moved at', moved.sum(), '| off-lattice moves:', [int(i) for i in idx if i % 3])
late = lat >= 57
print('   region cmd>=57 (x<512): lattice pts %d, moved %d, missed %d' % (late.sum(), (moved & late).sum(), (~moved & late).sum()))
print('   10deg positions: -30:', xs[[0,2]], ' after event:', xs[6], ' -30 via +1:', xs[17], '| -40:', xs[1], xs[7], '| -20:', D['x0'], xs[3], xs[5], ' via +1:', xs[27])
