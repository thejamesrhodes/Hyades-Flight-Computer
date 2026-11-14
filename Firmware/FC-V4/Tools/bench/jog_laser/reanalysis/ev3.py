import numpy as np
from ev import *
def motion_events(t, x, thr=0.5):
    segs = plateaus(t, x, win=4, thr=0.3, minlen=10)
    ev = []
    for (a, b), (c, d) in zip(segs[:-1], segs[1:]):
        xa = np.nanmedian(x[max(a, b-24):b]); xb = np.nanmedian(x[c:min(d, c+24)])
        if abs(xb - xa) < thr: continue
        # onset: last frame before b-region where |x-xa| < 0.25, search from b-6
        i = max(a, b-8)
        while i < c and abs(x[i] - xa) < 0.25: i += 1
        ev.append(dict(i_on=i, t_on=t[i], t_last_static=t[i-1], xa=xa, xb=xb, dx=xb-xa, i_end=c, t_end=t[c]))
    return ev, segs
if __name__ == '__main__':
    for tag in ['7983','7984']:
        t, x, y, o, d = load(tag)
        ev, segs = motion_events(t, x)
        b = np.load(f'burst_{tag}.npy'); tb = b[:,0]
        print(tag, 'motion events', len(ev))
        for e in ev:
            if abs(e['dx']) < 6: continue
            prev = tb[tb < e['t_on'] + 0.004]
            pr = prev[-3:] if len(prev) else []
            nxt = tb[tb >= e['t_on'] + 0.004][:2]
            print('  t_on %.4f dx %+8.2f dur %.0f ms | clicks before (ms before onset): %s | after: %s' % (e['t_on'], e['dx'], (e['t_end']-e['t_on'])*1e3,
                  ' '.join('%.0f' % ((e['t_on']-p)*1e3) for p in pr), ' '.join('+%.0f' % ((p-e['t_on'])*1e3) for p in nxt)))
