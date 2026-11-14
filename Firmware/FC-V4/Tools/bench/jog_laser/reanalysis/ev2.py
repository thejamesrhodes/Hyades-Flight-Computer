import numpy as np
from ev import *
for tag, ncmd in [('7983',71),('7984',134)]:
    on = np.load(f'on_{tag}.npy'); tc, pk = on[:,0], on[:,1]
    for merge in [0.05, 0.07, 0.09]:
        keep = [0]
        for i in range(1, len(tc)):
            if tc[i] - tc[keep[-1]] > merge: keep.append(i)
        tk = tc[keep]
        g = np.diff(tk)
        print(tag, 'merge', merge, 'bursts', len(tk), 'expected', 2*ncmd, 'gap hist(ms)', np.histogram(g*1e3, bins=[0,90,120,150,180,220,280,350,500,1000,1e5])[0])
    merge = 0.07; keep = [0]
    for i in range(1, len(tc)):
        if tc[i] - tc[keep[-1]] > merge: keep.append(i)
    tk = tc[keep]; pkk = np.array([pk[k:(keep[j+1] if j+1 < len(keep) else len(pk))].max() for j, k in enumerate(keep)])
    np.save(f'burst_{tag}.npy', np.c_[tk, pkk])
    print('   first 40 bursts:', ' '.join(f'{v:.3f}({p:.0f})' for v, p in zip(tk[:40], pkk[:40])))
