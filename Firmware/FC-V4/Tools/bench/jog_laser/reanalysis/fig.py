import numpy as np, pickle, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from ev import load
from dyn import sim
BLUE, ORANGE, INK, MUTED, GRID, SURF = '#2a78d6', '#eb6834', '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
plt.rcParams.update({'font.size': 12, 'axes.edgecolor': GRID, 'axes.labelcolor': MUTED, 'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.titlecolor': INK,
                     'axes.titlesize': 13, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'figure.facecolor': SURF, 'axes.facecolor': SURF, 'font.family': 'DejaVu Sans'})
fig, axs = plt.subplots(2, 2, figsize=(13, 10))
def clean(ax):
    ax.grid(True, color=GRID, lw=1); ax.set_axisbelow(True)
    for s in ['top', 'right']: ax.spines[s].set_visible(False)
ST = pickle.load(open('statics.pkl','rb'))
# A, B: fine sections
for ax, tag, tlim, title in [(axs[0,0], '7983', (28.0, 50.2), 'A  Bare servo: 0.1° jogs'), (axs[0,1], '7984', (46.0, 57.2), 'B  Flight-like: 0.1° jogs')]:
    D = pickle.load(open(f'tab_{tag}.pkl','rb')); t, x, y, o, d = load(tag)
    s = abs(ST[tag]['s_f']); sgn = 1 if tag == '7983' else -1
    fine = np.flatnonzero(np.abs(D['step']) < 0.5)
    tc = D['tc'][fine]; m = (tc >= tlim[0] - 0.6) & (tc <= tlim[1])
    k_first = fine[m][0]
    # reference level = measured level just before first shown command
    mref = (t > D['tc'][k_first] - 0.25) & (t < D['tc'][k_first] - 0.02)
    xref = np.nanmedian(x[mref])
    mt = (t >= tlim[0] - 0.3) & (t <= tlim[1])
    meas = sgn*(x[mt] - xref)/s
    tcs = tc[m]; cmd = 0.1*np.arange(1, len(tcs)+1)
    ax.step(np.r_[tlim[0]-0.3, tcs, tlim[1]], np.r_[0, cmd, cmd[-1]], where='post', color=ORANGE, lw=2, label='Commanded', solid_capstyle='round')
    ax.plot(t[mt], meas, color=BLUE, lw=2.4, label='Measured (laser)', solid_capstyle='round')
    ax.set_xlim(tlim[0]-0.3, tlim[1]); ax.set_xlabel('Video time (s)'); ax.set_ylabel('Angle change (commanded-degree units)')
    ax.set_title(title); clean(ax); ax.legend(loc='upper left', frameon=False)
axs[0,0].annotate('moves only on every 3rd jog\n(3 × 0.1° = 2.0 µs of pulse)', xy=(40.7, 2.35), xytext=(41.3, 0.75), color=INK, fontsize=11, arrowprops=dict(arrowstyle='-', color=MUTED, lw=1))
axs[0,1].annotate('update with no motion\n(6 jogs between moves)', xy=(51.7, 1.88), xytext=(52.3, 0.6), color=INK, fontsize=11, arrowprops=dict(arrowstyle='-', color=MUTED, lw=1))
# C: realised 1 deg steps
ax = axs[1,0]
rng = np.random.default_rng(3)
for i, (tag, col, lab) in enumerate([('7983', BLUE, 'Bare servo'), ('7984', ORANGE, 'Flight-like')]):
    d1 = np.abs(ST[tag]['d1'])
    ax.plot(i + rng.uniform(-0.16, 0.16, len(d1)), d1, 'o', ms=9, color=col, mec=SURF, mew=1.5, label=lab)
    ax.plot([i-0.3, i+0.3], [d1.mean()]*2, color=INK, lw=2)
    ax.text(i+0.33, d1.mean(), f'mean {d1.mean():.2f}°\nSD {d1.std(ddof=1):.2f}°', va='center', color=INK, fontsize=11)
ax.axhline(1.0, color=MUTED, lw=1); ax.set_xlim(-0.6, 1.9); ax.set_xticks([0, 1]); ax.set_xticklabels(['Bare servo (n = 22)', 'Flight-like (n = 19)'])
ax.set_ylim(0, 1.8); ax.set_ylabel('Realised size of each 1° jog (°)'); ax.set_title('C  Realised size of nominal 1° jogs'); clean(ax); ax.grid(axis='x', visible=False)
# D: step response
ax = axs[1,1]
res, allsteps = pickle.load(open('dynfit2.pkl','rb'))
from dyn import cost
for tag, col, lab, pxdeg in [('7984', ORANGE, 'Flight-like, 1° jogs', 15.7), ('7983', BLUE, 'Bare servo, 1° jogs', 16.8)]:
    st = allsteps[tag]['one']
    _, t0s = cost([135, 0.82], st, pxdeg, 'so', False, ret=True)
    first = True
    for s_, t0 in zip(st, t0s):
        sc = s_['amp_px']/s_['final_px'] if tag == '7984' else 1.0
        ax.plot((s_['tt'] - t0)*1e3, s_['yy']*sc, 'o', ms=4.5, color=col, mec='none', alpha=0.85, label=lab if first else None); first = False
tm, ym = sim(1.0, 135, 0.82)
ax.plot(tm*1e3, ym, color=INK, lw=2, label='2nd-order fit: ωn = 135 rad/s, ζ = 0.82')
ax.set_xlim(-10, 100); ax.set_ylim(-0.08, 1.2); ax.set_xlabel('Time from motion onset (ms)'); ax.set_ylabel('Fraction of final move')
ax.set_title('D  1° step response and fit'); clean(ax); ax.legend(loc='lower right', frameon=False, fontsize=11)
ax.annotate('stalls short, finishes later', xy=(45, 0.60), xytext=(52, 0.36), color=INK, fontsize=11, arrowprops=dict(arrowstyle='-', color=MUTED, lw=1))
fig.suptitle('BMS-127WV+ canard actuator, laser-on-wall test at 240 fps (9–10 Nov 2026)', x=0.012, ha='left', fontsize=15, fontweight='bold', color=INK)
plt.tight_layout(rect=(0, 0, 1, 0.965)); plt.savefig('bench01_reanalysis.png', dpi=120)
