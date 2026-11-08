#!/bin/bash
# gain factorial: $1 = list of C values
for C in $1; do for B in 1.1 0.3; do for Q in 0 1; do for QM in 2000 1000; do
  v="-DCTRL_ROLL_C=${C}.0f -DCTRL_TRIM_BAND_K=${B}f -DCTRL_TRIM_QSCHED=$Q -DCTRL_ROLL_Q_MIN=${QM}.0f"
  out=runs/sweep/C${C}_B${B}_Q${Q}_QM${QM}
  python3 mc.py --n 20 --variant "$v" --out $out > /dev/null 2>&1
  python3 -c "import json;s=json.load(open('$out/summary.json'));p=s['pct'];print('C=$C B=$B Q=$Q QM=$QM all',s['all_pass_pct'],'R5b',s['pass']['R5b_coast_rms_le_60dps'],'R5a',s['pass']['R5a_roll_max_le_360dps'],'rms95',p['p_rms_coast']['p95'],'pmax95',max(p['p_max_boost']['p95'],p['p_max_coast']['p95']),'pmax',max(p['p_max_boost']['max'],p['p_max_coast']['max']),'sat99',p['sat_frac']['p99'])"
done; done; done; done
