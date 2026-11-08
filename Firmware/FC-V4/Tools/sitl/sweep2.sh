#!/bin/bash
for C in $1; do for QM in 1000 2000; do
  v="-DCTRL_ROLL_C=${C}.0f -DCTRL_TRIM_BAND_K=0.3f -DCTRL_TRIM_QSCHED=1 -DCTRL_ROLL_Q_MIN=${QM}.0f"
  out=runs/sweep/C${C}_B0.3_Q1_QM${QM}
  python3 mc.py --n 20 --variant "$v" --out $out > /dev/null 2>&1
  python3 -c "import json;s=json.load(open('$out/summary.json'));p=s['pct'];print('C=$C QM=$QM all',s['all_pass_pct'],'R5b',s['pass']['R5b_coast_rms_le_60dps'],'R5a',s['pass']['R5a_roll_max_le_360dps'],'rms95',p['p_rms_coast']['p95'],'pmax95',max(p['p_max_boost']['p95'],p['p_max_coast']['p95']),'pmax',max(p['p_max_boost']['max'],p['p_max_coast']['max']),'defl99',p['defl_max']['p99'],'sat99',p['sat_frac']['p99'])"
done; done
