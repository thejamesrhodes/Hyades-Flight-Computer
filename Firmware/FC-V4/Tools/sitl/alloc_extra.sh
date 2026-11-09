#!/bin/sh
run() { out=$1; n=$2; var=$3; shift 3
  python3 mc.py --n $n --out $out --variant="$var" "$@" > $out.log 2>&1; echo "$out done"; }
H="-DPWM_TICK_HZ=8000000u -DCTRL_DITHER_NULL=1"
mkdir -p runs/alloc runs/alloc_quiet runs/alloc_hiq runs/alloc_zero
run runs/alloc/H1_8MHz_null 60 "$H"
Q="--set wind=0 --set turb=0 --set gust_amp=0 --set bear_coul=0.001 --set can_cant_deg=0 --set fin1_mis_deg=0 --set cl0_body=0 --set thrust_mis_deg=0.05"
run runs/alloc_quiet/H1_8MHz_null 40 "$H" $Q
for v in "A1_1MHz:" "B1_8MHz:-DPWM_TICK_HZ=8000000u" "C1_1MHz_stag:-DCTRL_ALLOC_STAGGER=1" "E1_8MHz_stag_null:-DPWM_TICK_HZ=8000000u -DCTRL_ALLOC_STAGGER=1 -DCTRL_DITHER_NULL=1" "H1_8MHz_null:$H" "G1_1MHz_nodither:-DCTRL_DITHER_ON=0"; do
  run runs/alloc_hiq/${v%%:*} 40 "${v#*:}" --set thrust_scale=1.8
done
# canard zero error (spline indexing), uncalibrated, 0.5-3 deg on one canard from launch
run runs/alloc_zero/H1_cal 40 "$H"
run runs/alloc_zero/H1_uncal 40 "$H" --fault offset --set fault_t=0
