#!/bin/sh
# ALLOC-01 campaign: allocation and PWM-tick variants, same seeds (paired).
#   sh alloc_campaign.sh [n_per_motor] [extra mc.py args...]
N=${1:-60}; shift
OUT=${OUT:-runs/alloc}
run() { name=$1; var=$2; shift 2
  python3 mc.py --n $N --out $OUT/$name --variant="$var" "$@" > $OUT/$name.log 2>&1; echo "$name done"; }
mkdir -p $OUT
run A0_continuous "" --set pwm_quant=0 "$@"
run A1_1MHz       "" "$@"
run B1_8MHz       "-DPWM_TICK_HZ=8000000u" "$@"
run C1_1MHz_stag  "-DCTRL_ALLOC_STAGGER=1" "$@"
run D1_1MHz_null  "-DCTRL_DITHER_NULL=1" "$@"
run E1_8MHz_stag_null "-DPWM_TICK_HZ=8000000u -DCTRL_ALLOC_STAGGER=1 -DCTRL_DITHER_NULL=1" "$@"
run F1_8MHz_stag_nodither "-DPWM_TICK_HZ=8000000u -DCTRL_ALLOC_STAGGER=1 -DCTRL_DITHER_ON=0" "$@"
run G1_1MHz_nodither "-DCTRL_DITHER_ON=0" "$@"
