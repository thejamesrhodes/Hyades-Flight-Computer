#!/bin/sh
N=60; OUT=runs/alloc
run() { name=$1; var=$2; shift 2
  python3 mc.py --n $N --out $OUT/$name --variant="$var" "$@" > $OUT/$name.log 2>&1; echo "$name done"; }
run B1_8MHz       "-DPWM_TICK_HZ=8000000u"
run C1_1MHz_stag  "-DCTRL_ALLOC_STAGGER=1"
run D1_1MHz_null  "-DCTRL_DITHER_NULL=1"
run G1_1MHz_nodither "-DCTRL_DITHER_ON=0"
