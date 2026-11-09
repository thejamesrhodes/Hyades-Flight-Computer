#!/bin/sh
# Limiting-factor study: remove one disturbance class at a time, or stiffen the loop.
run() { out=runs/lim/$1; var=$2; shift 2; python3 mc.py --n 40 --out $out --variant="$var" "$@" > $out.log 2>&1; echo "$1 done"; }
mkdir -p runs/lim
run base ""
run no_body_asym "" --set cl0_body=0
run no_fincan_cant "" --set can_cant_deg=0 --set fin1_mis_deg=0
run no_asym_any "" --set cl0_body=0 --set can_cant_deg=0 --set fin1_mis_deg=0
run no_wind_gust "" --set wind=0 --set gust_amp=0 --set turb=0
run ideal_servo "" --set sv_db=0.0002 --set sv_lash=0 --set sv_tau=0.002 --set sv_wn=300
run c20 "-DCTRL_ROLL_C=20.0f"
run c20_ideal_servo "-DCTRL_ROLL_C=30.0f" --set sv_db=0.0002 --set sv_lash=0 --set sv_tau=0.002 --set sv_wn=300
