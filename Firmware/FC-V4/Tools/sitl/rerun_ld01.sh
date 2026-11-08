#!/bin/sh
# LD-01: re-run the MC-01 campaigns with the corrected mass properties and load outputs
set -e
B="--cant_sd 0.1 --fin1_sd 0.1 --bear_max 0.006"
python3 mc.py --n 200 $B --out runs/L_active
python3 mc.py --n 100 $B --wind_max 12 --out runs/L_wind12
python3 mc.py --n 40 $B --fault hardover --out runs/L_fault_hardover
python3 mc.py --n 200 --out runs/L_wide
python3 mc.py --n 100 $B --shadow --out runs/L_shadow
echo DONE
