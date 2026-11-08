#!/bin/sh
# REC-01 rerun of the cord classes with the recontact metric, plus cord-hysteresis sensitivity
set -e
D=../sitl/runs/L_active/runs.csv
for c in kevlar nylon elastic; do python3 rec.py --n 2000 --cord $c --dep $D --out runs/R_$c --seed 11; done
for c in kevlar elastic; do python3 rec.py --n 1000 --cord $c --zeta 0.3 0.5 --dep $D --out runs/R_${c}_hyst --seed 11; done
for L in 6 8; do python3 rec.py --n 600 --cord elastic --cord_len $L --dep $D --out runs/S_elastic_L$L --seed 13; done
echo DONE
