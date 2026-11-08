#!/bin/sh
# REC-01 campaign set. Deploy states come from the LD-01 SITL Monte Carlo (L_active).
set -e
D=../sitl/runs/L_active/runs.csv
for c in kevlar nylon elastic; do python3 rec.py --n 2000 --cord $c --dep $D --out runs/R_$c --seed 11; done
# stiffness sweep (p99 tension vs EA), same dispersions
for ea in 50 100 200 500 1000 2000 5000 10000 20000; do python3 rec.py --n 400 --ea $ea --dep $D --out runs/S_ea$ea --seed 12; done
# cord length (kevlar)
for L in 2 3 4 6 8; do python3 rec.py --n 600 --cord kevlar --cord_len $L --dep $D --out runs/S_L$L --seed 13; done
# early / late motor delay: deployment 25-40 m/s
python3 rec.py --n 1000 --cord kevlar --v_dep 25 40 --dep $D --out runs/R_fast_kevlar --seed 14
python3 rec.py --n 1000 --cord elastic --v_dep 25 40 --dep $D --out runs/R_fast_elastic --seed 14
# separation margin: low charge / tight fit
python3 rec.py --n 1000 --cord kevlar --bp 0.05 0.4 --hold 10 80 --dep $D --out runs/R_lowcharge --seed 15
# larger canopy (0.9 m, Cd 0.75)
python3 rec.py --n 1000 --cord kevlar --chute_d 0.9 --chute_cds 0.477 --dep $D --out runs/R_chute09 --seed 16
python3 rec.py --n 1000 --cord nylon --chute_d 0.9 --chute_cds 0.477 --dep $D --out runs/R_chute09_nylon --seed 16
echo DONE
