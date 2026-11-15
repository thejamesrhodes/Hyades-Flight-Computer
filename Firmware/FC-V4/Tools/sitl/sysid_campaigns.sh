#!/bin/sh
# SYSID-01: the campaigns behind the study (run from Tools/sitl; about 2-3 min per 600-flight campaign on 2 cores).
B="--n 100 --servo bench02 --cant_sd 0.1 --fin1_sd 0.1 --bear_max 0.006"
W="--n 100 --servo bench02 --cant_sd 0.25 --fin1_sd 0.2 --bear_max 0.012"
IDEAL="--set sv_tau=0 --set sv_sod=0 --set sv_land=0 --set sv_lash=0 --set sv_wn=900 --set sv_zeta=0.7 --set sv_rate=200 --set sv_extra=0 --set sv_gain=1"
NOFR="--set bear_coul=0 --set bear_visc=0"
SLOW="--set sv_tau=0.021 --set sv_wn=60 --set sv_zeta=0.6"
# 1. what limits the current law (section 3)
python3 mc.py $B --out runs/NV5a_bench02_116
python3 mc.py $B $IDEAL --out runs/O1_ideal_act
python3 mc.py $B $NOFR --out runs/O2_nofric
python3 mc.py $B --variant=-DCTRL_ROLL_C=22.0f --out runs/O3_c22
python3 mc.py $B $IDEAL --variant=-DCTRL_ROLL_C=22.0f --out runs/O4_ideal_c22
python3 mc.py $B $IDEAL $NOFR --out runs/O5_ideal_nofric
# 2. observer law (section 4)
L2="-DCTRL_ROLL_LAW=2"
python3 mc.py $B "--variant=$L2" --out runs/E1_eso
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_BETA=1.118f" --out runs/E2_eso_b2x
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_BETA=0.2795f" --out runs/E3_eso_bhalf
python3 mc.py $W "--variant=$L2" --out runs/E4_eso_wide
python3 mc.py $W --out runs/E4b_law1_wide
python3 mc.py $B "--variant=$L2" $SLOW --out runs/E6_eso_slowservo
python3 mc.py $B "--variant=$L2" --fault hardover --n 40 --out runs/E7_eso_hardover
python3 mc.py $B --fault hardover --n 40 --out runs/E7b_law1_hardover
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_WO=20.0f" $SLOW --out runs/E8_eso_wo20_slow
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_WO=20.0f" --out runs/E9_eso_wo20
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_WO=20.0f -DCTRL_ESO_DN=3 -DCTRL_ESO_TA=0.02f" $SLOW --out runs/E10_eso_wo20_slowmodel_slow
python3 mc.py $B "--variant=$L2 -DCTRL_ESO_WO=20.0f -DCTRL_ESO_DN=3 -DCTRL_ESO_TA=0.02f" --out runs/E11_eso_wo20_slowmodel
# 3. test manoeuvres (sections 5-6): man.py builds nothing; make the two simulators first
make && cp sitl sitl_law1
cc -std=c11 -O2 -D_DEFAULT_SOURCE -DCTRL_ROLL_LAW=2 -I../../Core/hyades/math -I../../Core/hyades/gnc -I../../Core/hyades/mission \
   -I../../Core/hyades/svc -I../../Core/Inc -o sitl_law2 sitl.c ../../Core/hyades/mission/flight.c ../../Core/hyades/mission/phase.c \
   ../../Core/hyades/mission/fdir.c ../../Core/hyades/mission/detect.c ../../Core/hyades/gnc/control.c ../../Core/hyades/gnc/alloc.c \
   ../../Core/hyades/gnc/iekf.c ../../Core/hyades/gnc/nav.c -lm
python3 man.py --exe ./sitl_law1 --kind 0 --n 20 --out runs/K0_none
python3 man.py --exe ./sitl_law1 --kind 1 --amp 114.6 --T 1.0 --t0 0.25 --n 20 --keep 1 --out runs/K1_step2_v2
python3 man.py --exe ./sitl_law2 --kind 1 --amp 114.6 --T 1.0 --t0 0.25 --n 20 --keep 1 --out runs/K1_step2_eso
python3 man.py --exe ./sitl_law1 --kind 2 --amp 114.6 --T 0.8 --t0 0.25 --n 20 --out runs/K2_doublet2_v2
python3 man.py --exe ./sitl_law1 --kind 2 --amp 114.6 --T 0.8 --t0 0.8 --n 20 --out runs/K2b_doublet2_late
python3 man.py --exe ./sitl_law1 --kind 3 --ua 0.5 --dt 0.1 --t0 0.25 --n 20 --out runs/K3_3211
python3 man.py --exe ./sitl_law1 --kind 4 --ua 0.5 --dt 0.1 --amp 114.6 --T 0.8 --t0 0.25 --n 20 --out runs/K4_3211_doublet
python3 man.py --exe ./sitl_law1 --kind 5 --ua 0.4 --T 2.0 --t0 0.25 --n 20 --out runs/K5_multisine
python3 man.py --exe ./sitl_law1 --kind 7 --ua 0.5 --dt 0.1 --amp 114.6 --T 0.8 --t0 0.25 --gap 0.2 --n 20 --keep 1 --out runs/K7_combined
# kept traces are written to /tmp/man_<pid>_<seed>.csv: move them to runs/sysid_traces/{k1,k1eso,k7}_<seed>.csv, then
python3 sysid_figs.py
