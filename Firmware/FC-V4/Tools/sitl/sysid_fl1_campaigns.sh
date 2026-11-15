#!/bin/sh
# SYSID-01 rev 2 (15 Nov 2026): the flight-1 build (law 2, roll test) in the SITL. About 35 min on 2 cores.
# Names match SYSID-01 section 9. Build tolerance B, wide tolerance W, slow servo SLOW as in rev 1.
B="--n 100 --servo bench02 --cant_sd 0.1 --fin1_sd 0.1 --bear_max 0.006"
W="--n 100 --servo bench02 --cant_sd 0.25 --fin1_sd 0.2 --bear_max 0.012"
SLOW="--set sv_tau=0.021 --set sv_wn=60 --set sv_zeta=0.6"
FIRST="-DRT_HOLD_S=0.8f -DRT_T0_S=0.25f -DRT_Q_ABORT_PA=1500.0f"     # the rev-1 schedule
# the rev-1 schedule, and no test at all (paired seeds)
python3 mc.py $B "--variant=$FIRST" --out runs/F1_flight1
python3 mc.py $B --variant=-DCFG_ROLL_TEST=0 --out runs/F0_flight1_notest
python3 mc.py $W --variant=-DCFG_ROLL_TEST=0 --out runs/F2n_wide_notest
python3 mc.py $B --fault hardover --n 40 --variant=-DCFG_ROLL_TEST=0 --out runs/F4n_hardover_notest
# timing variants
python3 mc.py $B "--variant=-DRT_HOLD_S=0.5f -DRT_T0_S=0.25f -DRT_Q_ABORT_PA=1500.0f" --out runs/F5_hold05
python3 mc.py $B "--variant=-DRT_HOLD_S=0.5f -DRT_T0_S=0.15f -DRT_Q_ABORT_PA=1500.0f" --out runs/F6_hold05_t015
python3 mc.py $B "--variant=-DRT_HOLD_S=0.5f -DRT_T0_S=0.15f -DRT_Q_ABORT_PA=1200.0f" --out runs/F7_hold05_t015_q12
python3 mc.py $B "--variant=-DRT_HOLD_S=0.8f -DRT_T0_S=0.15f -DRT_Q_ABORT_PA=1200.0f" --out runs/F9_hold08_t015_q12
# the flight-1 build as in board.h (hold 0.6 s, +0.15 s, abort 1.2 kPa)
python3 mc.py $B --out runs/G1_fl1_final
python3 mc.py $W --out runs/G2_fl1_final_wide
python3 mc.py $B $SLOW --out runs/G3_fl1_final_slowservo
python3 mc.py $B --fault hardover --n 40 --out runs/G4_fl1_final_hardover
# what the observer's 1 kPa gate and the 1.3x beta setting cost (no test)
python3 mc.py $B "--variant=-DCFG_ROLL_TEST=0 -DCTRL_ESO_BETA=0.559f" --out runs/N2_notest_beta_nom
python3 mc.py $B "--variant=-DCFG_ROLL_TEST=0 -DCTRL_ESO_Q_ON=300.0f" --out runs/N1_notest_qon300
# identification and step metrics (120 flights): flight-1 build, and the 0.8 s hold
make -s && cp sitl sitl_fl1
python3 man.py --exe ./sitl_fl1 --kind 0 --n 20 --keep 1 --out runs/M_fl1_final
E=$(python3 -c "import mc;print(mc.build('$FIRST'))")
python3 man.py --exe "$E" --kind 0 --n 20 --out runs/M_fl1
E=$(python3 -c "import mc;print(mc.build('-DRT_HOLD_S=0.5f -DRT_T0_S=0.15f -DRT_Q_ABORT_PA=1200.0f'))")
python3 man.py --exe "$E" --kind 0 --n 20 --out runs/M_fl1_h05
E=$(python3 -c "import mc;print(mc.build('-DRT_HOLD_S=0.8f -DRT_T0_S=0.15f -DRT_Q_ABORT_PA=1200.0f'))")
python3 man.py --exe "$E" --kind 0 --n 20 --out runs/M_fl1_h08_q12
python3 sysid_fl1_figs.py
