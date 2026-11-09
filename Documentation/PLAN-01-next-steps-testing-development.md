# PLAN-01: next steps for testing and development (to flight 1 and V6)

**Date:** 8 November 2026.
**Baseline:**
- firmware at commit `5a8b294`;
- 12 host suites / 88 tests pass;
- SITL 3 840-flight re-run (LD-01);
- hardware HITL 4 / 4 on the final build.

**Sources:** ADR-004, MC-01, BP-01, STM-01, LD-01, REC-01.

The order is by what unblocks the most analysis. Each ground test closes named assumptions, and the simulation is then re-run on measured values rather than dispersed guesses.

## 1. Decisions needed now

| # | Decision | Options | Ref |
|---|---|---|---|
| D1 | Roll requirements after the CG correction (2 / 1 200 corner failures) | (a) R5a 260 °/s and R5b 50 °/s; (b) keep them and accept 0.17 %; (c) wait for the bench data (recommended, with (a) in the interim) | LD-01 §3.2 |
| D2 | Recovery configuration | **Leaning (9 Nov):** 48" Ultra-X cross canopy; 8 m of 3/16" tubular nylon with a parallel 1/8" tubular Kevlar backup at 15–20 % slack; a Kevlar leader at the motor end | REC-01 §5.7–5.8 |
| D3 | Flight-1 motor and delay | From the six-motor shortlist; sets `DET_T_APOGEE_MAX_S` and the delay | MC-01 §4, REC-01 §7 |

## 2. Ground tests, in order

| # | Test | Closes | Then |
|---|---|---|---|
| G1 | **Servo laser bench:** τ_d, f_N/ζ with a small-amplitude chirp, dead band, lash, step response at 6.0 / 7.4 / 8.4 V, dither A/B | MC-01 A17–A21 | `adr004_margin_check.m` and an MC re-run. Possibly c = 18–20. Settles D1. If the dead band is below 0.15°, re-run ALLOC-01 variant E1 |
| G2 | **Inertia:** trifilar Ixx (forward body), bifilar I_t (whole vehicle). **Weigh and balance** each section | A1, A2, A4, LD-01 A-L3 | MC re-run |
| G3 | **Fin can:** cant jig plus laser (≤ 0.25° net and per fin); bearing spin-down (T_c ≤ 6 mN m, at 0 and 20 °C) | A10–A12 (lock-in, friction) | Pass/fail against the build tolerance |
| G4 | **Canard zero** ≤ 0.1° by laser; polarity check (FDS-01 §6.6) | A9 | Align and save each canard's zero with the ground station Canards tab (CAL-01). READY is held until a calibration is stored. Uncalibrated 0.5–3° errors cost +27 % coast roll RMS p95 (ALLOC-01) |
| G5 | **Tap test** of the assembled airframe, then **static fire of a smaller motor** in the real fin can with FC-V4 capturing IMU at 896 Hz, LPF wide, clip flags logged | A24 (vibration), LD-01 A-L2 | Replace synthetic vibration in the MC; check the clip budget and LPF corner |
| G6 | **Recovery inputs:** weigh the reload's ejection charge; spring-scale the coupler fit; hang-test the cord for EA | REC-01 A-R1, A-R4, A-R9 | Re-run `rec_campaign.sh` |
| G7 | **Proof tests:** motor retention and bulkhead at 860 N (aft-ward and out-ward); cord and shroud attachments at 1.25 × limit (270 N with the recommended cord) | LD-01 T1–T4 | — |
| G8 | **Ejection ground test** with video (UKRA Safety Code 4.5): separation speed against the predicted p50 of 10.8 m/s; no motor movement | REC-01 A-R2 | Calibrates η |
| G9 | **Drop test** of the assembled sections at the descent-rate-equivalent height (3.3 m now, 1.5 m with the 0.9 m canopy) | LD-01 T7, A-L4 | Settles D2 if the 0.6 m chute is kept |
| G10 | **Flight-build HITL campaign:** build with `CFG_FLIGHT_BUILD=1` (bench features compiled out), full HITL-02 case list, chosen motor | Configuration as flown | Flight readiness review |
| G11 | **Pad end-to-end:** power-on to READY on the rail, the log-space inhibit, a `faults clear` drill, a phase-word injection on the pad (T16 → LOCKOUT), LED and buzzer indications | STM-01 T3, T13, T16 | Pad procedure sheet |
| G12 | **Static margin:** measure the CG (balance, loaded with the motor) and check the CP against OpenRocket/Barrowman using the measured as-built geometry (fin and canard dimensions, nose shape) | SAF-01 bound, MC-01 A4/A6 | Re-run `manoeuvre_sweep.py`; finalise the SAF-01 statement |
| G13 | **Canard effectiveness Cl_δ:** a fan or car-roof test of the forward body on a roll bearing (deflect the canards, measure the steady roll torque or spin rate at a known airspeed). A pitch-plane force check gives CN_δ for SAF-01 | CDS-01, MC-01 A7 (±30 %), SAF-01 | MC and SAF-01 re-run with the measured value. Narrows the largest single uncertainty in the roll loop |
| G14 | **Canopy CdS:** drop the chosen canopy (48" Ultra-X) with about 840 g from ≥ 10 m and time the steady fall (or tow it on a car at a known speed with a spring scale) | REC-01 A-R6 | Descent rate, drift, REC-01 re-run |
| G15 | **Cord stiffness:** hang test of the actual nylon (and the Kevlar backup) for EA; confirm backup slack > 1.5 × the predicted peak strain | REC-01 A-R9, §5.8 | `rec.py` re-run with the measured EA |
| G16 | **Fin-can spin test in airflow** (fan or car roof): measure the free-spin rate against speed, to check the 1.13 × \|cant°\| × f_pitch relation and the bearing friction under load | MC-01 FM3/FM4, lock-in | Validates the lock-in model |
| G17 | **PWM output check (8 MHz tick):** scope one fin output for the 1520 µs centre, 0.125 µs steps and the 6 ms fallback period, and check all four pulses start together | ALLOC-01 §7 | — |

G1–G4, G12–G15 and G17 need no motor. G5, G8 and G9 need the airframe assembled. G13 and G16 need a steady airflow (a car roof at a measured speed is the cheapest option; a 15–20 m/s run gives about 150–250 Pa, so the canard loads are small but measurable).

**Suggested order:**
1. Bench, no airflow: G2, G3, G4, G12, G17, G1.
2. Recovery inputs: G6, G14, G15, then G7.
3. Airflow: G13, G16.
4. Assembled airframe: G5, then G8 and G9.
5. G10 and G11 last, on the flight article.

After each block, re-run the affected simulation (MC-01 / `mc.py`, `rec.py`, `manoeuvre_sweep.py`) with the measured values.

## 3. Development

| # | Item | Why | Ref |
|---|---|---|---|
| S1 | Set `DET_T_APOGEE_MAX_S` for the chosen motor (pre-flight sheet) | The 11.5 s default suits the shortlist only | MC-01 §4 |
| S2 | Port the torque-space trim and the gain boost to MATLAB `controller.m`; re-run `montecarlo_roll_control` and Simulink | MATLAB still lags the C flight law | ADR-004 §10 |
| S3 | Fix the seed in `test_tools.py::test_imu_capture_to_noise_fit` | Flaky about 1 run in 3 | BP-01 §6 |
| S4 | LED and buzzer state patterns on FC-V4 | Pad GO/NO-GO is console-only today | FDS-01 §5.3, STM-01 §5 |
| S5 | Pad-knock false liftoff (≥ 10 g / 200 ms): evaluate a longer thrust-signature requirement against the liftoff latency budget (R2 ≤ 0.35 s; the pre-trigger ring covers 2.56 s) | Known limit; T4 cannot tell a big knock from a launch | STM-01 §4 |
| S6 | Recovery hardware per D2: cord, quick link and shroud attachments rated ≥ 420 N; FC tray and battery retention for 37 g (or 156 g with Kevlar) | REC-01 §5.5, §7 | |
| S7 | **V6 schematic review against BP-01 §5:** ±200 g accelerometer; raw IMU at ≥ 1 kHz to SD through apogee + 5 s; synchronous servo timers; backup-SRAM ring index; servo rail ADC | Adopted V6 requirements | BP-01 §5, REC-01 §7 |
| S9 | **NAV saturation vs the roll schedule:** with a > 16 g motor the IMU clips, q̂ is about 80 % wrong, and roll reaches 400–1 600 °/s (SITL and HITL I205W). Root-cause it: fallback gain vs corrupted q̂. Add a pre-flight check that motor thrust / mass ≤ 14 g. V6's ±200 g accelerometer is the hardware fix. | Outside the shortlist, but nothing in the firmware prevents it | FIG-01 F1 |
| S10 | Post-apogee roll (fins OFF, before deployment) reaches about 200 °/s in the active tail and about 400 °/s with wide tolerance. Decide whether it needs a requirement (cord twist, camera), or whether rate damping should continue at low q until deployment | Not covered by R5 | FIG-01 §3 |
| S11 | **CAL-01 bench run on the board:** pull the log with the old firmware first, then flash, run `log erase`, align and save, power-cycle and check the zeros reload, check that `log erase` rewrites the calibration and that unplugging USB mid-session reverts the zero | Implemented and host/simulator-tested only | CAL-01 §7–8 |
| S12 | **Firmware image headroom:** 95 880 B at -O2 against the 96 KB USB-update stage (2.4 KB left). Before the next feature, build at -Os (88 944 B) or move the bench-only code behind `CFG_FLIGHT_BUILD` | The next feature would no longer flash over USB | CAL-01 §8 |
| S8 | Later upgrades, after G1–G2: friction disturbance observer (DOB/ESO) in place of the trim integrator; SPRT detectors; INDI for the 3-axis H743 work | Dominant residual is bearing friction | BP-01 §2 |

## 4. Flight-1 readiness gates

1. D1–D3 decided.
2. G1–G4 measured, and the MC re-run on the measured values passes R1–R9 at ≥ 99.5 % with the build tolerances.
3. G3 tolerances met on the flight article.
4. G6–G9 passed with the flight recovery hardware.
5. G10 HITL campaign passes on the flight build. G11 drilled.
6. S1 set. S4 at least a minimal READY / NO-GO indication.
