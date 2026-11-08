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
| D2 | Recovery configuration | 0.9 m canopy, plus a 6 m cord with EA ≈ 300 N (and a Kevlar leader at the motor), or keep the 0.6 m chute and 4 m Kevlar | REC-01 §1 |
| D3 | Flight-1 motor and delay | From the six-motor shortlist; sets `DET_T_APOGEE_MAX_S` and the delay | MC-01 §4, REC-01 §7 |

## 2. Ground tests, in order

| # | Test | Closes | Then |
|---|---|---|---|
| G1 | **Servo laser bench:** τ_d, f_N/ζ with a small-amplitude chirp, dead band, lash, step response at 6.0 / 7.4 / 8.4 V, dither A/B | MC-01 A17–A21 | `adr004_margin_check.m` and an MC re-run. Possibly c = 18–20. Settles D1. |
| G2 | **Inertia:** trifilar Ixx (forward body), bifilar I_t (whole vehicle). **Weigh and balance** each section | A1, A2, A4, LD-01 A-L3 | MC re-run |
| G3 | **Fin can:** cant jig plus laser (≤ 0.25° net and per fin); bearing spin-down (T_c ≤ 6 mN m, at 0 and 20 °C) | A10–A12 (lock-in, friction) | Pass/fail against the build tolerance |
| G4 | **Canard zero** ≤ 0.1° by laser; polarity check (FDS-01 §6.6) | A9 | — |
| G5 | **Tap test** of the assembled airframe, then **static fire of a smaller motor** in the real fin can with FC-V4 capturing IMU at 896 Hz, LPF wide, clip flags logged | A24 (vibration), LD-01 A-L2 | Replace synthetic vibration in the MC; check the clip budget and LPF corner |
| G6 | **Recovery inputs:** weigh the reload's ejection charge; spring-scale the coupler fit; hang-test the cord for EA | REC-01 A-R1, A-R4, A-R9 | Re-run `rec_campaign.sh` |
| G7 | **Proof tests:** motor retention and bulkhead at 860 N (aft-ward and out-ward); cord and shroud attachments at 1.25 × limit (270 N with the recommended cord) | LD-01 T1–T4 | — |
| G8 | **Ejection ground test** with video (UKRA Safety Code 4.5): separation speed against the predicted p50 of 10.8 m/s; no motor movement | REC-01 A-R2 | Calibrates η |
| G9 | **Drop test** of the assembled sections at the descent-rate-equivalent height (3.3 m now, 1.5 m with the 0.9 m canopy) | LD-01 T7, A-L4 | Settles D2 if the 0.6 m chute is kept |
| G10 | **Flight-build HITL campaign:** build with `CFG_FLIGHT_BUILD=1` (bench features compiled out), full HITL-02 case list, chosen motor | Configuration as flown | Flight readiness review |
| G11 | **Pad end-to-end:** power-on to READY on the rail, the log-space inhibit, a `faults clear` drill, a phase-word injection on the pad (T16 → LOCKOUT), LED and buzzer indications | STM-01 T3, T13, T16 | Pad procedure sheet |

G1–G4 need no motor. G5, G8 and G9 need the airframe assembled.

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
| S8 | Later upgrades, after G1–G2: friction disturbance observer (DOB/ESO) in place of the trim integrator; SPRT detectors; INDI for the 3-axis H743 work | Dominant residual is bearing friction | BP-01 §2 |

## 4. Flight-1 readiness gates

1. D1–D3 decided.
2. G1–G4 measured, and the MC re-run on the measured values passes R1–R9 at ≥ 99.5 % with the build tolerances.
3. G3 tolerances met on the flight article.
4. G6–G9 passed with the flight recovery hardware.
5. G10 HITL campaign passes on the flight build. G11 drilled.
6. S1 set. S4 at least a minimal READY / NO-GO indication.
