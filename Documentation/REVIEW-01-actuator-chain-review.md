# REVIEW-01 — Actuator chain and flight software review

**Rev 2, 10 Nov 2026 (evening).** Rev 1 (same day) covered the actuator chain after the first servo bench data. Rev 2 adds the scale result and its fix (F1), the remaining rev 1 fixes, and a review of the flight software, tools and simulations (F16–F24).
**Scope.** Rev 1: BENCH-01 against the raw data, then every firmware constant, SITL/HITL/MATLAB model, margin claim and document that depends on a servo number. Rev 2: all hand-written flight code, host tests, SITL, ground tools, and the documents that quote a changed number.
**Not done.** LD-01, REC-01 and FDS-01 were not re-derived (§4). No electronics documents.
**Labels.** [F] verified fact, [I] inference, [S] speculation.

## 1. Bottom line

1. **The servo scale was wrong and is fixed (F1).** The firmware commanded 6.667 µs/°; the servo follows the maker's 8.889 µs/°, so every canard angle was 0.75 of the command. James confirmed 0.75 on the bench. `board.h` now uses `SERVO_RANGE_DEG 135`, and the build is flashed and running (`75638673+01b712`). **15 Nov: reopened.** Measured on the hub, the canard turns 0.86 of the command at 8.889 µs/° (BENCH-02 §11), so the servo is ≈ 10.3 µs/° and the 0.75 check is not confirmed. Corrected the same day: `SERVO_RANGE_DEG 116`.
2. **Requirements hold at the corrected scale.** With the build tolerances and the measured servo, 1 200 of 1 200 flights pass R1–R9; coast roll RMS p95 19.9 °/s and peak p95 111 °/s meet G1 and G2 (§3).
3. **Flight software: two defects fixed, neither on the nominal flight path (F16, F17).** One design gap needs a decision: READY does not depend on navigation initialisation or on a still gyro calibration (F18).
4. **c stays 15 (F4).** Sub-degree moves started 21–28 ms late in the quasi-static test. A dynamic test with dither decides.
5. **Bench work left:** confirm 10° of jog is 10° at the canard; hand check of the flight-like drive (F2); servo travel beyond the data-sheet pulse range (F22).

## 2. Findings

| # | Sev. | Finding | Evidence | Status |
|---|---|---|---|---|
| F1 | High | **Servo scale.** `board.h` mapped 900–2100 µs to 180° (6.667 µs/°). Blue Bird: 90° for 1100–1900 µs (8.889 µs/°) | [F] maker page, `board.h`. [F] James's bench check, 10 Nov: canard angle 0.75 of the command. [F] SITL at 0.75: goals G1/G2 missed at build tolerance, 89.7 % at default dispersions | **Fixed.** `SERVO_RANGE_DEG 135`; ±30° is now 266.7 µs. Zero limits re-derived (F11). Stored zeros converted × 3/4, the same pulse (CAL-01 §9). Flashed. **15 Nov (BENCH-02 §11): measured on the hub, the canard turns 0.86 of the command at this scale (servo ≈ 10.3 µs/°), so the data-sheet 8.889 µs/° is about 14 % off and the 10 Nov 0.75 check (which this predicts as 0.645) is not confirmed. Fixed again 15 Nov:** `SERVO_RANGE_DEG 116` (10.34 µs/°), zero limits ±56°/±26°, calibration record version 4 converts versions 3 and 2 to the same pulse (CAL-01 §10). Flashed `a5dc2489`; SITL with the BENCH-02 servo 600/600 |
| F2 | High | **Setup B is restricted above about −21° commanded (old scale).** A 10° jog delivered 4.75° and held 2.4 s; something then slipped by about 1.2° | [F] BENCH-01 §3.4 | **Open: check by hand.** A stalled servo draws up to 1.3 A |
| F3 | Med. | **Dead band is a 2.0 µs threshold on the input, not ±1 µs on the error**, plus a landing error of SD 0.15° | [F] BENCH-01 §3.1–3.2 | **Fixed in the models:** `sv_sod`, `sv_land` in `sitl.c`; `mc.py --servo bench01` (true angles); `hitl_model.SERVO_BENCH01`. MATLAB not changed |
| F4 | Med. | **Sub-degree moves start late.** 0.225° updates start 21–28 ms after larger jogs | [F] BENCH-01 §3.6. [F] Margins at c = 15 with τ_d 39 ms: PM 54.6°, GM 7.6 dB; at Ixx −30 %: 38.3°, 4.5 dB. At c = 18 and that corner: 26.9°, 3.0 dB | **Open: dynamic test** with dither (BENCH-01 §6). c stays 15 |
| F5 | Med. | **Favourable: servo faster than assumed.** f_N 130 rad/s, ζ 0.8 (model 80, 0.7). Dead time for larger jogs under about 13 ms | [F] BENCH-01 §3.5–3.6. [F] Margins at c = 15, τ_d 14 ms: PM 70.2°, GM 12.2 dB | Recorded. Unloaded, one servo |
| F6 | Med. | **BENCH-01 rev 1 errors:** B's scale ran through restricted levels; `audio.py` used A's audio offset for B; "1° steps start 30–40 ms late" was a mean with SD 72 ms; the 3-jog lattice was not reported | [F] rev 1 pipeline re-run; BENCH-01 §8 | **Fixed** (BENCH-01 rev 2) |
| F7 | Med. | **BENCH-01a errors.** Timing was referenced to the mouse press; `tk.Button` fires on release | [F] `fc_console.py`; motion start tracks the release click | **Fixed** (BENCH-01 rev 2 §3.6) |
| F8 | Low | **Dither margin.** At the old scale ±0.00294 rad was ±1.12 µs: 2.25 µs peak to peak against the 2.0 µs threshold | [F] `board.h`, BENCH-01 | **Closed by F1.** Now ±1.5 µs, 3.0 µs peak to peak, as designed |
| F9 | Low | **`rollopt.py` is cited as the margin tool but is not in the repo** | [F] repo search | **Fixed:** `Tools/sitl/roll_margins.py` reproduces ADR-004 §3.2 |
| F10 | Low | **CAL-01 fine jog.** ±0.02° is 1.4 PWM ticks but 1/11 of the servo's threshold. Alignment resolves to 0.225° with ±0.15° landing error | [F] BENCH-01 §3.1 | **Fixed in docs:** CAL-01 §4, PLAN-01 G4 |
| F11 | Low | **Zero limits ignored the off-centre neutral.** Neutral 1520 µs in 900–2100 µs leaves 580 µs one way | [F] `board.h`, `pwm.c` | **Fixed.** Zero range ±65°, full authority within ±35°, both compile-time checks against the nearer pulse limit |
| F12 | Low | **Stale comments** in `board.h` and `control.c` | [F] | **Fixed** |
| F13 | Low | **`sim_params.json` Trim.BandK is 1.1; `board.h` is 0.3.** MATLAB has not followed ADR-004 §3.3 | [F] | **Open** (PLAN-01 S2). Not changed: it cannot be verified without MATLAB |
| F14 | Info | **HITL servo model was the legacy one** | [F] `hitl_model.py` | **Fixed in the model:** optional `Sod` and `Land`, preset `SERVO_BENCH01`. The hardware campaign was not re-run |
| F15 | Info | **CDS-01 §3 frame rate read "250 Hz (3 ms max)"** | [F] | **Fixed** in CDS-01 |
| F16 | Low | **`app.c` included `calstore.h` inside `#if __has_include("build_id.h")`.** Without the generated build ID header the file would not compile | [F] `app.c` | **Fixed** |
| F17 | Low | **A corrupt phase word in CATO_SAFE went to LOCKOUT.** The flight-or-pad decision uses the detector's last phase, and CATO_SAFE did not count as flight. LOCKOUT is terminal, so LANDED was lost. Needs two faults | [F] `flight.c`; new test fails on the old code | **Fixed** with `test_corrupt_word_in_cato_stays_cato_then_lands` (STM-01 F1 note) |
| F18 | Med. | **READY does not depend on navigation initialisation or a still gyro calibration.** The boot gyro bias is an unchecked 1.3 s average. Roll-axis bias is not observable on the pad (roll axis along gravity, no magnetometer). A boot while the airframe moves leaves a roll-rate offset equal to the bias error. Above 5.7 °/s the pad never looks still, `NAV_F_INIT` is never set, q̂ is invalid for the flight, the roll gain falls back to its lowest value and apogee comes from the timer. Nothing on the pad shows it except the `w` values | [F] `imu.c`, `nav.c`, `flight.c`, `control.c`. [I] consequence chain; not flown in SITL, which boots still | **Open: design decision** (PLAN-01 S13). Proposed: hold READY until `NAV_F_INIT`; repeat the calibration if the samples were not still |
| F19 | Low | **The flight log stores fin commands as int8 × 127 of ±30°:** 0.24° per count against 1–3° commands in flight | [F] `logrec.h` | **Open** (PLAN-01 S14) |
| F20 | Info | **`test_imu_capture_to_noise_fit` was flaky** (its 15 % tolerance against an estimator SD of 11 %) | [F] 9 of 60 seeds outside tolerance | **Fixed:** the simulator's capture uses a fixed seed (PLAN-01 S3) |
| F21 | Info | **Stale comments** in `app.c`, `phase.h`, `control.h`, `flight.h`, `detect.h`, `imu.c/.h` (100 Hz frame, "detectors not implemented", "dither not carried across") | [F] | **Fixed** |
| F22 | Low | **Pulses outside 1100–1900 µs are not on the data sheet.** The zero limits assume 900–2100 µs is usable. With a zero near 0 the ±30° authority needs 1253–1787 µs, inside the data-sheet range | [F] maker page. [F] On 10 Nov a trial zero reached 2060 µs; whether the canard followed was not recorded | **Answered 15 Nov:** fin 2 follows to ±60° commanded (987–2053 µs) with no saturation (BENCH-02 §11). Other fins not checked |
| F23 | Low | **NAV-01 quoted superseded detector constants:** apogee timer 15 s (11.5 s), landing arm 5 s (15 s, plus the inertial rules), saturation on the first clipped frame (4-frame budget) | [F] `board.h`, `detect.c`, `nav.c` against NAV-01 §3 | **Fixed** in NAV-01 |
| F24 | Low | **Two flight-build items are not in PLAN-01.** `READY_HOLD_MS` is the 1 s bench value (flight 30–60 s) and the build guard does not check it. `REMAP_QMI` is still marked "verify on the bench" | [F] `board.h`, ADR-003, HITL-01, HITL-02 | **Open** (PLAN-01 S16) |

## 3. Simulation results

**At the corrected scale** (firmware as flashed; paired seeds):

| Set | Servo model | Coast roll RMS p50 / p95 / max (°/s) | Peak roll p95 / max (°/s) | All requirements |
|---|---|---|---|---|
| Default dispersions, 60 × 6 motors | Legacy | 15.2 / 32.3 / 73.4 | 201 / 303 | 96.7 % |
| | Measured (`--servo bench01`) | 13.9 / 31.1 / 73.4 | 200 / 307 | 96.4 % |
| Build tolerances, 200 × 6 motors | Legacy | 11.0 / 21.6 / 46.0 | 111 / 253 | 99.83 % (1 R5a, 1 R5b) |
| | Measured | 9.7 / 19.9 / 33.4 | 111 / 238 | **100 %** (0 of 1 200) |

- [F] The legacy rows equal the results before the change (ALLOC-01 H1; MC-01 99.8 %): the finer tick does not matter, as ALLOC-01 found.
- [F] Paired coast RMS, measured against legacy: median −1.1 °/s at build tolerance.
- [I] The simulations always took true = commanded, so they described the intended design, not the firmware as it was. The firmware now matches them.
- [I] PLAN-01 readiness gate 2 (≥ 99.5 %) is met for the servo part. It still needs G2–G4 and the loaded servo data.

**Before the correction** (rev 1, commanded units; kept for the record):

| Case, default dispersions, 360 flights | Coast roll RMS p50 / p95 (°/s) | Peak roll p95 (°/s) | All requirements |
|---|---|---|---|
| Legacy servo model | 15.2 / 32.4 | 201 | 96.7 % |
| Measured | 15.1 / 31.1 | 202 | 96.7 % |
| Measured, τ_d 39 ms | 15.8 / 33.0 | 199 | 96.1 % |
| Measured, true angle 0.75 of command | 18.1 / 45.3 | 258 | 89.7 % |

| Case, build tolerances, 1 200 flights | Coast roll RMS p50 / p95 (°/s) | Peak roll p95 / max (°/s) | All requirements |
|---|---|---|---|
| Legacy | 11.0 / 21.8 | 112 / 253 | 99.83 % |
| Measured | 10.5 / 20.8 | 112 / 258 | 99.92 % |
| Measured, true angle 0.75 of command (the firmware as it was) | 12.6 / 26.4 | 138 / 324 | 99.75 % |

Dither, 180 paired flights, change in coast RMS against the legacy baseline (median, old scale):

| Case | Change (°/s) | Peak change (°/s) |
|---|---|---|
| Legacy model, dither off | +3.8 | +7.9 |
| Measured model, dither as flown | −0.7 | −0.3 |
| Measured model, dither off | +5.9 | +16.0 |
| Measured model, dither halved | +1.9 | +7.4 |

Worst-case margins over the envelope and ±30 % aero (`roll_margins.py`, ADR-004 chain; independent of the scale):

| Servo | c = 15 | c = 18 | c = 20 |
|---|---|---|---|
| ADR-004 nominal (80, 0.7, 14 ms) | 66.8°, 10.8 dB | 61.0°, 9.2 dB | 57.2°, 8.3 dB |
| ADR-004 worst corner (52, 21 ms, Ixx −30 %) | 40.5°, 4.9 dB | 29.3°, 3.3 dB | 22.1°, 2.4 dB |
| Measured (130, 0.8), τ_d 14 ms | 70.2°, 12.2 dB | 65.1°, 10.6 dB | 61.8°, 9.7 dB |
| Measured (110, 0.8), τ_d 14 ms, Ixx −30 % | 58.5°, 8.7 dB | 51.2°, 7.1 dB | 46.4°, 6.2 dB |
| Measured, τ_d 39 ms | 54.6°, 7.6 dB | 46.4°, 6.0 dB | 41.1°, 5.1 dB |
| Measured, τ_d 39 ms, Ixx −30 % | 38.3°, 4.5 dB | 26.9°, 3.0 dB | 19.4°, 2.0 dB |

## 4. What was checked

- [F] **Read in full:** `board.h`, `app.c`, `flight.c`, `control.c`, `alloc.c`, `pwm.c`, `phase.c`, `detect.c`, `fdir.c`, `nav.c`, `iekf.c`, `align.c`, `fincal.c`, `calstore.c`, `flashlog.c`, `logq.c`, `imu.c`, `qmi8658.c`, `sched.c`, `wdg.c`, `gps.c`, `hitl.c`, `fwupdate.c`, and their headers.
- **Not read:** `main.c` and the other CubeMX files, `usbio.c`, `console.c`, `telemetry.c`, `leds.c`; `logrec.c`, `nmea.c` and `crc.c` only through their tests.
- [F] **Host tests:** 15 suites, 112 tests, 0 failures. Tool tests: 16, 0 failures, 1 skipped.
- [F] **Target build:** STM32CubeIDE headless, 0 errors, 0 warnings with `-Werror`. Image 79 352 B against the 98 304 B USB-update limit.
- [F] **On the board:** flashed over USB; the stored version 2 calibration was read and converted; READY reached.
- [F] **Estimator and navigation:** IEKF error dynamics, measurement Jacobian, Joseph update and the vertical Kalman update checked by hand against the code. No error found.
- [F] **State machine:** STM-01 T2–T7 match `detect.c` and `phase.c`.
- [F] **Documents that quote a changed number** corrected: CAL-01, ALLOC-01, CDS-01, MC-01, ADR-004, PLAN-01, STM-01, SW-01, NAV-01, BENCH-01.
- [F] **SAF-01:** its bound is stated up to aerodynamic stall, so it does not depend on the scale.
- **Not re-derived:** LD-01, REC-01 and FDS-01. A search found no dependence on the changed constants. They are not otherwise audited here.
- [F] PWM arithmetic, the timer latch, the jog sequence against the console log, and the ADR-004 margin table were confirmed in rev 1.

## 5. Changes made

| Area | Change |
|---|---|
| `board.h`, `alloc.h`, `pwm.c` | Scale 8.889 µs/°; zero limits ±65° and ±35° derived from the pulse range |
| `fincal.c/.h`, `app.c` | Calibration record version 3; version 2 read and converted × 3/4; boot report says so |
| `flight.c` | F17 |
| `app.c` | F16; comments |
| `align.c`, comments in six headers | Limit text; F21 |
| Tests | `test_fincal` (2 new), `test_flight` (1 new), `test_alloc`, `test_align`, `test_tools.py` |
| `fc_console.py`, `fc_sim.py` | Limits and Canards tab text; fixed capture seed (F20) |
| `sitl.c`, `mc.py`, `roll_margins.py`, `hitl_model.py` | Measured servo model in true angles (F3, F9, F14) |
| `Tools/bench/jog_laser/` | Audio offset fix; `reanalysis/` scripts and data |
| Documentation | BENCH-01 rev 3; this file; CAL-01 §9; notes in ALLOC-01, CDS-01, MC-01, ADR-004, PLAN-01, STM-01, SW-01, NAV-01 |

## 6. Actions for James

1. ~~Confirm on the bench that a 10° jog is 10° at the canard.~~ Measured 15 Nov: 0.86 (BENCH-02 §11); corrected to `SERVO_RANGE_DEG 116`. Check 10° is now 10° at the canard.
2. Hand check of setup B at the old −10° position (F2).
3. Decide F18 (READY gate) and F24 (flight READY hold).
4. Re-align and save, so the stored record is version 4.
5. Record the servo supply voltage, then the scripted dynamic run with dither on and off (BENCH-01 §6).
