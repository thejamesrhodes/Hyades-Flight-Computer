# ADR-004: 200 Hz synchronous loop, pre-trigger flight log, flight-core SITL, and roll-law retune

**Status:** accepted 8 November 2026 (James: 200 Hz synced PWM, OpenRocket inertia now and measured later, ≤ 14 g motor shortlist; checkpoint: flight 1 = full rate law, requirements tightened, V6 IMU log and high-g adopted).
**Supersedes:** ADR-003 decision 10 gains (c = 70) and the H3 open item.
**Code:** `Firmware/FC-V4` at the commit that adds this file.
**Evidence:**
- MC-01 (SITL 6DOF Monte Carlo, 7 680 flights);
- STM-01 (state machine hand check);
- the HITL hardware run of 8 Nov (`Tools/logs/hitl/campaign-20261108-004621`);
- `adr004_margin_check.m` (MATLAB cross-check);
- figures in `Images/adr004/`.

---

## 1. Context

Four findings from the 7–8 Nov work invalidated the roll design that HITL-02 had passed.

| # | Finding | Evidence | Consequence |
|---|---|---|---|
| F1 | The servo PWM ran at **50 Hz** (`SERVO_PERIOD_US 20000`), asynchronous to the 100 Hz loop. | `board.h` §4 | About 10 ms of hidden delay: every other control output was never sent. |
| F2 | The roll loop had about **20 ms more delay** than the MATLAB design assumed (IMU filter, sample age, ZOH, PWM resampling). | §2 below | At c = 70: PM 11°, GM 1.14 on the old inertia. This is the HITL-02 "combined worst" limit cycle. |
| F3 | The canards turn only the **forward body**. The fin can spins on bearings (Free-Spinning Fin Can spec, Ip ≈ 2.6e-4 kg m²). Controlled Ixx ≈ **2.2e-4**, not the 7e-4 in the MATLAB file (marked "STALE"). | OpenRocket whole vehicle 4.78e-4 less the can | c = 70 is **unstable** (PM −28° even at 200 Hz). The HITL plant used 7e-4, so it never showed this. |
| F4 | The fin can couples into body roll through **bearing friction** (5–10 mN m, spec estimate), and through a **rotating side force** when a single fin is misaligned. | 6DOF SITL, MC-01 | This, not the dead band, dominates residual roll. It also sets a lock-in tolerance on fin alignment. |

## 2. Loop rate and delay budget

**Decision:**
- 200 Hz frame (`FRAME_HZ 200`).
- QMI8658A at 448.4 Hz ODR with its LPF at 13.37 % ODR (about 60 Hz).
- Servo pulses started by software straight after the control law (`PWM_Commit()`: timer update event on TIM2 and TIM3).
- The free-running servo period (6 ms) remains only as a fallback if frames stop.

The BMS-127WV+ accepts frames down to 3 ms (333 Hz) and resolves 12 bits. Maker data: [microbirds.com BMS-127WV+ page](https://microbirds.com/bms-127wv-blue-bird-micro-servo-dual-ball-bearings-5-3kg-cm-0-05s-11-3g-10mm).

| Delay element | 7 Nov (100 Hz / 50 Hz async) | ADR-004 |
|---|---|---|
| IMU LPF (group delay) | 7.5 ms (30 Hz) | 3.75 ms (60 Hz) |
| Sample age (half the ODR period) | 2.2 ms | 1.1 ms |
| Compute | 1.5 ms | 1.5 ms (measured ≤ 0.49 ms p99 on hardware) |
| Frame ZOH, T/2 | 5.0 ms | 2.5 ms |
| PWM resampling, T_pwm/2 | 10.0 ms | 0 (synchronous) |
| Servo pulse read | 1.5 ms | 1.5 ms |
| **Digital chain total** | **27.7 ms** | **10.4 ms** |
| Servo transport τ_d (unmeasured) | 14 ms | 14 ms |

Figure: `Images/adr004/delay_budget.png`.

**Consequences:**
- Every persistence count in `board.h` is now written in milliseconds and converted with `MS_TO_FRAMES()`, so a future rate change cannot silently shorten a detection window.
- Text status, telemetry and log decimations are derived from `FRAME_HZ`.
- The HITL host reads `FRAME_HZ` from `board.h`, and its plant IMU runs at 448 Hz.
- CPU on hardware: loop p99 330–490 µs against the 4 ms (80 %) budget. Text size 72.7 KB of 128 KB. RAM unchanged at 117 KB.

## 3. Roll law retune

The law is still MATLAB `controller.m` Mode 1: δ = −(c/q̂)·p + bounded trim.

### 3.1 Margins

Analysis was done with the Python port (`rollopt.py`) and **cross-checked in MATLAB**. `optimise_roll_schedule.m` gained optional `cfg.tau_extra` and `cfg.lpf_hz` (defaults reproduce the old results).

| Case | MATLAB feasible c (PM ≥ 35°, GM ≥ 2, Ms ≤ 2) | Python | c = 70 PM (MATLAB / Python) |
|---|---|---|---|
| No digital delay | 11.2–29.9 | 11–29.9 | 0.1° / 0° |
| 7 Nov chain | 11.2–16.7 | 11–16.7 | −72.9° / −76° |
| ADR-004 chain | 11.2–22.6 | 11–22.6 | −28.2° / −28° |

### 3.2 Gain choice

**c = 15.** The criteria are SAE AS94900 (formerly MIL-F-9490D: 45° / 6 dB worst case, about half that under roughly 20 % derivative uncertainty) and NASA NESC TB 22-05 (6 dB / 30° undispersed, 3 dB / 20° dispersed).

| c | Nominal servo: PM / GM / Ms | Worst corner (Ixx −30 %, τ_d 21 ms, f_N 52 rad/s): PM / GM |
|---|---|---|
| 12 | 72.7° / 12.6 dB / 1.39 | 52.0° / 6.8 dB |
| **15** | **67.0° / 10.8 dB / 1.51** | **40.0° / 4.8 dB** |
| 18 | 61.2° / 9.2 dB / 1.66 | 28.6° / 3.2 dB |
| 22 | 53.3° / 7.4 dB / 1.92 | 15.1° / 1.6 dB |

Figure: `Images/adr004/loop_margins.png`.

The nonlinear Monte Carlo stayed stable up to c = 26, but saturation and the dead band hide early oscillation, so margins rather than pass rates choose c. **c can rise to 18–20 once the servo bench measures τ_d and f_N** (BP-01 §4). That is the cheapest available improvement in roll stiffness.

### 3.3 Trim

These choices come from a factorial over c, band, trim form and q floor: 22 variants × 120–180 dispersed flights.

| Parameter | Old | New | Reason |
|---|---|---|---|
| `CTRL_TRIM_BAND_K` | 1.1 | 0.3 | 1.1 was sized for c = 70. At c = 15 the band ignored about 45 °/s at max q. |
| `CTRL_TRIM_QSCHED` | — | 1 | The trim state is now torque (q̂·δ_trim), so a constant friction torque is cancelled at every q. Coast roll RMS p95 fell from 60 to 41 °/s in the sweep. |
| `CTRL_ROLL_Q_MIN` | 2000 Pa | 1000 Pa | Lowers the rail-exit roll peak. |
| `CTRL_TRIM_CLAMP` | ±6° | ±6° (kept) | ±9° made no difference (p95 RMS 39.7 vs 41.3 °/s with worst-case friction), so the damping travel is preserved. |
| `CTRL_TRIM_RATE`, `CTRL_TRIM_KI` | 3 °/s, 0.0234 | kept | Raising Ki destabilised the loop. Raising the rate alone changed nothing. |

### 3.4 Large-error gain boost

`CTRL_ROLL_BOOST_K 4`. Above |p| = 300 °/s, Kp ramps up to 4× over a further 300 °/s.
- Nominal flights never reach that rate: p99 160 °/s, maximum 236 °/s.
- In a canard hard-over it drives the healthy canards to oppose at once. Hard-over roll peak: p95 920 → 600 °/s, maximum 1 240 → 830 °/s.
- 8× was worse because it sits at the delay limit.

### 3.5 Dither

The roll command carries the MATLAB `P.Dither`: a square wave of ±`CTRL_DITHER_AMP_RAD` = 1.5 × dead band (0.00294 rad, 0.17°), +a +a −a −a at the frame rate, which is **50 Hz** at 200 Hz.
- It is added after the trim saturation test, so it never winds the trim.
- It is reset when control goes OFF. Shadow mode logs it but does not apply it.
- The MATLAB design used 125 Hz, which a 200 Hz servo frame cannot carry (Nyquist 100 Hz). The `P.Dither` note already named 60–80 Hz as the fallback for a capped frame rate.
- At the servo's 12.7 Hz (assumed) bandwidth, the surface follows about 6 % of the 50 Hz dither.
- `test_dither.c` (3 tests): wave shape and zero mean, no trim wind-up, reset on OFF.

MC-01 A/B over 600 matched flights with build tolerances:

| | Coast roll RMS p50 / p95 / max | Coast peak p95 | Canard max p95 |
|---|---|---|---|
| Dither off | 14.9 / 29.3 / 52.6 °/s | 107 °/s | 2.84° |
| **Dither 50 Hz (flown)** | **10.9 / 22.2 / 43.7 °/s** | **74 °/s** | 2.81° |
| Dither 100 Hz | 10.9 / 22.2 / 43.8 °/s | 75 °/s | 2.81° |

The benefit depends on the servo's real dead-band behaviour, which is still unmeasured. The servo bench (BP-01 §4) should repeat this A/B on the real servo.

### 3.6 Sources of truth

- The firmware constants live in `board.h` with `#ifndef` guards, so the SITL can sweep them with `-D`.
- `parameters_initialise_roll_control.m` now has Ixx = 2.2e-4, c = 15 and a 50 Hz dither. `sim_params.json` is re-exported from it.
- MATLAB `controller.m` does not yet have the torque-space trim or the boost. MC-01 therefore verified them on the C flight code, not in Simulink.

## 4. Flight log: pre-trigger ring and write-behind queue

**Decision:**
- **Armed in READY.** A 256-record (2.56 s at 100 Hz) ring in RAM. It shares the 96 KB firmware-update staging buffer, which is only used on the pad; `cmd_fw` disarms the ring first. No flash is written.
- **Committed at liftoff.** The BOOT record is written, then the ring drains oldest-first at 2 records per frame (400/s against 100/s production), so a full ring clears in 0.85 s.
- Once committed the queue **never overwrites**: a record that does not fit is dropped and counted (AUX `dropped`, EVENT `arg1`).
- Drain first, then this frame's record. That order is pinned by `test_logq.c`.
- **Phase-scheduled rate:** 100 Hz through READY, BOOST and COAST; 20 Hz in DESCENT and LANDED (FDS-01 §7.1).
- Closed 60 s after LANDED, which also covers a premature LANDED.
- **Format v2.** STATE adds h, v_up, q̂, roll trim and LOGF_SHADOW/LOGF_PRETRIG; gyro bias moves to a 1 Hz AUX record. `fc_pull.py` decodes v1 and v2, keyed on the BOOT version byte, and writes `sessionN_aux.csv`; the decoder round-trip test passes.
- **Flight builds** (`CFG_FLIGHT_BUILD=1`) cannot reach READY with less than 200 KB of erased log (`FlightIn.ready_inhibit`). Bench builds warn only.
- **Pure module:** `svc/logq.c`, with 4 host tests (order, no overwrite once committed, drain outpaces production with no loss, reset).
- **Hardware HITL:** "log armed (pre-trigger)" at READY and "log committed at liftoff" at BOOST in every run.

Flash timing per RM0383: 16 µs typical per word, CPU stalled, 12 words per record. Two records per frame cost about 0.4 ms typical and 2.4 ms worst. The erase (1–2 s per 128 KB sector) is still only ever done on command.

**Residual:** the ring does not survive an in-flight reset (the F411 has no backup SRAM). The H743 has 4 KB of backup SRAM; put the ring there (§9).

## 5. Landing detection

The old rule needed GPS (|v_up| < 1.5 m/s, GPS-aided) or the 300 s timer. It is replaced by three rules, any of which declares LANDED, all only once t_DESCENT ≥ 15 s:

1. **GPS rule** (unchanged).
2. **Impact shock then stillness.** |f| > 3 g latches touchdown. The stillness statistics restart after the shock, then 2 s of stillness is required.
3. **Stillness alone for 30 s.**

Stillness means EWMA (τ = 0.5 s) sd(|f|) < 5 mg and mean |ω| < 2.9 °/s and |mean |f| − g| < 1 m/s². The measured floors are 0.6 mg and 8 mdps/√Hz. The 300 s timer remains.

Two SITL findings drove the details:
- 10 s of stillness fired under a calm canopy, so it became 30 s.
- A deployment shock after a motor delay latched "touchdown", so the minimum became 15 s.

MC-01: R4a "never early" passed in 7 680 of 7 680 flights, and in 3 840 of 3 840 in the LD-01 re-run with corrected mass properties. HITL `gps_none` now lands at 33.8 s on the board clock instead of the 300 s timer.

## 6. Flight core extracted; SITL

- `Core/hyades/mission/flight.c` is now the single per-frame pipeline: estimate → monitors → detectors → arbiter → control. `app.c` calls it on the target; `Tools/sitl/sitl` links the same file on the host.
- Ordering rules from earlier HITL defects are preserved and documented in `flight.h`.
- Shadow mode (`CFG_CTRL_SHADOW`): the law runs and is logged, and the canards are held at 0.
- The SITL plant and Monte Carlo are specified in MC-01.

## 7. Navigation and detector changes

| Change | Why | Test |
|---|---|---|
| NAV saturation budget: `NAV_SAT_BUDGET_N` = 4 frames (20 ms) of clipping before q̂ is invalidated. Hidden velocity error is bounded at 4 × 5 ms × 2 g ≈ 0.4 m/s. | A single clipped frame (vibration peak on a 13.7 g motor) sent 2 % of G104T flights to the apogee timer, 7 s late. | `test_saturation_budget` |
| `DET_T_APOGEE_MAX_S` 15 s → 11.5 s, to be set per motor (MC-01 §6 table) | 15 s was sized for the I205W. | MC-01 R3 100 % |
| `DET_T_LAND_MIN_S` 5 s → 15 s | A deployment shock must not latch touchdown. | SITL `chute_delay=8` |
| Three state machine fixes (STM-01 F1–F3) | Hand verification. | `test_flight.c`, 2 new tests |

## 8. Verification summary

| Level | Result |
|---|---|
| Host unit tests | 12 suites, **all pass**. New: `test_logq` (4), `test_flight` (7), `test_dither` (3), `test_saturation_budget`; existing tests updated for STM-01 F3 and log v2. |
| ARM build | 0 errors, 0 warnings (−Werror, the full design-guideline warning set). |
| MATLAB cross-check | Matches the Python port (§3.1). |
| SITL vs OpenRocket | G79W apogee 432.7 m vs 439 m (3 m/s wind, 3° rail tilt in SITL); t_apogee 9.59 vs 9.66 s. After the LD-01 CG correction: 431.8 m. I205W 1 291.6 vs 1 303 m. |
| SITL Monte Carlo | Final build including dither, build tolerances met: **1 200 / 1 200 flights pass R1–R9**. Coast roll RMS p95 22 °/s; peak p95 112 °/s. Full campaign set in MC-01. Re-run with corrected mass properties (LD-01 §3): 1 198 / 1 200. |
| Hardware HITL, 8 Nov | Pre-dither build: 9 / 9 cases OK (nominal, wind 9 m/s + gust, vibration nominal and worst, 25 ms boost dropout, NaN sample, no GPS, pad knock, I205W). **Final build with dither** (`campaign-20261108-103621`): 4 / 4 OK. G79W nominal roll peak 95 °/s (was 105), RMS 29 °/s (was 32), canards ≤ 1.15°, q̂ error 0.9 % RMS, loop p99 329 µs. |

## 9. Consequences for hardware (V6 / H743 and the airframe)

1. **Measure before raising c:** servo τ_d and f_N (laser bench), and forward-body Ixx and Iyy (trifilar or bifilar pendulum). BP-01 §4 has the procedures.
2. **Build tolerance on the fin can:**
   - net rear-fin cant |ε| ≤ 0.25°;
   - single-fin misalignment ≤ 0.25°;
   - bearing Coulomb torque ≤ 6 mN m (spin-down test).
   
   MC-01: with these met, 100 % pass (99.8 % after the LD-01 CG correction, with 2 roll-limit corner cases and no lock-in failures). With the wide tolerance (σ_cant 0.25°), 3.4 % cross the lock-in limit (LD-01 re-run).
3. **V6 additions** (BP-01 §5; James, 8 Nov):
   - **adopted:** raw IMU at ≥ 1 kHz to SD; a high-g accelerometer (±200 g, REC-01 §7);
   - **proposed:** pre-trigger ring index in H743 backup SRAM, synchronous servo timers, servo rail sensing;
   - **not adopted:** can spin sensor and servo position/current feedback. Their risks are carried by the build tolerances and the gain boost; hard-over stays a residual hazard.
4. **Linkage ratio n:** in the 6DOF it does not help the roll floor (friction dominates, not the dead band). Keep n = 1 unless the canards are resized.

## 10. Open items

- Re-run `optimise_roll_schedule`, `montecarlo_roll_control` and Simulink with c = 15 and the torque-space trim, so the MATLAB suite matches the flight code.
- ~~Hardware HITL re-run of the dither build~~: done, 4 / 4 OK (§8).
- ~~Flight 1 mode~~: **decided 8 Nov: full rate law** (zero-rate target, no angle hold; `CFG_CTRL_SHADOW 0`).
- Requirements tightened (MC-01 §3): peak ≤ 240 °/s, coast RMS ≤ 45 °/s, lock-in ≤ 0.6. The flight configuration passed 1 200 / 1 200 on 8 Nov. After the LD-01 CG correction it passes 1 198 / 1 200.
- V6: ≥ 1 kHz IMU to SD and a high-g accelerometer adopted (BP-01 §5).
- ~~Phase 2~~: done 8 Nov. Results:
  - LD-01: flight loads, a mass-property correction to the SITL, and the re-run of every MC-01 campaign;
  - REC-01: ejection, cord and parachute sizing.
- **Open:** 2 / 1 200 corner flights miss the tightened R5 with the corrected plant (LD-01 §3.2). Decision pending.
- Set `DET_T_APOGEE_MAX_S` on the pre-flight sheet once the motor is chosen.
