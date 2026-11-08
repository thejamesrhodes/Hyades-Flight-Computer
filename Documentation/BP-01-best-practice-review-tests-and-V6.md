# BP-01: best-practice review, required ground tests, and V6 consequences

**Date:** 8 November 2026. Companion to ADR-004, MC-01 and STM-01.

Markers used in this document:
- **[fact]**: from a cited source;
- **[inference]**: my reasoning;
- **[assumption]**: unverified.

## 1. Practices this work now follows

| Practice | Where it comes from | Status in Hyades |
|---|---|---|
| Stability margins: 45° / 6 dB in the worst case, and at least half those under about 20 % derivative uncertainty | SAE AS94900 (successor to MIL-F-9490D), as summarised by NASA ([NTRS 20090036796](https://ntrs.nasa.gov/api/citations/20090036796/downloads/20090036796.pdf)) and Mansur et al., AHS 2009 ([link](https://gcp-web.sjsu.edu/researchfoundation/docs/AHS_2009_Mansur.pdf)) **[fact]** | c = 15: nominal 67° / 10.8 dB; worst corner 40° / 4.8 dB. **Meets.** |
| Launch-vehicle margins: 6 dB / 30° undispersed, 3 dB / 20° dispersed | NASA NESC Technical Bulletin 22-05 ([link](https://ntrs.nasa.gov/api/citations/20220011631/downloads/TB_22-05_Margin%20Reductions_080122v5.pdf)) **[fact]** | **Meets** with margin. |
| The same code runs in simulation and in flight (SIL → HIL → flight) | Standard practice in flight software verification **[inference]** | `flight.c` is shared by `app.c`, SITL and HITL (ADR-004 §6). |
| Thresholds traceable to noise statistics and dispersion analysis; flight plan pre-registered | FDS-01 P6, §4.3 | Detector windows in ms (`MS_TO_FRAMES`); apogee timer from the MC per motor. |
| Monte Carlo with dispersions on every uncertain parameter, an assumption register, and requirements judged on the full set | NASA/TP-2010-216447, *Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Analysis* (listed at [vibrationdata.com](https://vibrationdata.com/rocket.htm)) **[fact: the document exists; method summarised, not quoted]** | MC-01 §2–§4. 0 failures in 1 200 runs gives ≤ 0.25 % per flight at 95 % confidence (rule of three). |
| Pre-trigger ("black-box") logging, write-behind, no erase in flight, phase-scheduled rate | FDS-01 §7 | ADR-004 §4. |
| Digital control: synchronise actuator update to the control frame; budget every delay | Classical sampled-data control **[inference]** | ADR-004 §2: delay cut from 27.7 to 10.4 ms. |

## 2. Methods considered for replacement or upgrade

| Current | Candidate | Verdict |
|---|---|---|
| q-scheduled P + trim (MATLAB Mode 1) | INDI (incremental nonlinear dynamic inversion) | **Not now [inference].** INDI needs the control effectiveness and a clean angular acceleration. The fixed-effectiveness version limit-cycled in HITL (HITL-01). With q̂ scheduling of the effectiveness it becomes attractive for the 3-axis H743 work. |
| Trim integrator | Disturbance observer (DOB) or extended-state observer (ADRC) estimating the friction torque | **Good next step [inference].** The friction torque is the dominant disturbance (MC-01 FM4). A DOB uses the same plant model as the q-schedule. Worth doing once Ixx and Cl_δ are measured; the torque-space trim already captures part of the benefit. |
| Persistence-counter detectors | SPRT / CUSUM | Planned (FDS-01 §5.3). The persistence counters met every MC requirement, so it is not urgent. |
| Velocity from integrated acceleration (NAV-01) | Thrust-curve prior + drag model (UKF) | Only if a motor beyond 16 g is flown. The clip budget now covers the shortlist. |
| Python/MATLAB linear optimiser | Structured H∞ or μ-synthesis | **Not needed [inference]** for one SISO loop. A disk-margin check is a cheap addition to `rollopt.py`. |
| Hand-written dither square wave | Sigma-delta or random dither | **No.** Matched performance at 50 and 100 Hz in the SITL; the square wave is deterministic and testable. |

## 3. Vibration: general across motors, or motor-specific?

### Short answer

Both. The **shape** of the spectrum at the IMU is set mostly by your airframe and mount: structural modes and the transfer path from the motor case to the flight computer. The **level** and some spectral content come from the motor: thrust, propellant formulation, and combustion roughness or chuffing.

So a measurement with one motor in your airframe tells you a lot about another motor in the same airframe, but not everything.

### Evidence

- NASA-HDBK-7005 (*Dynamic Environmental Criteria*, 2001) devotes sections to **extrapolation techniques** for predicting vibration from a reference vehicle or configuration **[fact]**: [vibrationdata.com copy](https://www.vibrationdata.com/tutorials2/NASA7005.pdf).
- NASA MSFC's vibroacoustic data-bank method scales a reference spectrum to a new structure by **dynamic similarity**, including mass ratios **[fact]**: Ferebee, [NTRS 19820012364](https://ntrs.nasa.gov/api/citations/19820012364/downloads/19820012364.pdf).
- The point for Hyades **[inference]**: the structural path dominates the spectral shape, so a smaller motor in the **same** airframe gives the right shape. The level then scales with the forcing, as a first-order assumption with the RMS proportional to thrust. Different propellants (for example Blue Thunder against White Lightning) can add content that does not scale.

### What to do

1. **Tap / modal test of the assembled airframe** (free hanging, IMU logging, a light impact hammer or knuckle tap at the motor mount). This finds the structural modes. Cost: nothing.
2. **Static-fire your smaller motors in the real fin can and motor mount**, with FC-V4 in the avionics bay and the IMU in capture mode.
   - Run the capture at the highest ODR the I²C link sustains (896 Hz) with the IMU LPF bypassed, or at least widened, so you see the content that the flight LPF later removes.
   - Also log the ±16 g clip flags. **The flight log at 100 Hz after the 60 Hz LPF cannot see vibration.**
3. Build a PSD envelope (Welch, 50 % overlap). Scale it to the chosen motor's thrust and margin it per NASA-HDBK-7005 practice (a P95/50 envelope plus a margin).
4. **Replace assumption A24 in MC-01** with the measured envelope and re-run. The clip budget (ADR-004 §7) and the LPF corner are the parameters it might move.
5. Flight 1 gives the real environment, but only with a high-rate capture. **[V6]** Log raw IMU at ≥ 1 kHz to SD for the boost phase.

## 4. Ground tests the analysis now depends on

### 4.1 Servo bench (laser rig, already planned; `SERVO_BENCH.md`)

**Measure:**
- τ_d and f_N/ζ (small-amplitude chirp; a large amplitude biases f_N low);
- dead band (slow ramp, look for the first movement);
- lash;
- the step response at 6.0, 7.4 and 8.4 V.

**Then:** re-run `adr004_margin_check.m` and MC-01 with the measured ranges. If τ_d ≤ 14 ms and f_N ≥ 80 rad/s are confirmed with ±20 % bounds, c can rise to 18–20, giving about 25 % stiffer roll.

**Repeat the dither A/B on the real servo:** a sine of the roll command with the dither on and off, measuring the surface with the laser.

### 4.2 Inertia (trifilar and bifilar pendulum)

- **Ixx, forward body, can removed:** trifilar, three equal threads at radius R and length L. I = m g R² T² / (4π² L), where T is the measured period and m the body mass; calibrate the platform first and subtract it.
- **I_t, transverse, whole vehicle with the motor case:** bifilar.
- **Target accuracy ±5 %.** MC-01 currently spans −32 % / +60 % on Ixx.

### 4.3 Fin-can bearing friction (spin-down)

1. Spin the assembled can by hand to about 5 rev/s and film it at 240 fps, or use a magnet plus hall sensor and the FC.
2. Fit ω(t). A linear decay means Coulomb friction: T_c = I_can |dω/dt|. An exponential decay means viscous friction.
3. **Requirement: T_c ≤ 6 mN m** (MC-01 §6). Measure at 0 °C and 20 °C if possible, since grease viscosity varies.

### 4.4 Rear-fin alignment

A jig plus a laser on each fin, and a spin test in a fan airflow:
- **net cant ≤ 0.25°;**
- **single fin ≤ 0.25°.**

The can-spin / pitch-frequency ratio is 1.13 × |cant in degrees| (MC-01 fit, `mc_lockin.png`).

### 4.5 Canard zero (Cl₀)

Laser zeroing to ≤ 0.1°, which you were already planning. A fixed laser alignment error does not matter for the servo statistics, but it **does** matter for the absolute zero.

## 5. Consequences for the V6 / H743 hardware

**James, 8 Nov: V6-3 and V6-5 are adopted as V6 requirements.** V6-1 (can spin sensor) and V6-2 (servo feedback) are not adopted. Their risks are carried by the build tolerances (MC-01 §6) and the large-error gain boost; hard-over stays a residual hazard.

The numbers are V6 requirements derived from this analysis **[inference]**; check them against the V6 schematic.

| # | Change | Why |
|---|---|---|
| V6-1 (not adopted) | **Can spin sensor**: a magnet on the fin can and a hall sensor on the body ring, about 1 timer input. | Gives the slip rate, so friction-torque feedforward becomes possible (MC-01 FM4) and lock-in can be monitored directly (R6) instead of inferred. |
| V6-2 (not adopted) | **Servo position feedback or current sense per channel.** Use servos with feedback, tap the pot, or fit INA-type current shunts. | Hard-over detection (MC-01 FM9, the only failure the loop cannot contain). It also confirms polarity on the pad (FDS-01 §6.6). |
| V6-3 **(adopted)** | **IMU log at ≥ 1 kHz to SD** for boost and coast, and **through apogee + 5 s** to capture deployment (REC-01 §7). | The vibration environment (§3), replay of the estimator, and the recovery shock (15–20 ms snatch pulses). |
| V6-4 | **Pre-trigger ring in backup SRAM** (4 KB is too small for 2.5 s at 100 Hz × 48 B = 12 KB). Use the backup SRAM for the ring index and last-flush marker, with the ring in DTCM, or log fewer bytes. | Keeps the pre-trigger data across an in-flight reset. |
| V6-5 **(adopted)** | **High-g accelerometer, ±200 g**, sampled at ≥ 1 kHz in the deployment window, alongside the ±16 g IMU. | REC-01: snatch up to 156 g with the current Kevlar cord (37 g with the recommended cord); landing p99 49 g (LD-01). Also covers motors beyond the shortlist. |
| V6-6 | **Synchronous servo timers** (one advanced timer for all four channels, with the update event started by software as on FC-V4). | Keeps the ADR-004 delay budget. |
| V6-7 | **Servo rail sensing** (ADC on the 2S rail under load). | Servo slew scales with voltage (MC-01 A21); a monitor instance can flag sag. |
| V6-8 | **200 Hz frame minimum.** The H743 can run 400–500 Hz with SPI DMA IMU reads. Keep the servo frame ≤ 333 Hz. | More rate buys little once the 14 ms servo delay dominates (ADR-004 §2). |

## 6. Hidden-flaw checklist (what was looked for)

Each item says what was looked for, then the outcome.

- **Stale parameters marked "STALE" or "MEASURE" in the MATLAB file:** Ixx was wrong by 3×. **Fixed.** Others are listed in MC-01 §2.
- **Units and rates that silently depend on FRAME_HZ:** all persistence counts were in frames. **Fixed** (`MS_TO_FRAMES`).
- **Actuator update not synchronous to the control law:** 50 Hz async PWM. **Fixed.**
- **Two copies of the same parameter:** MATLAB, `board.h` and `sim_params.json` all now carry c = 15, Ixx and the 50 Hz dither. The torque-space trim and boost exist only in C; MATLAB `controller.m` still to be updated.
- **Single-sample latches:** NAV saturation. **Fixed** (budget).
- **Detectors that can fire on a benign look-alike:** calm canopy, deployment shock. **Fixed.**
- **Terminal states reachable from flight:** LOCKOUT on a corrupt word. **Fixed** (STM-01).
- **Test flakiness:** `test_tools.py::test_imu_capture_to_noise_fit` is stochastic (fails about 1 run in 3 at its ±15 % tolerance). It predates this work; give it a fixed seed.
