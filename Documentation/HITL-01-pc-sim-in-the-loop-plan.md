# HITL-01: PC-simulation-in-the-loop plan for FC-V4

**Status:** H0, H0b and H1 done and verified on hardware, 7 November 2026. H2 detectors are done in persistence form; the SPRT upgrade is still open. H3–H5 are open.
**Applies to:** `Firmware/FC-V4` (F411 bench board), carried over unchanged to the H743.
**Chosen form (James):** a PC simulation in the loop. Servo-in-the-loop and a spin rig come later.

**Code:**
- Firmware: `Core/Src/hitl.c/h` (seam), `app.c` (lockstep, reply), `board.h` §11.
- Host: `Tools/fc_hitl.py` (driver), `Tools/hitl_model.py` (plant), `Tools/hitl_crosscheck.py`.
- MATLAB: `Simulation/Matlab Roll Tuning/export_sim_params.m`, `hitl_crosscheck.m`.
- OpenRocket exports: `Simulation/or_sim1_G79W.csv`, `or_sim2_I205W.csv`, from `62mm Canard Control V1.0.0.ork`.

## Goal
The real flight firmware runs on the real MCU, and a simulation on the PC replaces the physical world. The board receives simulated IMU samples and GPS NMEA, runs its unmodified GNC, NAV, FDIR and phase logic at 100 Hz, and returns its fin commands. The simulation applies those commands to the vehicle.

## As built

**Seam (`CFG_HITL = 1`).**
- Dormant until the first 0x10 SAMPLE frame arrives; until then the board runs normally on the real IMU.
- Once live:
  - each frame waits for one sample, bounded by `HITL_WAIT_US` = 20 ms, below the ~100 ms IWDG timeout;
  - a missing sample is a failed IMU read, so `imu_read` trips as it would for a dead sensor;
  - the first sample re-seeds the IEKF and NAV;
  - GPS bytes come from 0x12 frames, and USART2 is drained and discarded;
  - a 0x11 REPLY goes out after every frame;
  - `reboot` is accepted even in a flight phase, because the flight is simulated.
- Console text keeps working. The RX demux strips frames and passes all other bytes to the line editor.
- Flight builds:
  - `-DCFG_FLIGHT_BUILD=1` is a compile error unless `CFG_HITL`, `CFG_FAULT_INJECTION` and `CFG_BENCH_ACTUATE_READY` are all 0;
  - with `CFG_HITL = 0`, no `Hitl_*` symbol is linked;
  - the seam costs 1.25 KB of flash and 160 B of RAM.

**Protocol.** `0xAA55 id len payload crc16`, the same framing as telemetry.

| Direction | ID | Size | Contents |
|---|---|---|---|
| Host → board | 0x10 SAMPLE | 32 B | idx, accel[3] (m/s²), gyro[3] (dps), flags |
| Host → board | 0x12 NMEA | ≤ 96 B | raw NMEA bytes |
| Board → host | 0x11 REPLY | 60 B | idx, phase, mode, faults, loop_us, nav flags, missing flag, fin[4], q[4], h, v_up, speed, q̂ |

**Plant (`hitl_model.py`).**
- **Translation:** 3-DOF in ENU. Thrust, mass, roll inertia, Cd(Mach), rail length and descent rate come from the OpenRocket CSV.
- **Atmosphere and wind:** ISA at the site elevation, constant wind.
- **Pitch/yaw:** a first-order weathercock toward the air-relative velocity (τ = 0.08 s). This stands in for the stable short period; it is not a pitch/yaw model.
- **Roll:** dynamic, using the Simulink `Plant` block equations. Mach comes from local sound speed (MATLAB uses V/343).
- **Actuator:** a port of `canard_servo_reduced.m`.
- **Parameters:** from `sim_params.json`, exported by MATLAB.
- **IMU:**
  - measured noise: ARW 7.9 mdps/√Hz, VRW 72 µg/√Hz;
  - 30 Hz first-order low-pass at 224.2 Hz ODR;
  - 0.02 dps residual gyro bias;
  - 16-bit quantisation and ±16 g / ±2048 dps saturation;
  - the latest sample per frame;
  - an optional accel scale-factor error.
- **GPS:** 1 Hz GGA+RMC with noise and a random walk. Lock is lost above 4 g and reacquired after 5 s.
- **Touchdown:** decelerates over 0.1 s, so the board sees the impact.
- **Site:** MRC, near Twycross: 52.64 N, 1.53 W, 110 m. All three values are approximate and unverified.

**Runs.** Each run reboots the board, holds on the pad until READY (at least 10 s), ignites, and stops 15 s after landing. Outputs go to `Tools/logs/hitl/`: a CSV of truth and board per frame, a JSON summary, and the board text.

## Results (7 November 2026)

| Package | Exit criterion | Result |
|---|---|---|
| **H0** interface + seam | 10⁵ frames against a static host with no missed frames or overruns; flight build unaffected | **Met.** 100 000 frames, 0 reply timeouts, 0 missing samples, 417 frames/s (4.2× real time). Loop max 314 µs in the soak and 456 µs in flight runs, against a 10 ms frame. `CFG_HITL = 0` builds with 0 warnings and links no HITL code. (Byte-identical was dropped as a criterion because NAV-01 changed the flight code in the same step.) |
| **H0b** Python plant | Matches the MATLAB model within 2 % RMS | **Met, exactly.** `hitl_crosscheck`: servo RMS difference 2×10⁻¹⁷ rad, roll rate 2×10⁻¹⁴ rad/s over 7000 steps (step, step, 0.5–20 Hz chirp). The reference is the code the Simulink model executes (`canard_servo_reduced.m` plus the `Plant` block, run as fixed-step Euler), not a `sim()` call. Plant vs OpenRocket, fins fixed: I205W apogee 1296 m vs 1303 m (−0.5 %), v_max 267.9 vs 267.0 m/s, a_max 294 m/s² vs 294; G79W apogee 436 m vs 439 m, t_apogee 9.60 vs 9.66 s. |
| **H1** open-loop estimator | Attitude and NAV vs truth | **Met for G79W** (see NAV-01): q̂ −0.9 % mean, apogee height within 0.4 m, all four phase edges detected. IEKF tilt (nose-axis) error vs truth: 0.12° mean, 0.15° max through boost and coast; 0.35° mean, 3.3° max under the parachute. **I205W fails by design:** 30 g saturates the IMU (see the NAV-01 motor constraint). |
| H2 phase detectors | Correct sequence, nominal and dispersed; no false liftoff in a 30 min pad hold with handling | Nominal sequence correct for both motors. The persistence form is implemented; SPRT, dispersions and the handling test are still to do. |

**Defects found and fixed by HITL:**
1. **Liftoff lost to demotion.** At liftoff on the I205W, `accel_rng` (SOFT) tripped in the same frames as the liftoff detector. The arbiter demoted READY → SETTLING first, and the board flew the whole flight in SETTLING with control off. Detectors now run before the demotion.
2. **GPS gated out after saturation.** After a saturated boost, the NAV-01 GPS gate rejected every descent fix (13 km error). GPS now re-anchors when the solution is saturated, or after 5 consecutive rejections.
3. **Stuck in DESCENT.** A board left in DESCENT with a latched HARD fault could never reach LANDED: DESCENT has no CATO edge, and detection was blocked by the HARD fault. It refused reboot and firmware updates until power was cycled. Landing detection now runs despite a HARD fault, and HITL builds accept `reboot` while live.
4. **Bug in the MATLAB parameter file.** `parameters_initialise_roll_control.m` line 24 was a comment line missing its `%`, so the file did not parse and every MATLAB script that loads P failed. Fixed.

**H3 preview: closed loop with the current placeholder gains.** G79W with `--closed` and Cl₀ = 0.002:
- Open loop, Cl₀ = 0.002 drives about 250 dps of steady roll.
- Closed loop, the 1-second-window mean roll rate is 6–150 dps, but the roll limit-cycles with peaks of 720 dps.
- **Cause:** the firmware's INDI effectiveness is fixed at 4 × `CTRL_EFF_ROLL` = 160 rad/s² per unit fin. The plant's value is k_b·q̄·30°, which is 270 rad/s² at 2 kPa, 820 at 6 kPa and 5500 at 40 kPa.
- The underestimated G makes the INDI gain 1.7–34× too high across the envelope. This is the expected result of unsized placeholders, and it is exactly what H3 is for.

## Work packages

| # | Package | Status / next step |
|---|---|---|
| H0 | Interface + seam | Done |
| H0b | Python plant | Done. To do: a `sim()`-level Simulink check, once the model logs its outputs |
| H1 | Open-loop estimator | Done for G79W; dispersions (wind, scale factor, seeds) to be scripted |
| H2 | Phase detectors | Persistence form done. To do: SPRT, a dispersed set, and a 30 min pad hold with handling |
| H3 | Closed-loop roll | **Next.** Schedule the INDI roll effectiveness on q̂: G_roll(q̂) = k_b·q̂·`FIN_DEFLECT` with k_b = A·d·C_lδ/I_xx from P, or switch the roll channel to the MATLAB `Kp = c/q̂` damper. Clamp at `NAV_Q_SPEED_MIN`. Then run the 50-run CDS-01 Monte Carlo through HITL |
| H4 | FDIR campaign | Use the SAMPLE flags for injections: per-frame read failure is already supported; NaN and saturation come from the host |
| H5 | Timing/latency | Real-time mode, not built yet. The lockstep loop currently suppresses overrun counting |

## Prerequisites still open
- `REMAP_QMI` check on the airframe (roll sign, nose-up |a|). The HITL sign convention assumes positive mean fin gives positive roll acceleration, matching the firmware's G.
- Servo f_N, ζ, delay and backlash (`SERVO_BENCH`, laser rig).
- `Ixx` and mass from CAD. Note that `P.Vehicle.Ixx` = 7.0×10⁻⁴ is still marked stale; OpenRocket gives 4.7–5.0×10⁻⁴ kg·m², and the HITL plant uses the OpenRocket value.
- Motor choice within the 14 g limit.
- MRC site coordinates and elevation.
- UKRA §8.3 (400 ft) ruling. Because NAV-01 now provides h, a gate is a config-level change.

## Not in scope
Magnetometer fusion, pitch/yaw attitude hold (roll only, as CDS-01), the H743 drivers.
