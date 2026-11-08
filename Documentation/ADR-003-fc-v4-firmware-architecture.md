# ADR-003: FC-V4 (STM32F411) firmware architecture and bench verification

**Status:** accepted 3 November 2026; **roll gains, loop rate, logging and landing superseded by ADR-004 (8 Nov)**; updated 7 November 2026 (single-port GPS, IMU noise capture, NAV-01, HITL-01, q̂-scheduled roll law, HITL-02 worst-case campaign). **Code:** `Hyades-Flight-Computer/Firmware/FC-V4`.

## Context

FC-V4 is the bench and HIL test-bed:
- STM32F411CEU at 96 MHz;
- QMI8658A IMU on I2C;
- ATGM336H GNSS;
- 4 fin servos;
- no barometer, magnetometer or SD card.

The STM32H743 (V6) is still a CubeMX skeleton. The aim is to reach the H743 with the GNC, mission and FDIR code already written and tested, without carrying F411 specifics across.

## Decisions (confirmed with James)

1. **Portable core plus platform layer.**
   - `Core/hyades/{math,gnc,mission,svc}` is pure C: no HAL, no I/O, no globals. It is unit-tested on the host.
   - `Core/Src` holds the F411 platform layer.
   - `main.c` is CubeMX-generated and only calls `App_Run()`.
2. **Core skeletons now; no pyro or SD yet.**
   - Rate-group task table with WCET per task.
   - Phase FSM (FDS-01 §3.1): 16-bit codes with pairwise Hamming distance ≥ 6, stored with their complement.
   - Table-driven monitor engine (FDS-01 P2), with the arbiter in `app.c` (P3).
   - Degraded-mode ladder: FULL (SO(3)+INDI) → RATE_DAMP (gyro only) → OFF.
   - The IWDG is kicked only when all mandatory tasks have checked in.
3. **Telemetry.**
   - 5 Hz text status, or the 0xAA55 binary STATE frame, selected at compile time.
   - Health bits for mode, fault and lockout were appended, so the existing format stays compatible.
   - The text status now ends with `h= vz= q=` (NAV-01). The fields are optional in the parser.
4. **USB-only workflow.** This ports the GMWM "Sheppard" approach.
   - A `fw` console command stages the image in SRAM, checks CRC-32 and the vector table, then reprograms flash from a RAM routine. It erases only the app sectors (0–4, 128 KB) and stretches the IWDG first.
   - Flight log: append-only 48-byte CRC'd records in flash sectors 5–7 (384 KB, about 160 s at 50 Hz). The log survives reflashing and is erased only on command.
   - Host tools use only pyserial (plus Tkinter for the GUI, and numpy for noise and HITL).
5. **Housekeeping.**
   - Removed the legacy LSM303/L3GD20 backend.
   - V4 is under git.
   - Unity host tests: 73 as of 7 Nov. On 3 Nov there were 55; an earlier draft of this ADR said 58, which was a miscount.
   - A check that the Python decoder reads records packed by the firmware.
6. **Operator fault clear (added 3 Nov, James's choice).**
   - `faults clear` (console F8) works on the pad only, and is refused while a HARD monitor is still failing.
   - It moves LOCKOUT to SETTLE through `Phase_Recover`, so READY needs the full fault-free hold again.
   - Without it, LOCKOUT lasts until a reboot.
7. **GPS on a single port (James, 3 Nov).**
   - USART2 only (RX PA3) at 9600 Bd.
   - A boot line probe reports levels, edges and estimated baud. There is no runtime baud hunt.
   - `gps raw` and `gps probe` are available for diagnosis.
8. **NAV-01 and detectors (7 Nov).** See NAV-01.
   - Strapdown v/h and q̂, with the gravity reference measured on the pad.
   - GPS altitude aiding after apogee only, with re-anchoring.
   - Liftoff, burnout, apogee and landing detectors. These run before the READY demotion, and landing is allowed despite a HARD fault.
9. **HITL seam (7 Nov).** See HITL-01.
   - `CFG_HITL` lockstep seam with 0x10/0x11/0x12 frames.
   - Compiled out at 0.
   - A `CFG_FLIGHT_BUILD` compile guard rejects any bench feature left on.
10. **Roll law scheduled on q̂ (7 Nov, closes H3).** `CTRL_ROLL_LAW = 1` replaces the placeholder fixed-effectiveness INDI roll channel, which limit-cycled at 720 °/s in HITL.
   - The law is the MATLAB `controller.m` Mode 1: δ = −(c/q̂)·p + bounded trim, with c = 70.
   - Trim settings: Ki 0.0234, band 1.1, deadband 0.00196, clamp ±0.1047 rad, rate 0.0524 rad/s. Travel is ±0.2094 rad.
   - When q̂ < 2 kPa, or NAV is saturated or invalid, the law uses a 40 kPa fallback gain.
   - RATE_DAMP uses the same law with trim off. Trim resets on OFF.
11. **NAV dropout bridging.** During flight, up to `NAV_BRIDGE_N` = 8 missing IMU frames are integrated with the last specific force. This cut a boost-dropout q̂ error from 12.7 % to 0.9 %.
12. **Liftoff velocity gate.** Liftoff needs either 10 frames above threshold with v_up > 8 m/s, or 30 frames of acceleration alone. A 6 g / 60 ms pad knock no longer triggers it.

## Host tools

`Tools/fc_console.py` is a single-window GUI that replaces Tera Term, `fc_flash.py` and `fc_pull.py` for bench and field use. The CLI tools remain for scripting.

- **Structure.**
  - `fc_link.py` holds everything that is not GUI:
    - the link thread, which owns the port and reconnects by VID:PID after a reset or reflash;
    - the stream demux;
    - the parsers;
    - `fault_summary`.
  - `fc_sim.py` is a protocol-accurate simulated board (`--sim`).
  - `fc_hitl.py` and `hitl_model.py` are the HITL host and plant.
- **Design rules followed:** ISO 9241-110 dialogue principles, Nielsen's heuristics, WCAG 2.1 contrast and use of colour. See the 3 Nov detail below.
  - **Header.** It never changes with the tab. It shows link, phase, control mode, faults (most severe first, never "none" in LOCKOUT/CATO), data age and the running build.
  - **Tabs.**
    - Console.
    - Dashboard: the large readouts, the 8 monitors and the event list.
    - Plots: attitude, body rates, fins and NAV.
      - All plots use fixed scales and freeze with Space.
      - Body rates default to ±500 dps. Clicking a chart cycles 100/500/2000 dps.
      - Off-scale samples are marked with a triangle on the rail. The body-rate scale was hard-coded to ±100 dps until 7 Nov, which flattened real peaks.
  - **Actions.**
    - Build (F4, headless CubeIDE).
    - Flash (F5).
    - Pull + decode (F6).
    - Erase, Reboot and Clear faults (F8), which confirm first.
    - All of these are disabled in BOOST/COAST/DESCENT, and the firmware refuses them there too.
- **Tests:** `Tests/test_tools.py` has 12 tests.
  - Demux, the parsers and the fault summary.
  - End-to-end runs against the simulator.
  - IMU capture → noise fit.
  - The HITL frame formats, and the plant against OpenRocket.

## Findings fixed during the reviews

- **Single-pose accel calibration absorbed tilt.** Now only the gyro is calibrated at boot.
- **GPS parsing used newlib `strtod`/`strtof`, which allocate from the heap.** Replaced with a bounded parser.
- **NaN fin commands bypassed the clamps.** They now map to 0.
- **The trace(P_θ) monitor false-tripped without a magnetometer.** It now uses tilt variance only.
- **`IEKF_P0_BIAS` was too large.** Reduced from 1e-3 to 1.2e-5 (rad/s)².
- **USB output was gated on DTR.** The port now counts as open on line coding, DTR, or any received byte.
- **`TELEM_DECIMATE` was undefined.**
- **GPS was being read on the wrong UART.**
  - The module is on USART2 (PA3), where the original V4 firmware read it; the 2 Nov refactor had moved it to USART1.
  - Now single-port USART2 with a boot line probe. The baud is taken from the 4th-shortest pulse, not the shortest, after one stray interval misread as 19 200 Bd.
  - The ring buffer went from 512 B to 2048 B.
  - On hardware: NMEA received, no fix indoors.
- **Two boot behaviours that were correct.** The boot control-path check reading zero, and all four fins equal at rest in READY/FULL, are both expected (bench INDI drift); see the 3 Nov notes.
- **7 Nov (HITL), four defects:**
  - liftoff lost to a same-frame SOFT demotion;
  - GPS permanently gated out after a saturated boost;
  - a board stuck in DESCENT after a HARD fault, refusing reboot and firmware updates;
  - `parameters_initialise_roll_control.m` line 24 missing its `%`, so the MATLAB suite did not parse.

  All four are fixed; details in HITL-01.

## Verification on the bench

**3 November 2026:**
- Build: 0 errors, 0 warnings with `-O2 -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat=2 -fno-common`.
- Frame compute: 0.91 ms typical, 1.5 ms worst, against an 8 ms budget, with 0 overruns.
- USB reflash 8/8.
- Log start, stop, pull and decode all worked.
- Fault injection: phase flip → LOCKOUT; IMU loss → HARD trip after 100 ms → LOCKOUT; frame stall → IWDG reset → LOCKOUT.

**7 November 2026:**
- Build: 0 warnings.
- Size: text 69.3 KB of 128 KB; RAM 120.0 of 128 KB (96 KB is the firmware staging buffer).
- `CFG_HITL = 0` builds clean and links no HITL code.
- HITL: 10⁵ lockstep frames with 0 missed. Frame compute in HITL is ≤ 0.46 ms, because the I2C read is replaced by the host sample.
- Detectors and NAV verified for the G79W and I205W profiles (HITL-01, NAV-01).
- **Worst-case HITL campaign: 29 cases** covering vibration, dispersions, injections, GPS faults and I205W (HITL-02; figures in `Images/hitl/`).
  - Nominal G79W: roll ≤ 46 °/s, canard command ≤ 1.1°, q̂ error −0.9 %, apogee detected +0.04 s late.
  - Final build re-verified on hardware: 73 Unity tests and 12 tool tests pass.

## Deviations from the software design guidelines (accepted for the F411 bench board)

- **C6:** the IMU uses blocking HAL I2C with a bounded timeout.
- **A5/C4:** a 5 Hz integer-only `snprintf` status line runs inside the frame, after actuation.
- **D1/D4:** no backup-SRAM fault log.
- **B2:** sustained overruns latch LOCKOUT on the pad but do not force a reset. In HITL lockstep, overrun counting is suppressed; H5 will add a real-time mode.
- **B3:** the USB OTG interrupt stays at NVIC priority 0.
- **Detectors** are persistence counters, a simplification of the FDS-01 §5.3 SPRT.
- **Bench-only switches:** `CFG_BENCH_ACTUATE_READY`, `CFG_FAULT_INJECTION` and `CFG_HITL` must be 0 for flight. `-DCFG_FLIGHT_BUILD=1` enforces this.

## Open items

- Verify `REMAP_QMI` against the airframe (roll sign, nose-up |a|).
- GPS: confirm a fix outdoors, and confirm the MRC site elevation and coordinates.
- ~~H3~~: closed by decision 10. Still to do: re-run `optimise_roll_schedule` with a 100 Hz ZOH and the q̂ error model (HITL-02 combined-worst case shows a 4 Hz limit cycle up to 1124 °/s).
- **Open failure modes from HITL-02:**
  - Vibration clipping inside the IMU is invisible to the firmware: −8.7 m/s velocity error and apogee detected 0.83 s early. Measure the real vibration level.
  - Accelerometer saturation leaves only the backup apogee timer, which fires +2.5 to +7 s late. Add a per-motor timer or a ballistic apogee check.
  - Without GPS, landing is detected only by the 300 s timer.
  - A 10 g / 200 ms pad knock still causes a false liftoff. Add an arming step.
  - At 3σ Cl₀, roll settles at 46 °/s, limited by the trim clamp.
- **Motor choice: peak ≤ 14 g.** The I205W, at 30 g, saturates the IMU. If it is kept, add a q(t) fallback or a higher-range accelerometer.
- Heading variance is unbounded without a magnetometer (resolved on the H743, which has one).
- Port to the H743: replace the `Core/Src` drivers (SPI/DMA IMU, LL timers, MPU/cache).
