# SW-01 — Flight software data flow (one 200 Hz frame)

**Figure:** `Documentation/Images/adr004/sw01_flight_dataflow.png` / `.svg`. Generator: `sw01_flight_dataflow.py` (same grid style as STM-01).

**Status:** describes FC-V4 firmware as built (Flight 1 configuration: `CTRL_ROLL_LAW 1`, `CTRL_ENABLE_PITCHYAW 0`, dither on). Companion to STM-01 (phase machine), ADR-003 (architecture), ADR-004 (200 Hz loop, roll law).

## 1. Frame schedule (app.c)

| Order | Task | Rate | Does |
|---|---|---|---|
| tick | — | 200 Hz | waits for the deadline. A late start sets `overran`. The deadline advances by exactly one period, with no re-sync. |
| 1 | sense | every frame | `IMU_ReadBody` (boot gyro bias removed) or `hitl_sense`; `GPS_Poll(256)` |
| 2 | flight | every frame | builds `FlightIn` → `Flight_Step` → `PWM_SetFin` ×4 → `PWM_Commit` |
| 3 | log | every frame | drains the LogQ backlog (2 records per frame), then writes STATE at 100 Hz (20 Hz in DESCENT/LANDED) and AUX at 1 Hz |
| 4 | telem | 100 Hz binary or 5 Hz text | `Telem_SendState` |
| 5 | comms | every frame | console or HITL poll, firmware update, log pull, USB |
| 6 | leds | every frame | phase and fault pattern |
| — | watchdog | — | IWDG kicked only if sense and flight both checked in (`CK_ALL`) |
| — | maintenance | on request | log erase only: fins centred, then the frame is re-based |

## 2. Flight_Step order (flight.c)

1. **Estimate:**
   - IEKF predict and accel update.
   - NAV vertical channel (h, v_up, q̂ and validity flags).
   - When an IMU frame is missing in flight, the step bridges on the last specific force, up to `bridge_n` frames.
2. **Monitor and arbitrate:**
   - Protected phase-word check. A corrupt word triggers `Phase_ForceSafe`.
   - Eight FDIR monitors run: imu_read, accel_rng, gyro_rng, loop time (80 % of the frame), overrun, est. NaN, est. tilt covariance, GPS link. Each produces trip or clear events.
   - The operator's fault-clear request is served.
   - Detectors run: liftoff, burnout, apogee, landing. They are skipped under a HARD fault, except in DESCENT and CATO_SAFE.
   - Severity arbitration: HARD → LOCKOUT on the pad or CATO_SAFE in flight; SOFT in READY → demote to SETTLING.
   - Control-mode ceiling: `Phase_ActuationAllowed` (BOOST, COAST; READY only on the bench), then `Fdir_Capability`:
     - no IMU → OFF;
     - estimator lost → RATE_DAMP;
     - otherwise → FULL.
3. **`Ctrl_SetQbar`:** passes q̂, marked valid only if NAV q is valid and the mode is FULL.
4. **`Ctrl_Update`** computes the law output:
   - FULL: δ = −(c/q_e)p with c = 15, plus boost (up to 4×) and torque-space trim (clamp ±6°). The command goes to all four fins in common mode.
   - RATE_DAMP: the same law on the raw gyro, with no trim.
   - OFF: 0.
   - Schedule input: q_e = q̂ if valid, else 40 kPa, with a floor of 1 kPa.
5. **`Alloc_Apply`** (ALLOC-01) turns the law output into the four fin commands:
   - null-space dither (±1.5 × dead band at 50 Hz, top/bottom against left/right), so the canards add no net moment from it;
   - roll-priority saturation.

   `PWM_SetFin` then quantises each command to the 8 MHz PWM tick (0.019° per tick).
6. **Shadow build:** fins are held at 0 while the commands are still computed and logged.

## 3. Outputs

- **Servos:** `PWM_SetFin` adds each fin's calibrated zero (CAL-01: stored in the flash journal, set with the `cal` commands in the ALIGN phase, up to ±90°) in whole ticks. `PWM_Commit` latches all four channels on one timer update, so there is no skew between canards.
- **Events:** each `emit()` produces an EVENT log record and console text.
- **LogQ:** the pre-trigger ring (256 records) is armed in READY and committed at BOOST. The log closes at LANDED + 60 s (see ADR-004).
