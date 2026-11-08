/* ============================================================================
 * board.h  —  Central configuration (single source of truth)
 * Target  :  STM32F411CEU (Cortex-M4F, 96 MHz), FC-V4 board, QMI8658A IMU
 *
 * This file collects every tunable that defines the vehicle/sensor/loop so
 * there is exactly ONE place to change rates, axis conventions, references,
 * gains and telemetry behaviour. Nothing here performs I/O.
 *
 * Conforms to the project engineering standard: fixed-width types, named
 * constants (no magic numbers in the modules), static configuration.
 * ============================================================================ */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>

/* ----------------------------------------------------------------------------
 * 1. BODY FRAME DEFINITION
 *
 *   Body Z  = longitudinal / roll axis, points OUT THE NOSE (up on the pad).
 *   Body X  = pitch axis        (pitch = rotation about +X)
 *   Body Y  = yaw   axis         (yaw   = rotation about +Y)
 *   Right-handed; ZYX (yaw-pitch-roll) Euler extraction in IEKF_GetEuler.
 *
 *   World frame = ENU (East, North, Up). The IEKF quaternion q maps
 *   body -> world (v_world = R(q) v_body).
 *
 *   At rest on the pad (nose up) the body-frame specific force the
 *   accelerometer reports must be (0, 0, +g): see SENSOR REMAP below.
 * -------------------------------------------------------------------------- */

#define BOARD_G_MPS2          9.80665f        /* standard gravity            */

/* World-frame references (ENU). Direction-only updates normalise these. */
#define BOARD_GRAV_REF_E      0.0f
#define BOARD_GRAV_REF_N      0.0f
#define BOARD_GRAV_REF_U     (+BOARD_G_MPS2)  /* specific force up = +g      */

/* Earth magnetic field, ENU, UK ~53 N (BGS geomagnetic model). Unit vector;
 * Up is negative because the field dips downward in the N hemisphere.        */
#define BOARD_MAG_REF_E      (-0.007f)
#define BOARD_MAG_REF_N      ( 0.390f)
#define BOARD_MAG_REF_U      (-0.921f)


/* ----------------------------------------------------------------------------
 * 2. SENSOR -> BODY AXIS REMAP   *** VERIFY ON THE BENCH ***
 *
 * Maps the QMI8658A's native (x,y,z) into the body frame above. It depends on
 * how the board is mounted in the airframe, so it cannot be derived here.
 * Measured so far: board flat, component side up -> sensor +Z reads +1 g.
 * Verification (text status over USB):
 *   a) Airframe nose-up and still: |a| ~ 1 g and RPY ~ (0, 0, *).
 *   b) Rotate positively about the nose (right hand, thumb out the nose):
 *      body w_z must read positive. Repeat for pitch (+X) and yaw (+Y).
 * -------------------------------------------------------------------------- */
#define REMAP_QMI(x,y,z, bx,by,bz)    do { (bx)=(x); (by)=(y); (bz)=(z); } while (0)

/* ----------------------------------------------------------------------------
 * 3. TIMING / CYCLIC EXECUTIVE
 *
 * Single 200 Hz minor frame paced by TIM5 (32-bit, 1 MHz free-running), ADR-004.
 * Raised from 100 Hz (7 Nov): the loop delay budget (ADR-004 §2) is set by
 * frame ZOH + servo PWM resampling + IMU filter; at 100 Hz frame / 50 Hz PWM it
 * cost ~20 ms beyond the servo and left the roll loop with 11 deg of phase
 * margin. The IMU runs at 448 Hz ODR so a sample <= 2.2 ms old is read each frame.
 * Sub-rates are counter-gated off the frame. EVERY persistence count below is
 * written in milliseconds and converted with MS_TO_FRAMES, so changing
 * FRAME_HZ does not silently change a detection window.
 * -------------------------------------------------------------------------- */
#define FRAME_HZ              200u
#define MS_TO_FRAMES(ms)      ((uint32_t)((((uint32_t)(ms) * FRAME_HZ) + 999u) / 1000u))
#define FRAME_PERIOD_US       (1000000u / FRAME_HZ)   /* 10 000 us            */
#define FRAME_DT_S            (1.0f / (float)FRAME_HZ) /* exact 0.010 s        */


/* RUN-state settle: hold fins centred and control inactive for this long
 * after seeding so the IEKF converges before any actuation. */
#define RUN_SETTLE_MS         1000u

/* ----------------------------------------------------------------------------
 * 4. SERVO / FIN OUTPUT (see pwm.c for channel-to-pin mapping)
 * -------------------------------------------------------------------------- */
/* Servo frame = control frame, and each pulse is STARTED by software right
 * after the control law (PWM_Commit, timer update event), so the command
 * latency is the compute time, not up to a full PWM period. The counter period
 * is a little longer than the frame so it never wraps on its own while frames
 * run on time; if the loop stalls (log erase) the servos still get a valid
 * frame every SERVO_PERIOD_US. BMS-127WV+: frame >= 3 ms (333 Hz max, maker
 * data). Was 20000 (50 Hz) until 7 Nov: an average 10 ms of hidden delay. */
#define SERVO_PERIOD_US       6000u    /* free-running fallback period          */
#define SERVO_SYNC_TO_FRAME   1        /* 1 = pulse started each frame (ADR-004) */
#define SERVO_MIN_US          900u
#define SERVO_MAX_US          2100u
#define SERVO_CENTRE_US       1520u    /* fin zero; all fin commands are about this */
#define SERVO_RANGE_DEG       180.0f   /* full mechanical sweep over MIN..MAX   */
#define FIN_DEFLECT_DEG       30.0f    /* +/- command authority about centre    */

/* ----------------------------------------------------------------------------
 * 5. CONTROLLER  —  geometric SO(3) outer loop + INDI inner loop
 *    *** TUNE / SYSTEM-ID ON THE BENCH AND IN SIM ***
 *
 * Outer loop (Lee et al. 2010, integral per Goodarzi et al. 2013): a geometric
 * attitude error e_R, e_Omega on SO(3) produces a DESIRED ANGULAR ACCELERATION
 * wdot_des (body frame). No linearisation, globally valid, setpoint = identity
 * (nose-up). Rate term is the measured (bias-corrected) gyro fed back directly.
 *
 * Inner loop (Smeur et al. 2016, INDI): realises wdot_des incrementally using
 * the MEASURED angular acceleration (filtered, differentiated gyro) and the
 * filtered actuator state, so only the control-effectiveness matrix G is needed
 * (the aero model is otherwise sidestepped). The control-effectiveness inverse
 * IS the allocation (one consolidated operation, not G^-1 then B^+ separately).
 *
 * FIXED effectiveness, NO gain scheduling (by explicit design): G does not
 * scale with dynamic pressure here. Consequence: closed-loop gain varies with
 * q_bar in real flight. Acceptable for the bench/no-flight-phase application;
 * the scheduled-G upgrade is the documented next step.
 *
 * Body-axis convention (board.h 1): X = pitch, Y = yaw, Z = roll(out nose).
 * Gains are per-axis groups: PY = pitch&yaw (lateral), ROLL = roll(longitudinal).
 *
 * Units: wdot_des in rad/s^2. k_R multiplies the (dimensionless) SO(3) error
 * vector; k_W multiplies body rate (rad/s). Both produce rad/s^2.
 * -------------------------------------------------------------------------- */

/* Outer-loop proportional (attitude) and derivative (rate) gains. */
/* Pitch/yaw control disabled for the roll-rate damper (gains 0 AND the
 * INDI pitch/yaw increments removed in control.c). 1 = re-enable. */
#define CTRL_ENABLE_PITCHYAW  0
#define CTRL_KR_PITCHYAW      0.0f    /* attitude stiffness, lateral           */
#define CTRL_KW_PITCHYAW       0.0f    /* rate damping,      lateral            */
#define CTRL_KR_ROLL          0.0f    /* attitude stiffness, roll              */
#define CTRL_KW_ROLL           1.0f    /* rate damping,      roll               */

/* Outer-loop integral on the Lie algebra (Goodarzi 2013). Default OFF (0):
 * bring up with PD first, add integral once the rate loop is trusted. */
#define CTRL_KI_PITCHYAW       0.0f
#define CTRL_KI_ROLL           0.0f
#define CTRL_KI_C              0.0f    /* e_I += (e_R + C*e_Omega)*dt            */

/* Gyro-only rate damper (CTRL_MODE_RATE_DAMP, the fallback when the estimator
 * is unhealthy): fin = -K * rate (rad/s), clipped. 0.2 saturates at 5 rad/s
 * (~290 dps). PLACEHOLDER: size from the roll-tuning sim before flight. */
#define CTRL_RD_GAIN_ROLL      0.20f   /* 1/(rad/s)                             */
#define CTRL_RD_GAIN_PITCHYAW  0.0f    /* 1/(rad/s), used only with PITCHYAW=1  */
#define CTRL_EI_LIMIT          0.30f   /* ||e_I|| clamp (anti-windup)           */

/* Roll law. 1 = the Matlab roll-tuning design (controller.m Mode 1, values
 * from parameters_initialise_roll_control.m / sim_params.json): delta =
 * -(c/q) p + bounded trim. 0 = INDI with the fixed CTRL_EFF_ROLL below, which
 * HITL showed limit-cycles (G is really k_b*q: 270-5500 rad/s^2 per unit fin
 * over the envelope against the fixed 160). */
#define CTRL_ROLL_LAW          1
#ifndef CTRL_ROLL_C   /* SITL sweeps override with -D */
#define CTRL_ROLL_C            15.0f    /* Pa s, Kp = c/q. ADR-004 §3 / MC-01: forward-
                                         * body Ixx 2.2e-4 (fin can decoupled), 200 Hz
                                         * synchronous chain. Nominal servo: PM 67 deg,
                                         * GM 10.8 dB (SAE AS94900: 45 deg / 6 dB). Worst
                                         * corner (Ixx -30 %, tau_d 21 ms, f_N 52 rad/s):
                                         * PM 40 deg, GM 4.8 dB. Raise to 18-20 only after
                                         * the servo bench measures tau_d and f_N. The
                                         * old 70 is UNSTABLE on the forward body. */
#endif
#ifndef CTRL_ROLL_Q_MIN   /* SITL sweeps override with -D */
#define CTRL_ROLL_Q_MIN        1000.0f  /* Pa: schedule floor. MC-01: 1000 lowers the
                                         * rail-exit roll peak (p95 222 -> 212 dps)   */
#endif
#define CTRL_ROLL_Q_FALLBACK   40000.0f /* Pa: q_hat invalid -> lowest gain       */
#define CTRL_ROLL_TRAVEL_RAD   0.2094f  /* P.Servo.Travel, +/-12 deg at the canard */
#ifndef CTRL_TRIM_KI   /* SITL sweeps override with -D */
#define CTRL_TRIM_KI           0.0234f  /* P.Trim.Ki                              */
#endif
#ifndef CTRL_TRIM_BAND_K   /* SITL sweeps override with -D */
#define CTRL_TRIM_BAND_K       0.3f     /* MC-01 (was 1.1, sized for c = 70: at c = 15
                                         * the band would ignore ~45 dps at max q)   */
#endif
#define CTRL_TRIM_DEADBAND_RAD 0.00196f /* P.Servo.Deadband (n = 1)               */
#ifndef CTRL_TRIM_CLAMP   /* SITL sweeps override with -D */
#define CTRL_TRIM_CLAMP        0.1047f  /* P.Trim.Clamp, +/-6 deg                  */
#endif
#ifndef CTRL_TRIM_RATE   /* SITL sweeps override with -D */
#define CTRL_TRIM_RATE         0.0524f  /* P.Trim.Rate, 3 deg/s                    */
#endif
/* Trim parameterisation (ADR-004 §3.3). 0 = deflection integrator (Matlab
 * Trim Mode 2: right for a Cl_0-type disturbance, whose torque scales with q).
 * 1 = torque-space integrator: the state is T = q * delta_trim, so a constant
 * TORQUE (the fin-can bearing friction, ~5-10 mN m) is cancelled at every q
 * without the integrator having to chase 1/q through the coast. */
#ifndef CTRL_ROLL_BOOST_K           /* large-error gain boost (control.c), 1 = off */
#define CTRL_ROLL_BOOST_K      4        /* MC-01: hard-over roll peak p95 920 -> 600 dps,
                                         * max 1240 -> 830; nominal flights unchanged */
#endif
#ifndef CTRL_ROLL_BOOST_P
#define CTRL_ROLL_BOOST_P      5.236f   /* rad/s (300 dps): boost starts here      */
#endif
/* Dither (Matlab P.Dither): square wave on the roll command, amplitude 1.5 x
 * the servo dead band at the canard, period CTRL_DITHER_PERIOD_N frames. */
#ifndef CTRL_DITHER_ON
#define CTRL_DITHER_ON         1
#endif
#ifndef CTRL_DITHER_AMP_RAD
#define CTRL_DITHER_AMP_RAD    (1.5f * CTRL_TRIM_DEADBAND_RAD)   /* 0.00294 rad, 0.17 deg */
#endif
#ifndef CTRL_DITHER_PERIOD_N
#define CTRL_DITHER_PERIOD_N   4u       /* 4 frames = 50 Hz at 200 Hz            */
#endif
#ifndef CTRL_TRIM_QSCHED
#define CTRL_TRIM_QSCHED       1        /* MC-01: coast roll RMS p95 60 -> 41 dps    */
#endif

/* INDI control effectiveness  G : fin deflection [-1,+1] -> body angular accel
 * (rad/s^2).  PLACEHOLDERS — identify from a 6-DOF sim or flight log. They set
 * the increment scale; on the bench (no aero feedback) saturation bounds the
 * fins regardless, but in flight these must be physically right (within the
 * ~+/-30% INDI tolerates). For a cruciform: top/bottom drive pitch(X),
 * left/right drive yaw(Y), all four drive roll(Z). */
#define CTRL_EFF_PITCH        20.0f    /* (rad/s^2) per unit fin, pitch         */
#define CTRL_EFF_YAW          20.0f    /* (rad/s^2) per unit fin, yaw           */
#define CTRL_EFF_ROLL         40.0f    /* (rad/s^2) per unit fin, roll          */

/* INDI synchronisation filter: Omega_dot and the actuator estimate pass through
 * the SAME 2nd-order low-pass so their delays match (Smeur 2016, rule 1).
 * Cutoff below Nyquist (FRAME_HZ/2) and above the closed-loop bandwidth. */
#define CTRL_INDI_LPF_HZ      12.0f

/* Per-fin command clamp (normalised). Pitch/yaw (vehicle stabilising) keep
 * priority over roll on saturation: roll common-mode is shed first. */
#define CTRL_FIN_LIMIT         1.0f

/* ----------------------------------------------------------------------------
 * 6. INVARIANT EKF  —  noise / gates (datasheet-derived; see iekf.c header)
 *
 * Left-invariant attitude + gyro-bias EKF (Barrau & Bonnabel 2017). For the
 * attitude-only problem with body-frame vector measurements this shares its
 * mechanisation with a body-frame MEKF; the invariant error gives the
 * trajectory-independent propagation Jacobian.
 * -------------------------------------------------------------------------- */
/* MEASURED, capture 20261103-201033: 2.46 h static, 224.2 Hz, flight config
 * (+/-16 g, +/-2048 dps, LPF 13.37 % ODR), 0 lost samples, 26.2-27.5 degC.
 * Tools/fc_noise.py, white level read over tau 0.1-1 s (below that the sensor
 * LPF shapes the curve):
 *   gyro ARW  7.4 / 8.5 / 7.7 mdps/rtHz (x y z; datasheet 13). Rate-register
 *             quantisation (white, Allan slope -1/2) is 2.0-2.7 % of that
 *             variance; rho = sigma/LSB = 0.93-1.2, i.e. at the eta knee.
 *   gyro BI   13.5 / 8.3 / 16.6 deg/h; tempco +0.023 / +0.008 / 0.000 dps/degC
 *   accel     66 / 67 / 84 ug/rtHz, 0.56-0.65 mg per sample, BI 53-105 ug
 * Q_ANGLE = worst-axis measured ARW (quantisation included: it is what the
 *   filter integrates).
 * Q_BIAS  = long-tau envelope max sigma(tau)*sqrt(3/tau), tau >= 100 s: covers
 *   the observed bias wander INCLUDING the 1.3 degC thermal drift (x axis);
 *   13x the old generic value. A cooling pad drifts the bias the same way.
 * R_ACCEL = sensor floor (0.65 mg)^2 = 4.3e-7 plus unmodelled non-gravitational
 *   acceleration on the pad, taken as 0.03 g 1-sigma (sway, handling that
 *   passes the |a| gate) -> 1e-3 (was 4e-2, 0.2 g). Larger transients are
 *   rejected by the |a| and NIS gates. Validate on the bench / HITL H1. */
#define IEKF_Q_ANGLE          1.49e-4f   /* rad/s/rtHz (ARW), 8.5 mdps/rtHz      */
#define IEKF_Q_BIAS           1.29e-5f   /* rad/s/rts  (bias random walk)        */
#define IEKF_R_ACCEL          1.0e-3f    /* accel dir update var (normalised)    */
#define IEKF_R_MAG            5.0e-3f     /* mag   dir update var (normalised)   */
#define IEKF_ACCEL_MIN        7.5f       /* |a| gate, m/s^2                     */
#define IEKF_ACCEL_MAX        12.0f
#define IEKF_MAG_MIN          0.20f      /* |B| gate, Gauss                     */
#define IEKF_MAG_MAX          0.70f
#define IEKF_NIS_GATE         11.34f     /* chi-square(3) 99%; <=0 disables     */
                                         /* (3DOF: 11.34=99%, 16.27=99.9%)      */
#define IEKF_P0_ANGLE         0.30f      /* initial attitude var (rad^2)        */
#define IEKF_P0_BIAS          1.2e-5f    /* initial bias var (rad/s)^2: residual
                                          * after the boot gyro calibration, 0.2 dps
                                          * 1-sigma. Also sets how fast the
                                          * unobservable heading variance grows. */

/* ----------------------------------------------------------------------------
 * 7. FEATURE FLAGS
 * -------------------------------------------------------------------------- */
#define CFG_ENABLE_IWDG       1     /* independent watchdog, frozen under debug */
#define CFG_ENABLE_BOOTVERIFY 1     /* synthetic mixer direction self-test      */
#define CFG_ENABLE_SERVO_SWEEP 0    /* boot sweep of every fin (airframe restrained) */
#define SWEEP_HOLD_MS         600u  /*   dwell at each deflection               */
#define SWEEP_STEP_MS         400u  /*   dwell at centre between steps          */
#define CFG_TELEM_BINARY      0     /* 1 = binary framed telemetry (PC logger)  */
                                    /* 0 = human-readable status text over USB  */
#define TEXT_STATUS_DECIMATE  (FRAME_HZ / 5u)   /* text status 5 Hz              */
#define TELEM_DECIMATE        (FRAME_HZ / 100u) /* binary STATE frame 100 Hz      */
#define CFG_ENABLE_GPS        1     /* ATGM336H NMEA on USART2 (PA3), 9600 Bd   */
#define GPS_TIMEOUT_MS        2500u /* no sentence for this long = GPS lost      */

/* ----------------------------------------------------------------------------
 * 8. FAULT DETECTION (FDS-01 instance table, fdir.c). Each bound states its
 *    source; persistence counts are written in ms (MS_TO_FRAMES).
 * -------------------------------------------------------------------------- */
#define FDIR_IMU_READ_N       MS_TO_FRAMES(100u)  /* failed reads = IMU lost (HARD) */
#define FDIR_ACCEL_MAX_MS2    (15.5f * BOARD_G_MPS2) /* +/-16 g FS less 3 %     */
#define FDIR_GYRO_MAX_DPS     1980.0f /* +/-2048 dps FS less 3 %                */
#define FDIR_RANGE_N          MS_TO_FRAMES(50u)
#define FDIR_LOOP_BUDGET_US   (FRAME_PERIOD_US * 8u / 10u) /* 80 % (std B1)     */
#define FDIR_LOOP_N           MS_TO_FRAMES(30u)
#define FDIR_OVERRUN_N        MS_TO_FRAMES(100u) /* sustained overrun (std B2): HARD on pad */
#define FDIR_EST_TILT_VAR_MAX 0.03f  /* rad^2, 2 tilt axes: ~7 deg 1-sigma each. Also
                                      * the 'covariance settled' READY gate (FDS-01
                                      * inst. 7): the seed P0 (0.6) must converge. */
#define FDIR_EST_COV_N        MS_TO_FRAMES(100u)

/* Pad READY gate: no SOFT/HARD fault for this long before READY (FDS-01 5.1;
 * bench value — the flight value is 30-60 s). */
#define READY_HOLD_MS         RUN_SETTLE_MS

/* Bench/HIL: allow fin actuation in READY (airframe on the ground). Flight
 * builds set 0 so fins move only in BOOST/COAST. */
#define CFG_BENCH_ACTUATE_READY 1

/* Bench fault injection console command ("inject wdg|phase|imu", FDS-01 §9.2
 * ground tests). Must be 0 in flight builds. */
#define CFG_FAULT_INJECTION   1

/* ----------------------------------------------------------------------------
 * 9. FLIGHT LOG (internal flash, flashlog.c) and USB firmware update
 *
 * F411CE flash: sectors 0-3 = 16 KB, 4 = 64 KB, 5-7 = 128 KB. The application
 * is linked into sectors 0-4 (128 KB, STM32F411CEUX_FLASH.ld); the log uses
 * sectors 5-7 (384 KB). A USB firmware update erases only the sectors the new
 * image covers, so the log survives reflashing.
 * -------------------------------------------------------------------------- */
#define LOG_FLASH_BASE        0x08020000u
#define LOG_FLASH_SIZE        (384u * 1024u)
#define LOG_FIRST_SECTOR      5u
#define LOG_SECTOR_COUNT      3u
/* Logging (ADR-004 §4): pre-trigger ring in RAM while READY; liftoff commits
 * it to flash oldest-first and recording continues. Rates are phase-scheduled
 * (FDS-01 §7.1): full rate through boost and coast, reduced under the canopy. */
#define LOG_HZ_ASCENT         100u   /* READY ring, BOOST, COAST                   */
#define LOG_HZ_DESCENT        20u    /* DESCENT                                    */
#define LOG_DECIMATE          (FRAME_HZ / LOG_HZ_ASCENT)
#define LOG_DECIMATE_DESCENT  (FRAME_HZ / LOG_HZ_DESCENT)
#define LOG_PRETRIG_RECS      256u   /* 2.56 s at 100 Hz, 12 KB, in the fw stage RAM */
#define LOG_DRAIN_PER_FRAME   2u     /* backlog records programmed per frame       */
#define LOG_POST_LAND_MS      60000u /* keep recording after LANDED, then close (covers
                                      * a premature LANDED under a calm canopy)       */
#define LOG_MIN_FREE_BYTES    (200u * 1024u) /* READY requires this much erased log */
#define LOG_AUTOSTART         1      /* 1 = arm the ring on READY (flight default) */

#define APP_FLASH_BASE        0x08000000u
#define APP_FLASH_SIZE        (128u * 1024u)
#define FW_STAGE_SIZE         (96u * 1024u)  /* SRAM staging buffer = max image  */
#define FW_RX_TIMEOUT_MS      2000u

/* Watchdog: normal timeout (CubeMX MX_IWDG_Init, ~100 ms) and the stretched
 * timeout used around flash erase (sector erase is up to 2 s, RM0383). */
#define WDG_LONG_RELOAD       4095u  /* with prescaler /256: ~32 s at 32 kHz LSI */

/* ----------------------------------------------------------------------------
 * 10. NAVIGATION, q_hat AND FLIGHT-EVENT DETECTION (NAV-01, nav.c, detect.c)
 * -------------------------------------------------------------------------- */
/* Site: MRC (Midland Rocketry Club), near Twycross, Leicestershire. The pad
 * elevation is NOT VERIFIED - the value below is an estimate for the area.
 * It is replaced on the pad by the GPS altitude average when a fix is held,
 * and density only moves ~1 %/100 m, so +/-50 m is ample. */
#define NAV_SITE_ELEV_M       110.0f
#define NAV_SAT_MPS2          (15.5f * BOARD_G_MPS2)  /* = FDIR accel range bound */
#define NAV_SAT_BUDGET_N      MS_TO_FRAMES(20u)       /* clipped frames tolerated (nav.h) */
#define NAV_STILL_ACC_MPS2    0.5f    /* pad zeroing: | |f| - g | below this ...   */
#define NAV_STILL_GYRO_RADPS  0.10f   /* ... and |w| below this (5.7 dps)          */
#define NAV_ACC_PSD           0.01f   /* (m/s^2)^2/Hz: VRW is ~5e-7 (measured); this
                                       * covers scale/misalignment error in boost  */
#define NAV_GPS_VAR_M2        100.0f  /* (10 m)^2, ATGM336H altitude, unverified   */
#define NAV_GPS_NIS_MAX       16.0f   /* chi^2(1) 4-sigma gate                     */
#define NAV_GPS_REJ_RESET     5u      /* consecutive rejections -> re-anchor to GPS */
#define NAV_GPS_V_VAR         100.0f  /* (m/s)^2 v_up after a re-anchor            */
#define NAV_GPS_HDOP_MAX      3.0f    /* fix quality for use (pad and descent)     */
#define NAV_GPS_SATS_MIN      6u
#define NAV_Q_SPEED_MIN       5.0f    /* m/s: below this q_hat is reported as 0    */

/* Detectors. Thresholds from the OpenRocket model (62mm Canard Control V1.0.0):
 * G79W peak 8.3 g, I205W 30 g; liftoff at 3 g is >= 2.8x margin below both and
 * 3x above the 1 g pad. Apogee backup timer: see DET_T_APOGEE_MAX_S (MC-01
 * per-motor table). */
#define CFG_PHASE_DETECT      1
#define DET_LIFTOFF_ACC       (3.0f * BOARD_G_MPS2)
#define DET_LIFTOFF_N         MS_TO_FRAMES(100u) /* above DET_LIFTOFF_ACC ...     */
#define DET_LIFTOFF_V_MPS     8.0f    /* ... and nav v_up above this (HITL: a 6 g /
                                       * 60 ms pad knock gave a false liftoff at the
                                       * old 50 ms / acc-only rule)                 */
#define DET_LIFTOFF_N_NOV     MS_TO_FRAMES(300u) /* acc-only if nav is invalid    */
#define NAV_BRIDGE_N          MS_TO_FRAMES(80u)  /* hold the last good specific force
                                       * through IMU read gaps (< FDIR_IMU_READ_N)  */
#define DET_T_BOOST_MIN_S     0.10f
#define DET_BURNOUT_ACC       0.0f    /* axial f < 0: drag exceeds thrust          */
#define DET_BURNOUT_N         MS_TO_FRAMES(50u)
#define DET_APOGEE_N          MS_TO_FRAMES(50u)
#define DET_T_APOGEE_MAX_S    11.5f   /* after burnout. MC-01 (G-motor shortlist):
                                       * coast p99 x 1.2 = 8.9 (G53FJ) .. 11.2 s (G80T).
                                       * Set per motor on the pre-flight sheet; 15 s was
                                       * for the I205W.                              */
#define DET_T_LAND_MIN_S      15.0f   /* > motor ejection delay (<= 10 s) + inflation:
                                       * a deployment shock must not latch 'touchdown' */
#define DET_T_LAND_MAX_S      300.0f  /* I205W descent 145 s (OpenRocket) x 2      */
#define DET_LAND_ACC_TOL      1.0f
#define DET_LAND_GYRO_RADPS   0.2f
#define DET_LAND_V_MPS        1.5f
#define DET_LAND_N            MS_TO_FRAMES(2000u) /* GPS rule                    */
/* Inertial landing (ADR-004 §5): impact shock + ground stillness. */
#define DET_LAND_TAU_S        0.5f
#define DET_LAND_STILL_SD     0.05f   /* m/s^2 (5 mg): 8x the measured sample noise */
#define DET_LAND_STILL_GYRO   0.05f   /* rad/s (2.9 dps)                           */
#define DET_LAND_SHOCK_ACC    (3.0f * BOARD_G_MPS2)
#define DET_LAND_N_SHOCK      MS_TO_FRAMES(2000u)
#define DET_LAND_N_STILL      MS_TO_FRAMES(30000u) /* no shock: 30 s of ground-level
                                       * stillness (SITL: 10 s fired under a calm canopy) */

/* ----------------------------------------------------------------------------
 * 11. HARDWARE-IN-THE-LOOP (HITL-01, hitl.c)
 *
 * CFG_HITL 1 compiles the seam in. It stays dormant (real IMU, timer-paced
 * frames) until the host sends its first 0x10 SAMPLE frame; from then on the
 * IMU sample comes from the host, each sample starts exactly one frame
 * (lockstep), the GPS parser takes host NMEA (0x12) instead of USART2, and a
 * 0x11 REPLY is sent at the end of every frame. A frame whose sample does not
 * arrive within HITL_WAIT_US runs with a failed IMU read, so a dead host
 * trips imu_read exactly as a dead sensor would. Must be 0 in flight builds.
 * -------------------------------------------------------------------------- */
#define CFG_HITL              1
#define HITL_WAIT_US          20000u  /* < IWDG (~100 ms) with margin           */
/* Passive (shadow) flight: the full law runs and is logged, the canards are
 * held at zero. Flight 1 per FDS-01 §9.1 / James 7 Nov. */
#define CFG_CTRL_SHADOW       0

/* Flight-build guard: define CFG_FLIGHT_BUILD=1 (compiler -D) for a flight
 * image; the bench-only features must then be off or the build fails. */
#if defined(CFG_FLIGHT_BUILD)
#if (CFG_FLIGHT_BUILD != 0) && ((CFG_HITL != 0) || (CFG_FAULT_INJECTION != 0) || (CFG_BENCH_ACTUATE_READY != 0))
#error "flight build with a bench feature enabled (CFG_HITL / CFG_FAULT_INJECTION / CFG_BENCH_ACTUATE_READY)"
#endif
#endif

#endif /* BOARD_H */
