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
 * Single 100 Hz minor frame paced by TIM5 (32-bit, 1 MHz free-running).
 * Sensor ODRs are matched/oversampled to the frame so a fresh, BDU-protected
 * sample is available each frame. Sub-rates are counter-gated off the frame.
 * -------------------------------------------------------------------------- */
#define FRAME_HZ              100u
#define FRAME_PERIOD_US       (1000000u / FRAME_HZ)   /* 10 000 us            */
#define FRAME_DT_S            (1.0f / (float)FRAME_HZ) /* exact 0.010 s        */


/* RUN-state settle: hold fins centred and control inactive for this long
 * after seeding so the IEKF converges before any actuation. */
#define RUN_SETTLE_MS         1000u

/* ----------------------------------------------------------------------------
 * 4. SERVO / FIN OUTPUT (see pwm.c for channel-to-pin mapping)
 * -------------------------------------------------------------------------- */
#define SERVO_PERIOD_US       20000u   /* 50 Hz, safe for any servo; 4000 (250 Hz) for digital */
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
#define TEXT_STATUS_DECIMATE  20u   /* text status every N frames -> 5 Hz      */
#define TELEM_DECIMATE        1u    /* binary STATE frame every N frames -> 100 Hz */
#define CFG_ENABLE_GPS        1     /* ATGM336H NMEA on USART2 (PA3), 9600 Bd   */
#define GPS_TIMEOUT_MS        2500u /* no sentence for this long = GPS lost      */

/* ----------------------------------------------------------------------------
 * 8. FAULT DETECTION (FDS-01 instance table, fdir.c). Each bound states its
 *    source; persistence counts are in 100 Hz frames.
 * -------------------------------------------------------------------------- */
#define FDIR_IMU_READ_N       10u    /* 100 ms of failed reads = IMU lost (HARD) */
#define FDIR_ACCEL_MAX_MS2    (15.5f * BOARD_G_MPS2) /* +/-16 g FS less 3 %     */
#define FDIR_GYRO_MAX_DPS     1980.0f /* +/-2048 dps FS less 3 %                */
#define FDIR_RANGE_N          5u     /* 50 ms                                    */
#define FDIR_LOOP_BUDGET_US   (FRAME_PERIOD_US * 8u / 10u) /* 80 % (std B1)     */
#define FDIR_LOOP_N           3u
#define FDIR_OVERRUN_N        10u    /* sustained overrun (std B2): HARD on pad  */
#define FDIR_EST_TILT_VAR_MAX 0.03f  /* rad^2, 2 tilt axes: ~7 deg 1-sigma each. Also
                                      * the 'covariance settled' READY gate (FDS-01
                                      * inst. 7): the seed P0 (0.6) must converge. */
#define FDIR_EST_COV_N        10u

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
#define LOG_DECIMATE          2u     /* STATE record every N frames -> 50 Hz      */
#define LOG_AUTOSTART         0      /* 1 = start logging on entry to READY       */

#define APP_FLASH_BASE        0x08000000u
#define APP_FLASH_SIZE        (128u * 1024u)
#define FW_STAGE_SIZE         (96u * 1024u)  /* SRAM staging buffer = max image  */
#define FW_RX_TIMEOUT_MS      2000u

/* Watchdog: normal timeout (CubeMX MX_IWDG_Init, ~100 ms) and the stretched
 * timeout used around flash erase (sector erase is up to 2 s, RM0383). */
#define WDG_LONG_RELOAD       4095u  /* with prescaler /256: ~32 s at 32 kHz LSI */

#endif /* BOARD_H */
