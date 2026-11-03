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
 * The three sensors (LSM303 accel, LSM303 mag, L3GD20 gyro) are SEPARATE dies
 * with DIFFERENT axis orientations on the PCB. Each raw triad must be mapped
 * into the common body frame defined above. These maps cannot be guessed from
 * software — set them once for your board and verify (procedure in README).
 *
 * Convention: each macro takes the sensor-native (x,y,z) and yields the body
 * (x,y,z). Defaults are IDENTITY so the mapping is explicit and obvious to
 * edit; they are almost certainly NOT correct for your mounting.
 *
 * Verification (use the binary telemetry stream):
 *   a) Hold the airframe nose-up, stationary. body accel must read ~(0,0,+g).
 *      Fix signs/permutation of REMAP_ACCEL until it does.
 *   b) Rotate +about nose (right-hand, thumb out the nose): body gyro Z must
 *      read positive. Repeat for pitch(+X) and yaw(+Y). Fix REMAP_GYRO.
 *   c) With accel/gyro correct, tilt the airframe and confirm IEKF roll/pitch
 *      track correctly; then rotate in heading and confirm yaw tracks without
 *      fighting tilt. If yaw is wrong/unstable, fix REMAP_MAG.
 * -------------------------------------------------------------------------- */
#define REMAP_ACCEL(x,y,z, bx,by,bz)  do { (bx)=(-z); (by)=(y); (bz)=(-x); } while (0)
#define REMAP_GYRO(x,y,z, bx,by,bz)   do { (bx)=(-z); (by)=(y); (bz)=(-x); } while (0)
#define REMAP_MAG(x,y,z, bx,by,bz)    do { (bx)=(-z); (by)=(y); (bz)=(-x); } while (0)

/* IMU backend select. 1 = on-board QMI8658A (fitted at U7 on the V3 board;
 * imu_qmi8658.c), 0 = legacy LSM303DLHC + L3GD20 I2C array (imu.c).        */
#define IMU_BACKEND_QMI8658   1

/* QMI8658A sensor -> body remap. *** VERIFY ON THE BENCH (README steps a,b) ***
 * Measured so far: board lying flat, component side up -> sensor +Z reads +1 g.
 * The board->airframe mounting is not known here, so this is IDENTITY. */
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
 * trajectory-independent propagation Jacobian. Same datasheet noise as before.
 * -------------------------------------------------------------------------- */
#define IEKF_Q_ANGLE          2.269e-4f  /* QMI8658A 13 mdps/rtHz in rad/s/rtHz  */
#define IEKF_Q_BIAS           1.0e-6f    /* gyro bias random walk               */
#define IEKF_R_ACCEL          4.0e-2f    /* accel dir update var (normalised)   */
#define IEKF_R_MAG            5.0e-3f     /* mag   dir update var (normalised)   */
#define IEKF_ACCEL_MIN        7.5f       /* |a| gate, m/s^2                     */
#define IEKF_ACCEL_MAX        12.0f
#define IEKF_MAG_MIN          0.20f      /* |B| gate, Gauss                     */
#define IEKF_MAG_MAX          0.70f
#define IEKF_NIS_GATE         11.34f     /* chi-square(3) 99%; <=0 disables     */
                                         /* (3DOF: 11.34=99%, 16.27=99.9%)      */
#define IEKF_P0_ANGLE         0.30f      /* initial attitude var (rad^2)        */
#define IEKF_P0_BIAS          1.0e-3f    /* initial bias var (rad/s)^2          */

/* ----------------------------------------------------------------------------
 * 7. FEATURE FLAGS
 * -------------------------------------------------------------------------- */
#define CFG_ENABLE_IWDG       1     /* independent watchdog, frozen under debug */
#define CFG_ENABLE_BOOTVERIFY 1     /* synthetic mixer direction self-test      */
#define CFG_ENABLE_MAG_CAL    0     /* interactive (rotate airframe) mag cal    */
#define CFG_ENABLE_SERVO_SWEEP 0    /* boot sweep of every fin (airframe restrained) */
#define SWEEP_HOLD_MS         600u  /*   dwell at each deflection               */
#define SWEEP_STEP_MS         400u  /*   dwell at centre between steps          */
#define CFG_TELEM_BINARY      0     /* 1 = binary framed telemetry (PC logger)  */
                                    /* 0 = human-readable status text over USB  */
#define TEXT_STATUS_DECIMATE  20u   /* text status every N frames -> 5 Hz      */
#define CFG_ENABLE_GPS        1     /* ATGM336H NMEA on USART1 (PA10/PA15)      */
#define GPS_TIMEOUT_MS        2500u /* no sentence for this long = GPS lost      */

#endif /* BOARD_H */
