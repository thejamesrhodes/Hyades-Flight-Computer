/* ============================================================================
 * telemetry.h  —  Binary framed telemetry over USB CDC (for a PC/Web-Serial log)
 *
 * Selected with CFG_TELEM_BINARY = 1 (else the 5 Hz text status is sent).
 * Fixed binary frames: cheap to emit, self-synchronising, trivial to parse.
 * Frames go through the shared USB TX ring (usbio.c) and are dropped whole if
 * it is full; console replies may appear between frames (resync on 0xAA55).
 *
 * FRAME (little-endian):
 *   +0  uint8  SYNC0 = 0xAA
 *   +1  uint8  SYNC1 = 0x55
 *   +2  uint8  MSG_ID
 *   +3  uint8  LEN            (payload length, bytes)
 *   +4  uint8  payload[LEN]
 *   +4+LEN  uint16 CRC16-CCITT over [MSG_ID, LEN, payload], init 0xFFFF, poly 0x1021
 *
 * MSG_ID 0x01 = STATE, payload = 104 bytes:
 *   off  type      field
 *     0  u32       t_us           timestamp (1 MHz)
 *     4  u32       frame          frame counter
 *     8  f32[4]    q              quaternion w,x,y,z (body->world, ENU)
 *    24  f32[3]    euler          roll,pitch,yaw (deg)
 *    36  f32[3]    omega          body rates (deg/s, bias not removed)
 *    48  f32[3]    accel          body specific force (m/s^2)
 *    60  f32[3]    mag            body field (Gauss)
 *    72  f32[3]    bias           gyro bias estimate (rad/s)
 *    84  f32[4]    fin            fin commands, normalised [-1,+1]
 *   100  u16       health         bit flags (see TELEM_H_* below)
 *   102  u16       loop_us        measured frame compute time (us)
 *
 * Python (receiver) payload unpack:
 *   import struct
 *   STATE = struct.Struct('<II4f3f3f3f3f3f4fHH')   # 104 bytes
 *   (t_us, frame, qw,qx,qy,qz, roll,pitch,yaw, wx,wy,wz,
 *    ax,ay,az, mx,my,mz, bx,by,bz, f0,f1,f2,f3, health, loop_us) = STATE.unpack(payload)
 * ============================================================================ */
#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>

#define TELEM_SYNC0     0xAAu
#define TELEM_SYNC1     0x55u
#define TELEM_MSG_STATE 0x01u
#define TELEM_MSG_IMU   0x02u   /* raw IMU sample, 'imu capture' only (below) */

/* health bit flags */
#define TELEM_H_ACCEL_OK     (1u << 0)
#define TELEM_H_GYRO_OK      (1u << 1)
#define TELEM_H_MAG_OK       (1u << 2)
#define TELEM_H_ACCEL_APPLIED (1u << 3)   /* accel update not gated            */
#define TELEM_H_MAG_APPLIED   (1u << 4)   /* mag update not gated              */
#define TELEM_H_CTRL_ACTIVE  (1u << 5)
#define TELEM_H_OVERRUN      (1u << 6)    /* previous frame overran            */
#define TELEM_H_SETTLING     (1u << 7)    /* not READY/flight (no actuation)   */
#define TELEM_H_FAULT        (1u << 8)    /* any SOFT/HARD monitor active      */
#define TELEM_H_LOCKOUT      (1u << 9)    /* LOCKOUT or CATO_SAFE latched      */
#define TELEM_H_MODE_SHIFT   10u          /* bits 10-11: ctrl mode 0 OFF,1 RDAMP,2 FULL */

typedef struct {
    uint32_t t_us;
    uint32_t frame;
    float    q[4];
    float    euler[3];
    float    omega[3];
    float    accel[3];
    float    mag[3];
    float    bias[3];
    float    fin[4];
    uint16_t health;
    uint16_t loop_us;
} TelemState;

/* Build a STATE frame and queue it. Non-blocking; returns false if dropped. */
bool     Telem_SendState(const TelemState *st);

/* MSG_ID 0x02 = IMU, payload 18 bytes, raw sensor counts, sensor frame:
 *   0 u32 ts (sensor sample counter, 24-bit, +1 per ODR tick)  4 i16 temp
 *   6 i16 a[3]   12 i16 g[3]          struct.Struct('<Ih3h3h')            */
bool     Telem_SendImu(uint32_t ts, int16_t temp, const int16_t a[3], const int16_t g[3]);

#endif /* TELEMETRY_H_ */
