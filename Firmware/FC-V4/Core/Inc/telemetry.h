/* ============================================================================
 * telemetry.h  —  Binary framed telemetry over USB CDC (for a PC/Web-Serial log)
 *
 * Replaces the printf text logger on the hot path. Fixed binary frames are
 * cheap to emit (no vsnprintf), self-synchronising, and trivial to parse.
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

/* health bit flags */
#define TELEM_H_ACCEL_OK     (1u << 0)
#define TELEM_H_GYRO_OK      (1u << 1)
#define TELEM_H_MAG_OK       (1u << 2)
#define TELEM_H_ACCEL_APPLIED (1u << 3)   /* accel update not gated            */
#define TELEM_H_MAG_APPLIED   (1u << 4)   /* mag update not gated              */
#define TELEM_H_CTRL_ACTIVE  (1u << 5)
#define TELEM_H_OVERRUN      (1u << 6)    /* previous frame overran            */
#define TELEM_H_SETTLING     (1u << 7)    /* RUN settle window (no actuation)  */

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

/* Build a STATE frame and hand it to USB CDC. Non-blocking: if the endpoint is
 * busy the frame is dropped (counter incremented) and the loop is never stalled.
 * Returns true if queued. */
bool     Telem_SendState(const TelemState *st);

/* Send a text line over USB CDC. Non-blocking: dropped (and counted) if the
 * endpoint is busy or USB is not enumerated. For the human-readable status
 * stream (CFG_TELEM_BINARY = 0). Copies into an internal buffer. */
bool     Telem_SendText(const char *txt, uint32_t len);

/* Count of frames dropped because USB was busy (diagnostics). */
uint32_t Telem_Dropped(void);

#endif /* TELEMETRY_H_ */
