/* ============================================================================
 * hitl.h  —  Hardware-in-the-loop seam (HITL-01), CFG_HITL builds only
 *
 * The flight firmware runs unmodified; only its INPUTS (IMU sample, GPS NMEA)
 * and the frame pacing change once a host starts sending samples. Frames use
 * the telemetry framing (telemetry.h): AA 55 id len payload crc16(id..payload).
 *
 * Host -> board
 *   0x10 SAMPLE, 32 B:  u32 idx | f32 accel[3] m/s^2 body | f32 gyro[3] dps body
 *                       (boot bias already removed, i.e. what IMU_ReadBody
 *                       returns) | u32 flags (bit0 = force a failed read)
 *   0x12 NMEA,  1..96 B: raw receiver bytes, fed to the GPS parser
 * Board -> host
 *   0x11 REPLY, 60 B:   u32 idx | u8 phase index | u8 ctrl mode | u16 faults
 *                       (active|latched) | u16 loop_us | u8 nav flags |
 *                       u8 hitl flags (bit0 = no sample this frame) |
 *                       f32 fin[4] | f32 q[4] | f32 h | f32 v_up | f32 speed |
 *                       f32 q_hat
 *       Python: struct.Struct('<IBBHHBB4f4f4f')
 *
 * Lockstep: once live, each frame waits (bounded, HITL_WAIT_US) for one
 * SAMPLE, runs, and replies. Console text still works: the RX demux strips
 * frames and hands every other byte to the line editor (text is 7-bit, so
 * 0xAA can only start a frame).
 * ============================================================================ */
#ifndef HITL_H_
#define HITL_H_

#include <stdint.h>
#include <stdbool.h>

#define HITL_MSG_SAMPLE  0x10u
#define HITL_MSG_REPLY   0x11u
#define HITL_MSG_NMEA    0x12u
#define HITL_SF_FAIL     (1u << 0)
#define HITL_RF_MISSING  (1u << 0)

typedef struct {
    uint32_t idx;
    float    accel[3];
    float    gyro[3];
    uint32_t flags;
} HitlSample;

typedef struct {
    uint32_t idx;
    uint8_t  phase, mode;
    uint16_t faults, loop_us;
    uint8_t  nav_flags, hitl_flags;
    float    fin[4], q[4];
    float    h, v_up, speed, qbar;
} HitlReply;

/* Drain USB RX: frames to the seam, everything else to the console. Bounded. */
void     Hitl_Poll(void);
bool     Hitl_Live(void);               /* a SAMPLE has been received */
bool     Hitl_Pending(void);
bool     Hitl_Take(HitlSample *s);      /* pop the pending sample      */
bool     Hitl_Reply(const HitlReply *r);

uint32_t Hitl_CrcErrors(void);
uint32_t Hitl_Overwrites(void);         /* sample replaced before use  */

#endif /* HITL_H_ */
