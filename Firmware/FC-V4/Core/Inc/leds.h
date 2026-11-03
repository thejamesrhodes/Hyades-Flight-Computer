/* ============================================================================
 * leds.h  —  Status LEDs (positive-indication principle, FDS-01 §5.3)
 *
 *  SYS = PA1 (D8)                         GPS = PA0 (D6)
 *    solid ......... BOOT / CALIBRATE       solid ......... position fix
 *    4 Hz .......... SETTLING (not ready)   1 Hz blip ..... NMEA, no fix
 *    1 Hz blip ..... READY / flight, OK     off ........... no NMEA
 *    10 Hz flicker . degraded (fault or reduced control mode, held 1 s)
 *  Both alternating 4 Hz = LOCKOUT / CATO_SAFE (fins centred).
 *  Both solid = CPU fault handler (stm32f4xx_it.c).
 * Dark / anything else = unknown = NO-GO.
 * ============================================================================ */
#ifndef LEDS_H_
#define LEDS_H_

#include <stdint.h>
#include <stdbool.h>
#include "phase.h"

typedef enum { GPS_LED_NONE = 0, GPS_LED_NOFIX, GPS_LED_FIX } GpsLed;

void Leds_Update(Phase ph, bool degraded, GpsLed gps, uint32_t t_ms);

#endif /* LEDS_H_ */
