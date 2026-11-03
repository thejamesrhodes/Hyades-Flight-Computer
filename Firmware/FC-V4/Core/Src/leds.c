/* ============================================================================
 * leds.c  —  see leds.h
 * ============================================================================ */
#include "leds.h"
#include "main.h"

#define SYS_PORT LED_2_GPIO_Port
#define SYS_PIN  LED_2_Pin
#define GPS_PORT GPIOA
#define GPS_PIN  GPIO_PIN_0

static void set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Leds_Update(Phase ph, bool degraded, GpsLed gps, uint32_t t)
{
    const bool blink4  = ((t / 125u) & 1u) != 0u;
    const bool blip1   = (t % 1000u) < 100u;
    const bool flick10 = ((t / 50u) & 1u) != 0u;

    if ((ph == PH_LOCKOUT) || (ph == PH_CATO_SAFE)) {
        set(SYS_PORT, SYS_PIN, blink4);
        set(GPS_PORT, GPS_PIN, !blink4);
        return;
    }
    bool sys;
    switch (ph) {
    case PH_BOOT:
    case PH_CALIBRATE: sys = true;   break;
    case PH_SETTLING:  sys = blink4; break;
    default:           sys = degraded ? flick10 : blip1; break;
    }
    set(SYS_PORT, SYS_PIN, sys);
    set(GPS_PORT, GPS_PIN, (gps == GPS_LED_FIX) || ((gps == GPS_LED_NOFIX) && ((t % 1000u) < 80u)));
}
