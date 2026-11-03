/* ============================================================================
 * baud.h  —  Snap a measured shortest UART pulse to a standard baud rate
 *
 * Pure. The shortest low/high pulse seen on an idle-high async line over a
 * second of NMEA is one bit time, so baud ~= clk / min_pulse_cycles. Returns
 * 0 if the estimate is not within +/-20 % of a standard rate.
 * ============================================================================ */
#ifndef BAUD_H_
#define BAUD_H_

#include <stdint.h>

static inline uint32_t Baud_Snap(uint32_t clk_hz, uint32_t min_pulse_cycles)
{
    static const uint32_t k_std[] = { 4800u, 9600u, 19200u, 38400u, 57600u, 115200u };
    if (min_pulse_cycles == 0u) { return 0u; }
    const uint32_t est = clk_hz / min_pulse_cycles;
    for (uint32_t i = 0u; i < (uint32_t)(sizeof k_std / sizeof k_std[0]); i++) {
        const uint32_t lo = k_std[i] - (k_std[i] / 5u), hi = k_std[i] + (k_std[i] / 5u);
        if ((est >= lo) && (est <= hi)) { return k_std[i]; }
    }
    return 0u;
}

#endif /* BAUD_H_ */
