/* ============================================================================
 * crc.h  —  Checksums shared by firmware and host tools
 *
 *   crc16_ccitt : poly 0x1021, init 0xFFFF, no reflection (CCITT-FALSE).
 *                 Telemetry frames and log records. Python: binascii.crc_hqx(b, 0xFFFF)
 *   crc32_ieee  : zlib / IEEE 802.3, reflected 0xEDB88320, init/xorout ~0.
 *                 Firmware images and log pulls.   Python: zlib.crc32(b)
 *   crc32 can be chained: crc32_ieee_update(crc32_ieee_update(0, a), b).
 * ============================================================================ */
#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

uint16_t crc16_ccitt(const uint8_t *p, uint32_t n);
uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *p, uint32_t n);
static inline uint32_t crc32_ieee(const uint8_t *p, uint32_t n) { return crc32_ieee_update(0u, p, n); }

#endif /* CRC_H_ */
