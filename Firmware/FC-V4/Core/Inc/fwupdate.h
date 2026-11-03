/* ============================================================================
 * fwupdate.h  —  Firmware update over the USB console (no ST-LINK)
 *
 * Port of the GMWM "Sheppard" self-flasher to the F411: the running image
 * receives its replacement, stages it in SRAM, verifies it, then reprograms
 * flash from a routine executing in RAM and resets.
 *
 *   host -> "fw <size> <crc32hex>\r"     (Tools/fc_flash.py does all of this)
 *   dev  -> "FWREADY <size>"             host waits for this, then streams
 *   host -> <size> raw bytes
 *   dev  -> "FWRECV <n>", "FWCRC ok", "FWVEC ok", "FWPROG"
 *           USB detaches, flash is rewritten, the board resets and re-enumerates
 *   any failure before FWPROG -> "FWABORT <reason>" with flash untouched
 *
 * Differences from GMWM: only the sectors the new image covers are erased
 * (the flight log in sectors 5-7 survives), the watchdog is stretched first,
 * and updates are refused in flight phases.
 * CRC-32 = zlib (crc.h). Recovery if a new image does not boot: SWD.
 * ============================================================================ */
#ifndef FWUPDATE_H_
#define FWUPDATE_H_

#include <stdint.h>
#include <stdbool.h>

/* Console command handler: "fw <size> <crc32hex>". */
void Fw_Command(uint32_t argc, char *argv[]);

/* Main-loop step: timeout, verification and programming. Never returns once
 * programming starts. */
void Fw_Task(void);

/* USB ISR hook: consumes bytes while receiving an image. */
bool Fw_RxIsr(const uint8_t *buf, uint32_t len);

/* True while a transfer/verification is in progress (keep the link quiet). */
bool Fw_Busy(void);

#endif /* FWUPDATE_H_ */
