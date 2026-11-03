/* ============================================================================
 * flashlog.h  —  Append-only flight log in internal flash (sectors 5-7)
 *
 * 384 KB = 8192 fixed 48-byte records (logrec.h): ~160 s of STATE at 50 Hz.
 * Append-only, never wraps, never erased automatically (FDS-01 P4: protect
 * recorded data). The write position is recovered at boot by scanning for the
 * first erased slot. Each "start" writes a BOOT record, so one log can hold
 * several sessions; the reader splits on them.
 *
 * Timing: programming a record is 12 word writes (~16 us each, RM0383) with
 * the CPU stalled ~0.2 ms; erase is up to 2 s per sector and is only done on
 * an explicit command, as a scheduler maintenance operation (fins centred).
 *
 * Pull: "log pull" streams  "LOGDATA <bytes>\r\n" <raw records> "LOGEND <crc32hex>\r\n"
 * in 2 KB chunks from the main loop (non-blocking). Tools/fc_pull.py decodes.
 * ============================================================================ */
#ifndef FLASHLOG_H_
#define FLASHLOG_H_

#include <stdint.h>
#include <stdbool.h>
#include "logrec.h"

void     FlashLog_Init(void);                       /* scan for the end      */
bool     FlashLog_Start(const LogBoot *boot);       /* false if full         */
void     FlashLog_Stop(void);
bool     FlashLog_Active(void);
bool     FlashLog_Write(const uint8_t rec[LOGREC_SIZE]);  /* false if inactive/full/error */
uint16_t FlashLog_NextSeq(void);

/* Blocking (up to ~6 s). Caller must have centred the fins and accept the
 * frame slip; the watchdog is stretched internally. */
bool     FlashLog_Erase(void);

uint32_t FlashLog_UsedBytes(void);
uint32_t FlashLog_CapacityBytes(void);
uint32_t FlashLog_WriteErrors(void);

/* Pull over USB (non-blocking state machine). */
bool     FlashLog_PullStart(void);
void     FlashLog_PullTask(void);
bool     FlashLog_Pulling(void);

#endif /* FLASHLOG_H_ */
