/* ============================================================================
 * logq.h  —  Pre-trigger ring / write-behind queue for flight-log records
 *            (ADR-004 §4). Pure module: no flash, no HAL.
 *
 * Two modes over one circular buffer of 48-byte records:
 *  ARMED     (pad, READY): Push overwrites the oldest record, so the buffer
 *            always holds the last `cap` records ("pre-trigger").
 *  COMMITTED (from liftoff): Push never overwrites; when full the NEW record
 *            is dropped and counted (the backlog is the older, rarer data —
 *            the moments before liftoff — and must not be lost to newer
 *            descent records). The caller drains oldest-first with Peek/Pop,
 *            a bounded number per frame, so flash programming stalls are
 *            spread out (F411 word program 16 us typ / 100 us max, CPU
 *            stalled; one 48-byte record = 12 words).
 * Sizing: drain rate must exceed the production rate or the backlog never
 * clears: at 100 Hz logging and 2 records/frame at 200 Hz (400/s), a full
 * 256-record ring clears in 256 / (400 - 100) = 0.85 s.
 * ============================================================================ */
#ifndef LOGQ_H_
#define LOGQ_H_

#include <stdint.h>
#include <stdbool.h>
#include "logrec.h"

typedef struct {
    uint8_t  (*buf)[LOGREC_SIZE];
    uint16_t cap, head, count;   /* head = index of the oldest record */
    bool     committed;
    uint32_t dropped, pushed;
} LogQ;

void LogQ_Init(LogQ *q, uint8_t (*buf)[LOGREC_SIZE], uint16_t cap);
void LogQ_Reset(LogQ *q);                     /* empty, ARMED, counters cleared */
void LogQ_Commit(LogQ *q);                    /* ARMED -> COMMITTED (keeps contents) */
bool LogQ_Push(LogQ *q, const uint8_t rec[LOGREC_SIZE]);   /* false = dropped */
const uint8_t *LogQ_Peek(const LogQ *q);      /* oldest, NULL if empty */
void LogQ_Pop(LogQ *q);
uint16_t LogQ_Count(const LogQ *q);

#endif /* LOGQ_H_ */
