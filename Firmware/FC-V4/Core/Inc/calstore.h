/* ============================================================================
 * calstore.h  —  Canard zero calibration in flash (CAL-01)
 *
 * Owns the 4 KB journal at CAL_FLASH_BASE (start of log sector 5): 128 slots
 * of FINCAL_REC_SIZE. A save appends one record (8 word writes, ~0.13 ms); it
 * never erases. The journal shares sector 5 with the flight log, so a log
 * erase wipes it: the app calls CalStore_Rewrite straight after, which puts
 * the current calibration back as the first record.
 *
 * A firmware update never touches it (that erases sectors 0-4 only).
 * ============================================================================ */
#ifndef CALSTORE_H_
#define CALSTORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "fincal.h"

typedef struct {
    FinCalScan s;          /* s.cal = calibration in force */
    uint32_t   write_errors;
} CalStore;

void CalStore_Init(CalStore *c);                           /* scan at boot      */
bool CalStore_Save(CalStore *c, const int16_t off_mdeg[FINCAL_FINS]);  /* false: full / error / range */
bool CalStore_Rewrite(CalStore *c);                        /* after a log erase  */
bool CalStore_Full(const CalStore *c);

#endif /* CALSTORE_H_ */
