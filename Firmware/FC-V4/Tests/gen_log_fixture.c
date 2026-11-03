/* Writes a small raw log (BOOT, 3 STATE, 1 EVENT, a torn record, then erased
 * 0xFF) with the firmware's own packers, for tools_check.py. */
#include "logrec.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc < 2) { return 2; }
    FILE *f = fopen(argv[1], "wb");
    if (f == NULL) { return 2; }
    uint8_t r[LOGREC_SIZE];
    LogBoot b; memset(&b, 0, sizeof b);
    b.frame_hz = 100u; b.log_hz = 50u; memcpy(b.build, "abc12345 Nov  3 2026", 20);
    LogRec_PackBoot(r, &b); fwrite(r, 1, sizeof r, f);
    for (uint16_t i = 0u; i < 3u; i++) {
        LogState s; memset(&s, 0, sizeof s);
        s.t_us = 1000000u + (20000u * i); s.seq = (uint16_t)(1u + i); s.phase = 3u; s.mode = 2u;
        s.q[0] = 1.0f; s.w_dps[2] = 12.5f * (float)i; s.a_ms2[2] = 9.81f; s.fin[0] = -0.5f;
        s.loop_us = 1160u; s.faults = 0x0080u;
        LogRec_PackState(r, &s); fwrite(r, 1, sizeof r, f);
    }
    LogEvent e; memset(&e, 0, sizeof e);
    e.t_us = 1050000u; e.code = 2u; e.phase = 3u; memcpy(e.text, "SETTLE->READY", 13);
    LogRec_PackEvent(r, &e); fwrite(r, 1, sizeof r, f);
    LogRec_PackEvent(r, &e); r[30] ^= 0x40u; fwrite(r, 1, sizeof r, f);   /* torn */
    memset(r, 0xFF, sizeof r); fwrite(r, 1, sizeof r, f);
    fclose(f);
    return 0;
}
