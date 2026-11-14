/* ============================================================================
 * fincal.c  —  see fincal.h
 * ============================================================================ */
#include "fincal.h"
#include "crc.h"
#include <string.h>

#define OFF_MAGIC  0u
#define OFF_SEQ    4u
#define OFF_VER    6u
#define OFF_COUNT  7u
#define OFF_FINS   8u
#define OFF_CRC    28u

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)(v >> 8); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8)); }
static void put_u32(uint8_t *p, uint32_t v) { for (uint32_t k = 0u; k < 4u; k++) { p[k] = (uint8_t)(v >> (8u * k)); } }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool FinCal_Within(const int32_t off_mdeg[FINCAL_FINS], int32_t lim_mdeg)
{
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) {
        if ((off_mdeg[i] > lim_mdeg) || (off_mdeg[i] < -lim_mdeg)) { return false; }
    }
    return true;
}

bool FinCal_InRange(const int32_t off_mdeg[FINCAL_FINS]) { return FinCal_Within(off_mdeg, FINCAL_LIMIT_MDEG); }

void FinCal_Pack(uint8_t out[FINCAL_REC_SIZE], uint16_t seq, const int32_t off_mdeg[FINCAL_FINS])
{
    memset(out, 0, FINCAL_REC_SIZE);
    put_u32(&out[OFF_MAGIC], FINCAL_MAGIC);
    put_u16(&out[OFF_SEQ], seq);
    out[OFF_VER] = (uint8_t)FINCAL_VERSION;
    out[OFF_COUNT] = (uint8_t)FINCAL_FINS;
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) { put_u32(&out[OFF_FINS + (4u * i)], (uint32_t)off_mdeg[i]); }
    put_u32(&out[OFF_CRC], crc32_ieee(out, OFF_CRC));
}

bool FinCal_Unpack(const uint8_t in[FINCAL_REC_SIZE], FinCal *out)
{
    out->valid = false;
    out->version = 0u;
    const uint8_t ver = in[OFF_VER];
    if (get_u32(&in[OFF_MAGIC]) != FINCAL_MAGIC) { return false; }
    if ((ver != (uint8_t)FINCAL_VERSION) && (ver != (uint8_t)FINCAL_VERSION_OLD)) { return false; }
    if (in[OFF_COUNT] != (uint8_t)FINCAL_FINS) { return false; }
    if (get_u32(&in[OFF_CRC]) != crc32_ieee(in, OFF_CRC)) { return false; }
    int32_t off[FINCAL_FINS];
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) { off[i] = (int32_t)get_u32(&in[OFF_FINS + (4u * i)]); }
    if (ver == (uint8_t)FINCAL_VERSION_OLD) {
        /* Old-scale degrees x 6.667 / 8.889 = x 3/4: the pulse, and so the canard
         * position found at alignment, is unchanged. Range-checked before the
         * multiply (|x| <= 90 000, x 3 fits int32), rounded to nearest. */
        if (!FinCal_Within(off, FINCAL_LIMIT_OLD_MDEG)) { return false; }
        for (uint32_t i = 0u; i < FINCAL_FINS; i++) {
            off[i] = (off[i] >= 0) ? (((off[i] * 3) + 2) / 4) : -((((-off[i]) * 3) + 2) / 4);
        }
    }
    if (!FinCal_InRange(off)) { return false; }
    out->seq = get_u16(&in[OFF_SEQ]);
    out->version = ver;
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) { out->off_mdeg[i] = off[i]; }
    out->valid = true;
    return true;
}

static bool erased(const uint8_t *p)
{
    for (uint32_t k = 0u; k < FINCAL_REC_SIZE; k++) { if (p[k] != 0xFFu) { return false; } }
    return true;
}

void FinCal_Scan(const uint8_t *base, uint32_t size, FinCalScan *s)
{
    memset(s, 0, sizeof *s);
    s->slots = size / FINCAL_REC_SIZE;
    /* The erased tail starts after the last non-erased slot: saves only ever
     * append, so anything before it is history (or foreign data to skip). */
    uint32_t tail = 0u;
    for (uint32_t i = s->slots; i > 0u; i--) {
        if (!erased(&base[(i - 1u) * FINCAL_REC_SIZE])) { tail = i; break; }
    }
    s->next = tail;
    for (uint32_t i = 0u; i < tail; i++) {
        FinCal c;
        if (FinCal_Unpack(&base[i * FINCAL_REC_SIZE], &c)) { s->cal = c; }    /* newest wins */
        else { s->bad++; }
    }
}
