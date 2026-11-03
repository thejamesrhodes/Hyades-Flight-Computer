/* ============================================================================
 * nmea.c  —  NMEA-0183 GGA/RMC parser (see nmea.h)
 * ============================================================================ */
#include "nmea.h"
#include <string.h>

#define FIELDS_MAX 24u

void Nmea_Init(NmeaParser *p, GpsData *g)
{
    memset(p, 0, sizeof(*p));
    memset(g, 0, sizeof(*g));
    g->hdop = 99.9f;
}

/* Decimal text ("-123.4567") -> double. At most 9 integer and 9 fraction
 * digits are used; NMEA fields are far shorter. Empty -> 0. */
double Nmea_ParseNum(const char *v)
{
    static const double p10[10] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9 };
    const bool neg = (*v == '-');
    if (neg) { v++; }
    uint32_t ip = 0u, fp = 0u, nf = 0u;
    for (uint32_t i = 0u; (i < 9u) && (*v >= '0') && (*v <= '9'); i++, v++) {
        ip = (ip * 10u) + (uint32_t)(*v - '0');
    }
    if (*v == '.') {
        v++;
        for (; (nf < 9u) && (*v >= '0') && (*v <= '9'); nf++, v++) {
            fp = (fp * 10u) + (uint32_t)(*v - '0');
        }
    }
    const double x = (double)ip + ((double)fp / p10[nf]);
    return neg ? -x : x;
}

static uint32_t parse_uint(const char *v)
{
    uint32_t x = 0u;
    for (uint32_t i = 0u; (i < 9u) && (*v >= '0') && (*v <= '9'); i++, v++) {
        x = (x * 10u) + (uint32_t)(*v - '0');
    }
    return x;
}

/* ddmm.mmmm + hemisphere -> signed degrees */
static double nmea_deg(const char *v, const char *hemi)
{
    if (v[0] == '\0') { return 0.0; }
    const double x = Nmea_ParseNum(v);
    const double d = (double)(uint32_t)(x / 100.0);
    double deg = d + (x - (d * 100.0)) / 60.0;
    if ((hemi[0] == 'S') || (hemi[0] == 'W')) { deg = -deg; }
    return deg;
}

static uint8_t hexval(char c)
{
    if ((c >= '0') && (c <= '9')) { return (uint8_t)(c - '0'); }
    if ((c >= 'A') && (c <= 'F')) { return (uint8_t)(c - 'A' + 10); }
    if ((c >= 'a') && (c <= 'f')) { return (uint8_t)(c - 'a' + 10); }
    return 0xFFu;
}

/* Split in place on ',' and '*'; returns the field count. */
static uint32_t split(char *s, char **f, uint32_t maxf)
{
    uint32_t n = 0u;
    f[n++] = s;
    for (char *q = s; (*q != '\0') && (n < maxf); q++) {
        if ((*q == ',') || (*q == '*')) { *q = '\0'; f[n++] = q + 1; }
    }
    return n;
}

/* s = "GNGGA,....*hh" (no '$'), NUL-terminated, len chars. */
static void handle_line(GpsData *g, char *s, uint32_t len, uint32_t now_ms)
{
    const char *star = memchr(s, '*', len);
    if ((star == NULL) || ((uint32_t)(star - s) + 3u > len)) { g->cksum_err++; return; }
    uint8_t ck = 0u;
    for (const char *q = s; q < star; q++) { ck ^= (uint8_t)*q; }
    const uint8_t h = hexval(star[1]), l = hexval(star[2]);
    if ((h > 15u) || (l > 15u) || (ck != (uint8_t)((h << 4) | l))) { g->cksum_err++; return; }

    g->sentences++;
    g->last_rx_ms = now_ms;
    if (len < 6u) { return; }
    const char *type = &s[2];                 /* skip the 2-char talker ID */

    char *f[FIELDS_MAX];
    if (strncmp(type, "GGA", 3) == 0) {
        const uint32_t n = split(s, f, FIELDS_MAX);
        if (n < 10u) { return; }
        /* 0 id,1 utc,2 lat,3 N/S,4 lon,5 E/W,6 qual,7 sats,8 hdop,9 alt,10 M */
        g->utc_hhmmss  = parse_uint(f[1]);
        g->fix_quality = (uint8_t)parse_uint(f[6]);
        g->sats        = (uint8_t)parse_uint(f[7]);
        g->hdop        = (f[8][0] != '\0') ? (float)Nmea_ParseNum(f[8]) : 99.9f;
        if (g->fix_quality > 0u) {
            g->lat_deg   = nmea_deg(f[2], f[3]);
            g->lon_deg   = nmea_deg(f[4], f[5]);
            g->alt_msl_m = (float)Nmea_ParseNum(f[9]);
        }
        g->last_gga_ms = now_ms;
        g->gga++;
    } else if (strncmp(type, "RMC", 3) == 0) {
        const uint32_t n = split(s, f, FIELDS_MAX);
        if (n < 9u) { return; }
        /* 0 id,1 utc,2 status,3 lat,4 N/S,5 lon,6 E/W,7 sog kn,8 cog */
        g->rmc_valid  = (f[2][0] == 'A');
        g->speed_mps  = (f[7][0] != '\0') ? (float)Nmea_ParseNum(f[7]) * 0.514444f : 0.0f; /* kn -> m/s */
        g->course_deg = (f[8][0] != '\0') ? (float)Nmea_ParseNum(f[8]) : 0.0f;
        g->rmc++;
    } else {
        /* other sentences: counted above, otherwise ignored */
    }
}

void Nmea_Feed(NmeaParser *p, GpsData *g, char c, uint32_t now_ms)
{
    g->bytes++;
    if (c == '$') { p->in_sentence = true; p->len = 0u; return; }
    if (!p->in_sentence) { return; }
    if ((c == '\r') || (c == '\n')) {
        if (p->len > 0u) { p->line[p->len] = '\0'; handle_line(g, p->line, p->len, now_ms); }
        p->in_sentence = false; p->len = 0u;
        return;
    }
    if (p->len < (NMEA_LINE_MAX - 1u)) { p->line[p->len++] = c; }
    else { p->in_sentence = false; p->len = 0u; g->cksum_err++; }      /* too long */
}

bool Nmea_Alive(const GpsData *g, uint32_t now_ms, uint32_t timeout_ms)
{
    return (g->sentences > 0u) && ((now_ms - g->last_rx_ms) < timeout_ms);
}

bool Nmea_HasFix(const GpsData *g, uint32_t now_ms, uint32_t timeout_ms)
{
    return (g->gga > 0u) && ((now_ms - g->last_gga_ms) < timeout_ms) && (g->fix_quality > 0u);
}
