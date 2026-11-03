/* ============================================================================
 * gps.c  —  NMEA-0183 receiver for the ATGM336H on USART1. See gps.h.
 * ============================================================================ */
#include "gps.h"
#include <string.h>
#include <stdlib.h>

GpsData gps;

#define RB_SIZE   512u                /* power of two; ~0.5 s at 9600 bps   */
#define LINE_MAX  96u                 /* NMEA max 82 chars + margin         */

static UART_HandleTypeDef *s_huart;
static volatile uint8_t  rb[RB_SIZE];
static volatile uint32_t rb_head = 0u, rb_tail = 0u;   /* head: ISR, tail: main */
static volatile uint32_t isr_overrun = 0u;

static char     line[LINE_MAX];
static uint32_t line_len = 0u;
static bool     in_sentence = false;

/* ---- ISR --------------------------------------------------------------- */
void USART1_IRQHandler(void)
{
    USART_TypeDef *u = USART1;
    uint32_t sr = u->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
        uint8_t c = (uint8_t)u->DR;        /* SR then DR read clears ORE/FE/NE */
        if ((sr & USART_SR_ORE) != 0u) { isr_overrun++; }
        if ((sr & USART_SR_RXNE) != 0u) {
            uint32_t next = (rb_head + 1u) & (RB_SIZE - 1u);
            if (next != rb_tail) { rb[rb_head] = c; rb_head = next; }
            else                 { isr_overrun++; }
        }
    }
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&gps, 0, sizeof(gps));
    rb_head = rb_tail = 0u;

    /* Receive-only: nothing is sent to the module, so PA15 (USART1_TX) is
     * released to a pulled-up input. If the harness has TX/RX crossed, the
     * module's TX then drives an input instead of fighting our output.
     * PA10 (RX) gets a pull-up so an unplugged line idles high, not floating. */
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_15; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
    gi.Pin = GPIO_PIN_10; gi.Mode = GPIO_MODE_AF_PP; gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW; gi.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gi);

    (void)s_huart->Instance->SR; (void)s_huart->Instance->DR;   /* flush */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);    /* below USB (0); above SysTick (15) */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_RXNE);
}

void GPS_LineActivity(uint32_t window_ms, uint32_t *edges_pa10, uint32_t *edges_pa15)
{
    uint32_t n10 = 0u, n15 = 0u;
    uint32_t prev = GPIOA->IDR;                 /* IDR reads the pin in AF mode too */
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < window_ms) {  /* ~MHz polling >> 9.6 kbit/s */
        uint32_t now = GPIOA->IDR;
        uint32_t chg = now ^ prev;
        if ((chg & GPIO_PIN_10) != 0u) { n10++; }
        if ((chg & GPIO_PIN_15) != 0u) { n15++; }
        prev = now;
    }
    *edges_pa10 = n10;
    *edges_pa15 = n15;
}

/* ---- field helpers ----------------------------------------------------- */
/* Split in place on ',' (and '*'); returns field count. */
static uint32_t split(char *s, char **f, uint32_t maxf)
{
    uint32_t n = 0u;
    f[n++] = s;
    for (char *p = s; *p != '\0' && n < maxf; p++) {
        if (*p == ',' || *p == '*') { *p = '\0'; f[n++] = p + 1; }
    }
    return n;
}

/* Decimal text ("-123.4567") -> double. Used instead of strtod/strtof, which
 * in newlib allocate from the heap (no hidden malloc at runtime). At most 9
 * integer and 9 fraction digits are used; NMEA fields are far shorter. */
static double nmea_num(const char *v)
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

/* ddmm.mmmm + hemisphere -> signed degrees */
static double nmea_deg(const char *v, const char *hemi)
{
    if (v[0] == '\0') { return 0.0; }
    double x = nmea_num(v);
    double d = (double)((int)(x / 100.0));
    double deg = d + (x - d * 100.0) / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') { deg = -deg; }
    return deg;
}

static uint8_t hexval(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFu;
}

/* line = "GNGGA,....*hh" (without '$'). Validate and parse. */
static void handle_line(char *s, uint32_t len)
{
    /* checksum: XOR of chars before '*' */
    char *star = memchr(s, '*', len);
    if (star == NULL || (uint32_t)(star - s) + 3u > len) { gps.cksum_err++; return; }
    uint8_t ck = 0u;
    for (char *p = s; p < star; p++) { ck ^= (uint8_t)*p; }
    uint8_t h = hexval(star[1]), l = hexval(star[2]);
    if (h > 15u || l > 15u || ck != (uint8_t)((h << 4) | l)) { gps.cksum_err++; return; }

    gps.sentences++;
    gps.last_rx_ms = HAL_GetTick();
    if (len < 6u) { return; }
    const char *type = &s[2];            /* skip 2-char talker ID */

    char *f[24];
    if (strncmp(type, "GGA", 3) == 0) {
        uint32_t n = split(s, f, 24u);
        if (n < 10u) { return; }
        /* 0 id,1 utc,2 lat,3 N/S,4 lon,5 E/W,6 qual,7 sats,8 hdop,9 alt,10 M */
        gps.utc_hhmmss  = (uint32_t)strtoul(f[1], NULL, 10);
        gps.fix_quality = (uint8_t)atoi(f[6]);
        gps.sats        = (uint8_t)atoi(f[7]);
        gps.hdop        = (f[8][0] != '\0') ? (float)nmea_num(f[8]) : 99.9f;
        if (gps.fix_quality > 0u) {
            gps.lat_deg   = nmea_deg(f[2], f[3]);
            gps.lon_deg   = nmea_deg(f[4], f[5]);
            gps.alt_msl_m = (float)nmea_num(f[9]);
        }
        gps.last_gga_ms = HAL_GetTick();
        gps.gga++;
    } else if (strncmp(type, "RMC", 3) == 0) {
        uint32_t n = split(s, f, 24u);
        if (n < 9u) { return; }
        /* 0 id,1 utc,2 status,3 lat,4 N/S,5 lon,6 E/W,7 sog kn,8 cog */
        gps.rmc_valid  = (f[2][0] == 'A');
        gps.speed_mps  = (f[7][0] != '\0') ? (float)nmea_num(f[7]) * 0.514444f : 0.0f;   /* knots -> m/s */
        gps.course_deg = (f[8][0] != '\0') ? (float)nmea_num(f[8]) : 0.0f;
        gps.rmc++;
    }
}

void GPS_Poll(uint32_t max_bytes)
{
    gps.overrun = isr_overrun;
    for (uint32_t i = 0u; i < max_bytes; i++) {
        if (rb_tail == rb_head) { break; }
        char c = (char)rb[rb_tail];
        rb_tail = (rb_tail + 1u) & (RB_SIZE - 1u);
        gps.bytes++;

        if (c == '$') { in_sentence = true; line_len = 0u; continue; }
        if (!in_sentence) { continue; }
        if (c == '\r' || c == '\n') {
            if (line_len > 0u) { line[line_len] = '\0'; handle_line(line, line_len); }
            in_sentence = false; line_len = 0u;
            continue;
        }
        if (line_len < (LINE_MAX - 1u)) { line[line_len++] = c; }
        else { in_sentence = false; line_len = 0u; gps.cksum_err++; }   /* too long */
    }
}

bool GPS_Alive(uint32_t timeout_ms)
{
    return (gps.sentences > 0u) && ((HAL_GetTick() - gps.last_rx_ms) < timeout_ms);
}

bool GPS_HasFix(uint32_t timeout_ms)
{
    return (gps.gga > 0u) && ((HAL_GetTick() - gps.last_gga_ms) < timeout_ms)
           && (gps.fix_quality > 0u);
}
