/* ============================================================================
 * gps.c  —  USART2 receive ring + boot line probe + NMEA glue (see gps.h)
 * ============================================================================ */
#include "gps.h"
#include "baud.h"
#include <string.h>

#define RB_SIZE  2048u               /* power of two; ~2 s at 9600 Bd: covers the blocking boot calibration */
#define PIN_RX   GPIO_PIN_3          /* PA3 USART2_RX <- module TX */
#define PIN_TX   GPIO_PIN_2          /* PA2 USART2_TX (unused)     */
#define PROBE_K  4u                  /* bit time = 4th-shortest interval (see probe) */

static GpsData           s_gps;
static NmeaParser        s_parser;
static GpsProbe          s_probe;
static volatile uint8_t  s_rb[RB_SIZE];
static volatile uint32_t s_head, s_tail;      /* head: ISR, tail: main loop */
static volatile uint32_t s_overrun;

static char     s_raw[GPS_RAW_LINES][NMEA_LINE_MAX];
static uint32_t s_raw_next, s_raw_count;
static char     s_cur[NMEA_LINE_MAX];
static uint32_t s_cur_len;

/* ISR: capture byte, exit (std B3). SR then DR read also clears ORE/FE/NE. */
void USART2_IRQHandler(void)
{
    const uint32_t sr = USART2->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
        const uint8_t c = (uint8_t)USART2->DR;
        if ((sr & USART_SR_ORE) != 0u) { s_overrun++; }
        if ((sr & USART_SR_RXNE) != 0u) {
            const uint32_t next = (s_head + 1u) & (RB_SIZE - 1u);
            if (next != s_tail) { s_rb[s_head] = c; s_head = next; }
            else                { s_overrun++; }
        }
    }
}

static void pins_input(uint32_t pull)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin = PIN_RX | PIN_TX; gi.Mode = GPIO_MODE_INPUT; gi.Pull = pull; gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gi);
}

static uint8_t level(uint32_t up, uint32_t dn, uint16_t pin)
{
    const bool hu = (up & pin) != 0u, hd = (dn & pin) != 0u;
    return (hu && !hd) ? LINE_FLOAT : (hu ? LINE_HIGH : LINE_LOW);
}

/* Rest levels (a line that follows the pull is floating), then edges and the
 * bit time, polling IDR against the cycle counter (~0.3 us vs a 104 us bit).
 * The 4th-shortest interval is used: a second of NMEA has hundreds of 1-bit
 * pulses, so a few spurious short ones cannot halve the estimate. */
static void probe(uint32_t window_ms)
{
    pins_input(GPIO_PULLUP);   HAL_Delay(2);
    const uint32_t up = GPIOA->IDR;
    pins_input(GPIO_PULLDOWN); HAL_Delay(2);
    const uint32_t dn = GPIOA->IDR;
    s_probe.rx_level = level(up, dn, PIN_RX);
    s_probe.tx_level = level(up, dn, PIN_TX);

    uint32_t minp[PROBE_K], last = 0u;
    for (uint32_t k = 0u; k < PROBE_K; k++) { minp[k] = 0xFFFFFFFFu; }
    const uint32_t glitch = SystemCoreClock / 500000u;       /* ignore < 2 us */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    pins_input(GPIO_PULLUP);   HAL_Delay(2);
    uint32_t prev = GPIOA->IDR;
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < window_ms) {
        const uint32_t now = GPIOA->IDR, cyc = DWT->CYCCNT;
        const uint32_t chg = now ^ prev;
        prev = now;
        if ((chg & PIN_TX) != 0u) { s_probe.tx_edges++; }
        if ((chg & PIN_RX) == 0u) { continue; }
        if (s_probe.rx_edges > 0u) {
            uint32_t w = cyc - last;
            if (w >= glitch) {                       /* insert into the sorted k smallest */
                for (uint32_t k = 0u; k < PROBE_K; k++) {
                    if (w < minp[k]) { const uint32_t t = minp[k]; minp[k] = w; w = t; }
                }
            }
        }
        last = cyc;
        s_probe.rx_edges++;
    }
    s_probe.rx_baud = (s_probe.rx_edges >= 20u) ? Baud_Snap(SystemCoreClock, minp[PROBE_K - 1u]) : 0u;
}

void GPS_Init(uint32_t probe_window_ms)
{
    Nmea_Init(&s_parser, &s_gps);
    s_head = s_tail = 0u; s_overrun = 0u;
    memset(&s_probe, 0, sizeof s_probe);
    probe(probe_window_ms);

    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = PIN_RX; gi.Mode = GPIO_MODE_AF_PP; gi.Pull = GPIO_PULLUP;   /* unplugged idles high */
    gi.Speed = GPIO_SPEED_FREQ_LOW; gi.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gi);
    gi.Pin = PIN_TX; gi.Mode = GPIO_MODE_INPUT;    /* never driven: a crossed harness meets an input */
    HAL_GPIO_Init(GPIOA, &gi);

    USART2->CR1 = 0u; USART2->CR2 = 0u; USART2->CR3 = 0u;
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + (GPS_BAUD / 2u)) / GPS_BAUD;    /* oversampling x16 */
    (void)USART2->SR; (void)USART2->DR;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);   /* below USB (0); above SysTick (15) */
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

const GpsProbe *GPS_ProbeResult(void) { return &s_probe; }
const char *GPS_LevelName(uint8_t l)  { return (l == LINE_HIGH) ? "high" : ((l == LINE_LOW) ? "LOW" : "float"); }

const char *GPS_Advice(void)
{
    const GpsProbe *p = &s_probe;
    if (p->rx_edges >= 20u) {
        if (p->rx_baud == GPS_BAUD) { return "module transmitting"; }
        return (p->rx_baud != 0u) ? "module at a different baud: set GPS_BAUD or reconfigure it"
                                  : "traffic but baud not recognised (noise?)";
    }
    if (p->tx_edges >= 20u)          { return "traffic only on PA2: module TX/RX crossed, swap the wires"; }
    if (p->rx_level == LINE_LOW)     { return "PA3 held low: module unpowered or shorted"; }
    if (p->rx_level == LINE_HIGH)    { return "PA3 high but silent: module powered, not sending (check its LED)"; }
    return "PA3 floating: nothing connected to UART header 2 (plug, harness, GND)";
}

static void raw_push(char c)
{
    if (c == '$') { s_cur_len = 0u; }
    if ((c == '\r') || (c == '\n')) {
        if ((s_cur_len > 0u) && (s_cur[0] == '$')) {
            s_cur[s_cur_len] = '\0';
            memcpy(s_raw[s_raw_next], s_cur, s_cur_len + 1u);
            s_raw_next = (s_raw_next + 1u) % GPS_RAW_LINES;
            if (s_raw_count < GPS_RAW_LINES) { s_raw_count++; }
        }
        s_cur_len = 0u;
        return;
    }
    if ((c >= ' ') && (c < 0x7F) && (s_cur_len < (NMEA_LINE_MAX - 1u))) { s_cur[s_cur_len++] = c; }
}

void GPS_Poll(uint32_t max_bytes)
{
    const uint32_t now = HAL_GetTick();
    for (uint32_t i = 0u; (i < max_bytes) && (s_tail != s_head); i++) {
        const char c = (char)s_rb[s_tail];
        s_tail = (s_tail + 1u) & (RB_SIZE - 1u);
        raw_push(c);
        Nmea_Feed(&s_parser, &s_gps, c, now);
    }
}

uint32_t GPS_RawLines(const char *out[GPS_RAW_LINES])
{
    const uint32_t start = (s_raw_next + GPS_RAW_LINES - s_raw_count) % GPS_RAW_LINES;
    for (uint32_t k = 0u; k < s_raw_count; k++) { out[k] = s_raw[(start + k) % GPS_RAW_LINES]; }
    return s_raw_count;
}

const GpsData *GPS_Get(void)      { return &s_gps; }
uint32_t       GPS_Overruns(void) { return s_overrun; }
bool GPS_Alive(uint32_t timeout_ms)  { return Nmea_Alive(&s_gps, HAL_GetTick(), timeout_ms); }
bool GPS_HasFix(uint32_t timeout_ms) { return Nmea_HasFix(&s_gps, HAL_GetTick(), timeout_ms); }
