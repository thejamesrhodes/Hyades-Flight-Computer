/* ============================================================================
 * gps.c  —  Dual-port UART receive ring + line probe + NMEA glue (see gps.h)
 * ============================================================================ */
#include "gps.h"
#include "baud.h"
#include <string.h>

#define RB_SIZE 2048u                /* power of two; ~2 s at 9600 bps: covers the blocking boot calibration */

typedef struct {
    USART_TypeDef *u;
    uint16_t rx, tx;                 /* GPIOA pins */
    IRQn_Type irq;
} Port;

static const Port k_port[2] = {
    { USART1, GPIO_PIN_10, GPIO_PIN_15, USART1_IRQn },
    { USART2, GPIO_PIN_3,  GPIO_PIN_2,  USART2_IRQn },
};
static const uint16_t k_pin[GPS_NPINS] = { GPIO_PIN_10, GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_2 };
static const char *const k_pin_name[GPS_NPINS] = { "PA10 rx1", "PA15 tx1", "PA3 rx2", "PA2 tx2" };

static GpsData           s_gps;
static NmeaParser        s_parser;
static GpsProbe          s_probe;
static uint32_t          s_baud[2];
static volatile uint8_t  s_rb[RB_SIZE];
static volatile uint32_t s_head, s_tail;      /* head: ISRs, tail: main loop */
static volatile uint32_t s_overrun;
static volatile uint32_t s_port_bytes[2];

/* Runtime baud hunt: if a port receives HUNT_BYTES without one valid
 * sentence, try the next standard rate. Self-corrects a wrong probe guess or a
 * module reconfigured to another rate; settles once NMEA parses. */
#define HUNT_BYTES 400u
static const uint32_t k_hunt[] = { 9600u, 38400u, 115200u, 4800u, 19200u, 57600u };
static uint32_t s_hunt_bytes0[2], s_hunt_sent0[2], s_hunt_idx[2], s_retunes;

static char     s_raw[GPS_RAW_LINES][NMEA_LINE_MAX];
static uint32_t s_raw_next, s_raw_count;
static char     s_cur[NMEA_LINE_MAX];
static uint32_t s_cur_len;

/* ISR body: capture byte, exit (std B3). SR then DR read also clears ORE/FE/NE.
 * Both ports share the ring; they never run at the same time in practice, and
 * the USART1 and USART2 IRQs have the same priority so they cannot nest. */
static void rx_isr(USART_TypeDef *u, uint32_t idx)
{
    const uint32_t sr = u->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
        const uint8_t c = (uint8_t)u->DR;
        if ((sr & USART_SR_ORE) != 0u) { s_overrun++; }
        if ((sr & USART_SR_RXNE) != 0u) {
            s_port_bytes[idx]++;
            const uint32_t next = (s_head + 1u) & (RB_SIZE - 1u);
            if (next != s_tail) { s_rb[s_head] = c; s_head = next; }
            else                { s_overrun++; }
        }
    }
}
void USART1_IRQHandler(void) { rx_isr(USART1, 0u); }
void USART2_IRQHandler(void) { rx_isr(USART2, 1u); }

static void pins_input(uint32_t pull)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_10 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_2;
    gi.Mode = GPIO_MODE_INPUT; gi.Pull = pull; gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gi);
}

/* Pull-up then pull-down read: a line that follows the pull is floating
 * (nothing driving it); one that does not is held by the module. */
static void probe_levels(void)
{
    pins_input(GPIO_PULLUP);   HAL_Delay(2);
    const uint32_t up = GPIOA->IDR;
    pins_input(GPIO_PULLDOWN); HAL_Delay(2);
    const uint32_t dn = GPIOA->IDR;
    for (uint32_t i = 0u; i < GPS_NPINS; i++) {
        const bool hu = (up & k_pin[i]) != 0u, hd = (dn & k_pin[i]) != 0u;
        s_probe.level[i] = (hu && !hd) ? LINE_FLOAT : (hu ? LINE_HIGH : LINE_LOW);
    }
}

/* Edge count and bit time per pin, polling IDR against the cycle counter
 * (~0.3 us resolution vs a 104 us bit at 9600 Bd). The bit time is taken as
 * the 4th-shortest interval: a second of NMEA has hundreds of single-bit
 * pulses, so up to three spurious short ones (a polling coincidence, a slow
 * edge) cannot halve the estimate. */
#define PROBE_K 4u
static void probe_activity(uint32_t window_ms)
{
    uint32_t minp[GPS_NPINS][PROBE_K], last[GPS_NPINS];
    const uint32_t glitch = SystemCoreClock / 500000u;    /* ignore < 2 us */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (uint32_t i = 0u; i < GPS_NPINS; i++) {
        for (uint32_t k = 0u; k < PROBE_K; k++) { minp[i][k] = 0xFFFFFFFFu; }
        last[i] = 0u; s_probe.edges[i] = 0u;
    }

    pins_input(GPIO_PULLUP);
    HAL_Delay(2);
    uint32_t prev = GPIOA->IDR;
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < window_ms) {
        const uint32_t now = GPIOA->IDR, cyc = DWT->CYCCNT;
        const uint32_t chg = now ^ prev;
        prev = now;
        if (chg == 0u) { continue; }
        for (uint32_t i = 0u; i < GPS_NPINS; i++) {
            if ((chg & k_pin[i]) == 0u) { continue; }
            if (s_probe.edges[i] > 0u) {
                uint32_t w = cyc - last[i];
                if (w >= glitch) {                       /* insert into the sorted k smallest */
                    for (uint32_t k = 0u; k < PROBE_K; k++) {
                        if (w < minp[i][k]) { const uint32_t t = minp[i][k]; minp[i][k] = w; w = t; }
                    }
                }
            }
            last[i] = cyc;
            s_probe.edges[i]++;
        }
    }
    for (uint32_t i = 0u; i < GPS_NPINS; i++) {
        s_probe.baud[i] = (s_probe.edges[i] >= 20u) ? Baud_Snap(SystemCoreClock, minp[i][PROBE_K - 1u]) : 0u;
    }
}

static void port_start(uint32_t idx, uint32_t baud)
{
    const Port *p = &k_port[idx];
    GPIO_InitTypeDef gi = {0};
    if (idx == 0u) { __HAL_RCC_USART1_CLK_ENABLE(); } else { __HAL_RCC_USART2_CLK_ENABLE(); }
    /* RX: AF7, pulled up so an unplugged line idles high instead of floating.
     * TX: never driven — left a pulled-up input, so a crossed harness puts the
     * module's TX onto an input, not against our output. */
    gi.Pin = p->rx; gi.Mode = GPIO_MODE_AF_PP; gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW; gi.Alternate = GPIO_AF7_USART1;   /* AF7 = USART1/2 */
    HAL_GPIO_Init(GPIOA, &gi);
    gi.Pin = p->tx; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);

    const uint32_t pclk = (idx == 0u) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    p->u->CR1 = 0u;
    p->u->CR2 = 0u;
    p->u->CR3 = 0u;
    p->u->BRR = (pclk + (baud / 2u)) / baud;                /* oversampling x16 */
    (void)p->u->SR; (void)p->u->DR;                         /* flush */
    p->u->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    s_baud[idx] = baud;
    s_hunt_bytes0[idx] = s_port_bytes[idx];
    s_hunt_sent0[idx] = s_gps.sentences;
    HAL_NVIC_SetPriority(p->irq, 6, 0);    /* below USB (0); above SysTick (15) */
    HAL_NVIC_EnableIRQ(p->irq);
}

void GPS_Init(uint32_t probe_window_ms)
{
    Nmea_Init(&s_parser, &s_gps);
    s_head = s_tail = 0u; s_overrun = 0u;
    memset(&s_probe, 0, sizeof s_probe);

    probe_levels();
    probe_activity(probe_window_ms);

    const uint32_t e1 = s_probe.edges[0], e2 = s_probe.edges[2];
    s_probe.port = (e1 >= 20u || e2 >= 20u) ? ((e2 > e1) ? 2u : 1u) : 0u;
    s_probe.crossed = (s_probe.port == 0u) && ((s_probe.edges[1] >= 20u) || (s_probe.edges[3] >= 20u));

    port_start(0u, (s_probe.baud[0] != 0u) ? s_probe.baud[0] : 9600u);
    port_start(1u, (s_probe.baud[2] != 0u) ? s_probe.baud[2] : 9600u);
}

const GpsProbe *GPS_ProbeResult(void) { return &s_probe; }
const char *GPS_PinName(uint32_t i)   { return (i < GPS_NPINS) ? k_pin_name[i] : "?"; }
const char *GPS_LevelName(uint8_t l)  { return (l == LINE_HIGH) ? "high" : ((l == LINE_LOW) ? "LOW" : "float"); }

const char *GPS_Advice(void)
{
    const GpsProbe *p = &s_probe;
    if (p->port != 0u) {
        const uint32_t b = p->baud[(p->port == 1u) ? 0u : 2u];
        return (b != 0u) ? "module transmitting" : "traffic but baud not recognised (noise?)";
    }
    if (p->crossed)                  { return "traffic only on a TX pin: module TX/RX crossed, swap the wires"; }
    if ((p->level[0] == LINE_LOW) || (p->level[2] == LINE_LOW)) {
        return "an RX line is held low: module unpowered (TX pulled low through it) or shorted";
    }
    if ((p->level[0] == LINE_HIGH) || (p->level[2] == LINE_HIGH)) {
        return "RX held high but silent: module powered and connected but not sending (antenna/boot? check its LED)";
    }
    return "both RX lines floating: nothing connected to either GPS port (plug, harness, GND)";
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

static void hunt(uint32_t idx)
{
    if (s_gps.sentences != s_hunt_sent0[idx]) {            /* parsing: this rate is right */
        s_hunt_bytes0[idx] = s_port_bytes[idx];
        s_hunt_sent0[idx] = s_gps.sentences;
        return;
    }
    if ((s_port_bytes[idx] - s_hunt_bytes0[idx]) < HUNT_BYTES) { return; }
    uint32_t k = 0u;                                        /* next rate after the current one */
    const uint32_t n = (uint32_t)(sizeof k_hunt / sizeof k_hunt[0]);
    for (uint32_t j = 0u; j < n; j++) { if (k_hunt[j] == s_baud[idx]) { k = j + 1u; } }
    s_hunt_idx[idx] = k % n;
    s_retunes++;
    port_start(idx, k_hunt[s_hunt_idx[idx]]);
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
    hunt(0u);
    hunt(1u);
}

uint32_t GPS_Retunes(void) { return s_retunes; }

uint32_t GPS_RawLines(const char *out[GPS_RAW_LINES])
{
    const uint32_t start = (s_raw_next + GPS_RAW_LINES - s_raw_count) % GPS_RAW_LINES;
    for (uint32_t k = 0u; k < s_raw_count; k++) { out[k] = s_raw[(start + k) % GPS_RAW_LINES]; }
    return s_raw_count;
}

const GpsData *GPS_Get(void)            { return &s_gps; }
uint32_t       GPS_Overruns(void)       { return s_overrun; }
uint32_t       GPS_PortBytes(uint32_t p){ return ((p == 1u) || (p == 2u)) ? s_port_bytes[p - 1u] : 0u; }
uint32_t       GPS_PortBaud(uint32_t p) { return ((p == 1u) || (p == 2u)) ? s_baud[p - 1u] : 0u; }
bool GPS_Alive(uint32_t timeout_ms)  { return Nmea_Alive(&s_gps, HAL_GetTick(), timeout_ms); }
bool GPS_HasFix(uint32_t timeout_ms) { return Nmea_HasFix(&s_gps, HAL_GetTick(), timeout_ms); }
