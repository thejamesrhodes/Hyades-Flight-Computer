/* ============================================================================
 * gps.c  —  USART1 receive ring + NMEA parser glue (see gps.h)
 * ============================================================================ */
#include "gps.h"
#include <string.h>

#define RB_SIZE 512u                 /* power of two; ~0.5 s at 9600 bps */

static GpsData           s_gps;
static NmeaParser        s_parser;
static volatile uint8_t  s_rb[RB_SIZE];
static volatile uint32_t s_head, s_tail;      /* head: ISR, tail: main loop */
static volatile uint32_t s_overrun;

/* ISR: capture byte, exit (std B3). SR then DR read also clears ORE/FE/NE. */
void USART1_IRQHandler(void)
{
    const uint32_t sr = USART1->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
        const uint8_t c = (uint8_t)USART1->DR;
        if ((sr & USART_SR_ORE) != 0u) { s_overrun++; }
        if ((sr & USART_SR_RXNE) != 0u) {
            const uint32_t next = (s_head + 1u) & (RB_SIZE - 1u);
            if (next != s_tail) { s_rb[s_head] = c; s_head = next; }
            else                { s_overrun++; }
        }
    }
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    Nmea_Init(&s_parser, &s_gps);
    s_head = s_tail = 0u; s_overrun = 0u;

    /* PA15 (TX) released to a pulled-up input: nothing is ever sent, and if the
     * harness is crossed the module's TX drives an input, not our output. PA10
     * pulled up so an unplugged line idles high instead of floating. */
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_15; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
    gi.Pin = GPIO_PIN_10; gi.Mode = GPIO_MODE_AF_PP; gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW; gi.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gi);

    (void)huart->Instance->SR; (void)huart->Instance->DR;      /* flush */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);    /* below USB (0); above SysTick (15) */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void GPS_Poll(uint32_t max_bytes)
{
    const uint32_t now = HAL_GetTick();
    for (uint32_t i = 0u; (i < max_bytes) && (s_tail != s_head); i++) {
        const char c = (char)s_rb[s_tail];
        s_tail = (s_tail + 1u) & (RB_SIZE - 1u);
        Nmea_Feed(&s_parser, &s_gps, c, now);
    }
}

const GpsData *GPS_Get(void)      { return &s_gps; }
uint32_t       GPS_Overruns(void) { return s_overrun; }
bool GPS_Alive(uint32_t timeout_ms)  { return Nmea_Alive(&s_gps, HAL_GetTick(), timeout_ms); }
bool GPS_HasFix(uint32_t timeout_ms) { return Nmea_HasFix(&s_gps, HAL_GetTick(), timeout_ms); }

void GPS_LineActivity(uint32_t window_ms, uint32_t *edges_pa10, uint32_t *edges_pa15)
{
    uint32_t n10 = 0u, n15 = 0u;
    uint32_t prev = GPIOA->IDR;                  /* IDR samples the pin in AF mode too */
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < window_ms) {   /* ~MHz polling >> 9.6 kbit/s */
        const uint32_t now = GPIOA->IDR;
        const uint32_t chg = now ^ prev;
        if ((chg & GPIO_PIN_10) != 0u) { n10++; }
        if ((chg & GPIO_PIN_15) != 0u) { n15++; }
        prev = now;
    }
    *edges_pa10 = n10;
    *edges_pa15 = n15;
}
