/* ============================================================================
 * pwm.c  —  Servo PWM driver
 *
 * TIM2/TIM3 are configured by CubeMX as 32-bit/16-bit up-counters; this driver
 * sets PSC for a PWM_TICK_HZ tick (board.h, ALLOC-01) and ARR for the servo
 * frame, then drives compare registers directly. APB1 timer clock 96 MHz.
 * Fin commands are quantised by Alloc_NormToTicks, the same function the
 * allocator and the SITL use, and offset by the per-fin mechanical zero.
 * No logging here (keeps the actuator path I/O-free aside from the timer regs).
 * ============================================================================ */
#include "pwm.h"
#include "board.h"
#include "alloc.h"
#include "fincal.h"
#include <math.h>

extern TIM_HandleTypeDef htim2, htim3;

typedef struct { TIM_HandleTypeDef *htim; uint32_t ch; } PwmChan;

static const PwmChan pwm_map[PWM_CH_COUNT] = {
    { &htim2, TIM_CHANNEL_1 },   /* CH1  PA5  */
    { &htim3, TIM_CHANNEL_1 },   /* CH2  PA6  */
    { &htim3, TIM_CHANNEL_2 },   /* CH3  PA7  (fin 0) */
    { &htim3, TIM_CHANNEL_3 },   /* CH4  PB0  (fin 1) */
    { &htim3, TIM_CHANNEL_4 },   /* CH5  PB1  (fin 2) */
    { &htim2, TIM_CHANNEL_3 },   /* CH6  PB10 (fin 3) */
};

/* Fin index 0..3 -> PWM_CH3..PWM_CH6. */
static const PWM_Channel fin_chan[FIN_COUNT] = { PWM_CH3, PWM_CH4, PWM_CH5, PWM_CH6 };

#define PWM_PSC           ((PWM_TIM_CLK_HZ / PWM_TICK_HZ) - 1u)
#define PWM_ARR           ((SERVO_PERIOD_US * PWM_TICKS_PER_US) - 1u)
#define PWM_CENTRE_TICKS  ((int32_t)(SERVO_CENTRE_US * PWM_TICKS_PER_US))
#define PWM_MIN_TICKS     (SERVO_MIN_US * PWM_TICKS_PER_US)
#define PWM_MAX_TICKS     (SERVO_MAX_US * PWM_TICKS_PER_US)
_Static_assert((PWM_TIM_CLK_HZ % PWM_TICK_HZ) == 0u, "PWM tick must divide the timer clock");
_Static_assert((PWM_TICK_HZ % 1000000u) == 0u, "PWM tick must be a whole number of ticks per us");
_Static_assert(PWM_ARR <= 65535u, "TIM3 is 16-bit: servo period too long for this tick");
#define SERVO_RANGE_DEG_INT   ((int32_t)SERVO_RANGE_DEG)   /* 180: a whole number of degrees */
_Static_assert(FIN_COUNT == FINCAL_FINS, "one calibrated zero per fin");
_Static_assert(PWM_TICKS_PER_US <= 16u, "zero conversion overflows int32 above 16 ticks/us");
_Static_assert(FIN_ZERO_FULL_AUTH_MDEG == ((((int32_t)SERVO_RANGE_DEG / 2) - (int32_t)FIN_DEFLECT_DEG) * 1000),
               "full-authority zero limit = half the servo range minus the command authority");

/* Per-fin mechanical zero in ticks (CAL-01). Integer, so PWM_CentreFins stays
 * safe to call from fault handlers. Written only through PWM_SetZeroMdeg. */
static int32_t fin_zero_ticks[FIN_COUNT];

void PWM_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_6 | GPIO_PIN_7; g.Alternate = GPIO_AF2_TIM3;  HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_5;              g.Alternate = GPIO_AF1_TIM2;  HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1; g.Alternate = GPIO_AF2_TIM3;  HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_10;             g.Alternate = GPIO_AF1_TIM2;  HAL_GPIO_Init(GPIOB, &g);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = (uint32_t)PWM_CENTRE_TICKS;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH; oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4);

    __HAL_TIM_SET_PRESCALER(&htim2, PWM_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim2, PWM_ARR);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    __HAL_TIM_SET_PRESCALER(&htim3, PWM_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_ARR);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    for (uint32_t i = 0u; i < FIN_COUNT; i++) { fin_zero_ticks[i] = 0; }   /* calibration applied by the app */
    for (uint32_t i = 0u; i < (uint32_t)PWM_CH_COUNT; i++) { PWM_SetPulse((PWM_Channel)i, SERVO_CENTRE_US); }
    PWM_CentreFins();
}

static void set_ticks(PWM_Channel ch, int32_t ticks)
{
    uint32_t t = (ticks > 0) ? (uint32_t)ticks : 0u;
    if (t < PWM_MIN_TICKS) { t = PWM_MIN_TICKS; }
    if (t > PWM_MAX_TICKS) { t = PWM_MAX_TICKS; }
    __HAL_TIM_SET_COMPARE(pwm_map[ch].htim, pwm_map[ch].ch, t);
}

void PWM_SetPulse(PWM_Channel ch, uint32_t pulse_us)
{
    if (ch >= PWM_CH_COUNT) { return; }
    if (pulse_us > SERVO_MAX_US) { pulse_us = SERVO_MAX_US; }   /* before the multiply: no overflow */
    set_ticks(ch, (int32_t)(pulse_us * PWM_TICKS_PER_US));
}

/* Fin command: normalised [-1,+1] about the fin's zero, +/-1 = +/-FIN_DEFLECT_DEG.
 * Quantised by Alloc_NormToTicks (clamps, rejects NaN, range-checked cast). */
void PWM_SetFin(uint8_t fin_idx, float norm)
{
    if (fin_idx >= FIN_COUNT) { return; }
    set_ticks(fin_chan[fin_idx], PWM_CENTRE_TICKS + fin_zero_ticks[fin_idx] + Alloc_NormToTicks(norm));
}

/* Millidegrees at the servo -> ticks, rounded: mdeg x 1200 us / 180 deg x
 * ticks/us / 1000. |mdeg| <= 90 000 and <= 16 ticks/us keep every product
 * inside int32 (90 000 x 1200 x 16 = 1.7e9). A zero near +/-90 deg leaves the
 * pulse at the end-stop clamp on that side: the app holds READY beyond
 * FIN_ZERO_FULL_AUTH_MDEG for that reason. */
void PWM_SetZeroMdeg(const int32_t mdeg[FIN_COUNT])
{
    const int32_t den = (SERVO_RANGE_DEG_INT * 1000);
    for (uint32_t i = 0u; i < FIN_COUNT; i++) {
        int32_t m = mdeg[i];
        if (m > FINCAL_LIMIT_MDEG) { m = FINCAL_LIMIT_MDEG; }
        if (m < -FINCAL_LIMIT_MDEG) { m = -FINCAL_LIMIT_MDEG; }
        const int32_t num = m * (int32_t)(SERVO_MAX_US - SERVO_MIN_US) * (int32_t)PWM_TICKS_PER_US;
        fin_zero_ticks[i] = (num >= 0) ? ((num + (den / 2)) / den) : -(((-num) + (den / 2)) / den);
    }
}

/* Integer-only so it is also safe to call from fault handlers. */
void PWM_CentreFins(void)
{
    for (uint8_t i = 0u; i < FIN_COUNT; i++) { set_ticks(fin_chan[i], PWM_CENTRE_TICKS + fin_zero_ticks[i]); }
}

/* Start a new servo pulse on every channel NOW with the compare values just
 * written (ADR-004 §2). The compare registers are preloaded (HAL PWM config
 * sets OCxPE), so the update event both resets the counters and latches the
 * new widths: command-to-pulse latency is a few cycles instead of up to one
 * PWM period, and the servo frame rate equals the control frame rate. If the
 * loop stops calling this (log erase, a stall) the counters free-run at
 * SERVO_PERIOD_US and the servos keep receiving the last valid pulse. TIM2
 * and TIM3 are updated back to back, so the fins start within ~0.1 us. */
void PWM_Commit(void)
{
#if SERVO_SYNC_TO_FRAME
    TIM2->EGR = TIM_EGR_UG;
    TIM3->EGR = TIM_EGR_UG;
#endif
}
