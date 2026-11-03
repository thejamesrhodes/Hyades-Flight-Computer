/* ============================================================================
 * pwm.c  —  Servo PWM driver
 *
 * TIM2/TIM3 are configured by CubeMX as 32-bit/16-bit up-counters; this driver
 * sets PSC for a 1 MHz tick and ARR for the servo frame, then drives compare
 * registers directly. APB1 timer clock on this board is 96 MHz -> PSC=95.
 * No logging here (keeps the actuator path I/O-free aside from the timer regs).
 * ============================================================================ */
#include "pwm.h"
#include "board.h"

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
    oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = SERVO_CENTRE_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH; oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4);

    __HAL_TIM_SET_PRESCALER(&htim2, 95u);
    __HAL_TIM_SET_AUTORELOAD(&htim2, SERVO_PERIOD_US - 1u);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    __HAL_TIM_SET_PRESCALER(&htim3, 95u);
    __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_PERIOD_US - 1u);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    for (int i = 0; i < PWM_CH_COUNT; i++) { PWM_SetPulse((PWM_Channel)i, SERVO_CENTRE_US); }
}

void PWM_SetPulse(PWM_Channel ch, uint32_t pulse_us)
{
    if (ch >= PWM_CH_COUNT) { return; }
    if (pulse_us < SERVO_MIN_US) { pulse_us = SERVO_MIN_US; }
    if (pulse_us > SERVO_MAX_US) { pulse_us = SERVO_MAX_US; }
    __HAL_TIM_SET_COMPARE(pwm_map[ch].htim, pwm_map[ch].ch, pulse_us);
}

void PWM_SetAngle(PWM_Channel ch, float deg)
{
    if (deg < 0.0f) { deg = 0.0f; }
    if (deg > SERVO_RANGE_DEG) { deg = SERVO_RANGE_DEG; }
    PWM_SetPulse(ch, (uint32_t)((float)SERVO_MIN_US +
        (deg / SERVO_RANGE_DEG) * (float)(SERVO_MAX_US - SERVO_MIN_US)));
}

void PWM_SetNormalized(PWM_Channel ch, float v)
{
    if (v < -1.0f) { v = -1.0f; }
    if (v >  1.0f) { v =  1.0f; }
    PWM_SetPulse(ch, (uint32_t)((float)SERVO_CENTRE_US +
        v * (float)(SERVO_MAX_US - SERVO_CENTRE_US)));
}

/* Fin command: normalised [-1,+1] about SERVO_CENTRE_US, +/-1 = +/-FIN_DEFLECT_DEG. */
void PWM_SetFin(uint8_t fin_idx, float norm)
{
    if (fin_idx >= FIN_COUNT) { return; }
    if (norm < -1.0f) { norm = -1.0f; }
    if (norm >  1.0f) { norm =  1.0f; }
    const float us_per_deg = (float)(SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG;
    PWM_SetPulse(fin_chan[fin_idx],
                 (uint32_t)((float)SERVO_CENTRE_US + norm * FIN_DEFLECT_DEG * us_per_deg + 0.5f));
}

/* Integer-only so it is also safe to call from fault handlers. */
void PWM_CentreFins(void)
{
    for (uint8_t i = 0u; i < FIN_COUNT; i++) { PWM_SetPulse(fin_chan[i], SERVO_CENTRE_US); }
}
