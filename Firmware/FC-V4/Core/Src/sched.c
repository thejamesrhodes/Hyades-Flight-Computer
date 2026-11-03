/* ============================================================================
 * sched.c  —  Cyclic-executive timebase (TIM5, 1 MHz, 32-bit free-run)
 *
 * TIM5 is a 32-bit general-purpose timer on the F411 and is otherwise unused
 * (TIM2/TIM3 drive the servos). APB1 timer clock on this board is 96 MHz
 * (APB1 = 48 MHz, x2 for the timer domain), so PSC = 96-1 gives a 1 MHz tick.
 * ============================================================================ */
#include "sched.h"
#include "board.h"

void Sched_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* Direct register config: free-running, no interrupt, no HAL handle.
     * PSC : 96 MHz / (95+1) = 1 MHz tick.
     * ARR : full 32-bit range -> free running. */
    TIM5->CR1  = 0u;            /* disabled while configuring                  */
    /* APB1 timer clock = PCLK1, doubled unless the APB1 prescaler is /1 */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t timclk = (((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) < 4u)
                      ? pclk1 : (pclk1 * 2u);
    TIM5->PSC = (uint16_t)((timclk / 1000000u) - 1u);   /* -> 1 MHz tick */
    TIM5->ARR  = 0xFFFFFFFFu;
    TIM5->CNT  = 0u;
    TIM5->EGR  = TIM_EGR_UG;    /* latch PSC/ARR                               */
    TIM5->SR   = 0u;            /* clear the update flag raised by UG          */
    TIM5->DIER = 0u;            /* no interrupts                               */
    TIM5->CR1  = TIM_CR1_CEN;   /* enable                                      */
}

void Sched_WaitUntil(uint32_t deadline_us)
{
    /* Signed delta: negative => deadline still in the future. Bounded because
     * the counter advances every microsecond. */
    while ((int32_t)(TIM5->CNT - deadline_us) < 0) {
        __NOP();
    }
}
