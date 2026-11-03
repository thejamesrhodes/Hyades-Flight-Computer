/* ============================================================================
 * pwm.h  —  Servo PWM (TIM2 + TIM3), cruciform fins
 *
 * Physical channel map (unchanged from the board):
 *   PWM_CH1 = PA5  (TIM2_CH1)     PWM_CH2 = PA6  (TIM3_CH1)
 *   PWM_CH3 = PA7  (TIM3_CH2)     PWM_CH4 = PB0  (TIM3_CH3)
 *   PWM_CH5 = PB1  (TIM3_CH4)     PWM_CH6 = PB10 (TIM2_CH3)
 *
 * The four fins are PWM_CH3..PWM_CH6, i.e. fin index 0..3 -> CH3..CH6.
 * Timing/limits come from board.h (single source of truth). CH1/CH2 are
 * configured (centred) but unused spare outputs.
 * ============================================================================ */
#ifndef PWM_H_
#define PWM_H_

#include "main.h"
#include <stdint.h>

typedef enum {
    PWM_CH1 = 0, PWM_CH2, PWM_CH3, PWM_CH4, PWM_CH5, PWM_CH6,
    PWM_CH_COUNT = 6
} PWM_Channel;

#define FIN_COUNT 4   /* fins occupy PWM_CH3..PWM_CH6 */

void PWM_Init(void);

void PWM_SetPulse(PWM_Channel ch, uint32_t pulse_us);   /* clamped MIN..MAX    */

/* Fin command: idx 0..3, norm in [-1,+1] -> centre +/- FIN_DEFLECT_DEG. */
void PWM_SetFin(uint8_t fin_idx, float norm);

/* Centre all fins (failsafe / inactive). */
void PWM_CentreFins(void);

#endif /* PWM_H_ */
