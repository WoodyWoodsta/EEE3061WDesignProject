/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | pwmMotor_lib.c
 * @brief     | Motor Control Library (PWM)
 * @pre       | mtr
 * @authors   | Team 13
 *
 * This library is used for control of two motors in both directions via PWM
 * to a quad-h-bridge (L293DNE). NOTE: Protection against shoot-through is
 * critical!
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "pwmMotor_lib.h"

// == Defines ==


// == Global Variables ==


// == Declarations ==



/**
 * @brief Initialise the timers for motor control
 * @param None
 * @retval None
 */

void mtr_motorTimInit(void) {
  // Left Motor Forward: PA11 - TIM1_CH4
  // Left Motor Reverse: PA3 - TIM15_CH2
  // Left Motor Enable: PB6
  // Right Motor Forward: PA8 - TIM1_CH1
  // Right Motor Reverse: PA2 - TIM15_CH1
  // Right Motor Enable: PB7

  // Gather the RCC clocks for the timers we need
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);


}
