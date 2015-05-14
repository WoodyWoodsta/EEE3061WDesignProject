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

  TIM_DeInit(TIM1);
  TIM_DeInit(TIM15);

  //Initialise the Enable pins
  GPIO_InitTypeDef mtr_GPIOENInitStruct;
  mtr_GPIOENInitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  mtr_GPIOENInitStruct.GPIO_Mode = GPIO_Mode_OUT;
  mtr_GPIOENInitStruct.GPIO_OType = GPIO_OType_PP;
  mtr_GPIOENInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  mtr_GPIOENInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOB, &mtr_GPIOENInitStruct);

  // Double check they are low
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
  GPIO_ResetBits(GPIOB, GPIO_Pin_7);

  // Initialise the PWM GPIO pins
  GPIO_InitTypeDef mtr_GPIOPWMFInitStruct;
  mtr_GPIOPWMFInitStruct.GPIO_Mode = GPIO_Mode_AF; // Alternate Function for timer PWM output
  mtr_GPIOPWMFInitStruct.GPIO_OType = GPIO_OType_PP;
  mtr_GPIOPWMFInitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_11;
  mtr_GPIOPWMFInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOA, &mtr_GPIOPWMFInitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

  // Initialise the PWM TIM1
  TIM_TimeBaseInitTypeDef mtr_TIM1TimebaseInitStruct;
  TIM_TimeBaseStructInit(&mtr_TIM1TimebaseInitStruct); // Get the structure ready for init
  mtr_TIM1TimebaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  mtr_TIM1TimebaseInitStruct.TIM_Prescaler = 48; // Will give 1Mhz base
  mtr_TIM1TimebaseInitStruct.TIM_Period = 1000; // 1KHz PWM frequency
  TIM_TimeBaseInit(TIM1, &mtr_TIM1TimebaseInitStruct);

  TIM_OCInitTypeDef mtr_TIM1OCInitStruct;
  mtr_TIM1OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  mtr_TIM1OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2; // With upcounting, this gives positive leading pulse and directly proportional duty cycle
  mtr_TIM1OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // Enable the output?
  TIM_OC1Init(TIM1, &mtr_TIM1OCInitStruct); // Channel one output compare init
  TIM_OC4Init(TIM1, &mtr_TIM1OCInitStruct); // Channel two output compare init

}
