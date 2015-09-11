/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | ledGPIOB_lib.c
 * @brief     | LED Library (GPIOB)
 * @pre       | gyr
 * @authors   | Team 13
 *
 * This library is for controlling the LEDs that are on GPIOB 0-7 on James
 * Gowans' STM32F0 Development Board.
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "ledGPIOB_lib.h"
#include "stm32f0xx.h"

// == Defines ==

// == Declarations ==

/**
 * @brief Initialise the pins for the LEDs
 * @param None
 * @retval None
 */

void led_init(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Initialise the LED Strip PWM (PB8 - TIM16_CH2)
  GPIO_InitTypeDef led_LEDStripInitStruct;
  led_LEDStripInitStruct.GPIO_Mode = GPIO_Mode_AF; // Alternate Function for timer PWM output
  led_LEDStripInitStruct.GPIO_OType = GPIO_OType_PP;
  led_LEDStripInitStruct.GPIO_Pin = GPIO_Pin_8;
  led_LEDStripInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  led_LEDStripInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &led_LEDStripInitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);

  // Initialise the PWM for TIM16
  TIM_TimeBaseInitTypeDef led_LEDStripTIMTimebaseInitStruct;
  TIM_TimeBaseStructInit(&led_LEDStripTIMTimebaseInitStruct); // Get the structure ready for init
  led_LEDStripTIMTimebaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  led_LEDStripTIMTimebaseInitStruct.TIM_Prescaler = 480; // Will give 1Mhz base
  led_LEDStripTIMTimebaseInitStruct.TIM_Period = 1000; // 1KHz PWM frequency
  TIM_TimeBaseInit(TIM16, &led_LEDStripTIMTimebaseInitStruct);

  TIM_OCInitTypeDef led_LEDStripTIMOCInitStruct;
  TIM_OCStructInit(&led_LEDStripTIMOCInitStruct); // These are here to set everything to default to make the param assertion happy
  led_LEDStripTIMOCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  led_LEDStripTIMOCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; // With upcounting, this gives positive leading pulse and directly proportional duty cycle
  led_LEDStripTIMOCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // Enable the output?

  TIM_OC1Init(TIM16, &led_LEDStripTIMOCInitStruct); // TIM16 Channel 2 output compare init
  TIM_SetCompare1(TIM16, 0); // Turn the LED Strip off
  // Enable the capture compare on TIM16
  TIM_CtrlPWMOutputs(TIM16, ENABLE);
  TIM_CCxCmd(TIM16, TIM_Channel_2, TIM_CCx_Enable);

}

/**
 * @brief Turn LED 0 on :)
 * @param None
 * @retval None
 */

void led_0On(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

/**
 * @brief Turn LED 0 off :)
 * @param None
 * @retval None
 */

void led_0Off(void) {
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/**
 * @brief Turn LED 1 on :)
 * @param None
 * @retval None
 */

void led_1On(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/**
 * @brief Turn LED 1 off :)
 * @param None
 * @retval None
 */

void led_1Off(void) {
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}

