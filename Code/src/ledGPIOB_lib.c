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
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
      | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief Turn half of the LEDs on
 * @param None
 * @retval None
 */

void led_oddOn() {
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOB, GPIO_Pin_2);
  GPIO_ResetBits(GPIOB, GPIO_Pin_3);
  GPIO_SetBits(GPIOB, GPIO_Pin_4);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  GPIO_SetBits(GPIOB, GPIO_Pin_6);
  GPIO_ResetBits(GPIOB, GPIO_Pin_7);
}

/**
 * @brief Turn the other half of the LEDs on
 * @param None
 * @retval None
 */

void led_evenOn() {
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
  GPIO_ResetBits(GPIOB, GPIO_Pin_2);
  GPIO_SetBits(GPIOB, GPIO_Pin_3);
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
  GPIO_SetBits(GPIOB, GPIO_Pin_7);
}

