/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | ledLightSensor_lib.h
 * @brief     | Bidirectional LED Sensing Library (GPIOA)
 * @pre       | lls
 * @authors   | Team 13
 *
 * This library is for the operation of bidirectional LED light sensing.
 *
 * ============================================================================
 */

// == Includes ==
#include "ledLightSensor_lib.h"

// == Defines ==

// == Declarations ==

/**
 * @brief Initialise PA8 and TIM2 for the LED Light Sensor
 * @param None
 * @retval None
 */

void lls_lightSensorInit(void) {
  // Setup PA6 for light sensing
  GPIO_InitTypeDef GPIO_PA6SenseInitStruct;
  GPIO_StructInit(&GPIO_PA6SenseInitStruct);

  GPIO_PA6SenseInitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_PA6SenseInitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_PA6SenseInitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_PA6SenseInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pullup for charging/discharging the LED
  GPIO_PA6SenseInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOA, &GPIO_PA6SenseInitStruct);

  // Setup TIM6 for the light sensor
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Enable the RCC clock for TIM6

  TIM_TimeBaseInitTypeDef lightSensorInitStruct;
  TIM_TimeBaseStructInit(&lightSensorInitStruct); // Used for the light sensor

  lightSensorInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  lightSensorInitStruct.TIM_Prescaler = 1;
  lightSensorInitStruct.TIM_Period = 0xFFFFFFFF;

  TIM_TimeBaseInit(TIM2, &lightSensorInitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Calculate the light intensity
 * @param None
 * @retval 32-bit time of discharge (1/48e6 seconds)
 */

uint32_t lls_getLight(void) {
  uint32_t lightSensorDischarge = 0;
  // Start the charge-up (should have been set last time the function was called)
  GPIO_SetBits(GPIOA, GPIO_Pin_6);
  // TODO: May need to add a delay here

  TIM_SetCounter(TIM2, 0); // Reset the timer
  GPIOA->MODER &= ~GPIO_MODER_MODER6; // Switch it to an input
  // Check for a digital 1
  while ((GPIOA->IDR) & (1 << 6)) {
    // Do nothing
  }

  lightSensorDischarge = TIM_GetCounter(TIM2); // Get the discharge time

  // TODO: Set PA8 back to an output pin and drive it high
  GPIO_SetBits(GPIOA, GPIO_Pin_6);

  return lightSensorDischarge;
}
