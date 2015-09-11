/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | pbGPIOA_lib.c
 * @brief     | Pushbutton Library (GPIOA)
 * @pre       | pb
 * @authors   | Team 13
 *
 * This library is for using the pushbuttons on PA0-3.
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "pbGPIOA_lib.h"

// == Defines ==

// == Declarations ==

/**
 * @brief Initialise all the pushbuttons (PA0 - PA3)
 * @param None
 * @retval None
 */

void pb_pbGPIOAInit(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitTypeDef GPIOInitStructure;

  GPIOInitStructure.GPIO_Mode = GPIO_Mode_IN; // Digital input
  GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Pullup for the buttons
  GPIOInitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Fastest speed, cos faster is always better
  GPIOInitStructure.GPIO_Pin =   (GPIO_PinSource0 |
                                  GPIO_PinSource1); // All the pushbuttons

  GPIO_Init(GPIOA, &GPIOInitStructure);
}

/**
 * @brief Handle pressing of SW0, specifically to zero the gyro
 * @param None
 * @retval result: result of the button press (0 = no press, 1 = one second press)
 */

uint8_t pb_zeroButtonHandler(void) {
  uint8_t result = 0;

  // Check for an actual button press
  if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
    result = 1;
    delay(1000000); // Wait for 1 second
  }

  // Check to see if the button is still pressed after 1 second
  if ((result == 1) && (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))) {
    result = 0;
  }

  return result;
}


