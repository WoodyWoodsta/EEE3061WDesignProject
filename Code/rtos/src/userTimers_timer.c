/**
 * ============================================================================
 * File Name          : userTimers.c
 * Description        : Timer callback definitions and other timer related things
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Private Function Declarations ==

// == Function Definitions ==

/**
 * @brief lineSensorTask
 * @param argument
 */
void ledTimerCallback(void const * argument) {
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

  // Check what type of blinking is going on
  // Restart the timer with the appropriate period
  switch (globalFlags.states.ledState) {
  case LED_STATE_BLINK_SLOW:
    osTimerStart(ledTimerHandle, LED_BLINK_SLOW_PERIOD);
    break;
  case LED_STATE_BLINK_FAST:
    osTimerStart(ledTimerHandle, LED_BLINK_FAST_PERIOD);
    break;
  case LED_STATE_BLINK_SUPERFAST:
    osTimerStart(ledTimerHandle, LED_BLINK_SUPERFAST_PERIOD);
    break;
  default:
    break;
  }
}

/* buzzerTimerCallback function */
void buzzerTimerCallback(void const * argument) {
}
