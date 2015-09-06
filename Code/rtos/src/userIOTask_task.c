/**
 * ============================================================================
 * File Name          : userIOTask_task.c
 * Description        : userIOTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Private Function Declerations ==
static void interpretCommand(msgCommand_t rxCommand);
static void stopLED(void);

// == Function Definitions ==

/**
 * @brief lineSensorTask
 * @param argument
 */
void StartUserIOTask(void const * argument) {
  msg_genericMessage_t rxMessage;

  // Initialise flags
  globalFlags.states.ledState = LED_STATE_OFF;

  /* Infinite loop */
  for (;;) {
    fetchMessage(msgQUserIO, &rxMessage, osWaitForever);

    switch (rxMessage.messageType) {
    case MSG_TYPE_NO_MESSAGE:
      break;
    case MSG_TYPE_COMMAND:
      interpretCommand(decodeCommand(&rxMessage));
      break;
    default:
      break;
    }

    osDelay(1);
  }
}

/**
 * @brief Interpret the command received, and act on it
 * @param rxCommand: Command received in the message
 */
static void interpretCommand(msgCommand_t rxCommand) {
  switch (rxCommand) {
  case MSG_CMD_LED_ON:
    // Check if the LED is doing anything else
    if (globalFlags.states.ledState != LED_STATE_OFF) {
      stopLED();
    }

    // Update flags and turn LED on
    globalFlags.states.ledState = LED_STATE_ON;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    break;
  case MSG_CMD_LED_OFF:
    stopLED();
    globalFlags.states.ledState = LED_STATE_OFF;
    break;
  case MSG_CMD_LED_BLINK_SLOW:
    // Check if the LED is doing anything else
    if (globalFlags.states.ledState != LED_STATE_OFF) {
      stopLED();
    }

    // Update flags, turn LED on and start LED timer
    globalFlags.states.ledState = LED_STATE_BLINK_SLOW;
    osTimerStart(ledTimerHandle, LED_BLINK_SLOW_PERIOD);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    break;
  case MSG_CMD_LED_BLINK_FAST:
    // Check if the LED is doing anything else
    if (globalFlags.states.ledState != LED_STATE_OFF) {
      stopLED();
    }

    // Update flags, turn LED on and start LED timer
    globalFlags.states.ledState = LED_STATE_BLINK_FAST;
    osTimerStart(ledTimerHandle, LED_BLINK_FAST_PERIOD);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    break;
  case MSG_CMD_LED_BLINK_SUPERFAST:
    // Check if the LED is doing anything else
    if (globalFlags.states.ledState != LED_STATE_OFF) {
      stopLED();
    }

    // Update flags, turn LED on and start LED timer
    globalFlags.states.ledState = LED_STATE_BLINK_SUPERFAST;
    osTimerStart(ledTimerHandle, LED_BLINK_SUPERFAST_PERIOD);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    break;
  default:
    break;
  }
}

/**
 * @brief Turn off the LED and stop timers associated with it
 */
static void stopLED(void) {
  // Stop the timer
  osTimerStop(ledTimerHandle);

  // Make sure the LED is off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
