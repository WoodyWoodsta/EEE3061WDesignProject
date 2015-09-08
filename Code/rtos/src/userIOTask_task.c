/**
 * ============================================================================
 * File Name          : userIOTask_task.c
 * Description        : userIOTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Defintions ==
#define DEBOUNCE_PERIOD       10 // Amount of time to delay when debouncing switches [ms]

// == Private Function Declerations ==
static void interpretCommand(msgCommand_t rxCommand);
static void stopLED(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

static void SW0UpAction(void);
static void SW0DownAction(void);
static void SW1UpAction(void);
static void SW1DownAction(void);

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
    globalFlags.generalData.userIOTaskStackHWM = uxTaskGetStackHighWaterMark(userIOTaskHandle);

    fetchMessage(msgQUserIO, &rxMessage, osWaitForever);

    volatile size_t freeHeap = xPortGetFreeHeapSize();
    volatile size_t minFreeHeap = xPortGetMinimumEverFreeHeapSize();

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
  case MSG_CMD_SW0_DOWN:
    // Implement some debouncing
    osDelay(DEBOUNCE_PERIOD);
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
      // Fire the callback
      SW0DownAction();
    }
    break;
  case MSG_CMD_SW0_UP:
    // Implement some debouncing
    osDelay(DEBOUNCE_PERIOD);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
      // Fire the callback
      SW0UpAction();
    }
    break;
  case MSG_CMD_SW1_DOWN:
    // Implement some debouncing
    osDelay(DEBOUNCE_PERIOD);
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
      // Fire the callback
      SW1DownAction();
    }
    break;
  case MSG_CMD_SW1_UP:
    // Implement some debouncing
    osDelay(DEBOUNCE_PERIOD);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
      // Fire the callback
      SW1UpAction();
    }
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

/**
 * @brief Callback for a switch 0 UP event
 */
static void SW0UpAction(void) {
  sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_OFF, 0);
}

/**
 * @brief Callback for a switch 0 DOWN event
 */
static void SW0DownAction(void) {
  sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_ON, 0);
//  osSignalSet(motorTaskHandle, MTR_SIG_START_TRACKING);
}

/**
 * @brief Callback for a switch 1 UP event
 */
static void SW1UpAction(void) {
  sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_OFF, 0);
}

/**
 * @brief Callback for a switch 1 DOWN event
 */
static void SW1DownAction(void) {
  osSignalSet(motorTaskHandle, MTR_SIG_STOP_TRACKING);
}

/**
 * @brief User defined EXTI interrupt callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case GPIO_PIN_0:
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_SW0_UP, 0);
    } else {
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_SW0_DOWN, 0);
    }
    break;
  case GPIO_PIN_1:
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_SW1_UP, 0);
    } else {
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_SW1_DOWN, 0);
    }
    break;
  }

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
