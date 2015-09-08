/**
 * ============================================================================
 * File Name          : lineSensorTask_task.c
 * Description        : lineSensorTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Defines ==
#define LNS_UPDATE_PERIOD           5 // Time to wait in between line sensor updates [ms]

// == Private Function Declarations ==
static void interpretSignal(osEvent *signalEvent);
static void updateLinePos(void);
static void checkLightSensor(void);
static uint8_t fetchSensors(void);

// == Function Definitions ==

/**
 * @brief lineSensorTask
 * @param argument
 */
void StartLineSensorTask(void const * argument) {
  msg_genericMessage_t rxMessage;

  globalFlags.lineSensorData.linePos = LINE_POS_CENTER;


  /* Infinite loop */
  for (;;) {
    globalFlags.generalData.lineSensorTaskStackHWM = uxTaskGetStackHighWaterMark(lineSensorTaskHandle);
    // TODO Switch signal receiving to a proper handler function!
    // Wait for the signal - if the light or line sensor is on, don't wait, just check
    osEvent signalEvent = osSignalWait(0, (globalFlags.states.lineSensorState == LNS_STATE_OFF
                                           || globalFlags.states.lightSensorState == LIGHT_STATE_OFF)
                                           ? (osWaitForever) : (LNS_UPDATE_PERIOD));

    if (signalEvent.status == osEventSignal) {
      // Decode the signal and act on it
      interpretSignal(&signalEvent);
    }

    if (globalFlags.states.lineSensorState == LNS_STATE_ON) {
      updateLinePos();
    } else if (globalFlags.states.lightSensorState == LIGHT_STATE_ON) {
      checkLightSensor();
    }
  }
}

/**
 * @brief Decode the signal received and act on the command
 * @param *signalEvent: Pointer to the received signal even structure
 */
static void interpretSignal(osEvent *signalEvent) {
  int32_t signalEventValue = signalEvent->value.signals;
  switch (signalEventValue) {
  case LINE_SIG_START:
    // Enable line sensor
    globalFlags.states.lineSensorState = LNS_STATE_ON;
    break;
  case LINE_SIG_STOP:
    // This must only be fired if the motors are actually running
    if (globalFlags.states.motorState == MTR_STATE_RUNNING) {
      globalFlags.states.lineSensorState = LNS_STATE_OFF;
    }
    break;
  case LIGHT_SIG_START:
    break;
  case LIGHT_SIG_STOP:
    break;
  default:
    break;
  }
}

/**
 * @brief Run the logic to determine where the line is based on where it was and what the sensors are reading
 */
static void updateLinePos(void) {
  // Read in the sensor data
  uint8_t sensorReading = fetchSensors();

  // Fetch the previous line position from the global flags
  linePos_t previousLinePos = globalFlags.lineSensorData.linePos;
  linePos_t newLinePos = previousLinePos;

  // Do the logic!
  switch (sensorReading) {
  case 0b00: // Left: no line | Right: no line
    if (previousLinePos == LINE_POS_LEFT) {
      // Robot has moved off further towards the left
      newLinePos = LINE_POS_LEFTLEFT;
    } else if (previousLinePos == LINE_POS_RIGHT) {
      // Robot has moved off further towards the right
      newLinePos = LINE_POS_RIGHTRIGHT;
    }
    break;
  case 0b01: // Left: no line | Right: line
    // Robot has either moved in from the far left, or has moved out to the left
    newLinePos = LINE_POS_LEFT;
    break;
  case 0b10: // Left: line | Right: no line
    // Robot has either moved in from the far right, or has moved out to the right
    newLinePos = LINE_POS_RIGHT;
    break;
  case 0b11: // Left: line | Right: line
    if (previousLinePos != LINE_POS_RIGHTRIGHT && previousLinePos != LINE_POS_LEFTLEFT) {
      // Robot has moved in from the left or right
      newLinePos = LINE_POS_CENTER;
    } // ELSE: Robot has moved over a perpendicular line whilst over to the far right or far left
    break;
  }

  globalFlags.lineSensorData.linePos = newLinePos;
}

/**
 * @brief Check the light sensor for green light
 */
static void checkLightSensor(void) {
  uint32_t lightSensorDischarge = 0;
  // Start the charge-up (should have been set last time the function was called)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  osDelay(200);

  TIM_SetCounter(TIM2, 0); // Reset the timer
  GPIOA->MODER &= ~GPIO_MODER_MODER6; // Switch it to an input
  // Check for a digital 1
  while ((GPIOA->IDR) & (1 << 6)) {
    // Do nothing
  }

  lightSensorDischarge = TIM_GetCounter(TIM2); // Get the discharge time

  // TODO: Set PA8 back to an output pin and drive it high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  return lightSensorDischarge;
}

/**
 * @brief Get the reading of the line sensors
 * @retval uint8_t number, MSB being the left sensor and LSB being the right sensor
 */
static uint8_t fetchSensors(void) {
  // Read the sensors
  // LSB is right and MSB is left
  return (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) | ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) << 1);
}
