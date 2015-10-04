/**
 * ============================================================================
 * File Name          : sensorTask_task.c
 * Description        : sensorTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Defines ==
#define LNS_UPDATE_PERIOD           5 // Time to wait in between line sensor updates [ms]
#define LIGHT_CAL_ITERATIONS        10 // Number of readings to take of the light sensor to find the threshold
#define LIGHT_THRESHOLD_DIV         3 // Divide the ambient light by 1/# to find the threshold

// == Private Function Declarations ==
static void interpretSignal(osEvent *signalEvent);
static void updateLinePos(void);
static uint32_t checkLightSensor(void);
static uint8_t fetchSensors(void);
static void checkStartLight(void);

// == Function Definitions ==

/**
 * @brief sensorTask
 * @param argument
 */
void StartSensorTask(void const * argument) {
  msg_genericMessage_t rxMessage;

  // Do the light sensor calibration
  sendCommand(msgQUserIO, MSG_SRC_LINE_SENSOR_TASK, MSG_CMD_LED_BLINK_SLOW, osWaitForever);
  uint32_t i = 0;
  for (i = 0; i < LIGHT_CAL_ITERATIONS; i++) {
    globalFlags.lineSensorData.lightSensorThreshold += checkLightSensor()/(LIGHT_CAL_ITERATIONS);
  }

  // TODO Handle buzzer in the IO task
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
  sendCommand(msgQUserIO, MSG_SRC_LINE_SENSOR_TASK, MSG_CMD_LED_OFF, osWaitForever);

  // Calculate the threshold with the preset division
  globalFlags.lineSensorData.lightSensorThreshold = globalFlags.lineSensorData.lightSensorThreshold/LIGHT_THRESHOLD_DIV;

  // Start the timer to record the capacitive discharge
  HAL_TIM_Base_Start(&htim6); // TODO Check to see if I need this

  // Put the motors into standby
  osSignalSet(motorTaskHandle, MTR_SIG_STANDBY);

  /* Infinite loop */
  for (;;) {
    globalFlags.generalData.lineSensorTaskStackHWM = uxTaskGetStackHighWaterMark(sensorTaskHandle);
    // TODO Switch signal receiving to a proper handler function!
    // Wait for the signal - if the light or line sensor is on, don't wait, just check
    osEvent signalEvent = osSignalWait(0, (globalFlags.states.lineSensorState == LNS_STATE_OFF
                                           && globalFlags.states.lightSensorState == LIGHT_STATE_OFF)
                                           ? (1000) : (LNS_UPDATE_PERIOD)); // TODO Double check the working of this timeout

    if (signalEvent.status == osEventSignal) {
      // Decode the signal and act on it
      interpretSignal(&signalEvent);
    }

    if (globalFlags.states.lineSensorState == LNS_STATE_ON) {
      updateLinePos();
    }

    if (globalFlags.states.lightSensorState == LIGHT_STATE_ON) {
      checkStartLight();
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
    // Disable the line sensor
    globalFlags.states.lineSensorState = LNS_STATE_OFF;
    break;
  case LIGHT_SIG_START:
    // Enable the light sensor
    globalFlags.states.lightSensorState = LIGHT_STATE_ON;
    break;
  case LIGHT_SIG_STOP:
    // Disable the light sensor
    globalFlags.states.lightSensorState = LIGHT_STATE_OFF;
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
  case 0b10: // Left: no line | Right: line
    // Robot has either moved in from the far left, or has moved out to the left
    newLinePos = LINE_POS_LEFT;
    break;
  case 0b01: // Left: line | Right: no line
    // Robot has either moved in from the far right, or has moved out to the right
    newLinePos = LINE_POS_RIGHT;
    break;
  case 0b11: // Left: line | Right: line
    if (1 || (previousLinePos != LINE_POS_RIGHTRIGHT && previousLinePos != LINE_POS_LEFTLEFT)) {
      // Robot has moved in from the left or right
      newLinePos = LINE_POS_CENTER;
    } // ELSE: Robot has moved over a perpendicular line whilst over to the far right or far left
    break;
  }

  // Update the flags
  globalFlags.lineSensorData.linePos = newLinePos;
}

/**
 * @brief Check the light sensor for green light
 * @retval uin32_t time taken for the LED to discharge
 */
static uint32_t checkLightSensor(void) {
  volatile uint32_t lightSensorDischarge = 0;

  // Start the charge-up (should have been set last time the function was called)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  osDelay(200);

  // Make sure the thread priority is highest so nothing interrupts the sensitive timing
  osThreadSetPriority(sensorTaskHandle, osPriorityAboveNormal);

  __HAL_TIM_SET_COUNTER(&htim6, 0); // Reset the timer
  GPIOB->MODER &= ~GPIO_MODER_MODER2; // Switch it to an input

  // Check for a digital 1
  while ((GPIOB->IDR) & (1 << 2)) {
    __asm("nop");
  }

  lightSensorDischarge = __HAL_TIM_GET_COUNTER(&htim6); // Get the discharge time

  // Restore priority
  osThreadSetPriority(sensorTaskHandle, osPriorityNormal);

  GPIOB->MODER |= GPIO_MODER_MODER2_0; // Switch it to an output
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  // Return the time taken for the LED to discharge
  return lightSensorDischarge;
}

/**
 * @brief Wait for the light sensor to sense the green light and initiate a motor start
 */
static void checkStartLight(void) {
  // Wait for the light sensor to detect the green light
  volatile uint32_t lightReading = 0xFFFFFFFF;

  while(!(lightReading <= globalFlags.lineSensorData.lightSensorThreshold)) {
    // If something else starts the motors, stop this process and return
    if (globalFlags.states.motorState == MTR_STATE_RUNNING) {
      globalFlags.states.lightSensorState = LIGHT_STATE_OFF;
      return;
    }
    lightReading = checkLightSensor();
    osDelay(1);
  }

  // When the green light is sensed, send a signal to start the motors
  osSignalSet(motorTaskHandle, MTR_SIG_START_TRACKING);

  sendCommand(msgQUserIO, MSG_SRC_LINE_SENSOR_TASK, MSG_CMD_LED_BLINK_SUPERFAST, osWaitForever);
  // TODO Handle the buzzer in the IO task
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
  osDelay(500);
  sendCommand(msgQUserIO, MSG_SRC_LINE_SENSOR_TASK, MSG_CMD_LED_OFF, osWaitForever);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  globalFlags.states.lightSensorState = LIGHT_STATE_OFF;
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
