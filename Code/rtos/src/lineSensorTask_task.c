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
static void updateLinePos(void);
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
    // Wait for the signal - if line sensor is on, don't wait just check
    osEvent signalEvent = osSignalWait(0, (globalFlags.states.lineSensorState == LNS_STATE_OFF) ? (osWaitForever) : (LNS_UPDATE_PERIOD));

    if (signalEvent.status == osEventSignal) {
      // Check for signal, enable line sensor if the signal is 1
      if (signalEvent.value.signals == 1) {
        globalFlags.states.lineSensorState = LNS_STATE_ON;
      // If the signal is 0, disable the line sensor
      } else if (signalEvent.value.signals == 0) {
        globalFlags.states.lineSensorState = LNS_STATE_OFF;
      }
    }

    updateLinePos();
  }
}

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

static uint8_t fetchSensors(void) {
  // Read the sensors
  // LSB is right and MSB is left
  return (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) | ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) << 1);
}
