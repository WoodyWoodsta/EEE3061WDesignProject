/**
 * ============================================================================
 * File Name          : lineSensorTask_task.c
 * Description        : lineSensorTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Private Function Declerations ==
static void interpretCommand(msgCommand_t rxCommand);

// == Function Definitions ==

/**
 * @brief lineSensorTask
 * @param argument
 */
void StartLineSensorTask(void const * argument) {
  msg_genericMessage_t rxMessage;
  /* Infinite loop */
  for (;;) {
    // Wait for the signal - if line sensor is on, don't wait just check
    osEvent signalEvent = osSignalWait(0, (globalFlags.states.lineSensorState == LNS_STATE_OFF) ? (osWaitForever) : (0));

    if (signalEvent.status == osEventSignal) {
      // Check for signal, enable line sensor if the signal is 1
      if (signalEvent.value.signals == 1) {
        globalFlags.states.lineSensorState = LNS_STATE_ON;
      // If the signal is 0, disable the line sensor
      } else if (signalEvent.value.signals == 0) {
        globalFlags.states.lineSensorState = LNS_STATE_ON;
      }
    }
  }
}

/**
 * @brief Interpret the command received, and act on it
 * @param rxCommand: Command received in the message
 */
static void interpretCommand(msgCommand_t rxCommand) {
  switch (rxCommand) {
  default:
    break;
  }
}
