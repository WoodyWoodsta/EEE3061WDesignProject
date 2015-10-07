/**
 * ============================================================================
 * File Name          : USARTOutTask_task.c
 * Description        : USARTOutTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Definitions ==
#define TELEMETRY_PERIOD          1000 // How long between sending telemetry packet [ms]

// == Private Function Declarations ==
static void interpretCommand(msgCommand_t rxCommand);

// == Function Definitions ==

/**
 * @brief USARTOutTask
 * @param argument
 */
void StartUSARTOutTask(void const * argument) {
  size_t freeHeap;
  msg_genericMessage_t rxMessage;
  /* Infinite loop */
  for (;;) {
    globalFlags.generalData.USARTOutTaskStackHWM = uxTaskGetStackHighWaterMark(USARTOutTaskHandle);

    // Wait for messages
    fetchMessage(msgQUSARTOut, &rxMessage, TELEMETRY_PERIOD);

    // For now, we don't want to send the telemetry - just to save time and resources
//    if (globalFlags.states.connectState == CONN_CONNECTED) {
//      uint8_t telemetryLength = 3;
//      uint8_t telemetryHeaderLength = (((telemetryLength > 9) ? (2) : (1)) + 15)*sizeof(uint8_t);
//      uint8_t telemetryStringLength = (telemetryLength * sizeof(uint8_t)) + sizeof('\0');
//
//      uint8_t *telemetryHeader = pvPortMalloc(telemetryHeaderLength);
//      sprintf(telemetryHeader, "AT+CIPSEND=0,%d\r\n", telemetryLength);
//      cHAL_USART_sTransmit_IT(&huart2, telemetryHeader, telemetryHeaderLength, 1);
//
//      uint8_t *telemetryString = pvPortMalloc(telemetryStringLength);
//      telemetryString[0] = globalFlags.states.lightSensorState;
//      telemetryString[1] = globalFlags.states.lineSensorState;
//      telemetryString[2] = globalFlags.states.motorState;
//
//      cHAL_USART_sTransmit_IT(&huart2, telemetryString, telemetryStringLength, 1);
//    }

    // Identify the type of message
    switch (rxMessage.messageType) {
    case MSG_TYPE_COMMAND:
      // If we have received a command, decode and interpret it
      interpretCommand(decodeCommand(&rxMessage));
      break;
    default:
      break;
    }
  }
}

/**
 * @brief Interpret the command received, and act on it
 * @param rxCommand: Command received in the message
 */
static void interpretCommand(msgCommand_t rxCommand) {
  switch (rxCommand) {
  case MSG_CMD_WIFI_TX_AT:
    cHAL_USART_sTransmit_IT(&huart2, txString_ATCommandTest, strlen(txString_ATCommandTest), 0);
    break;
  case MSG_CMD_WIFI_TX_ATE_0:
    cHAL_USART_sTransmit_IT(&huart2, txString_ATE0, strlen(txString_ATE0), 0);
    break;
  case MSG_CMD_WIFI_TX_CIPMUX_1:
    cHAL_USART_sTransmit_IT(&huart2, txString_multiConnect1, strlen(txString_multiConnect1), 0);
    break;
  case MSG_CMD_WIFI_TX_CWMODE_3:
    cHAL_USART_sTransmit_IT(&huart2, txString_stationMode3, strlen(txString_stationMode3), 0);
    break;
  case MSG_CMD_WIFI_TX_CONNECT_AP: {
    // Formulate the connect command
    uint8_t apSSID[] = "coreNet";
    uint8_t apKey[] = "electronics9663";
    uint8_t *txApCommand = pvPortMalloc(strlen(apSSID) + strlen(apKey)
        + strlen(txString_connectAPCommand) + 7);
    sprintf(txApCommand, "%s\"%s\",\"%s\"\r\n", txString_connectAPCommand, apSSID, apKey);

    HAL_StatusTypeDef status =
        cHAL_USART_sTransmit_IT(&huart2, txApCommand, strlen(txApCommand), 1);
    if (status != HAL_OK) {
      vPortFree(txApCommand);
    }

    break;
  }
  case MSG_CMD_WIFI_TX_START_SERVER:
    cHAL_USART_sTransmit_IT(&huart2, txString_startServer, strlen(txString_startServer), 0);

    break;
  default:
    break;
  }
}
