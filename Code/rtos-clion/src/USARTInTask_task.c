/**
 * ============================================================================
 * File Name          : USARTInTask_task.c
 * Description        : USARTInTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Private Function Declarations ==
static void interpretUSBString(msg_stringMessage_t *pStringMessageIn);
static void interpretWifiString(msg_stringMessage_t *pStringMessageIn);
static void fetchString(msg_stringMessage_t *pMessage);

// == Function Definitions ==
/**
 * @brief USARTInTask
 * @param argument
 */
void StartUSARTInTask(void const * argument) {
  globalFlags.generalData.USARTInTaskStackHWM = uxTaskGetStackHighWaterMark(USARTInTaskHandle);

  msg_stringMessage_t rxMessage;

  /* Infinite loop */
  for (;;) {
    fetchString(&rxMessage);

    switch (rxMessage.messageSource) {
    case MSG_SRC_USB:
      interpretUSBString(&rxMessage);
      break;
    case MSG_SRC_WIFI:
      interpretWifiString(&rxMessage);
      break;
    }
  }
}

/**
 * @brief Interpret a string received from the USB USART peripheral
 * @param *pStringMessageIn: Pointer to a generic message struct to interpret
 */
static void interpretUSBString(msg_stringMessage_t *pStringMessageIn) {
  // If we are in manual mode, direct string to the Wifi module
  if (globalFlags.states.commState == COMM_STATE_MANUAL) {
    if (strncmp(rxString_commStateAuto, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      globalFlags.states.commState = COMM_STATE_AUTO;
      cHAL_USART_sTransmit_IT(&huart1, txString_commStateAuto, strlen(txString_commStateAuto), 0);
    } else {
      HAL_StatusTypeDef status =
          cHAL_USART_sTransmit_IT(&huart2, pStringMessageIn->pString, pStringMessageIn->stringLength, 1);

      // If we run into issues, get rid of the string
      if (status != osOK) {
        vPortFree(pStringMessageIn->pString);
      }
    }
  } else if (globalFlags.states.commState == COMM_STATE_AUTO) {
    if (strncmp(rxString_commStateManual, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      globalFlags.states.commState = COMM_STATE_MANUAL;
      cHAL_USART_sTransmit_IT(&huart1, txString_commStateManual, strlen(txString_commStateManual), 0);
    } else if (strncmp(rxString_ATCommandTest, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_TEST_AT, osWaitForever);
    } else if (strncmp(rxString_wifiInit, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_INIT, osWaitForever);
    } else if (strncmp(rxString_wifiConnectAp, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_CONNECT_AP, osWaitForever);
    } else if (strncmp(rxString_wifiStartServer, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_START_SERVER, osWaitForever);
    }

    vPortFree(pStringMessageIn->pString);
  }
}

/**
 * @brief Interpret a string received from the Wifi module
 * @param *pStringMessageIn: Pointer to a generic message struct to interpret
 */
static void interpretWifiString(msg_stringMessage_t *pStringMessageIn) {
  // If we are in manual mode, direct string to the USB module
  if (globalFlags.states.commState == COMM_STATE_MANUAL) {
    HAL_StatusTypeDef status =
        cHAL_USART_sTransmit_IT(&huart1, pStringMessageIn->pString, pStringMessageIn->stringLength, 1);

    // If we run into issues, get rid of the string
    if (status != osOK) {
      vPortFree(pStringMessageIn->pString);
    }
  } else if (globalFlags.states.commState == COMM_STATE_AUTO) {
    if (strncmp(rxString_receiveDataCommand, pStringMessageIn->pString, strlen(rxString_receiveDataCommand))
        == 0) {
      uint16_t pos = 0;
      while ((pStringMessageIn->pString[pos++] != ':') && (pStringMessageIn->stringLength != pos));

      // Check for incoming strings
      if (strncmp(rxString_WFR, pStringMessageIn->pString+pos, strlen(rxString_WFR)) == 0) {
        // Start the robot!
        osSignalSet(motorTaskHandle, MTR_SIG_START_TRACKING);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

      } else if (strncmp(rxString_WFS, pStringMessageIn->pString+pos, strlen(rxString_WFS)) == 0) {
        // Stop the robot
        osSignalSet(motorTaskHandle, MTR_SIG_STOP_TRACKING);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

      } else if (strncmp(rxString_WFL, pStringMessageIn->pString+pos, strlen(rxString_WFL)) == 0) {
        // Start the launcher
        osSignalSet(motorTaskHandle, MTR_SIG_START_LAUNCHER);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
      }
    } else if (strncmp(rxString_OK, pStringMessageIn->pString, pStringMessageIn->stringLength) == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_OK, osWaitForever);

    } else if (strncmp(rxString_noChange, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_NO_CHANGE, osWaitForever);

    } else if (strncmp(rxString_error, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_ERROR, osWaitForever);

    } else if (strncmp(rxString_noSuchFunction, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_NO_FUNCTION, osWaitForever);

    } else if (strncmp(rxString_fail, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      sendCommand(msgQBoss, MSG_SRC_USART_IN_TASK, MSG_CMD_WIFI_RX_FAIL, osWaitForever);
    } else if (strncmp(rxString_link, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      globalFlags.states.connectState = CONN_CONNECTED;
    } else if (strncmp(rxString_unlink, pStringMessageIn->pString, pStringMessageIn->stringLength)
        == 0) {
      globalFlags.states.connectState = CONN_DISCONNECTED;
    }

    vPortFree(pStringMessageIn->pString);
  }
}

/**
 * @brief Fetch a string message from the USARTIn Queue
 * @param *messagePtr: Pointer to a generic message struct copy into
 */
static void fetchString(msg_stringMessage_t *pMessage) {
  osEvent messageEvent;
  msg_stringMessage_t *pRxMessage;

  // Wait for a message in the message Q, msgQUSARTIn
  messageEvent = osMessageGet(msgQUSARTIn, osWaitForever);

  // If the recieved data is a message
  if (messageEvent.status == osEventMessage) {
    // Grab the pointer to the string message and copy out to an external struct
    pRxMessage = messageEvent.value.p;
    memcpy(pMessage, pRxMessage, sizeof(msg_stringMessage_t));

    // Then free the message from the string buffer memory pool
    osPoolFree(strBufMPool, pRxMessage);
  }
}

