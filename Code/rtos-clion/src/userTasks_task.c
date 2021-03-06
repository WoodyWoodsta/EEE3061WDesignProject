/**
  * ============================================================================
  * File Name          : userTasks_task.c
  * Description        : Common source between all tasks, specific to tasks
  *                      Also includes source common to memory pools and
  *                      messaging queues
  * Author             : Sean Wood
  * ============================================================================
  */

// == Includes ==
#include "userTasks_task.h"

// == Exported Variables ==
osThreadId bossTaskHandle;
osThreadId USARTInTaskHandle;
osThreadId USARTOutTaskHandle;
osThreadId motorTaskHandle;
osThreadId sensorTaskHandle;
osThreadId userIOTaskHandle;

osTimerId ledTimerHandle;
osTimerId buzzerTimerHandle;

globalFlags_t globalFlags;

// USART In Task String Queue
osMessageQId msgQUSARTIn;

// USART Out Task String Queue
osMessageQId msgQUSARTOut;

// Boss Task Command Queue
osMessageQId msgQBoss;

// UserIO Task Command Queue
osMessageQId msgQUserIO;
