/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "hal_lib.h"
#include "cmsis_os.h"
#include "userTasks_task.h"

/* Private variables ---------------------------------------------------------*/

// == Message Pools and Queues ==
// Global generic message and string message pool definitions
osPoolDef(genericMPool, GLOBAL_MESSAGE_MPOOL_SIZE, msg_genericMessage_t);
osPoolDef(strBufMPool, STRING_BUFFER_MPOOL_SIZE, msg_stringMessage_t);

// USART In Task String Queue
osMessageQDef(msgQUSARTIn, STRING_BUFFER_MPOOL_SIZE, msg_stringMessage_t);

// USART Out Task Message Queue
osMessageQDef(msgQUSARTOut, 2, msg_genericMessage_t);

// Boss Task Message Queue
osMessageQDef(msgQBoss, 5, msg_genericMessage_t);

// UserIO Task Message Queue
osMessageQDef(msgQUserIO, 4, msg_genericMessage_t);

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

int main(void) {
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
//  MX_ADC_Init();
//  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  // Beep da buzza
  int i = 0;
  int j = 0;
  while (j < 4) {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
    for (i = 0; i < 300000; i++);
    j++;
  }

  // Initialise global flags
  globalFlags.states.commState = COMM_STATE_AUTO;
  globalFlags.states.wifiState = GEN_STATE_READY;
  globalFlags.procedures.wifiProcedures = WIFI_PROC_NONE;

  globalFlags.states.motorState = MTR_STATE_OFF;
  globalFlags.states.lineSensorState = LNS_STATE_OFF;

  globalFlags.motorData.leftMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.leftMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.leftMotorSpeed = 0;
  globalFlags.motorData.rightMotorSpeed = 0;
  globalFlags.motorData.controlState = MTR_CTRL_BANG_BANG;

  globalFlags.lineSensorData.linePos = LINE_POS_CENTER; // Initial condition is for the robot to be centered
  globalFlags.states.lightSensorState = LIGHT_STATE_OFF;



  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* Definition and creation of ledTimer */
  osTimerDef(ledTimer, ledTimerCallback);
  ledTimerHandle = osTimerCreate(osTimer(ledTimer), osTimerOnce, NULL);

  /* Definition and creation of buzzerTiemr */
  osTimerDef(buzzerTimer, buzzerTimerCallback);
  buzzerTimerHandle = osTimerCreate(osTimer(buzzerTimer), osTimerOnce, NULL);

  /* Create the thread(s) */
  osThreadDef(bossTask, StartBossTask, osPriorityNormal, 0, 128);
  bossTaskHandle = osThreadCreate(osThread(bossTask), NULL);

  osThreadDef(USARTInTask, StartUSARTInTask, osPriorityNormal, 0, 64);
  USARTInTaskHandle = osThreadCreate(osThread(USARTInTask), NULL);

  osThreadDef(USARTOutTask, StartUSARTOutTask, osPriorityNormal, 0, 128);
  USARTOutTaskHandle = osThreadCreate(osThread(USARTOutTask), NULL);

  osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 64);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  osThreadDef(lineSensorTask, StartLineSensorTask, osPriorityNormal, 0, 64);
  lineSensorTaskHandle = osThreadCreate(osThread(lineSensorTask), NULL);

  osThreadDef(userIOTask, StartUserIOTask, osPriorityNormal, 0, 128);
  userIOTaskHandle = osThreadCreate(osThread(userIOTask), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  // Generic messaging and string message memory pools
  genericMPool = osPoolCreate(osPool(genericMPool));
  strBufMPool = osPoolCreate(osPool(strBufMPool));

  // USART In Task String Queue
  msgQUSARTIn = osMessageCreate(osMessageQ(msgQUSARTIn), NULL);

  // USART Out Task Queue
  msgQUSARTOut = osMessageCreate(osMessageQ(msgQUSARTOut), NULL);

  // Boss Task Queue
  msgQBoss = osMessageCreate(osMessageQ(msgQBoss), NULL);

  // UserIO Task Queue
  msgQUserIO = osMessageCreate(osMessageQ(msgQUserIO), NULL);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  while (1) {
    __asm("nop");
  }
}

/* USART1 init function */
//void MX_USART1_UART_Init(void) {
//
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  HAL_UART_Init(&huart1);
//
//}
//
///* USART2 init function */
//void MX_USART2_UART_Init(void) {
//
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  HAL_UART_Init(&huart2);
//
//}


#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
//void assert_failed(uint8_t* file, uint32_t line) {
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//
//}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
