/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | adcTempSense_lib.c
 * @brief     | LM35 Analog Thermal Sensor Library (ADC)
 * @pre       | ats
 * @authors   | Team 13
 *
 * This library is used for the LM35 Analog Thermal Sensor to measure the
 * temperature for the gyro calibration (error correction)
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "adcTempSense_lib.h"

/**
 * @brief Get data from the ADC and convert to voltage
 * @param None
 * @retval 16-bit unsigned voltage in mV
 */

uint16_t ats_getVoltage(void) {
  uint16_t ADCVoltage;
  uint16_t data = ADCData[0];

  data = (data * 10000 * 3.3) / 4096;
  ADCVoltage = data / 10;

  return ADCVoltage;
}

/**
 * @brief Get data from the ADC via ats_getVoltage() and convert to temperature
 * @param None
 * @retval 16-bit unsigned temperature in degrees C TODO: Should consider doing this in m deg C
 */

uint16_t ats_getTemp(void) {
  uint16_t arrTemp[20];
  uint16_t temp;
  uint8_t icount;
  uint16_t voltsSum;
  voltsSum = 0;

  for (icount = 0; icount < 20; ++icount) {  //averages out the sensor readings
    arrTemp[icount] = ats_getVoltage()/10;
    voltsSum = voltsSum + arrTemp[icount];
  }

  temp = voltsSum/20;

  return temp;
}

/**
 * @brief Get temp and voltage data and convert and print to the LCD
 * @param None
 * @retval None
 */

void ats_tempDisplay(void) {
  char resultLine1[16];
  char resultLine2[16];

  uint16_t voltage = ats_getVoltage(); // Get the data
  uint16_t temp = ats_getTemp();

  sprintf(resultLine1, "mV:%d", voltage); // Format the strings for output
  sprintf(resultLine2, "T:%d", temp);

  lcd_command(LCD_CLEAR_DISPLAY); // Write to the LCD
  lcd_string(resultLine1);
  lcd_command(LCD_GOTO_LINE_2);
  lcd_string(resultLine2);

#ifdef CAPTURE
  trace_puts(resultLine1);
  trace_puts(resultLine2);
#endif
}

/**
 * @brief Initialise the temp sensor (ADC)
 * @param None
 * @retval None
 */

void ats_tempSenseInit(void) {
  ats_DMAInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable the clocks
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Enable the two pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Set to analogue mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
  RCC_HSI14Cmd(ENABLE);

  ADC_DeInit(ADC1);

  ADC_InitTypeDef ADC1struct;
  ADC_StructInit(&ADC1struct);
  ADC_StructInit(&ADC1struct);
  ADC1struct.ADC_Resolution = ADC_Resolution_12b; // Set ADC to 12-bit mode
  ADC1struct.ADC_ContinuousConvMode = ENABLE; // Put in continuous conversion mode
  ADC1struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC1struct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1struct.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC1struct);

  ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_55_5Cycles); // Set up the ADC channels
  ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_55_5Cycles);

  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); // Link it to the DMA

  ADC_GetCalibrationFactor(ADC1); // Calibrate the ADC

  ADC_Cmd(ADC1, ENABLE); // Enable the ADC
  ADC_DMACmd(ADC1, ENABLE); // Enable the DMA
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN))
    ;
  ADC_StartOfConversion(ADC1);
}

/**
 * @brief Initialise the DMA for use with the temp sensor (ADC)
 * @param None
 * @retval None
 */

void ats_DMAInit(void) {
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(ADC1->DR)); // The address of the ADC
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADCData; // Where the data is stored
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2; // Linked to two ADC channels
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //changed
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // Circular because the ADC is in continous mode
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, ENABLE);
}
