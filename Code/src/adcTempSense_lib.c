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

void ats_tempDisplay (void) {
  char resultLine1[16];
  char resultLine2[16];
  lcd_command(LCD_CLEAR_DISPLAY);
  val0 = RegularConvData_Tab[0];
//  val1 = 2.361341+0.083664*val0;

  val1=3.2377937194828+0.08493537368387*val0;

  sprintf(resultLine1, "T:%d", val0);
  sprintf(resultLine2, "T:%d", val1);
  lcd_string(resultLine1);
  lcd_command(LCD_GOTO_LINE_2);
  lcd_string(resultLine2);
}

uint16_t ats_getVoltage (void){

 val0=RegularConvData_Tab[0];
 val0=(val0*10000*3.3)/4096;
 ADC_Voltage = val0/10;

return ADC_Voltage;

}

void ats_tempDisplayNewMethod (void) {
  char resultLine1[16];
  char resultLine2[16];
  uint16_t temp;
  lcd_command(LCD_CLEAR_DISPLAY);
  temp = ats_getVoltage();
  sprintf(resultLine1, "mV:%d", temp);
  temp = temp/10;
  sprintf(resultLine2, "T:%d", temp);
  lcd_string(resultLine1);
  lcd_command(LCD_GOTO_LINE_2);
  lcd_string(resultLine2);
}

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

  ADC_GetCalibrationFactor(ADC1);

  ADC_Cmd(ADC1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN))
    ;
  ADC_StartOfConversion(ADC1);
}

void ats_DMAInit(void) {
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(ADC1->DR)); // The address of the ADC
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) RegularConvData_Tab; // Where the data is stored
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
