#include "TempSense_lib.h"


void temp_display(void){
  char resultLine1[16];;
 lcd_command(LCD_CLEAR_DISPLAY);
 val0= RegularConvData_Tab[0];
 sprintf(resultLine1, "T:%d", val0);
 lcd_string(resultLine1);



}

void init_adc_POTs(void)
{
  DMA_ADC_init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//enable the clocks
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;//enable the two pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//set to analogue mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
  RCC_HSI14Cmd(ENABLE);

  ADC_DeInit(ADC1);

  ADC_InitTypeDef ADC1struct;
  ADC_StructInit(&ADC1struct);


  ADC_StructInit(&ADC1struct);
  ADC1struct.ADC_Resolution=ADC_Resolution_8b;//set ADC to 8-bit mode
  ADC1struct.ADC_ContinuousConvMode=ENABLE;//put in continous conversion mode
  ADC1struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
  ADC1struct.ADC_DataAlign=ADC_DataAlign_Right;
  ADC1struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
  ADC_Init(ADC1,&ADC1struct);

  ADC_ChannelConfig(ADC1,ADC_Channel_6,ADC_SampleTime_55_5Cycles);//set up the ADC channels
  ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);

  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);//link it to the DMA

  ADC_Cmd(ADC1,ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));
  ADC_StartOfConversion(ADC1);
}


void DMA_ADC_init(void)
{
  DMA_InitTypeDef     DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t) (&(ADC1->DR));//the address of the ADC
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;//where the data is stored
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;//linked to two ADC channels
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//circular because the ADC is in continous mode
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, ENABLE);
}
