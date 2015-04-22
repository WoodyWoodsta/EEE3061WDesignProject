/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | spiGyro_lib.h
 * @brief     | LD3G20 3-Axis Gyro Library Header (SPI)
 * @authors   | Team 13
 *
 * This header file for the tempsense_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef TEMPSENSE_LIB_H_
#define TEMPSENSE_LIB_H_

//==Includes==
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include <stm32f0xx_tim.h>
#include "lcd_stm32f0.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
// ==Global Variables
uint8_t val0;
uint8_t val1;
uint8_t RegularConvData_Tab[2];

//  ==Defines  ==
void init_adc_POTs(void);
void DMA_ADC_init(void);

void temp_display(void);












#endif /* TEMPSENSE_LIB_H_ */
