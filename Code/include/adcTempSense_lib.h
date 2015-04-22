/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | adcTempSense_lib.h
 * @brief     | LM35 Analog Thermal Sensor Library (ADC)
 * @pre       | ats
 * @authors   | Team 13
 *
 * This header file for the adcTempSense_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef ADCTEMPSENSE_LIB_H_
#define ADCTEMPSENSE_LIB_H_

// == Includes ==
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include <stm32f0xx_tim.h>
#include "lcd_stm32f0.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"

// == Global Variables ==
uint8_t val0;
uint8_t val1;
uint8_t RegularConvData_Tab[2];

// == Defines ==

// == Declarations ==
void ats_tempSenseInit(void);
void ats_DMAInit(void);
void ats_tempDisplay(void);

#endif /* ADCTEMPSENSE_LIB_H_ */
