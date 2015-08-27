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
uint16_t val1; // Not used??
uint16_t ADCData[2];

// == Defines ==

// == Declarations ==
void ats_tempSenseInit(void);
void ats_DMAInit(void);
float ats_getVoltage(void);
uint32_t ats_getTemp(void);
void ats_displayTemp(void);

#endif /* ADCTEMPSENSE_LIB_H_ */
