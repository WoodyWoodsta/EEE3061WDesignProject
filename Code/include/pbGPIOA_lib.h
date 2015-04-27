/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | pbGPIOA_lib.h
 * @brief     | Pushbutton Library Header (GPIOA)
 * @pre       | pb
 * @authors   | Team 13
 *
 * This header file for the pbGPIOA_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef PBGPIOA_LIB_H_
#define PBGPIOA_LIB_H_

// == Includes ==
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

// == Defines ==
#define FALSE 0
#define TRUE 1

// == Global Variables ==

// == Declarations ==
void pb_pbGPIOAInit(void);

#endif /* PBGPIOA_LIB_H_ */
