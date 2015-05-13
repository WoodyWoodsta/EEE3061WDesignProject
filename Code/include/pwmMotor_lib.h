/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | pwmMotor_lib.h
 * @brief     | Motor Control Library (PWM)
 * @pre       | mtr
 * @authors   | Team 13
 *
 * This header file for the pwmMotor_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef PWMMOTOR_LIB_H_
#define PWMMOTOR_LIB_H_

// == Includes ==
#include <stdint.h>
#include "math.h"
#include "diag/Trace.h" // Trace output via STDOUT
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"

// == Defines ==

// == Global Variables ==

// == Declarations ==

void mtr_motorTimInit(void);

#endif /* PWMMOTOR_LIB_H_ */
