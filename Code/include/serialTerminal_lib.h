/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | serialTerminal_lib.h
 * @brief     | Serial Communications Library Header (UART)
 * @pre       | srl
 * @authors   | James Gowans (University of Cape Town)
 *
 * This header file for the serialTerminal_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef SERIALTERMINAL_LIB_H_
#define SERIALTERMINAL_LIB_H_

// == Includes ==
#include "stdint.h"
#include "stm32f0xx.h"

// == Defines ==
#define STM32F051

// == Declarations ==
// initialises the USART and enable the incomming char interupt
void srl_serialTerminalInit(void);

#endif /* SERIALTERMINAL_LIB_H_ */
