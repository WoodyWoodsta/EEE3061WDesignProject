/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | ledGPIOB_lib.h
 * @brief     | LED Library (GPIOB)
 * @pre       | led
 * @authors   | Team 13
 *
 * This header file for the ledGPIOB_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef LEDGPIOB_LIB_H_
#define LEDGPIOB_LIB_H_

void led_init(void);
void led_oddOn(void);
void led_evenOn(void);

#endif /* LEDGPIOB_LIB_H_ */
