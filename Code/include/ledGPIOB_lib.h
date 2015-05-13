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
void led_0On(void);
void led_0Off(void);
void led_1On(void);
void led_1Off(void);

#endif /* LEDGPIOB_LIB_H_ */
