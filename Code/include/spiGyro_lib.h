/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | spiGyro_lib.h
 * @brief     | LD3G20 3-Axis Gyro Library Header (SPI)
 * @authors   | Team 13
 *
 * This header file for the spiGyro_lib.c library
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

#ifndef SPIGYRO_LIB_H_
#define SPIGYRO_LIB_H_

// == Includes ==
#include <stdint.h>
#include "math.h"
#include "diag/Trace.h" // Trace output via STDOUT
#include "stm32f0xx.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "lcd_stm32f0.h"

// == Defines ==
#pragma GCC diagnostic ignored "-Wpointer-sign"

#define FALSE 0
#define TRUE 1

// == Global Variables ==
float gyro[3];

// == Declarations ==
void gyr_SPIInit(void);
void gyroChipSelect();
void gyroChipDeselect();
void EEPROMChipSelect();
void EEPROMChipDeselect();
uint8_t gyr_writeSPIgyro(uint8_t regAdr, uint8_t data);
void gyr_setupRegisters(void);
void gyr_getGyro(float* out);
void gyr_prettyTraceGyro(float *input);
void gyr_prettyLCDGyro(float *gyro);
void gyr_checkSPIResponse();

int16_t twosCompToDec16(uint16_t val);
static void delay(uint32_t delay_in_us);

void init_leds(void);
void half_on(void);
void other_half_on(void);


#endif /* SPIGYRO_LIB_H_ */
