/*
 * spiGyro_lib.h
 *
 *  Created on: 11 Apr 2015
 *      Author: Sean
 */

#ifndef SPIGYRO_LIB_H_
#define SPIGYRO_LIB_H_

#include <stdint.h>
#include "diag/Trace.h" // Trace output via STDOUT
#include "stm32f0xx.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "lcd_stm32f0.h"
#include <stdint.h>
#include "math.h"

#pragma GCC diagnostic ignored "-Wpointer-sign"

#define FALSE 0
#define TRUE 1

void init_leds(void);
void init_spi(void);
void chipSelect(void);
void chipDeSelect(void);
uint8_t writeSPIgyro(uint8_t regAdr, uint8_t data);
void setup_gyro_registers(void);
void getGyro(float* out);
int16_t twosCompToDec16(uint16_t val);
static void delay(uint32_t delay_in_us);
void half_on(void);
void other_half_on(void);

#endif /* SPIGYRO_LIB_H_ */
