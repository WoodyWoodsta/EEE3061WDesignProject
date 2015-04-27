/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | spiGyro_lib.h
 * @brief     | LD3G20 3-Axis Gyro Library Header (SPI)
 * @pre       | gyr
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
#include "stm32f0xx_tim.h"
#include "lcd_stm32f0.h"
#include "adcTempSense_lib.h"

// == Defines ==
#pragma GCC diagnostic ignored "-Wpointer-sign"

#define FALSE 0
#define TRUE 1

#define BIAS_SAMPLE_WIDTH 100 // samples
#define VELOCITY_THRESHOLD 1 // +-dps
#define READ_INTERVAL 5000 // us
#define CALIBRATE_INTERVAL 2000 // Reads
#define DISPLAY_INTERVAL 10 // Reads

typedef enum { // Use to determine which state the gyro is in
  GYROSTATE_OFF,
  GYROSTATE_WAITING_FOR_ZERO,
  GYROSTATE_CALIBRATING,
  GYROSTATE_STANDBY,
  GYROSTATE_RUNNING
} gyr_gyroState_t;

typedef enum {
  GYROCAL_FULL,
  GYROCAL_INTERVAL,
  GYROCAL_PLAIN_ZERO
} gyr_calType_t;

typedef enum {
  GYROAXIS_X,
  GYROAXIS_Y,
  GYROAXIS_Z
} gyr_gyroAxis_t;

// == Global Variables ==
float gyro_velocityData[3];
float gyro_angleData[3];
gyr_gyroState_t gyroState;

float zeroBias[3];
float senseConst;
uint8_t gyroFSReg;

// == Declarations ==
void gyr_SPIInit(void);
void gyr_opInit(void);
void gyroChipSelect();
void gyroChipDeselect(void);
void EEPROMChipSelect(void);
void EEPROMChipDeselect(void);
uint8_t gyr_writeSPIgyro(uint8_t regAdr, uint8_t data);
void gyr_setupRegisters(void);
void gyr_getGyro(float* out);
void gyr_prettyTraceGyroVelocity(float *input);
void gyr_prettyLCDGyroVelocity(float *gyro);
void gyr_prettyLCDAxis(float *velocity, float *angle, gyr_gyroAxis_t axis);
void gyr_checkSPIResponse(void);
void gyr_gyroStart(void);
void gyr_calibrate(gyr_calType_t calibrationType);
void gyr_getAngle(float *out);

int16_t twosCompToDec16(uint16_t val);
void delay(uint32_t delay_in_us);

#endif /* SPIGYRO_LIB_H_ */
