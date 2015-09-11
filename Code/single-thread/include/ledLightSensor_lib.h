/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | ledLightSensor_lib.h
 * @brief     | Bidirectional LED Sensing Library Header (GPIOA)
 * @pre       | lls
 * @authors   | Team 13
 *
 * This header file for the ledLightSensor_lib.c library
 *
 * ============================================================================
 */

#ifndef LEDLIGHTSENSOR_LIB_H_
#define LEDLIGHTSENSOR_LIB_H_

// == Includes ==
#include <stdint.h>
#include "math.h"
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"

// == Defines ==
#define GREEN_LIGHT_THRESHOLD 1000000 // Some really small time constant

// == Declarations ==
void lls_lightSensorInit(void);
uint32_t lls_getLight(void);

#endif /* LEDLIGHTSENSOR_LIB_H_ */
