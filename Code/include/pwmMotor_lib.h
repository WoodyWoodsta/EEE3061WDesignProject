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
#define ANGLE_OUTOFBOUND_THRESHOLD
#define CORRECTION_GAIN
#define MOTOR_ATTACK 1 // How fast should the motors accelerate (% per 10 ms) NB! If a value too low is causing stalling - just increase
#define MOTOR_ATTACK_THRESHOLD 20 // When should the motors start attacking

// == Declarations ==

typedef enum {
  MTR_DISABLED,
  MTR_ENABLED
} mtr_motorENState_t;

typedef enum {
  MTR_STOPPED, // Both timers disabled
  MTR_FORWARD, // TIM1 enabled, TIM15 disabled
  MTR_REVERSE, // TIM15 enabled, TIM1 disabled
  MTR_ROTATECW, // Both timers enabled
  MTR_ROTATECCW // Both timers enabled
} mtr_motorOpState_t;

// == Global Variables ==
mtr_motorENState_t mtr_motorENState; // Will be disabled by default
mtr_motorOpState_t mtr_motorOpState; // Will be still by default
int16_t mtr_motorRCrtSpd; // Speed from 0-100 (duty cycle)
int16_t mtr_motorLCrtSpd; // Speed from 0-100 (duty cycle)

// == Prototypes ==

void mtr_motorTimInit(void);
void mtr_setSpeed(mtr_motorOpState_t direction, uint16_t leftSpeed, uint16_t rightSpeed);
void mtr_setLeftSpeed(mtr_motorOpState_t direction, uint16_t speed);
void mtr_setRightSpeed(mtr_motorOpState_t direction, uint16_t speed);
void mtr_rotate(mtr_motorOpState_t direction, uint16_t angle, uint16_t speed);
void mtr_stop(void);
void mtr_leftEnable(void);
void mtr_leftDisable(void);
void mtr_rightEnable(void);
void mtr_rightDisable(void);


#endif /* PWMMOTOR_LIB_H_ */
