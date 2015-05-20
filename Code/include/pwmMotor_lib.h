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
 * ============================================================================
 */

#ifndef PWMMOTOR_LIB_H_
#define PWMMOTOR_LIB_H_

// == Includes ==
#include <stdint.h>
#include "math.h"
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "spiGyro_lib.h"

// == Defines ==
#define FULL_SPEED 98 // Percentage duty cycle which will be full speed
#define ANGLE_OUTOFBOUND_THRESHOLD 360 // Degrees
#define MOTOR_ATTACK 4 // How fast should the motors accelerate (% per 10 ms) NB! If a value too low is causing stalling - just increase
#define MOTOR_ATTACK_THRESHOLD 100 // When should the motors start attacking (100 will make it never attack)
#define MOTOR_ROTATE_SPEED 90

#define PID_PROPORTIONAL_GAIN 3
#define PID_DERIVATIVE_GAIN 0
#define PID_INTEGRAL_GAIN 0
#define PID_THRESHOLD 2

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

typedef enum {
  MTR_LIB_DISABLED,
  MTR_LIB_STANDBY,
  MTR_LIB_ENABLED
} mtr_libState_t;

// == Global Variables ==
mtr_motorENState_t mtr_motorENState; // Will be disabled by default
mtr_motorOpState_t mtr_motorOpState; // Will be still by default
int16_t mtr_motorRCrtSpd; // Speed from 0-100 (duty cycle)
int16_t mtr_motorLCrtSpd; // Speed from 0-100 (duty cycle)
mtr_libState_t mtr_motorLibraryState;

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
