/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | main.c
 * @brief     | Main program file
 * @authors   | Team 13
 *
 * This is the main C file, holds initializations and high level execution
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * OpenOCD Config: -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg
 *
 * ============================================================================
 */

// == Includes ==
#include <stdio.h>
#include "spiGyro_lib.h"
#include "adcTempSense_lib.h"
#include "ledGPIOB_lib.h"
#include "pbGPIOA_lib.h"
#include "pwmMotor_lib.h"
#include "diag/Trace.h" // Trace output via STDOUT
#include "serialTerminal_lib.h"

// == Defines ==
//#define SERIAL_SEND

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// == Global Variables ==

// == Declarations ==

int main(int argc, char* argv[]) {

  volatile uint32_t delay_counter = 0;
  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ; // Start up delay - Not entirely sure why we need this?
  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ;
  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ;

  // Initializations
  pb_pbGPIOAInit();
  led_init();
  ats_tempSenseInit();
  gyr_SPIInit();
  gyr_setupRegisters();
  gyr_opInit();
  mtr_motorTimInit();

#ifdef SERIAL_SEND
  srl_serialTerminalInit();
#endif


  while(TRUE) {
    mtr_stop();
    mtr_rotate(MTR_ROTATECW, 75, 75);
//    mtr_stop();
    mtr_rotate(MTR_ROTATECCW, 10, 30);
    mtr_stop();
  }

  // Control
  float motorPIDIntegral = 0; // Initial conditions
  float motorPIDDerivative = 0; // Instantaneous derivative
  int16_t prevError = 0; // Previous and current errors to calculate derivative and integral
  int16_t curError = 0;
  TIM_Cmd(TIM7, ENABLE);
  TIM_SetCounter(TIM7, 0); // Enable and zero the controlCounter

  // In the loop
  gyr_getAngle(gyro_angleData);
  curError = ANGLE_SET_POINT;
  // Check if we are too angled, and stop and rotate to correct before driving out of the track
  if (abs(curError) >= ANGLE_OUTOFBOUND_THRESHOLD) {
    if (curError > 0) {
      mtr_rotate(MTR_ROTATECW, curError, 90);
    } else if (curError < 0) {
      mtr_rotate(MTR_ROTATECCW, abs(curError), 90);
    }
    // Then reset the PID system
    motorPIDIntegral = 0;
    prevError = 0;

    // However, if we are over the threshold, implement some PID
  } else if (abs(curError) > PID_THRESHOLD){
    uint32_t elapsedTime = TIM_GetCounter(TIM7);
    TIM_SetCounter(TIM7, 0);
    motorPIDDerivative = (curError-prevError)/elapsedTime; // Calculate the derivative
    motorPIDIntegral = motorPIDIntegral + (curError-prevError); // Calculate and update the integral

    if (curError < 0) { // If we need to correct to the left, then make the correction
      uint16_t newLeftSpeed = FULL_SPEED + (curError*(PID_PROPORTIONAL_GAIN +
                                            (PID_DERIVATIVE_GAIN*motorPIDDerivative) +
                                            (PID_INTEGRAL_GAIN*motorPIDIntegral)));
      mtr_setSpeed(MTR_FORWARD, newLeftSpeed, FULL_SPEED);
    } else if (curError > 0) { // If we need to move to the right, then make the correction
      uint16_t newRightSpeed = FULL_SPEED - (curError*(PID_PROPORTIONAL_GAIN +
                                            (PID_DERIVATIVE_GAIN*motorPIDDerivative) +
                                            (PID_INTEGRAL_GAIN*motorPIDIntegral)));
      mtr_setSpeed(MTR_FORWARD, FULL_SPEED, newRightSpeed);

    }

  }


//  // Start the gyro (which includes the first run calibration)
//  gyr_gyroStart();
//
//  uint16_t gyroCountCalibrate = 0;
//  uint16_t gyroCountDisplay = 0;
//  uint16_t uartCountDataSend = 0;
//  uint16_t uartData = 0;
//  uint8_t isZero = FALSE;
//  uint8_t hasZeroButtonPressed = FALSE;
//  float lastAngle = gyro_angleData[2];
//
//  for (;;) {
//    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && (!hasZeroButtonPressed)) { // Check to see if the button is being pressed and was not pressed previously
//      if (pb_zeroButtonHandler()) {
//        // At the moment, zeroing the gyro simply resets the angle, no
//        // other calibrations are done
//        gyr_calibrate(GYROCAL_PLAIN_ZERO);
//        hasZeroButtonPressed = TRUE;
//      }
//    } else if (hasZeroButtonPressed && (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))) { // Acknowledge the button not being pressed anymore
//      hasZeroButtonPressed = FALSE;
//    }
//
//    if(gyroState == GYROSTATE_RUNNING) {
//      if(gyroCountCalibrate < CALIBRATE_INTERVAL) { // Check whether we need to calibrate again
//        delay(READ_INTERVAL); // Make sure that the reads happen under the gyro ODR frequency
//        gyr_getAngle(gyro_angleData);
//        gyroCountCalibrate++;
//        gyroCountDisplay++;
//        uartCountDataSend++;
//
//      } else {
//        gyr_calibrate(GYROCAL_INTERVAL);
//        gyroCountCalibrate = 0;
//      }
//
//#ifdef SERIAL_SEND
//      if (uartCountDataSend > DATA_SEND_INTERVAL) {
//        uint32_t temperature = ats_getTemp();
//        int16_t gyro_angleXInt = gyro_angleData[0];
//        int16_t gyro_angleYInt = gyro_angleData[1];
//        int16_t gyro_angleZInt = gyro_angleData[2];
//        printf("%d;%d;%d;%d;%d;%d;%d;\r\n",  (int16_t) gyro_angleData[0],
//                                             (int16_t) gyro_angleData[1],
//                                             (int16_t) gyro_angleData[2],
//                                             (int16_t) gyro_velocityData[0],
//                                             (int16_t) gyro_velocityData[1],
//                                             (int16_t) gyro_velocityData[2],
//                                             (uint32_t) temperature);
//        uartCountDataSend = 0;
//      }
//#endif
//
//    }
//  }

  return 0;
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
