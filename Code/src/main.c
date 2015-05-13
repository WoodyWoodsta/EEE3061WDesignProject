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

#ifdef SERIAL_SEND
  srl_serialTerminalInit();
#endif

  // Start the gyro (which includes the first run calibration)
  gyr_gyroStart();

  uint16_t gyroCountCalibrate = 0;
  uint16_t gyroCountDisplay = 0;
  uint16_t uartCountDataSend = 0;
  uint16_t uartData = 0;
  uint8_t isZero = FALSE;
  uint8_t hasZeroButtonPressed = FALSE;
  float lastAngle = gyro_angleData[2];

  for (;;) {
    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && (!hasZeroButtonPressed)) { // Check to see if the button is being pressed and was not pressed previously
      if (pb_zeroButtonHandler()) {
        // At the moment, zeroing the gyro simply resets the angle, no
        // other calibrations are done
        gyr_calibrate(GYROCAL_PLAIN_ZERO);
        hasZeroButtonPressed = TRUE;
      }
    } else if (hasZeroButtonPressed && (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))) { // Acknowledge the button not being pressed anymore
      hasZeroButtonPressed = FALSE;
    }

    if(gyroState == GYROSTATE_RUNNING) {
      if(gyroCountCalibrate < CALIBRATE_INTERVAL) { // Check whether we need to calibrate again
        delay(READ_INTERVAL); // Make sure that the reads happen under the gyro ODR frequency
        gyr_getAngle(gyro_angleData);
        gyroCountCalibrate++;
        gyroCountDisplay++;
        uartCountDataSend++;

      } else {
        gyr_calibrate(GYROCAL_INTERVAL);
        gyroCountCalibrate = 0;
      }

#ifdef SERIAL_SEND
      if (uartCountDataSend > DATA_SEND_INTERVAL) {
        uint32_t temperature = ats_getTemp();
        int16_t gyro_angleXInt = gyro_angleData[0];
        int16_t gyro_angleYInt = gyro_angleData[1];
        int16_t gyro_angleZInt = gyro_angleData[2];
        printf("%d;%d;%d;%d;%d;%d;%d;\r\n",  (int16_t) gyro_angleData[0],
                                             (int16_t) gyro_angleData[1],
                                             (int16_t) gyro_angleData[2],
                                             (int16_t) gyro_velocityData[0],
                                             (int16_t) gyro_velocityData[1],
                                             (int16_t) gyro_velocityData[2],
                                             (uint32_t) temperature);
        uartCountDataSend = 0;
      }
#endif

    }
  }

  return 0;
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
