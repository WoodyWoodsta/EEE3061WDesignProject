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

// == Defines ==

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
  lcd_init();
  lcd_command(LCD_CLEAR_DISPLAY);
  lcd_two_line_write("L3GD20 YAW-DUDE", "      V2.0"); // Hehehe
  pb_pbGPIOAInit();
  led_init();
  ats_tempSenseInit();
  gyr_SPIInit();
  gyr_setupRegisters();
  gyr_opInit();

  // Start the gyro (which includes the first run calibration)
  gyr_gyroStart();

  uint16_t gyroCountCalibrate = 0;
  uint16_t gyroCountDisplay = 0;
  uint8_t isZero = FALSE;
  float lastAngle = gyro_angleData[2];
  gyr_prettyLCDAxis(gyro_velocityData, gyro_angleData, GYROAXIS_Z);

  for (;;) {
    if(gyroState == GYROSTATE_RUNNING) {
      if(gyroCountCalibrate < CALIBRATE_INTERVAL) { // Check whether we need to calibrate again
        delay(READ_INTERVAL); // Make sure that the reads happen under the gyro ODR frequency
        gyr_getAngle(gyro_angleData);
        gyroCountCalibrate++;
        gyroCountDisplay++;

      } else {
        gyr_calibrate(GYROCAL_INTERVAL);
        gyroCountCalibrate = 0;
      }

      if ((gyroCountDisplay > DISPLAY_INTERVAL) && (gyro_angleData[2] != lastAngle)) { // Check to see if the LCD needs to be updated
        lastAngle = gyro_angleData[2];
        gyr_prettyLCDAxis(gyro_velocityData, gyro_angleData, GYROAXIS_Z);
        gyroCountDisplay = 0;
        isZero = FALSE;
      } else if ((gyroCountDisplay > DISPLAY_INTERVAL) && (!isZero)) {
        lcd_command(LCD_CURSOR_HOME);
        lcd_command(LCD_GOTO_LINE_2);
        lcd_string("V = 0.0     ");
        isZero = TRUE;
      }
    }
  }

  return 0;
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
