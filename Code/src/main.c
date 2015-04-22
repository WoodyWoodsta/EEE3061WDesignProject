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

  lcd_init();
  lcd_command(LCD_CLEAR_DISPLAY);
  lcd_string("Hello World");
  led_init();

  ats_tempSenseInit();

  gyr_SPIInit();
  gyr_setupRegisters();
//  ats_tempDisplay();

  for (;;) {
    ats_tempDisplayNewMethod();
    trace_printf("%d\n",RegularConvData_Tab[0]);

    for (delay_counter = 0; delay_counter < 655350; delay_counter++)
      ;
//    trace_puts("Checking gyro...");
//    gyr_getGyro(gyro);
//    gyr_prettyTraceGyro(gyro);
//    gyr_prettyLCDGyro(gyro);
//    led_oddOn();
    for (delay_counter = 0; delay_counter < 655350; delay_counter++)
      ;
//    trace_puts("Checking gyro...");
//    gyr_getGyro(gyro);
//    gyr_prettyTraceGyro(gyro);
//    gyr_prettyLCDGyro(gyro);
//    led_evenOn();
  }

  return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
