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
 * ============================================================================
 */

// == Includes ==
#include <stdio.h>
#include "spiGyro_lib.h"
#include "diag/Trace.h" // Trace output via STDOUT

// == Defines ==

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// == Global Variables ==
float gyro[3];

// == Declarations ==

int main(int argc, char* argv[]) {

  volatile uint32_t delay_counter = 0;

  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ; // Start up delay
  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ;
  for (delay_counter = 0; delay_counter < 655350; delay_counter++)
    ;

  lcd_init();
  lcd_command(LCD_CLEAR_DISPLAY);
  lcd_string("Hello World");
  init_leds();

  init_spi();
  setup_gyro_registers();

  for (;;) {
    for (delay_counter = 0; delay_counter < 655350; delay_counter++)
      ;
    half_on();
    for (delay_counter = 0; delay_counter < 655350; delay_counter++)
      ;
    other_half_on();

    trace_puts("Checking gyro...");
    getGyro(gyro);
    trace_printf("Gyro value 1 is: %f \n", gyro[0]);
  }

  return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
