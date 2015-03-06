/*
 * -----------------------------------------
 * EEE3061W MECHATRONICS DESIGN PROJECT 2015
 * -----------------------------------------
 *
 * @file: main.c
 * @description: Main source file
 * @authors: Team 13
 *
 */

#define TESTING

// ----------------------------------------------------------------------------
// == Includes
#include <stdio.h>
#include "diag/Trace.h" // In case we would like to use trace output

// == Defines
#define EVER ;;
#define STM32F051

// ----------------------------------------------------------------------------
//
// Standalone STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// == Global Variables

// == Global Declarations

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/*
 * ============================================================================
 * @brief   | Main program execution
 * @param   | none
 * @return  | 0 (never)
 * ============================================================================
 */

int main(int argc, char* argv[]) {
  // TODO: Add some code!
  // Infinite loop
  while (1) {
  }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
