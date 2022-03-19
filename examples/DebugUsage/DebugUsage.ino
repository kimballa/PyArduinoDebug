// (c) Copyright 2022 Aaron Kimball
// This file is open source under the terms of the BSD 3-Clause license.
//
// Example usage for the PyArduinoDebug library.

#include<Arduino.h>

// Ensure the debugger is enabled even if we don't compile with -g.
#define DEBUG

// Use function names in TRACE() instead of the filename.
#define DBG_PRETTY_FUNCTIONS

// Uncomment the next line if you want the sketch to wait for the debugger to connect to the
// serial port before proceeding with your sketch.
//#define DBG_WAIT_FOR_CONNECT

// Uncomment the next line if you want the sketch to pause *before* entering your setup() method.
//#define DBG_START_PAUSED

// After setting the configuration with macros above, now include PyArduinoDebug.h.
// If you set or change those macros below this point, they will not take effect.
#include <PyArduinoDebug.h>


// You *must* define a setup method to use this debugger -- even if it's empty.
void setup() {
}

static uint8_t state = 0;

void loop() {
  DBGPRINT("Hello, world");   // Print something to the debug console.
  BREAK();                    // Unconditional breakpoint.
  TRACE("foo");               // Print a message along with the function & line.

  ASSERT(state < 2);          // Break to debugger if this condition is ever violated.
  state = !state;
  DBGPRINT(state);            // DBGPRINT() and TRACE() accept non-string values too.
  delay(500);
}

