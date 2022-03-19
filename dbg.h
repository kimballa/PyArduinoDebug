// dbg.h - Add debugger support to your Arduino sketch.
// For use with the arduino-dbg python Arduino debugger.
//
// (c) Copyright 2021 Aaron Kimball
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice, this list of
//      conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright notice, this list
//      of conditions and the following disclaimer in the documentation and/or other materials
//      provided with the distribution.
//   3. Neither the name of the copyright holder nor the names of its contributors may be
//      used to endorse or promote products derived from this software without specific prior
//      written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
// ** Setup:
//
// To enable the debugging system, use the following steps:
//
// 1) Include the "PyArduinoDebug" library from the Arduino IDE library manager,
//    or for arduino.mk makefile projects, include `PyArduinoDebug` in your  Makefile's
//    `libs` list variable.
//
// 2) #include <dbg.h> in any files where you want to use the debug API macros,
//    but most importantly in the file containing your `setup()` method:
//
//      /* define DBG_START_PAUSED, etc. here if desired. */
//      #include<dbg.h>
//
//      void setup() {
//        /* your setup function. */
//      }
//
//    You must define a setup() { ... } method, even if its body is empty.
//
//    Under the hood, your `setup()` function will be renamed to `__user_setup()` and
//    invoked by a setup() function that first injects debugger handling into the
//    environment.
//
// ** Timers:
//
// * On AVR, this will use the Timer1 IRQ to listen for debugger interrupts.
// * On SAMD, this will use Timer4.
//
// ** API:
//
// The API is defined as a set of macros; do not call internal `__dbg` functions directly.
// The debugger enable-control (see below) will cleanly no-op the macros when debug mode
// is disabled. Moreover, the macros make possible some things like breakpoint disabling.
//
// BREAK() -- Set an unconditional breakpoint. You can disable and re-enable these in the
//            debugger during runtime with `breakpoint disable` and `breakpoint enable`.
//            Up to 16 per translation unit (each .cpp file, or entire sketch if .ino) can
//            be selectively controlled; subsequent breakpoints are unconditional.
//
// ASSERT(cond)  -- Evaluate `cond` and break if false. Prints assertion expression and
//                  location in the debugger if the assertion fails.
//
// DBGPRINT(msg) -- Print a message to the serial console, printed in the debugger.
// TRACE(msg)    -- Print a message to the serial console, along with file/line info.
//
// ** Configuration:
//
// You can control this library by defining the following macros before dbg.h is #include'd:
//
// * DEBUG or DBG_ENABLED: Debugger support is explicitly enabled.
// * Debugger support is enabled by default if NDEBUG is *not* defined.
//   If NDEBUG *is* defined, you must also define DEBUG or DBG_ENABLED to enforce debugger
//   integration.
//
// * DBG_PRETTY_FUNCTIONS: Use pretty-printed function names in ASSERT() and TRACE().
//   Without it, the filename (via __FILE__ macro) will be used instead. Function names will
//   consume RAM whereas filenames are stored in Flash via the `F()` macro.
// * DBG_SERIAL: The name of the serial interface to use. Default is `Serial`.
// * DBG_SERIAL_SPEED: Speed of the serial connection. Default is 57600. The DBG_SERIAL_SPEED_FAST
//   (57600) and DBG_SERIAL_SPEED_SLOW (9600) macros are available for your convenience.
// * DBG_START_PAUSED: The sketch will immediately enter the debugger and require
//   an explicit 'continue' ('C') command before proceeding to your setup() fn.
// * DBG_WAIT_FOR_CONNECT: The sketch will wait for a Serial connection before
//   beginning. (On supported systems - all SAMD Arduinos, and arduino:avr:leonardo.)
//
// ** Optional component compilation:
//
// The following macros will suppress certain sections of the debugger from being compiled in,
// for environments where memory is tight.
//
// * DBG_NO_GPIO: If defined, GPIO control is not compiled into the debugger service. Saves
//   approximately 120 bytes of flash.
// * DBG_NO_MEM_REPORT: If defined, memory usage reporting is disabled. Saves 6 bytes of RAM,
//   ~70 flash.
// * DBG_NO_TIME: If defined, system time reporting is disabled.
//

#ifndef _DBG_H
#define _DBG_H

// Set the serial interface to use.
#ifndef DBG_SERIAL
#define DBG_SERIAL Serial
#endif

#define DBG_SERIAL_SPEED_FAST (57600)
#define DBG_SERIAL_SPEED_SLOW (9600)

#ifndef DBG_SERIAL_SPEED
#define DBG_SERIAL_SPEED DBG_SERIAL_SPEED_FAST
#endif

#define _DBG_MAX_BP_FLAGS_PER_FILE (16)
typedef unsigned short int bp_bitfield_t;

// Determine whether to enable the debugger, based on preprocessor flags.
#if defined(DEBUG) && !defined(DBG_ENABLED)
#  define DBG_ENABLED
#endif /* DEBUG => DBG_ENABLED */

#if !defined(NDEBUG) && !defined(DBG_ENABLED)
#  define DBG_ENABLED
#endif /* !NDEBUG => DBG_ENABLED */

#if defined(NDEBUG) && defined(DBG_ENABLED)
/* DEBUG/DBG_ENABLED => !NDEBUG. */
#  undef NDEBUG
#endif /* NDEBUG && DBG_ENABLED */

#ifndef DBG_ENABLED /* Suppress debugger support. */

#define BREAK()
#define ASSERT(x)
#define TRACE(x)
#define DBGPRINT(x)

#else /* DBG_ENABLED */
// Debugger support enabled.

#include<Arduino.h>  // For typedefs e.g. uint8_t
#ifdef __AVR_ARCH__
#include<avr/wdt.h>  // For watchdog timer control
#endif /* AVR_ARCH */

// Define bitfield of 16 flags / translation unit to switch on/off up to 16 breakpoints/file
// dynamically from the debugger. Initially set to 'all enabled'.
static bp_bitfield_t breakpoint_en_flags = (bp_bitfield_t)(-1);

extern void __dbg_setup() __attribute__((no_instrument_function));

/* Enter user breakpoint. */
extern void __dbg_break(const uint8_t flag_num, uint16_t* flags,
    const char *funcOrFile, const uint16_t lineno) __attribute__((noinline));
extern void __dbg_break(const uint8_t flag_num, uint16_t* flags,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno)
    __attribute__((noinline));

extern bool __dbg_assert(const bool test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(const bool test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(const bool test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(const bool test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno)
    __attribute__((no_instrument_function));

// Forward-declare the inline methods to attach no_instrument_function attribute to each of them.
inline bool __dbg_assert(uint8_t test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(uint8_t test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(void *test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(void *test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) __attribute__((no_instrument_function));
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) __attribute__((no_instrument_function));

// Define inline method-type overrides for __dbg_assert()
inline bool __dbg_assert(uint8_t test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(unsigned int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(unsigned long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert((bool)(test != 0), assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(void *test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != NULL), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != NULL), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert((bool)(test != NULL), assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert((bool)(test != NULL), assertStr, funcOrFile, lineno);
  return test;
}

extern void __dbg_print(const char *message) __attribute__((no_instrument_function));
extern void __dbg_print(const __FlashStringHelper *msg) __attribute__((no_instrument_function));
extern void __dbg_print(bool msg) __attribute__((no_instrument_function));
extern void __dbg_print(uint8_t msg) __attribute__((no_instrument_function));
extern void __dbg_print(int msg) __attribute__((no_instrument_function));
extern void __dbg_print(long msg) __attribute__((no_instrument_function));
extern void __dbg_print(unsigned int msg) __attribute__((no_instrument_function));
extern void __dbg_print(unsigned long msg) __attribute__((no_instrument_function));

extern void __dbg_trace(const char *tracemsg, const char *funcOrFile, const uint16_t lineno)
    __attribute__((no_instrument_function));
extern void __dbg_trace(const char *tracemsg, const __FlashStringHelper *funcOrFile, const uint16_t lineno)
    __attribute__((no_instrument_function));
extern void __dbg_trace(const __FlashStringHelper *tracemsg, const char *funcOrFile, const uint16_t lineno)
    __attribute__((no_instrument_function));
extern void __dbg_trace(const __FlashStringHelper *tracemsg, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno)
    __attribute__((no_instrument_function));


#ifdef __AVR_ARCH__
// On AVR if we use WDT to reset the device, disable WDT early in boot process.
void __dbg_disable_watchdog() __attribute__((naked, used, no_instrument_function, section(".init3")));
#endif /* AVR */

#ifdef DBG_PRETTY_FUNCTIONS
#  define ASSERT(x) __dbg_assert(x, F(#x), __PRETTY_FUNCTION__, __LINE__)
#  define TRACE(x) __dbg_trace(F(#x),  __PRETTY_FUNCTION__, __LINE__)
#  define BREAK() __dbg_break((const uint8_t)__COUNTER__, &breakpoint_en_flags, \
    __PRETTY_FUNCTION__, (const uint16_t)__LINE__);
#else
#  define ASSERT(x) __dbg_assert(x, F(#x), F(__FILE__), __LINE__)
#  define TRACE(x) __dbg_trace(F(#x),  F(__FILE__), __LINE__)
#  define BREAK() __dbg_break((const uint8_t)__COUNTER__, &breakpoint_en_flags, \
    F(__FILE__), (const uint16_t)__LINE__);
#endif /* DBG_PRETTY_FUNCTIONS */


#ifdef DBG_START_PAUSED
// Within the setup() macro, start paused immediately prior to user's setup().
#  define __optional_immediate_brk() { DBGPRINT("Break on init"); BREAK(); }
#else
#  define __optional_immediate_brk()
#endif /* DBG_START_PAUSED */

#if defined(__AVR_ATmega32U4__) or defined(ARUDINO_ARCH_SAMD)
// Arduino AVR Leonardo and SAMD-based systems support `Serial::operator bool()`;
// allows user to require (via DBG_WAIT_FOR_CONNECT) serial connection before proceeding.
#  define WAIT_FOR_CONNECT_SUPPORTED  1
#else
#  define WAIT_FOR_CONNECT_SUPPORTED  0
#endif /* arch detection */

#if WAIT_FOR_CONNECT_SUPPORTED == 1 && defined(DBG_WAIT_FOR_CONNECT)
#  define __optional_wait_for_conn()  while (!DBG_SERIAL) { delay(1); };
#else
#  define __optional_wait_for_conn()
#endif /* DBG_WAIT_FOR_CONNECT ? */

#define setup(x) \
    __user_setup(); /* fwd declare */       \
    void setup() {                          \
      __dbg_setup();                        \
      __optional_wait_for_conn();           \
      __optional_immediate_brk();           \
      __user_setup();                       \
    }                                       \
    /* user's code starts below. */         \
    void __user_setup()

#define DBGPRINT(x) __dbg_print(x)

#endif /* DBG_ENABLED */

#endif /* _DBG_H */