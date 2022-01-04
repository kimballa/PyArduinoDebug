// (c) Copyright 2021 Aaron Kimball
//
// dbg.h - Add debugger support to your Arduino sketch
//
// To enable the debugging system, use the following steps:
//
// 1) include `dbg` in your  Makefile's `libs` list variable.
// 2) #include <dbg.h>
// 3) replace the words `void setup()` in your sketch with `SETUP()`. e.g.:
//
//     SETUP() {
//       /* your setup function. */
//     }
//
// 4) (Optional) To enable stack tracing, see the `Stack tracing` section below.
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
// * DBG_MAX_STACK_FRAMES: Max number of stack frames to trace on the call stack.
//   Setting shallower than your deepest call stack will cause some backtrace info to be lost.
//   However, this consumes `sizeof(void*) * DBG_MAX_STACK_FRAMES` bytes of memory.
// * DBG_PRETTY_FUNCTIONS: Use pretty-printed function names in ASSERT() and TRACE().
//   Without it, the filename (via __FILE__ macro) will be used instead. Function names will
//   consume RAM whereas filenames are stored in Flash via the `F()` macro.
// * DBG_SERIAL: The name of the serial interface to use. Default is `Serial`.
// * DBG_SERIAL_SPEED: Speed of the serial connection. Default is 57600. The DBG_SERIAL_SPEED_FAST
//   (57600) and DBG_SERIAL_SPEED_SLOW (9600) macros are available for your convenience.
// * DBG_START_PAUSED: The sketch will immediately enter the debugger and require
//   an explicit 'continue' ('C') command before proceeding to your setup() fn.
// * DBG_WAIT_FOR_CONNECT: The sketch will wait for a Serial connection before
//   beginning. (On supported systems - Arduino Leonardo.)
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
// * DBG_NO_STACKTRACE: If defined, stack trace support is disabled. Frees up flash as well as
//   RAM (2 * DBG_MAX_STACK_FRAMES).
// * DBG_NO_TIME: If defined, system time reporting is disabled.
//
// ** Stack tracing:
//
// To use stack tracing on AVR, you must enable function instrumentation hooks with
// `gcc -finstrument-functions` and recompile your sketch and any libraries with this option.
// If using the `arduino.mk` Makefile in this system, set `DBGFLAGS = -g -finstrument-functions`
// before including `arduino.mk`. This will expand your sketch image size by approximately 5--15%.
//
// There is also some performance impact to using -finstrument-functions:
//   __cyg_profile_func_enter: worst case ~44 instructions (~2.7us)
//   __cyg_profile_func_exit: worst case ~25 instructions  (~1.6us)
//
// This may disrupt timing-sensitive code paths. You can declare a function with
// __attribute__((no_instrument_function)) to suppress its inclusion in stacktrace logging.


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

// The call stack for backtracing will be tracked in a buffer of function
// pointers, with this many elements. You can set this to any value between
// 1 and 64.
#ifndef DBG_MAX_STACK_FRAMES
#define DBG_MAX_STACK_FRAMES 8
#endif

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

#define SETUP() void setup()
#define BREAK()
#define ASSERT(x)
#define TRACE(x)
#define DBGPRINT(x)

#else /* DBG_ENABLED */
// Debugger support enabled.

// Under no circumstances enumerate more than this many stack frames.
#define __DBG_STACK_FRAME_LIMIT 64

#ifndef DBG_NO_STACKTRACE
#  if (__DBG_STACK_FRAME_LIMIT != 64) // No, really, don't be clever.
#    pragma GCC warning "DBG stack frame limit must be 64. Resetting."
#    undef __DBG_STACK_FRAME_LIMIT
#    define __DBG_STACK_FRAME_LIMIT 64
#  endif /* limit != 64 */

#  if (DBG_MAX_STACK_FRAMES > __DBG_STACK_FRAME_LIMIT)
#    pragma GCC warning "Max call stack unwind count set higher than limit; unwind count will be capped at 64."
#  elif (DBG_MAX_STACK_FRAMES < 1)
#    pragma GCC error "Max call stack unwind count (DBG_MAX_STACK_FRAMES) must be greater than 0."
#  endif
#endif /* DBG_NO_STACKTRACE */

#include<Arduino.h>  // For typedefs e.g. uint8_t
#include<avr/wdt.h>  // For watchdog timer control

extern void __dbg_setup() __attribute__((no_instrument_function));

/* Enter user breakpoint. */
extern void __dbg_break(const char *funcOrFile, const uint16_t lineno) __attribute__((noinline));
extern void __dbg_break(const __FlashStringHelper *funcOrFile, const uint16_t lineno)
    __attribute__((noinline));

extern bool __dbg_assert(bool test, const char *assertStr, const char *funcOrFile,
    const unsigned int lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(bool test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const unsigned int lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(bool test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const unsigned int lineno) __attribute__((no_instrument_function));
extern bool __dbg_assert(bool test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const unsigned int lineno)
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
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(uint8_t test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(int test, const __FlashStringHelper *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(unsigned int test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned int test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(unsigned long test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(unsigned long test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert(test != 0, assertStr, funcOrFile, lineno);
  return test;
}

inline bool __dbg_assert(void *test, const char *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != NULL, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const char *assertStr, const __FlashStringHelper *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != NULL, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr, const char *funcOrFile,
    const uint16_t lineno) {
  __dbg_assert(test != NULL, assertStr, funcOrFile, lineno);
  return test;
}
inline bool __dbg_assert(void *test, const __FlashStringHelper *assertStr,
    const __FlashStringHelper *funcOrFile, const uint16_t lineno) {
  __dbg_assert(test != NULL, assertStr, funcOrFile, lineno);
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


void __dbg_disable_watchdog() __attribute__((naked, used, no_instrument_function, section(".init3")));

#ifdef DBG_PRETTY_FUNCTIONS
#  define ASSERT(x) __dbg_assert(x, F(#x), __PRETTY_FUNCTION__, __LINE__)
#  define TRACE(x) __dbg_trace(F(#x),  __PRETTY_FUNCTION__, __LINE__)
#  define BREAK() __dbg_break(__PRETTY_FUNCTION__, __LINE__);
#else
#  define ASSERT(x) __dbg_assert(x, F(#x), F(__FILE__), __LINE__)
#  define TRACE(x) __dbg_trace(F(#x),  F(__FILE__), __LINE__)
#  define BREAK() __dbg_break(F(__FILE__), __LINE__);
#endif /* DBG_PRETTY_FUNCTIONS */


#ifdef DBG_START_PAUSED
// Within the SETUP() macro, start paused immediately prior to user's setup().
#  define __optional_immediate_brk() { DBGPRINT("Break on init"); BREAK(); }
#else
#  define __optional_immediate_brk()
#endif /* DBG_START_PAUSED */

#ifdef __AVR_ATmega32U4__
// Arduino Leonardo -- require serial.
#  define WAIT_FOR_CONNECT_SUPPORTED  1
#else
#  define WAIT_FOR_CONNECT_SUPPORTED  0
#endif /* __AVR_ATmega32U4__ */

#if WAIT_FOR_CONNECT_SUPPORTED == 1 && defined(DBG_WAIT_FOR_CONNECT)
#  define __optional_wait_for_conn()  while (!DBG_SERIAL) { delay(1); };
#else
#  define __optional_wait_for_conn()
#endif /* DBG_WAIT_FOR_CONNECT ? */

#define SETUP() \
    static void __user_setup(); /* fwd declare. */ \
    void setup() {                          \
      __dbg_setup();                        \
      __optional_wait_for_conn();           \
      __optional_immediate_brk();           \
      __user_setup();                       \
    }                                       \
    /* user's code starts below. */         \
    static void __user_setup()

#define DBGPRINT(x) __dbg_print(x)

#endif /* DBG_ENABLED */

#ifdef __cplusplus
extern "C" {
#endif /* C++ */

  extern void __cyg_profile_func_enter(void *this_fn, void *call_site)
      __attribute__((used, no_instrument_function));
  extern void __cyg_profile_func_exit(void *this_fn, void *call_site)
      __attribute__((used, no_instrument_function));

#ifdef __cplusplus
} /* Close 'extern "C"' */
#endif /* C++ */


#endif /* _DBG_H */
