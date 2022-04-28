
PyArduinoDebug
==============

An Arduino debugging library that interfaces with the python-based
[arduino-dbg](https://github.com/kimballa/arduino-dbg) debugger over USB-serial.

Usage
-----

* Include the "PyArduinoDebug" library in your sketch in the Arduino IDE, or --
* Build this library according to the compilation instructions below and include `PyArduinoDebug.h`
  in your sketch source. Add `PyArduinoDebug` to the `libs` list variable in your Makefile.

The first line of your `setup()` method must be `DBGSETUP();` -- this will initialize the debugging
service.

See instructions in the
[`PyArduinoDebug.h`](https://github.com/kimballa/PyArduinoDebug/blob/main/src/PyArduinoDebug.h)
header file for how to use debugging macros to create software breakpoints, assertions, or
emit debug log info to the debugger console.

To debug your sketch you will need the companion
[arduino-dbg](https://github.com/kimballa/arduino-dbg) debugger. See the README.md in
that repository for instructions on how to install and use the debugger itself.

Compiling
---------

I build this with my [Arduino makefile](https://github.com/kimballa/arduino-makefile).

* Clone the makefile project such that `arduino-makefile/` is a sibling of this project directory.
* Create `~/arduino_mk.conf` from the template in that directory and customize it to your board
  and local environment. See other one-time setup instructions in that project's README.md and/or
  the comment header of `arduino.mk`.
* Build this library with `make install`
* Add `libs += PyArduinoDebug` to your Makefile if you also use arduino.mk; or link with
  `gcc -lPyArduinoDebug` if you use a different Makefile system.

License
-------

This project is licensed under the BSD 3-Clause license. See LICENSE.txt for complete details.
