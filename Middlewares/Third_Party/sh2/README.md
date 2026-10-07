# SH-2 sensor hub library (BNO085)

CEVA / Hillcrest Laboratories' SH-2 and SHTP code: the protocol layer the
BNO085 speaks. Copied **unmodified** from the `src/` folder of the Adafruit
BNO08x Arduino library, version 1.2.7 (the same code the Pico firmware ran).
Only the SH-2 files were taken, not Adafruit's C++ wrapper.

Licence: Apache License 2.0, see `LICENSE.txt` and `NOTICE.txt`.

The library talks to hardware only through the `sh2_Hal_t` function table
in `sh2_hal.h`. This firmware's implementation of that table, over I2C1, is
in `Devices/bno085/bno085.c`.
