# Hardware checklist

What to run on the bench to turn "built" into "tested". Work top to bottom:
each group assumes the ones before it passed. Write down the numbers you see,
not just pass/fail.

**Propellers off for everything here.**

---

## 1. Rerun the automated bench test

```bash
python tools/hil/hil_rtos.py --port COM12
```

The 5 Oct run ([report](../tools/hil/hil_rtos_report.md)) passed 18 of 19
tests. Rerun it on the current firmware (Debug build), with the PA9→PA10
jumper fitted. Things to look at in the new report:

- **Comms CPU load.** It was 27% idle and 54% with two frames per tick, from
  the busy-wait transmit. With DMA transmit it should be close to 0%.
- **Loopback and soak.** Telemetry frames still arrive with 0 CRC failures.
  This confirms the DMA transmit path.
- **I2C test.** Run with an MPU-6050 on I2C1. This is now at 400 kHz and
  through the mutex.
- **Stack marks** for the new tasks (IMU, Depth, SPIOwner). Each should have
  at least 25% free.
- **Heap.** Free must still equal the minimum ever free.

Also flash the **Release** build once and check the boot banner says
`IWDG: ENABLED` and has no `BENCH_HIL` box.

## 2. Fault handling

1. Debug build with `-DENABLE_IWDG`, scope on one thruster pin (or the PA15
   capture jumper), outputs at a non-neutral value via the `P` bench key.
2. Trigger a fault (`F` key, or a temporary null-pointer write in a task).
3. **Expected:** the pin returns to 1500 µs within one 20 ms frame, the board
   resets about 1 s later, and the next boot prints
   `*** LATCHED FAULT FROM PREVIOUS RUN ***`. For a real CPU fault the kind is
   `HARDFAULT` and `info` shows the fault registers (`C=... H=...`).

## 3. Thruster outputs

1. Scope or capture each of PB4, PB5, PB0, PB1, PC6, PC7, PC8, PC9: a 20 ms
   period and 1500 µs high, from boot.
2. ESCs connected (current-limited supply): all eight arm and stay still.
3. Failsafe: with commands streaming and the vehicle armed, pull the Pi link.
   All eight go to 1500 µs within 500 ms. Two valid packets after reconnecting
   must not move anything; the third does, and the outputs ramp up.
4. Disarm from the Pi, then re-arm: the outputs ramp up from 1500 µs again
   (they used to jump straight to the command).

## 4. Sensors, one at a time

Check each with the `h` report on the console, and the TFT once it works.

**BNO085 (I2C1, PB8/PB9)**
- Boot prints `BNO085: open`. `h` shows `BNO085=ok`.
- With the VN-200 absent the IMU source is `BNO085`: `IMU source: none -> BNO085`.
- Tilt it 90° about each axis and note which of roll, pitch and yaw changes,
  and the sign. Write this down: it's needed before the VN-200 and BNO085 are
  used interchangeably.
- Unplug it: the source goes to `none` within ~0.2 s and the state to
  `FAULTED`. Plug it back in: `BNO085: open` again within a few seconds.

**Bar30 (I2C1)**
- Boot prints `Bar30: ok, surface ~101000 Pa`.
- In air the depth reads within a few cm of 0. In a bucket, 10 cm and 30 cm
  read within ±2 cm.
- Unplug and replug: `FAULTED`, then `Bar30: ok` again, with no hang.

**Analog pressure sensor (PA4)**
- **Before connecting:** with a multimeter, the divider output must not exceed
  2.25 V at 4.5 V in. Never put the sensor straight onto the pin.
- Boot prints `ADC depth: zero ~500 mV`, plus `ADC depth: ok`.
- With the Bar30 unplugged the depth source switches to `ADC` within ~0.2 s.
  Plug the Bar30 back in and it switches back after 1 s.
- Bucket test: expect only ±0.5–1 m accuracy (1% of the 1 MPa range).

**Wayfinder DVL (UART4 through the RS-232 converter)**
- **Never connect the DVL's RS-232 lines straight to the STM32.** Read the
  Wayfinder guide's notes on running it out of water first.
- In air: `h` shows `dvl frame` ok rising and bad at 0, and the TFT shows
  `DVL no bottom lock`.
- Note which of `sum incl` / `sum excl` increases. That tells which bytes the
  checksum covers. Then remove the other branch from `checksum_ok()` in
  `Devices/dvl/dvl.c`.
- In water, move it by hand: the velocity sign must match the direction.

**VN-200 (USART3) — when available**
- Configure it once from a PC with the three commands in `Devices/vn200/vn200.h`.
- `h` shows `vn200 pkt` ok rising at ~100 per second and bad at 0.
- IMU source becomes `VN200` (preferred over the BNO085).
- Tilt test as for the BNO085, and compare the two.

## 5. SD card and display

- Boot prints `SD: SDHC, log starts at block N`. Log for a few minutes, power
  cycle, and check N has moved on: the previous run was kept.
- If you have an old card of 2 GB or less, check it says `SDSC` and logs too.
- Pull the card while logging: the error count rises, nothing else is affected.
- TFT: the status screen appears at boot and updates twice a second. If it
  stays white or black, the start-up sequence in `Devices/ili9341/ili9341.c`
  may need the longer command table from Adafruit's ILI9341 library.

## 6. With the Pi

- The Pi receives `0x01`, `0x03` and `0x04` frames with 0 CRC errors, using the
  `struct` formats in `Protocol/struct.h`.
- About 60% of the link is in use at 115200 baud. Raise the baud rate on both
  ends before adding anything else.

## Not covered here

Anything that needs the vehicle: PID tuning (the gains are zero), thruster
calibration against the allocation matrix, sensor mounting and alignment, and
the sealed enclosure.
