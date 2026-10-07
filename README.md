# TIBURON AUV — STM32 firmware (FreeRTOS)

Low-level controller for our team's underwater vehicle, running on an
STM32F446RE (Nucleo board). Drivers are written directly on the registers
(no HAL), with FreeRTOS for scheduling.

## What it does

The vehicle has two computers:

- a **Raspberry Pi**, which does vision, navigation and sensor fusion;
- this **STM32**, which talks to the sensors and drives the thrusters.

The STM32:

1. reads the sensors: an IMU, a depth sensor and a DVL (velocity over the
   seabed);
2. sends their readings to the Pi;
3. receives commands from the Pi (where the vehicle is and where it should be);
4. runs a PID controller at 50 Hz and drives 8 thrusters;
5. stops the thrusters by itself if something goes wrong.

```
 Sensors ──► STM32 ──► readings ──► Raspberry Pi
                 ◄── commands ◄────
             │
             └──► 8 thrusters
```

## Hardware

| Part | Connected to |
|---|---|
| Raspberry Pi | USART1 (PA9 / PA10), 115200 baud |
| VN-200 IMU (main IMU) | USART3 (PC10 / PC11) |
| BNO085 IMU (backup) | I2C1 (PB8 / PB9) |
| Bar30 depth sensor (main) | I2C1 (PB8 / PB9) |
| Analog pressure sensor (backup depth) | ADC on PA4, through a 2:1 voltage divider |
| Wayfinder DVL | UART4 (PA0 / PA1), through an RS-232 converter |
| SD card + ILI9341 display | SPI2 |
| 8 thrusters (ESCs) | TIM3 and TIM8 PWM, 1100–1900 µs, 1500 µs = stop |
| Debug console | USART2, the Nucleo's USB serial port |

SD card CS is PC4; the display uses CS PC5, DC PC0, RST PC1.

## How the firmware is organised

The work is split into FreeRTOS tasks. A higher-priority task always runs
first, so the important work is never held up by the less important work.

| Priority | Task | Job |
|---|---|---|
| 7 (highest) | `control_task` | Runs every 20 ms: PID, thrusters, safety checks |
| 5 | `comms_task` | Talks to the Pi |
| 4 | `imu_task` | Reads the IMUs |
| 4 | `depth_task` | Reads the depth sensors |
| 4 | `dvl_task` | Reads the DVL |
| 3 | `spi_owner_task` | Writes to the SD card and the display |
| 2 | `logging_task` | Groups log records into SD blocks |
| 1 | `dummy_task` | Prints to the debug console, blinks the LED |

The control task is on top because it is the only one with a hard deadline:
the thrusters must be updated every 20 ms. Printing is at the bottom because
it is never urgent.

### How tasks pass data

Tasks don't share variables directly; they pass data through FreeRTOS queues.

- **Commands and log records** use normal queues (first in, first out),
  because every one of them matters.
- **Sensor readings** use a one-slot queue that is overwritten with each new
  reading, because only the latest reading matters. Any task can read it
  without removing it.
- The control task never waits on a queue. If the SD card is slow, a log
  record is dropped (and counted), but the thrusters are never delayed.

Interrupts don't do any real work. They just wake up the task that will
handle the data (using FreeRTOS task notifications).

### How the UARTs receive data

Each UART uses **DMA**: the hardware copies incoming bytes into a buffer
without the CPU. An interrupt then wakes the task when:

- the line goes quiet (a packet has probably finished), or
- the buffer is half full or full.

The second part matters: with only the first, a burst of exactly one buffer
length looked like no data at all. The bench test reproduces this.

Sending to the Pi uses DMA too, so the CPU never waits for bytes to go out.

## Sensors

Each sensor has a driver in `Devices/`. If a sensor isn't connected, its task
keeps running and reports it as missing on the console.

**Main and backup sensors.** There are two IMUs and two depth sensors. Each
task uses the main sensor (VN-200, Bar30) while it is working. If it stops
sending data for 200 ms, the task switches to the backup (BNO085, analog
sensor). It switches back once the main sensor has been working for 1 s, so a
loose wire doesn't make it flip back and forth. Every reading says which sensor
it came from.

| Sensor | Notes |
|---|---|
| VN-200 | Sends yaw/pitch/roll, gyro and acceleration in binary packets at 100 Hz. Packets are checked with a CRC. |
| BNO085 | Uses the manufacturer's SH-2 library (the same one our Pico code used). No magnetometer, because it is unreliable next to motors. |
| Bar30 | Pressure → depth, for fresh water. Its calibration data is checked with a CRC at start-up. |
| Analog pressure sensor | 0–1 MPa sensor; a backup only, accurate to about ±0.5–1 m. |
| DVL | Velocity over the seabed. When it can't see the bottom, the reading is sent but marked invalid. |

## Talking to the Pi

Same packet format as our Pico firmware, so the existing Pi code works:

```
0xAA 0x55 | length | type | 56-byte payload | CRC-16
```

| Type | Direction | Contents |
|---|---|---|
| 0x01 | STM32 → Pi, 50 Hz | Status: armed, PID outputs, thruster values |
| 0x02 | Pi → STM32 | Current position and target position, armed flag |
| 0x03 | STM32 → Pi, 50 Hz | IMU and depth readings |
| 0x04 | STM32 → Pi | DVL readings, when a new one arrives |

The exact byte layouts are in `Protocol/struct.h`.

## Safety

| If... | then... |
|---|---|
| the Pi stops sending commands for 500 ms | the thrusters go to stop and the vehicle disarms |
| the control task freezes | the watchdog resets the board within about 1 s |
| the program crashes | the thrusters go to stop first, then the crash is saved and reported on the next boot |

**The command timeout is checked inside the control task, not the comms task.**
If the comms task crashes, the timeout still works.

After a timeout, the vehicle only re-arms after **3 good packets in a row**.
The thrusters then ramp up slowly from stop instead of jumping.

**Build types.** The *Release* build is the one for the vehicle and always has
the watchdog. The *Debug* build has test features and no watchdog, so the
debugger can pause it.

## Logging and display

- Every 200 ms a log record (time, position, thrusters, sensor readings) is
  saved to the SD card. Each new power-up continues after the previous run, so
  old logs aren't overwritten.
- The TFT shows a status screen: armed or not, sensor readings, which sensors
  are working.

## Testing

| What | How | Status |
|---|---|---|
| RTOS timing, Pi link, failsafe, watchdog, I2C | `tools/hil/hil_rtos.py` drives the board from a laptop | 18 of 19 tests passed on the bench ([report](tools/hil/hil_rtos_report.md)) |
| Sensor parsers and maths | PC tests in `tests/host`, checked against the datasheets | All pass |
| New sensor drivers, display, SD log changes | `docs/HARDWARE_CHECKLIST.md` | Not yet run on the board |

Measured on the bench: the control loop runs every 20 ms with about 1 µs of
jitter, and each run takes about 0.1 ms.

The PID gains are still zero: the controller runs, but it has to be tuned in
the water.

## Building

- **STM32CubeIDE:** open the project and build *Debug* or *Release*.
- **PC tests:** `sh tests/host/run.sh`
- **Bench test:** `python tools/hil/hil_rtos.py --port COM12`
- **Console** (USB serial, 115200): press `h` for a health report, `s` for
  stack usage.

## Folder layout

```
Core/        tasks, main, FreeRTOS config
Drivers/     UART, I2C, SPI, timer and watchdog drivers
Devices/     one folder per sensor / device
Protocol/    Pi packets, SD card
Middlewares/ FreeRTOS, BNO085 library
tests/host/  PC tests
tools/hil/   bench test script
docs/        hardware checklist, datasheet list
```
