# TIBURON AUV — FreeRTOS firmware (STM32F446RE)

Firmware for the low-level controller of an 8-thruster AUV, on a NUCLEO-F446RE.
Register-level drivers (no HAL), FreeRTOS on top.

The STM32 is the vehicle's sensor hub and thruster controller:

- reads the IMU (VN-200, or a BNO085 as backup), the depth sensor (Bar30, or an
  analog pressure sensor as backup) and the Wayfinder DVL;
- sends all of it to the Raspberry Pi, which does navigation and sensor fusion;
- takes setpoints and the fused pose back from the Pi, runs a 6-DOF PID at 50 Hz,
  maps it to 8 thrusters, and drives the ESCs;
- stops the thrusters on its own if the Pi goes quiet, if the firmware hangs,
  or if the CPU faults.

This started as a bare-metal superloop
([Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal))
and was moved onto FreeRTOS, then tested on the bench.

---

## Contents

1. [What has been tested, and how](#1-what-has-been-tested-and-how)
2. [Hardware and pins](#2-hardware-and-pins)
3. [How the firmware is organised](#3-how-the-firmware-is-organised)
4. [Tasks and priorities](#4-tasks-and-priorities)
5. [How tasks talk to each other](#5-how-tasks-talk-to-each-other)
6. [Interrupts and receiving with DMA](#6-interrupts-and-receiving-with-dma)
7. [Sensors](#7-sensors)
8. [Choosing between two sensors](#8-choosing-between-two-sensors)
9. [The Pi link protocol](#9-the-pi-link-protocol)
10. [Control loop and failsafes](#10-control-loop-and-failsafes)
11. [SPI bus, SD logging and display](#11-spi-bus-sd-logging-and-display)
12. [Memory, stacks and timing](#12-memory-stacks-and-timing)
13. [Bugs found and fixed](#13-bugs-found-and-fixed)
14. [Building and testing](#14-building-and-testing)
15. [Not done yet](#15-not-done-yet)
16. [Design decisions in one line each](#16-design-decisions-in-one-line-each)

---

## 1. What has been tested, and how

Three levels of evidence, kept separate on purpose:

| Level | What it covers |
|---|---|
| **On hardware** (bench, 5 Oct 2026, [`tools/hil/hil_rtos_report.md`](tools/hil/hil_rtos_report.md)) | Clock tree, 50 Hz control timing and jitter, Pi link over the real UART (loopback), resync through junk, bad-CRC rejection, the command-timeout failsafe (including with the comms task suspended), 3-packet recovery, the DMA wrap bug and its fix, I²C reads under preemption, a 60 s soak, stack high-water marks, the fault latch, the watchdog. 18 of 19 tests passed; the PWM capture test was skipped (no jumper fitted). |
| **On the PC, against the datasheets** ([`tests/host`](tests/host)) | VN-200 packet parsing and CRC, Bar30 CRC-4 and compensation (against the datasheet's worked example), DVL frame parsing, analog-sensor conversion, and the sensor-switching logic. 211 checks. |
| **Built, not yet run** | Everything written after that bench run: the IMU and depth tasks, the BNO085, Bar30 and analog drivers on this board, the new packets, DMA transmit, the fault-handler change. Each is marked `UNTESTED` in its source until it has been run. |

The BNO085, Bar30 and analog sensor have all been run on the team's Pico
firmware, so the sensors, wiring and settings are known good. The STM32 drivers
for them are new code and still need their own bench run. The VN-200 and DVL
have not been connected to anything yet.

**The PID gains are zero.** The loop runs end to end (error, integral,
derivative, saturation, allocation, PWM) but outputs zero until it is tuned in
the water.

---

## 2. Hardware and pins

| | |
|---|---|
| MCU | STM32F446RE, 180 MHz (8 MHz HSE bypass from the ST-LINK, PLL, over-drive on) |
| RTOS | FreeRTOS, `heap_4`, ARM_CM4F port, single-precision FPU |
| Thrusters | 8 × Blue Robotics T200 / Basic ESC, 1100–1900 µs, 1500 µs neutral, 50 Hz frame |

| Function | Peripheral | Pins |
|---|---|---|
| Debug console | USART2 (ST-LINK virtual COM), 115200 | PA2 / PA3 |
| Raspberry Pi | USART1, 115200, RX DMA2 S2, TX DMA2 S7 | PA9 TX / PA10 RX |
| VN-200 | USART3, 115200, RX DMA1 S1 | PC10 / PC11 |
| Wayfinder DVL (via RS-232 converter) | UART4, 115200, RX DMA1 S2 | PA0 / PA1 |
| Bar30 (0x76) and BNO085 (0x4A/0x4B) | I2C1, 400 kHz | PB8 SCL / PB9 SDA |
| Analog pressure sensor | ADC1 channel 4, through a 2:1 divider | PA4 |
| SD card + display | SPI2 | PB13 / PB14 / PB15, SD CS PC4 |
| Display control | GPIO | CS PC5, DC PC0, RST PC1 |
| Thrusters 1–4 | TIM3 CH1–4 | PB4, PB5, PB0, PB1 |
| Thrusters 5–8 | TIM8 CH1–4 | PC6, PC7, PC8, PC9 |
| Heartbeat LED | GPIO | PA5 |

Pin details that broke things before they were caught:

- **PB4 boots as NJTRST** (a JTAG pin). Its mode and alternate-function bits have
  to be cleared, not OR-ed, or thruster 1 never moves.
- **TIM8 is an advanced timer**: outputs stay disconnected until `BDTR.MOE` is set.
- **The two PWM timers need different prescalers**: TIM3's clock is 90 MHz,
  TIM8's is 180 MHz. Both count at 1 MHz (1 µs per count), 20 ms period.
- All eight compare registers are set to 1500 µs **before** the outputs are
  enabled, so an ESC never sees a 0 µs pulse.

---

## 3. How the firmware is organised

```
Core/        tasks, main, FreeRTOS config, interrupt handlers, console, fault latch
Drivers/     register-level peripheral drivers: uart, i2c, spi, timers, watchdog
Devices/     one folder per external device: vn200, bno085, bar30, adc_depth, dvl, oled
Protocol/    Pi packet framing, CRC-16, ring buffer, SD card + log layout
Middlewares/ FreeRTOS kernel; CEVA's SH-2 library for the BNO085 (unmodified)
tests/host/  PC unit tests for the parsers and maths
tools/hil/   laptop-driven bench test (hil_rtos.py) and its last report
docs/        hardware checklist, list of datasheets used (docs/datasheets)
```

Data flow, from sensor to thruster:

```
VN-200  ─USART3 DMA─┐                             ┌─► comms_task ─USART1 DMA─► Pi
BNO085  ─I2C1──────►├─ imu_task ───► imuQueue ────┤      (telemetry, sensors, DVL frames)
                    │                             │
Bar30   ─I2C1──────►├─ depth_task ─► depthQueue ──┤
Analog  ─ADC1──────►┘                             ├─► logging_task ─► SD card
DVL     ─UART4 DMA────── dvl_task ─► dvlQueue ────┘

Pi ─USART1 DMA─► comms_task ─► commandQueue ─► control_task ─► TIM3/TIM8 ─► 8 ESCs
                                                    ▲
                            TIM7 interrupt, 50 Hz ──┘
```

Sensor fusion happens on the Pi. The pose the PID controls on comes back from
the Pi in every command packet; the STM32's own sensors are sent up as raw
readings and are not inputs to its controller.

---

## 4. Tasks and priorities

| Prio | Task | Wakes on | Rate | Stack | Does |
|---|---|---|---|---|---|
| 7 | `control_task` | TIM7 interrupt (task notification) | 50 Hz | 1 KB | PID, thrust allocation, PWM, **command timeout**, watchdog refresh |
| 5 | `comms_task` | USART1 RX, control's tick, TX-done (notification bits) | 50 Hz out | 1 KB | parses commands, sends telemetry / sensors / DVL frames |
| 4 | `imu_task` | USART3 RX, or a 5 ms timeout to poll the BNO085 | 100 Hz | 1.5 KB | VN-200 + BNO085 → `imuQueue` |
| 4 | `depth_task` | `vTaskDelayUntil`, 20 ms | ~35 Hz | 1.5 KB | Bar30 + analog sensor → `depthQueue` |
| 4 | `dvl_task` | UART4 RX | per ping | 1 KB | Wayfinder frames → `dvlQueue` |
| 3 | `spi_owner_task` | its request queue, or 500 ms timeout | — | 1 KB | the only task that touches SPI2 (SD card, display) |
| 2 | `logging_task` | `logQueue` | 5 Hz | 1 KB | packs log records into 512-byte SD blocks |
| 1 | `dummy_task` | `consoleQueue`, or 50 ms timeout | — | 1.5 KB | the only task that prints; heartbeat LED; `s` / `h` reports |
| 0 | idle | | | | FreeRTOS's own |

**Why this order.**

- **Control is on top** because it is the only thing with a hard deadline that
  moves hardware. Nothing below it can delay a thruster update.
- **Comms is next**: commands and telemetry are time-sensitive, but a late
  telemetry frame is harmless and the failsafe does not depend on comms (§10).
- **Sensor tasks share priority 4.** Their work is short (parse a few hundred
  bytes, or wait for a conversion) and they block most of the time. Time
  slicing is on, so equal-priority tasks take turns; none can hog the CPU.
- **The SPI owner sits at 3**: above `logging_task` (which feeds it, so logging
  can't pile up more work mid-transfer) and below every task with a deadline
  (an SD write can take 250 ms and must be preemptible).
- **Console at the bottom**: printing is never urgent, and putting the
  heartbeat LED here means a blinking LED proves the *whole* schedule runs.

**One task for two IMUs, one for two depth sensors** (rather than one per chip):
each pair produces the same kind of reading, and choosing between them is
simplest where both are visible (§8).

Every UART interrupt is enabled **from the task that consumes it**, on its first
run, not from `main`. An interrupt that tried to wake a task before the
scheduler started would write through a null stack pointer (PSP is still 0
until the first task starts).

---

## 5. How tasks talk to each other

Two rules decide the shape of every queue:

- **FIFO when every item matters** (commands, log records, console lines).
- **A one-slot "mailbox" when only the latest value matters** (sensor readings):
  the writer uses `xQueueOverwrite`, readers use `xQueuePeek`, so the writer
  never blocks, readers never wait, and several readers can share one value.

| Queue | From → to | Holds | Kind | When full |
|---|---|---|---|---|
| `commandQueue` | comms → control | `CommandPayload` | FIFO, 4 deep | wait 1 ms, then drop and count. Control empties it every tick and uses the newest |
| `imuQueue` | imu_task → comms, logging | `ImuSample` | mailbox | newest overwrites |
| `depthQueue` | depth_task → comms, logging | `DepthSample` | mailbox | newest overwrites |
| `dvlQueue` | dvl_task → comms, logging | `DVLData` | mailbox | newest overwrites |
| `logQueue` | control → logging | log record or "flush" | FIFO, 8 deep | **never waits**: drop and count |
| `spiRequestQueue` | logging → SPI owner | a whole 512-byte block | FIFO, 2 deep | wait 500 ms, then drop and count |
| `consoleQueue` | any task → console | an 80-char line | FIFO, 8 deep | never waits: drop and count |

**Nothing on the control path ever blocks.** `control_task` reads
`commandQueue` with a zero timeout and posts to `logQueue` with a zero timeout.
An SD card stuck in a 250 ms write can fill the log queue; the cost is lost log
records (counted), never a missed control tick.

**Every place data can be dropped has a counter**, shown by the `h` report.

Other ways tasks synchronise:

| Mechanism | Used for | Why this one |
|---|---|---|
| Task notifications | interrupt → task wake-ups (TIM7, every UART) | lighter and faster than a semaphore; one waiting task per source |
| Notification **bits** | `comms_task`: RX / telemetry tick / TX done | one task, several events; bits are latched, so two events before it runs are both seen |
| Mutex | I2C1 (Bar30 in `depth_task`, BNO085 in `imu_task`) | held per transfer; priority inheritance; released during the Bar30's 12 ms conversions |
| A single owning task | SPI2 (SD card, display) | the bus needs a different clock speed per device; one owner sets it every time, so no caller can forget |
| Critical section | `control_loop_snapshot()` | copies pose, PID output and PWM together, so a telemetry frame never mixes two control ticks |

---

## 6. Interrupts and receiving with DMA

FreeRTOS's rule: an interrupt may call a `...FromISR` function only if its
priority number is **5 or higher** (`configMAX_SYSCALL_INTERRUPT_PRIORITY`).
Lower numbers are more urgent and are not masked by the kernel's critical
sections, so they could interrupt the kernel mid-update.

| Interrupt | Priority | Calls |
|---|---|---|
| TIM7 (control tick) | 5 | `vTaskNotifyGiveFromISR` |
| USART1 + DMA2 S2 (Pi RX) | 5 | `xTaskNotifyFromISR` |
| DMA2 S7 (Pi TX done) | 6 | `xTaskNotifyFromISR` |
| USART3 + DMA1 S1 (VN-200) | 6 | `xTaskNotifyFromISR` |
| UART4 + DMA1 S2 (DVL) | 6 | `xTaskNotifyFromISR` |
| USART2 (console RX) | 6 | nothing (stores one byte) |

A UART and its RX DMA stream always have the **same** priority: both update the
same "last position" variable, and equal priorities can't interrupt each other,
so no lock is needed.

**How every UART receives.** The DMA copies bytes into a 256-byte circular
buffer with no CPU involvement. Three interrupts move new bytes into a software
ring and wake the task:

- **IDLE line** — the sender paused, so a packet has probably finished;
- **half transfer** and **transfer complete** — the DMA reached the middle or
  the end of the buffer.

IDLE alone is not enough, and the bench proved it: the code works out "new
bytes" from the DMA's position, and if exactly 256 bytes arrive between two
IDLE events the position comes back to where it was and the whole burst looks
like nothing arrived. A sensor that streams with no gaps never raises IDLE at
all. The half/complete interrupts fire every 128 bytes, so the DMA can never go
a full lap unseen. The `w` / `v` bench test sends exactly 256 bytes with and
without the fix and shows the difference.

**Sending to the Pi** uses DMA too. `comms_task` builds whole frames in a
4-slot ring; the DMA sends one, its transfer-complete interrupt notifies the
task, and the task starts the next. Before this, the task wrote each byte
itself and waited on the UART in between: 5.4 ms per frame, measured at **27%
of the CPU** at 50 Hz (54% with two frames per tick).

---

## 7. Sensors

### VN-200 (USART3)

Set up once from a PC (VectorNav Control Center or a terminal) and saved to the
sensor, so the firmware only listens. From the user manual (UM004):

```
$VNWRG,06,0*XX                 turn off ASCII output
$VNWRG,75,1,8,01,0128*XX       binary output 1: serial port 1, 800/8 = 100 Hz,
                               group 1 (Common), fields 0x0128 =
                               YawPitchRoll (bit 3) + AngularRate (bit 5) + Accel (bit 8)
$VNWNV*57                      save to flash
```

That gives a 42-byte packet: `0xFA`, group byte, 2-byte field mask, 36 bytes of
floats (yaw/pitch/roll in degrees, angular rate in rad/s, acceleration in m/s²
including gravity), CRC-16. The CRC is checked the way the manual suggests: run
it over everything after the sync byte **including** the CRC, and a good packet
gives 0. On a bad packet the parser slides forward to the next `0xFA` rather
than throwing away 42 bytes, so a real packet starting inside a bad one is not
lost.

### BNO085 (I2C1, no INT/RST pins)

CEVA's SH-2 library does the protocol (it's the code inside the Adafruit
library the Pico used). The firmware provides the four functions it needs:
open (soft reset, which also finds the address), read, write, and a
microsecond clock. Without the INT pin, the sensor is polled every 5 ms: read
the 4-byte header, and the whole packet only if the header says one is waiting.

Reports, at 50 Hz: **game rotation vector** (accel + gyro fusion, no
magnetometer, which is unreliable next to thruster motors), calibrated gyro, and
accelerometer. One SHTP packet can carry several reports, so each report type
is cached separately.

Opening the sensor blocks `imu_task` for up to ~0.5 s (300 ms for the sensor to
reboot, up to 200 ms waiting for its start-up messages). The VN-200's receive
ring is 4 KB so that a second of VN-200 data waits safely during that time.

### Bar30 (MS5837-30BA, I2C1)

Reset, read the 7 calibration words, **check their CRC-4**, then for each
reading: start a pressure conversion, wait, read 24 bits; same for
temperature; apply the datasheet's first- and second-order compensation.

- The wait is **12 ms**. A conversion at OSR 4096 takes up to 9.04 ms, and
  `vTaskDelay(10)` can return after only 9 ms (the first tick may be almost
  over). An ADC read during a conversion returns 0, which shows up as a
  negative depth spike.
- The I2C mutex is released during each wait, so the BNO085 can use the bus.
- Depth = (pressure − surface pressure) / (997 kg/m³ × g). Surface pressure is
  measured at start-up only if it looks like air pressure (950–1050 hPa);
  otherwise 1013.25 hPa is used, so a reboot underwater doesn't read 0 m.
- If the sensor stops answering, it is re-initialised when it comes back, so it
  can be plugged in after boot.

### Analog pressure sensor (DFRobot SEN0257, PA4)

0–1 MPa → 0.5–4.5 V, so **250 kPa per volt** (the Pico test sketch used 400,
which reads 1.6× too high). A 2:1 divider keeps the pin at or below 2.25 V.

- 16 ADC samples are averaged per reading. The chip's internal reference is
  measured too, which gives the true supply voltage, so the reading doesn't
  scale with supply error.
- The zero (0.5 V nominal) is measured at start-up and only accepted between
  0.45 and 0.55 V, for the same underwater-reboot reason as the Bar30.
- Accuracy is 0.5–1% of full scale, about **±0.5–1 m**. It's a backup.

### Wayfinder DVL (UART4)

116-byte frames, layout from Teledyne's "Wayfinder Binary Interface Packet
Protocol" page: header, timestamp, velocity x/y/z/error, four beam ranges,
status, voltages, two checksums. When there is no bottom lock the velocities
are **NaN**; those frames are still sent to the Pi, marked
`velocity_valid = 0`, so the Pi can tell "DVL alive, no bottom" from "no DVL".

Teledyne's page doesn't say exactly which bytes the final checksum covers, so
the parser accepts both readings and counts which one matched (`h` report).
The first run with the real DVL settles it.

### Sensor health

Each sensor is `ok`, `ABSENT` (never answered since boot: check the wiring) or
`FAULTED` (answered, then stopped: check the sensor). Health is judged on
**good packets**, not on bytes arriving; a misconfigured VN-200 sends plenty of
bytes and never a valid packet. Every change is printed once on the console.

---

## 8. Choosing between two sensors

`source_select.c`, used for the IMU (VN-200 preferred, BNO085 backup) and depth
(Bar30 preferred, analog backup):

- A sensor is **fresh** if its last reading is ≤ 200 ms old.
- Use the preferred sensor whenever it's fresh.
- If it goes stale, switch to the backup.
- Switch back only after the preferred one has been fresh for **1 s without a
  break**, so a loose connector doesn't make the output flip back and forth.
- Neither fresh: source "none".

The active source travels with every reading (`ImuSample.source`,
`DepthSample.source`) into the Pi packets and the log, and every switch is
printed on the console. It's pure logic with no RTOS calls, so it's tested on
the PC, including the 32-bit tick wrapping through zero.

**Caveat for the Pi:** the VN-200 gives true heading; the BNO085's game rotation
vector starts at an arbitrary yaw. A switch between them is a jump in yaw, and
the two sensors' axis conventions still need checking against each other on
the bench.

---

## 9. The Pi link protocol

Same frame as the team's Pico firmware, so the existing Pi code keeps working:

```
[0xAA][0x55][LEN=56][TYPE][56-byte payload][CRC-16 hi][CRC-16 lo]   62 bytes
```

CRC-16-CCITT (poly 0x1021, init 0xFFFF) over LEN, TYPE and the payload. All
fields little-endian. The receiver resyncs by sliding one byte at a time until
the header, length and CRC all check out.

| Type | Direction | Rate | Contents | Python `struct` |
|---|---|---|---|---|
| `0x01` TELEMETRY | STM32 → Pi | 50 Hz | Pi's depth echoed, onboard depth, PID output ×6, PWM ×8, armed, saturation flags, link ok | see `Protocol/struct.h` |
| `0x02` CMD | Pi → STM32 | Pi's rate | current pose ×6, target pose ×6, armed, sequence | `'<12f3B5s'` |
| `0x03` SENSORS | STM32 → Pi | 50 Hz | IMU source, accuracy, yaw/pitch/roll, gyro, accel, depth source, depth, water temp, ages | `'<IHHBBBB3f3f3fff'` |
| `0x04` DVL | STM32 → Pi | each new DVL frame | velocity valid, velocities, 4 ranges, mean range, status, voltages, age | `'<IHBB4f4ffHHff'` |

Every outgoing payload is zeroed before it's filled, so stale stack bytes
never end up in the CRC. Ages (`*_age_ms`) say how old a reading was when the
frame was built; the Pi should ignore anything older than ~200 ms.

**Link budget at 115200 baud:** a frame is 5.4 ms on the wire. Telemetry +
sensors every 20 ms plus DVL frames is about 60% of the link. Fine for now; the
baud rate should go up when the Pi side is updated.

---

## 10. Control loop and failsafes

Every 20 ms TIM7 wakes `control_task`, which:

1. empties `commandQueue` and takes the newest setpoint and pose;
2. checks the command timeout;
3. if armed: PID per axis (surge, sway, heave, roll, pitch, yaw), clamped, then
   thrust allocation through a precomputed 8×6 pseudo-inverse, with the
   vertical, horizontal and yaw groups scaled separately when they saturate;
4. writes the 8 PWM outputs;
5. refreshes the watchdog;
6. tells `comms_task` to send telemetry, and every 10th tick posts a log record.

Measured on the bench: the TIM7 period is 20 000.00 µs with at most 1.3 µs of
jitter; the task starts 6–7 µs after the interrupt; the loop body takes 17 µs
on average and 109 µs at worst.

### The failsafes, from most to least likely

| Failure | What happens | Where |
|---|---|---|
| Pi stops sending (or sends garbage) | after **500 ms** without a CRC-valid command: disarm, thrusters to 1500 µs, setpoint and PID state cleared | `checkCommandTimeout()` in **`control_task`** |
| `comms_task` hangs | same as above: the timeout only reads a timestamp, it doesn't depend on comms | |
| `control_task` hangs | the watchdog (~1 s) resets the board; the ESCs lose their signal and stop | only `control_task` refreshes the IWDG |
| CPU fault (HardFault etc.) or failed `configASSERT` | thrusters set to 1500 µs first, then the fault and its address are saved in RAM that survives reset, then halt (and the watchdog resets) | `stm32f4xx_it.c`, `fault_latch.c` |

Putting the timeout in the control task rather than in comms is the key
decision: the things that kill the link and the things that hang the comms task
overlap a lot, so the check must not live in either. The bench test suspends
`comms_task` entirely and checks the vehicle still disarms.

**Recovery is deliberately harder than failure.** Going into failsafe takes one
missed deadline. Coming out takes **3 consecutive valid packets**, none more
than 500 ms apart: one packet after a dropout proves nothing. The setpoint was
zeroed on entry, so a stale packet can't bring back the throttle the vehicle
had when the link died.

**Ramp-up.** After any disarm (failsafe or the Pi's own `armed = 0`), the PID's
integral, last error and output are cleared and the slew limiter is switched
back on: outputs walk up from 1500 µs at 50 µs per tick (2500 µs/s) until they
catch up with the command. Then the limiter turns off and the PID has full
authority. Before this was fixed, only the failsafe reset that state; a normal
disarm and re-arm stepped straight to the commanded thrust, with a stale
integral and a one-tick derivative spike.

**Build configurations.** *Release* is the vehicle build: watchdog always on,
bench test hooks never compiled in. *Debug* has the bench hooks and no
watchdog, so a breakpoint doesn't reset the board. A build without the watchdog
says so on boot and blinks the LED in a different pattern.

---

## 11. SPI bus, SD logging and display

SPI2 carries the SD card and the display. One task owns it
(`spi_owner_task`); other tasks send it requests. The bus needs a different
clock speed per device (352 kHz for SD start-up, 5.6 MHz for SD data and the
display), and with one owner the speed is set in one place before every
transfer. A mutex would make every user responsible for setting it, and
forgetting fails only sometimes.

A mutex would be the better choice if a high-priority task ever needed the bus
directly: then a priority-3 owner serving it through a queue is priority
inversion with no inheritance. Nothing at priority ≥ 4 touches SPI.

**Logging.** Every 10th control tick (5 Hz) a 40-byte record (time, pose, PWM,
armed, link, CRC) goes to `logging_task`, which packs 12 of them into one
512-byte SD block with a small header. Writing each record as its own block
would program 12.8× more flash and hit the card's 250 ms worst-case busy time 5
times a second. The cost: up to 2.4 s of records are lost on a sudden power cut
(a disarm or failsafe flushes the block early).

---

## 12. Memory, stacks and timing

Release build (`-Os`): **41 KB flash** of 512 KB, **36.5 KB RAM** of 128 KB
(of which the FreeRTOS heap is 22 KB, the VN-200 ring 4 KB).

**All allocation happens before the scheduler starts**: tasks and queues are
created in `main`, every result is checked, and a failure stops the boot with
its name saved in the fault latch. The bench confirmed it: heap free equals the
minimum-ever free (9448 B) after every test, so nothing allocates at run time.

**Stacks** were sized from the measured high-water marks (`s` on the console)
after the error paths had run (failsafe, junk frames, bad CRC, missing I2C
device, console reports), with headroom on top. That run caught `dummy_task`
using 928 of its 1024 bytes, more than static analysis predicted (the
formatted-print code is bigger than estimated); it now has 1.5 KB. `imu_task`
and `depth_task` get 1.5 KB because they print and call into bigger drivers;
their marks still need measuring.

**printf without floats.** `--specs=nano.specs` has no `%f`, and `%f` in a
varargs call would promote floats to double anyway. Numbers are printed as
fixed-point with `console_fmt_milli()`.

---

## 13. Bugs found and fixed

The interesting ones, and how each was found.

**Found on the bench:**

| Bug | Symptom | Fix |
|---|---|---|
| UART receive used only the IDLE interrupt | a 256-byte burst with no gap vanished | half/full-transfer interrupts too (§6) |
| I2C write set STOP right after loading the last byte | the last byte was never sent (MPU-6050 stayed asleep) | wait for BTF (byte transfer finished) before STOP |
| I2C read cleared ACK in software in time "usually" | under preemption one byte too many was ACKed, corrupting the next read | the reference manual's sequences, which stretch the clock while deciding |
| I2C timeout checked without one last poll | a task woken late reported a timeout on a finished transfer, then sent STOP mid-transfer and wedged the bus | poll once more after the deadline; recover a stuck BUSY flag |
| Console report longer than the queue | the 13-line health report printed 8 lines | the printing task writes its own lines directly |
| Recovery counted "3 packets since the dropout", any spacing | two packets, ten seconds of silence, one more: re-armed | a gap over 500 ms restarts the count |
| `dummy_task` stack | 928 / 1024 bytes used | 1.5 KB |

**Found by checking against datasheets and manuals:**

| Bug | Effect |
|---|---|
| VN-200 field mask `0x0018` in the IMU group | means uncompensated gyro + temperature, not accel + gyro; no real packet could ever match |
| VN-200 CRC read little-endian | every packet would fail |
| Bar30 compensation used C's `/` | rounds negative values toward zero; the datasheet's worked example needs rounding down (19.82 °C instead of 19.81) — fixed with shifts |
| Bar30 never checked its PROM CRC | a corrupted calibration word would scale every reading |
| Analog sensor at 400 kPa/V (Pico sketch) | the part is 0–1 MPa over 4 V = 250 kPa/V |

**Found by reviewing the design:**

| Bug | Effect |
|---|---|
| CPU fault handlers just looped | PWM kept running at the last command; a crash mid-manoeuvre left the thrusters on |
| Disarm didn't reset the ramp or PID | re-arming jumped to full thrust (§10) |
| Control took one command per tick | a Pi sending faster than 50 Hz left the vehicle up to 80 ms behind |
| Telemetry busy-waited on the UART | 27–54% of the CPU |
| Release configuration had only CubeMX's default include paths | it had never been able to build; the "vehicle build" didn't exist |
| Default build had bench hooks on and the watchdog off | the easiest build to flash was the unsafe one |
| Bar30 initialised once at boot | a sensor plugged in later was read with an all-zero calibration |

Earlier fixes from the bare-metal → RTOS move (static review): the PLL register
OR-ed onto its reset value (wrong clock), APB1 over its limit, `HSE_VALUE`
undefined, six pins claimed by two drivers, `pwm_set_us()` ignoring its channel
argument, unbounded I2C waits starving low-priority tasks when no sensor was
connected, ring-buffer variables shared with an interrupt but not `volatile`.

---

## 14. Building and testing

**STM32CubeIDE:** import as an existing project and build *Debug* (bench) or
*Release* (vehicle). Any new folder under `Devices/` must be added to the
project's source folders and include paths.

**Host tests** (PC, needs `gcc`):

```bash
sh tests/host/run.sh
```

**Bench test** (Nucleo on USB; optional jumpers: PA9→PA10 for the Pi-link
loopback, a PWM pin → PA15 for PWM capture; optional MPU-6050 on I2C1):

```bash
python tools/hil/hil_rtos.py --port COM12
```

It drives the console keys, measures timing with the DWT cycle counter, and
writes a pass/fail report. [`docs/HARDWARE_CHECKLIST.md`](docs/HARDWARE_CHECKLIST.md)
lists the checks that need a scope, an ESC or a sensor.

**Console** (115200 on the ST-LINK COM port): `s` stack marks, `h` health
(clock, heap, every drop counter, sensor states, packet counts, link and
failsafe state), `?` help.

---

## 15. Not done yet

- Bench-run everything marked `UNTESTED` (§1), then rerun `hil_rtos.py`.
- VN-200 and DVL on real hardware; confirm the DVL checksum variant.
- Check the BNO085's axis signs against the VN-200.
- Replace the OLED with the ILI9341 TFT; make the SD log start after the last
  run instead of overwriting it; add sensor readings to the log record.
- Update the Pi parser for packets `0x03` and `0x04`, and raise the baud rate.
- Tune the PID in the water.

---

## 16. Design decisions in one line each

- **Failsafe in the control task**: the check must not depend on the thing that failed.
- **Watchdog refreshed only by the control task**: if the loop that moves thrusters stops, the board resets.
- **Thrusters to neutral before anything else in a fault handler**: reading a bad stack can fault again and lock the core up.
- **Mailbox queues for sensors, FIFOs for commands and logs**: latest value vs every item.
- **Zero timeouts on the control path**: losing a log record is fine; missing a control tick is not.
- **DMA in both directions on the Pi link**: the CPU shouldn't wait on a UART.
- **Half/full-transfer interrupts as well as IDLE**: a position counter can't tell "nothing" from "a full lap".
- **Interrupts enabled from their own tasks**: no wake-up can happen before the scheduler exists.
- **SPI bus owner, I2C mutex**: SPI needs per-device clock speeds; the I2C devices don't, and their transfers are short.
- **Sensor health from valid packets, not bytes**: noise is not a sensor.
- **Switch back to the preferred sensor only after 1 s**: no flapping on a loose wire.
- **Measured zero accepted only if plausible**: a reboot underwater must not zero the depth.
- **Allocate everything before the scheduler**: heap exhaustion at run time becomes impossible, and the bench checks it.
- **Parsers and maths testable on a PC**: datasheet examples become unit tests.
