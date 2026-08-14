# TIBURON AUV — FreeRTOS Firmware

A FreeRTOS migration of the register-level bare-metal AUV control firmware ([Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal)), running on an STM32F446RE (Nucleo-64). Peripheral drivers are unchanged register-level C — no HAL, no CubeMX-generated init. The migration changed *scheduling and synchronization* around the same drivers, not the drivers themselves.

## Overview

Nine FreeRTOS tasks replace a single bare-metal superloop, handling closed-loop attitude/depth control, multi-sensor fusion, thruster PWM, telemetry, and onboard SD logging — with an explicit priority scheme protecting the 50Hz control loop from every lower-priority task, including the two (logging, display) that share a single SPI bus with no built-in locking.

- **MCU:** STM32F446RE @ 180MHz (HSE bypass + PLL, M=8 N=360 P=2)
- **RTOS:** FreeRTOS, heap_4, `configTOTAL_HEAP_SIZE` = 22KB
- **Toolchain:** arm-none-eabi-gcc (CubeIDE)
- **Flash footprint:** ~39KB (`-O2`; varies with optimization level — CubeIDE Debug builds will be larger)

## Task Architecture

| Task | Priority | Trigger | Role |
|---|---|---|---|
| Control | 7 | TIM7 ISR notify, 50Hz | PID, thrust allocation, PWM output, failsafe, command timeout |
| Filter | 6 | 10ms periodic | Complementary filter — fuses VN-200 + DVL + Bar30 into a state estimate |
| Comms | 5 | UART1 IDLE ISR notify | Parses incoming command packets, builds telemetry |
| VN-200 | 4 | USART3 DMA+IDLE | IMU driver — attitude, raw gyro/accel |
| DVL | 4 | UART4 DMA+IDLE | Doppler velocity log driver |
| Bar30 | 4 | 20ms periodic | Depth sensor (I2C) |
| Display | 3 | Queue receive, 500ms timeout | Sole owner of SPI1 (OLED + SD) — see below |
| Logging | 2 | Queue receive from Control | Builds SD-write requests, forwards to Display |
| Dummy | 1 | 500ms periodic | Heartbeat GPIO toggle; runs a one-shot stack-usage audit 5s after boot |

Priority order is deliberate: Control can never be blocked by anything below it; Filter must finish before Control needs a fresh estimate; sensor/comms drivers sit below both; Display/Logging are lowest among the "real" tasks since neither is timing-critical.

## SPI Arbitration: Bus-Owner, Not Mutex

Both the OLED and SD card sit on SPI1. Neither `oled.c` nor `sd_card.c` has any built-in locking — they're raw, polled, register-level drivers with no task awareness.

**Decision: only `display_task` is ever allowed to call SPI/OLED/SD functions.** Every other task that needs the bus (`logging_task`) sends a request through a queue instead. The textbook mutex benefit — priority inheritance protecting a high-priority task from a low-priority lock holder — doesn't apply here, since `control_task` never touches SPI at all. The actual risk is silent byte-level corruption between two *low-priority* tasks if they ever hit the peripheral at once. Bus-owner makes the exclusion structural — a second task simply cannot call `spi_transmit` — rather than relying on every future call site remembering to lock correctly.

Accepted tradeoff: `sd_write_block()` ends in a raw CPU busy-wait (up to 100,000 iterations, no yield) waiting for the SD card to clear its busy state — duration is card-dependent and not boundable from source. Since `display_task` owns the bus, OLED refreshes queue up behind SD writes when one is in flight. Control never touches SPI, so this doesn't threaten 50Hz timing directly, but it will visibly stutter the OLED during a log write — expected, not a bug.

## Logging Pipeline

`control_task` builds a `LogRecord` (timestamp, pose, PWM, armed/link state, CRC16) every 10th tick — 50Hz / 10 = 5Hz — and sends it non-blocking into `logQueue`. Control never blocks on logging; a full queue silently drops the record rather than risk a missed 50Hz tick. `logging_task` blocks on that queue, wraps each record as a bus request, and forwards it to `display_task` with a bounded 50ms send — dropped, not queued indefinitely, if the bus owner is mid SD-write. CRC16 is computed over the whole record except the CRC field itself, which is why that field is last in the struct.

## Hardware & Toolchain

| Item | Detail |
|---|---|
| Board | Nucleo-F446RE |
| Toolchain | arm-none-eabi-gcc (CubeIDE) |
| Clock | 180MHz via HSE bypass + PLL (M=8, N=360, P=2) |
| RTOS | FreeRTOS heap_4, 22KB heap |
| Max priorities | 8 (`configMAX_PRIORITIES`) |

## Peripheral Map

| Peripheral | Pins | Role |
|---|---|---|
| USART2 | PA2/PA3 | Debug printf via ST-Link VCP |
| USART1 | PA9/PA10 | Companion-computer packet protocol, DMA RX + IDLE ISR |
| USART3 | PB10/PB11 | VN-200 IMU, DMA1 Stream1, DMA + IDLE |
| UART4 | PC10/PC11 | Wayfinder DVL, DMA1 Stream2, DMA + IDLE |
| SPI1 | PA5/PA6/PA7 | SSD1306 OLED + SD card — owned exclusively by `display_task` |
| I2C1 | PB8/PB9 | Bar30 MS5837 depth sensor, AF4, open-drain, 100kHz |
| TIM7 | — | 50Hz basic timer ISR — control-loop notify, hardware priority 5 |
| TIM3 | — | PWM output for ESC control |
| Hardware CRC | — | CRC32 packet validation, CRC16 log-record validation |
| IWDG | — | Watchdog |

`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` — TIM7 sits at this ceiling deliberately, since it's the ISR making the `xTaskNotifyFromISR` call into the scheduler.

## Packet Protocol

| Field | Value |
|---|---|
| Header | 0xAA 0x55 |
| Length | 1 byte |
| MSG ID 0x01 | Telemetry: STM32 → companion computer |
| MSG ID 0x02 | Command: companion computer → STM32 |
| CRC | Hardware CRC32 unit (uint32_t) |

## Bare-Metal → RTOS: What Actually Changed

- SysTick, SVC, and PendSV handlers are owned by the FreeRTOS port — the old bare-metal handler bodies were deleted on remap to avoid duplicate-symbol link errors.
- `g_tick`, used for timing in the bare-metal version, is gone from timing-critical paths — `xTaskGetTickCount()` is the source of truth wherever timing matters (command timeout, log timestamps).
- Two flags from the bare-metal main loop (`telem_pending`, `log_pending`) were removed during the FreeRTOS logging rework — one was being written every tick inside the TIM7 ISR but never read anywhere. Decimated logging now lives entirely in a counter inside `control_task`.
- The concurrency model shifted from a single superloop (implicitly safe — only one thing running at a time) to priority-preemptive tasks, which is why SPI1 needed the explicit bus-owner arbitration above; in the bare-metal version this was never a hazard because nothing could preempt an in-progress SPI transfer.

## File Structure

```
Core/
├── Inc/    control_loop.h, comms_task.h, filter_task.h, bar30_task.h,
│           vn200_task.h, dvl_task.h, display_task.h, logging_task.h,
│           FreeRTOSConfig.h, main.h
└── Src/    main.c, control_loop.c, comms_task.c, filter_task.c, comp_filter.c,
            bar30_task.c, vn200_task.c, dvl_task.c, display_task.c,
            logging_task.c, stm32f4xx_it.c, system_init.c
Drivers/
├── gpio/  uart/ (uart.c, uart3.c, uart4.c, uart_packet.c)  i2c/  spi/
├── adc/  dac/  timer/  crc_hw/  iwdg/
Devices/
├── bar30/    depth sensor driver
├── oled/     SSD1306 OLED driver
├── vn200/    IMU parser
└── dvl/      Wayfinder DVL parser
Protocol/
├── packet.c/.h, struct.c/.h, ring_buffer.c/.h
└── sd_card/  sd_card.c/.h, sd_logger.c/.h
Middlewares/FreeRTOS/    kernel + GCC/ARM_CM4F port
```

## Building

Import into STM32CubeIDE as an existing project. Any new folder under `Devices/` or `Drivers/` must have build inclusion checked: right-click the folder in Project Explorer → Resource Configurations → Exclude from Build → leave **unchecked**, or the linker won't see it.

## Verification Status

Compile + link verified against the real linker script and target flags. **Not fully hardware-verified.** Specifically still needed on real hardware:
- SD writes confirmed not to disturb 50Hz control timing (scope/logic analyzer on the control task's timing signal during an active log write).
- Real stack high-water-mark numbers from the boot-time audit, followed by manual per-task stack trimming based on those numbers.
- Full power-cycle test with all nine tasks live, then a sustained-load pass checking for deadlock, starvation, priority inversion, queue overrun, missed notify, and jitter.
- Final clean build + flash + power-cycle sign-off.

Sensor-fusion tasks (VN-200, DVL, filter) are implemented and integrated but cannot be meaningfully exercised until the target hardware (custom STM32H723-based PCB, expected end of September) and its associated sensors/thrusters are available — the current Nucleo-F446RE dev setup has no IMU, DVL, or thrusters attached.

## Notes

- All flag-polling loops (I2C, SPI, UART) inherited from the bare-metal drivers are direct register polls; `sd_write_block()`'s busy-wait specifically has no bounded timeout independent of the SD card's own response — a future revision could add one to cap worst-case bus-owner latency.
- `logging_task` shares priority 2 with FreeRTOS's internal Timer Service task. No software timers (`xTimerCreate`) are used anywhere in this codebase today, so this is currently inert — worth re-checking if timers get added later.
