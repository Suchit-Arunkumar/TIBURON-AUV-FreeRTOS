# TIBURON-EMB-001 — FreeRTOS Port (AUV_Baremetal-Rtos)

STM32F446RE, register-level drivers (no HAL/CubeMX), SYSCLK 180MHz, FreeRTOS heap_4.

## Architecture — 9 tasks

| Task      | Priority | Period / trigger              | Role |
|-----------|----------|--------------------------------|------|
| Control   | 7        | TIM7 ISR notify, 50Hz          | PID, thrust allocation, PWM output, failsafe |
| Filter    | 6        | 10ms periodic                  | Complementary filter — fuses VN-200 + DVL + Bar30 into a state estimate |
| Comms     | 5        | UART1 IDLE ISR notify          | Parses incoming command packets, builds telemetry |
| VN-200    | 4        | UART DMA+IDLE                  | IMU driver — attitude, raw gyro/accel |
| DVL       | 4        | UART DMA+IDLE                  | Doppler velocity log driver |
| Bar30     | 4        | 20ms periodic                  | Depth sensor |
| Display   | 3        | queue-receive w/ 500ms timeout  | Sole owner of SPI1 (OLED + SD) — see below |
| Logging   | 2        | blocks on queue from Control    | Builds SD-write requests, forwards to Display |
| Dummy     | 1        | 500ms periodic                 | Heartbeat GPIO toggle; runs a one-shot stack audit 5s after boot |

Priority order is deliberate: Control can never be blocked by anything below it; Filter must finish before Control needs a fresh estimate; Comms/sensor drivers sit below both; Display/Logging are lowest among "real" tasks since neither is timing-critical.

Note: `logging_task` shares priority 2 with FreeRTOS's internal Timer Service task (`configTIMER_TASK_PRIORITY`). No software timers (`xTimerCreate`) are currently used anywhere in this codebase, so this is inert today — but if timers get added later, they'll round-robin with logging_task at that priority. Worth re-checking then.

## Interrupt priorities

TIM7 (50Hz control notify): hardware priority 5. `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` — this is the ceiling above which an ISR cannot safely call FreeRTOS `...FromISR()` APIs. TIM7 sits right at that ceiling deliberately, since it's the ISR making the notify call.

## RAM / heap budget

`configTOTAL_HEAP_SIZE = 22KB` (heap_4). Per-task stacks: 256 words (1KB) for all tasks except Dummy at 128 words (512B) — see `main.c` `xTaskCreate` calls for current allocations.

**Not yet trimmed.** P9 added `uxTaskGetStackHighWaterMark()` instrumentation (`print_stack_audit()` in `main.c`, runs once 5s after boot, prints over UART2). The printed numbers are the actual per-task headroom — use them to right-size the 256-word default down per task rather than trusting the default blindly. This step needs a real flash+boot cycle to produce numbers; it hasn't been run yet.

## SPI: bus-owner architecture (not mutex)

Both the OLED and SD card sit on SPI1. Neither `oled.c` nor `sd_card.c` has any built-in locking — they're raw, polled, register-level drivers with no awareness of tasks.

**Decision: bus-owner, not mutex.** `display_task` is the only task ever allowed to call SPI/OLED/SD functions. Every other task that needs the bus (`logging_task`) sends a request through `spiRequestQueue` instead.

**Why not a mutex:** the textbook mutex benefit — priority inheritance protecting a high-priority task from a low-priority lock holder — doesn't apply here, because `control_task` (the only task with real timing pressure) never touches SPI at all. The actual risk is silent byte-level corruption between two *low-priority* tasks (`logging_task` and `display_task`) if they ever both hit the peripheral at once. A mutex would work, but only if every future SPI call site remembers to lock — easy to forget across two unrelated driver files with no task-awareness built in. Bus-owner makes the exclusion structural: a second task simply doesn't have the ability to call `spi_transmit`.

**Accepted tradeoff:** `sd_write_block()` ends in a raw CPU busy-wait spin (up to 100,000 iterations, no yield) waiting for the SD card to clear its internal busy state — duration is card-dependent and not boundable from source. Because `display_task` owns the bus, OLED refreshes queue up behind SD writes when one is in flight. Since control never touches SPI, this doesn't threaten the 50Hz control loop directly — but it will visibly stutter the OLED display during a log write. That's expected, not a bug.

## Logging pipeline

`control_task` builds a `LogRecord` (timestamp, pose, PWM, armed/link state, CRC16) every 10th tick — 50Hz / 10 = 5Hz — and sends it non-blocking (`xQueueSend(..., 0)`) into `logQueue`. Control never blocks on logging; a full queue silently drops the record rather than risk a missed 50Hz tick.

`logging_task` blocks on `logQueue`, wraps each record as an `SpiRequest`, and forwards it to `spiRequestQueue` with a bounded 50ms send — if `display_task` is mid SD-write and doesn't drain in time, the record is dropped rather than backing the pipeline up indefinitely.

CRC16 is computed via the hardware CRC unit (`crc_compute`, truncated to 16 bits) over the whole `LogRecord` *except* the CRC field itself — which is why `crc16` is the last field in the struct and is zeroed before the compute call.

Removed during this pass: `telem_pending` / `log_pending` (`control_loop.c`, `stm32f4xx_it.c`) — these were declared and, in `log_pending`'s case, written every tick inside the TIM7 ISR, but never read anywhere. Decimation now lives entirely in `control_task`'s own counter; the ISR no longer does any logging-related work.

## Bare-metal vs RTOS — what actually changed

- SysTick, SVC, and PendSV handlers are owned by the FreeRTOS port, not the old bare-metal firmware — the old handler bodies were deleted on remap to avoid duplicate-symbol link errors.
- `g_tick`, used for timing in the bare-metal version, is gone from the control-timing-critical path — `xTaskGetTickCount()` is the source of truth wherever timing matters (command timeout, log timestamps).
- Peripheral drivers themselves are unchanged register-level C — no HAL, no CubeMX-generated init. The RTOS migration only changed *scheduling and synchronization* (tasks, queues, notifications) around the same drivers, not the drivers.
- Concurrency model shifted from a single superloop to priority-preemptive tasks — meaning shared state (e.g. SPI1) that was implicitly safe in a superloop (only one thing running at a time) needed explicit arbitration once multiple tasks could preempt each other. The SPI bus-owner decision above exists specifically because of this shift.

## Verification status

Compile + link verified (this pass, sandbox toolchain, ARM GCC against the actual linker script — not CubeIDE, but same target and flags). **Not HW-verified.** Needs, on real hardware:
- P8: SD writes confirmed not to disturb 50Hz control timing (scope/logic analyzer or timestamped UART on the control task's GPIO toggle during an active log write).
- P9: actual stack high-water-mark numbers from `print_stack_audit()`'s UART output, then manually trim `xTaskCreate` stack sizes based on real headroom.
- P10/P11: full power-cycle test with all 9 tasks running, then sustained-load pass checking for deadlock/starvation/priority-inversion/queue-overrun/missed-notify/stack-exhaustion/jitter.
- P14: final clean build, full flash, power-cycle, confirm no stack overflow / malloc-fail / deadlock across the whole system.

## Suggested CV bullet (draft — adjust scope claims once HW-verified)

"Migrated a bare-metal STM32F446RE AUV firmware to a 9-task FreeRTOS architecture; designed a bus-owner SPI arbitration scheme eliminating driver-level lock discipline as a failure mode, and a decimated non-blocking telemetry-logging pipeline preserving 50Hz control-loop timing guarantees." — *only claim "HW-validated" once P8/P9/P10/P11/P14 above are actually run.*
