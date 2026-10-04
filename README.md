# TIBURON AUV — FreeRTOS Firmware

Register-level FreeRTOS firmware for an 8-thruster AUV control node on an
STM32F446RE (NUCLEO-F446RE). No HAL and no CubeMX-generated init: every
peripheral driver is hand-written against the reference manual. It began as a
migration of [Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal)
that changed the *scheduling and synchronisation* around the same drivers, then
hardened them.

**Verified on a NUCLEO-F446RE**: 18 of 18 hardware-in-the-loop tests pass on
the final firmware, including the vehicle's real Pi-link path (USART1, DMA,
IDLE interrupt) in loopback. Measured: 1.87 µs worst control-tick jitter under
load, a failsafe that trips 520 ms after the last command even with the comms
task suspended, and 0 CRC errors over a 60 s soak. Bring-up found three
defects, and the bench reproduced a DMA data-loss bug that analysis had only
flagged as untested. Results are in
[Hardware-in-the-loop verification](#14-hardware-in-the-loop-verification) and
[Key figures](#15-key-figures).

**Scope.** The STM32 control path for Team Tiburon's AUV (SAUVC 2026),
developed and verified on a bench rig. The competition vehicle ran the team's
RP2350 firmware, whose 62-byte frame and CRC-16 this firmware speaks unchanged;
this stack is bench-verified and not yet integrated on the vehicle. The PID
gains are untuned (all zero) until the vehicle is in water.

---

## Contents

1. [The failsafe is the architecture](#1-the-failsafe-is-the-architecture)
2. [At a glance](#2-at-a-glance)
3. [Tasks and priorities](#3-tasks-and-priorities)
4. [SPI arbitration: bus owner, not mutex](#4-spi-arbitration-bus-owner-not-mutex)
5. [Logging: batching, and what it costs](#5-logging-batching-and-what-it-costs)
6. [Queue design](#6-queue-design)
7. [Interrupts](#7-interrupts)
8. [Memory](#8-memory)
9. [Pin map](#9-pin-map)
10. [Clock](#10-clock)
11. [Two decisions worth the words](#11-two-decisions-worth-the-words)
12. [Defects found](#12-defects-found)
13. [Bare-metal → RTOS: what changed](#13-bare-metal--rtos-what-changed)
14. [Hardware-in-the-loop verification](#14-hardware-in-the-loop-verification)
15. [Key figures](#15-key-figures)
16. [What is not verified](#16-what-is-not-verified)
17. [Building](#17-building)
18. [Console](#18-console)

---

## 1. The failsafe is the architecture

If a comms link to a submerged vehicle drops, the thrusters must stop. The obvious implementation puts that check in the task that handles comms, and that is exactly where it must not be: the failure that kills the link and the failure that hangs the comms task overlap a great deal.

**The 500 ms command timeout is evaluated inside `control_task`, on every 20 ms TIM7 tick, and depends on nothing else in the system.**

```
TIM7 (hardware, 50 Hz)  ──notify──▶  control_task ──▶ checkCommandTimeout()
                                          │                    │
                                          │              reads last_cmd_tick
                                          ▼                    ▼
                                    all 8 thrusters ◀── enterFailsafe()
```

It reads one timestamp. It does not read a queue, does not wait on a notification, and does not call into `comms_task`. So the failsafe still fires on schedule when:

- `comms_task` is hung, starved, crashed, or was never created
- the USART1 ISR has stopped firing entirely
- `commandQueue` is full and every command is being dropped
- the Pi is powered but sending garbage that fails CRC

**Measured on hardware:** the failsafe tripped 520 ms after the last valid command in every run: 3 / 3 with the link simply going quiet, once on a stream of bad-CRC frames, and once **with `comms_task` suspended**. 520 ms is the 500 ms timeout detected on the next 20 ms tick, exactly as designed.

The single point of failure is TIM7 itself. If TIM7 stops, `control_task` stops refreshing the watchdog, and the board resets within ~1 s, which drives the ESCs to no-signal. The failure mode of the failsafe is therefore *also* a failsafe. **Measured:** a deliberately hung `control_task` reset the board 3 / 3 times, in 1,055–1,126 ms, with `RCC_CSR` reporting `IWDG` on the next boot.

**Recovery is deliberately asymmetric.** Going into failsafe takes one missed deadline. Coming out takes **three consecutive CRC-valid packets, none more than 500 ms after the one before**, because one packet after a dropout is not evidence of a restored link; it may be the only one that got through. `enterFailsafe()` also zeroes the setpoint and the integrator state, so a stale packet arriving after a long dropout **cannot re-apply the throttle the vehicle was carrying when the link died**. Recovery starts from a stationary command, and the slew limiter walks the outputs up from neutral at 2500 µs/s.

The "consecutive" part was not true until the bench test was written: the counter accepted three packets at any spacing, so two packets, ten seconds of silence and one more would re-arm. `checkCommandTimeout()` now resets the streak on a gap longer than the timeout. **Measured:** packets 1, 2, 3 re-arm; packets 1, 2, a 1.2 s gap, then 3 leave the vehicle in failsafe with the streak back at 1.

---

## 2. At a glance

| | |
|---|---|
| **MCU** | STM32F446RE @ 180 MHz: HSE bypass (no crystal fitted), PLL M=8 N=360 P=2, over-drive enabled |
| **RTOS** | FreeRTOS V11.1.0+, `heap_4`, GCC `ARM_CM4F` port, hard-float `fpv4-sp-d16` |
| **Tasks** | 8 application tasks over 6 priority levels; 9 tasks over 7 levels at runtime with the idle task |
| **Code** | 5,278 lines of C (cloc; 2,943 more of comments), kernel and CMSIS excluded |
| **Flash** | 34,916 B, 6.7 % of 512 KiB (vehicle build) |
| **RAM** | 29,736 B static, 22.7 % of 128 KiB (vehicle build) |
| **Heap free** | 9,448 B of 22,528 B: measured, and equal to the minimum-ever-free |
| **Link** | 62-byte frames, CRC-16/IBM-3740, USART1 115,200 baud, DMA RX with IDLE and half/full-transfer draining |
| **Verified** | 18 / 18 HIL tests on a NUCLEO-F446RE ([§14](#14-hardware-in-the-loop-verification)) |
| **Toolchain** | `arm-none-eabi-gcc` at `-O2`, 0 warnings in both builds |

---

## 3. Tasks and priorities

**8 application tasks across 6 priority levels.** Stated separately because it is a real distinction: FreeRTOS adds an idle task the application never mentions, so the *runtime* figure is **9 tasks across 7 levels**.

| Prio | Task | Stack | Trigger | Role | Measured stack use | CPU (loaded) |
|---|---|---|---|---|---|---|
| 7 | `control_task` | 1024 B | TIM7 notify, 50 Hz | PID, allocation, PWM, **failsafe**, IWDG refresh | 464 B | 0.3 % |
| 5 | `comms_task` | 1024 B | USART1 IDLE / DMA notify | CRC-16 packet parse, telemetry build and send | 768 B | 54.4 % |
| 4 | `vn200_task` | 1024 B | USART3 DMA + IDLE | IMU parser | 664 B | 0 % |
| 4 | `dvl_task` | 1024 B | UART4 DMA + IDLE | Doppler velocity log parser | 760 B | 0 % |
| 4 | `bar30_task` | 1024 B | 20 ms periodic, backs off when absent | Depth sensor (I²C); owns I2C1 | 736 B | 0 % |
| 3 | `spi_owner_task` | 1024 B | Queue, 500 ms timeout | **Sole owner of SPI2**: OLED + SD | 576 B | 0.3 % |
| 2 | `logging_task` | 1024 B | Queue from control | Batches records into 512 B blocks | 252 B | 0 % |
| 1 | `dummy_task` | 1536 B | Queue, 50 ms timeout | Heartbeat, **sole stdio owner**, on-demand reports | 928 B | 0.1 % |
| 0 | `IDLE` | 512 B | — | Kernel | — | 44.6 % |

Stack use is the high-water mark after every error path the bench can reach had run (failsafe, junk, bad CRC, sensor absence, reports). CPU is from FreeRTOS run-time stats over a 60 s soak with commands in and telemetry out at 50 Hz. Both are from the bench build, which runs extra code in `comms_task`, `bar30_task` and `dummy_task`.

**Where the CPU goes.** `comms_task` takes 27.0 % of the CPU with the link idle and 54.4 % under load, almost all of it busy-waiting on `TXE` while it sends 62-byte frames at 115,200 baud (5.4 ms each, one or two per tick). It is preemptible and sits below `control_task`, so it costs no deadline, but it is the obvious next optimisation: a DMA transmit path would return that time to idle.

`configUSE_TIMERS` is **0**. `xTimerCreate` is called zero times, so the timer-service daemon was pure cost: 816 B of heap and a pinned priority 2 that collided with `logging_task`. Turning it off freed both.

Three tasks share priority 4 with `configUSE_TIME_SLICING 1`, so they round-robin on each 1 ms tick. None can monopolise; each may wait up to 2 ms for its turn, against budgets of ≥10 ms.

> **What "implemented" means for the controller.** The control loop's
> *structure* is complete and runs end to end: error, clamped integral,
> derivative, feedforward, per-axis saturation, and thrust allocation
> through a pre-computed pseudo-inverse. Pose comes from the Pi's fused
> navigation state in each command packet, as on the competition Pico
> firmware. **The PID gains are all zero**, so the loop currently computes
> an identically zero command; tuning needs the vehicle in water. The
> whole task body runs in 17 µs on average at idle and 57 µs under load
> (109 µs worst), measured.

**LD2 is a liveness indicator at the lowest priority.** That is a deliberate trade: the LED proves the *whole* schedule is running, not just the top of it. The consequence: **a starved system and a crashed system look identical from the LED alone.** If the heartbeat stops, the console (`h`) and the fault latch are what distinguish them.

---

## 4. SPI arbitration: bus owner, not mutex

> The timing figures in this section are **computed** from the clock tree and the SD specification. The OLED and SD card were not fitted on the bench, so none of them has been measured.

Both the SSD1306 OLED and the SD card sit on SPI2. Neither driver has any locking. The choice was a mutex or a single owning task, and it was made by arithmetic.

### The numbers

SPI2 is on APB1, so `BR[2:0]` divides **PCLK1 = 45 MHz**:

| BR | ÷ | f_SCK | Used for |
|---|---|---|---|
| 001 | 4 | 11.250 MHz | — (available; see below) |
| 010 | 8 | **5.625 MHz** | **OLED** and **SD data** |
| 110 | 128 | **351.563 kHz** | **SD identification** |

Two divisors land in the SD spec's mandatory 100–400 kHz init window — ÷128 (351.6 kHz) and ÷256 (175.8 kHz). ÷128 is chosen for being faster, **not** for being unique. The SSD1306 tops out near 10 MHz (100 ns min cycle), so ÷4 is out for the OLED. SD data starts at ÷8 rather than ÷4 because 11.25 MHz is optimistic over Nucleo jumper wiring with no controlled impedance — `SPI_BR_SD_DATA` is one named constant to raise once the card reads reliably.

Per-byte cost is 8/f_SCK plus **≈0.40 µs** of polled driver overhead, derived from `spi_transmit`'s four APB1 register accesses at ~0.11 µs each across the HCLK-180/PCLK1-45 bridge. *That 0.40 µs is computed, not measured, and it propagates into every figure below.*

| | Bus time | Busy | Total | vs 20 ms budget |
|---|---|---|---|---|
| OLED full frame (1030 B) | 1.88 ms | — | **1.88 ms** | 9.4 % |
| SD block write, typical | 0.97 ms | 2.5 ms | **3.5 ms** | 17 % |
| SD block write, **worst case** | 0.97 ms | **250 ms** | **251 ms** | **12.5 control periods** |

250 ms is the SD Physical Layer Simplified Specification's host timeout for a single-block write (SDHC/SDXC; for SDSC it is derived from TAAC/NSAC/R2W_FACTOR in the CSD and capped at the same figure).

### The decision

**Bus owner.** But not for the reason the arithmetic first suggests — 1.88 ms vs 251 ms is a latency argument, and *both* schemes serialise on the same single bus, so both suffer it identically.

What actually decides it: **`control_task` never touches SPI.** Priority inversion against the 50 Hz deadline is therefore impossible under either scheme, which removes the mutex's only real advantage — priority inheritance. What remains is that the bus needs three different SCK rates. Under a mutex, every acquirer becomes responsible for programming `CR1.BR` correctly on every acquisition, and a miss fails **silently and intermittently** — an SSD1306 driven at 11.25 MHz will often work, then not, depending on wiring. As a bus owner, BR is one task's private state, set from the request type it just dequeued. Exclusion becomes structural: no other task has a code path that reaches `spi_transmit`.

### What would reverse this decision

Kept verbatim, because these are the conditions under which the above stops being true:

1. **If any task at priority ≥ 5 needed the bus directly.** This is the decisive one and it inverts the whole argument. A priority-3 owner serving a priority-7 requester through a queue is **unbounded priority inversion with no inheritance whatsoever** — the owner can be preempted by everything at 4, 5 and 6 while holding a 250 ms transaction. A mutex with inheritance would be strictly better. *If the control loop ever needs a synchronous SD config-block read, switch to a mutex.*
2. **If worst-case single-operation time dropped below ~one control period (20 ms) AND more than one task needed the bus.** At ~5 ms held by a priority-2 task with inheritance, the dedicated task's 1 KB stack + 84 B TCB + queue hop stops paying for itself.
3. **If BR never had to change** — one device, or two sharing a rate. That single fact is carrying most of the bus-owner case; remove it and the two schemes are near-equivalent, and the mutex is cheaper.

### Why priority 3 specifically

The owner must sit **strictly above every task that sends it requests** (`logging_task` at 2), so a requester cannot preempt the owner and pile up more work while a transaction is in flight. It must sit **strictly below every task with a deadline** (control 7, comms 5, sensors 4), so a 250 ms program cycle is preemptible by all of them. Priority 3 is the unique slot satisfying both.

The OLED is a request **type** (`SPI_REQ_OLED_FRAME`), not the task's identity. Adding a third device to the bus is a new enum value and a `switch` case, not a restructure.

---

## 5. Logging: batching, and what it costs

> Computed, like §4: the logging pipeline ran on the bench, but with no SD card fitted every block write failed and was counted.

`control_task` builds a `LogRecord` (40 B) every 10th tick — 5 Hz — and posts it with **zero block time**, counting drops. It never waits on the logging pipeline: a full queue means the bus owner is mid-program-cycle, which can legitimately last 250 ms, and blocking would miss twelve deadlines.

Writing each 40 B record as its own 512 B block meant **12.8× write amplification** and one full program/erase cycle per record. Records now accumulate into a 512 B staging block — a 16 B self-describing header plus 12 records — emitted when full (~2.4 s) or on an explicit flush.

| | Before | After |
|---|---|---|
| Bytes programmed | 2560 B/s | **213 B/s** (12×) |
| Write amplification | 12.8× | **1.067×** |
| Bus duty, typical | 1.9 % | **0.53 %** |
| Bus duty, all-worst-case | 1250 ms/s — **saturated** | **105 ms/s = 10.5 %** |
| Exposures to the 250 ms worst case | 5/s | **0.42/s** |

> **The cost, stated plainly: batching loses up to 2.4 s of records on an unexpected power cut** — and that is the most interesting 2.4 seconds of any run. Flush-on-disarm and flush-on-failsafe cover the graceful cases, and both are triggered by the armed→disarmed edge inside `control_task`. Nothing covers a hard power loss. If that matters more than SD endurance, reduce `LOG_RECORDS_PER_BLOCK`.

`SpiRequest` carries the whole 512 B block **by value**. That costs 1120 B of heap for a depth-2 queue and one ~3 µs memcpy per block. The alternative — passing a pointer into a shared staging buffer — saves ~500 B against 8704 free and buys a buffer-ownership handoff between two tasks, which is the exact bug class that corrupts log blocks under load. The bytes are the cheaper thing to spend.

---

## 6. Queue design

| Queue | Depth | Item | Heap | Full policy | Counter | Measured drops |
|---|---|---|---|---|---|---|
| `commandQueue` | 4 | 56 B | 304 B | 1 ms block, then drop | `comms_cmd_drops()` | 0 in a 60 s soak at 50 Hz |
| `dvlQueue` | 1 | 76 B | 160 B | `xQueueOverwrite`: newest wins | — | — |
| `vn200Queue` | 1 | 40 B | 120 B | `xQueueOverwrite` | — | — |
| `bar30Queue` | 1 | 4 B | 88 B | `xQueueOverwrite` | — | — |
| `logQueue` | 8 | 41 B | 408 B | **zero block**, drop | `control_log_drops()` | — |
| `spiRequestQueue` | 2 | 520 B | 1120 B | 500 ms block, then drop | `logging_spi_post_drops()` | — |
| `consoleQueue` | 8 | 80 B | 720 B | **zero block**, drop | `console_dropped()` | 0 over a full HIL run |

Depth-1 queues use `xQueueOverwrite` because the consumer only ever wants the latest measurement, never a backlog. Absorption: `logQueue` holds 1.6 s of records; `spiRequestQueue` holds 4.8 s of block production.

**Every drop site has a counter**, including the UART receive ring buffer (`rx_dropped_count()`), whose silent overflow used to look exactly like a CRC failure. Measured: 0 ring-buffer drops over 118,978 bytes received.

**The stdio owner writes its own lines directly.** `dummy_task` is the only reader of `consoleQueue`, so lines it posted there itself could not be printed until it returned to its loop, and the 13-line `h` and `s` reports lost their last 5 lines to the depth-8 queue. Measured after the fix: all 13 lines, 0 console drops.

---

## 7. Interrupts

`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`, and with `configPRIO_BITS = 4` that makes `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 4 = 0x50`.

The rule: **an ISR may call a `...FromISR` API only if its raw IPR byte is numerically ≥ 0x50.** Lower numbers are higher priority and are *not* masked by `BASEPRI = 0x50`, so they could preempt a kernel critical section.

| IRQ | `NVIC_SetPriority` | Raw IPR | Calls kernel API? | Legal |
|---|---|---|---|---|
| TIM7 | 5 | 0x50 | `vTaskNotifyGiveFromISR` | Yes, **at the ceiling** |
| USART1 | 5 | 0x50 | `xTaskNotifyFromISR` | Yes, at the ceiling |
| DMA2 Stream2 | 5 | 0x50 | `xTaskNotifyFromISR` | Yes, at the ceiling; equal to USART1 on purpose |
| USART2 | 6 | 0x60 | none (one volatile store) | Yes |
| USART3 | 6 | 0x60 | `xTaskNotifyFromISR` | Yes |
| UART4 | 6 | 0x60 | `xTaskNotifyFromISR` | Yes |
| TIM2 CC1 (bench build) | 6 | 0x60 | none | Yes |
| SysTick / PendSV | — | 0xF0 | kernel-owned | Yes |

TIM7, USART1 and DMA2 Stream2 sit *exactly* at the ceiling: legal, with zero margin. Dropping `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` to 4 would break all three, and `configASSERT` would catch it.

`SCB->AIRCR` PRIGROUP is set to 3 (4 bits preemption, 0 subpriority) as **init step 2**, before any `NVIC_SetPriority` call.

**Measured latency.** From the TIM7 interrupt to `control_task` running: 6.2–7.2 µs over every run, idle and loaded. The tick itself: worst 0.80 µs from the 20 ms nominal at idle and 1.31–1.87 µs under full UART load across runs, measured ISR entry to ISR entry with the DWT cycle counter.

### USART1 receive: IDLE plus half/full-transfer

The DMA position is `SIZE − NDTR`, which says where the DMA is, not how far it has gone. If exactly 256 bytes arrive between two drains, the position comes back to where it was, and a whole buffer is indistinguishable from no data at all. With the IDLE interrupt as the only drain trigger, that happens whenever a burst is exactly 256 bytes with no gap.

The DMA2 Stream2 half-transfer and transfer-complete interrupts now drain too. They fire 128 bytes apart, so the DMA can never lap between two drains; IDLE still catches the tail of a burst. **Reproduced and fixed on hardware:** a 256-byte burst through the PA9 → PA10 loopback delivered **0 of 256 bytes** with IDLE-only draining, silently, and **256 of 256** with the fix. Over a full run, 118,978 bytes were drained by 929 half/full-transfer events and 1,919 IDLE events.

### No kernel API is reachable from any ISR before the scheduler runs

`NVIC_EnableIRQ` for every interrupt-driven peripheral is called from the *consuming task's* first iteration, never from `main`:

| Peripheral | Enabled by | In |
|---|---|---|
| TIM7 | `tim7_init()` | `control_task` |
| USART1, DMA2 Stream2 | `uart1_irq_enable()` | `comms_task` |
| USART3 | `uart3_irq_enable()` | `vn200_task` |
| UART4 | `uart4_irq_enable()` | `dvl_task` |

Between the last `xTaskCreate` and `vTaskStartScheduler()`, `pxCurrentTCB` is populated but **PSP is still zero**: `vPortSVCHandler` sets it, and that runs inside `vPortStartFirstTask`. An ISR reaching `portYIELD_FROM_ISR` in that window pends PendSV, whose context save does `mrs r0, PSP` then `stmdb r0!, {...}`: a write through a null stack pointer.

USART2 is the sole exception, enabled in `main`. Its handler's entire reachable body is one volatile byte store, with no FreeRTOS API and no yield. From `vTaskStartScheduler()` onward the kernel closes the window itself: `portDISABLE_INTERRUPTS()` sets `BASEPRI = 0x50` before `xPortStartScheduler()`, masking every interrupt in this design.

---

## 8. Memory

### Static

| Build | text | data | bss | Flash (text + data) | RAM (data + bss) |
|---|---|---|---|---|---|
| Vehicle (`-DBENCH_HIL=0`) | 34,800 | 116 | 29,620 | 34,916 B (6.7 % of 512 KiB) | 29,736 B (22.7 % of 128 KiB) |
| Bench (default) | 43,428 | 120 | 30,416 | 43,548 B (8.3 %) | 30,536 B (23.3 %) |

`bss` includes `ucHeap` (22,528 B), the OLED framebuffer (1,024 B), the log staging block (512 B), four 256 B UART DMA/ring buffers, and `._user_heap_stack` (1,536 B).

### Heap: a prediction that could have failed

**Every allocation happens before `vTaskStartScheduler()`**, and every `xTaskCreate` / `xQueueCreate` return value is checked; failure latches the name and halts before the scheduler starts. No task calls `pvPortMalloc` at runtime. That makes a falsifiable prediction: `xPortGetMinimumEverFreeHeapSize()` must **equal** `xPortGetFreeHeapSize()` for as long as the board runs.

**Measured:** 9,448 B free and 9,448 B minimum-ever, read at boot and again after a full HIL run (failsafe, fault paths, a 60 s soak). Equal. (9,960 B before `dummy_task` grew by 512 B; also equal.)

| | Bytes |
|---|---|
| `ucHeap` declared | 22,528 |
| Lost to 8-byte alignment of `ucHeap` (linked at `0x…BC`) and the `xEnd` marker | 12 |
| Queues: 7 × (72 B `Queue_t` + storage), each + 8 B `heap_4` header, 8-aligned | 2,920 |
| Task stacks + TCBs: 7 × (1,032 + 96) + (1,544 + 96) | 9,536 |
| IDLE stack + TCB | 616 |
| **Free, predicted from the map** | **9,444** |
| **Free, measured** | **9,448** (41.9 %), 4 B from the prediction |

### Stacks: predicted vs measured

High-water marks after every error path the bench can reach had run, bench build:

| Task | Alloc | Static prediction (vehicle) | Measured (bench) | Free |
|---|---|---|---|---|
| `control_task` | 1024 | 548 | 464 | 560 |
| `comms_task` | 1024 | 492 | 768 | 256 |
| `vn200_task` | 1024 | 556 | 664 | 360 |
| `dvl_task` | 1024 | 668 | 760 | 264 |
| `bar30_task` | 1024 | 476 | 736 | 288 |
| `spi_owner_task` | 1024 | 436 | 576 | 448 |
| `logging_task` | 1024 | 348 | 252 | 772 |
| `dummy_task` | **1536** | 692 | 928 | 608 |

Two findings. **`dummy_task` was at 928 of its original 1,024 B** (96 B left) once the bench reports ran in it; it was grown to 1,536 B. And **`vn200_task` and `dvl_task` came in about 100 B over their static predictions** even though no bench code runs in them; their only deep path is `console_printf` on the sensor-absent branch. That puts the real newlib-nano `vsnprintf` frame near 220 B, not the ~120 B the analysis estimated and flagged as its weakest number. No stack was ever trimmed on static numbers, which is why none overflowed. `comms_task`, `bar30_task` and `dummy_task` carry bench code (the injector, the MPU-6050 stress test, the reports), so their vehicle-build use is lower than measured here.

### MSP budget

Tasks run on PSP and handler bodies on MSP. A first exception from thread mode pushes its hardware frame to PSP; **only nested exceptions consume MSP.** Same-priority peers cannot nest, so the chain is at most 3 deep, budgeted at **360 B** of the 1,024 B reserved.

**Measured** with a painted MSP under full UART and DMA load: **176–248 B** peak across runs, inside the prediction.

---

## 9. Pin map

| Function | Pins | Mode |
|---|---|---|
| SWD | PA13 / PA14 | AF0 — untouched |
| Heartbeat LD2 | PA5 | GPIO out |
| Debug VCP | PA2 / PA3 | USART2 AF7 |
| Pi link | PA9 / PA10 | USART1 AF7, DMA2 Stream2 + IDLE |
| VN-200 | PC10 / PC11 | USART3 AF7, DMA1 Stream1 + IDLE |
| DVL | PA0 / PA1 | UART4 AF8, DMA1 Stream2 + IDLE |
| Bar30 (MS5837) | PB8 / PB9 | I²C1 AF4, open-drain, 100 kHz |
| SPI bus | PB13 / PB14 / PB15 | SPI2 AF5 |
| SD_CS | PC4 | GPIO out, idle high |
| OLED_CS | PC5 | GPIO out, idle high |
| OLED_DC | PC0 | GPIO out |
| OLED_RES | PC1 | GPIO out |
| PWM 1–4 | PB4 / PB5 / PB0 / PB1 | TIM3 CH1–4, AF2 |
| PWM 5–8 | PC6 / PC7 / PC8 / PC9 | TIM8 CH1–4, AF3 |
| Reserved | PA4 | future ADC depth — left unconfigured |
| Bench: PWM capture | PA15 | TIM2_CH1 AF1, pull-down (BENCH_HIL build only) |

The Pi connects **GPIO-to-GPIO on USART1**, not over USB, so USART2/VCP stays free for the console.

Three details that are silent failures if missed:

- **PB4 is NJTRST** and boots in an alternate-function state. Its MODER *and* AFR bits are cleared explicitly before TIM3_CH1 is configured — a bare OR leaves thruster 1 dead with nothing in the code looking wrong.
- **TIM8 is an advanced-control timer**: `BDTR.MOE` must be set or CH1–4 stay electrically disconnected regardless of `CCER`. TIM3 has no equivalent bit.
- **The two prescalers differ on purpose.** TIM3 is on APB1 (90 MHz timer clock) → `PSC=89`; TIM8 is on APB2 (180 MHz) → `PSC=179`. Both `ARR=19999` for 1 µs resolution and a 20 ms frame. Equal prescalers would run thrusters 5–8 at twice the frame rate.

**Measured:** with channel *k* driven at 1,100 + 100*k* µs and a jumper moved from PA15 (TIM2_CH1 input capture) across the outputs, every measured pin carried exactly its own channel's width and a 20,000 µs period, over 62–65 pulses each ([run `d53de2b`](tools/hil/reports/2026-10-05_d53de2b.md); see §14 for PC8).

All eight CCRs are written to 1500 µs **before** any output stage is enabled, then `EGR.UG` forces the shadow registers — so no ESC ever sees a frame built from the CCR reset value of 0.

---

## 10. Clock

180 MHz needs the over-drive sequence, in this order, and it is not optional:

```
1. RCC->APB1ENR |= PWREN
2. PWR->CR VOS = Scale 1
3. HSE bypass (no crystal fitted) → PLL M=8 N=360 P=2 Q=7 → wait PLLRDY
4. PWR->CR |= ODEN    → wait PWR->CSR ODRDY
5. PWR->CR |= ODSWEN  → wait PWR->CSR ODSWRDY
6. FLASH->ACR = 5 wait states + prefetch + I-cache + D-cache
7. AHB ÷1, APB1 ÷4 (45 MHz), APB2 ÷2 (90 MHz)
8. RCC->CFGR SW = PLL → wait SWS
```

`PLLCFGR` and `CFGR` are each written **once, fully composed** (`0x27405A08` and `0x9402`), never OR-ed onto their reset values — `PLLCFGR` resets to `0x24003010` with PLLM=16 and PLLN=192, so `|=` produces a silently wrong clock rather than a compile error.

Every hardware wait is bounded. Three outcomes:

| Outcome | Behaviour | Indication |
|---|---|---|
| HSE locks | 180 MHz, crystal-grade | steady 1 Hz LD2, `CLK: OK` |
| HSE dead | PLL retuned to **M=16 off the HSI — still 180 MHz** | double-pulse LD2, `CLK: DEGRADED` |
| PLL / over-drive / switch fails | raw HSI 16 MHz, **scheduler never starts** | LD2 blinks the status code (2/3/4) |

The HSI fallback holding 180 MHz is what keeps every BRR/CCR/PSC valid — only accuracy degrades, and HSI's ±1 % can corrupt 115200 framing, which is why the blink pattern exists at all. The fatal path halts *before* `vTaskStartScheduler()`, and **that is the invariant that lets `configCPU_CLOCK_HZ` be a literal `180000000UL`** instead of the `SystemCoreClock` variable.

**Measured.** Over 10.14 s the DWT cycle counter (HCLK), TIM2 (APB1 timer clock / 90) and SysTick (HCLK / 180,000) agreed exactly: 10,140,000 µs of TIM2 over 10,140 ticks, and 180.0000 DWT cycles per TIM2 microsecond. Against the laptop's clock the board was within 0.16 %, the limit of USB timestamp jitter. Every boot reported `CLK: OK (HSE bypass, 180 MHz)`.

---

## 11. Two decisions worth the words

### Bounded vs unbounded polling — the distinction is who controls completion

`i2c.c`'s waits are bounded and yielding. `spi.c`'s and the UART TX paths' are not, and that is deliberate rather than an oversight.

**I²C completion is controlled by an external agent.** A slave can stretch SCL indefinitely, or simply never ACK. `while (!(SR1 & flag));` therefore has no bound in the physical sense, not merely in the code. With no Bar30 attached, `bar30_task` spun forever at priority 4 — control and comms kept running so the board *looked* alive, while logging, the SPI owner and the console were starved permanently.

**SPI and UART TX completion is master-generated.** The STM32 produces SCK; the UART shifts at its configured baud regardless of any receiver. `TXE`, `RXNE` and `BSY` transitions depend only on internal clocking, so those loops are bounded by hardware even though they are not bounded by code. Adding timeouts there would be ceremony.

The I²C wait is **spin-then-yield**: 250 µs of spinning covers a healthy 90 µs phase with margin so a working bus never pays a context switch, then 1 ms yields up to a 2.5 ms budget, with backoff on consecutive failures.

**The hardware run found the flaw in it.** `bar30_task` (priority 4) is preempted for 5.4 ms at a time by `comms_task`'s blocking telemetry send, longer than the whole 2.5 ms budget. It woke past the deadline, left the loop without polling, and reported a timeout on a phase the peripheral had finished long before; the abort then sent STOP mid-transfer and latched the I²C peripheral's BUSY flag with both lines idle high. Result: 99 % of MPU-6050 transfers failed, with **0 wrong values** among those that completed. The wait now polls once more before giving up (clock stretching holds the bus while the task is away, so a late poll is always safe), and every error path resets the peripheral if BUSY is still latched. **Measured after the fix:** 4,000 reads with the RM0390 receive sequence, 0 errors, while preempted mid-transfer; individual 14-byte reads stretched to 12.6 ms and still returned correct data.

### No floating-point conversions in the console

`--specs=nano.specs` without `-u _printf_float` means `%f` **prints nothing and takes the rest of the line with it** — no error, no warning, no wrong number. Confirmed absent from the ELF. The decision is fixed-point milli-units via `console_fmt_milli()`, not the ~6 KB of `_printf_float`, and the reason beyond the flash is this:

> **Varargs promote `float` to `double` unconditionally — that promotion is mandated by the language, so `-Wdouble-promotion` cannot flag it.** Every `%f` call site would quietly reintroduce soft-float double conversion into a codebase that is otherwise strictly single-precision on a single-precision FPU.

Float formatting also costs ~100 B of stack per call, on the *caller's* stack, which is the least-characterised part of the budget. `console_fmt_milli` takes a caller-supplied buffer on purpose: a shared one would make `console_printf("d=%s u=%s", fmt(a), fmt(b))` print one value twice, since argument evaluation order is unspecified — and that failure reads as a sensor fault, not a formatting fault.

---

## 12. Defects found

### Found on hardware

Found by the HIL runs on 2026-10-05, each in code that compiled cleanly and had passed static review.

| Defect | How it presented | Fix |
|---|---|---|
| I²C wait gave up without a final poll after being preempted past its deadline | 99 % of MPU-6050 transfers failed under load, 0 wrong values among the rest | Poll once more before a timeout. `02ee61a` |
| An aborted I²C transfer could latch BUSY with SCL = SDA = 1 | Every later START timed out until reset (`SR2 = 0x0002`) | `i2c_fail()`: STOP, wait, reset the peripheral if BUSY persists. `02ee61a` |
| `dummy_task` stack at 928 of 1,024 B | Stack test failed: 96 B free after the error paths ran | Grown to 1,536 B. `b429abe` |
| `i2c_write()` set STOP before BTF, dropping the last byte of multi-byte writes | Found on the bare-metal tree's hardware run (MPU-6050 never woke); fixed here before it could bite | Wait for BTF. `21c8ad9` |

### Found while designing the bench test, then confirmed on hardware

| Defect | Effect | Confirmed on hardware | Fix |
|---|---|---|---|
| Failsafe recovery counted 3 valid packets at any spacing | 2 packets, a long silence, 1 more re-armed the vehicle | Gap test: streak resets after 1.2 s | `7f7b9e9` |
| IDLE-only DMA draining | An exactly 256-byte burst vanished silently | 0 / 256 before, 256 / 256 after | `7f7b9e9` |
| `consoleQueue` depth 8, posted to by its only reader | 13-line reports printed 8 lines | 13 lines, 0 drops | `7f7b9e9` |
| One frame parsed per RX notification | A second frame in a burst waited a tick or more | 506 / 506 commands applied within one tick | `7f7b9e9` |
| UART ring-buffer overflow not counted | Data loss looked like CRC noise | 0 drops counted over 118,978 bytes | `7f7b9e9` |

### Found by static analysis


Found by reading the map file, the `-fstack-usage` output, or the
disassembly, before the board was available. Several were later confirmed
by the hardware run (the clock, the fault latch, the pre-scheduler ISR
window); none of the fixes needed revisiting.

| Defect | How it presented | How it was found |
|---|---|---|
| `PLLCFGR` written with `\|=` onto its reset values (PLLM=16, PLLN=192) | PLL configured out of spec — VCO input 0.33 MHz against a 0.95 MHz floor, so the output is undefined and lands near half the intended clock | Reading the reference manual's reset values against the code |
| `RCC->CFGR` assigned twice; the second wiped `PPRE1` | APB1 left at HCLK, 36 MHz over its 45 MHz maximum | Same |
| `HSE_VALUE` never defined anywhere | Fell through to ST's 25 MHz default; `SystemCoreClockUpdate()` would report 562.5 MHz, and FreeRTOS programs SysTick from it | Grep for the symbol after fixing the PLL |
| Six pins claimed twice, later `_init()` silently winning | `printf` died the moment `oled_init()` ran; SPI1_MISO and TIM3_CH1 both broken by an `\|=` producing a nonexistent AF7 on PA6 | Cross-referencing every driver's pin config against the AF table |
| `pwm_set_us()` ignored its `channel` argument | All 8 thrusters collapsed onto CCR1, last write winning | Reading the function |
| `dummy_task` stack 180 B short | Would have overflowed on its first stack-audit print | `-fstack-usage` + call-chain summation |
| Null-PSP window before `vTaskStartScheduler()` | An early UART byte pends PendSV, whose context save writes through PSP = 0 | Disassembling `PendSV_Handler` and `vPortStartFirstTask` |
| Unbounded I²C polls | With no sensor attached, `bar30_task` spins forever at priority 4, starving priorities 1–3 while the board looks alive | Enumerating every `while (!(REG & flag))` and asking who controls completion |
| `ring_buffer.c`: `rx_head` / `rx_tail` / `rx_count` **not** `volatile` | `rx_write()` runs in `USART1_IRQHandler` while `rx_avail()` / `rx_peek()` / `rx_eat()` run in `comms_task`. `rx_count` is read in the packet framing loop — exactly the shape a compiler is entitled to hoist. `uart3.c` and `uart4.c` had qualified their equivalents; this file had not | Systematic `volatile` audit of every cross-context variable |
| `ring_buffer.c`: `rx_write()` called `__enable_irq()` unconditionally **from ISR context** | Re-enables interrupts regardless of the caller's state, breaking any critical section in effect further up the stack | Same audit — reading who calls the function, not just what it does |
| SD busy timeout expressed as a loop count | ~110 ms against a 250 ms spec allowance, so a merely-slow card was reported as failed | Converting the loop count to time |
| Slew limiter active on every armed tick | Capped control authority at 6.25 % of range per tick, invisible from outside and indistinguishable from mis-tuned gains | Reading `applyPWM` while checking a different claim |

The last two `ring_buffer.c` entries are worth singling out: they were the
only defects that a purely structural review would have missed. Finding
them needed the question *"who executes this function, and who else
touches what it touches"* rather than *"is this function correct"*.

---

## 13. Bare-metal → RTOS: what changed

| | Bare-metal | FreeRTOS |
|---|---|---|
| Scheduling | Single superloop | 8 priority-preemptive tasks |
| Control timing | Best-effort loop iteration | TIM7 → 50 Hz notify, jitter bounded by priority |
| Concurrency safety | Implicit — one thing ran at a time | Explicit: bus owner, queues, `volatile` audit |
| SPI | No hazard — nothing could preempt a transfer | Structural exclusion via a single owning task |
| Timing source | `g_tick` from SysTick | `xTaskGetTickCount()`; SysTick is kernel-owned |
| Sensor absence | Blocking poll, board hangs | `SENSOR_ABSENT` / `SENSOR_FAULTED`, task keeps running |
| Failure reporting | LED blink | `.noinit` fault latch surviving warm reset, + console |
| Diagnostics | None | On-demand stack/health over VCP, drop counters everywhere |

`SysTick`, `SVC` and `PendSV` are owned by the FreeRTOS port — the old bare-metal handler bodies were deleted on remap to avoid duplicate-symbol link errors. `g_tick` and `delay_ms()` are gone: FreeRTOS owns SysTick, so `g_tick` was never incremented and `delay_ms()` would have hung forever if called.

Packets and log records both use CRC-16/IBM-3740, computed in software. The STM32's hardware CRC unit is CRC-32 only, and a pure function removes the shared-peripheral race the hardware unit had between `comms_task` and `control_task`.

---

## 14. Hardware-in-the-loop verification

### The rig

```
  Laptop                                     NUCLEO-F446RE
  ┌──────────────────┐   ST-LINK USB       ┌──────────────────────────────────────────┐
  │ hil_rtos.py      │◄───────────────────►│ USART2 console: single-key bench commands│
  │  presses keys    │   (flash, power,    │                                          │
  │  parses reports  │    console)         │ USART1 TX PA9 ──┐  loopback jumper       │
  │  writes a report │                     │ USART1 RX PA10 ◄┘  the real Pi-link path │
  └──────────────────┘                     │ TIM2_CH1 PA15 ◄── jumper ── any ESC pin  │
                                           │ I2C1 PB8/PB9 ◄──────────────► MPU-6050   │
                                           │ DWT cycle counter, run-time stats        │
                                           └──────────────────────────────────────────┘
```

No Raspberry Pi, scope or USB-serial adapter. With PA9 jumpered to PA10, the
board plays the Pi itself: the bench layer injects command frames on USART1
TX, and they come back through the vehicle's real receive path (DMA, IDLE and
half/full-transfer interrupts, ring buffer, parser, queue, control task),
while the board's own telemetry loops back alongside them. The board measures
itself with the DWT cycle counter, FreeRTOS run-time stats, a painted MSP,
and TIM2 input capture. The laptop presses console keys and keeps score. The
`BENCH_HIL` build flag carries all of it (`Core/Src/bench.c`);
`-DBENCH_HIL=0` removes it for the vehicle.

### Results

Firmware `02ee61a`, 2026-10-05, ENABLE_IWDG defined for the run. Verbatim
transcripts: [`02ee61a`](tools/hil/reports/2026-10-05_02ee61a.md) (final),
[`b429abe`](tools/hil/reports/2026-10-05_b429abe.md),
[`d53de2b`](tools/hil/reports/2026-10-05_d53de2b.md) (first run, PWM pin sweep).

| Test | Result | Measured |
|---|---|---|
| Boot and console | PASS | `CLK: OK (HSE bypass, 180 MHz)`; reset cause `PIN`; sensors absent handled; 13-line health report, 0 drops |
| Clock tree | PASS | DWT / TIM2 = 180.0000 cycles per µs; TIM2 vs SysTick exact over 10.14 s; laptop within 0.16 % |
| Scheduling, idle | PASS | tick 19,999.20–20,000.80 µs; TIM7 → task 6.31–6.88 µs; control body 17.0 µs avg; MSP 176 B |
| Heap prediction | PASS | free 9,448 B == minimum-ever 9,448 B |
| USART1 loopback | PASS | 104 frames back in 2 s |
| Telemetry over USART1 | PASS | 506 frames in 10 s, 0 CRC failures, 0 ring drops |
| Commands in, round trip out | PASS | armed after 3 packets; 506 / 506 values exact; parse → applied 8.8 ms (≤ one tick); tick jitter 1.31 µs loaded |
| Junk and split frames | PASS | 257 round trips, 0 corrupted values used, 0 failsafe trips |
| Corrupted frames | PASS | poisoned value never reached control; failsafe at 520 ms |
| Command-timeout failsafe | PASS | 3 / 3 at 520 ms |
| Failsafe, `comms_task` suspended | PASS | 520 ms |
| Recovery streak | PASS | 3 consecutive packets re-arm; a 1.2 s gap resets the streak |
| DMA exact-wrap | PASS | IDLE-only: 0 / 256 bytes (defect reproduced); with HT/TC: 256 / 256 |
| I²C under preemption (MPU-6050) | PASS | RM0390 sequence: 4,000 reads, 0 errors; existing `i2c_read()`: 8 bus errors in 4,000; 0 wrong values either way |
| Soak, 60 s | PASS | 3,013 frames, 0 CRC failures, 3,013 / 3,013 exact, 0 link drops, 0 resets; CPU idle 44.6 % |
| Eight PWM outputs | 7 / 8 (`d53de2b`) | PB4, PB5, PB0, PB1, PC6, PC7, PC9 exact at their channel widths, period 20,000 µs; PC8 see below |
| Stacks after error paths | PASS | all 8 tasks ≥ 256 B free; MSP 208 B |
| Fault latch | PASS | `configASSERT` latched in `.noinit`, reported after the watchdog reset with file and line |
| Watchdog, hung `control_task` | PASS | 3 / 3 resets in 1,055–1,121 ms, cause `IWDG`; LSI ≈ 29 kHz |

**PC8 (thruster 7).** In the PWM sweep the jumper landed on the neighbouring
pin and read thruster 5's 1,500 µs signature, which the test correctly
rejected. PC8 is on the same timer (TIM8 CH3) as three channels that passed.

### Running it

```
pip install pyserial
python tools/hil/hil_rtos.py          # press RESET when prompted; move the PA15 jumper 8 times
python tools/hil/hil_rtos.py --selftest   # the script against a built-in fake board
```

Wiring: PA9 (D8) → PA10 (D2); MPU-6050 VCC → 3V3, GND, SCL → PB8 (D15), SDA → PB9 (D14);
one jumper from PA15 (CN7 pin 17), moved when prompted. Each block of tests
SKIPs without its wiring. For the fault and watchdog tests to reset the board
by themselves, build with `ENABLE_IWDG` defined, and power-cycle the board
after flashing so the debug unit is off.

---

## 15. Key figures

Every number is measured on a NUCLEO-F446RE ([§14](#14-hardware-in-the-loop-verification), transcripts in
[`tools/hil/reports/`](tools/hil/reports/)) or read from a build output.

| | Figure | Source |
|---|---|---|
| **Scope** | | |
| Code | 5,278 lines of C (cloc), 2,943 lines of comments; FreeRTOS kernel and CMSIS excluded | cloc |
| Tasks | 8 application tasks, 6 priority levels; 7 queues; 0 runtime allocations | source, heap test |
| Peripherals at register level | RCC, PWR, FLASH, GPIO, USART1/2/3, UART4, DMA1, DMA2, SPI2, I2C1, TIM2/3/7/8, IWDG | source |
| HIL tests | 19 automated tests, ~8 min per run; built-in fake board for development | `tools/hil/` |
| Defects found and fixed | 4 on hardware (1 of them on the bare-metal board), 5 more confirmed on hardware, 12 by static analysis | §12 |
| **Real-time behaviour, measured** | | |
| Control-tick jitter (TIM7, 50 Hz) | 0.25–0.80 µs idle, ≤ 1.87 µs under full UART load (worst across all runs) | DWT |
| Interrupt → control task running | 6.2–7.2 µs | DWT |
| Control task body (PID + 6×8 allocation + PWM + log post) | 17 µs avg idle; 57 µs avg, 109 µs worst under load (0.55 % of the tick) | DWT |
| Command latency, parse → applied | 8.8 ms average, 8.9 ms worst; bounded by one 20 ms tick | DWT |
| CPU load under 50 Hz traffic both ways | idle 44.6 %; `comms_task` 54.4 % (blocking TX), control 0.3 % | run-time stats |
| **Safety behaviour, measured** | | |
| Command-timeout failsafe | 520 ms in 5 / 5 trips, including with `comms_task` suspended | HIL |
| Failsafe recovery | 3 consecutive packets; a 1.2 s gap restarts the count | HIL |
| Watchdog on a hung control task | 3 / 3 resets, 1,055–1,126 ms across runs; cause read back from `RCC_CSR` | HIL |
| Fault latch | survives a warm reset; reports kind, file and line | HIL |
| **Link integrity, measured on the vehicle UART path (USART1 + DMA)** | | |
| Soak, 60 s | 3,013 frames, 0 CRC failures, 3,013 / 3,013 values exact, 0 drops | HIL |
| DMA exact-wrap defect | 0 / 256 bytes before the fix, 256 / 256 after | HIL |
| Parser robustness | 257 frames recovered through junk, decoys and split IDLE events | HIL |
| **Peripherals, measured** | | |
| PWM outputs | 7 of 8 pins measured exact at 1 µs resolution, 20,000 µs period, 62–65 pulses each | TIM2 capture |
| I²C under preemption | 4,000 MPU-6050 reads, 0 errors (RM0390 sequence); 14-byte burst 1,570 µs at 100 kHz | HIL |
| **Memory** | | |
| Flash, vehicle build | 34,916 B, 6.7 % of 512 KiB | `arm-none-eabi-size` |
| Static RAM, vehicle build | 29,736 B, 22.7 % of 128 KiB | `arm-none-eabi-size` |
| Heap | 9,448 B free == minimum-ever; predicted 9,444 B from the map | HIL |
| MSP peak | 176–248 B, against a 360 B static budget | painted MSP |
| Stack margins | every task ≥ 256 B free after all error paths | HIL |
| Compiler warnings at `-O2` | 0, both builds | build |

---

## 16. What is not verified

Run on hardware: everything in §14. Not reached by the bench:

- **The SD card and the OLED** (not fitted). Every figure in §4 and §5 is computed. With no card, the logger's block writes fail and are counted.
- **The VN-200, the Wayfinder DVL and the Bar30** (not fitted). Their tasks were exercised only on the absent-sensor path; the parsers have not seen real traffic.
- **ESCs and thrusters.** Outputs were measured at the pin, not driving an ESC.
- **PC8 (thruster 7) PWM**, see §14.
- **The in-water behaviour**: PID gains, thruster calibration, the allocation matrix against real thrust.
- **Remaining checklist items** in [`docs/HARDWARE_CHECKLIST.md`](docs/HARDWARE_CHECKLIST.md) that need the parts above (SD timing, OLED refresh, ESC arming).

---

## 17. Building

Import into STM32CubeIDE as an existing project. Any new folder under `Devices/` or `Drivers/` must have build inclusion checked: right-click → Resource Configurations → Exclude from Build → leave **unchecked**. For a headless build, the project name contains a `+`, which the builder reads as a regular expression: pass it as `"Nucleo_AUV_Bare_Metal\+RTOS/Debug"`.

**Build options**

| Define | Where | Default | Effect |
|---|---|---|---|
| `BENCH_HIL` | `Core/Inc/bench_config.h` | `1` | HIL instrumentation and console bench keys. **Build the vehicle with `-DBENCH_HIL=0`**: the bench keys can drive ESC outputs off neutral |
| `ENABLE_IWDG` | `Drivers/iwdg/iwdg.h` | off | Independent watchdog, refreshed only by `control_task`. Off by default for debugging; a build without it says so in a boxed banner and a distinct LD2 pattern |

The figures in this document come from an out-of-tree build at `-O2` (the CubeIDE Debug configuration is `-O0`):

```
arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
  -std=gnu11 -O2 -g3 -DSTM32F446xx -DHSE_VALUE=8000000U \
  -ffunction-sections -fdata-sections -Wall -Wdouble-promotion \
  -fstack-usage --specs=nano.specs
```

`-DHSE_VALUE=8000000U` matters: nothing in the tree defined it, so `system_stm32f4xx.c` fell through to ST's 25 MHz default and `SystemCoreClockUpdate()` would have reported 562.5 MHz.

---

## 18. Console

115200 8N1 on the ST-Link VCP. Single-key commands:

| Key | Output |
|---|---|
| `s` | Per-task stack high-water marks, with the lower-bound caveat |
| `h` | Clock status, heap free / min-ever-free, all drop counters, SD block counts, sensor states, link and failsafe state |
| `?` | Help |

The bench build adds, all printed as `B:` lines for `hil_rtos.py`:

| Key | Does |
|---|---|
| `b` / `z` | Bench report (timing, CPU, MSP, link counters) / zero the statistics |
| `a` `d` `1` `x` `e` `j` | Inject armed / disarmed / one / no / bad-CRC / junk-wrapped command frames on USART1 |
| `w` / `v` | 256-byte DMA wrap test with / without half- and full-transfer draining |
| `i` / `I` | MPU-6050 stress test with `i2c_read()` / `i2c_read_rm()` |
| `p` / `P` | PWM capture report on PA15 / toggle the per-channel PWM signature |
| `K` / `k` | Suspend / resume `comms_task` |
| `F` / `W` | Trip a `configASSERT` / hang `control_task` |

**`IWDG` is disabled by default.** A build without it announces itself with a boxed boot banner *and* a distinct LD2 pattern (short blip, long dark), because a watchdog-less build must never be mistakable for one with a watchdog on the bench.
