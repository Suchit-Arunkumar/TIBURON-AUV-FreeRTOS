# TIBURON AUV — FreeRTOS Firmware

Register-level FreeRTOS firmware for an 8-thruster AUV on an STM32F446RE (Nucleo-64). No HAL, no CubeMX-generated init — every peripheral driver is hand-written against the reference manual. A migration of [Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal), which changed *scheduling and synchronisation* around the same drivers, then hardened them.

> **Verification status, up front.** Everything here is compile-verified and statically analysed against a pinned `-O2` build. **Nothing in this document has been measured on hardware** — the target board and its sensors are not yet available. Every number is labelled `COMPUTED`, and [§14](#14-what-is-hardware-pending) lists exactly what remains unverified. See [`docs/HARDWARE_CHECKLIST.md`](docs/HARDWARE_CHECKLIST.md).

---

## 1. The failsafe is the architecture

If a comms link to a submerged vehicle drops, the thrusters must stop. The obvious implementation puts that check in the task that handles comms — and that is exactly where it must not be, because the failure that kills the link and the failure that hangs the comms task have a great deal of overlap.

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

The single point of failure is TIM7 itself. If TIM7 stops, `control_task` stops refreshing the watchdog, and the board resets within ~1 s — which drives the ESCs to no-signal. The failure mode of the failsafe is therefore *also* a failsafe.

**Recovery is deliberately asymmetric.** Going into failsafe takes one missed deadline. Coming out takes **three consecutive CRC-valid packets**, because one packet after a dropout is not evidence of a restored link — it may be the only one that got through. And `enterFailsafe()` zeroes the setpoint as well as the integrator state, so a stale packet arriving after a long dropout **cannot re-apply the throttle the vehicle was carrying when the link died**. Recovery starts from a stationary command and the slew limiter walks the outputs up from neutral at 2500 µs/s.

---

## 2. At a glance

| | |
|---|---|
| **MCU** | STM32F446RE @ 180 MHz — HSE bypass (no crystal fitted), PLL M=8 N=360 P=2, over-drive enabled |
| **RTOS** | FreeRTOS V11.1.0+, `heap_4`, GCC `ARM_CM4F` port, hard-float `fpv4-sp-d16` |
| **Tasks** | 9 application tasks over 7 priority levels; **11 tasks over 8 levels** at runtime (see [§3](#3-tasks-and-priorities)) |
| **Flash** | 36 248 B — 6.9 % of 512 KiB |
| **RAM** | 29 784 B — 22.7 % of 128 KiB |
| **Heap free** | 8 704 B of 22 512 usable — 38.7 % `COMPUTED` |
| **Toolchain** | `arm-none-eabi-gcc` 11.3.1 (GNU Tools for STM32 11.3.rel1), `-O2`, zero warnings |

---

## 3. Tasks and priorities

**9 application tasks across 7 distinct priority levels.** Stated separately because it is a real distinction: FreeRTOS adds an idle task the application never mentions, so the *runtime* figure is **11 tasks across 8 levels (0–7)**.

| Prio | Task | Stack | Trigger | Role |
|---|---|---|---|---|
| 7 | `control_task` | 1024 B | TIM7 notify, 50 Hz | PID, allocation, PWM, **failsafe**, IWDG refresh |
| 6 | `filter_task` | 1024 B | 10 ms periodic | Complementary filter — VN-200 + DVL + Bar30 → state estimate |
| 5 | `comms_task` | 1024 B | USART1 IDLE notify | CRC-32 packet parse, telemetry build |
| 4 | `vn200_task` | 1024 B | USART3 DMA+IDLE | IMU parser |
| 4 | `dvl_task` | 1024 B | UART4 DMA+IDLE | Doppler velocity log parser |
| 4 | `bar30_task` | 1024 B | 20 ms periodic | Depth sensor (I²C) |
| 3 | `spi_owner_task` | 1024 B | Queue, 500 ms timeout | **Sole owner of SPI2** — OLED + SD |
| 2 | `logging_task` | 1024 B | Queue from control | Batches records into 512 B blocks |
| 1 | `dummy_task` | 1024 B | Queue, 50 ms timeout | Heartbeat, **sole stdio owner**, on-demand reports |
| 0 | `IDLE` | 512 B | — | Kernel |

`configUSE_TIMERS` is **0**. `xTimerCreate` is called zero times, so the timer-service daemon was pure cost — 816 B of heap and a pinned priority 2 that collided with `logging_task`. Turning it off freed both.

Three tasks share priority 4 with `configUSE_TIME_SLICING 1`, so they round-robin on each 1 ms tick. None can monopolise; each may wait up to 2 ms for its turn, against budgets of ≥10 ms.

> **What "implemented" means for the controller.** The control loop's
> *structure* is complete and runs end to end: error, clamped integral,
> derivative, feedforward, per-axis saturation, and thrust allocation
> through a pre-computed pseudo-inverse. **The PID gains are all zero**,
> so the loop currently computes an identically zero command. Tuning
> needs the vehicle in water, not the board. The complementary filter, by
> contrast, is fully implemented — body-to-world rotation, gravity
> compensation, strapdown integration, and per-sensor correction gates.

**LD2 is a liveness indicator at the lowest priority.** That is a deliberate trade — it means the LED proves the *whole* schedule is running, not just the top of it — but it has a consequence worth stating: **a starved system and a crashed system look identical from the LED alone.** If the heartbeat stops, the console (`h`) and the fault latch are what distinguish them.

---

## 4. SPI arbitration: bus owner, not mutex

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

The owner must sit **strictly above every task that sends it requests** (`logging_task` at 2), so a requester cannot preempt the owner and pile up more work while a transaction is in flight. It must sit **strictly below every task with a deadline** (control 7, filter 6, comms 5, sensors 4), so a 250 ms program cycle is preemptible by all of them. Priority 3 is the unique slot satisfying both.

The OLED is a request **type** (`SPI_REQ_OLED_FRAME`), not the task's identity. Adding a third device to the bus is a new enum value and a `switch` case, not a restructure.

---

## 5. Logging: batching, and what it costs

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

| Queue | Depth | Item | Heap | Full policy | Counter |
|---|---|---|---|---|---|
| `commandQueue` | 4 | 56 B | 304 B | 1 ms block, then drop | `comms_cmd_drops()` |
| `dvlQueue` | 1 | 76 B | 160 B | `xQueueOverwrite` — newest wins | — |
| `vn200Queue` | 1 | 40 B | 120 B | `xQueueOverwrite` | — |
| `bar30Queue` | 1 | 4 B | 88 B | `xQueueOverwrite` | — |
| `stateQueue` | 1 | 40 B | 120 B | `xQueueOverwrite` | — |
| `logQueue` | 8 | 41 B | 408 B | **zero block**, drop | `control_log_drops()` |
| `spiRequestQueue` | 2 | 520 B | 1120 B | 500 ms block, then drop | `logging_spi_post_drops()` |
| `consoleQueue` | 8 | 80 B | 720 B | **zero block**, drop | `console_dropped()` |

Depth-1 queues use `xQueueOverwrite` because the consumer only ever wants the latest measurement, never a backlog. Absorption: `logQueue` holds 1.6 s of records; `spiRequestQueue` holds 4.8 s of block production.

**Every drop site has a counter.** That was not true until Phase 12 — `commandQueue` was the one silent-loss path.

---

## 7. Interrupts

`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`, and with `configPRIO_BITS = 4` that makes `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 4 = 0x50`.

The rule: **an ISR may call a `...FromISR` API only if its raw IPR byte is numerically ≥ 0x50.** Lower numbers are higher priority and are *not* masked by `BASEPRI = 0x50`, so they could preempt a kernel critical section.

| IRQ | `NVIC_SetPriority` | Raw IPR | Calls kernel API? | Legal |
|---|---|---|---|---|
| TIM7 | 5 | 0x50 | `vTaskNotifyGiveFromISR` | Yes — **at the ceiling** |
| USART1 | 5 | 0x50 | `xTaskNotifyFromISR` | Yes — at the ceiling |
| USART2 | 6 | 0x60 | none (one volatile store) | Yes |
| USART3 | 6 | 0x60 | `xTaskNotifyFromISR` | Yes |
| UART4 | 6 | 0x60 | `xTaskNotifyFromISR` | Yes |
| SysTick / PendSV | — | 0xF0 | kernel-owned | Yes |

TIM7 and USART1 sit *exactly* at the ceiling — legal, with zero margin. Dropping `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` to 4 would break both, and `configASSERT` would catch it.

`SCB->AIRCR` PRIGROUP is set to 3 (4 bits preemption, 0 subpriority) as **init step 2**, before any `NVIC_SetPriority` call.

### No kernel API is reachable from any ISR before the scheduler runs

`NVIC_EnableIRQ` for every interrupt-driven peripheral is called from the *consuming task's* first iteration, never from `main`:

| Peripheral | Enabled by | In |
|---|---|---|
| TIM7 | `tim7_init()` | `control_task` |
| USART1 | `uart1_irq_enable()` | `comms_task` |
| USART3 | `uart3_irq_enable()` | `vn200_task` |
| UART4 | `uart4_irq_enable()` | `dvl_task` |

Between the last `xTaskCreate` and `vTaskStartScheduler()`, `pxCurrentTCB` is populated but **PSP is still zero** — `vPortSVCHandler` sets it, and that runs inside `vPortStartFirstTask`. An ISR reaching `portYIELD_FROM_ISR` in that window pends PendSV, whose context save does `mrs r0, PSP` then `stmdb r0!, {...}`: a write through a null stack pointer.

USART2 is the sole exception, enabled in `main`. Its handler's entire reachable body is one volatile byte store — no FreeRTOS API, no yield. From `vTaskStartScheduler()` onward the kernel closes the window itself: `portDISABLE_INTERRUPTS()` sets `BASEPRI = 0x50` before `xPortStartScheduler()`, masking every interrupt in this design.

---

## 8. Memory

### Static, from `out.map`

```
   text	   data	    bss	    dec	    hex
  36128	    120	  29664	  65912	  10178
```

**Flash = text + data = 36 248 B (6.9 % of 512 KiB).**
**RAM = data + bss = 29 784 B (22.7 % of 128 KiB)**, leaving 101 288 B unallocated.

`bss` includes `ucHeap` (22 528 B), the OLED framebuffer (1024 B), the log staging block (512 B), four 256 B UART DMA/ring buffers, and `._user_heap_stack` (1536 B).

### Heap ledger `COMPUTED — NOT HARDWARE VALIDATED`

`ucHeap` links at `0x200013BC`, which is **not 8-byte aligned**, so `prvHeapInit` loses 4 B to alignment and 8 B to `xEnd`. Usable is **22 512** of the 22 528 declared — read from the map, not assumed.

```
queues                 3040
task stacks            9288   (9 × 1024 B, +8 header, 8-aligned)
TCBs                    864   (9 × sizeof(TCB_t)=84 → 96)
IDLE stack + TCB        616
timer daemon              0   (configUSE_TIMERS 0)
────────────────────────────
used                  13808
free                   8704   (38.7 %)
```

**Every allocation happens before `vTaskStartScheduler()`**, and every `xTaskCreate` / `xQueueCreate` return value is checked — failure latches the name and halts before the scheduler starts. No task calls `pvPortMalloc` at runtime, so heap exhaustion after init cannot occur. That gives a falsifiable prediction: on hardware, `xPortGetMinimumEverFreeHeapSize()` should **equal** `xPortGetFreeHeapSize()`. If it does not, something allocates at runtime that this analysis has not found.

### Stacks `COMPUTED`

From `-fstack-usage` frames plus deepest call chains, plus **204 B** of context overhead per task (104 B hardware exception frame with FPU context reserved by lazy stacking, ~100 B PendSV save of s16–s31/r4–r11/lr).

| Task | Alloc | Worst case | Free | |
|---|---|---|---|---|
| `control_task` | 1024 | 548 | 476 | 54 % |
| `filter_task` | 1024 | 588 | 436 | 57 % |
| `comms_task` | 1024 | 492 | 532 | 48 % |
| `vn200_task` | 1024 | 556 | 468 | 54 % |
| `dvl_task` | 1024 | 668 | 356 | 65 % |
| `bar30_task` | 1024 | 476 | 548 | 46 % |
| `spi_owner_task` | 1024 | 436 | 588 | 43 % |
| `logging_task` | 1024 | 348 | 676 | 34 % |
| `dummy_task` | 1024 | 692 | 332 | 68 % |

Static analysis found `dummy_task` **overflowing** at its original 512 B (692 B required — certain even discounting the `vsnprintf` estimate: 128+144+96+204 = 572 > 512). It was grown to 1024 B. `spi_owner_task` and `logging_task` were at 93 % and 84 % from carrying a 520 B `SpiRequest` on the stack; both are now `static`, which does not touch the by-value queue decision.

**No stack has been trimmed, and the evidence for that is concrete: `vn200_task` computed at 33 % in Phase 9 and 54 % in Phase 10, purely from `console_printf` calls added one phase later — had it been trimmed to 512 B on the earlier number, it would now overflow.**

### MSP budget `COMPUTED`

Tasks run on PSP and handler bodies on MSP. A first exception from thread mode pushes its hardware frame to PSP; **only nested exceptions consume MSP.** Same-priority peers cannot nest, so the chain is at most 3 deep:

```
L1  SysTick or PendSV                             8 B
L2  USART3/UART4  hw 104 + align 4 + handler 72  180 B
L3  USART1        hw 104 + align 4 + handler 64  172 B
                                                 ─────
                                                 360 B  of 1024 reserved
```

**L1 contributes only 8 B, and that is the load-bearing claim.** `SysTick_Handler` pushes `{r3,lr}` and `PendSV_Handler` pushes `{r0,r3}`, and both then set `BASEPRI = 0x50` within two instructions — masking every interrupt in this design. So `xTaskIncrementTick` (48 B) and `vTaskSwitchContext` (16 B) are **unreachable from any nest chain**: the deepest kernel work can never stack on top of a UART burst. Verified in the disassembly, not assumed.

664 B spare; 468 B used even with a terminal HardFault stacked on top.

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

The Pi connects **GPIO-to-GPIO on USART1**, not over USB, so USART2/VCP stays free for the console.

Three details that are silent failures if missed:

- **PB4 is NJTRST** and boots in an alternate-function state. Its MODER *and* AFR bits are cleared explicitly before TIM3_CH1 is configured — a bare OR leaves thruster 1 dead with nothing in the code looking wrong.
- **TIM8 is an advanced-control timer**: `BDTR.MOE` must be set or CH1–4 stay electrically disconnected regardless of `CCER`. TIM3 has no equivalent bit.
- **The two prescalers differ on purpose.** TIM3 is on APB1 (90 MHz timer clock) → `PSC=89`; TIM8 is on APB2 (180 MHz) → `PSC=179`. Both `ARR=19999` for 1 µs resolution and a 20 ms frame. Equal prescalers would run thrusters 5–8 at twice the frame rate.

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

---

## 11. Two decisions worth the words

### Bounded vs unbounded polling — the distinction is who controls completion

`i2c.c`'s waits are bounded and yielding. `spi.c`'s and the UART TX paths' are not, and that is deliberate rather than an oversight.

**I²C completion is controlled by an external agent.** A slave can stretch SCL indefinitely, or simply never ACK. `while (!(SR1 & flag));` therefore has no bound in the physical sense, not merely in the code. With no Bar30 attached, `bar30_task` spun forever at priority 4 — control, filter and comms kept running so the board *looked* alive, while logging, the SPI owner and the console were starved permanently.

**SPI and UART TX completion is master-generated.** The STM32 produces SCK; the UART shifts at its configured baud regardless of any receiver. `TXE`, `RXNE` and `BSY` transitions depend only on internal clocking, so those loops are bounded by hardware even though they are not bounded by code. Adding timeouts there would be ceremony.

The I²C wait is now **spin-then-yield**: 250 µs of spinning covers a healthy 90 µs phase with margin so a working bus never pays a context switch, then 1 ms yields up to a 2.5 ms budget. Combined with backoff on consecutive *failures* (not merely on a confirmed-absent sensor), worst-case duty at priority 4 falls from ~65 % to under 2 %. The intermittent-NACK case matters more than clean absence: a sensor on jumper wires to a breakout NACKs intermittently far more often than it goes cleanly missing.

### No floating-point conversions in the console

`--specs=nano.specs` without `-u _printf_float` means `%f` **prints nothing and takes the rest of the line with it** — no error, no warning, no wrong number. Confirmed absent from the ELF. The decision is fixed-point milli-units via `console_fmt_milli()`, not the ~6 KB of `_printf_float`, and the reason beyond the flash is this:

> **Varargs promote `float` to `double` unconditionally — that promotion is mandated by the language, so `-Wdouble-promotion` cannot flag it.** Every `%f` call site would quietly reintroduce soft-float double conversion into a codebase that is otherwise strictly single-precision on a single-precision FPU.

Float formatting also costs ~100 B of stack per call, on the *caller's* stack, which is the least-characterised part of the budget. `console_fmt_milli` takes a caller-supplied buffer on purpose: a shared one would make `console_printf("d=%s u=%s", fmt(a), fmt(b))` print one value twice, since argument evaluation order is unspecified — and that failure reads as a sensor fault, not a formatting fault.

---

## 12. Defects found by static analysis

Every one of these was found by reading the map file, the `-fstack-usage`
output, or the disassembly — not by running the board, which has not
happened. They are listed because the method is the point: a target you
cannot power is still a target you can analyse.

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
| Scheduling | Single superloop | 9 priority-preemptive tasks |
| Control timing | Best-effort loop iteration | TIM7 → 50 Hz notify, jitter bounded by priority |
| Concurrency safety | Implicit — one thing ran at a time | Explicit: bus owner, queues, `volatile` audit |
| SPI | No hazard — nothing could preempt a transfer | Structural exclusion via a single owning task |
| Timing source | `g_tick` from SysTick | `xTaskGetTickCount()`; SysTick is kernel-owned |
| Sensor absence | Blocking poll, board hangs | `SENSOR_ABSENT` / `SENSOR_FAULTED`, task keeps running |
| Failure reporting | LED blink | `.noinit` fault latch surviving warm reset, + console |
| Diagnostics | None | On-demand stack/health over VCP, drop counters everywhere |

`SysTick`, `SVC` and `PendSV` are owned by the FreeRTOS port — the old bare-metal handler bodies were deleted on remap to avoid duplicate-symbol link errors. `g_tick` and `delay_ms()` are gone: FreeRTOS owns SysTick, so `g_tick` was never incremented and `delay_ms()` would have hung forever if called.

The CRC field on a log record is named `crc16` for historical reasons but is **a truncated CRC-32**: the STM32 has one hardware CRC-32/MPEG-2 unit, and `control_task` takes its low 16 bits. Packet validation uses the full CRC-32.

---

## 14. What is hardware-pending

**Nothing in this repository has been run on a board.** The target hardware (custom STM32H723 PCB, plus VN-200, Wayfinder DVL, Bar30 and thrusters) is not yet available, and the Nucleo-F446RE bench rig has no sensors attached.

| Phase | Status |
|---|---|
| A — audit | Complete, static |
| B — clock, fault latch, pin map, PWM | **Implemented, compile-verified, not run** |
| 8 — SPI arbitration, logging | **Implemented; all timing figures COMPUTED** |
| 9 — priority + stack audit | **Static only.** High-water marks print on demand; no numbers exist yet |
| 10 — integration | **Implemented; init order verified by inspection** |
| 11 — load check | **Analysis, not tested** |
| 12 — residual risks | Implemented, static |
| 13 — documentation | This file |
| 14 — hardware checklist | [`docs/HARDWARE_CHECKLIST.md`](docs/HARDWARE_CHECKLIST.md) — 15 ordered points |

**The controller is un-tuned.** All PID gains are zero. Everything around
the gains — timing, allocation, saturation, failsafe, logging — is
implemented and analysed; the gains themselves are a wet-testing task and
are not claimed as done.

**Known-weakest numbers**, both unresolvable without the board:

- The **`vsnprintf` stack frame is estimated at ~120 B** and newlib-nano is often more. It appears in the chain of every task that can call `console_printf` on an error path, and `dummy_task` at 68 % has the least margin. The audit prints this caveat alongside the numbers so it cannot harden into a fact through repetition.
- The three UART ISRs treat `current_pos == last_dma_pos` as "no new data". An **exact full-buffer DMA wrap** between two IDLE events is indistinguishable from no data, and 256 B would be silently lost. Unlikely at 115200 with IDLE detection; needs measurement under sustained load, not more analysis.

---

## 15. Building

Import into STM32CubeIDE as an existing project. Any new folder under `Devices/` or `Drivers/` must have build inclusion checked: right-click → Resource Configurations → Exclude from Build → leave **unchecked**.

The figures in this document come from a pinned out-of-tree build, not the CubeIDE Debug configuration (which is `-O0`):

```
arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
  -std=gnu11 -O2 -g3 -DSTM32F446xx -DHSE_VALUE=8000000U \
  -ffunction-sections -fdata-sections -Wall -Wdouble-promotion \
  -fstack-usage --specs=nano.specs
```

`-DHSE_VALUE=8000000U` matters: nothing in the tree defined it, so `system_stm32f4xx.c` fell through to ST's 25 MHz default and `SystemCoreClockUpdate()` would have reported 562.5 MHz.

## 16. Console

115200 8N1 on the ST-Link VCP. Single keypress commands:

| Key | Output |
|---|---|
| `s` | Per-task stack high-water marks, with the lower-bound caveat |
| `h` | Clock status, heap free / min-ever-free, all drop counters, SD block counts, sensor states, link and failsafe state |
| `?` | Help |

**`IWDG` is disabled by default.** A build without it announces itself with a boxed boot banner *and* a distinct LD2 pattern (short blip, long dark), because a watchdog-less build must never be mistakable for a watchdog-ed one on the bench. Uncomment `ENABLE_IWDG` in `Drivers/iwdg/iwdg.h` to arm it.
