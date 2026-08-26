# Hardware Verification Checklist

Fifteen points, in order. **Each assumes every prerequisite point has passed.**

Nothing in this firmware has run on a board. Every figure in the README is computed from the map file, `-fstack-usage` output, or disassembly. This checklist is the list of things that turns those into measurements — and, more importantly, the list of things that could falsify them.

**Work in order.** The ordering is not stylistic: point 1 gates every timing number in the repository, points 2–4 gate the console you need to read the rest, and points 5–7 gate anything involving thrusters. Skipping ahead produces results you cannot interpret.

**Record actual values, not pass/fail.** A point that "passes" with a number 30 % off its prediction has told you something a checkbox would hide.

---

## Group A — the clock, and being able to see anything

### 1. SYSCLK is actually 180 MHz

**Prerequisite:** none. This is first because **every other number in the repository depends on it** — every UART divisor, the I²C bit rate, both PWM prescalers, the TIM7 period, and the SysTick reload all assume 180 MHz with APB1 at 45 and APB2 at 90.

**Expected:** 180.000 MHz ± 0.01 % (HSE bypass from the ST-LINK MCO is crystal-derived).

**How to measure — two independent ways, do both:**

- **MCO2 on PA9 to a scope.** PA9 is USART1_TX in the final pin map, so this requires temporarily lifting the Pi link. Set `RCC->CFGR` `MCO2 = 00` (SYSCLK) with `MCO2PRE = 110` (÷4) and configure PA9 as AF0 high-speed. Expect **45.000 MHz**. Do not attempt to observe 180 MHz directly on a Nucleo header pin — the trace and probe capacitance will not support it and you will measure your probe.
- **GPIO toggle against a known interval.** In `dummy_task`, toggle a spare pin (PC2 is free) inside a loop of exactly N iterations and time it. Better: use TIM2 — it is already running at 1 µs/count off APB1. Print `micros()` deltas across a `vTaskDelay(1000)`. Expect **1 000 000 ± 1000 µs**. This cross-checks SYSCLK, the APB1 prescaler and the SysTick reload in one measurement.

**On failure:**
- Reads **~40 MHz on MCO2 (160 MHz SYSCLK)** → PLL locked but the over-drive step silently did not take. Stop; check `PWR->CSR` ODRDY/ODSWRDY.
- Reads **4 MHz (16 MHz SYSCLK)** → you are on the raw-HSI fallback and LD2 will be blinking a fault code. Count the blinks: 2 = PLL never locked, 3 = over-drive timeout, 4 = SYSCLK switch timeout.
- **`micros()` delta is 1 000 000 but MCO disagrees** → the APB1 prescaler is wrong, not SYSCLK. TIM2 would be counting at the wrong rate and coincidentally cancelling.
- **Anything other than 180 MHz: stop the checklist here.** Every subsequent number is meaningless. Fix the clock first.

---

### 2. Clock source is HSE, not the HSI fallback

**Prerequisite:** 1.

**Expected:** console prints `CLK: OK (HSE bypass, 180 MHz)`, and LD2 blinks a **steady, symmetric 1 Hz**.

**How to measure:** read the boot banner. If the console is unreadable, read LD2: a **double-pulse** (blink-blink-pause, once per second) means the HSE never reported ready and the PLL was retuned to M=16 off the HSI.

**On failure (DEGRADED):** the board still runs at 180 MHz and everything works, but the clock is now HSI-grade (±1 % at room temperature, worse across range). **Continue the checklist, but treat every timing measurement as ±1 % rather than ±0.01 %,** and note that the console itself may be marginal — 115200 framing tolerates roughly ±2 %, so a hot board could push it over. Investigate SB54/SB55 solder bridges and the ST-LINK MCO output before trusting any point past 8.

---

### 3. Console is readable end to end

**Prerequisite:** 2.

**Expected:** at 115200 8N1 on the ST-Link VCP, a clean boot banner: `BOOT OK`, `CLK: ...`, `SD: ...`, `UARTS CONFIGURED`, and the boxed `IWDG DISABLED` warning.

**How to measure:** open the VCP. Then press `?` — the help line proves the **RX** path works, which the banner alone does not.

**On failure:**
- **Garbage characters** → baud mismatch. Cross-check against point 1; if SYSCLK is right, `USART2->BRR` should be 391.
- **Banner appears, `?` does nothing** → USART2 RX is not working. `NVIC_EnableIRQ(USART2_IRQn)` is in `uart2_init`; check `USART_CR1_RE` and `RXNEIE`. Points 8, 10, 11, 13 and 14 all depend on the RX path, so fix this before continuing.
- **Nothing at all** → check that PA2/PA3 are not being reconfigured by something later in init. This was a live defect once (`oled_init()` used to steal them) and the whole pin map was rearranged to prevent it.

---

### 4. Fault latch survives a warm reset

**Prerequisite:** 3.

**Expected:** a deliberately triggered `configASSERT` halts the board; pressing the reset button produces `*** LATCHED FAULT FROM PREVIOUS RUN ***` with a plausible file, line and PC.

**How to measure:** add a temporary `configASSERT(0)` somewhere in a task, build, run, confirm the board halts, press **reset** (not power-cycle — `.noinit` is RAM and does not survive power loss by design), and read the banner. Remove the temporary assert afterwards.

**On failure:**
- **No banner after reset** → `.noinit` is being zeroed. Check the map: `_ebss` and `_snoinit` must be equal, and `g_fault_latch` must sit at or above `_ebss`, outside the range `startup_stm32f446retx.s` clears.
- **Banner appears on a cold power-up too** → the magic word is colliding with uninitialised RAM. Astronomically unlikely; if it repeats, something is writing the struct.

This point matters disproportionately: **every later point that ends in a halted board is diagnosed through this mechanism.** If it does not work, a failure at point 9 or 12 tells you nothing.

---

## Group B — outputs, before anything can move

### 5. All eight PWM channels produce 1500 µs at 50 Hz, from boot

**Prerequisite:** 1. **Do this with ESCs disconnected.**

**Expected:** on each of PB4, PB5, PB0, PB1, PC6, PC7, PC8, PC9 — a **20.00 ms period** with a **1.500 ms high time**, present from the moment `pwm_init()` runs and before the scheduler starts.

**How to measure:** scope each pin individually. Do not sample two and assume the rest.

**On failure:**
- **Channels 1–4 correct, 5–8 dead** → `TIM8->BDTR.MOE` is not set. TIM8 is an advanced-control timer and its outputs stay electrically disconnected regardless of `CCER`.
- **Channels 5–8 at 10 ms period / 750 µs pulse** → the two prescalers are equal. TIM3 is on APB1 (90 MHz timer clock, `PSC=89`); TIM8 is on APB2 (180 MHz, `PSC=179`).
- **Thruster 1 (PB4) alone dead** → PB4 is NJTRST and boots in an alternate-function state. Its MODER *and* AFR bits must be cleared explicitly, not OR-ed onto.
- **Any channel showing a 0 µs pulse** → a CCR is still at its reset value and an output stage was enabled before the neutral write. Do not connect ESCs until this is clean.

---

### 6. ESCs arm and hold neutral

**Prerequisite:** 5 fully passed on all eight channels.

**Expected:** with ESCs and a current-limited supply connected, all eight arm on their startup tone and hold stationary. Bench current should be near quiescent.

**How to measure:** listen for the arming sequence; watch a bench supply's current display.

**On failure:** any thruster that spins at boot means a CCR reached the output stage at something other than 1500 µs. Kill power. Return to point 5 — do not attempt to compensate in software.

---

### 7. Failsafe drives all eight to neutral within 500 ms

**Prerequisite:** 6. **Props off, or vehicle secured.**

**Expected:** with the Pi streaming valid commands and the vehicle armed, disconnect the link. All eight channels reach 1500 µs within **500 ms**. Reconnect: nothing moves until **three consecutive CRC-valid packets** have arrived, and outputs then ramp from neutral at 2500 µs/s rather than stepping.

**How to measure:** scope one thruster channel while pulling the USART1 connection. Trigger on the falling edge of the pulse width. For recovery, have the Pi send exactly two valid packets and stop — outputs must **not** move. Then send a third.

**On failure:**
- **Outputs do not go neutral** → the failsafe path is broken. This is the most important single behaviour in the firmware; stop and fix before anything else.
- **Outputs step rather than ramp on recovery** → `g_ramping` is not being set by `enterFailsafe()`.
- **Motion after one or two packets** → the recovery streak is not being enforced.
- **Outputs return to a pre-loss throttle** → `enterFailsafe()` is not zeroing `target[]`. This is the specific failure the design exists to prevent.

---

## Group C — the storage and display path

### 8. SD card enumerates at 351.6 kHz, then reads at 5.625 MHz

**Prerequisite:** 3.

**Expected:** `SD: OK` on the boot banner. On a scope or logic analyser on PB13 (SCK): **≈351.6 kHz during CMD0/CMD8/ACMD41/CMD58**, switching to **≈5.625 MHz** afterwards.

**How to measure:** capture SCK across boot with a logic analyser; the rate change is unmistakable.

**On failure:** the banner now names the stage — `NO CARD (CMD0 no response)`, `CMD8 check pattern mismatch`, `ACMD41 timeout`, or `OCR: 3.3V window unsupported`. `NO CARD` with a card inserted means wiring or power, not the card. If enumeration works but block writes fail, try lowering `SPI_BR_SD_DATA` before suspecting the card.

---

### 9. SD block write worst-case busy time

**Prerequisite:** 8.

**Expected:** typical single-block program **2–3 ms**; the README's worst case assumes the spec ceiling of **250 ms**.

**How to measure:** toggle a spare GPIO around the `sd_wait_ready()` call and capture for at least ten minutes of continuous logging — the long tail only appears when the card runs internal wear-levelling. Record the **maximum**, not the mean.

**On failure:**
- **Maximum exceeds 250 ms** → the timeout is too short and healthy writes are being failed. Raise `SD_WRITE_TIMEOUT_MS` and re-derive the duty-cycle figures in README §5.
- **Maximum is well under 250 ms** (likely) → the README's all-worst-case duty of 10.5 % is conservative. Record the real number; it is the one that matters for whether logging can ever back up.

---

### 10. OLED full-frame refresh time vs the 1.88 ms prediction

**Prerequisite:** 8 (shares the bus).

**Expected:** **≈1.88 ms** for a 1030-byte frame, of which 1.46 ms is irreducible bit time at 5.625 MHz.

**How to measure:** toggle a GPIO around `oled_update()`.

**On failure — and this is the point of the measurement:** the difference between the measured time and 1.46 ms **is the real per-byte driver overhead**, against the **computed ≈0.40 µs/byte** that propagates into every SPI figure in the README. If the measured overhead is materially different, **update README §4 and re-derive §5's duty cycles.** This is the single measurement that most improves the document's accuracy.

---

## Group D — the analysis, tested against reality

### 11. Stack high-water marks vs the static predictions

**Prerequisite:** 3, and ideally 7 and 14 so the error paths have actually run.

**Expected:** each task's used figure at or below the README §8 prediction. Predicted worst cases: control 548, filter 588, comms 492, vn200 556, dvl 668, bar30 476, spi_owner 436, logging 348, dummy 692 bytes.

**How to measure:** press `s`. **Then exercise the error paths and press `s` again** — pull the SD card to force `spi_owner_task`'s `console_printf` branch, disconnect the link to enter failsafe, and let a sensor time out. A high-water mark records only paths that actually executed, so a mark taken on a quiet board is a lower bound, not a result.

**On failure:**
- **Any task exceeding its prediction** → the call-chain analysis missed a path. Find it before trusting any other stack number.
- **`dummy_task` over 692 B** → most likely the `vsnprintf` frame; see point 12.

---

### 12. The `vsnprintf` frame against the ~120 B estimate

**Prerequisite:** 11.

**Expected:** `dummy_task` reports **≥332 B free** after a stack audit has run. That figure assumes newlib-nano's `vsnprintf` uses ~120 B, which is **an estimate and the weakest number in the repository.**

**How to measure:** two ways.
- **Indirect:** `dummy_task`'s worst case is `128 + 144 + 96 + vsnprintf + 204`. Its reported used figure minus 572 is the actual `vsnprintf` frame.
- **Direct:** break in the debugger inside `_svfiprintf_r` during a `console_printf` and compare SP against the task's stack base.

**On failure:** if the real frame materially exceeds 120 B, the deficit applies to **every task that can call `console_printf` on an error path** — control, comms, both sensor tasks, `spi_owner_task` and `dummy_task`. Recompute all of them before trimming anything. This is why no stack was trimmed on static numbers.

---

### 13. The falsifiable heap prediction

**Prerequisite:** 3. Run after at least ten minutes of normal operation.

**Expected — this is a prediction, not a check:**

```
xPortGetFreeHeapSize()            == 8704 B
xPortGetMinimumEverFreeHeapSize() == 8704 B     <- must be EQUAL
```

Every allocation happens before `vTaskStartScheduler()` and no task calls `pvPortMalloc` at runtime. If that is true, the free heap can never have been lower than it is now.

**How to measure:** press `h`. Both figures are printed.

**On failure — and failure here is genuinely informative:**
- **Minimum-ever is lower than current** → **something allocates and frees at runtime that the static analysis did not find.** That is a real defect in the analysis, not a tuning issue. Find it: the likely candidates are a newlib call pulling in `malloc` (stdio buffering via `_sbrk` is separate, but a locale or reentrancy path could reach `pvPortMalloc`), or a queue/task creation on a path that only runs conditionally.
- **Free heap is not 8704 B** → the ledger is wrong. Re-derive it from the map; note that `ucHeap`'s link address affects usable size through 8-byte alignment, and it was `0x200013BC` (losing 4 B) at the time of writing.

---

### 14. MSP high-water mark under DMA load

**Prerequisite:** 3, plus live UART traffic on as many of USART1/USART3/UART4 as available.

**Expected:** peak MSP usage **≤ 360 B** of the 1024 B reserved by `_Min_Stack_Size`.

**How to measure — fill pattern:**
1. Very early in `main`, before anything else, fill the region from `_end + _Min_Heap_Size` up to `_estack − 64` with `0xA5A5A5A5`. (Stay clear of the current frame.)
2. Run with the heaviest interrupt load available — ideally all three UARTs streaming while the SD card is writing, to maximise the chance of a genuine three-deep nest.
3. Halt and scan upward from the fill base for the first non-`0xA5A5A5A5` word. The distance from `_estack` to that word is the high-water mark.

**On failure:**
- **Above 360 B but below 1024** → the nest analysis under-counted. Most likely the FP context assumption: the analysis budgets 104 B per nested frame assuming lazy stacking reserves the FP area. Recheck which handlers have `CONTROL.FPCA` set.
- **At or near 1024 B** → raise `_Min_Stack_Size`. Note that `_sbrk` clamps the newlib heap at `_estack − _Min_Stack_Size`, so growing it costs available heap, not RAM in general.

---

### 15. DMA exact-wrap aliasing

**Prerequisite:** 3, and sustained traffic on at least one DMA UART.

**Expected:** no silent data loss under continuous streaming.

**The defect being tested for:** all three UART ISRs compute the DMA position as `SIZE − NDTR` and treat `current_pos == last_dma_pos` as "no new data". If the DMA writes **exactly one full buffer** (256 bytes) between two IDLE events, those two values are equal and the entire buffer is discarded — indistinguishable from silence. This is analysed as unlikely at 115200 with IDLE detection, but it has never been tested, and analysis cannot settle it.

**How to measure:** stream a monotonically increasing byte sequence from the Pi at the highest sustainable rate, with deliberately variable gap timing to vary where IDLE lands. In `comms_task`, check for sequence discontinuities and count them. Run for at least an hour.

**On failure:** any discontinuity of exactly 256 bytes confirms the aliasing. The fix is to track a byte counter that cannot alias — compare against total bytes DMA'd rather than against buffer position, or use the DMA half-transfer and transfer-complete interrupts alongside IDLE.

---

## What this checklist does not cover

**Nothing that requires the vehicle rather than the board:** in-water behaviour, PID gain tuning (all gains are currently zero), thruster calibration and the B⁺ allocation matrix against real thrust measurements, buoyancy and trim, sensor mounting alignment, watertight-enclosure thermal behaviour, or the VN-200 and DVL against real motion rather than a bench feed.
