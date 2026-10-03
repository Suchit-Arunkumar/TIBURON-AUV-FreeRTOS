#ifndef BENCH_CONFIG_H
#define BENCH_CONFIG_H

/*
 * BENCH_HIL - hardware-in-the-loop instrumentation for a bare Nucleo.
 *
 * 1 (default): the build carries the bench layer in Core/Src/bench.c:
 *   DWT cycle timing of the TIM7 -> control_task path, per-task CPU load,
 *   MSP high-water mark, a USART1 loopback injector (PA9 jumpered to PA10),
 *   PWM measured by the chip itself (TIM2_CH1 input capture on PA15), an
 *   I2C stress test against an MPU-6050, and fault/watchdog triggers. All
 *   of it is driven from single-key console commands (see bench.c) and the
 *   laptop script tools/hil/hil_rtos.py.
 *
 *   Nothing in the bench layer acts until a console key asks it to. Two of
 *   those keys drive the ESC outputs away from neutral (P: PWM signature)
 *   or stop the control task (W). A BENCH_HIL build must never be flashed
 *   to a vehicle with ESCs and props attached; the boot banner says so.
 *
 * 0: none of it is compiled. Use -DBENCH_HIL=0 for the vehicle.
 */
#ifndef BENCH_HIL
#define BENCH_HIL 1
#endif

#endif /* BENCH_CONFIG_H */
