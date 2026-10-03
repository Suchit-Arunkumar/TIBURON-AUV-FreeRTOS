#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>
#include <stdbool.h>
#include "bench_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "struct.h"

#if BENCH_HIL

/* main(), after timer2_timebase_init(): DWT cycle counter + PA15 capture. */
void bench_init(void);
/* main(), after each xTaskCreate: lets the CPU-load report name the task. */
void bench_register_task(const char *short_name, TaskHandle_t handle);

/* TIM7_IRQHandler, first statement. */
void bench_tim7_isr(void);

/* control_task */
void bench_control_wake(void);
void bench_cmd_consumed(const CommandPayload *cmd);
void bench_after_control_tick(void);       /* PWM signature, hang request */
void bench_control_done(void);
void bench_failsafe_entered(uint32_t ms_since_last_cmd);

/* comms_task */
void bench_cmd_parsed(CommandPayload *cmd);  /* stamps cmd->reserved */
bool bench_comms_skip_telemetry(void);
void bench_comms_after_tx(void);

/* packet.c, comms_task context: a CRC-valid TELEMETRY frame came back
 * through the PA9->PA10 loopback. */
void bench_telemetry_looped(const uint8_t *payload);

/* uart_packet.c, ISR context */
void bench_uart1_rx_bytes(uint16_t n, bool from_dma_event);
bool bench_uart1_dma_events_ignored(void);

/* dummy_task */
void bench_dummy_first_run(void);          /* paints the MSP */
bool bench_console_key(char c);            /* true if the key was a bench key */
void bench_print_help(void);

/* bar30_task: the I2C bus owner runs bench I2C work in its own context. */
void bench_i2c_service(void);

#else  /* !BENCH_HIL */

#define bench_init()                     ((void)0)
#define bench_register_task(n, h)        ((void)0)
#define bench_tim7_isr()                 ((void)0)
#define bench_control_wake()             ((void)0)
#define bench_cmd_consumed(c)            ((void)0)
#define bench_after_control_tick()       ((void)0)
#define bench_control_done()             ((void)0)
#define bench_failsafe_entered(ms)       ((void)0)
#define bench_cmd_parsed(c)              ((void)0)
#define bench_comms_skip_telemetry()     (false)
#define bench_comms_after_tx()           ((void)0)
#define bench_telemetry_looped(p)        ((void)0)
#define bench_uart1_rx_bytes(n, d)       ((void)0)
#define bench_uart1_dma_events_ignored() (false)
#define bench_dummy_first_run()          ((void)0)
#define bench_console_key(c)             (false)
#define bench_print_help()               ((void)0)
#define bench_i2c_service()              ((void)0)

#endif /* BENCH_HIL */

#endif /* BENCH_H */
