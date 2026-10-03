#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "bench_config.h"

/*-----------------------------------------------------------
 * System configuration
 *----------------------------------------------------------*/

/*
 * Literal, deliberately NOT the SystemCoreClock variable.
 *
 * port.c reads this once, in xPortStartScheduler, to compute the SysTick
 * reload. Routing that through a runtime variable means the tick rate
 * depends on something having called SystemCoreClockUpdate() after the
 * clock switch, and on SystemCoreClockUpdate() re-deriving the right
 * answer from HSE_VALUE and PLLCFGR. Two independent ways to silently
 * desync SysTick from the hardware; a literal has none.
 *
 * The invariant that makes a literal safe: every code path that reaches
 * vTaskStartScheduler() is running at 180 MHz. system_clock_init() either
 * achieves 180 MHz (from HSE, or from the HSI PLL fallback) or returns a
 * fatal status, and main halts in the LD2 fault blink without starting
 * the scheduler. See clock_status_is_fatal().
 */
#define configCPU_CLOCK_HZ                     (180000000UL)
#define configTICK_RATE_HZ                     ((TickType_t)1000)

#define configTICK_TYPE_WIDTH_IN_BITS          TICK_TYPE_WIDTH_32_BITS

#define configUSE_PREEMPTION                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                0

#define configMAX_PRIORITIES                   8
#define configMINIMAL_STACK_SIZE               ((unsigned short)128)
#define configTOTAL_HEAP_SIZE                  ((size_t)(22 * 1024))

#define configMAX_TASK_NAME_LEN                16
#define configIDLE_SHOULD_YIELD               1
#define configUSE_TIME_SLICING                 1

/*-----------------------------------------------------------
 * Synchronization
 *----------------------------------------------------------*/

#define configUSE_MUTEXES                      1
#define configUSE_RECURSIVE_MUTEXES            0
#define configUSE_COUNTING_SEMAPHORES           1

#define configQUEUE_REGISTRY_SIZE              8
#define configUSE_QUEUE_SETS                   0

/*-----------------------------------------------------------
 * Hooks / debugging
 *----------------------------------------------------------*/

#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    0

#define configCHECK_FOR_STACK_OVERFLOW         2
#define configUSE_MALLOC_FAILED_HOOK           1

#define configUSE_APPLICATION_TASK_TAG         0
/*
 * Run-time stats only in the bench build, where hil_rtos.py reads per-task
 * CPU load. The counter is TIM2, already free-running at 1 MHz from main()
 * step 8, so there is nothing to configure; CNT is read by address because
 * this header is pulled into every kernel translation unit. The extra
 * 4-byte TCB field does not change the heap ledger: an 84 B TCB and an
 * 88 B TCB both round to a 96 B heap_4 block.
 */
#if BENCH_HIL
#define configGENERATE_RUN_TIME_STATS          1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()       (*(volatile uint32_t *)0x40000024UL)  /* TIM2->CNT */
#else
#define configGENERATE_RUN_TIME_STATS          0
#endif

/*-----------------------------------------------------------
 * Software timers
 *----------------------------------------------------------*/

/*
 * Off: xTimerCreate is called zero times in this codebase. Leaving it on
 * cost a 128-word daemon task, its TCB and a 10-deep queue - 816 bytes of
 * heap - and pinned priority 2, colliding with logging_task (D12).
 */
#define configUSE_TIMERS                       0

/*-----------------------------------------------------------
 * Co-routines
 *----------------------------------------------------------*/

#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        2

/*-----------------------------------------------------------
 * Memory allocation
 *----------------------------------------------------------*/

/*
 * Left 0. newlib reentrancy is not needed because stdio is confined to a
 * single task: dummy_task owns every printf after the scheduler starts,
 * and main owns every printf before it. Turning this on would add a
 * struct _reent (~96 bytes) to every task's TCB for no benefit. If a
 * second task ever calls printf, this must become 1.
 */
#define configUSE_NEWLIB_REENTRANT              0

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/*-----------------------------------------------------------
 * Cortex-M4 interrupt priority configuration
 *
 * STM32F446RE:
 * __NVIC_PRIO_BITS = 4
 * Therefore 16 interrupt priority levels exist: 0-15.
 *----------------------------------------------------------*/

#define configPRIO_BITS                         4

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*-----------------------------------------------------------
 * Newer FreeRTOS kernel configuration
 *----------------------------------------------------------*/

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/*-----------------------------------------------------------
 * API functions
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1

#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    1

#if BENCH_HIL
#define INCLUDE_xTaskGetIdleTaskHandle         1   /* CPU load report       */
#else
#define INCLUDE_xTaskGetIdleTaskHandle         0
#endif
#define INCLUDE_eTaskGetState                  1

#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0

/*-----------------------------------------------------------
 * FreeRTOS exception handler mapping
 *
 * The startup vector expects the standard CMSIS names.
 * The actual mapping becomes relevant when we hand the
 * handlers over to FreeRTOS later.
 *----------------------------------------------------------*/

#define vPortSVCHandler       SVC_Handler
#define xPortPendSVHandler    PendSV_Handler
#define xPortSysTickHandler   SysTick_Handler

/*-----------------------------------------------------------
 * Assertions
 *
 * A failing configASSERT latches file, line and the caller's PC into a
 * .noinit struct that survives a warm reset, then breaks and halts. See
 * Core/Inc/fault_latch.h. Declared rather than included because this
 * header is pulled into every kernel translation unit.
 *----------------------------------------------------------*/

void fault_assert_failed(const char *file, uint32_t line) __attribute__((noreturn));


#define configASSERT(x)                                      \
    do                                                       \
    {                                                        \
        if ((x) == 0)                                        \
        {                                                    \
            fault_assert_failed(__FILE__, __LINE__);         \
        }                                                    \
    } while (0)

#endif /* FREERTOS_CONFIG_H */
