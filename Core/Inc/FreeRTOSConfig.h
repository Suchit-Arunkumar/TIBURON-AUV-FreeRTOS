#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/*-----------------------------------------------------------
 * System configuration
 *----------------------------------------------------------*/

extern uint32_t SystemCoreClock;

#define configCPU_CLOCK_HZ                     (SystemCoreClock)
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
#define configGENERATE_RUN_TIME_STATS          0

/*-----------------------------------------------------------
 * Software timers
 *----------------------------------------------------------*/

#define configUSE_TIMERS                       1
#define configTIMER_TASK_PRIORITY              2
#define configTIMER_QUEUE_LENGTH               10
#define configTIMER_TASK_STACK_DEPTH           configMINIMAL_STACK_SIZE

/*-----------------------------------------------------------
 * Co-routines
 *----------------------------------------------------------*/

#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        2

/*-----------------------------------------------------------
 * Memory allocation
 *----------------------------------------------------------*/

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

#define INCLUDE_xTaskGetIdleTaskHandle         0
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
 *----------------------------------------------------------*/

#define configASSERT(x)                                      \
    do                                                       \
    {                                                        \
        if ((x) == 0)                                        \
        {                                                    \
            taskDISABLE_INTERRUPTS();                       \
            for (;;)                                         \
            {                                                \
            }                                                \
        }                                                    \
    } while (0)

#endif /* FREERTOS_CONFIG_H */
