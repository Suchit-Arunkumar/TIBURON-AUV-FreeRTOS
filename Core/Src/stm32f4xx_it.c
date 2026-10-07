/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "system_init.h"
#include "stm32f446xx.h"
#include "ring_buffer.h"
#include "uart_packet.h"
#include "control_loop.h"
#include "timer_basic.h"
#include "sd_logger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bench.h"
#include "fault_latch.h"
#include "timer_pwm.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* Nothing in this design raises an NMI (the clock security system is
   * off), but if one arrives, stop the thrusters before halting. */
  pwm_fault_neutral();

  while (1)
  {
  }
}

/*
 * CPU faults.
 *
 * Before this, all four handlers were bare spin loops: the board froze
 * with TIM3/TIM8 still producing whatever pulse widths were last set, so
 * a crash mid-manoeuvre left the thrusters running. Now every path sets
 * neutral first, then records the fault in the .noinit latch so the next
 * boot prints where it happened.
 *
 * MemManage, BusFault and UsageFault are not enabled in SCB->SHCSR, so
 * they escalate and arrive here as a HardFault. They share the same entry
 * in case they are enabled later.
 *
 * The entry is naked (no compiler prologue) so it can look at the stack
 * the faulting code was using. Bit 2 of EXC_RETURN in LR says which one:
 * 0 = MSP (an ISR faulted), 1 = PSP (a task faulted). The hardware pushed
 * r0-r3, r12, lr, pc, xpsr there, so frame[6] is the faulting PC.
 */
#define FAULT_ENTRY_ASM          \
    "tst   lr, #4          \n"   \
    "ite   eq              \n"   \
    "mrseq r0, msp         \n"   \
    "mrsne r0, psp         \n"   \
    "b     fault_from_frame\n"

static void hex8(char *out, uint32_t v)
{
    static const char digits[] = "0123456789ABCDEF";

    for (int i = 7; i >= 0; i--)
    {
        out[i] = digits[v & 0xFU];
        v >>= 4;
    }
}

void fault_from_frame(const uint32_t *frame) __attribute__((used, noreturn));
void fault_from_frame(const uint32_t *frame)
{
    /*
     * Neutral before touching the frame. If the fault came from a blown
     * stack, reading it can fault again, and a fault inside the HardFault
     * handler locks the core up - with the timers still running.
     */
    pwm_fault_neutral();

    /* CFSR says what kind of fault, HFSR whether it escalated. */
    char detail[] = "C=00000000 H=00000000";
    hex8(&detail[2],  SCB->CFSR);
    hex8(&detail[13], SCB->HFSR);

    fault_latch_fail(FAULT_HARDFAULT, __FILE__, 0U, frame[6], detail);
}

__attribute__((naked)) void HardFault_Handler(void)  { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void MemManage_Handler(void)  { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void BusFault_Handler(void)   { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void UsageFault_Handler(void) { __asm volatile (FAULT_ENTRY_ASM); }

/**
  * @brief This function handles System service call via SWI instruction.
  */

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */


/**
  * @brief This function handles System tick timer.
  */


/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void TIM7_IRQHandler(void)
{
    bench_tim7_isr();      /* DWT stamp; compiles to nothing without BENCH_HIL */

    // Clear TIM7 update interrupt flag
    TIM7->SR &= ~TIM_SR_UIF;

    // Tell FreeRTOS whether a higher-priority task was woken
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake the Control Task
    vTaskNotifyGiveFromISR(
        controlTaskHandle,
        &xHigherPriorityTaskWoken
    );

    // Switch to the woken task if appropriate
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* USER CODE END 1 */
