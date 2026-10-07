// Fault handlers and the TIM7 control tick. SysTick, SVC and PendSV belong
// to the FreeRTOS port.

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

void NMI_Handler(void)
{
  pwm_fault_neutral();

  while (1)
  {
  }
}

/*
 * CPU faults: thrusters to neutral first, then save the fault and where it
 * happened in the fault latch and halt (the watchdog then resets the board).
 * Before this the handlers just looped, leaving the thrusters running.
 *
 * MemManage/BusFault/UsageFault aren't enabled, so they arrive as a
 * HardFault, but share the entry anyway. The entry is naked so it can pick
 * the stack the faulting code used (LR bit 2: 0 = MSP, 1 = PSP); the
 * hardware saved r0-r3, r12, lr, pc, xpsr there, so frame[6] is the PC.
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
    // Before touching the frame: if the stack is bad, reading it can fault
    // again and lock the core up with the timers still running.
    pwm_fault_neutral();

    // CFSR: what kind of fault. HFSR: whether it escalated.
    char detail[] = "C=00000000 H=00000000";
    hex8(&detail[2],  SCB->CFSR);
    hex8(&detail[13], SCB->HFSR);

    fault_latch_fail(FAULT_HARDFAULT, __FILE__, 0U, frame[6], detail);
}

__attribute__((naked)) void HardFault_Handler(void)  { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void MemManage_Handler(void)  { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void BusFault_Handler(void)   { __asm volatile (FAULT_ENTRY_ASM); }
__attribute__((naked)) void UsageFault_Handler(void) { __asm volatile (FAULT_ENTRY_ASM); }

void DebugMon_Handler(void)
{
}

// 50 Hz: only wakes control_task.
void TIM7_IRQHandler(void)
{
    bench_tim7_isr();

    TIM7->SR &= ~TIM_SR_UIF;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(
        controlTaskHandle,
        &xHigherPriorityTaskWoken
    );

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
