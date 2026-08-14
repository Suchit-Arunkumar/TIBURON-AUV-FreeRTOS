
#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "timer_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "comp_filter.h"

#define N_DOF           6
#define N_THR           8

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
#define PWM_RAMP_STEP   50      /* µs per 50 Hz tick → 2500 µs/s slew rate  */
#define CMD_TIMEOUT_MS  500u    /* ms without a valid CMD before failsafe     */

extern volatile uint32_t g_tick;
extern bool link_ok;
extern QueueHandle_t stateQueue;

void control_loop_get_pwm(uint16_t *out, uint8_t len);
void control_loop_get_pose(float out[N_DOF]);
bool control_loop_get_armed(void);
bool control_loop_get_link(void);

void control_loop_init(void);

void control_loop_tick(void);

void state_update(const StateEstimate *state);
void target_update(const float new_target[N_DOF], bool arm_flag);

void enterFailsafe(void);

void checkCommandTimeout(void);

void computePID(float dt);

void computeAllocation(void);

void applyPWM(void);

extern TaskHandle_t controlTaskHandle;
void control_task(void *argument);

#endif /* __CONTROL_LOOP_H */
