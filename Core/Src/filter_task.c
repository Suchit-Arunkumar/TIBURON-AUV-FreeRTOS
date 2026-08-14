#include "filter_task.h"

#include "bar30.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/* Filter runs faster than the 50 Hz control loop so control always
 * has a fresh estimate available when it wakes. */
#define FILTER_TASK_PERIOD_MS     10U

QueueHandle_t stateQueue = NULL;


void filter_task(void *argument)
{
    (void)argument;

    VN200Data vn200_data;
    DVLData dvl_data;
    StateEstimate state;

    float depth = 0.0f;

    bool have_vn200 = false;
    bool have_dvl = false;
    bool have_depth = false;

    bool dvl_new;
    bool depth_new;

    comp_filter_init();

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t previous_tick = last_wake;

    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FILTER_TASK_PERIOD_MS));

        /* Use actual elapsed time rather than the nominal period —
         * absorbs scheduling jitter instead of feeding it into the filter. */
        TickType_t current_tick = xTaskGetTickCount();
        float dt = (float)(current_tick - previous_tick) / (float)configTICK_RATE_HZ;
        previous_tick = current_tick;

        if (dt <= 0.0f)
        {
            dt = (float)FILTER_TASK_PERIOD_MS / 1000.0f;
        }

        /* VN-200: retain last valid reading if nothing new arrived. */
        if (vn200Queue != NULL)
        {
            VN200Data new_vn200;

            if (xQueueReceive(vn200Queue, &new_vn200, 0) == pdPASS)
            {
                vn200_data = new_vn200;
                have_vn200 = true;
            }
        }

        /* DVL: dvl_new gates the correction step in comp_filter_update(). */
        dvl_new = false;

        if (dvlQueue != NULL)
        {
            DVLData new_dvl;

            if (xQueueReceive(dvlQueue, &new_dvl, 0) == pdPASS)
            {
                dvl_data = new_dvl;
                have_dvl = true;
                dvl_new = true;
            }
        }

        /* Bar30: depth_new gates the Z correction step. */
        depth_new = false;

        if (bar30Queue != NULL)
        {
            float new_depth;

            if (xQueueReceive(bar30Queue, &new_depth, 0) == pdPASS)
            {
                depth = new_depth;
                have_depth = true;
                depth_new = true;
            }
        }

        /* Attitude is required to run the filter at all; velocity/depth
         * corrections are optional and simply stay disabled until fresh. */
        if (!have_vn200)
        {
            continue;
        }

        if (!have_dvl)
        {
            dvl_data.vx = 0.0f;
            dvl_data.vy = 0.0f;
            dvl_data.vz = 0.0f;
            dvl_data.valid = false;
        }

        if (!have_depth)
        {
            depth = 0.0f;
        }

        comp_filter_update(
            &vn200_data,
            &dvl_data,
            depth,
            depth_new,
            dvl_new,
            dt,
            &state
        );

        /* Overwrite semantics: control only ever needs the latest
         * estimate, never a backlog. */
        if (stateQueue != NULL)
        {
            xQueueOverwrite(stateQueue, &state);
        }
    }
}
