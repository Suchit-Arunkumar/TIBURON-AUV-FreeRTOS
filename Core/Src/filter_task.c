#include "filter_task.h"

#include "bar30.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>


/*
 * Filter task runs at 100 Hz.
 *
 * The task does not wait for slower sensors.
 */
#define FILTER_TASK_PERIOD_MS     10U


void filter_task(void *argument)
{
    (void)argument;


    VN200Data vn200_data;
    DVLData dvl_data;

    float depth = 0.0f;


    /*
     * Cached sensor validity.
     */
    bool have_vn200 = false;
    bool have_dvl = false;
    bool have_depth = false;


    /*
     * Freshness flags.
     *
     * These are set only when xQueueReceive() actually
     * obtains a new sensor measurement.
     */
    bool dvl_new;
    bool depth_new;


    StateEstimate state;


    comp_filter_init();


    TickType_t last_wake =
        xTaskGetTickCount();


    TickType_t previous_tick =
        last_wake;


    while (1)
    {
        /*
         * Maintain a fixed filter execution period.
         */
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(FILTER_TASK_PERIOD_MS)
        );


        /*
         * Calculate actual elapsed filter time.
         */
        TickType_t current_tick =
            xTaskGetTickCount();


        float dt =
            (float)(current_tick - previous_tick)
            / (float)configTICK_RATE_HZ;


        previous_tick = current_tick;


        /*
         * Prevent an invalid/zero timestep.
         */
        if (dt <= 0.0f)
        {
            dt =
                (float)FILTER_TASK_PERIOD_MS
                / 1000.0f;
        }


        /*
         * -----------------------------------------------------
         * VN-200
         * -----------------------------------------------------
         *
         * Non-blocking.
         *
         * If no new packet arrived, retain the latest VN-200
         * measurement.
         */
        if (vn200Queue != NULL)
        {
            VN200Data new_vn200;


            if (xQueueReceive(
                    vn200Queue,
                    &new_vn200,
                    0
                ) == pdPASS)
            {
                vn200_data = new_vn200;
                have_vn200 = true;
            }
        }


        /*
         * -----------------------------------------------------
         * DVL
         * -----------------------------------------------------
         *
         * Non-blocking.
         *
         * Only a successful receive sets dvl_new.
         */
        dvl_new = false;


        if (dvlQueue != NULL)
        {
            DVLData new_dvl;


            if (xQueueReceive(
                    dvlQueue,
                    &new_dvl,
                    0
                ) == pdPASS)
            {
                dvl_data = new_dvl;
                have_dvl = true;
                dvl_new = true;
            }
        }


        /*
         * -----------------------------------------------------
         * BAR30
         * -----------------------------------------------------
         *
         * Non-blocking.
         *
         * Only a successful receive sets depth_new.
         */
        depth_new = false;


        if (bar30Queue != NULL)
        {
            float new_depth;


            if (xQueueReceive(
                    bar30Queue,
                    &new_depth,
                    0
                ) == pdPASS)
            {
                depth = new_depth;
                have_depth = true;
                depth_new = true;
            }
        }


        /*
         * The filter cannot operate until we have a valid
         * VN-200 measurement.
         *
         * DVL and Bar30 may temporarily be unavailable.
         */
        if (!have_vn200)
        {
            continue;
        }


        /*
         * If DVL has not arrived yet, provide a zero
         * velocity structure.
         *
         * dvl_new remains false, so no DVL correction occurs.
         */
        if (!have_dvl)
        {
            dvl_data.vx = 0.0f;
            dvl_data.vy = 0.0f;
            dvl_data.vz = 0.0f;
            dvl_data.valid = false;
        }


        /*
         * If Bar30 has not arrived yet, no depth correction
         * occurs because depth_new is false.
         */
        if (!have_depth)
        {
            depth = 0.0f;
        }


        /*
         * -----------------------------------------------------
         * FILTER
         * -----------------------------------------------------
         *
         * comp_filter_update() performs:
         *
         * 1. body -> world rotation
         * 2. gravity compensation
         * 3. Z acceleration integration
         * 4. Bar30 correction when fresh
         * 5. XY acceleration integration
         * 6. DVL correction when fresh
         * 7. VN-200 YPR pass-through
         */
        comp_filter_update(
            &vn200_data,
            &dvl_data,
            depth,
            depth_new,
            dvl_new,
            dt,
            &state
        );


        /*
         * Phase 6 ends here.
         *
         * DO NOT send state to the control task yet.
         *
         * Filter -> Control wiring belongs to Phase 7.
         */
    }
}
