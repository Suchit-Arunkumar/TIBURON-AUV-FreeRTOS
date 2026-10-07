/*
 * Host test for the analog depth conversion (Devices/adc_depth/adc_depth.h),
 * against the DFRobot SEN0257 figures: 0.5-4.5 V for 0-1 MPa.
 */
#include "test.h"
#include "adc_depth.h"

int main(void)
{
    /* The ends of the range. */
    CHECK_NEAR(adc_depth_pressure_pa(0.5f, 0.5f), 0.0, 1e-3);
    CHECK_NEAR(adc_depth_pressure_pa(4.5f, 0.5f), 1.0e6, 1.0);

    /* DFRobot's own example: offset 0.483 V; 1.483 V is 250 kPa. */
    CHECK_NEAR(adc_depth_pressure_pa(1.483f, 0.483f), 250000.0, 1.0);

    /* 1 m of fresh water is 997 x 9.80665 = 9777 Pa. */
    CHECK_NEAR(adc_depth_from_pa(9777.23f), 1.0, 1e-4);

    /* The Pico sketch's 400 kPa/V would have said 1.6 m here. */
    float v_1m = 0.5f + 9777.23f / 250000.0f;
    CHECK_NEAR(adc_depth_from_pa(adc_depth_pressure_pa(v_1m, 0.5f)), 1.0, 1e-3);

    /* Worst case at the pin through the 2:1 divider stays under 3.3 V. */
    CHECK(4.5f / ADC_DEPTH_DIVIDER < 3.3f);

    return test_summary("adc_depth");
}
