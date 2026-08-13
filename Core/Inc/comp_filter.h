#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <stdbool.h>

#include "vn200.h"
#include "dvl.h"


/*
 * Z-depth complementary blend.
 *
 * Higher alpha:
 *     trust IMU prediction more.
 *
 * Lower alpha:
 *     trust Bar30 more.
 */
#define COMP_FILTER_Z_ALPHA       0.90f


/*
 * XY DVL correction gain.
 *
 * This is intentionally separate from the Z alpha because
 * DVL correction is a velocity/position correction mechanism,
 * not the same complementary blend as the Bar30 depth update.
 */
#define COMP_FILTER_DVL_GAIN      0.50f


#define COMP_FILTER_GRAVITY       9.80665f


typedef struct
{
    /*
     * Estimated position.
     */
    float x;
    float y;
    float z;


    /*
     * Estimated velocity.
     */
    float vx;
    float vy;
    float vz;


    /*
     * VN-200 reported YPR.
     *
     * These are passed through untouched by the filter.
     */
    float roll;
    float pitch;
    float yaw;


    bool attitude_valid;
    bool velocity_valid;

} StateEstimate;


/*
 * Initialize filter state.
 */
void comp_filter_init(void);


/*
 * Update Z state.
 *
 * az_world:
 *     Gravity-compensated world-frame Z acceleration.
 *
 * depth:
 *     Latest Bar30 depth.
 *
 * depth_new:
 *     true only when a fresh Bar30 measurement arrived.
 *
 * dt:
 *     Actual filter task timestep in seconds.
 */
void comp_filter_z_update(
    float az_world,
    float depth,
    bool depth_new,
    float dt
);


/*
 * Update XY state.
 *
 * ax_world / ay_world:
 *     Gravity-compensated world-frame acceleration.
 *
 * dvl_vx / dvl_vy:
 *     Latest DVL velocity.
 *
 * dvl_new:
 *     true only when a fresh DVL measurement arrived.
 *
 * dt:
 *     Actual filter task timestep in seconds.
 */
void comp_filter_xy_update(
    float ax_world,
    float ay_world,
    float dvl_vx,
    float dvl_vy,
    bool dvl_new,
    float dt
);


/*
 * Top-level filter update.
 *
 * VN-200 YPR is copied directly into StateEstimate.
 *
 * No attitude estimation is performed here.
 */
bool comp_filter_update(
    const VN200Data *vn200,
    const DVLData *dvl,
    float depth,
    bool depth_new,
    bool dvl_new,
    float dt,
    StateEstimate *out
);

#endif
