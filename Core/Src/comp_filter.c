#include "comp_filter.h"

#include <math.h>
#include <string.h>


typedef struct
{
    float x;
    float y;
    float z;

    float vx;
    float vy;
    float vz;

    /*
     * DVL velocity-integrated reference position.
     *
     * This is maintained independently from the IMU
     * dead-reckoned position.
     */
    float dvl_x;
    float dvl_y;

} FilterState;


static FilterState g_state;


/*
 * VN-200 YPR is reported in degrees.
 *
 * Convert to radians only for the trigonometric rotation.
 */
static float deg_to_rad(float degrees)
{
    return degrees * 0.01745329251994329577f;
}


/*
 * Rotate body-frame acceleration into navigation/world frame.
 *
 * Only roll and pitch are required for gravity compensation.
 * Yaw is intentionally not used.
 *
 * The attitude itself is NOT modified.
 */
static void rotate_body_to_world(
    float ax_body,
    float ay_body,
    float az_body,
    float roll_deg,
    float pitch_deg,
    float *ax_world,
    float *ay_world,
    float *az_world
)
{
    float roll = deg_to_rad(roll_deg);
    float pitch = deg_to_rad(pitch_deg);

    float cr = cosf(roll);
    float sr = sinf(roll);

    float cp = cosf(pitch);
    float sp = sinf(pitch);


    /*
     * Body -> world rotation using roll and pitch.
     */
    *ax_world =
        cp * ax_body +
        sr * sp * ay_body +
        cr * sp * az_body;


    *ay_world =
        cr * ay_body -
        sr * az_body;


    *az_world =
        -sp * ax_body +
        sr * cp * ay_body +
        cr * cp * az_body;
}


/*
 * Gravity compensation.
 *
 * First rotate acceleration into the world frame.
 * Then remove gravity from world Z.
 */
static void gravity_compensate(
    const VN200Data *vn200,
    float *ax_world,
    float *ay_world,
    float *az_world
)
{
    rotate_body_to_world(
        vn200->accel_x,
        vn200->accel_y,
        vn200->accel_z,
        vn200->roll,
        vn200->pitch,
        ax_world,
        ay_world,
        az_world
    );


    /*
     * Remove gravity from world-frame Z.
     */
    *az_world -= COMP_FILTER_GRAVITY;
}


void comp_filter_init(void)
{
    memset(
        &g_state,
        0,
        sizeof(g_state)
    );
}


void comp_filter_z_update(
    float az_world,
    float depth,
    bool depth_new,
    float dt
)
{
    /*
     * ---------------------------------------------------------
     * PREDICTION
     * ---------------------------------------------------------
     *
     * Acceleration -> velocity
     * Velocity     -> position
     */
    g_state.vz += az_world * dt;

    g_state.z += g_state.vz * dt;


    /*
     * ---------------------------------------------------------
     * BAR30 CORRECTION
     * ---------------------------------------------------------
     *
     * Only correct when a NEW depth value arrived.
     */
    if (depth_new)
    {
        float predicted_z = g_state.z;


        g_state.z =
            (COMP_FILTER_Z_ALPHA * predicted_z) +
            ((1.0f - COMP_FILTER_Z_ALPHA) * depth);


        /*
         * Explicit re-sync decision:
         *
         * Once the position is corrected by Bar30,
         * discard the accumulated Z velocity so that
         * the old integrated velocity does not immediately
         * drive the corrected position away again.
         */
        g_state.vz = 0.0f;
    }
}


void comp_filter_xy_update(
    float ax_world,
    float ay_world,
    float dvl_vx,
    float dvl_vy,
    bool dvl_new,
    float dt
)
{
    /*
     * ---------------------------------------------------------
     * IMU DEAD RECKONING
     * ---------------------------------------------------------
     */
    g_state.vx += ax_world * dt;
    g_state.vy += ay_world * dt;


    g_state.x += g_state.vx * dt;
    g_state.y += g_state.vy * dt;


    /*
     * ---------------------------------------------------------
     * DVL VELOCITY REFERENCE
     * ---------------------------------------------------------
     *
     * Integrate DVL velocity into an independent XY
     * position reference.
     *
     * This reference is only advanced when a fresh DVL
     * measurement arrives.
     */
    if (dvl_new)
    {
        g_state.dvl_x += dvl_vx * dt;
        g_state.dvl_y += dvl_vy * dt;


        /*
         * Correct velocity toward DVL velocity.
         */
        g_state.vx +=
            COMP_FILTER_DVL_GAIN *
            (dvl_vx - g_state.vx);


        g_state.vy +=
            COMP_FILTER_DVL_GAIN *
            (dvl_vy - g_state.vy);


        /*
         * Correct accumulated XY position toward the
         * DVL-velocity-integrated reference.
         */
        g_state.x +=
            COMP_FILTER_DVL_GAIN *
            (g_state.dvl_x - g_state.x);


        g_state.y +=
            COMP_FILTER_DVL_GAIN *
            (g_state.dvl_y - g_state.y);
    }
}


bool comp_filter_update(
    const VN200Data *vn200,
    const DVLData *dvl,
    float depth,
    bool depth_new,
    bool dvl_new,
    float dt,
    StateEstimate *out
)
{
    if (vn200 == NULL ||
        dvl == NULL ||
        out == NULL)
    {
        return false;
    }


    /*
     * ---------------------------------------------------------
     * GRAVITY COMPENSATION
     * ---------------------------------------------------------
     */
    float ax_world;
    float ay_world;
    float az_world;


    gravity_compensate(
        vn200,
        &ax_world,
        &ay_world,
        &az_world
    );


    /*
     * ---------------------------------------------------------
     * Z UPDATE
     * ---------------------------------------------------------
     */
    comp_filter_z_update(
        az_world,
        depth,
        depth_new,
        dt
    );


    /*
     * ---------------------------------------------------------
     * XY UPDATE
     * ---------------------------------------------------------
     */
    comp_filter_xy_update(
        ax_world,
        ay_world,
        dvl->vx,
        dvl->vy,
        dvl_new,
        dt
    );


    /*
     * ---------------------------------------------------------
     * OUTPUT
     * ---------------------------------------------------------
     */
    memset(
        out,
        0,
        sizeof(StateEstimate)
    );


    /*
     * Estimated position.
     */
    out->x = g_state.x;
    out->y = g_state.y;
    out->z = g_state.z;


    /*
     * Estimated velocity.
     */
    out->vx = g_state.vx;
    out->vy = g_state.vy;
    out->vz = g_state.vz;


    /*
     * VN-200 YPR pass-through.
     *
     * No filtering.
     * No recalculation.
     */
    out->roll = vn200->roll;
    out->pitch = vn200->pitch;
    out->yaw = vn200->yaw;


    out->attitude_valid =
        vn200->valid;


    out->velocity_valid =
        dvl->valid;


    return out->attitude_valid;
}
