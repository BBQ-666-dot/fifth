#include "AHRS.h"
#include <math.h>

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    (void)mag;
    const fp32 roll = atan2f(accel[1], accel[2]);
    const fp32 pitch = atan2f(-accel[0],
                             sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
    const fp32 half_roll = 0.5f * roll;
    const fp32 half_pitch = 0.5f * pitch;

    quat[0] = cosf(half_roll) * cosf(half_pitch);
    quat[1] = sinf(half_roll) * cosf(half_pitch);
    quat[2] = cosf(half_roll) * sinf(half_pitch);
    quat[3] = -sinf(half_roll) * sinf(half_pitch);
}
