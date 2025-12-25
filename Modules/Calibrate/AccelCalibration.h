#ifndef __ACCEL_CALIBRATION_H
#define __ACCEL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdbool.h>
#include <stdint.h>

bool collect_six_point(float accelx, float accely, float accelz);
void get_accel_sample(float sample[3][6]);
void do_gyro_calibration(float gyro[3]);
void do_accel_calibration(float accel[3]);
#ifdef __cplusplus
}
#endif

#endif
