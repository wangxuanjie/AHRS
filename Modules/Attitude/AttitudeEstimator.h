#ifndef __ATTITUDEESTIMATOR_H
#define __ATTITUDEESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "eigen"

void attitude_estimator_update(float gyro[3], float accel[3], float mag[3], float dt);
void attitude_estimator_init(float gyro[3], float accel[3], float mag[3]);
void get_euler(float euler[3]);
void get_gyro_bias(float bias[3]);
Eigen::Quaternionf get_quaternion(void);
void get_accel_filter(float data[3]);
void attitude_fast_align();
void attitude_fast_align_mag_cali();
void heading_reset_check();
void bias_fast_tracking(float acc[3], float gyro[3], float acc_filt[3]);
#endif

