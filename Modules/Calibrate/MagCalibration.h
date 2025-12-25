#ifndef __MAG_CALIBRATION_H
#define __MAG_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define SIDE_NUM 2
#define MAG_SPHERE_RADIUS (350)
#define MAG_SAMPLE_NUM 1000
#define ROTATE_SPEED 5
#define MAX_POINT_NUM_PER_SIDE (MAG_SAMPLE_NUM/SIDE_NUM)

enum MAG_CALI_STEP
{
    PREPARE = 0,
    RIGHTSIDE_UP,     
    NOSE_UP,                   
    FINISH      
};

bool mag_sample_collect(float gyro[3], float accel[3], float mag_raw[3], float sample[3][MAG_SAMPLE_NUM], 
                        uint16_t sample_num, uint16_t *collected_num, enum MAG_CALI_STEP *step);
uint8_t detect_orientation(float ax, float ay, float az);
void mag_calibration_init();
void do_mag_calibration(float accel[3], float gyro[3], float mag[3]);
#endif

