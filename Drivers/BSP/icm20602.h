#ifndef __ICM20602_H
#define __ICM20602_H
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#pragma pack(push, 1)
struct icm20602_t
{
    float ax;//unit g
    float ay;
    float az;
    float gx;//unit rad/s
    float gy;
    float gz;  
    float ax_cali;//unit g
    float ay_cali;
    float az_cali;
    float gx_cali;//unit rad/s
    float gy_cali;
    float gz_cali;      
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    float temp;
};
#pragma pack(pop)

bool icm20602_init(void);
void icm20602_update(void);
void get_icm20602_accel_gyro(struct icm20602_t *data);

#endif
