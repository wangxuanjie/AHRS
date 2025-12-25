#ifndef __IST8310_H
#define __IST8310_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#pragma pack(push, 1)
struct ist8310_t
{
    float mx;
    float my;
    float mz;
    float mx_cali;
    float my_cali;
    float mz_cali;
    int16_t mx_raw;
    int16_t my_raw;
    int16_t mz_raw;     
    float temp;
};
#pragma pack(pop)

bool ist8310_init(void);
void ist8310_update(void);
void get_ist8310(struct ist8310_t *data);
#endif
