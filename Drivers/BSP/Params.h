#ifndef __PARAMS_H
#define __PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdbool.h>
#include <stdint.h>

#define STM32_FLASH_BASE        FLASH_BASE  //0x08000000UL
#define STM_SECTOR_SIZE         1024
#define STM32_FLASH_SIZE 	    1024 //1024KB

#define FLASH_WAITETIME         50000

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//Sector 0 start address,16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//Sector 1 start address,16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//Sector 2 start address,16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//Sector 3 start address,16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//Sector 4 start address,64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//Sector 5 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//Sector 6 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//Sector 7 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//Sector 8 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//Sector 9 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//Sector 10 start address,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//Sector 11 start address,128 Kbytes 

//Last sector
#define PARAMS_BASE             (FLASH_BASE + (STM32_FLASH_SIZE-1)*STM_SECTOR_SIZE)

#pragma pack(push, 1)
struct sensor_params_t
{
    uint8_t acc_calibrated;
    uint8_t gyro_calibrated;
    uint8_t mag_calibrated;
    float acc_scale[3];
    float acc_offset[3];
    float gyro_offset[3];
    float mag_scale[3];
    float mag_offset[3];
    float mag_offdiag[3];
    float mag_radius;
};
#pragma pack(pop)

extern struct sensor_params_t _params;

void param_init(void);
void param_write();
void param_read();

#ifdef __cplusplus
}
#endif

#endif
