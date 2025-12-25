#include <string.h>
#include "stm32f4xx_hal.h"
#include "Params.h"

extern void FLASH_PageErase(uint32_t PageAddress); 

static uint32_t _flash_buffer[STM_SECTOR_SIZE/4]; 

struct sensor_params_t _params = {};
uint32_t _params_buff[64] = {};

/**
  * @brief  Read word from flash
  *
  * @param  address: 
  * @param  buffer :
  * @param  num    : the number of dword(32 bits)!
  */
uint32_t Flash_ReadWord(uint32_t faddr)
{
    return *(__IO uint32_t*)faddr; 
}

void Flash_Read(uint32_t address, uint32_t *buffer, uint32_t size)   	
{
    uint32_t i;
    for(i=0; i<size; i++)
    {
        buffer[i] = Flash_ReadWord(address);
        address += 4;	
    }
}

uint8_t FLASH_GetFlashSector(uint32_t addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)
        return FLASH_SECTOR_0;
    else if(addr<ADDR_FLASH_SECTOR_2)
        return FLASH_SECTOR_1;
    else if(addr<ADDR_FLASH_SECTOR_3)
        return FLASH_SECTOR_2;
    else if(addr<ADDR_FLASH_SECTOR_4)
        return FLASH_SECTOR_3;
    else if(addr<ADDR_FLASH_SECTOR_5)
        return FLASH_SECTOR_4;
    else if(addr<ADDR_FLASH_SECTOR_6)
        return FLASH_SECTOR_5;
    else if(addr<ADDR_FLASH_SECTOR_7)
        return FLASH_SECTOR_6;
    else if(addr<ADDR_FLASH_SECTOR_8)
        return FLASH_SECTOR_7;
    else if(addr<ADDR_FLASH_SECTOR_9)
        return FLASH_SECTOR_8;
    else if(addr<ADDR_FLASH_SECTOR_10)
        return FLASH_SECTOR_9;
    else if(addr<ADDR_FLASH_SECTOR_11)
        return FLASH_SECTOR_10;  
    
    return FLASH_SECTOR_11;	
}

/**
  * @brief  Write a buffer of dword to flash
  *
  * @param  address: 
  * @param  buffer :
  * @param  num    : the number of double word(32 bits)!
  */
void Flash_Write(uint32_t address, uint32_t *buffer, uint32_t num)	
{ 
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    uint32_t SectorError = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;	
    if(address<STM32_FLASH_BASE || address%4)
        return;

    HAL_FLASH_Unlock();          
    addrx = address;			
    endaddr = address+num * 4;	

    if(addrx < 0X1FFF0000)
    {
        while(addrx < endaddr)
        {
            if(Flash_ReadWord(addrx) != 0XFFFFFFFF)
            {   
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
                FlashEraseInit.Sector = FLASH_GetFlashSector(addrx);
                FlashEraseInit.NbSectors = 1;                       
                FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;  
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError) != HAL_OK) 
                {
                    break;
                }
            }else 
                addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME);               
        }
    }
    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME);           
    if(FlashStatus == HAL_OK)
    {
        while(address < endaddr)
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,*buffer) != HAL_OK)
            { 
                break;	
            }
            address += 4;
            buffer++;
        }  
    }
    HAL_FLASH_Lock();
}

void param_write()
{
    memset(_params_buff, 0, sizeof(_params_buff));
    memcpy(_params_buff, &_params, sizeof(_params));
    Flash_Write(PARAMS_BASE, _params_buff, 64);
}

void param_read()
{
    memset(_params_buff, 0, sizeof(_params_buff));
    Flash_Read(PARAMS_BASE, _params_buff, 64);
    memcpy(&_params, _params_buff, sizeof(_params));
}

void param_init(void)
{
    _params.acc_calibrated = 0;
    _params.gyro_calibrated = 0;
    _params.mag_calibrated = 0;
    
    _params.acc_scale[0] = 1.0f;_params.acc_scale[1] = 1.0f;_params.acc_scale[2] = 1.0f;
    _params.acc_offset[0] = 0.0f;_params.acc_offset[1] = 0.0f;_params.acc_offset[2] = 0.0f;
    
    _params.gyro_offset[0] = 0.0f;_params.gyro_offset[1] = 0.0f;_params.gyro_offset[2] = 0.0f;
    
    _params.mag_scale[0] = 1.0f;_params.mag_scale[1] = 1.0f;_params.mag_scale[2] = 1.0f;
    _params.mag_offset[0] = 0.0f;_params.mag_offset[1] = 0.0f;_params.mag_offset[2] = 0.0f;
    _params.mag_offdiag[0] = 0.0f;_params.mag_offdiag[1] = 0.0f;_params.mag_offdiag[2] = 0.0f;
   
    param_read();
}
