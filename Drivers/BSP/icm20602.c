#include <stdbool.h>
#include <string.h>
#include "spi.h"
#include "icm20602.h"
#include "Params.h"

#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPU_GYRO_DLPF_CFG_256HZ_NOLPF2	0x00  // delay: 0.98ms
#define MPU_GYRO_DLPF_CFG_188HZ	0x01  // delay: 1.9ms
#define MPU_GYRO_DLPF_CFG_98HZ		0x02  // delay: 2.8ms
#define MPU_GYRO_DLPF_CFG_42HZ		0x03  // delay: 4.8ms
#define MPU_GYRO_DLPF_CFG_20HZ		0x04  // delay: 8.3ms
#define MPU_GYRO_DLPF_CFG_10HZ		0x05  // delay: 13.4ms
#define MPU_GYRO_DLPF_CFG_5HZ		0x06  // delay: 18.6ms
#define MPU_GYRO_DLPF_CFG_2100HZ_NOLPF	0x07
#define MPU_DLPF_CFG_MASK		0x07

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
#define BIT_INT_ANYRD_2CLEAR	0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

#define ICM_WHOAMI_20602		0x12

// ICM2608 specific registers

#define ICMREG_ACCEL_CONFIG2		0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF	0x00
#define ICM_ACC_DLPF_CFG_218HZ		0x01
#define ICM_ACC_DLPF_CFG_99HZ		0x02
#define ICM_ACC_DLPF_CFG_44HZ		0x03
#define ICM_ACC_DLPF_CFG_21HZ		0x04
#define ICM_ACC_DLPF_CFG_10HZ		0x05
#define ICM_ACC_DLPF_CFG_5HZ		0x06
#define ICM_ACC_DLPF_CFG_420HZ		0x07

/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1		0x11
#define MPUREG_ICM_UNDOC1_VALUE	0xc9

#define DEG_TO_RAD(x) (x*3.14159f/180.0f)

struct icm20602_t _icm20602;

static bool icm20602_read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx[20] = {0};
    
    if(spi1_read(&tx, rx, len) != true)
    {
        return false;
    }
    memcpy(data, rx, len);
    return true;
}

static bool icm20602_write_register(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = {reg, v};
    spi1_write(tx, 2);
    return true;
}

static bool icm20602_write_register_check(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = {0};
    tx[0] = reg;
    tx[1] = v;
    for(uint8_t i=0; i<8; i++)
    {
        spi1_write(tx, 2);
        uint8_t v2 = 0;
        if(icm20602_read_registers(reg, &v2, 1) && v2 == v)
        {
            return true;
        }
    }
    return false;
}

bool icm20602_init(void)
{
    uint8_t whoami = 0x00;
    uint8_t retries = 10;
    
    while(retries--)
    {
        icm20602_read_registers(MPUREG_WHOAMI, &whoami, 1);
        HAL_Delay(10);
        if (whoami == ICM_WHOAMI_20602)
            break;
    }
    if(whoami != ICM_WHOAMI_20602)
        return false;
    
    uint16_t bit_mask = 0x00;
    uint8_t ret = 0;
    retries = 10;
    
    icm20602_write_register(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    HAL_Delay(10);
    icm20602_write_register(MPUREG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);
    
    while((retries--) && bit_mask != 0x007f)
    {
        // Wake up device and select GyroZ clock
        ret = icm20602_write_register_check(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
        if(ret)
            bit_mask |= (0x01<<0);
        HAL_Delay(100);
        // Disable I2C bus
        ret = icm20602_write_register_check(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
        if(ret)
            bit_mask |= (0x01<<1);
        HAL_Delay(30);
        // set sample rate (approximate) - 1kHz, for both accel and gyro
        ret = icm20602_write_register_check(MPUREG_SMPLRT_DIV, 0);
        if(ret)
            bit_mask |= (0x01<<2);
        HAL_Delay(10);
        // set the DLPF filter frequency. This affects both accel and gyro.
        ret = icm20602_write_register_check(MPUREG_CONFIG, MPU_GYRO_DLPF_CFG_42HZ);
        if(ret)
            bit_mask |= (0x01<<3);
        HAL_Delay(10);
        ret = icm20602_write_register_check(ICMREG_ACCEL_CONFIG2, ICM_ACC_DLPF_CFG_44HZ); //Enable fifo
        if(ret)
            bit_mask |= (0x01<<4);
        HAL_Delay(10);
        // Gyro range +-2000 deg/s ()
        ret = icm20602_write_register_check(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
        if(ret)
            bit_mask |= (0x01<<5);
        HAL_Delay(10);
        // Accel range +-16g
        ret = icm20602_write_register_check(MPUREG_ACCEL_CONFIG, 0x03<<3);
        if(ret)
            bit_mask |= (0x01<<6);
        HAL_Delay(10);
        //icm20602_write_register_check(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN); 
//        ret = icm20602_write_register_check(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
//        if(ret)
//            bit_mask |= (0x01<<7);
        HAL_Delay(10);
    }
    
    icm20602_write_register_check(MPUREG_INT_ENABLE, 0x00);
    icm20602_write_register_check(MPUREG_FIFO_EN, 0x00);
    icm20602_write_register_check(MPUREG_PWR_MGMT_2, 0x00);

    if(bit_mask != 0x007f)
        return false;
    return true;
}

void icm20602_update(void)
{
    uint8_t data[14] = {0};
    uint16_t lsb, msb = 0;
    icm20602_read_registers(MPUREG_ACCEL_XOUT_H, data, 14);
    /*------------------------Read acceleration--------------------------------*/
    msb = data[0];
    lsb = data[1];
    _icm20602.ax_raw = (int16_t)(msb << 8) | lsb;
    msb = data[2];
    lsb = data[3];
    _icm20602.ay_raw = (int16_t)(msb << 8) | lsb;
    msb = data[4];
    lsb = data[5];
    _icm20602.az_raw = (int16_t)(msb << 8) | lsb;
    /*------------------------Read temperature--------------------------------*/
    msb = data[6];
    lsb = data[7];
    _icm20602.temp = (int16_t)(msb << 8) | lsb;
    /*----------------------------Read gyro-----------------------------------*/
    msb = data[8];
    lsb = data[9];
    _icm20602.gx_raw = (int16_t)(msb << 8) | lsb;
    msb = data[10];
    lsb = data[11];
    _icm20602.gy_raw = (int16_t)(msb << 8) | lsb;
    msb = data[12];
    lsb = data[13];
    _icm20602.gz_raw = (int16_t)(msb << 8) | lsb;
    //Apply swap
    int16_t swap = _icm20602.ax_raw;
    _icm20602.ax_raw = _icm20602.az_raw;
    _icm20602.az_raw = swap;
    _icm20602.ay_raw = -_icm20602.ay_raw;
    
    swap = _icm20602.gx_raw;
    _icm20602.gx_raw = _icm20602.gz_raw;
    _icm20602.gz_raw = swap;
    _icm20602.gy_raw = -_icm20602.gy_raw;
    
    /*----------------------------Data process--------------------------------*/
    _icm20602.ax = _icm20602.ax_raw / 2048.0f;
    _icm20602.ay = _icm20602.ay_raw / 2048.0f;
    _icm20602.az = _icm20602.az_raw / 2048.0f;
    _icm20602.temp = (_icm20602.temp / 326.8f + 25.0f);
    _icm20602.gx = DEG_TO_RAD(_icm20602.gx_raw / 16.384f);
    _icm20602.gy = DEG_TO_RAD(_icm20602.gy_raw / 16.384f);
    _icm20602.gz = DEG_TO_RAD(_icm20602.gz_raw / 16.384f);
    
    if(_params.gyro_calibrated == 0x01)
    {
        _icm20602.gx_cali = _icm20602.gx - _params.gyro_offset[0];
        _icm20602.gy_cali = _icm20602.gy - _params.gyro_offset[1];
        _icm20602.gz_cali = _icm20602.gz - _params.gyro_offset[2];
    }
    else
    {
        _icm20602.gx_cali = _icm20602.gx;
        _icm20602.gy_cali = _icm20602.gy;
        _icm20602.gz_cali = _icm20602.gz; 
    }
    if(_params.acc_calibrated == 0x01)
    {
        _icm20602.ax_cali = (_icm20602.ax - _params.acc_offset[0]) * _params.acc_scale[0];
        _icm20602.ay_cali = (_icm20602.ay - _params.acc_offset[1]) * _params.acc_scale[1];
        _icm20602.az_cali = (_icm20602.az - _params.acc_offset[2]) * _params.acc_scale[2];
    }
    else
    {
        _icm20602.ax_cali = _icm20602.ax;
        _icm20602.ay_cali = _icm20602.ay;
        _icm20602.az_cali = _icm20602.az;
    }
}

void get_icm20602_accel_gyro(struct icm20602_t *data)
{
    memcpy(data, &_icm20602, sizeof(struct icm20602_t));
}
