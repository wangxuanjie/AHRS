#include <string.h>
#include "ist8310.h"
#include "i2c.h"
#include "Params.h"

/* Hardware definitions */
#define IST8310_BUS_I2C_ADDR		0x0C

/*
 * FSR:
 *   x, y: +- 1600 uT
 *   z:    +- 2500 uT
 *
 * Resolution according to datasheet is 0.3uT/LSB
 */
#define IST8310_RESOLUTION	0.3

static const int16_t IST8310_MAX_VAL_XY	= (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY	= -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;

#define ADDR_WAI                0		/* WAI means 'Who Am I'*/
# define WAI_EXPECTED_VALUE     0x10

#define ADDR_STAT1              0x02
# define STAT1_DRDY_SHFITS      0x0
# define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
# define STAT1_DRO_SHFITS       0x1
# define STAT1_DRO              (1 << STAT1_DRO_SHFITS)

#define ADDR_DATA_OUT_X_LSB     0x03
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x05
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x07
#define ADDR_DATA_OUT_Z_MSB     0x08

#define ADDR_STAT2              0x09
# define STAT2_INT_SHFITS       3
# define STAT2_INT              (1 << STAT2_INT_SHFITS)

#define ADDR_CTRL1              0x0a
# define CTRL1_MODE_SHFITS      0
# define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
# define CTRL1_MODE_SINGLE      (1 << CTRL1_MODE_SHFITS)

#define ADDR_CTRL2              0x0b
# define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
# define CTRL2_SRST             (0x01 << CTRL2_SRST_SHFITS)
# define CTRL2_DRP_SHIFTS       2
# define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
# define CTRL2_DREN_SHIFTS      3
# define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define ADDR_CTRL3				0x41
# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

#define ADDR_CTRL4				0x42
# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration */

#define ADDR_STR                0x0c
# define STR_SELF_TEST_SHFITS   6
# define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
# define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad

#define ADDR_TEMPL              0x1c
#define ADDR_TEMPH              0x1d

static uint8_t _ctrl1 = CTRL1_MODE_SINGLE;
struct ist8310_t _ist8310;


static bool ist8310_read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    bool ret = false;
    ret = i2c1_read(IST8310_BUS_I2C_ADDR, &reg, 1, data, len);
    return ret;
}

static bool ist8310_write_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buf[20] = {0};
    bool ret = false;
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    ret = i2c1_write(IST8310_BUS_I2C_ADDR, buf, len+1);
    return ret;
}

static bool ist8310_write_register_check(uint8_t reg, uint8_t data)
{
    for(uint8_t i=0; i<8; i++)
    {
        ist8310_write_registers(reg, &data, 1);
        uint8_t v2 = 0;
        if(ist8310_read_registers(reg, &v2, 1) && v2 == data)
        {
            return true;
        }
    }
    return false;
}

bool ist8310_init(void)
{
    uint8_t retries = 10;
    uint8_t whoami = 0x00;

    uint8_t ctrl2 = CTRL2_SRST;
    /* software reset */
    ist8310_read_registers(ADDR_CTRL2, &ctrl2, 1);
    
    while(retries--)
    {
        ist8310_read_registers(ADDR_WAI, &whoami, 1);
        if (whoami == WAI_EXPECTED_VALUE)
            break;
        HAL_Delay(10);
    }
    if(whoami == WAI_EXPECTED_VALUE)
        HAL_Delay(10);//Detect ist8310
    else{
        //Not detect ist8310
        return false;
    }
    uint16_t bit_mask = 0x00;
    uint8_t ret = 0;
    retries = 10;

    /* software reset */
    ist8310_read_registers(ADDR_CTRL2, &ctrl2, 1);
    
    HAL_Delay(10);
    while((retries--) && bit_mask != 0x0003)
    {
        bit_mask = 0x00;
        /* configure control register 3 */
        ret = ist8310_write_register_check(ADDR_CTRL3, CTRL3_SAMPLEAVG_16);
        if(ret)
            bit_mask |= (0x01<<0);
        HAL_Delay(10);
        /* configure control register 4 */
        ret = ist8310_write_register_check(ADDR_CTRL4, CTRL4_SRPD);
        if(ret)
            bit_mask |= (0x01<<1);
        HAL_Delay(10);
    }
    if(bit_mask != 0x0003)
    {
        //Init ist8310 fail
        return false;
    }
    ist8310_write_registers(ADDR_CTRL1, &_ctrl1, 1);
    HAL_Delay(10);
    //Init ist8310 success
    return true;
}

void ist8310_update(void)
{   
    uint8_t data[6] = {0};
    uint16_t lsb, msb;
    ist8310_read_registers(ADDR_DATA_OUT_X_LSB, data, 6);
    lsb = data[0];
    msb = data[1];
    _ist8310.mx_raw = (int16_t)(msb << 8) | lsb;
    lsb = data[2];
    msb = data[3];
    _ist8310.my_raw = (int16_t)(msb << 8) | lsb;
    lsb = data[4];
    msb = data[5];
    _ist8310.mz_raw = (int16_t)(msb << 8) | lsb;
    
    //Apply swap
    int16_t swap = _ist8310.mx_raw;
    _ist8310.mx_raw = _ist8310.mz_raw;
    _ist8310.mz_raw = -swap;
    _ist8310.my_raw = -_ist8310.my_raw;
    
    ist8310_write_registers(ADDR_CTRL1, &_ctrl1, 1);
	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers
	 */
	if (_ist8310.mx_raw > IST8310_MAX_VAL_XY || _ist8310.mx_raw < IST8310_MIN_VAL_XY ||
	    _ist8310.my_raw > IST8310_MAX_VAL_XY || _ist8310.my_raw < IST8310_MIN_VAL_XY ||
	    _ist8310.mz_raw > IST8310_MAX_VAL_Z  || _ist8310.mz_raw < IST8310_MIN_VAL_Z)
    {
        return;
	}
    /* Resolution: 0.3 uT/LSB - already convert to milligauss */
    _ist8310.mx = _ist8310.mx_raw * 3.0f;
    _ist8310.my = _ist8310.my_raw * 3.0f;
    _ist8310.mz = _ist8310.mz_raw * 3.0f;
    
    if(_params.mag_calibrated == 0x01)
    {
        float mx_remove_bias = _ist8310.mx - _params.mag_offset[0];
        float my_remove_bias = _ist8310.my - _params.mag_offset[1];
        float mz_remove_bias = _ist8310.mz - _params.mag_offset[2];
        
        _ist8310.mx_cali = _params.mag_scale[0] * mx_remove_bias + _params.mag_offdiag[0] * my_remove_bias + _params.mag_offdiag[1] * mz_remove_bias;
        _ist8310.my_cali = _params.mag_offdiag[0] * mx_remove_bias + _params.mag_scale[1] * my_remove_bias + _params.mag_offdiag[2] * mz_remove_bias;
        _ist8310.mz_cali = _params.mag_offdiag[1] * mx_remove_bias + _params.mag_offdiag[2] * my_remove_bias + _params.mag_scale[2] * mz_remove_bias;
    }
    else
    {
        _ist8310.mx_cali = _ist8310.mx;
        _ist8310.my_cali = _ist8310.my;
        _ist8310.mz_cali = _ist8310.mz;
    }
}

void get_ist8310(struct ist8310_t *data)
{
    memcpy(data, &_ist8310, sizeof(struct ist8310_t));
}
