#include "MagCalibration.h"
#include "AttitudeEstimator.h"
#include "Ellipsoid_fit.h"
#include "usart.h"
#include "gpio.h"
#include "gcs.h"
#include "stm32f4xx_it.h"
#include "params.h"
#include "AHRS_Math.h"

/******************************************************************************/
/********************************��ͨ�˲���**************************************/
#define M_2PI (3.14159 * 2)
#define CUTOFF_FREQUENCY 1.0f
#define THRESHOLD (7.0f)
#define CONTINUE_TIME (2 * 100) //�жϼ��ټƾ�ֹ����Ҫ��ʱ��2s
#define ROTATE_ANGLE 360.0f
static float accel_filt[3] = {0};
float _mag_offset[3] = {};
float _mag_scale[3] = {};
float _mag_offdiag[3] = {};
float _mag_radius = 0.0f;
Eigen::Matrix<double, 9, 1> _v;
Eigen::Matrix<double, 9, 9> _P;
#pragma pack(push, 1)
struct Mag_Cali_t 
{
    uint8_t inited;
    uint8_t step;
    float rotation_angle;
    uint16_t pos_detect_cnt;
    uint64_t start_time_ms;
}_mag_cali;
#pragma pack(pop)

uint8_t detect_orientation(float ax, float ay, float az)
{
    float accel[3] = {ax*CONSTANTS_ONE_G, ay*CONSTANTS_ONE_G, az*CONSTANTS_ONE_G};
    for(uint8_t i=0; i< 3; i++)
        low_pass_filter(accel[i], CUTOFF_FREQUENCY, 1.0f/100.0f, &accel_filt[i]);

    if(accel_filt[2] < 0 && (fabs(fabs(accel_filt[2])-CONSTANTS_ONE_G) < THRESHOLD) )
        return RIGHTSIDE_UP;
    else if(accel_filt[1] < 0 && (fabs(fabs(accel_filt[1])-CONSTANTS_ONE_G) < THRESHOLD))
        return NOSE_UP;
    else
        return PREPARE;
}

void mag_calibration_init()
{
    _mag_cali.inited = 1;
    _mag_cali.pos_detect_cnt = 0;
    _mag_cali.step = 0;
    _mag_cali.rotation_angle = 0.0f;
    _mag_cali.start_time_ms = get_system_ms();
    _v.setZero();
    _P.setZero();
    
    _P(0,0) = 10.0f; _P(1,1) = 10.0f; _P(2,2) = 10.0f;
    _P(3,3) = 0.0f; _P(4,4) = 0.0f; _P(5,5) = 0.0f;
    _P(6,6) = 1.0f; _P(7,7) = 1.0f; _P(8,8) = 1.0f;
}

void do_mag_calibration(float accel[3], float gyro[3], float mag[3])
{
    static uint64_t timestamp_last = 0;
    static uint16_t report_switch_pos = 0;
    uint8_t pos = detect_orientation(accel[0], accel[1], accel[2]);
    
    if(_mag_cali.inited == 0)
    {
        timestamp_last = get_system_ms();
        mag_calibration_init();
        LOG_I("Hold imu LEVEL, Don't move!");
    }
    
    if(get_system_ms() - _mag_cali.start_time_ms >= 60*1000)
    {
        _mag_cali.inited = 0;
        _sensor_need_cali.mag = false;
        LOG_I("Mag calibration timeout!");
        ano_seng_mag_cali(0x00,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
    }
    if(_sensor_need_cali.cancel_mag)
    {
        _sensor_need_cali.cancel_mag = false;
        _sensor_need_cali.mag = false;
        _mag_cali.inited = 0;
        LOG_I("Mag calibration cancel!");
    }
    
    float dt = (get_system_ms() - timestamp_last) / 1000.0f;
    timestamp_last = get_system_ms();
    
    switch(_mag_cali.step)
    {
        case 0:
        {
            if(pos == RIGHTSIDE_UP)
            {
                _mag_cali.pos_detect_cnt ++;
                if(_mag_cali.pos_detect_cnt >= CONTINUE_TIME)
                {
                    _mag_cali.pos_detect_cnt = 0;
                    _mag_cali.step = 1;
                    LOG_I("Rotate 360 deg!");
                    ano_seng_mag_cali(0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
                }
            }
            else
            {
                _mag_cali.pos_detect_cnt = 0;
            }
            break;
        }
        case 1:
        {
            if(pos == RIGHTSIDE_UP)
            {
                _mag_cali.rotation_angle += dt * gyro[2] * 57.3f;

                if(fabs(_mag_cali.rotation_angle) <= ROTATE_ANGLE)
                {
                    Ellipsoid_Fit_KF(mag[0], mag[1], mag[2], _P, _v);
                }
                else
                {
                    _mag_cali.step = 2;
                    //Clean all status
                    _mag_cali.pos_detect_cnt = 0;
                    _mag_cali.rotation_angle = 0.0f;
                    LOG_I("Switch imu to NOSE UP!");
                    ano_seng_mag_cali(0x00,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00);
                }
            }
            break;
        }
        case 2:
        {
            if(pos != NOSE_UP)
            {
                report_switch_pos = (report_switch_pos + 1) % 100;
                if(report_switch_pos == 0)
                {
                    LOG_I("Switch imu to NOSE UP!");
                    ano_seng_mag_cali(0x00,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00);
                }
            }
            else
            {
                _mag_cali.step = 3;
                LOG_I("Hold imu NOSE UP, Don't move!");
            }
            break;
        }
        case 3:
        {
            if(pos == NOSE_UP)
            {
                _mag_cali.pos_detect_cnt ++;
                if(_mag_cali.pos_detect_cnt >= CONTINUE_TIME)
                {
                    _mag_cali.pos_detect_cnt = 0;
                    _mag_cali.step = 4;
                    LOG_I("Rotate 360 deg!");
                    ano_seng_mag_cali(0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
                }
            }
            else
            {
                _mag_cali.pos_detect_cnt = 0;
            }
            break;
        }
        case 4:
        {
            if(pos == NOSE_UP)
            {
                _mag_cali.rotation_angle += dt * gyro[1] * 57.3f;

                if(fabs(_mag_cali.rotation_angle) <= ROTATE_ANGLE)
                {
                    Ellipsoid_Fit_KF(mag[0], mag[1], mag[2], _P, _v);
                }
                else
                {
                    _mag_cali.step = 5;
                    //Clean all status
                    _mag_cali.pos_detect_cnt = 0;
                    _mag_cali.rotation_angle = 0.0f;
                    LOG_I("Mag calibration finish");
                    ano_seng_mag_cali(0x00,0x01,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00);
                }
            }
            break;
        }
        case 5:
        {
            _mag_cali.inited = 0;
            
            //solve ellipsoid
            Ellipsoid_Fit_Solve(1, _v, _mag_offset, _mag_scale, _mag_offdiag);
            
            if(isnan(_mag_offset[0]) || isnan(_mag_offset[1]) || isnan(_mag_offset[2]) || 
               isnan(_mag_scale[0]) ||isnan(_mag_scale[0]) ||isnan(_mag_scale[0]))
            {
                LOG_W("Mag calibration fail");
                ano_seng_mag_cali(0x00,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
                _sensor_need_cali.mag = false;
                return;
            }
            
            LOG_D("scale: %.3f,%.3f,%.3f \r\n", _mag_scale[0], _mag_scale[1], _mag_scale[2]);
            LOG_D("offset:%.1f,%.1f,%.1f \r\n", _mag_offset[0], _mag_offset[1], _mag_offset[2]);
            ano_seng_mag_cali(0x00,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
            gcs_tx_buff_flush();
            
            memcpy(_params.mag_offset, _mag_offset, sizeof(float)*3);
            memcpy(_params.mag_scale, _mag_scale, sizeof(float)*3);
            memcpy(_params.mag_offdiag, _mag_offdiag, sizeof(float)*3);
            _params.mag_radius = _mag_radius;
            _params.mag_calibrated = 1;
            param_write();
            
			attitude_fast_align_mag_cali();
            _sensor_need_cali.mag = false;
            break;
        }
    }
}

