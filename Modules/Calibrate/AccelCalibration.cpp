#include <math.h>
#include <string.h>
#include "AccelCalibration.h"
#include "gcs.h"
#include "gpio.h"
#include "params.h"
#include "Ellipsoid_fit.h"
#include "AHRS_Math.h"

enum SIX_POSITION{
    LEVEL = 0,
    NOSE_DOWN,
    NOSE_UP,
    LEFT,
    RIGHT,
    BACK,
    PREPARE,
};

struct Vector3f_t
{
    float x;
    float y;
    float z;
};

#define COLLECT_COUNT 100 //100个样本
#define DEVIATION_SCALE 0.98f //加速计倾斜比例
#define CONTINUE_TIME_STATIC (0.1 * 50) //判断静止多久 0.1s
static unsigned char cali_bitmask = 0; //校准标志位(0,BACK,RIGHT,LEFT,NOSE_UP,NOSE_DOWN,LEVEL,0)
static Vector3f_t accel_avg[6] = {0};//加速计平均值
static bool calibrate_complete = false;//校准完成标志
static float _acc_beta[9] = { 1, 1, 1, 0, 0, 0, 0, 0, 0 };

/********************************低通滤波器**************************************/
#define CUTOFF_FREQUENCY 0.2f
#define THRESHOLD (0.05 * 0.05)
#define CONTINUE_TIME (2 * 100) //判断加速计静止，需要的时间2s

static float accel_filt[3] = {0};

/*****************************************************************************/
/**
  * @brief 检测加速计是否静止
  * @param accel:输入加速计采样值
  * @note 调用频率 100hz
  * @retval 静止返回 1
  */
static unsigned int count = 0;
static float accel_diff[3] = {0};
bool static_detect(float accel_x, float accel_y, float accel_z)
{
    //static unsigned int count = 0;
    //float accel_diff[3] = {0};
    float accel[3];
    accel[0] = accel_x * 9.8f;
    accel[1] = accel_y * 9.8f;
    accel[2] = accel_z * 9.8f;
    unsigned short i = 0;
    for(i=0; i< 3; i++)
        low_pass_filter(accel[i], CUTOFF_FREQUENCY, 1.0f/100.0f, &accel_filt[i]);
    for(i=0; i< 3; i++)
        accel_diff[i] = pow(accel[i] - accel_filt[i], 2);
    if(accel_diff[0] <= THRESHOLD && accel_diff[1] <= THRESHOLD && accel_diff[2] <= THRESHOLD )
    {
        count = count + 1;
        if(count >= CONTINUE_TIME)
        {
            count = CONTINUE_TIME;
            return true;
        }
    }
    else
    {
        count = 0;
    }

    return false;
}

/**
  * @brief 检测当前放置状态
  * @param 输入三个加速计数值
  * @retval 返回当前哪一个面朝下
  */
static enum SIX_POSITION position_detect(float accelx, float accely, float accelz)
{
    float length = sqrt(accelx*accelx + accely*accely + accelz*accelz);
    float n_length = 0-length;

    if( accelz < n_length*DEVIATION_SCALE )
    {
        return LEVEL;
    }else if( accelz > length*DEVIATION_SCALE )
    {
        return BACK;
    }else if( accelx < n_length*DEVIATION_SCALE )
    {
        return NOSE_DOWN;
    }else if( accelx > length*DEVIATION_SCALE )
    {
        return NOSE_UP;
    }else if( accely > length*DEVIATION_SCALE )
    {
        return LEFT;
    }else if(accely < n_length*DEVIATION_SCALE )
    { 
        return RIGHT;
    }else
    { //如果以上均不符合则还是返回到判断状态
        return PREPARE;
    }
}

//收集数据返回平均值
/**
  * @brief 收集数据返回平均值，数据保存于accel_avg，gyro_avg
  * @param 输入当前放置位置
  * @retval 无
  */
static Vector3f_t samples_accel = {0};//累加数据
static uint16_t count_1 = COLLECT_COUNT;
bool collect_data(enum SIX_POSITION position, float ax, float ay, float az)
{
    //static unsigned short count = COLLECT_COUNT;
    //static Vector3f_t samples_accel = {0};//累加数据
    
    //读取imu数据
    if(count_1 > 0)
    {
        count_1 = count_1 - 1;
        samples_accel.x = samples_accel.x + ax; //累加加速计数值
        samples_accel.y = samples_accel.y + ay;
        samples_accel.z = samples_accel.z + az;
    }
    else
    {
        accel_avg[position].x = samples_accel.x / COLLECT_COUNT; //求得平均结果保存于全局变量 accel_avg
        accel_avg[position].y = samples_accel.y / COLLECT_COUNT;
        accel_avg[position].z = samples_accel.z / COLLECT_COUNT;
        
        count_1 = COLLECT_COUNT;
        memset(&samples_accel, 0, sizeof(Vector3f_t));
        return true;
    }
    return false;
}

/**
  * @brief 校准函数
  * @param 输入三个加速计数值
  * @retval 无
  */
static enum SIX_POSITION step = PREPARE;
bool collect_six_point(float accelx, float accely, float accelz)
{
    //static enum SIX_POSITION step = PREPARE;
    static enum SIX_POSITION curr_pos = PREPARE;
    static unsigned char static_count = 0;
    static bool running = false;
    switch(step)
    {
        case PREPARE:{//准备阶段
            if(static_detect(accelx, accely, accelz)){//检测是否静止
                static_count++;
                if(static_count >= CONTINUE_TIME_STATIC)
                {//静止超过指定时间则可进入收集数据
                    static_count = CONTINUE_TIME_STATIC;
                    curr_pos = position_detect(accelx, accely, accelz);//判断当前所放位置
                    step = curr_pos;
                    if(!running)
                    {//提示开始
                        running = true;
                        LOG_I("Accel calibration start\r\n");
                    }
                }
            }
            else
            {
                static_count = 0;
            }
            if(cali_bitmask == 0x3f)
            {//校准完成
                calibrate_complete = true;
                running = false;//清除标志
                curr_pos = PREPARE;
                step = PREPARE;
                static_count = 0;
                cali_bitmask = 0;
                LOG_I("Accel calibration finish\r\n");	
                return true;
            }
            break;
        }
        case LEVEL:{
            if( (cali_bitmask & (1 << LEVEL)) == 0 )
            {//该面未校准才进入
                if(collect_data(LEVEL, accelx, accely, accelz))
                {
                    LOG_I("vehicle LEVEL finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << LEVEL);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
        case NOSE_DOWN:{
            if( (cali_bitmask & (1 << NOSE_DOWN)) == 0 )
            {//该面未校准才进入
                if(collect_data(NOSE_DOWN, accelx, accely, accelz))
                {
                    LOG_I("vehicle NOSE_DOWN finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << NOSE_DOWN);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
        case NOSE_UP:{
            if( (cali_bitmask & (1 << NOSE_UP)) == 0 )
            {//该面未校准才进入
                if(collect_data(NOSE_UP, accelx, accely, accelz))
                {
                    LOG_I("vehicle NOSE_UP finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << NOSE_UP);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
        case LEFT:{
            if( (cali_bitmask & (1 << LEFT)) == 0 )
            {//该面未校准才进入
                if(collect_data(LEFT, accelx, accely, accelz))
                {
                    LOG_I("vehicle LEFT finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << LEFT);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
        case RIGHT:{
            if( (cali_bitmask & (1 << RIGHT)) == 0 )
            {//该面未校准才进入
                if(collect_data(RIGHT, accelx, accely, accelz))
                {
                    LOG_I("vehicle RIGHT finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << RIGHT);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
        case BACK:{
            if( (cali_bitmask & (1 << BACK)) == 0 )
            {//该面未校准才进入
                if(collect_data(BACK, accelx, accely, accelz))
                {
                    LOG_I("vehicle BACK finish\r\n");
                    cali_bitmask = cali_bitmask | (1 << BACK);//填写已完成步骤标志
                }
            }
            step = PREPARE;
            break;
        }
    }
    return false;
}

void get_accel_sample(float sample[3][6])
{
    for(uint8_t i=0; i<6; i++)
    {
        sample[0][i] = accel_avg[i].x;
        sample[1][i] = accel_avg[i].y;
        sample[2][i] = accel_avg[i].z;
    }
}

void do_gyro_calibration(float gyro[3])
{
        static uint8_t collect_cnt = 100;
        static float sum[3] = {0};
        if(collect_cnt > 0)
        {
            led_blue_toggle();
            sum[0] = sum[0] + gyro[0];
            sum[1] = sum[1] + gyro[1];
            sum[2] = sum[2] + gyro[2];
            collect_cnt = collect_cnt - 1;
        }
        else
        {
            _sensor_need_cali.gyro = false;
            
            _params.gyro_calibrated = 1;
            _params.gyro_offset[0] = sum[0] /100.0f;
            _params.gyro_offset[1] = sum[1] /100.0f;
            _params.gyro_offset[2] = sum[2] /100.0f;
            param_write();
            
            sum[0] = 0;sum[1] = 0;sum[2] = 0;
            collect_cnt = 100;
            LOG_I("Gyro calibration finish\r\n");
            LOG_D("offset:%.3f,%.3f,%.3f \r\n", _params.gyro_offset[0], _params.gyro_offset[1], _params.gyro_offset[2]);
        }
}

float _acc_sample[3][6] = {0};
float _acc_radius = 0.0f;
float _accel_offset[3] = {};
float _accel_scale[3] = {};   
void do_accel_calibration(float accel[3])
{
    led_blue_toggle();
    bool ret = collect_six_point(accel[0], accel[1], accel[2]);      
    if(ret)
    {
        get_accel_sample(_acc_sample);
        Ellipsoid_Fit_RSL(false, _acc_sample[0], _acc_sample[1], _acc_sample[2], 6, 
                      _accel_offset, _accel_scale, NULL, &_acc_radius);
        
        if(isnan(_accel_offset[0]) || isnan(_accel_offset[1]) || isnan(_accel_offset[2]) || 
           isnan(_accel_scale[0]) ||isnan(_accel_scale[0]) ||isnan(_accel_scale[0]))
        {
            return;
        }
        memcpy(_params.acc_offset, _accel_offset, sizeof(float)*3);
        memcpy(_params.acc_scale, _accel_scale, sizeof(float)*3);
        _params.acc_calibrated = 1;
        param_write();
        _sensor_need_cali.acc = 0;
        LOG_D("scale: %.3f,%.3f,%.3f \r\n", _accel_scale[0], _accel_scale[1], _accel_scale[2]);
        LOG_D("offset:%.3f,%.3f,%.3f \r\n", _accel_offset[0], _accel_offset[1], _accel_offset[2]);
    }
}
