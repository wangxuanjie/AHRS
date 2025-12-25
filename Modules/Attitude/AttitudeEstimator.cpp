#include <stdbool.h>
#include "eigen"
#include <math.h>
#include "stm32f4xx_it.h"
#include "AttitudeEstimator.h"
#include "AHRS_Math.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Quaternionf;
/*************************************Parameters*************************************/
uint8_t _param_att_ext_hdg_m = 1;
uint8_t _param_att_acc_comp = 1;
float _param_att_w_mag = 0.1f;
float _param_att_w_acc = 0.2f;
float _param_att_w_gyro_bias = 0.1f;
float _param_bias_max = 0.05f;
bool _param_heading_stabilization = true;
/*************************************Private value**********************************/
Quaternionf _q(0, 0, 0, 0);
Vector3f _euler_deg(0, 0, 0);
Vector3f _gyro_bias(0, 0, 0);
Vector3f _rates(0, 0, 0);
float _mag_decl = -M_PI / 2;
bool _inited = false;
uint64_t _mag_cali_fast_align_time = 0;
uint64_t _fast_align_time = 0;
float _mag_error = 0.0f;
float _acc_filter[3] = {0};
static uint64_t _good_heading_timestamp = 0;
#define FAST_ALIGN_TIME (5*1e3)
#define HEADING_RESET_TH (10.0f)
#define HEADING_RESET_TIME (60*1e3)


void attitude_estimator_init(float gyro[3], float accel[3], float mag[3])
{
    // Rotation matrix can be easily constructed from acceleration and mag field vectors
    // 'k' is Earth Z axis (Down) unit vector in body frame
    Vector3f k = Vector3f(-accel[0], -accel[1], -accel[2]);
    k.normalize();
    
    // 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
    Vector3f mag_ga = Vector3f(mag[0], mag[1], mag[2]);
    Vector3f i = mag_ga - k * (mag_ga.dot(k)); //k already normalize, so we don't have to do mag_ga - k * (mag_ga.dot(k))/k.dot(k)
    i.normalize();
    // 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
    Vector3f j = k.cross(i);
    
    // Fill rotation matrix
    Matrix3f R;
    R.row(0) = i;
    R.row(1) = j;
    R.row(2) = k;
    
    // Convert to quaternion
    _q = R;
    
    // Compensate for magnetic declination
    Quaternionf decl_rotation = euler_to_quaternion(Vector3f(0.0f, 0.0f, _mag_decl));
    _q = decl_rotation * _q;
    
    _q.normalize();
    
    _rates.Zero();
    _gyro_bias.Zero();
    
    if (isfinite(_q.x()) && isfinite(_q.y()) &&
        isfinite(_q.z()) && isfinite(_q.w()) &&
        _q.norm() > 0.95f && _q.norm() < 1.05f) {
            
        _inited = true;
    } else {
        _inited = false;
    }
    _euler_deg = RAD_TO_DEG(quaternion_to_euler(_q));
	
	_fast_align_time = get_system_ms();
}

void attitude_estimator_update(float gyro[3], float accel[3], float mag[3], float dt)
{
    Vector3f gyro_f = Vector3f(gyro[0], gyro[1], gyro[2]);
    Vector3f accel_f = Vector3f(accel[0], accel[1], accel[2]);
    Vector3f mag_f = Vector3f(mag[0], mag[1], mag[2]);
    
    if(gyro_f.norm() > DEG_TO_RAD(2000))
    {
        attitude_fast_align_mag_cali();
    }
    
	bool do_mag_cali_fast_align = ((get_system_ms()-_mag_cali_fast_align_time) < 0.5*1000) && 
							      (_mag_cali_fast_align_time != 0);
    if (!_inited || do_mag_cali_fast_align)
    {
		_gyro_bias.Zero();
        return attitude_estimator_init(gyro, accel, mag);
    }
    
    if(get_system_ms() - _fast_align_time <= FAST_ALIGN_TIME)
    {
        _param_att_w_mag = 1.0f;
        _param_att_w_acc = 1.0f;
    }
    else
    {
        _param_att_w_mag = 0.1f;
        _param_att_w_acc = 0.2f;
    }
	
    bias_fast_tracking(accel, gyro, _acc_filter);
	heading_reset_check();
    
    Quaternionf q_last = _q;

    // Angular rate of correction
    Vector3f corr(0, 0, 0);
    float spinRate = gyro_f.norm();
    
    if(_param_att_ext_hdg_m == 1)
    {
        // Magnetometer correction
        // Project mag field vector to global frame and extract XY component
        Vector3f mag_earth = conjugate(_q, mag_f);//_q.conjugate() * mag_f;//
        float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
        _mag_error = mag_err;
		
        if(fabs(_mag_error) >= DEG_TO_RAD(2.0f) && _param_heading_stabilization == true &&
		   get_system_ms() - _fast_align_time > FAST_ALIGN_TIME)
        {
            _param_att_w_mag = 0.0f;
        }
        
        float gainMult = 1.0f;
        const float fifty_dps = 0.873f;

        if (spinRate > fifty_dps) {
            gainMult = fmin(spinRate / fifty_dps, 10.0f);
        }

        // Project magnetometer correction to body frame
        corr += conjugate_inversed(_q, Vector3f(0.0f, 0.0f, -mag_err)) * _param_att_w_mag * gainMult;
    }
    
    _q.normalize();
    
    // Accelerometer correction
    // Project 'k' unit vector of earth frame to body frame
    // Vector3f k = _q.conjugate_inversed(Vector3f(0.0f, 0.0f, 1.0f));
    // Optimized version with dropped zeros

    Vector3f k(
        2.0f * (_q.x() * _q.z() - _q.w() * _q.y()),
        2.0f * (_q.y() * _q.z() + _q.w() * _q.x()),
        (_q.w() * _q.w() - _q.x() * _q.x() - _q.y() * _q.y() + _q.z() * _q.z())
    );

    // If we are not using acceleration compensation based on GPS velocity,
    // fuse accel data only if its norm is close to 1 g (reduces drift).
    const float accel_norm_sq = accel_f.squaredNorm();
    const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
    const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
    
    accel_f.normalize();
    
    if(_param_att_acc_comp || ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
       (accel_norm_sq < upper_accel_limit * upper_accel_limit)))
    {
        corr += (k.cross(accel_f) * _param_att_w_acc);
    }
    
    // Gyro bias estimation
    if(spinRate < 0.175f)
    {
       _gyro_bias += corr * (_param_att_w_gyro_bias * dt);
        for(int i = 0; i < 3; i++)
        {
            _gyro_bias(i) = constrain_float(_gyro_bias(i), -_param_bias_max, _param_bias_max);
        }
    }
    
    _rates = gyro_f + _gyro_bias;

    // Feed forward gyro
    corr += _rates;

    // Apply correction to state
    _q = quat_add(_q, quat_scale(derivative1(_q, corr), dt));

    // Normalize quaternion
    _q.normalize();

    if (!(isfinite(_q.w()) && isfinite(_q.x()) &&
        isfinite(_q.y()) && isfinite(_q.z())))
    {
        // Reset quaternion to last good state
        _q = q_last;
        _rates.Zero();
        _gyro_bias.Zero();
    }
    _euler_deg = RAD_TO_DEG(quaternion_to_euler(_q));
}

void get_euler(float euler[3])
{
    euler[0] = _euler_deg.x();
    euler[1] = _euler_deg.y();
    euler[2] = _euler_deg.z();
}

Quaternionf get_quaternion(void)
{
    return _q;
}

void get_gyro_bias(float bias[3])
{
	bias[0] = _gyro_bias.x();
	bias[1] = _gyro_bias.y();
	bias[2] = _gyro_bias.z();
}

void get_accel_filter(float data[3])
{
    for(uint8_t i =0; i<3; i++)
        data[i] = _acc_filter[i];
}

void attitude_fast_align_mag_cali()
{
	_mag_cali_fast_align_time = get_system_ms();
}

void heading_reset_check()
{
	if(fabs(_mag_error) >= DEG_TO_RAD(HEADING_RESET_TH))
	{
		if(get_system_ms() - _good_heading_timestamp >= HEADING_RESET_TIME)
		{
			_gyro_bias.Zero();
			_inited = false;
		}
	}
	else
	{
		_good_heading_timestamp = get_system_ms();
	}
		
}

#define CUTOFF_FREQUENCY 0.2f
#define THRESHOLD (0.05 * 0.05)
#define CONTINUE_TIME (3 * 200) //判断加速计静止，需要的时间3s
#define SAMPLE_BUFF_WIDTH 200
double _gyro_sample_sum[3] = {0};
uint16_t _gyro_sample_num = 0;
bool _static_flag = false;

void bias_fast_tracking(float acc[3], float gyro[3], float acc_filt[3])
{
    static unsigned int count = 0;
    static float accel_filt[3] = {0};
    float accel_diff[3] = {0};
    float accel[3];
    accel[0] = acc[0];
    accel[1] = acc[1];
    accel[2] = acc[2];
    unsigned short i = 0;
    for(i=0; i< 3; i++)
    {
        low_pass_filter(accel[i], CUTOFF_FREQUENCY, 1.0f/200.0f, &accel_filt[i]);
        acc_filt[i] = accel_filt[i];
    }
    for(i=0; i< 3; i++)
        accel_diff[i] = pow(accel[i] - accel_filt[i], 2);
    if(accel_diff[0] <= THRESHOLD && accel_diff[1] <= THRESHOLD && accel_diff[2] <= THRESHOLD )
    {
        count = count + 1;
        if(count >= CONTINUE_TIME/3 && count <= CONTINUE_TIME/3*2)
        {
            for(uint8_t i=0; i<3; i++)
                _gyro_sample_sum[i] = _gyro_sample_sum[i] + gyro[i];
            _gyro_sample_num = (_gyro_sample_num + 1);
        }
        
        if(count >= CONTINUE_TIME)
        {
            _static_flag = true;
            count = 0;
            _gyro_bias.z() = -_gyro_sample_sum[2] / _gyro_sample_num;
            memset(_gyro_sample_sum, 0, sizeof(_gyro_sample_sum));
            _gyro_sample_num = 0;
        }
    }
    else
    {
        _static_flag = false;
        memset(_gyro_sample_sum, 0, sizeof(_gyro_sample_sum));
        _gyro_sample_num = 0;
        count = 0;
    }
}
