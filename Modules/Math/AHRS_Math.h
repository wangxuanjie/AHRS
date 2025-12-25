#ifndef __AHRS_MATH_H
#define __AHRS_MATH_H
    
#include "eigen"
	
#define M_2PI (3.14159 * 2)
#define M_PI 3.14159265f
#define ABS(x) ((x)>=0?(x):(-(x)))
#define CONSTANTS_ONE_G  9.80665f
#define DEG_TO_RAD(x) (x/180.0f*M_PI)
#define RAD_TO_DEG(x) (x/M_PI*180.0f)
    
float constrain_float(float data, float min, float max);
float low_pass_filter(float sample, float cutoff_freq, float dt, float *output);

/*************************************Convert function*******************************/
Eigen::Vector3f quaternion_to_euler(Eigen::Quaternionf q);
Eigen::Quaternionf euler_to_quaternion(Eigen::Vector3f Euler);
Eigen::Vector3f conjugate_inversed(const Eigen::Quaternionf q, const Eigen::Vector3f &vec);
Eigen::Vector3f conjugate(Eigen::Quaternionf q, Eigen::Vector3f vec);
Eigen::Quaternionf derivative1(Eigen::Quaternionf q, const Eigen::Vector3f w);
Eigen::Quaternionf quat_scale(Eigen::Quaternionf q, float scale);
Eigen::Quaternionf quat_add(Eigen::Quaternionf q, const Eigen::Quaternionf w);
float wrap_pi(float x);
    
	
#endif
