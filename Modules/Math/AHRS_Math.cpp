#include <stdbool.h>
#include "eigen"
#include <math.h>
#include "AHRS_Math.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Quaternionf;

float constrain_float(float data, float min, float max)
{
    return (data < min) ? min : ((data > max) ? max : data);
}

/**
  * @brief 计算 滤波后数值
  * @param sample:样本, cutoff_freq:设置截止频率, dt:时间
  * @retval 返回滤波后数值
  */
float low_pass_filter(float sample, float cutoff_freq, float dt, float *output)
{
	float alpha = 0;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) 
    {
        *output = sample;
        return *output;
    }
    float rc = 1.0f/(M_2PI*cutoff_freq);
    alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
    *output += (sample - *output) * alpha;
    return *output;
}

/**
 * Wrap value in range [-??, ??)
 */
float wrap_pi(float x)
{
    if (!isfinite(x))
        return x;
    
    int c = 0;
    while (x >= M_PI)
    {
        x -= 2 * M_PI;
        if (c++ > 100)
            return INFINITY;
    }

    c = 0;
    while (x < -M_PI)
    {
        x += 2 * M_PI;
        if(c++ > 100)
            return INFINITY;
    }
    return x;
}

Vector3f conjugate(Quaternionf q, Vector3f vec)
{
    Quaternionf v(0, vec(0), vec(1), vec(2));
    Quaternionf res = q*v*q.inverse();
    return Vector3f(res.x(), res.y(), res.z());
}

/**
 * Rotates vector v_2 in frame 2 to vector v_1 in frame 1
 * using the rotation quaternion q_21
 * describing the rotation from frame 1 to 2
 * v_1 = q_21^-1 * v_2 * q_21
 *
 * @param vec vector to rotate in frame 2 (typically reference frame)
 * @return rotated vector in frame 1 (typically body frame)
 */
Vector3f conjugate_inversed(const Quaternionf q, const Vector3f &vec)
{
    Quaternionf v(0.0f, vec(0), vec(1), vec(2));
    Quaternionf res = q.inverse()*v*q;
    return Vector3f(res.x(), res.y(), res.z());
}

Quaternionf derivative1(Quaternionf q, const Vector3f w)
{
    Quaternionf v(0, w(0), w(1), w(2));
    Quaternionf res(0, 0, 0, 0);
    Quaternionf temp = q * v;
    res = quat_scale(temp, 0.5f);
    return res;
}

Quaternionf quat_add(Quaternionf q, const Quaternionf w)
{
    return Quaternionf(q.w()+w.w(),
                       q.x()+w.x(),
                       q.y()+w.y(),
                       q.z()+w.z());
}

Quaternionf quat_scale(Quaternionf q, float scale)
{
    return Quaternionf(q.w()*scale,
                       q.x()*scale,
                       q.y()*scale,
                       q.z()*scale);
}

Vector3f quaternion_to_euler(Eigen::Quaternionf q)
{
    Vector3f euler;
    euler(0) = atan2(2 * (q.y()*q.z() + q.w()*q.x()), (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()));
    euler(1) = asin(-2 * q.x()*q.z() + 2 * q.w()*q.y());
    euler(2) = atan2(2 * (q.x()*q.y() + q.w()*q.z()), (q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()));
    return euler;
}

Quaternionf euler_to_quaternion(Vector3f Euler)
{
    Quaternionf q(0, 0, 0, 0);
    Euler = Euler * 0.5;
    float cosPhi   = cos(Euler(0));
    float sinPhi   = sin(Euler(0));
    float cosTheta = cos(Euler(1));
    float sinTheta = sin(Euler(1));
    float cosPsi   = cos(Euler(2));
    float sinPsi   = sin(Euler(2));

    q.w() = (cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi);
    q.x() = (sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi);
    q.y() = (cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi);
    q.z() = (cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi);
    return q;
}

