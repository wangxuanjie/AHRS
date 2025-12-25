#ifndef __ELLIPSOID_FIT_H
#define __ELLIPSOID_FIT_H

#include <stdbool.h>
#include <stdint.h>
#include "eigen"

void Ellipsoid_Fit_KF(float x, float y, float z, Eigen::Matrix<double, 9, 9> &mat_P, Eigen::Matrix<double, 9, 1> &mat_v);
void Ellipsoid_Fit_Solve(bool cross_axis_gain, Eigen::Matrix<double, 9, 1> &mat_v, float offset[3], float scale[3], float offdiag[3]);
void Ellipsoid_Fit_RSL(bool cross_axis_gain ,float x[], float y[], float z[], unsigned int sample_num, float offset[3], float scale[3], float offdiag[3], float *sphere_radius);

#endif
