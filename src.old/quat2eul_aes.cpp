//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quat2eul_aes.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 06-Nov-2021 16:07:21
//

// Include Files
#include "quat2eul_aes.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cmath>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : double ow
//                double a
//                double b
//                double ox
//                double *r
//                double *p
//                double *y
// Return Type  : void
//
void quat2eul_aes(double ow, double a, double b, double ox, double *r,
                  double *p, double *y)
{
  double aSinInput;
  double b_r_tmp;
  double c_r_tmp;
  double d_r_tmp;
  double q_idx_0;
  double q_idx_1;
  double q_idx_2;
  double q_idx_3;
  double r_tmp;
  aSinInput = 1.0 / std::sqrt(((ow * ow + a * a) + b * b) + ox * ox);
  q_idx_0 = ow * aSinInput;
  q_idx_1 = a * aSinInput;
  q_idx_2 = b * aSinInput;
  q_idx_3 = ox * aSinInput;
  aSinInput = -2.0 * (q_idx_1 * q_idx_3 - q_idx_0 * q_idx_2);
  if (aSinInput > 1.0) {
    aSinInput = 1.0;
  }
  if (aSinInput < -1.0) {
    aSinInput = -1.0;
  }
  r_tmp = q_idx_0 * q_idx_0;
  b_r_tmp = q_idx_1 * q_idx_1;
  c_r_tmp = q_idx_2 * q_idx_2;
  d_r_tmp = q_idx_3 * q_idx_3;
  *r = rt_atan2d_snf(2.0 * (q_idx_1 * q_idx_2 + q_idx_0 * q_idx_3),
                     ((r_tmp + b_r_tmp) - c_r_tmp) - d_r_tmp);
  *p = std::asin(aSinInput);
  *y = rt_atan2d_snf(2.0 * (q_idx_2 * q_idx_3 + q_idx_0 * q_idx_1),
                     ((r_tmp - b_r_tmp) - c_r_tmp) + d_r_tmp);
}

//
// File trailer for quat2eul_aes.cpp
//
// [EOF]
//
