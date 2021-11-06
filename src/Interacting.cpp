//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Interacting.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "Interacting.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : double Prob_ctrv_old
//                double Prob_cv_old
//                const double x_ctrv_old[5]
//                const double x_cv_old[5]
//                const double P_ctrv_old[25]
//                const double P_cv_old[25]
//                double *c_ctrv
//                double x_ctrv_out[5]
//                double P_ctrv_out[25]
//                double *c_cv
//                double x_cv_out[5]
//                double P_cv_out[25]
// Return Type  : void
//
void Interacting(double Prob_ctrv_old, double Prob_cv_old,
                 const double x_ctrv_old[5], const double x_cv_old[5],
                 const double P_ctrv_old[25], const double P_cv_old[25],
                 double *c_ctrv, double x_ctrv_out[5], double P_ctrv_out[25],
                 double *c_cv, double x_cv_out[5], double P_cv_out[25])
{
  double b_P_cv_old[25];
  double dv[5];
  double dv1[5];
  double a;
  double b_a;
  int b_i;
  int i;
  int i1;
  //  ts=0.2;
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  //  Prob_ctrv_old=;
  //  Prob_cv_old=;
  //  x_ctrv_old=;
  //  x_cv_old=;
  *c_ctrv = 0.9 * Prob_ctrv_old + 0.1 * Prob_cv_old;
  *c_cv = 0.1 * Prob_ctrv_old + 0.9 * Prob_cv_old;
  for (i = 0; i < 5; i++) {
    double d;
    a = x_ctrv_old[i];
    b_a = x_cv_old[i];
    d = a / *c_ctrv * 0.9 * Prob_ctrv_old + b_a / *c_ctrv * 0.1 * Prob_cv_old;
    x_ctrv_out[i] = d;
    x_cv_out[i] =
        a / *c_cv * 0.1 * Prob_ctrv_old + b_a / *c_cv * 0.9 * Prob_cv_old;
    dv[i] = a - d;
    dv1[i] = b_a - d;
  }
  a = 0.9 * Prob_ctrv_old / *c_ctrv;
  b_a = 0.1 * Prob_cv_old / *c_ctrv;
  for (b_i = 0; b_i < 5; b_i++) {
    for (i1 = 0; i1 < 5; i1++) {
      i = i1 + 5 * b_i;
      P_ctrv_out[i] = P_ctrv_old[i] + dv[i1] * dv[b_i];
      b_P_cv_old[i] = P_cv_old[i] + dv1[i1] * dv1[b_i];
    }
  }
  for (b_i = 0; b_i < 25; b_i++) {
    P_ctrv_out[b_i] = a * P_ctrv_out[b_i] + b_a * b_P_cv_old[b_i];
  }
  for (i = 0; i < 5; i++) {
    a = x_cv_out[i];
    dv[i] = x_ctrv_old[i] - a;
    dv1[i] = x_cv_old[i] - a;
  }
  a = 0.1 * Prob_ctrv_old / *c_cv;
  b_a = 0.9 * Prob_cv_old / *c_cv;
  for (b_i = 0; b_i < 5; b_i++) {
    for (i1 = 0; i1 < 5; i1++) {
      i = i1 + 5 * b_i;
      P_cv_out[i] = P_ctrv_old[i] + dv[i1] * dv[b_i];
      b_P_cv_old[i] = P_cv_old[i] + dv1[i1] * dv1[b_i];
    }
  }
  for (b_i = 0; b_i < 25; b_i++) {
    P_cv_out[b_i] = a * P_cv_out[b_i] + b_a * b_P_cv_old[b_i];
  }
}

//
// File trailer for Interacting.cpp
//
// [EOF]
//
