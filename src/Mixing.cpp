//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Mixing.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "Mixing.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// if time >= 180
//      time = time;
//  end
//
//  if time >= 240
//      time = time;
//  end
//
// Arguments    : double c_ctrv
//                const double x_ctrv[5]
//                const double P_ctrv[25]
//                double mu_ctrv
//                double c_cv
//                const double x_cv[5]
//                const double P_cv[25]
//                double mu_cv
//                double *Prob_ctrv
//                double *Prob_cv
//                double X[5]
//                double P[25]
// Return Type  : void
//
void Mixing(double c_ctrv, const double x_ctrv[5], const double P_ctrv[25],
            double mu_ctrv, double c_cv, const double x_cv[5],
            const double P_cv[25], double mu_cv, double *Prob_ctrv,
            double *Prob_cv, double X[5], double P[25])
{
  double b_P_ctrv[25];
  double b_P_cv[25];
  double dv[5];
  double dv1[5];
  double Prob_ctrv_tmp;
  double b_Prob_ctrv_tmp;
  double c_Prob_ctrv_tmp;
  int b_i;
  int i;
  int i1;
  //  x_k = [y x yaw v yawrate]';
  // -------------------------------------------------------------------------
  //  Parameter
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  Prob_ctrv_tmp = c_ctrv * mu_ctrv;
  b_Prob_ctrv_tmp = c_cv * mu_cv;
  c_Prob_ctrv_tmp = Prob_ctrv_tmp + b_Prob_ctrv_tmp;
  *Prob_ctrv = Prob_ctrv_tmp / c_Prob_ctrv_tmp;
  *Prob_cv = b_Prob_ctrv_tmp / c_Prob_ctrv_tmp;
  for (i = 0; i < 5; i++) {
    Prob_ctrv_tmp = x_ctrv[i];
    b_Prob_ctrv_tmp = x_cv[i];
    c_Prob_ctrv_tmp = *Prob_ctrv * Prob_ctrv_tmp + *Prob_cv * b_Prob_ctrv_tmp;
    X[i] = c_Prob_ctrv_tmp;
    dv[i] = Prob_ctrv_tmp - c_Prob_ctrv_tmp;
    dv1[i] = b_Prob_ctrv_tmp - c_Prob_ctrv_tmp;
  }
  for (b_i = 0; b_i < 5; b_i++) {
    for (i1 = 0; i1 < 5; i1++) {
      i = i1 + 5 * b_i;
      b_P_ctrv[i] = P_ctrv[i] + dv[i1] * dv[b_i];
      b_P_cv[i] = P_cv[i] + dv1[i1] * dv1[b_i];
    }
  }
  for (b_i = 0; b_i < 5; b_i++) {
    for (i1 = 0; i1 < 5; i1++) {
      Prob_ctrv_tmp = 0.0;
      for (i = 0; i < 5; i++) {
        Prob_ctrv_tmp += P_cv[b_i + 5 * i] * b_P_cv[i + 5 * i1];
      }
      i = b_i + 5 * i1;
      P[i] = *Prob_ctrv * b_P_ctrv[i] + Prob_ctrv_tmp;
    }
  }
  // -------------------------------------------------------------------------
  //  if time >= 0
  //      time = time;
  //  end
}

//
// File trailer for Mixing.cpp
//
// [EOF]
//
