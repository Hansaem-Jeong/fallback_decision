//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CTRV_MODEL.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

#ifndef CTRV_MODEL_H
#define CTRV_MODEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
double CTRV_MODEL(const double x_ctrv_k[5], const double P_ctrv_k[25],
                  const double y_ctrv_k[3], double x_ini[5], double b_flag,
                  double old_flag, double Q_CTRV_IMM[25],
                  const double R_CTRV_IMM[9], double ts);

#endif
//
// File trailer for CTRV_MODEL.h
//
// [EOF]
//
