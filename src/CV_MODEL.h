//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CV_MODEL.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 13-Dec-2021 11:53:25
//

#ifndef CV_MODEL_H
#define CV_MODEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
double CV_MODEL(const double x_cv_k[5], const double P_cv_k[25],
                const double y_k[3], double x_ini[5], double b_flag,
                double old_flag, double Q_CV_IMM[25], const double R_CV_IMM[9],
                double ts);

#endif
//
// File trailer for CV_MODEL.h
//
// [EOF]
//
