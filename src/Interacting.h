//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Interacting.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 13-Dec-2021 11:53:25
//

#ifndef INTERACTING_H
#define INTERACTING_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void Interacting(double Prob_ctrv_old, double Prob_cv_old,
                 const double x_ctrv_old[5], const double x_cv_old[5],
                 const double P_ctrv_old[25], const double P_cv_old[25],
                 double *c_ctrv, double x_ctrv_out[5], double P_ctrv_out[25],
                 double *c_cv, double x_cv_out[5], double P_cv_out[25]);

#endif
//
// File trailer for Interacting.h
//
// [EOF]
//
