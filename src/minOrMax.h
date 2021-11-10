//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

#ifndef MINORMAX_H
#define MINORMAX_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
double b_maximum(const double x[61]);

double b_minimum(const double x[61]);

void b_minimum(const double x[251], double *ex, int *idx);

void c_minimum(const double x[61], double *ex, int *idx);

double maximum(const double x[251]);

double minimum(const double x[251]);

void minimum(const double x[255], double *ex, int *idx);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for minOrMax.h
//
// [EOF]
//
