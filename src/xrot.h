//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

#ifndef XROT_H
#define XROT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xrot(int n, double x[25], int ix0, int iy0, double c, double s);

void xrot(double x[25], int ix0, int iy0, double c, double s);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xrot.h
//
// [EOF]
//
