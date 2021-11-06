//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide_helper.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : const double A[15]
//                const double B[9]
//                double Y[15]
// Return Type  : void
//
namespace coder {
namespace internal {
void mrdiv(const double A[15], const double B[9], double Y[15])
{
  double b_A[9];
  double a21;
  double maxval;
  int r1;
  int r2;
  int r3;
  int rtemp;
  std::copy(&B[0], &B[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(B[0]);
  a21 = std::abs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (std::abs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  for (rtemp = 0; rtemp < 5; rtemp++) {
    int Y_tmp;
    int b_Y_tmp;
    int c_Y_tmp;
    Y_tmp = rtemp + 5 * r1;
    Y[Y_tmp] = A[rtemp] / b_A[r1];
    b_Y_tmp = rtemp + 5 * r2;
    Y[b_Y_tmp] = A[rtemp + 5] - Y[Y_tmp] * b_A[r1 + 3];
    c_Y_tmp = rtemp + 5 * r3;
    Y[c_Y_tmp] = A[rtemp + 10] - Y[Y_tmp] * b_A[r1 + 6];
    Y[b_Y_tmp] /= b_A[r2 + 3];
    Y[c_Y_tmp] -= Y[b_Y_tmp] * b_A[r2 + 6];
    Y[c_Y_tmp] /= b_A[r3 + 6];
    Y[b_Y_tmp] -= Y[c_Y_tmp] * b_A[r3 + 3];
    Y[Y_tmp] -= Y[c_Y_tmp] * b_A[r3];
    Y[Y_tmp] -= Y[b_Y_tmp] * b_A[r2];
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for mrdivide_helper.cpp
//
// [EOF]
//
