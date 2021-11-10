//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "xrot.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : int n
//                double x[25]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xrot(int n, double x[25], int ix0, int iy0, double c, double s)
{
  if (n >= 1) {
    for (int k{0}; k < n; k++) {
      double b_temp_tmp;
      double d_temp_tmp;
      int c_temp_tmp;
      int temp_tmp;
      temp_tmp = (iy0 + k) - 1;
      b_temp_tmp = x[temp_tmp];
      c_temp_tmp = (ix0 + k) - 1;
      d_temp_tmp = x[c_temp_tmp];
      x[temp_tmp] = c * b_temp_tmp - s * d_temp_tmp;
      x[c_temp_tmp] = c * d_temp_tmp + s * b_temp_tmp;
    }
  }
}

//
// Arguments    : double x[25]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(double x[25], int ix0, int iy0, double c, double s)
{
  for (int k{0}; k < 5; k++) {
    double b_temp_tmp;
    double d_temp_tmp;
    int c_temp_tmp;
    int temp_tmp;
    temp_tmp = (iy0 + k) - 1;
    b_temp_tmp = x[temp_tmp];
    c_temp_tmp = (ix0 + k) - 1;
    d_temp_tmp = x[c_temp_tmp];
    x[temp_tmp] = c * b_temp_tmp - s * d_temp_tmp;
    x[c_temp_tmp] = c * d_temp_tmp + s * b_temp_tmp;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xrot.cpp
//
// [EOF]
//
