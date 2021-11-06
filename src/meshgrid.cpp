//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: meshgrid.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "meshgrid.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : const double x[251]
//                const double y[61]
//                double xx[15311]
//                double yy[15311]
// Return Type  : void
//
namespace coder {
void meshgrid(const double x[251], const double y[61], double xx[15311],
              double yy[15311])
{
  for (int j{0}; j < 251; j++) {
    std::copy(&y[0], &y[61], &yy[j * 61]);
    for (int i{0}; i < 61; i++) {
      int xx_tmp;
      xx_tmp = i + 61 * j;
      xx[xx_tmp] = x[j];
    }
  }
}

} // namespace coder

//
// File trailer for meshgrid.cpp
//
// [EOF]
//
