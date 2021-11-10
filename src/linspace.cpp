//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linspace.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "linspace.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double d1
//                double d2
//                double y[10]
// Return Type  : void
//
namespace coder {
void linspace(double d1, double d2, double y[10])
{
  y[9] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    double delta1;
    delta1 = d2 / 9.0;
    for (int k{0}; k < 8; k++) {
      y[k + 1] = (2.0 * (static_cast<double>(k) + 2.0) - 11.0) * delta1;
    }
  } else if (((d1 < 0.0) != (d2 < 0.0)) &&
             ((std::abs(d1) > 8.9884656743115785E+307) ||
              (std::abs(d2) > 8.9884656743115785E+307))) {
    double delta1;
    double delta2;
    delta1 = d1 / 9.0;
    delta2 = d2 / 9.0;
    for (int k{0}; k < 8; k++) {
      y[k + 1] = (d1 + delta2 * (static_cast<double>(k) + 1.0)) -
                 delta1 * (static_cast<double>(k) + 1.0);
    }
  } else {
    double delta1;
    delta1 = (d2 - d1) / 9.0;
    for (int k{0}; k < 8; k++) {
      y[k + 1] = d1 + (static_cast<double>(k) + 1.0) * delta1;
    }
  }
}

} // namespace coder

//
// File trailer for linspace.cpp
//
// [EOF]
//
