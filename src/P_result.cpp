//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: P_result.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 01-Sep-2021 20:42:36
//

// Include Files
#include "P_result.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const float u[7]
// Return Type  : double
//
double P_result(const float u[7])
{
  double y;
  int idx;
  int k;
  if (!std::isnan(u[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 8)) {
      if (!std::isnan(u[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    idx = 1;
  } else {
    float ex;
    int i;
    ex = u[idx - 1];
    i = idx + 1;
    for (k = i; k < 8; k++) {
      float f;
      f = u[k - 1];
      if (ex < f) {
        ex = f;
        idx = k;
      }
    }
  }
  // class(u)
  if (idx == 1) {
    // ACC
    y = 1.0;
  } else if (idx == 2) {
    // CM
    y = 2.0;
  } else if (idx == 3) {
    // DEC
    y = 3.0;
  } else if (idx == 4) {
    // ELCL
    y = 4.0;
  } else if (idx == 5) {
    // ELCR
    y = 5.0;
  } else if (idx == 6) {
    //  ESL
    y = 6.0;
  } else {
    //  ESR
    y = 7.0;
  }
  return y;
}

//
// File trailer for P_result.cpp
//
// [EOF]
//
