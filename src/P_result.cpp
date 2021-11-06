//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: P_result.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 06-Nov-2021 17:00:00
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
  if (idx == 1) {
    // CM 7
    y = 7.0;
  } else if (idx == 2) {
    //  ELCL 5
    y = 5.0;
  } else if (idx == 3) {
    //  ELCR 6
    y = 6.0;
  } else if (idx == 4) {
    //  ESL 2
    y = 2.0;
  } else if (idx == 5) {
    //  ESR 3
    y = 3.0;
  } else if (idx == 6) {
    //  ESS 4
    y = 4.0;
  } else {
    //  Not Crash
    y = 8.0;
  }
  return y;
}

//
// File trailer for P_result.cpp
//
// [EOF]
//
