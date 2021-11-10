//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[61]
// Return Type  : double
//
namespace coder {
namespace internal {
double b_maximum(const double x[61])
{
  double ex;
  ex = x[0];
  for (int k{0}; k < 60; k++) {
    double d;
    d = x[k + 1];
    if (ex < d) {
      ex = d;
    }
  }
  return ex;
}

//
// Arguments    : const double x[61]
// Return Type  : double
//
double b_minimum(const double x[61])
{
  double ex;
  ex = x[0];
  for (int k{0}; k < 60; k++) {
    double d;
    d = x[k + 1];
    if (ex > d) {
      ex = d;
    }
  }
  return ex;
}

//
// Arguments    : const double x[251]
//                double *ex
//                int *idx
// Return Type  : void
//
void b_minimum(const double x[251], double *ex, int *idx)
{
  int k;
  if (!std::isnan(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 252)) {
      if (!std::isnan(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 252; k++) {
      double d;
      d = x[k - 1];
      if (*ex > d) {
        *ex = d;
        *idx = k;
      }
    }
  }
}

//
// Arguments    : const double x[61]
//                double *ex
//                int *idx
// Return Type  : void
//
void c_minimum(const double x[61], double *ex, int *idx)
{
  int k;
  if (!std::isnan(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 62)) {
      if (!std::isnan(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 62; k++) {
      double d;
      d = x[k - 1];
      if (*ex > d) {
        *ex = d;
        *idx = k;
      }
    }
  }
}

//
// Arguments    : const double x[251]
// Return Type  : double
//
double maximum(const double x[251])
{
  double ex;
  ex = x[0];
  for (int k{0}; k < 250; k++) {
    double d;
    d = x[k + 1];
    if (ex < d) {
      ex = d;
    }
  }
  return ex;
}

//
// Arguments    : const double x[251]
// Return Type  : double
//
double minimum(const double x[251])
{
  double ex;
  ex = x[0];
  for (int k{0}; k < 250; k++) {
    double d;
    d = x[k + 1];
    if (ex > d) {
      ex = d;
    }
  }
  return ex;
}

//
// Arguments    : const double x[255]
//                double *ex
//                int *idx
// Return Type  : void
//
void minimum(const double x[255], double *ex, int *idx)
{
  int k;
  if (!std::isnan(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 256)) {
      if (!std::isnan(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 256; k++) {
      double d;
      d = x[k - 1];
      if (*ex > d) {
        *ex = d;
        *idx = k;
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
