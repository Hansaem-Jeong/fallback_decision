//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inpolygon.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "inpolygon.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace coder {
static void contrib(double x1, double b_y1, double x2, double y2,
                    signed char quad1, signed char quad2, double scale,
                    signed char *diffQuad, bool *onj);

}

// Function Definitions
//
// Arguments    : double x1
//                double b_y1
//                double x2
//                double y2
//                signed char quad1
//                signed char quad2
//                double scale
//                signed char *diffQuad
//                bool *onj
// Return Type  : void
//
namespace coder {
static void contrib(double x1, double b_y1, double x2, double y2,
                    signed char quad1, signed char quad2, double scale,
                    signed char *diffQuad, bool *onj)
{
  double cp;
  *onj = false;
  *diffQuad = static_cast<signed char>(quad2 - quad1);
  cp = x1 * y2 - x2 * b_y1;
  if (std::abs(cp) < scale) {
    *onj = (x1 * x2 + b_y1 * y2 <= 0.0);
    if ((*diffQuad == 2) || (*diffQuad == -2)) {
      *diffQuad = 0;
    } else if (*diffQuad == -3) {
      *diffQuad = 1;
    } else if (*diffQuad == 3) {
      *diffQuad = -1;
    }
  } else if (cp < 0.0) {
    if (*diffQuad == 2) {
      *diffQuad = -2;
    } else if (*diffQuad == -3) {
      *diffQuad = 1;
    } else if (*diffQuad == 3) {
      *diffQuad = -1;
    }
  } else if (*diffQuad == -2) {
    *diffQuad = 2;
  } else if (*diffQuad == -3) {
    *diffQuad = 1;
  } else if (*diffQuad == 3) {
    *diffQuad = -1;
  }
}

//
// Arguments    : const double x[15311]
//                const double y[15311]
//                const double xv[5]
//                const double yv[5]
//                bool in[15311]
//                bool on[15311]
// Return Type  : void
//
void inpolygon(const double x[15311], const double y[15311], const double xv[5],
               const double yv[5], bool in[15311], bool on[15311])
{
  double scale[5];
  int last[5];
  int i;
  int k;
  int nloops;
  signed char first[5];
  signed char dquad;
  bool onj;
  std::memset(&in[0], 0, 15311U * sizeof(bool));
  std::memset(&on[0], 0, 15311U * sizeof(bool));
  nloops = -1;
  for (i = 0; i < 5; i++) {
    first[i] = 0;
    last[i] = 0;
  }
  k = 0;
  while ((k + 1 <= 5) && std::isnan(xv[k])) {
    k++;
  }
  while (k + 1 <= 5) {
    bool exitg1;
    nloops++;
    i = k;
    first[nloops] = static_cast<signed char>(k + 1);
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 5)) {
      k++;
      if (std::isnan(xv[k]) || std::isnan(yv[k])) {
        k--;
        exitg1 = true;
      }
    }
    if ((xv[k] == xv[i]) && (yv[k] == yv[i])) {
      last[nloops] = k;
    } else {
      last[nloops] = k + 1;
    }
    k += 2;
    while ((k + 1 <= 5) && std::isnan(xv[k])) {
      k++;
    }
  }
  if (nloops + 1 != 0) {
    double a;
    double b;
    double maxxv;
    double maxyv;
    double minxv_tmp;
    double minyv_tmp;
    int b_i;
    int i1;
    int j;
    minxv_tmp = xv[first[0] - 1];
    maxxv = minxv_tmp;
    minyv_tmp = yv[first[0] - 1];
    maxyv = minyv_tmp;
    for (k = 0; k <= nloops; k++) {
      b_i = first[k];
      i1 = last[k];
      for (j = b_i; j <= i1; j++) {
        a = xv[j - 1];
        if (a < minxv_tmp) {
          minxv_tmp = a;
        } else if (a > maxxv) {
          maxxv = a;
        }
      }
      i1 = last[k];
      for (j = b_i; j <= i1; j++) {
        a = yv[j - 1];
        if (a < minyv_tmp) {
          minyv_tmp = a;
        } else if (a > maxyv) {
          maxyv = a;
        }
      }
    }
    for (i = 0; i < 5; i++) {
      scale[i] = 0.0;
    }
    for (j = 0; j <= nloops; j++) {
      b_i = first[j];
      i1 = last[j] - 1;
      for (i = b_i; i <= i1; i++) {
        a = std::abs(0.5 * (xv[i - 1] + xv[i]));
        b = std::abs(0.5 * (yv[i - 1] + yv[i]));
        if ((a > 1.0) && (b > 1.0)) {
          a *= b;
        } else if ((b > a) || std::isnan(a)) {
          a = b;
        }
        scale[i - 1] = a * 6.6613381477509392E-16;
      }
      i = first[j] - 1;
      b_i = last[j];
      a = std::abs(0.5 * (xv[b_i - 1] + xv[i]));
      b = std::abs(0.5 * (yv[b_i - 1] + yv[i]));
      if ((a > 1.0) && (b > 1.0)) {
        a *= b;
      } else if ((b > a) || std::isnan(a)) {
        a = b;
      }
      scale[b_i - 1] = a * 6.6613381477509392E-16;
    }
    for (j = 0; j < 15311; j++) {
      a = x[j];
      b = y[j];
      in[j] = false;
      on[j] = false;
      if ((a >= minxv_tmp) && (a <= maxxv) && (b >= minyv_tmp) &&
          (b <= maxyv)) {
        signed char sdq;
        sdq = 0;
        k = 0;
        int exitg3;
        do {
          exitg3 = 0;
          if (k <= nloops) {
            double xv2;
            double xvFirst;
            double yv2;
            double yvFirst;
            int exitg2;
            signed char quad2;
            signed char quadFirst;
            i = first[k] - 1;
            xvFirst = xv[i] - a;
            yvFirst = yv[i] - b;
            if (xvFirst > 0.0) {
              if (yvFirst > 0.0) {
                quadFirst = 0;
              } else {
                quadFirst = 3;
              }
            } else if (yvFirst > 0.0) {
              quadFirst = 1;
            } else {
              quadFirst = 2;
            }
            xv2 = xvFirst;
            yv2 = yvFirst;
            quad2 = quadFirst;
            i = first[k];
            do {
              exitg2 = 0;
              if (i <= last[k] - 1) {
                double xv1;
                double yv1;
                signed char quad1;
                xv1 = xv2;
                yv1 = yv2;
                xv2 = xv[i] - a;
                yv2 = yv[i] - b;
                quad1 = quad2;
                if (xv2 > 0.0) {
                  if (yv2 > 0.0) {
                    quad2 = 0;
                  } else {
                    quad2 = 3;
                  }
                } else if (yv2 > 0.0) {
                  quad2 = 1;
                } else {
                  quad2 = 2;
                }
                contrib(xv1, yv1, xv2, yv2, quad1, quad2, scale[i - 1], &dquad,
                        &onj);
                on[j] = onj;
                if (onj) {
                  in[j] = true;
                  exitg2 = 1;
                } else {
                  sdq = static_cast<signed char>(sdq + dquad);
                  i++;
                }
              } else {
                contrib(xv2, yv2, xvFirst, yvFirst, quad2, quadFirst,
                        scale[last[k] - 1], &dquad, &onj);
                on[j] = onj;
                exitg2 = 2;
              }
            } while (exitg2 == 0);
            if (exitg2 == 1) {
              exitg3 = 1;
            } else if (onj) {
              in[j] = true;
              exitg3 = 1;
            } else {
              sdq = static_cast<signed char>(sdq + dquad);
              k++;
            }
          } else {
            in[j] = (sdq != 0);
            exitg3 = 1;
          }
        } while (exitg3 == 0);
      }
    }
  }
}

} // namespace coder

//
// File trailer for inpolygon.cpp
//
// [EOF]
//
