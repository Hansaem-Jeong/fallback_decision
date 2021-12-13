//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sqrtm.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 13-Dec-2021 11:53:25
//

// Include Files
#include "sqrtm.h"
#include "BEV_image_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "xdhseqr.h"
#include "xdlanv2.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[25]
//                creal_T X[25]
// Return Type  : void
//
namespace coder {
void sqrtm(const double A[25], creal_T X[25])
{
  creal_T Q[25];
  creal_T R[25];
  creal_T T[25];
  double Vr[25];
  double b_A[25];
  double work[5];
  double tau[4];
  double a;
  double b_d;
  double cs;
  double d;
  double d_tmp;
  double mu1_im;
  double mu1_re;
  double rt1i;
  double rt1r;
  double s;
  double t1_im;
  double xnorm;
  int b_i;
  int c_i;
  int exitg1;
  int i;
  int i1;
  int j;
  int k;
  int knt;
  int m;
  int work_tmp;
  bool exitg2;
  bool p;
  p = true;
  for (k = 0; k < 25; k++) {
    if ((!p) || (std::isinf(A[k]) || std::isnan(A[k]))) {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 25; i++) {
      Q[i].re = rtNaN;
      Q[i].im = 0.0;
    }
    knt = 3;
    for (j = 0; j < 3; j++) {
      if (knt <= 5) {
        std::memset(&Q[(j * 5 + knt) + -1], 0, (6 - knt) * sizeof(creal_T));
      }
      knt++;
    }
    for (i = 0; i < 25; i++) {
      T[i].re = rtNaN;
      T[i].im = 0.0;
    }
  } else {
    std::copy(&A[0], &A[25], &b_A[0]);
    for (b_i = 0; b_i < 5; b_i++) {
      work[b_i] = 0.0;
    }
    for (b_i = 0; b_i < 4; b_i++) {
      int alpha1_tmp;
      int im1n_tmp;
      int in;
      int iv0;
      int lastc;
      int lastv;
      im1n_tmp = b_i * 5 + 2;
      in = (b_i + 1) * 5;
      alpha1_tmp = (b_i + 5 * b_i) + 1;
      t1_im = b_A[alpha1_tmp];
      if (b_i + 3 < 5) {
        c_i = b_i + 1;
      } else {
        c_i = 3;
      }
      c_i += im1n_tmp;
      tau[b_i] = 0.0;
      xnorm = internal::blas::xnrm2(3 - b_i, b_A, c_i);
      if (xnorm != 0.0) {
        xnorm = rt_hypotd_snf(t1_im, xnorm);
        if (t1_im >= 0.0) {
          xnorm = -xnorm;
        }
        if (std::abs(xnorm) < 1.0020841800044864E-292) {
          knt = -1;
          i = (c_i - b_i) + 2;
          do {
            knt++;
            for (k = c_i; k <= i; k++) {
              b_A[k - 1] *= 9.9792015476736E+291;
            }
            xnorm *= 9.9792015476736E+291;
            t1_im *= 9.9792015476736E+291;
          } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));
          xnorm =
              rt_hypotd_snf(t1_im, internal::blas::xnrm2(3 - b_i, b_A, c_i));
          if (t1_im >= 0.0) {
            xnorm = -xnorm;
          }
          tau[b_i] = (xnorm - t1_im) / xnorm;
          a = 1.0 / (t1_im - xnorm);
          i = (c_i - b_i) + 2;
          for (k = c_i; k <= i; k++) {
            b_A[k - 1] *= a;
          }
          for (k = 0; k <= knt; k++) {
            xnorm *= 1.0020841800044864E-292;
          }
          t1_im = xnorm;
        } else {
          tau[b_i] = (xnorm - t1_im) / xnorm;
          a = 1.0 / (t1_im - xnorm);
          i = (c_i - b_i) + 2;
          for (k = c_i; k <= i; k++) {
            b_A[k - 1] *= a;
          }
          t1_im = xnorm;
        }
      }
      b_A[alpha1_tmp] = 1.0;
      iv0 = (b_i + im1n_tmp) - 1;
      m = in + 1;
      if (tau[b_i] != 0.0) {
        lastv = 3 - b_i;
        c_i = (iv0 - b_i) + 3;
        while ((lastv + 1 > 0) && (b_A[c_i] == 0.0)) {
          lastv--;
          c_i--;
        }
        lastc = 5;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          knt = in + lastc;
          k = knt;
          do {
            exitg1 = 0;
            if (k <= knt + lastv * 5) {
              if (b_A[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k += 5;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        if (lastc != 0) {
          if (0 <= lastc - 1) {
            std::memset(&work[0], 0, lastc * sizeof(double));
          }
          knt = iv0;
          i = (in + 5 * lastv) + 1;
          for (c_i = m; c_i <= i; c_i += 5) {
            i1 = (c_i + lastc) - 1;
            for (k = c_i; k <= i1; k++) {
              work_tmp = k - c_i;
              work[work_tmp] += b_A[k - 1] * b_A[knt];
            }
            knt++;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          knt = in;
          for (j = 0; j <= lastv; j++) {
            d = b_A[iv0 + j];
            if (d != 0.0) {
              xnorm = d * -tau[b_i];
              i = knt + 1;
              i1 = lastc + knt;
              for (c_i = i; c_i <= i1; c_i++) {
                b_A[c_i - 1] += work[(c_i - knt) - 1] * xnorm;
              }
            }
            knt += 5;
          }
        }
      }
      internal::reflapack::xzlarf(4 - b_i, 4 - b_i, b_i + im1n_tmp, tau[b_i],
                                  b_A, (b_i + in) + 2, work);
      b_A[alpha1_tmp] = t1_im;
    }
    std::copy(&b_A[0], &b_A[25], &Vr[0]);
    for (j = 3; j >= 0; j--) {
      k = (j + 1) * 5;
      for (b_i = 0; b_i <= j; b_i++) {
        Vr[k + b_i] = 0.0;
      }
      i = j + 3;
      for (b_i = i; b_i < 6; b_i++) {
        knt = k + b_i;
        Vr[knt - 1] = Vr[knt - 6];
      }
    }
    for (b_i = 0; b_i < 5; b_i++) {
      Vr[b_i] = 0.0;
    }
    Vr[0] = 1.0;
    for (b_i = 0; b_i < 5; b_i++) {
      work[b_i] = 0.0;
    }
    for (b_i = 3; b_i >= 0; b_i--) {
      knt = (b_i + b_i * 5) + 6;
      if (b_i + 1 < 4) {
        Vr[knt] = 1.0;
        internal::reflapack::xzlarf(4 - b_i, 3 - b_i, knt + 1, tau[b_i], Vr,
                                    knt + 6, work);
        c_i = knt + 2;
        i = (knt - b_i) + 4;
        for (k = c_i; k <= i; k++) {
          Vr[k - 1] *= -tau[b_i];
        }
      }
      Vr[knt] = 1.0 - tau[b_i];
      for (j = 0; j < b_i; j++) {
        Vr[(knt - j) - 1] = 0.0;
      }
    }
    internal::reflapack::eml_dlahqr(b_A, Vr);
    knt = 4;
    for (j = 0; j < 2; j++) {
      if (knt <= 5) {
        std::memset(&b_A[(j * 5 + knt) + -1], 0, (6 - knt) * sizeof(double));
      }
      knt++;
    }
    for (i = 0; i < 25; i++) {
      T[i].re = b_A[i];
      T[i].im = 0.0;
      Q[i].re = Vr[i];
      Q[i].im = 0.0;
    }
    for (m = 3; m >= 0; m--) {
      c_i = m + 1;
      i = m + 5 * m;
      d = b_A[i + 1];
      if (d != 0.0) {
        a = b_A[i];
        k = 5 * (m + 1);
        knt = m + k;
        xnorm = b_A[knt];
        t1_im = d;
        d_tmp = b_A[knt + 1];
        b_d = d_tmp;
        internal::reflapack::xdlanv2(&a, &xnorm, &t1_im, &b_d, &rt1r, &rt1i,
                                     &mu1_im, &mu1_re, &cs, &s);
        mu1_re = rt1r - d_tmp;
        xnorm = rt_hypotd_snf(rt_hypotd_snf(mu1_re, rt1i), d);
        if (rt1i == 0.0) {
          a = mu1_re / xnorm;
          cs = 0.0;
        } else if (mu1_re == 0.0) {
          a = 0.0;
          cs = rt1i / xnorm;
        } else {
          a = mu1_re / xnorm;
          cs = rt1i / xnorm;
        }
        s = d / xnorm;
        for (j = c_i; j < 6; j++) {
          work_tmp = m + 5 * (j - 1);
          xnorm = T[work_tmp].re;
          t1_im = T[work_tmp].im;
          d_tmp = T[work_tmp + 1].re;
          rt1r = T[work_tmp + 1].im;
          T[work_tmp].re = (a * xnorm + cs * t1_im) + s * d_tmp;
          T[work_tmp].im = (a * t1_im - cs * xnorm) + s * rt1r;
          mu1_re = a * d_tmp - cs * rt1r;
          mu1_im = a * rt1r + cs * d_tmp;
          T[work_tmp + 1].re = mu1_re - s * xnorm;
          T[work_tmp + 1].im = mu1_im - s * t1_im;
        }
        for (b_i = 0; b_i <= m + 1; b_i++) {
          work_tmp = b_i + 5 * m;
          xnorm = T[work_tmp].re;
          t1_im = T[work_tmp].im;
          c_i = b_i + k;
          d_tmp = T[c_i].re;
          rt1r = T[c_i].im;
          mu1_re = a * xnorm - cs * t1_im;
          mu1_im = a * t1_im + cs * xnorm;
          T[work_tmp].re = mu1_re + s * d_tmp;
          T[work_tmp].im = mu1_im + s * rt1r;
          T[c_i].re = (a * d_tmp + cs * rt1r) - s * xnorm;
          T[c_i].im = (a * rt1r - cs * d_tmp) - s * t1_im;
        }
        for (b_i = 0; b_i < 5; b_i++) {
          work_tmp = b_i + 5 * m;
          xnorm = Q[work_tmp].re;
          t1_im = Q[work_tmp].im;
          c_i = b_i + k;
          d_tmp = Q[c_i].re;
          rt1r = Q[c_i].im;
          mu1_re = a * xnorm - cs * t1_im;
          mu1_im = a * t1_im + cs * xnorm;
          Q[work_tmp].re = mu1_re + s * d_tmp;
          Q[work_tmp].im = mu1_im + s * rt1r;
          Q[c_i].re = (a * d_tmp + cs * rt1r) - s * xnorm;
          Q[c_i].im = (a * rt1r - cs * d_tmp) - s * t1_im;
        }
        T[i + 1].re = 0.0;
        T[i + 1].im = 0.0;
      }
    }
  }
  std::memset(&R[0], 0, 25U * sizeof(creal_T));
  j = -1;
  int exitg3;
  do {
    exitg3 = 0;
    if (j + 1 < 5) {
      b_i = 0;
      do {
        exitg1 = 0;
        if (b_i <= j) {
          knt = b_i + 5 * (j + 1);
          if ((T[knt].re != 0.0) || (T[knt].im != 0.0)) {
            for (j = 0; j < 5; j++) {
              m = j + 5 * j;
              R[m] = T[m];
              b_sqrt(&R[j + 5 * j]);
              for (b_i = j; b_i >= 1; b_i--) {
                mu1_re = 0.0;
                mu1_im = 0.0;
                i = b_i + 1;
                for (k = i; k <= j; k++) {
                  knt = (b_i + 5 * (k - 1)) - 1;
                  work_tmp = (k + 5 * j) - 1;
                  mu1_re +=
                      R[knt].re * R[work_tmp].re - R[knt].im * R[work_tmp].im;
                  mu1_im +=
                      R[knt].re * R[work_tmp].im + R[knt].im * R[work_tmp].re;
                }
                knt = (b_i + 5 * j) - 1;
                cs = T[knt].re - mu1_re;
                mu1_im = T[knt].im - mu1_im;
                c_i = (b_i + 5 * (b_i - 1)) - 1;
                t1_im = R[c_i].re + R[m].re;
                d_tmp = R[c_i].im + R[m].im;
                if (d_tmp == 0.0) {
                  if (mu1_im == 0.0) {
                    a = cs / t1_im;
                    cs = 0.0;
                  } else if (cs == 0.0) {
                    a = 0.0;
                    cs = mu1_im / t1_im;
                  } else {
                    a = cs / t1_im;
                    cs = mu1_im / t1_im;
                  }
                } else if (t1_im == 0.0) {
                  if (cs == 0.0) {
                    a = mu1_im / d_tmp;
                    cs = 0.0;
                  } else if (mu1_im == 0.0) {
                    a = 0.0;
                    cs = -(cs / d_tmp);
                  } else {
                    a = mu1_im / d_tmp;
                    cs = -(cs / d_tmp);
                  }
                } else {
                  rt1r = std::abs(t1_im);
                  xnorm = std::abs(d_tmp);
                  if (rt1r > xnorm) {
                    s = d_tmp / t1_im;
                    b_d = t1_im + s * d_tmp;
                    a = (cs + s * mu1_im) / b_d;
                    cs = (mu1_im - s * cs) / b_d;
                  } else if (xnorm == rt1r) {
                    if (t1_im > 0.0) {
                      t1_im = 0.5;
                    } else {
                      t1_im = -0.5;
                    }
                    if (d_tmp > 0.0) {
                      xnorm = 0.5;
                    } else {
                      xnorm = -0.5;
                    }
                    a = (cs * t1_im + mu1_im * xnorm) / rt1r;
                    cs = (mu1_im * t1_im - cs * xnorm) / rt1r;
                  } else {
                    s = t1_im / d_tmp;
                    b_d = d_tmp + s * t1_im;
                    a = (s * cs + mu1_im) / b_d;
                    cs = (s * mu1_im - cs) / b_d;
                  }
                }
                R[knt].re = a;
                R[knt].im = cs;
              }
            }
            exitg1 = 1;
          } else {
            b_i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg3 = 1;
      }
    } else {
      for (j = 0; j < 5; j++) {
        m = j + 5 * j;
        R[m] = T[m];
        b_sqrt(&R[j + 5 * j]);
      }
      exitg3 = 1;
    }
  } while (exitg3 == 0);
  for (i = 0; i < 5; i++) {
    for (i1 = 0; i1 < 5; i1++) {
      a = 0.0;
      cs = 0.0;
      for (work_tmp = 0; work_tmp < 5; work_tmp++) {
        knt = i + 5 * work_tmp;
        c_i = work_tmp + 5 * i1;
        d = Q[knt].re;
        d_tmp = Q[knt].im;
        xnorm = R[c_i].re;
        t1_im = R[c_i].im;
        a += d * xnorm - d_tmp * t1_im;
        cs += d * t1_im + d_tmp * xnorm;
      }
      work_tmp = i + 5 * i1;
      T[work_tmp].re = a;
      T[work_tmp].im = cs;
    }
    for (i1 = 0; i1 < 5; i1++) {
      a = 0.0;
      cs = 0.0;
      for (work_tmp = 0; work_tmp < 5; work_tmp++) {
        knt = i1 + 5 * work_tmp;
        xnorm = Q[knt].re;
        t1_im = -Q[knt].im;
        knt = i + 5 * work_tmp;
        d = T[knt].re;
        d_tmp = T[knt].im;
        a += d * xnorm - d_tmp * t1_im;
        cs += d * t1_im + d_tmp * xnorm;
      }
      work_tmp = i + 5 * i1;
      X[work_tmp].re = a;
      X[work_tmp].im = cs;
    }
  }
  for (i = 0; i < 25; i++) {
    b_A[i] = X[i].im;
  }
  xnorm = 0.0;
  j = 0;
  exitg2 = false;
  while ((!exitg2) && (j < 5)) {
    s = 0.0;
    for (b_i = 0; b_i < 5; b_i++) {
      s += std::abs(b_A[b_i + 5 * j]);
    }
    if (std::isnan(s)) {
      xnorm = rtNaN;
      exitg2 = true;
    } else {
      if (s > xnorm) {
        xnorm = s;
      }
      j++;
    }
  }
  t1_im = 0.0;
  j = 0;
  exitg2 = false;
  while ((!exitg2) && (j < 5)) {
    s = 0.0;
    for (b_i = 0; b_i < 5; b_i++) {
      knt = b_i + 5 * j;
      s += rt_hypotd_snf(X[knt].re, X[knt].im);
    }
    if (std::isnan(s)) {
      t1_im = rtNaN;
      exitg2 = true;
    } else {
      if (s > t1_im) {
        t1_im = s;
      }
      j++;
    }
  }
  if (xnorm <= 1.1102230246251565E-14 * t1_im) {
    for (j = 0; j < 5; j++) {
      for (b_i = 0; b_i < 5; b_i++) {
        X[b_i + 5 * j].im = 0.0;
      }
    }
  }
}

} // namespace coder

//
// File trailer for sqrtm.cpp
//
// [EOF]
//
