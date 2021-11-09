//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CV_MODEL.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Nov-2021 00:41:02
//

// Include Files
#include "CV_MODEL.h"
#include "BEV_image_data.h"
#include "det.h"
#include "inv.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "sqrtm.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double x_cv_k[5]
//                const double P_cv_k[25]
//                const double y_k[3]
//                double x_ini[5]
//                double b_flag
//                double old_flag
//                double Q_CV_IMM[25]
//                const double R_CV_IMM[9]
//                double ts
// Return Type  : double
//
double CV_MODEL(const double x_cv_k[5], const double P_cv_k[25],
                const double y_k[3], double x_ini[5], double b_flag,
                double old_flag, double Q_CV_IMM[25], const double R_CV_IMM[9],
                double ts)
{
  creal_T dcv[25];
  double X_k[55];
  double X_k_hat[55];
  double Y_k_hat[33];
  double L[25];
  double K[15];
  double P_xz[15];
  double W[11];
  double P_zz[9];
  double dv1[9];
  double x_k_hat[5];
  double x_kk[5];
  double dv[3];
  double y_k_hat[3];
  double d;
  double d1;
  double mu_cv;
  double x_kk_tmp;
  int L_tmp;
  int b_i;
  int i;
  int i1;
  //  if time >=18.49
  //      time = time;
  //  end
  //  x_k = [y x yaw v yawrate]';
  // -------------------------------------------------------------------------
  //  Parameter
  //  L = real(sqrtm((n_x+KAPPA)*P_cv_k));
  for (i = 0; i < 25; i++) {
    L[i] = 10.0 * P_cv_k[i];
  }
  coder::sqrtm(L, dcv);
  for (i = 0; i < 25; i++) {
    L[i] = dcv[i].re;
  }
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  //  Sampling
  std::memset(&X_k[0], 0, 55U * sizeof(double));
  for (b_i = 0; b_i < 11; b_i++) {
    W[b_i] = 0.0;
    if (b_i == 0) {
      for (i = 0; i < 5; i++) {
        X_k[i] = x_cv_k[i];
      }
      W[0] = 0.5;
    } else if (b_i <= 5) {
      for (i = 0; i < 5; i++) {
        X_k[i + 5 * b_i] = x_cv_k[i] + L[(b_i + 5 * i) - 1];
      }
      W[b_i] = 0.05;
    } else {
      for (i = 0; i < 5; i++) {
        X_k[i + 5 * b_i] = x_cv_k[i] - L[(b_i + 5 * i) - 6];
      }
      W[b_i] = 0.05;
    }
  }
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  //  Model prediction
  for (b_i = 0; b_i < 5; b_i++) {
    x_k_hat[b_i] = 0.0;
  }
  std::memset(&L[0], 0, 25U * sizeof(double));
  x_kk[4] = 0.0;
  for (b_i = 0; b_i < 11; b_i++) {
    //  x_k = [Y X Yaw v yawrate]';
    d = X_k[5 * b_i + 3];
    d1 = X_k[5 * b_i + 2];
    x_kk_tmp = d * ts;
    x_kk[0] = X_k[5 * b_i] + x_kk_tmp * std::sin(d1);
    x_kk[1] = X_k[5 * b_i + 1] + x_kk_tmp * std::cos(d1);
    x_kk[2] = d1;
    x_kk[3] = d;
    for (i = 0; i < 5; i++) {
      d = x_kk[i];
      X_k_hat[i + 5 * b_i] = d;
      x_k_hat[i] += W[b_i] * d;
    }
  }
  for (b_i = 0; b_i < 11; b_i++) {
    for (i = 0; i < 5; i++) {
      x_kk[i] = X_k_hat[i + 5 * b_i] - x_k_hat[i];
    }
    for (i = 0; i < 5; i++) {
      for (i1 = 0; i1 < 5; i1++) {
        L_tmp = i1 + 5 * i;
        L[L_tmp] += W[b_i] * x_kk[i1] * x_kk[i];
      }
    }
  }
  for (i = 0; i < 25; i++) {
    L[i] += Q_CV_IMM[i];
  }
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  //  Measurement update
  y_k_hat[0] = 0.0;
  y_k_hat[1] = 0.0;
  y_k_hat[2] = 0.0;
  std::memset(&P_zz[0], 0, 9U * sizeof(double));
  std::memset(&P_xz[0], 0, 15U * sizeof(double));
  for (b_i = 0; b_i < 11; b_i++) {
    for (i = 0; i < 3; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 5; i1++) {
        d += static_cast<double>(iv[i + 3 * i1]) * X_k[i1 + 5 * b_i];
      }
      Y_k_hat[i + 3 * b_i] = d;
      y_k_hat[i] += W[b_i] * d;
    }
  }
  for (b_i = 0; b_i < 11; b_i++) {
    dv[0] = Y_k_hat[3 * b_i] - y_k_hat[0];
    dv[1] = Y_k_hat[3 * b_i + 1] - y_k_hat[1];
    dv[2] = Y_k_hat[3 * b_i + 2] - y_k_hat[2];
    x_kk_tmp = W[b_i];
    for (i = 0; i < 3; i++) {
      P_zz[3 * i] += x_kk_tmp * dv[0] * dv[i];
      L_tmp = 3 * i + 1;
      P_zz[L_tmp] += x_kk_tmp * dv[1] * dv[i];
      L_tmp = 3 * i + 2;
      P_zz[L_tmp] += x_kk_tmp * dv[2] * dv[i];
    }
    for (i = 0; i < 5; i++) {
      x_kk[i] = x_kk_tmp * (X_k_hat[i + 5 * b_i] - x_k_hat[i]);
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 5; i1++) {
        L_tmp = i1 + 5 * i;
        P_xz[L_tmp] += x_kk[i1] * dv[i];
      }
    }
  }
  for (i = 0; i < 9; i++) {
    P_zz[i] += R_CV_IMM[i];
  }
  coder::internal::mrdiv(P_xz, P_zz, K);
  y_k_hat[0] = y_k[0] - y_k_hat[0];
  y_k_hat[1] = y_k[1] - y_k_hat[1];
  y_k_hat[2] = y_k[2] - y_k_hat[2];
  coder::inv(P_zz, dv1);
  d = 0.0;
  for (i = 0; i < 3; i++) {
    d +=
        ((-0.5 * y_k_hat[0] * dv1[3 * i] + -0.5 * y_k_hat[1] * dv1[3 * i + 1]) +
         -0.5 * y_k_hat[2] * dv1[3 * i + 2]) *
        y_k_hat[i];
  }
  mu_cv = 1.0 / std::sqrt(248.05021344239853 * coder::det(P_zz)) * std::exp(d);
  //  P_kk = P_k_hat - K*S*K';
  // -------------------------------------------------------------------------
  if ((b_flag == 0.0) && (old_flag == 1.0)) {
    for (b_i = 0; b_i < 5; b_i++) {
      x_ini[b_i] = 0.0;
    }
  } else if ((!(b_flag == 1.0)) || (!(old_flag == 0.0))) {
    if (b_flag != 0.0) {
      for (i = 0; i < 5; i++) {
        d = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          L_tmp = i + 5 * i1;
          d += K[L_tmp] * y_k_hat[i1];
          P_xz[L_tmp] = (K[i] * P_zz[3 * i1] + K[i + 5] * P_zz[3 * i1 + 1]) +
                        K[i + 10] * P_zz[3 * i1 + 2];
        }
        x_ini[i] = x_k_hat[i] + d;
        d = P_xz[i];
        d1 = P_xz[i + 5];
        x_kk_tmp = P_xz[i + 10];
        for (i1 = 0; i1 < 5; i1++) {
          L_tmp = i + 5 * i1;
          Q_CV_IMM[L_tmp] =
              L[L_tmp] - ((d * K[i1] + d1 * K[i1 + 5]) + x_kk_tmp * K[i1 + 10]);
        }
      }
    } else {
      for (b_i = 0; b_i < 5; b_i++) {
        x_ini[b_i] = 0.0;
      }
    }
  }
  // -------------------------------------------------------------------------
  //  if time >=63.230
  //  figure(1); hold on
  //  cla;
  //  plot(x_kk(1),x_kk(2),'xr');
  //  plot(x_k(1),x_k(2),'xk')
  //  plot([x_kk(1) x_k(1)],[x_kk(2) x_k(2)])
  //  plot(y_k(1),y_k(2),'or')
  //  axis([-6 6 -5 10])
  //  axis equal
  //  drawnow
  //      time = time;
  //  end
  return mu_cv;
}

//
// File trailer for CV_MODEL.cpp
//
// [EOF]
//
