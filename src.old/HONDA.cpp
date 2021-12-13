//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: HONDA.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "HONDA.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// HONDA calculates Honda's time to collision algorithm in ROI.
//
//  HONDA_out = HONDA(rel_pos_x, rel_pos_y, rel_vel_x, ROI)
//  rel_pos_x {double} : Relative longitudinal position (m)
//  rel_pos_y {double} : Relative lateral position (m)
//  rel_vel_x {double} : Relative longitudinal velocity (m/s)
//  HONDA_PARAM {struct} : Parameters for calculation of Honda algorithm
//                        HONDA_PARAM.ROI.Y_MIN : minimum relative lateral
//                        position of ROI HONDA_PARAM.ROI.Y_MAX : maximum
//                        relative lateral position of ROI HONDA_PARAM.ROI.X_MIN
//                        : minimum relative longitudinal position of ROI
//                        HONDA_PARAM.ROI.X_MAX : maximum relative longitudinal
//                        position of ROI HONDA_PARAM.HONDA_MIN   : default
//                        value for exception
//
// Arguments    : double rel_pos_x
//                double rel_pos_y
//                double rel_vel_x
//                double Va
//                double Vb
//                double *HONDA_w
//                double *HONDA_br
// Return Type  : void
//
void HONDA(double rel_pos_x, double rel_pos_y, double rel_vel_x, double Va,
           double Vb, double *HONDA_w, double *HONDA_br)
{
  if ((rel_pos_y >= -2.0) && (rel_pos_y <= 2.0) && (rel_pos_x >= -1.0) &&
      (rel_pos_x <= 120.0)) {
    *HONDA_w = -2.2 * rel_vel_x + 6.2;
  } else {
    *HONDA_w = 1.0;
  }
  if (Va >= 11.67) {
    *HONDA_br = (0.2 * -rel_vel_x - 0.20000000000000004) - 0.05000000000000001;
  } else {
    *HONDA_br = (0.2 * Va - 0.05000000000000001) - Vb * Vb / 20.0;
  }
  if (Vb < -0.1) {
    *HONDA_br = (0.2 * Va - 0.05000000000000001) + Vb * Vb / 20.0;
  }
  if (*HONDA_br < 0.0) {
    *HONDA_br = 0.0;
  }
}

//
// File trailer for HONDA.cpp
//
// [EOF]
//
