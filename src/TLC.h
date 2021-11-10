//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: TLC.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

#ifndef TLC_H
#define TLC_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void TLC(double rel_pos_y, double rel_vel_y, double heading_angle,
         double target_width, double target_length, double distance_to_leftlane,
         double distance_to_rightlane, double *TLC_out, double *DLC_out);

#endif
//
// File trailer for TLC.h
//
// [EOF]
//
