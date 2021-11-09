//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: TLC.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Nov-2021 00:41:02
//

// Include Files
#include "TLC.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// TLC Calculates Time to Line Crossing.
//
//  [TLC_out, DLC_out] = TLC(rel_pos_y, rel_vel_y, heading_angle,
//  target_width,...
//                            target_length, distance_to_leftlane,
//                            distance_to_rightlane, TLC_PARAM)
//  rel_pos_y {double}             : Relative lateral position (m)
//  rel_vel_y {double}             : Relative lateral velocity (m/s)
//  heading_angle {double}         : Relative heading angle (rad)
//  target_width {double}          : width of target vehicle (m)
//  target_length {double}         : length of target vehicle (m)
//  distance_to_leftlane {double}  : distance between ego vehicle center to left
//  lane (m), positive number distance_to_rightlane {double} : distance between
//  ego vehicle center to right lane (m), negative number
//
//  TLC_PARAM {struct}           : Parameters for calculation of time to line
//  crossing
//                                    TLC_PARAM.TLC_MAX : default value for
//                                    exception
//
//  Reference : Mammar, Said, S?bastien Glaser, and Mariana Netto. "Time to line
//  crossing for lane departure avoidance: A theoretical study and an
//  experimental setting." IEEE Transactions on intelligent transportation
//  systems 7.2 (2006): 226-241.
//
// Arguments    : double rel_pos_y
//                double rel_vel_y
//                double heading_angle
//                double target_width
//                double target_length
//                double distance_to_leftlane
//                double distance_to_rightlane
//                double *TLC_out
//                double *DLC_out
// Return Type  : void
//
void TLC(double rel_pos_y, double rel_vel_y, double heading_angle,
         double target_width, double target_length, double distance_to_leftlane,
         double distance_to_rightlane, double *TLC_out, double *DLC_out)
{
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  parameter in algorithm
  //  parameter for initialization/exception
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  //  calculate DLC
  if (rel_pos_y >= distance_to_leftlane) {
    double x;
    //  outside of leftlane : calculate DLC( DLC > 0 )
    x = heading_angle;
    if (heading_angle < 0.0) {
      x = -1.0;
    } else if (heading_angle > 0.0) {
      x = 1.0;
    } else if (heading_angle == 0.0) {
      x = 0.0;
    }
    *DLC_out = rel_pos_y - ((target_length / 2.0 * std::sin(x * heading_angle) +
                             target_width / 2.0 * std::cos(heading_angle)) +
                            distance_to_leftlane);
    if (*DLC_out < 0.0) {
      *DLC_out = 0.0;
    }
  } else if (rel_pos_y <= distance_to_rightlane) {
    double x;
    //  outside of rightlane : calculate DLC( DLC < 0 )
    x = heading_angle;
    if (heading_angle < 0.0) {
      x = -1.0;
    } else if (heading_angle > 0.0) {
      x = 1.0;
    } else if (heading_angle == 0.0) {
      x = 0.0;
    }
    *DLC_out = rel_pos_y + ((target_length / 2.0 * std::sin(x * heading_angle) +
                             target_width / 2.0 * std::cos(heading_angle)) -
                            distance_to_rightlane);
    if (*DLC_out > 0.0) {
      *DLC_out = 0.0;
    }
  } else {
    //  in-lane : do not calculate DLC
    *DLC_out = 0.0;
  }
  //  calculate TLC
  if (rel_vel_y != 0.0) {
    if (std::abs(rel_vel_y) > 0.01) {
      if (*DLC_out != 0.0) {
        *TLC_out = -*DLC_out / rel_vel_y;
      } else {
        //  exception : In case of DLC = 0, 1/TLC is -inf
        *TLC_out = 0.0;
      }
    } else {
      //  exception : In case of small value of rel vel y, TLC has too big value
      *TLC_out = 11.0;
    }

    //  prevent infinite TLC
  } else if (*DLC_out == 0.0) {
    *TLC_out = 0.0;
  } else {
    *TLC_out = 11.0;
  }
  if (*TLC_out < 0.0) {
    *TLC_out = 11.0;
  }
  if (std::abs(rel_pos_y) < 2.0) {
    *TLC_out = 0.0;
  }
}

//
// File trailer for TLC.cpp
//
// [EOF]
//
