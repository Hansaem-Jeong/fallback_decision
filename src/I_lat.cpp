//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: I_lat.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "I_lat.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// I_lat Calculates lateral collision index.
//
//  I_lat_out = I_lat(rel_pos_x, rel_pos_y, rel_vel_x, rel_vel_y,
//  heading_angle,...
//                    target_width, target_length, ego_length,
//                    distance_to_leftlane, distance_to_rightlane, I_LAT_PARAM)
//  rel_pos_x {double}             : Relative longitudinal position (m)
//  rel_pos_y {double}             : Relative lateral position (m)
//  rel_vel_x {double}             : Relative longitudinal velocity (m/s)
//  rel_vel_y {double}             : Relative lateral velocity (m/s)
//  heading_angle {double}         : Relative heading angle (rad)
//  target_width {double}          : width of target vehicle (m)
//  target_length {double}         : length of target vehicle (m)
//  ego_length {double}            : length of ego vehicle (m)
//  distance_to_leftlane {double}  : distance between ego vehicle center to left
//  lane (m) distance_to_rightlane {double} : distance between ego vehicle
//  center to right lane (m)
//
//  I_LAT_PARAM {struct}           : Parameters for calculation of lateral
//  collision index
//                                    I_LAT_PARAM.TLC_THRESHOLD : minimum
//                                    relative lateral position of ROI
//                                    I_LAT_PARAM.T_THINKING : the delay in
//                                    human response between recognition and
//                                    manipulation (s) I_LAT_PARAM.T_BRAKE : the
//                                    system delay (s) I_LAT_PARAM.A_X_MAX :
//                                    maximum deceleration of vehicle(negative
//                                    number) (m/s^2) I_LAT_PARAM.X_THRESHOLD :
//                                    if warning index below threshold, it
//                                    indicates current situation is dangerous
//                                    I_LAT_PARAM.TTC_INVERSE_THRESHOLD : if TTC
//                                    inverse beyond threshold threshold, it
//                                    indecates current situation is dangerous
//                                    I_LAT_PARAM.P_LONG_DEFAULT_VALUE : default
//                                    value for initialization
//                                    I_LAT_PARAM.TTC_MAX : default value for
//                                    exception I_LAT_PARAM.TLC_MAX : default
//                                    value for exception
//
//  Reference : K. Kim, “Predicted potential risk-based vehicle motion control
//  of automated vehicles for integrated risk management,” Doctoral
//  Dissertation, Seoul National University Graduate, 2016.
//
// Arguments    : double rel_pos_x
//                double rel_pos_y
//                double rel_vel_x
//                double rel_vel_y
//                double Va
//                double Vb
//                double heading_angle
//                double target_width
//                double target_length
//                double ego_length
//                double distance_to_leftlane
//                double distance_to_rightlane
//                double *I_lat_out
//                double *I_long
//                double *TTC
//                double *x_p
//                double *d_br
//                double *d_w
//                double *DLC
//                double *b_TLC
// Return Type  : void
//
void I_lat(double rel_pos_x, double rel_pos_y, double rel_vel_x,
           double rel_vel_y, double Va, double Vb, double heading_angle,
           double target_width, double target_length, double ego_length,
           double distance_to_leftlane, double distance_to_rightlane,
           double *I_lat_out, double *I_long, double *TTC, double *x_p,
           double *d_br, double *d_w, double *DLC, double *b_TLC)
{
  double b_d_br_tmp;
  double b_d_br_tmp_tmp;
  double d_br_tmp;
  double d_br_tmp_tmp;
  double d_w_tmp;
  double p_long;
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  parameter in algorithm
  //  X_THRESHOLD=10;
  //  parameter for initialization/exception
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  //  longitudinal collision index
  // --------------------------------------------------------------------------
  //  calculate the clearance between two vehicles(p_long)
  if (rel_pos_x >= 0.0) {
    p_long = rel_pos_x;
  } else {
    //  exception for adjacent and rear vehicle
    p_long = (rel_pos_x + ego_length) + target_length;
  }
  //  calculate TTC(time to collision) : TTC positive = dangerous, TTC negative
  //  = safe
  *TTC = 0.0;
  if (p_long != 300.0) {
    if (rel_vel_x < 0.0) {
      if (((rel_pos_x > 0.0) && (p_long > 0.0)) ||
          ((rel_pos_x < 0.0) && (p_long < 0.0))) {
        //  TTC for front vehicle || TTC for rear vehicle
        *TTC = p_long / -rel_vel_x;
      }
    } else {
      //  prevent infinite TTC
      *TTC = 11.0;
      if ((rel_pos_x < 0.0) && (p_long < 0.0)) {
        //  exception : RT_LFL2R_IN rear collision
        *TTC = p_long / -rel_vel_x;
      }
      if ((rel_pos_x <= 0.0) && (p_long > 0.0)) {
        //  exception : TTC for adjacent vehicle PSD-22-13(1020)
        //  LK_CIR_MER_data_2
        *TTC = 0.0;
      }
    }
  } else {
    *TTC = 11.0;
  }
  if (*TTC > 11.0) {
    *TTC = 11.0;
  }
  //  calculate warning braking critical distance(d_br)
  d_br_tmp = -rel_vel_x * 0.1;
  d_br_tmp_tmp = Vb * Vb;
  b_d_br_tmp_tmp = Va * Va;
  b_d_br_tmp = (b_d_br_tmp_tmp - d_br_tmp_tmp) / -20.0;
  *d_br = d_br_tmp - b_d_br_tmp;
  //  calculate warning critical distance(d_w)
  d_w_tmp = d_br_tmp + d_br_tmp;
  *d_w = d_w_tmp - b_d_br_tmp;
  if (Vb < 0.0) {
    b_d_br_tmp = (d_br_tmp_tmp + b_d_br_tmp_tmp) / -20.0;
    *d_br = d_br_tmp + b_d_br_tmp;
    *d_w = d_w_tmp + b_d_br_tmp;
  }
  //  calculate warning index(x_p)
  *x_p = (p_long - *d_br) / (*d_w - *d_br);
  if ((*x_p > 4.5) || (Va - Vb < 0.0)) {
    *x_p = 4.5;
  }
  if (*x_p < 0.0) {
    *x_p = 0.0;
  }
  //  calculate collision risk index(I_long)
  *I_long = std::fmax((4.5 - *x_p) / 10.5, std::abs(1.0 / *TTC) / 2.0);
  if (*I_long > 1.0) {
    *I_long = 1.0;
  }
  // --------------------------------------------------------------------------
  //  lateral collision index
  // --------------------------------------------------------------------------
  //  calculate DLC
  if (rel_pos_y >= distance_to_leftlane) {
    //  outside of leftlane : calculate DLC, DLC > 0
    b_d_br_tmp = heading_angle;
    if (heading_angle < 0.0) {
      b_d_br_tmp = -1.0;
    } else if (heading_angle > 0.0) {
      b_d_br_tmp = 1.0;
    } else if (heading_angle == 0.0) {
      b_d_br_tmp = 0.0;
    }
    *DLC = (rel_pos_y -
            (target_length / 2.0 * std::sin(b_d_br_tmp * heading_angle) +
             target_width / 2.0 * std::cos(heading_angle))) -
           distance_to_leftlane;
    if (*DLC < 0.0) {
      *DLC = 0.0;
    }
  } else if (rel_pos_y <= distance_to_rightlane) {
    //  outside of rightlane : calculate DLC, DLC < 0
    b_d_br_tmp = heading_angle;
    if (heading_angle < 0.0) {
      b_d_br_tmp = -1.0;
    } else if (heading_angle > 0.0) {
      b_d_br_tmp = 1.0;
    } else if (heading_angle == 0.0) {
      b_d_br_tmp = 0.0;
    }
    *DLC = (rel_pos_y +
            (target_length / 2.0 * std::sin(b_d_br_tmp * heading_angle) +
             target_width / 2.0 * std::cos(heading_angle))) -
           distance_to_rightlane;
    if (*DLC > 0.0) {
      *DLC = 0.0;
    }
  } else {
    //  in-lane : do not calculate DLC
    *DLC = 0.0;
  }
  //  calculate TLC
  if (rel_vel_y != 0.0) {
    if (std::abs(rel_vel_y) > 0.01) {
      if (*DLC != 0.0) {
        *b_TLC = -*DLC / rel_vel_y;
        if (*b_TLC < 0.0) {
          *b_TLC = 11.0;
        }
      } else {
        //  exception : In case of DLC = 0, 1/TLC is -inf, then
        //  min(TLC_THRESHOLD/TLC,1) is -inf
        *b_TLC = 0.0;
      }
    } else {
      //  exception : In case of small value of rel vel y, TLC has too big value
      *b_TLC = 11.0;
      if (*DLC == 0.0) {
        *b_TLC = 0.0;
      }
    }
  } else if (*DLC == 0.0) {
    *b_TLC = 0.0;
  } else {
    //  prevent infinite TLC
    *b_TLC = 11.0;
  }
  //  out of FOV
  //  if abs(rel_pos_y)>2
  //      TLC=TLC_MAX;
  //  end
  //  if abs(DLC)>2||rel_vel_x > -0.2
  //      I_long=0;
  //  end
  //  calculate lateral collision index
  *I_lat_out = std::fmin(*I_long, 1.0) * std::fmin(0.5 / *b_TLC, 1.0);
}

//
// File trailer for I_lat.cpp
//
// [EOF]
//
