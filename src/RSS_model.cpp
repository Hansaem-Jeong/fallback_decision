//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RSS_model.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 03-Nov-2021 20:39:09
//

// Include Files
#include "RSS_model.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Responsibility-Sensitive Safety(RSS) model
//
//  RSS_model - Calulate the safe longitudinal/lateral distance.
//  [RSS_x, RSS_y] = RSS_model(Rel_x,Rel_y,vxA,vxB,vyA,vyB,RSS_Param)
//
//  RSS_model Input
//
//  Rel_x : Relative longitudinal position (m)
//  Rel_y : Relative lateral position (m)
//  vxA : Longitudinal velocity of ego vehicle (m/s)
//  vxB : Longitudinal velocity of target vehicle (m/s)
//  vyA : Lateral velocity of ego vehicle (m/s)
//  vyB : Lateral velocity of target vehicle (m/s)
//
//  RSS_Param {struct} : Parameters of RSS_model
//                       AMAX_XA  : Maximum long acc of ego veh.
//                       AMIN_XA  : Minimum long decc of ego veh.
//                       AMAX_XB  : Maximum long acc of target veh.
//                       RHO_X    : Response time.
//                       AMAX_Y   : Maximum lat acc of veh.
//                       AMIN_Y   : Minimum lat decc of veh.
//                       F        : Fluctutation margin.
//                       RHO_Y    : Response time.
//                       L_F      : Distance of the front tire from the c.g. of
//                       the veh. W        : Width of ego veh.
//
//  RSS_model output
//  RSS_x : Safe longitudinal distance between ego veh and target veh.
//  RSS_y : Safe lateral distance between ego veh and target veh.
//
// Arguments    : double Rel_x
//                double Rel_y
//                double vxA
//                double vxB
//                double vyA
//                double vyB
//                double *RSS_x
//                double *RSS_y
// Return Type  : void
//
void RSS_model(double Rel_x, double Rel_y, double vxA, double vxB, double vyA,
               double vyB, double *RSS_x, double *RSS_y)
{
  double RSS_x_tmp;
  double a;
  double d0_y;
  //  Reference : Shai Shalev-Shwartz et al, “On a Formal Model of Safe and
  //  Scalable Self-driving Cars,” https://arxiv.org/abs/1708.06374, Mobileye,
  //  2017.
  // %%%%%%%%%%%%%%%%%%%%% Parameter %%%%%%%%%%%%%%%%%%%%%%%%
  // %%%%%%%%%%%%%%%%%%%%% long model %%%%%%%%%%%%%%%%%%%%%%%%
  d0_y = vxB * vxB / 20.0;
  RSS_x_tmp = vxA * 0.1 + vxA * vxA / 20.0;
  *RSS_x = (Rel_x - d0_y) + RSS_x_tmp;
  if (vxB < -0.1) {
    *RSS_x = (Rel_x + d0_y) + RSS_x_tmp;
  }
  if (*RSS_x < -10.0) {
    *RSS_x = -10.0;
  }
  if (*RSS_x > 20.0) {
    *RSS_x = 20.0;
  }
  // %%%%%%%%%%%%%%%%%%%%% lat model %%%%%%%%%%%%%%%%%%%%%%%%%
  //  if Rel_y>=0
  //      vyB=-vyB;
  //  end
  d0_y = Rel_y - 1.7;
  if (Rel_y < 0.0) {
    d0_y = -Rel_y - 1.7;
  }
  if (d0_y < 0.0) {
    d0_y = 0.0;
  }
  RSS_x_tmp = vyA + vyA;
  a = -2.0 * vyB;
  *RSS_y =
      (d0_y - ((0.3 * (RSS_x_tmp / 2.0) + 0.1) + RSS_x_tmp * RSS_x_tmp / 8.0)) +
      (0.3 * ((vyB + vyB) / 2.0) - a * a / 8.0);
  if (*RSS_y < -10.0) {
    *RSS_y = -10.0;
  }
  if (*RSS_y > 10.0) {
    *RSS_y = 10.0;
  }
}

//
// File trailer for RSS_model.cpp
//
// [EOF]
//
