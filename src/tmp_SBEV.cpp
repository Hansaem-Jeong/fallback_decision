//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: tmp_SBEV.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "tmp_SBEV.h"
#include "find.h"
#include "inpolygon.h"
#include "isequal.h"
#include "linspace.h"
#include "meshgrid.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void ab_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 12690];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 12708];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 12870] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[12654] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[12654] / 2.0;
    tmp_target_y[3] = Tmp_State[12654] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[12672];
    tmp_target_x[2] = Tmp_State[12672];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[12780]);
    Image_Position_Y = std::sin(Tmp_State[12780]);
    d = Tmp_State[12690];
    d1 = Tmp_State[12708];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[12870] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[12870] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[12654] / 2.0, Tmp_State[12654] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[12672], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1250];
      Image_Position_X = X_pred[5 * iindx + 1251];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
// AES
//    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
//    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    coder::linspace(-1.82 / 2.0, 1.82 / 2.0, TV_range_y);
    coder::linspace(-4.85, 0.0, TV_range_x);

    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void b_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 594];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 612];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 774] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[558] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[558] / 2.0;
    tmp_target_y[3] = Tmp_State[558] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[576];
    tmp_target_x[2] = Tmp_State[576];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[684]);
    Image_Position_Y = std::sin(Tmp_State[684]);
    d = Tmp_State[594];
    d1 = Tmp_State[612];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[774] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[774] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[558] / 2.0, Tmp_State[558] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[576], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 50];
      Image_Position_X = X_pred[5 * iindx + 51];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void bb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 13194];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 13212];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 13374] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[13158] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[13158] / 2.0;
    tmp_target_y[3] = Tmp_State[13158] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[13176];
    tmp_target_x[2] = Tmp_State[13176];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[13284]);
    Image_Position_Y = std::sin(Tmp_State[13284]);
    d = Tmp_State[13194];
    d1 = Tmp_State[13212];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[13374] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[13374] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[13158] / 2.0, Tmp_State[13158] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[13176], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1300];
      Image_Position_X = X_pred[5 * iindx + 1301];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void c_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 1098];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 1116];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 1278] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[1062] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[1062] / 2.0;
    tmp_target_y[3] = Tmp_State[1062] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[1080];
    tmp_target_x[2] = Tmp_State[1080];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[1188]);
    Image_Position_Y = std::sin(Tmp_State[1188]);
    d = Tmp_State[1098];
    d1 = Tmp_State[1116];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[1278] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[1278] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[1062] / 2.0, Tmp_State[1062] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[1080], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 100];
      Image_Position_X = X_pred[5 * iindx + 101];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void cb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 13698];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 13716];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 13878] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[13662] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[13662] / 2.0;
    tmp_target_y[3] = Tmp_State[13662] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[13680];
    tmp_target_x[2] = Tmp_State[13680];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[13788]);
    Image_Position_Y = std::sin(Tmp_State[13788]);
    d = Tmp_State[13698];
    d1 = Tmp_State[13716];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[13878] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[13878] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[13662] / 2.0, Tmp_State[13662] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[13680], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1350];
      Image_Position_X = X_pred[5 * iindx + 1351];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void d_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 1602];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 1620];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 1782] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[1566] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[1566] / 2.0;
    tmp_target_y[3] = Tmp_State[1566] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[1584];
    tmp_target_x[2] = Tmp_State[1584];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[1692]);
    Image_Position_Y = std::sin(Tmp_State[1692]);
    d = Tmp_State[1602];
    d1 = Tmp_State[1620];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[1782] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[1782] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[1566] / 2.0, Tmp_State[1566] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[1584], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 150];
      Image_Position_X = X_pred[5 * iindx + 151];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void db_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 14202];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 14220];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 14382] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[14166] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[14166] / 2.0;
    tmp_target_y[3] = Tmp_State[14166] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[14184];
    tmp_target_x[2] = Tmp_State[14184];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[14292]);
    Image_Position_Y = std::sin(Tmp_State[14292]);
    d = Tmp_State[14202];
    d1 = Tmp_State[14220];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[14382] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[14382] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[14166] / 2.0, Tmp_State[14166] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[14184], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1400];
      Image_Position_X = X_pred[5 * iindx + 1401];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void e_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 2106];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 2124];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 2286] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[2070] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[2070] / 2.0;
    tmp_target_y[3] = Tmp_State[2070] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[2088];
    tmp_target_x[2] = Tmp_State[2088];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[2196]);
    Image_Position_Y = std::sin(Tmp_State[2196]);
    d = Tmp_State[2106];
    d1 = Tmp_State[2124];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[2286] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[2286] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[2070] / 2.0, Tmp_State[2070] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[2088], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 200];
      Image_Position_X = X_pred[5 * iindx + 201];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void eb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 14706];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 14724];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 14886] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[14670] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[14670] / 2.0;
    tmp_target_y[3] = Tmp_State[14670] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[14688];
    tmp_target_x[2] = Tmp_State[14688];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[14796]);
    Image_Position_Y = std::sin(Tmp_State[14796]);
    d = Tmp_State[14706];
    d1 = Tmp_State[14724];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[14886] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[14886] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[14670] / 2.0, Tmp_State[14670] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[14688], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1450];
      Image_Position_X = X_pred[5 * iindx + 1451];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void f_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 2610];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 2628];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 2790] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[2574] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[2574] / 2.0;
    tmp_target_y[3] = Tmp_State[2574] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[2592];
    tmp_target_x[2] = Tmp_State[2592];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[2700]);
    Image_Position_Y = std::sin(Tmp_State[2700]);
    d = Tmp_State[2610];
    d1 = Tmp_State[2628];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[2790] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[2790] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[2574] / 2.0, Tmp_State[2574] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[2592], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 250];
      Image_Position_X = X_pred[5 * iindx + 251];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void fb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 15210];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 15228];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 15390] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[15174] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[15174] / 2.0;
    tmp_target_y[3] = Tmp_State[15174] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[15192];
    tmp_target_x[2] = Tmp_State[15192];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[15300]);
    Image_Position_Y = std::sin(Tmp_State[15300]);
    d = Tmp_State[15210];
    d1 = Tmp_State[15228];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[15390] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[15390] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[15174] / 2.0, Tmp_State[15174] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[15192], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1500];
      Image_Position_X = X_pred[5 * iindx + 1501];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void g_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 3114];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 3132];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 3294] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[3078] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[3078] / 2.0;
    tmp_target_y[3] = Tmp_State[3078] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[3096];
    tmp_target_x[2] = Tmp_State[3096];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[3204]);
    Image_Position_Y = std::sin(Tmp_State[3204]);
    d = Tmp_State[3114];
    d1 = Tmp_State[3132];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[3294] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[3294] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[3078] / 2.0, Tmp_State[3078] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[3096], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 300];
      Image_Position_X = X_pred[5 * iindx + 301];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void gb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], const unsigned char tmp_image[275598], const
                 double X_pred[1600], const double TJ_X[70], const double TJ_Y
                 [70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 15714];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 15732];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 15894] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[15678] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[15678] / 2.0;
    tmp_target_y[3] = Tmp_State[15678] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[15696];
    tmp_target_x[2] = Tmp_State[15696];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[15804]);
    Image_Position_Y = std::sin(Tmp_State[15804]);
    d = Tmp_State[15714];
    d1 = Tmp_State[15732];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[15894] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[15894] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[15678] / 2.0, Tmp_State[15678] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[15696], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1550];
      Image_Position_X = X_pred[5 * iindx + 1551];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void h_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 3618];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 3636];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 3798] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[3582] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[3582] / 2.0;
    tmp_target_y[3] = Tmp_State[3582] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[3600];
    tmp_target_x[2] = Tmp_State[3600];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[3708]);
    Image_Position_Y = std::sin(Tmp_State[3708]);
    d = Tmp_State[3618];
    d1 = Tmp_State[3636];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[3798] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[3798] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[3582] / 2.0, Tmp_State[3582] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[3600], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 350];
      Image_Position_X = X_pred[5 * iindx + 351];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void i_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 4122];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 4140];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 4302] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[4086] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[4086] / 2.0;
    tmp_target_y[3] = Tmp_State[4086] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[4104];
    tmp_target_x[2] = Tmp_State[4104];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[4212]);
    Image_Position_Y = std::sin(Tmp_State[4212]);
    d = Tmp_State[4122];
    d1 = Tmp_State[4140];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[4302] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[4302] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[4086] / 2.0, Tmp_State[4086] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[4104], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 400];
      Image_Position_X = X_pred[5 * iindx + 401];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void j_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 4626];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 4644];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 4806] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[4590] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[4590] / 2.0;
    tmp_target_y[3] = Tmp_State[4590] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[4608];
    tmp_target_x[2] = Tmp_State[4608];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[4716]);
    Image_Position_Y = std::sin(Tmp_State[4716]);
    d = Tmp_State[4626];
    d1 = Tmp_State[4644];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[4806] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[4806] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[4590] / 2.0, Tmp_State[4590] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[4608], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 450];
      Image_Position_X = X_pred[5 * iindx + 451];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void k_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 5130];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 5148];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 5310] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[5094] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[5094] / 2.0;
    tmp_target_y[3] = Tmp_State[5094] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[5112];
    tmp_target_x[2] = Tmp_State[5112];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[5220]);
    Image_Position_Y = std::sin(Tmp_State[5220]);
    d = Tmp_State[5130];
    d1 = Tmp_State[5148];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[5310] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[5310] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[5094] / 2.0, Tmp_State[5094] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[5112], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 500];
      Image_Position_X = X_pred[5 * iindx + 501];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void l_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 5634];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 5652];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 5814] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[5598] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[5598] / 2.0;
    tmp_target_y[3] = Tmp_State[5598] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[5616];
    tmp_target_x[2] = Tmp_State[5616];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[5724]);
    Image_Position_Y = std::sin(Tmp_State[5724]);
    d = Tmp_State[5634];
    d1 = Tmp_State[5652];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[5814] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[5814] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[5598] / 2.0, Tmp_State[5598] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[5616], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 550];
      Image_Position_X = X_pred[5 * iindx + 551];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void m_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 6138];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 6156];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 6318] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[6102] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[6102] / 2.0;
    tmp_target_y[3] = Tmp_State[6102] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[6120];
    tmp_target_x[2] = Tmp_State[6120];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[6228]);
    Image_Position_Y = std::sin(Tmp_State[6228]);
    d = Tmp_State[6138];
    d1 = Tmp_State[6156];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[6318] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[6318] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[6102] / 2.0, Tmp_State[6102] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[6120], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 600];
      Image_Position_X = X_pred[5 * iindx + 601];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void n_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 6642];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 6660];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 6822] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[6606] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[6606] / 2.0;
    tmp_target_y[3] = Tmp_State[6606] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[6624];
    tmp_target_x[2] = Tmp_State[6624];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[6732]);
    Image_Position_Y = std::sin(Tmp_State[6732]);
    d = Tmp_State[6642];
    d1 = Tmp_State[6660];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[6822] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[6822] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[6606] / 2.0, Tmp_State[6606] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[6624], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 650];
      Image_Position_X = X_pred[5 * iindx + 651];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void o_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 7146];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 7164];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 7326] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[7110] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[7110] / 2.0;
    tmp_target_y[3] = Tmp_State[7110] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[7128];
    tmp_target_x[2] = Tmp_State[7128];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[7236]);
    Image_Position_Y = std::sin(Tmp_State[7236]);
    d = Tmp_State[7146];
    d1 = Tmp_State[7164];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[7326] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[7326] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[7110] / 2.0, Tmp_State[7110] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[7128], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 700];
      Image_Position_X = X_pred[5 * iindx + 701];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void p_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 7650];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 7668];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 7830] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[7614] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[7614] / 2.0;
    tmp_target_y[3] = Tmp_State[7614] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[7632];
    tmp_target_x[2] = Tmp_State[7632];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[7740]);
    Image_Position_Y = std::sin(Tmp_State[7740]);
    d = Tmp_State[7650];
    d1 = Tmp_State[7668];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[7830] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[7830] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[7614] / 2.0, Tmp_State[7614] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[7632], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 750];
      Image_Position_X = X_pred[5 * iindx + 751];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void q_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 8154];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 8172];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 8334] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[8118] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[8118] / 2.0;
    tmp_target_y[3] = Tmp_State[8118] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[8136];
    tmp_target_x[2] = Tmp_State[8136];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[8244]);
    Image_Position_Y = std::sin(Tmp_State[8244]);
    d = Tmp_State[8154];
    d1 = Tmp_State[8172];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[8334] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[8334] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[8118] / 2.0, Tmp_State[8118] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[8136], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 800];
      Image_Position_X = X_pred[5 * iindx + 801];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void r_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 8658];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 8676];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 8838] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[8622] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[8622] / 2.0;
    tmp_target_y[3] = Tmp_State[8622] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[8640];
    tmp_target_x[2] = Tmp_State[8640];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[8748]);
    Image_Position_Y = std::sin(Tmp_State[8748]);
    d = Tmp_State[8658];
    d1 = Tmp_State[8676];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[8838] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[8838] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[8622] / 2.0, Tmp_State[8622] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[8640], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 850];
      Image_Position_X = X_pred[5 * iindx + 851];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void s_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 9162];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 9180];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 9342] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[9126] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[9126] / 2.0;
    tmp_target_y[3] = Tmp_State[9126] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[9144];
    tmp_target_x[2] = Tmp_State[9144];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[9252]);
    Image_Position_Y = std::sin(Tmp_State[9252]);
    d = Tmp_State[9162];
    d1 = Tmp_State[9180];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[9342] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[9342] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[9126] / 2.0, Tmp_State[9126] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[9144], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 900];
      Image_Position_X = X_pred[5 * iindx + 901];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void t_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 9666];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 9684];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 9846] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[9630] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[9630] / 2.0;
    tmp_target_y[3] = Tmp_State[9630] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[9648];
    tmp_target_x[2] = Tmp_State[9648];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[9756]);
    Image_Position_Y = std::sin(Tmp_State[9756]);
    d = Tmp_State[9666];
    d1 = Tmp_State[9684];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[9846] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[9846] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[9630] / 2.0, Tmp_State[9630] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[9648], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 950];
      Image_Position_X = X_pred[5 * iindx + 951];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
              double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
              double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
              double laneInfoR[5], const unsigned char tmp_image[275598], const double
              X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 90];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 108];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 270] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[54] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[54] / 2.0;
    tmp_target_y[3] = Tmp_State[54] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[72];
    tmp_target_x[2] = Tmp_State[72];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[180]);
    Image_Position_Y = std::sin(Tmp_State[180]);
    d = Tmp_State[90];
    d1 = Tmp_State[108];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[270] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[270] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[54] / 2.0, Tmp_State[54] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[72], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx];
      Image_Position_X = X_pred[5 * iindx + 1];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void u_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 10170];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 10188];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 10350] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[10134] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[10134] / 2.0;
    tmp_target_y[3] = Tmp_State[10134] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[10152];
    tmp_target_x[2] = Tmp_State[10152];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[10260]);
    Image_Position_Y = std::sin(Tmp_State[10260]);
    d = Tmp_State[10170];
    d1 = Tmp_State[10188];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[10350] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[10350] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[10134] / 2.0, Tmp_State[10134] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[10152], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1000];
      Image_Position_X = X_pred[5 * iindx + 1001];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void v_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 10674];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 10692];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 10854] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[10638] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[10638] / 2.0;
    tmp_target_y[3] = Tmp_State[10638] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[10656];
    tmp_target_x[2] = Tmp_State[10656];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[10764]);
    Image_Position_Y = std::sin(Tmp_State[10764]);
    d = Tmp_State[10674];
    d1 = Tmp_State[10692];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[10854] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[10854] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[10638] / 2.0, Tmp_State[10638] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[10656], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1050];
      Image_Position_X = X_pred[5 * iindx + 1051];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void w_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 11178];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 11196];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 11358] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[11142] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[11142] / 2.0;
    tmp_target_y[3] = Tmp_State[11142] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[11160];
    tmp_target_x[2] = Tmp_State[11160];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[11268]);
    Image_Position_Y = std::sin(Tmp_State[11268]);
    d = Tmp_State[11178];
    d1 = Tmp_State[11196];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[11358] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[11358] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[11142] / 2.0, Tmp_State[11142] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[11160], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1100];
      Image_Position_X = X_pred[5 * iindx + 1101];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void x_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 11682];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 11700];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 11862] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[11646] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[11646] / 2.0;
    tmp_target_y[3] = Tmp_State[11646] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[11664];
    tmp_target_x[2] = Tmp_State[11664];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[11772]);
    Image_Position_Y = std::sin(Tmp_State[11772]);
    d = Tmp_State[11682];
    d1 = Tmp_State[11700];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[11862] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[11862] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[11646] / 2.0, Tmp_State[11646] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[11664], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1150];
      Image_Position_X = X_pred[5 * iindx + 1151];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// Arguments    : const double Chassis[11]
//                const double Tmp_State[16128]
//                const double RANGE_X_RANGE[251]
//                const double RANGE_Y_RANGE[61]
//                const double RANGE_I_LAT_RANGE[255]
//                const double laneInfoL[5]
//                const double laneInfoR[5]
//                unsigned char image[275598]
//                const double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void y_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], const unsigned char tmp_image[275598], const double
                X_pred[1600], const double TJ_X[70], const double TJ_Y[70], unsigned char image[275598])
{
  std::copy(&tmp_image[0], &tmp_image[275598], &image[0]);
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double c_varargin_1[61];
  double TV_range_x[10];
  double TV_range_y[10];
  double b_tmp_target_x[5];
  double tmp_target_x[5];
  double tmp_target_y[5];
  double Image_Position_X;
  double Image_Position_Y;
  double d;
  double d1;
  double i;
  double range_X_max_tmp;
  double range_X_min_tmp;
  double range_Y_max_tmp;
  double range_Y_min_tmp;
  int jj_data[15311];
  int I_LAT_uint8;
  int b_i;
  int i1;
  int iindx;
  int jj_size;
  int k;
  unsigned char u;
  unsigned char u1;
  unsigned char u2;
  unsigned char u3;
  unsigned char u4;
  unsigned char u5;
  unsigned char u6;
  unsigned char u7;
  bool a__8[15311];
  bool in_tmp[15311];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (I_LAT_uint8 = 0; I_LAT_uint8 < 18; I_LAT_uint8++) {
    d = Tmp_State[I_LAT_uint8 + 12186];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[I_LAT_uint8 + 12204];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[I_LAT_uint8 + 12366] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
        Image_Position_X = std::round(d * -5.0 + 126.0);
        Image_Position_Y = std::round(d1 * -5.0 + 31.0);
        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 251.0) {
          Image_Position_X = 251.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 61.0) {
          Image_Position_Y = 61.0;
        }

        //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        b_i = static_cast<int>(static_cast<float>(Image_Position_X) + 251.0F * (
          static_cast<float>(Image_Position_Y) - 1.0F));
        image[b_i - 1] = u;

        //  R
        image[b_i + 45932] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 91865] = u;

        //  R
        image[b_i + 137798] = u1;

        //  R
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u = 0U;
            u1 = 0U;
          }
        } else {
          u = MAX_uint8_T;
          u1 = MAX_uint8_T;
        }

        image[b_i + 183731] = u;

        //  R
        image[b_i + 229664] = u1;

        //  R
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    //  && track_number == 1
    for (Image_Position_X = 0.0; Image_Position_X < laneInfoL[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoL[0] * rt_powd_snf(d, 3.0) + laneInfoL[1] * (d * d)) +
               laneInfoL[2] * d) + laneInfoL[3]) + i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (Image_Position_X = 0.0; Image_Position_X < laneInfoR[4];
         Image_Position_X++) {
      i = Image_Position_X * 3.5;
      for (jj_size = 0; jj_size < 251; jj_size++) {
        d = RANGE_X_RANGE[jj_size];
        d1 = (((laneInfoR[0] * rt_powd_snf(d, 3.0) + laneInfoR[1] * (d * d)) +
               laneInfoR[2] * d) + laneInfoR[3]) - i;
        tmp_lane_y[jj_size] = d1;
        if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp) && (d1 >=
             range_Y_min_tmp) && (d1 <= range_Y_max_tmp)) {
          for (k = 0; k < 251; k++) {
            b_varargin_1[k] = std::abs(RANGE_X_RANGE[jj_size] - RANGE_X_RANGE[k]);
          }

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y, &iindx);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          image[(iindx + 251 * (I_LAT_uint8 - 1)) + 15310] = MAX_uint8_T;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (iindx = 0; iindx < 251; iindx++) {
        i1 = iindx + 251 * b_i;
        image[i1 + 61244] = image[i1 + 15311];
        image[i1 + 107177] = image[i1 + 15311];
        image[i1 + 153110] = image[i1 + 15311];
        image[i1 + 199043] = image[i1 + 15311];
        image[i1 + 244976] = image[i1 + 15311];
      }
    }

    //                          BEV_Window_out(:,:,20) = BEV_Window_out(:,:,2);
    //                          BEV_Window_out(:,:,23) = BEV_Window_out(:,:,2);
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    tmp_target_y[0] = 0.0;
    tmp_target_y[1] = -Chassis[9];
    tmp_target_y[2] = -Chassis[9];
    tmp_target_y[3] = 0.0;
    tmp_target_y[4] = 0.0;
    Image_Position_Y = -Chassis[10] / 2.0;
    tmp_target_x[0] = Image_Position_Y;
    tmp_target_x[1] = Image_Position_Y;
    tmp_target_x[2] = Chassis[10] / 2.0;
    tmp_target_x[3] = Chassis[10] / 2.0;
    tmp_target_x[4] = Image_Position_Y;
    coder::inpolygon(Rx, Ry, tmp_target_y, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (iindx = 0; iindx < I_LAT_uint8; iindx++) {
        for (i1 = 0; i1 < jj_size; i1++) {
          image[((jj_data[i1] + 251 * (ii_data[iindx] - 1)) + 15311 * b_i) - 1] =
            MAX_uint8_T;
        }
      }
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    Image_Position_Y = -Tmp_State[12150] / 2.0;
    tmp_target_y[0] = Image_Position_Y;
    tmp_target_y[1] = Image_Position_Y;
    tmp_target_y[2] = Tmp_State[12150] / 2.0;
    tmp_target_y[3] = Tmp_State[12150] / 2.0;
    tmp_target_y[4] = Image_Position_Y;
    tmp_target_x[0] = 0.0;
    tmp_target_x[1] = Tmp_State[12168];
    tmp_target_x[2] = Tmp_State[12168];
    tmp_target_x[3] = 0.0;
    tmp_target_x[4] = 0.0;
    Image_Position_X = std::cos(Tmp_State[12276]);
    Image_Position_Y = std::sin(Tmp_State[12276]);
    d = Tmp_State[12186];
    d1 = Tmp_State[12204];
    for (b_i = 0; b_i < 5; b_i++) {
      range_X_max_tmp = tmp_target_x[b_i];
      range_Y_min_tmp = tmp_target_y[b_i];
      b_tmp_target_x[b_i] = (range_X_max_tmp * Image_Position_X -
        range_Y_min_tmp * Image_Position_Y) + d;
      range_X_max_tmp = (range_X_max_tmp * Image_Position_Y + range_Y_min_tmp *
                         Image_Position_X) + d1;
      tmp_target_x[b_i] = range_X_max_tmp;
    }

    coder::inpolygon(Rx, Ry, b_tmp_target_x, tmp_target_x, in_tmp, a__8);
    coder::eml_find(in_tmp, ii_data, &I_LAT_uint8, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[12366] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &iindx);
    if ((jj_size != 0) || (I_LAT_uint8 != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(iindx) - 1.0 < 256.0) {
          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(iindx) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(iindx) - 1.0);
          } else {
            u6 = 0U;
            u7 = 0U;
          }
        } else {
          u2 = MAX_uint8_T;
          u3 = MAX_uint8_T;
          u4 = MAX_uint8_T;
          u5 = MAX_uint8_T;
          u6 = MAX_uint8_T;
          u7 = MAX_uint8_T;
        }
      }

      for (iindx = 0; iindx < jj_size; iindx++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[iindx])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[iindx])) - 1);
        image[b_i - 1] = u2;

        //  R
        image[b_i + 45932] = u3;

        //  R
        image[b_i + 91865] = u4;

        //  R
        image[b_i + 137798] = u5;

        //  R
        image[b_i + 183731] = u6;

        //  R
        image[b_i + 229664] = u7;

        //  R
      }
    }
  }

  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[12366] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if (!coder::isequal(empty_black_image, image)) {
    unsigned char u10;
    unsigned char u11;
    unsigned char u12;
    unsigned char u13;
    unsigned char u14;
    unsigned char u15;
    unsigned char u16;
    unsigned char u17;
    unsigned char u18;
    unsigned char u19;
    unsigned char u8;
    unsigned char u9;
    coder::linspace(-Tmp_State[12150] / 2.0, Tmp_State[12150] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[12168], TV_range_x);
    for (iindx = 0; iindx < 10; iindx++) {
      Image_Position_Y = X_pred[5 * iindx + 1200];
      Image_Position_X = X_pred[5 * iindx + 1201];
      for (b_i = 0; b_i < 10; b_i++) {
        jj_size = b_i + 10 * iindx;
        xp1[jj_size] = Image_Position_X;
        d = TV_range_y[b_i] + Image_Position_Y;
        yp1[jj_size] = d;
        xp2[jj_size] = TV_range_x[9] + Image_Position_X;
        yp2[jj_size] = d;
        d = TV_range_x[b_i] + Image_Position_X;
        xp3[jj_size] = d;
        yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
        xp4[jj_size] = d;
        yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
      }
    }

    if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u = 0U;
        u1 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u2 = 0U;
        u3 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u4 = 0U;
        u5 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u6 = 0U;
        u7 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u8 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u9 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u8 = 0U;
        u9 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u10 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u11 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u10 = 0U;
        u11 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u12 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u13 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u12 = 0U;
        u13 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u14 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u15 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u14 = 0U;
        u15 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u16 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u17 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u16 = 0U;
        u17 = 0U;
      }

      if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
        u18 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
        u19 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) - 1.0);
      } else {
        u18 = 0U;
        u19 = 0U;
      }
    } else {
      u = MAX_uint8_T;
      u1 = MAX_uint8_T;
      u2 = MAX_uint8_T;
      u3 = MAX_uint8_T;
      u4 = MAX_uint8_T;
      u5 = MAX_uint8_T;
      u6 = MAX_uint8_T;
      u7 = MAX_uint8_T;
      u8 = MAX_uint8_T;
      u9 = MAX_uint8_T;
      u10 = MAX_uint8_T;
      u11 = MAX_uint8_T;
      u12 = MAX_uint8_T;
      u13 = MAX_uint8_T;
      u14 = MAX_uint8_T;
      u15 = MAX_uint8_T;
      u16 = MAX_uint8_T;
      u17 = MAX_uint8_T;
      u18 = MAX_uint8_T;
      u19 = MAX_uint8_T;
    }

    for (jj_size = 0; jj_size < 100; jj_size++) {
      d = std::round(xp1[jj_size] * -5.0 + 126.0);
      xp1[jj_size] = d;
      d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
      yp1[jj_size] = d1;
      range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
      xp2[jj_size] = range_X_max_tmp;
      range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
      yp2[jj_size] = range_Y_min_tmp;
      Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
      xp3[jj_size] = Image_Position_Y;
      Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
      yp3[jj_size] = Image_Position_X;
      i = std::round(xp4[jj_size] * -5.0 + 126.0);
      xp4[jj_size] = i;
      range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
      yp4[jj_size] = range_X_min_tmp;
      if (d < 1.0) {
        d = 1.0;
        xp1[jj_size] = 1.0;
      } else if (d > 251.0) {
        d = 251.0;
        xp1[jj_size] = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
        yp1[jj_size] = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
        yp1[jj_size] = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
        xp2[jj_size] = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
        xp2[jj_size] = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
        yp2[jj_size] = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
        yp2[jj_size] = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
        xp3[jj_size] = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
        xp3[jj_size] = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
        yp3[jj_size] = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
        yp3[jj_size] = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
        xp4[jj_size] = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
        xp4[jj_size] = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
        yp4[jj_size] = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
        yp4[jj_size] = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      iindx = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[iindx + 45932] = u1;

      //  R
      i1 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[i1 + 45932] = u2;

      //  R
      I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[iindx + 91865] = u5;

      //  R
      image[i1 + 91865] = u6;

      //  R
      image[I_LAT_uint8 + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[iindx + 137798] = u9;

      //  R
      image[i1 + 137798] = u10;

      //  R
      image[I_LAT_uint8 + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[iindx + 183731] = u13;

      //  R
      image[i1 + 183731] = u14;

      //  R
      image[I_LAT_uint8 + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[iindx + 229664] = u17;

      //  R
      image[i1 + 229664] = u18;

      //  R
      image[I_LAT_uint8 + 229664] = u19;

      //  R
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (iindx = 0; iindx < 10; iindx++) {
        I_LAT_uint8 = iindx + 10 * (k + 2);
        Image_Position_Y = TJ_Y[I_LAT_uint8];
        Image_Position_X = TJ_X[I_LAT_uint8];
        for (b_i = 0; b_i < 10; b_i++) {
          jj_size = b_i + 10 * iindx;
          xp1[jj_size] = TV_range_x[0] + Image_Position_X;
          d = TV_range_y[b_i] + Image_Position_Y;
          yp1[jj_size] = d;
          xp2[jj_size] = TV_range_x[9] + Image_Position_X;
          yp2[jj_size] = d;
          d = TV_range_x[b_i] + Image_Position_X;
          xp3[jj_size] = d;
          yp3[jj_size] = TV_range_y[0] + Image_Position_Y;
          xp4[jj_size] = d;
          yp4[jj_size] = TV_range_y[9] + Image_Position_Y;
        }
      }

      b_i = 3 * (k + 1);
      iindx = 15311 * (b_i + 1);
      i1 = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 100; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        xp1[jj_size] = d;
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        yp1[jj_size] = d1;
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        xp2[jj_size] = range_X_max_tmp;
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        yp2[jj_size] = range_Y_min_tmp;
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        xp3[jj_size] = Image_Position_Y;
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        yp3[jj_size] = Image_Position_X;
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        xp4[jj_size] = i;
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        yp4[jj_size] = range_X_min_tmp;
        if (d < 1.0) {
          d = 1.0;
          xp1[jj_size] = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
          xp1[jj_size] = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
          yp1[jj_size] = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
          yp1[jj_size] = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
          xp2[jj_size] = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
          xp2[jj_size] = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
          yp2[jj_size] = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
          yp2[jj_size] = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
          xp3[jj_size] = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
          xp3[jj_size] = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
          yp3[jj_size] = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
          yp3[jj_size] = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
          xp4[jj_size] = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
          xp4[jj_size] = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
          yp4[jj_size] = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
          yp4[jj_size] = 61.0;
        }

        I_LAT_uint8 = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
        I_LAT_uint8 = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(I_LAT_uint8 + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + iindx) - 1] = MAX_uint8_T;
        image[(I_LAT_uint8 + i1) - 1] = MAX_uint8_T;

        //  R
      }
    }
  }
}

//
// File trailer for tmp_SBEV.cpp
//
// [EOF]
//
