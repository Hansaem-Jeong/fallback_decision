//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: tmp_SBEV.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 13-Dec-2021 11:53:25
//

// Include Files
#include "tmp_SBEV.h"
#include "find.h"
#include "ifWhileCond.h"
#include "inpolygon.h"
#include "isequal.h"
#include "linspace.h"
#include "meshgrid.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void ab_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 12690];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 12708];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 12870] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[12870] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[12870] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1256] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1251]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1251] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1250]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1250] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[12654] / 2.0, Tmp_State[12654] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[12672], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1296];
        d1 = X_pred[1295];
        range_X_max_tmp = X_pred[1295];
        range_Y_min_tmp = X_pred[1296];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1251];
        d1 = X_pred[5 * c_i + 1250];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void b_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 594];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 612];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 774] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[774] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[774] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[56] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 51]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 51] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 50]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 50] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[558] / 2.0, Tmp_State[558] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[576], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[96];
        d1 = X_pred[95];
        range_X_max_tmp = X_pred[95];
        range_Y_min_tmp = X_pred[96];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 51];
        d1 = X_pred[5 * c_i + 50];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void bb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 13194];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 13212];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 13374] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[13374] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[13374] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1306] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1301]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1301] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1300]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1300] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[13158] / 2.0, Tmp_State[13158] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[13176], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1346];
        d1 = X_pred[1345];
        range_X_max_tmp = X_pred[1345];
        range_Y_min_tmp = X_pred[1346];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1301];
        d1 = X_pred[5 * c_i + 1300];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void c_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 1098];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 1116];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 1278] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[1278] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[1278] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[106] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 101]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 101] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 100]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 100] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[1062] / 2.0, Tmp_State[1062] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[1080], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[146];
        d1 = X_pred[145];
        range_X_max_tmp = X_pred[145];
        range_Y_min_tmp = X_pred[146];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 101];
        d1 = X_pred[5 * c_i + 100];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void cb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 13698];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 13716];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 13878] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[13878] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[13878] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1356] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1351]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1351] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1350]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1350] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[13662] / 2.0, Tmp_State[13662] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[13680], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1396];
        d1 = X_pred[1395];
        range_X_max_tmp = X_pred[1395];
        range_Y_min_tmp = X_pred[1396];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1351];
        d1 = X_pred[5 * c_i + 1350];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void d_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 1602];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 1620];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 1782] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[1782] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[1782] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[156] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 151]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 151] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 150]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 150] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[1566] / 2.0, Tmp_State[1566] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[1584], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[196];
        d1 = X_pred[195];
        range_X_max_tmp = X_pred[195];
        range_Y_min_tmp = X_pred[196];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 151];
        d1 = X_pred[5 * c_i + 150];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void db_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 14202];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 14220];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 14382] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[14382] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[14382] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1406] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1401]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1401] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1400]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1400] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[14166] / 2.0, Tmp_State[14166] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[14184], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1446];
        d1 = X_pred[1445];
        range_X_max_tmp = X_pred[1445];
        range_Y_min_tmp = X_pred[1446];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1401];
        d1 = X_pred[5 * c_i + 1400];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void e_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 2106];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 2124];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 2286] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[2286] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[2286] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[206] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 201]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 201] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 200]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 200] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[2070] / 2.0, Tmp_State[2070] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[2088], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[246];
        d1 = X_pred[245];
        range_X_max_tmp = X_pred[245];
        range_Y_min_tmp = X_pred[246];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 201];
        d1 = X_pred[5 * c_i + 200];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void eb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 14706];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 14724];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 14886] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[14886] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[14886] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1456] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1451]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1451] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1450]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1450] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[14670] / 2.0, Tmp_State[14670] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[14688], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1496];
        d1 = X_pred[1495];
        range_X_max_tmp = X_pred[1495];
        range_Y_min_tmp = X_pred[1496];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1451];
        d1 = X_pred[5 * c_i + 1450];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void f_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 2610];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 2628];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 2790] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[2790] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[2790] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[256] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 251]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 251] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 250]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 250] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[2574] / 2.0, Tmp_State[2574] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[2592], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[296];
        d1 = X_pred[295];
        range_X_max_tmp = X_pred[295];
        range_Y_min_tmp = X_pred[296];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 251];
        d1 = X_pred[5 * c_i + 250];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void fb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 15210];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 15228];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 15390] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[15390] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[15390] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1506] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1501]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1501] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1500]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1500] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[15174] / 2.0, Tmp_State[15174] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[15192], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1546];
        d1 = X_pred[1545];
        range_X_max_tmp = X_pred[1545];
        range_Y_min_tmp = X_pred[1546];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1501];
        d1 = X_pred[5 * c_i + 1500];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void g_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 3114];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 3132];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 3294] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[3294] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[3294] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[306] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 301]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 301] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 300]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 300] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[3078] / 2.0, Tmp_State[3078] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[3096], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[346];
        d1 = X_pred[345];
        range_X_max_tmp = X_pred[345];
        range_Y_min_tmp = X_pred[346];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 301];
        d1 = X_pred[5 * c_i + 300];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void gb_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                 double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61],
                 const double RANGE_I_LAT_RANGE[255], const double laneInfoL[5],
                 const double laneInfoR[5], unsigned char image[275598], double
                 X_pred[1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 15714];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 15732];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 15894] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[15894] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[15894] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1556] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1551]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1551] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1550]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1550] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[15678] / 2.0, Tmp_State[15678] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[15696], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1596];
        d1 = X_pred[1595];
        range_X_max_tmp = X_pred[1595];
        range_Y_min_tmp = X_pred[1596];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1551];
        d1 = X_pred[5 * c_i + 1550];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void h_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 3618];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 3636];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 3798] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[3798] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[3798] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[356] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 351]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 351] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 350]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 350] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[3582] / 2.0, Tmp_State[3582] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[3600], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[396];
        d1 = X_pred[395];
        range_X_max_tmp = X_pred[395];
        range_Y_min_tmp = X_pred[396];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 351];
        d1 = X_pred[5 * c_i + 350];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void i_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 4122];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 4140];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 4302] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[4302] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[4302] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[406] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 401]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 401] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 400]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 400] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[4086] / 2.0, Tmp_State[4086] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[4104], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[446];
        d1 = X_pred[445];
        range_X_max_tmp = X_pred[445];
        range_Y_min_tmp = X_pred[446];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 401];
        d1 = X_pred[5 * c_i + 400];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void j_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 4626];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 4644];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 4806] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[4806] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[4806] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[456] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 451]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 451] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 450]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 450] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[4590] / 2.0, Tmp_State[4590] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[4608], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[496];
        d1 = X_pred[495];
        range_X_max_tmp = X_pred[495];
        range_Y_min_tmp = X_pred[496];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 451];
        d1 = X_pred[5 * c_i + 450];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void k_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 5130];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 5148];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 5310] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[5310] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[5310] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[506] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 501]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 501] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 500]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 500] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[5094] / 2.0, Tmp_State[5094] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[5112], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[546];
        d1 = X_pred[545];
        range_X_max_tmp = X_pred[545];
        range_Y_min_tmp = X_pred[546];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 501];
        d1 = X_pred[5 * c_i + 500];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void l_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 5634];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 5652];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 5814] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[5814] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[5814] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[556] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 551]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 551] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 550]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 550] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[5598] / 2.0, Tmp_State[5598] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[5616], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[596];
        d1 = X_pred[595];
        range_X_max_tmp = X_pred[595];
        range_Y_min_tmp = X_pred[596];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 551];
        d1 = X_pred[5 * c_i + 550];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void m_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 6138];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 6156];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 6318] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[6318] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[6318] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[606] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 601]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 601] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 600]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 600] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[6102] / 2.0, Tmp_State[6102] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[6120], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[646];
        d1 = X_pred[645];
        range_X_max_tmp = X_pred[645];
        range_Y_min_tmp = X_pred[646];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 601];
        d1 = X_pred[5 * c_i + 600];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void n_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 6642];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 6660];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 6822] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[6822] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[6822] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[656] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 651]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 651] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 650]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 650] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[6606] / 2.0, Tmp_State[6606] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[6624], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[696];
        d1 = X_pred[695];
        range_X_max_tmp = X_pred[695];
        range_Y_min_tmp = X_pred[696];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 651];
        d1 = X_pred[5 * c_i + 650];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void o_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 7146];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 7164];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 7326] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[7326] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[7326] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[706] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 701]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 701] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 700]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 700] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[7110] / 2.0, Tmp_State[7110] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[7128], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[746];
        d1 = X_pred[745];
        range_X_max_tmp = X_pred[745];
        range_Y_min_tmp = X_pred[746];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 701];
        d1 = X_pred[5 * c_i + 700];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void p_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 7650];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 7668];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 7830] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[7830] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[7830] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[756] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 751]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 751] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 750]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 750] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[7614] / 2.0, Tmp_State[7614] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[7632], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[796];
        d1 = X_pred[795];
        range_X_max_tmp = X_pred[795];
        range_Y_min_tmp = X_pred[796];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 751];
        d1 = X_pred[5 * c_i + 750];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void q_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 8154];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 8172];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 8334] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[8334] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[8334] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[806] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 801]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 801] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 800]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 800] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[8118] / 2.0, Tmp_State[8118] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[8136], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[846];
        d1 = X_pred[845];
        range_X_max_tmp = X_pred[845];
        range_Y_min_tmp = X_pred[846];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 801];
        d1 = X_pred[5 * c_i + 800];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void r_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 8658];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 8676];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 8838] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[8838] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[8838] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[856] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 851]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 851] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 850]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 850] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[8622] / 2.0, Tmp_State[8622] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[8640], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[896];
        d1 = X_pred[895];
        range_X_max_tmp = X_pred[895];
        range_Y_min_tmp = X_pred[896];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 851];
        d1 = X_pred[5 * c_i + 850];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void s_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 9162];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 9180];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 9342] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[9342] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[9342] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[906] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 901]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 901] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 900]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 900] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[9126] / 2.0, Tmp_State[9126] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[9144], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[946];
        d1 = X_pred[945];
        range_X_max_tmp = X_pred[945];
        range_Y_min_tmp = X_pred[946];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 901];
        d1 = X_pred[5 * c_i + 900];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void t_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 9666];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 9684];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 9846] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[9846] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[9846] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[956] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 951]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 951] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 950]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 950] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[9630] / 2.0, Tmp_State[9630] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[9648], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[996];
        d1 = X_pred[995];
        range_X_max_tmp = X_pred[995];
        range_Y_min_tmp = X_pred[996];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 951];
        d1 = X_pred[5 * c_i + 950];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
              double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
              double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
              double laneInfoR[5], unsigned char image[275598], double X_pred
              [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 90];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 108];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 270] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[270] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[270] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[6] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1)] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[54] / 2.0, Tmp_State[54] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[72], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[46];
        d1 = X_pred[45];
        range_X_max_tmp = X_pred[45];
        range_Y_min_tmp = X_pred[46];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1];
        d1 = X_pred[5 * c_i];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void u_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 10170];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 10188];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 10350] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[10350] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[10350] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1006] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1001]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1001] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1000]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1000] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[10134] / 2.0, Tmp_State[10134] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[10152], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1046];
        d1 = X_pred[1045];
        range_X_max_tmp = X_pred[1045];
        range_Y_min_tmp = X_pred[1046];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1001];
        d1 = X_pred[5 * c_i + 1000];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void v_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 10674];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 10692];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 10854] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[10854] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[10854] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1056] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1051]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1051] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1050]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1050] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[10638] / 2.0, Tmp_State[10638] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[10656], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1096];
        d1 = X_pred[1095];
        range_X_max_tmp = X_pred[1095];
        range_Y_min_tmp = X_pred[1096];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1051];
        d1 = X_pred[5 * c_i + 1050];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void w_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 11178];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 11196];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 11358] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[11358] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[11358] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1106] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1101]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1101] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1100]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1100] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[11142] / 2.0, Tmp_State[11142] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[11160], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1146];
        d1 = X_pred[1145];
        range_X_max_tmp = X_pred[1145];
        range_Y_min_tmp = X_pred[1146];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1101];
        d1 = X_pred[5 * c_i + 1100];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void x_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 11682];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 11700];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 11862] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[11862] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[11862] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1156] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1151]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1151] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1150]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1150] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[11646] / 2.0, Tmp_State[11646] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[11664], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1196];
        d1 = X_pred[1195];
        range_X_max_tmp = X_pred[1195];
        range_Y_min_tmp = X_pred[1196];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1151];
        d1 = X_pred[5 * c_i + 1150];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
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
//                double X_pred[1600]
//                const double TJ_X[70]
//                const double TJ_Y[70]
// Return Type  : void
//
void y_tmp_SBEV(const double Chassis[11], const double Tmp_State[16128], const
                double RANGE_X_RANGE[251], const double RANGE_Y_RANGE[61], const
                double RANGE_I_LAT_RANGE[255], const double laneInfoL[5], const
                double laneInfoR[5], unsigned char image[275598], double X_pred
                [1600], const double TJ_X[70], const double TJ_Y[70])
{
  static double Rx[15311];
  static double Ry[15311];
  static int ii_data[15311];
  static int jj_data[15311];
  static unsigned char empty_black_image[275598];
  double varargin_1[255];
  double b_varargin_1[251];
  double tmp_lane_y[251];
  double xp1[200];
  double xp2[200];
  double xp3[200];
  double xp4[200];
  double yp1[200];
  double yp2[200];
  double yp3[200];
  double yp4[200];
  double c_varargin_1[61];
  double TV_range_x[20];
  double TV_range_y[20];
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
  int I_LAT_uint8;
  int b_i;
  int c_i;
  int image_index;
  int jj_size;
  int k;
  signed char b_tmp_data[10];
  signed char tmp_data[10];
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
  bool bv[10];

  //  Chassis
  range_X_min_tmp = coder::internal::minimum(RANGE_X_RANGE);
  range_X_max_tmp = coder::internal::maximum(RANGE_X_RANGE);
  range_Y_min_tmp = coder::internal::b_minimum(RANGE_Y_RANGE);
  range_Y_max_tmp = coder::internal::b_maximum(RANGE_Y_RANGE);
  coder::meshgrid(RANGE_X_RANGE, RANGE_Y_RANGE, Rx, Ry);
  for (image_index = 0; image_index < 18; image_index++) {
    d = Tmp_State[image_index + 12186];
    if ((d >= range_X_min_tmp) && (d <= range_X_max_tmp)) {
      d1 = Tmp_State[image_index + 12204];
      if ((d1 >= range_Y_min_tmp) && (d1 <= range_Y_max_tmp) && (d != 0.0) &&
          (d1 != 0.0)) {
        for (k = 0; k < 255; k++) {
          varargin_1[k] = std::abs(Tmp_State[image_index + 12366] -
            RANGE_I_LAT_RANGE[k]);
        }

        coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u1 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
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

          coder::internal::b_minimum(b_varargin_1, &Image_Position_Y,
            &I_LAT_uint8);
          for (k = 0; k < 61; k++) {
            c_varargin_1[k] = std::abs(tmp_lane_y[jj_size] - RANGE_Y_RANGE[k]);
          }

          coder::internal::c_minimum(c_varargin_1, &Image_Position_Y,
            &image_index);
          b_i = I_LAT_uint8 + 251 * (image_index - 1);
          image[b_i - 1] = 0U;
          image[b_i + 15310] = MAX_uint8_T;
          image[b_i + 30621] = 0U;
        }
      }
    }

    for (b_i = 0; b_i < 61; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < 251; I_LAT_uint8++) {
        c_i = I_LAT_uint8 + 251 * b_i;
        image[c_i + 61244] = image[c_i + 15311];
        image[c_i + 107177] = image[c_i + 15311];
        image[c_i + 153110] = image[c_i + 15311];
        image[c_i + 199043] = image[c_i + 15311];
        image[c_i + 244976] = image[c_i + 15311];
        image[c_i] = 0U;
        image[c_i + 30622] = 0U;
        image[c_i + 45933] = 0U;
        image[c_i + 76555] = 0U;
        image[c_i + 91866] = 0U;
        image[c_i + 122488] = 0U;
        image[c_i + 137799] = 0U;
        image[c_i + 168421] = 0U;
        image[c_i + 183732] = 0U;
        image[c_i + 214354] = 0U;
        image[c_i + 229665] = 0U;
        image[c_i + 260287] = 0U;
      }
    }
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (b_i = 0; b_i < 18; b_i++) {
      for (I_LAT_uint8 = 0; I_LAT_uint8 < image_index; I_LAT_uint8++) {
        for (c_i = 0; c_i < jj_size; c_i++) {
          image[((jj_data[c_i] + 251 * (ii_data[I_LAT_uint8] - 1)) + 15311 * b_i)
            - 1] = MAX_uint8_T;
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
    coder::eml_find(in_tmp, ii_data, &image_index, jj_data, &jj_size);
    for (k = 0; k < 255; k++) {
      varargin_1[k] = std::abs(Tmp_State[12366] - RANGE_I_LAT_RANGE[k]);
    }

    coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
    if ((jj_size != 0) || (image_index != 0)) {
      if (0 <= jj_size - 1) {
        if (static_cast<double>(I_LAT_uint8) - 1.0 < 256.0) {
          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u2 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u3 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u2 = 0U;
            u3 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u4 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u5 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
          } else {
            u4 = 0U;
            u5 = 0U;
          }

          if (static_cast<double>(I_LAT_uint8) - 1.0 >= 0.0) {
            u6 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
            u7 = static_cast<unsigned char>(static_cast<double>(I_LAT_uint8) -
              1.0);
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

      for (c_i = 0; c_i < jj_size; c_i++) {
        b_i = static_cast<int>(static_cast<float>(jj_data[c_i])) + 251 * (
          static_cast<int>(static_cast<float>(ii_data[c_i])) - 1);
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
        image[b_i + 15310] = 0U;

        //  G
        image[b_i + 61243] = 0U;

        //  G
        image[b_i + 107176] = 0U;

        //  G
        image[b_i + 153109] = 0U;

        //  G
        image[b_i + 199042] = 0U;

        //  G
        image[b_i + 244975] = 0U;

        //  G
        image[b_i + 30621] = 0U;

        //  B
        image[b_i + 76554] = 0U;

        //  B
        image[b_i + 122487] = 0U;

        //  B
        image[b_i + 168420] = 0U;

        //  B
        image[b_i + 214353] = 0U;

        //  B
        image[b_i + 260286] = 0U;

        //  B
      }
    }
  }

  //  Number of box dash
  //  Number of box dash
  for (k = 0; k < 255; k++) {
    varargin_1[k] = std::abs(Tmp_State[12366] - RANGE_I_LAT_RANGE[k]);
  }

  coder::internal::minimum(varargin_1, &Image_Position_Y, &I_LAT_uint8);
  if (std::abs(static_cast<double>(I_LAT_uint8)) > 255.0) {
    I_LAT_uint8 = 1;
  }

  if ((!coder::isequal(empty_black_image, image)) && (X_pred[1206] != 0.0)) {
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
    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1201]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (tmp_data[b_i] - 1) + 1201] = 0.0;
      }
    }

    for (b_i = 0; b_i < 10; b_i++) {
      bv[b_i] = std::isnan(X_pred[5 * b_i + 1200]);
    }

    if (coder::internal::ifWhileCond(bv)) {
      jj_size = 0;
      image_index = 0;
      for (c_i = 0; c_i < 10; c_i++) {
        if (bv[c_i]) {
          jj_size++;
          b_tmp_data[image_index] = static_cast<signed char>(c_i + 1);
          image_index++;
        }
      }

      for (b_i = 0; b_i < jj_size; b_i++) {
        X_pred[5 * (b_tmp_data[b_i] - 1) + 1200] = 0.0;
      }
    }

    coder::linspace(-Tmp_State[12150] / 2.0, Tmp_State[12150] / 2.0, TV_range_y);
    coder::linspace(0.0, Tmp_State[12168], TV_range_x);
    for (c_i = 0; c_i < 10; c_i++) {
      if (c_i + 1 > 9) {
        d = X_pred[1246];
        d1 = X_pred[1245];
        range_X_max_tmp = X_pred[1245];
        range_Y_min_tmp = X_pred[1246];
        for (b_i = 0; b_i < 20; b_i++) {
          xp1[b_i + 180] = TV_range_x[0] + d;
          Image_Position_Y = TV_range_y[b_i] + range_X_max_tmp;
          yp1[b_i + 180] = Image_Position_Y;
          xp2[b_i + 180] = d + TV_range_x[19];
          yp2[b_i + 180] = Image_Position_Y;
          Image_Position_Y = TV_range_x[b_i] + range_Y_min_tmp;
          xp3[b_i + 180] = Image_Position_Y;
          yp3[b_i + 180] = TV_range_y[0] + d1;
          xp4[b_i + 180] = Image_Position_Y;
          yp4[b_i + 180] = d1 + TV_range_y[19];
        }
      } else {
        d = X_pred[5 * c_i + 1201];
        d1 = X_pred[5 * c_i + 1200];
        for (b_i = 0; b_i < 20; b_i++) {
          image_index = b_i + 20 * c_i;
          xp1[image_index] = d;
          yp1[image_index] = d1;
          xp2[image_index] = d;
          yp2[image_index] = d1;
          xp3[image_index] = d;
          yp3[image_index] = d1;
          xp4[image_index] = d;
          yp4[image_index] = d1;
        }
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

    for (jj_size = 0; jj_size < 200; jj_size++) {
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
      } else if (d > 251.0) {
        d = 251.0;
      }

      if (d1 < 1.0) {
        d1 = 1.0;
      } else if (d1 > 61.0) {
        d1 = 61.0;
      }

      if (range_X_max_tmp < 1.0) {
        range_X_max_tmp = 1.0;
      } else if (range_X_max_tmp > 251.0) {
        range_X_max_tmp = 251.0;
      }

      if (range_Y_min_tmp < 1.0) {
        range_Y_min_tmp = 1.0;
      } else if (range_Y_min_tmp > 61.0) {
        range_Y_min_tmp = 61.0;
      }

      if (Image_Position_Y < 1.0) {
        Image_Position_Y = 1.0;
      } else if (Image_Position_Y > 251.0) {
        Image_Position_Y = 251.0;
      }

      if (Image_Position_X < 1.0) {
        Image_Position_X = 1.0;
      } else if (Image_Position_X > 61.0) {
        Image_Position_X = 61.0;
      }

      if (i < 1.0) {
        i = 1.0;
      } else if (i > 251.0) {
        i = 251.0;
      }

      if (range_X_min_tmp < 1.0) {
        range_X_min_tmp = 1.0;
      } else if (range_X_min_tmp > 61.0) {
        range_X_min_tmp = 61.0;
      }

      b_i = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
      image[b_i + 45932] = u;

      //  R
      I_LAT_uint8 = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
        (range_Y_min_tmp) - 1);
      image[I_LAT_uint8 + 45932] = u1;

      //  R
      c_i = static_cast<int>(Image_Position_Y) + 251 * (static_cast<int>
        (Image_Position_X) - 1);
      image[c_i + 45932] = u2;

      //  R
      image_index = static_cast<int>(i) + 251 * (static_cast<int>
        (range_X_min_tmp) - 1);
      image[image_index + 45932] = u3;

      //  R
      image[b_i + 91865] = u4;

      //  R
      image[I_LAT_uint8 + 91865] = u5;

      //  R
      image[c_i + 91865] = u6;

      //  R
      image[image_index + 91865] = u7;

      //  R
      image[b_i + 137798] = u8;

      //  R
      image[I_LAT_uint8 + 137798] = u9;

      //  R
      image[c_i + 137798] = u10;

      //  R
      image[image_index + 137798] = u11;

      //  R
      image[b_i + 183731] = u12;

      //  R
      image[I_LAT_uint8 + 183731] = u13;

      //  R
      image[c_i + 183731] = u14;

      //  R
      image[image_index + 183731] = u15;

      //  R
      image[b_i + 229664] = u16;

      //  R
      image[I_LAT_uint8 + 229664] = u17;

      //  R
      image[c_i + 229664] = u18;

      //  R
      image[image_index + 229664] = u19;

      //  R
      image[b_i + 61243] = 0U;

      //  G
      image[I_LAT_uint8 + 61243] = 0U;

      //  G
      image[c_i + 61243] = 0U;

      //  G
      image[image_index + 61243] = 0U;

      //  G
      image[b_i + 107176] = 0U;

      //  G
      image[I_LAT_uint8 + 107176] = 0U;

      //  G
      image[c_i + 107176] = 0U;

      //  G
      image[image_index + 107176] = 0U;

      //  G
      image[b_i + 153109] = 0U;

      //  G
      image[I_LAT_uint8 + 153109] = 0U;

      //  G
      image[c_i + 153109] = 0U;

      //  G
      image[image_index + 153109] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 199042] = 0U;

      //  G
      image[c_i + 199042] = 0U;

      //  G
      image[image_index + 199042] = 0U;

      //  G
      image[b_i + 199042] = 0U;

      //  G
      image[I_LAT_uint8 + 244975] = 0U;

      //  G
      image[c_i + 244975] = 0U;

      //  G
      image[image_index + 244975] = 0U;

      //  G
      image[b_i + 244975] = 0U;

      //  G
      image[b_i + 76554] = 0U;

      //  B
      image[I_LAT_uint8 + 76554] = 0U;

      //  B
      image[c_i + 76554] = 0U;

      //  B
      image[image_index + 76554] = 0U;

      //  B
      image[b_i + 122487] = 0U;

      //  B
      image[I_LAT_uint8 + 122487] = 0U;

      //  B
      image[c_i + 122487] = 0U;

      //  B
      image[image_index + 122487] = 0U;

      //  B
      image[b_i + 168420] = 0U;

      //  B
      image[I_LAT_uint8 + 168420] = 0U;

      //  B
      image[c_i + 168420] = 0U;

      //  B
      image[image_index + 168420] = 0U;

      //  B
      image[b_i + 214353] = 0U;

      //  B
      image[I_LAT_uint8 + 214353] = 0U;

      //  B
      image[c_i + 214353] = 0U;

      //  B
      image[image_index + 214353] = 0U;

      //  B
      image[b_i + 260286] = 0U;

      //  B
      image[I_LAT_uint8 + 260286] = 0U;

      //  B
      image[c_i + 260286] = 0U;

      //  B
      image[image_index + 260286] = 0U;

      //  B
    }
  }

  if (!coder::isequal(empty_black_image, image)) {
    // && track_number == Traffic_Number
    coder::linspace(-Chassis[10] / 2.0, Chassis[10] / 2.0, TV_range_y);
    coder::linspace(-Chassis[9], 0.0, TV_range_x);
    for (k = 0; k < 5; k++) {
      //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
      for (c_i = 0; c_i < 10; c_i++) {
        if (c_i + 1 > 9) {
          image_index = 10 * (k + 2) + 9;
          Image_Position_Y = TJ_Y[image_index];
          Image_Position_X = TJ_X[image_index];
          for (b_i = 0; b_i < 20; b_i++) {
            xp1[b_i + 180] = TV_range_x[0] + Image_Position_X;
            d = TV_range_y[b_i] + Image_Position_Y;
            yp1[b_i + 180] = d;
            xp2[b_i + 180] = TV_range_x[19] + Image_Position_X;
            yp2[b_i + 180] = d;
            d = TV_range_x[b_i] + Image_Position_X;
            xp3[b_i + 180] = d;
            yp3[b_i + 180] = TV_range_y[0] + Image_Position_Y;
            xp4[b_i + 180] = d;
            yp4[b_i + 180] = TV_range_y[19] + Image_Position_Y;
          }
        } else {
          b_i = c_i + 10 * (k + 2);
          d = TJ_X[b_i];
          d1 = TJ_Y[b_i];
          for (b_i = 0; b_i < 20; b_i++) {
            image_index = b_i + 20 * c_i;
            xp1[image_index] = d;
            yp1[image_index] = d1;
            xp2[image_index] = d;
            yp2[image_index] = d1;
            xp3[image_index] = d;
            yp3[image_index] = d1;
            xp4[image_index] = d;
            yp4[image_index] = d1;
          }
        }
      }

      b_i = 3 * (k + 1);
      I_LAT_uint8 = 15311 * (b_i + 1);
      c_i = 15311 * (b_i + 2);
      for (jj_size = 0; jj_size < 200; jj_size++) {
        d = std::round(xp1[jj_size] * -5.0 + 126.0);
        d1 = std::round(yp1[jj_size] * -5.0 + 31.0);
        range_X_max_tmp = std::round(xp2[jj_size] * -5.0 + 126.0);
        range_Y_min_tmp = std::round(yp2[jj_size] * -5.0 + 31.0);
        Image_Position_Y = std::round(xp3[jj_size] * -5.0 + 126.0);
        Image_Position_X = std::round(yp3[jj_size] * -5.0 + 31.0);
        i = std::round(xp4[jj_size] * -5.0 + 126.0);
        range_X_min_tmp = std::round(yp4[jj_size] * -5.0 + 31.0);
        if (d < 1.0) {
          d = 1.0;
        } else if (d > 251.0) {
          d = 251.0;
        }

        if (d1 < 1.0) {
          d1 = 1.0;
        } else if (d1 > 61.0) {
          d1 = 61.0;
        }

        if (range_X_max_tmp < 1.0) {
          range_X_max_tmp = 1.0;
        } else if (range_X_max_tmp > 251.0) {
          range_X_max_tmp = 251.0;
        }

        if (range_Y_min_tmp < 1.0) {
          range_Y_min_tmp = 1.0;
        } else if (range_Y_min_tmp > 61.0) {
          range_Y_min_tmp = 61.0;
        }

        if (Image_Position_Y < 1.0) {
          Image_Position_Y = 1.0;
        } else if (Image_Position_Y > 251.0) {
          Image_Position_Y = 251.0;
        }

        if (Image_Position_X < 1.0) {
          Image_Position_X = 1.0;
        } else if (Image_Position_X > 61.0) {
          Image_Position_X = 61.0;
        }

        if (i < 1.0) {
          i = 1.0;
        } else if (i > 251.0) {
          i = 251.0;
        }

        if (range_X_min_tmp < 1.0) {
          range_X_min_tmp = 1.0;
        } else if (range_X_min_tmp > 61.0) {
          range_X_min_tmp = 61.0;
        }

        image_index = static_cast<int>(d) + 251 * (static_cast<int>(d1) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(range_X_max_tmp) + 251 * (static_cast<int>
          (range_Y_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(Image_Position_Y) + 251 * (static_cast<
          int>(Image_Position_X) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
        image_index = static_cast<int>(i) + 251 * (static_cast<int>
          (range_X_min_tmp) - 1);
        image[(image_index + 15311 * b_i) - 1] = MAX_uint8_T;
        image[(image_index + I_LAT_uint8) - 1] = MAX_uint8_T;
        image[(image_index + c_i) - 1] = MAX_uint8_T;

        //  W
      }
    }
  }
}

//
// File trailer for tmp_SBEV.cpp
//
// [EOF]
//
