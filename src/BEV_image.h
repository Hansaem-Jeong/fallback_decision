//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BEV_image.h
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 31-Aug-2021 16:30:47
//

#ifndef BEV_IMAGE_H
#define BEV_IMAGE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void BEV_image(const double Chassis[12], const double Sensor[11],
                      const double Traffic[10], const double Lane[11],
                      const double AEB_in[2],
                      unsigned char b_BEV_image[367464]);

void BEV_image_init();

void just_one_check_not_empty_init();

void out_Prob_ctrv_not_empty_init();

#endif
//
// File trailer for BEV_image.h
//
// [EOF]
//
