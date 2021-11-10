//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BEV_image.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

#ifndef BEV_IMAGE_H
#define BEV_IMAGE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void BEV_image(const double Chassis[11], const double Traffic[288],
                      const double Lane[10], double AEB_in,
                      unsigned char b_BEV_image[275598],
                      unsigned char image_magick[275598]);

void out_Prob_ctrv_not_empty_init();

#endif
//
// File trailer for BEV_image.h
//
// [EOF]
//
