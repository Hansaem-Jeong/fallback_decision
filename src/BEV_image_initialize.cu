//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BEV_image_initialize.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 31-Aug-2021 16:30:47
//

// Include Files
#include "BEV_image_initialize.h"
#include "BEV_image.h"
#include "BEV_image_data.h"
#include "rt_nonfinite.h"
#include "MWCUSOLVERUtils.hpp"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void BEV_image_initialize()
{
  just_one_check_not_empty_init();
  out_Prob_ctrv_not_empty_init();
  BEV_image_init();
  cusolverEnsureInitialization();
  cudaGetLastError();
  isInitialized_BEV_image = true;
}

//
// File trailer for BEV_image_initialize.cu
//
// [EOF]
//
