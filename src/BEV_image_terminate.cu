//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BEV_image_terminate.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 31-Aug-2021 16:30:47
//

// Include Files
#include "BEV_image_terminate.h"
#include "BEV_image_data.h"
#include "rt_nonfinite.h"
#include "MWCUSOLVERUtils.hpp"
#include <cstdio>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void BEV_image_terminate()
{
  cudaError_t errCode;
  errCode = cudaGetLastError();
  if (errCode != cudaSuccess) {
    fprintf(stderr, "ERR[%d] %s:%s\n", errCode, cudaGetErrorName(errCode),
            cudaGetErrorString(errCode));
    exit(errCode);
  }
  cusolverDestroyWorkspace();
  cusolverEnsureDestruction();
  isInitialized_BEV_image = false;
}

//
// File trailer for BEV_image_terminate.cu
//
// [EOF]
//
