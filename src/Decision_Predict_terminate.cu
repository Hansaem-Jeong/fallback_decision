//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Decision_Predict_terminate.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 03-Nov-2021 17:28:26
//

// Include Files
#include "Decision_Predict_terminate.h"
#include "Decision_Predict_data.h"
#include <cstdio>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void Decision_Predict_terminate()
{
  cudaError_t errCode;
  errCode = cudaGetLastError();
  if (errCode != cudaSuccess) {
    fprintf(stderr, "ERR[%d] %s:%s\n", errCode, cudaGetErrorName(errCode),
            cudaGetErrorString(errCode));
    exit(errCode);
  }
  isInitialized_Decision_Predict = false;
}

//
// File trailer for Decision_Predict_terminate.cu
//
// [EOF]
//
