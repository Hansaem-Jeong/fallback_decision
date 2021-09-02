//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Decision_Predict_initialize.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 01-Sep-2021 16:48:58
//

// Include Files
#include "Decision_Predict_initialize.h"
#include "Decision_Predict.h"
#include "Decision_Predict_data.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void Decision_Predict_initialize()
{
  Decision_Predict_init();
  cudaGetLastError();
  isInitialized_Decision_Predict = true;
}

//
// File trailer for Decision_Predict_initialize.cu
//
// [EOF]
//
