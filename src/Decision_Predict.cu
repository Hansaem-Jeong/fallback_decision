//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Decision_Predict.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 01-Sep-2021 16:48:58
//

// Include Files
#include "Decision_Predict.h"
#include "Decision_Predict_data.h"
#include "Decision_Predict_initialize.h"
#include "Decision_Predict_internal_types.h"
#include "DeepLearningNetwork.h"
#include "predict.h"

// Variable Definitions
static aesnet0_0 aesDecNet;

static bool aesDecNet_not_empty;

// Function Definitions
//
// Arguments    : const unsigned char in[367464]
//                float out[7]
// Return Type  : void
//
void Decision_Predict(const unsigned char in[367464], float out[7])
{
  if (!isInitialized_Decision_Predict) {
    Decision_Predict_initialize();
  }
  if (!aesDecNet_not_empty) {
    coder::DeepLearningNetwork_setup(&aesDecNet);
    aesDecNet_not_empty = true;
  }
  coder::DeepLearningNetwork_predict(&aesDecNet, in, out);
}

//
// Arguments    : void
// Return Type  : void
//
void Decision_Predict_init()
{
  aesDecNet_not_empty = false;
}

//
// File trailer for Decision_Predict.cu
//
// [EOF]
//
