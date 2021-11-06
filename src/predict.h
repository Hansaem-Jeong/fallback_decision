//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.h
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 03-Nov-2021 17:28:26
//

#ifndef PREDICT_H
#define PREDICT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

class aesnet0_0;

// Function Declarations
namespace coder {
void DeepLearningNetwork_predict(aesnet0_0 *obj,
                                 const unsigned char varargin_1[275598],
                                 float varargout_1[7]);

}

#endif
//
// File trailer for predict.h
//
// [EOF]
//
