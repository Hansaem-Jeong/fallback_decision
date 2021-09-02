//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 01-Sep-2021 16:48:58
//

// Include Files
#include "predict.h"
#include "Decision_Predict_internal_types.h"
#include "DeepLearningNetwork.h"
#include "MWCudaDimUtility.hpp"

// Type Definitions
struct cell_wrap_10 {
  float f1[7];
};

struct cell_wrap_6 {
  float f1[367464];
};

struct cell_wrap_9 {
  float f1[367464];
};

// Function Declarations
static __global__ void
DeepLearningNetwork_predict_kernel1(const unsigned char varargin_1[367464],
                                    cell_wrap_6 dataInputsSingle[1]);

static __global__ void
DeepLearningNetwork_predict_kernel2(const cell_wrap_6 dataInputsSingle[1],
                                    cell_wrap_9 inMiniBatchGroup[1]);

static __global__ void
DeepLearningNetwork_predict_kernel3(const cell_wrap_10 outMiniBatchGroup[1],
                                    float varargout_1[7]);

// Function Definitions
//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char varargin_1[367464]
//                cell_wrap_6 dataInputsSingle[1]
// Return Type  : void
//
static __global__
    __launch_bounds__(512, 1) void DeepLearningNetwork_predict_kernel1(
        const unsigned char varargin_1[367464], cell_wrap_6 dataInputsSingle[1])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 367464) {
    dataInputsSingle[0].f1[i] = static_cast<float>(varargin_1[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const cell_wrap_6 dataInputsSingle[1]
//                cell_wrap_9 inMiniBatchGroup[1]
// Return Type  : void
//
static __global__
    __launch_bounds__(512, 1) void DeepLearningNetwork_predict_kernel2(
        const cell_wrap_6 dataInputsSingle[1], cell_wrap_9 inMiniBatchGroup[1])
{
  unsigned long threadId;
  int i;
  int i1;
  int p;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 61UL);
  threadId = (threadId - static_cast<unsigned long>(i)) / 61UL;
  i1 = static_cast<int>(threadId % 251UL);
  threadId = (threadId - static_cast<unsigned long>(i1)) / 251UL;
  p = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(p < 24)) &&
                        (static_cast<int>(i1 < 251)))) &&
      (static_cast<int>(i < 61))) {
    inMiniBatchGroup[0].f1[(i + 61 * i1) + 15311 * p] =
        dataInputsSingle[0].f1[(i1 + 251 * i) + 15311 * p];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const cell_wrap_10 outMiniBatchGroup[1]
//                float varargout_1[7]
// Return Type  : void
//
static __global__
    __launch_bounds__(32, 1) void DeepLearningNetwork_predict_kernel3(
        const cell_wrap_10 outMiniBatchGroup[1], float varargout_1[7])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 7) {
    varargout_1[i] = outMiniBatchGroup[0].f1[i];
  }
}

//
// Arguments    : aesnet0_0 *obj
//                const unsigned char varargin_1[367464]
//                float varargout_1[7]
// Return Type  : void
//
namespace coder {
void DeepLearningNetwork_predict(aesnet0_0 *obj,
                                 const unsigned char varargin_1[367464],
                                 float varargout_1[7])
{
  cell_wrap_10(*gpu_outMiniBatchGroup)[1];
  cell_wrap_6(*gpu_dataInputsSingle)[1];
  cell_wrap_9(*gpu_inMiniBatchGroup)[1];
  float(*gpu_varargout_1)[7];
  unsigned char(*gpu_varargin_1)[367464];
  cudaMalloc(&gpu_varargout_1, 28UL);
  cudaMalloc(&gpu_outMiniBatchGroup, 28UL);
  cudaMalloc(&gpu_inMiniBatchGroup, 1469856UL);
  cudaMalloc(&gpu_dataInputsSingle, 1469856UL);
  cudaMalloc(&gpu_varargin_1, 367464UL);
  cudaMemcpy(*gpu_varargin_1, varargin_1, 367464UL, cudaMemcpyHostToDevice);
  DeepLearningNetwork_predict_kernel1<<<dim3(718U, 1U, 1U),
                                        dim3(512U, 1U, 1U)>>>(
      *gpu_varargin_1, *gpu_dataInputsSingle);
  DeepLearningNetwork_predict_kernel2<<<dim3(718U, 1U, 1U),
                                        dim3(512U, 1U, 1U)>>>(
      *gpu_dataInputsSingle, *gpu_inMiniBatchGroup);
  cudaMemcpy(obj->getInputDataPointer(0), (*gpu_inMiniBatchGroup)[0].f1,
             obj->getLayerOutputSize(0, 0), cudaMemcpyDeviceToDevice);
  obj->predict();
  cudaMemcpy((*gpu_outMiniBatchGroup)[0].f1, obj->getLayerOutput(9, 0),
             obj->getLayerOutputSize(9, 0), cudaMemcpyDeviceToDevice);
  DeepLearningNetwork_predict_kernel3<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(
      *gpu_outMiniBatchGroup, *gpu_varargout_1);
  cudaMemcpy(varargout_1, *gpu_varargout_1, 28UL, cudaMemcpyDeviceToHost);
  cudaFree(*gpu_varargin_1);
  cudaFree(*gpu_dataInputsSingle);
  cudaFree(*gpu_inMiniBatchGroup);
  cudaFree(*gpu_outMiniBatchGroup);
  cudaFree(*gpu_varargout_1);
}

} // namespace coder

//
// File trailer for predict.cu
//
// [EOF]
//
