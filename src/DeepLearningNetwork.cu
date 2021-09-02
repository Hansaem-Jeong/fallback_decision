//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DeepLearningNetwork.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 01-Sep-2021 16:48:58
//

// Include Files
#include "DeepLearningNetwork.h"
#include "Decision_Predict_internal_types.h"
#include "MWCNNLayer.hpp"
#include "MWElementwiseAffineLayer.hpp"
#include "MWFCLayer.hpp"
#include "MWFusedConvReLULayer.hpp"
#include "MWInputLayer.hpp"
#include "MWMaxPoolingLayer.hpp"
#include "MWOutputLayer.hpp"
#include "MWSoftmaxLayer.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWTensor.hpp"
#include "MWTensorBase.hpp"
#include <cstdio>


// Named Constants
const char *errorString{
    "Abnormal termination due to: %s.\nError in %s (line %d)."};

// Function Declarations
static void checkCleanupCudaError(cudaError_t errCode, const char *file,
                                  unsigned int line);

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::allocate()
{
  this->targetImpl->allocate(367464, 2);
  for (int idx{0}; idx < 10; idx++) {
    this->layers[idx]->allocate();
  }
  (static_cast<MWTensor<float> *>(this->inputTensors[0]))
      ->setData(this->layers[0]->getLayerOutput(0));
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::cleanup()
{
  this->deallocate();
  for (int idx{0}; idx < 10; idx++) {
    this->layers[idx]->cleanup();
  }
  if (this->targetImpl) {
    this->targetImpl->cleanup();
  }
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::deallocate()
{
  this->targetImpl->deallocate();
  for (int idx{0}; idx < 10; idx++) {
    this->layers[idx]->deallocate();
  }
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::postsetup()
{
  this->targetImpl->postSetup(this->layers, this->numLayers);
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::resetState()
{
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::setSize()
{
  for (int idx{0}; idx < 10; idx++) {
    this->layers[idx]->propagateSize();
  }
  this->allocate();
  this->postsetup();
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::setup()
{
  if (this->isInitialized) {
    this->resetState();
  } else {
    this->isInitialized = true;
    this->targetImpl->preSetup();
    this->targetImpl->setAutoTune(true);
    (static_cast<MWInputLayer *>(this->layers[0]))
        ->createInputLayer(this->targetImpl, this->inputTensors[0], 251, 61, 24,
                           0, "", 0);
    (static_cast<MWElementwiseAffineLayer *>(this->layers[1]))
        ->createElementwiseAffineLayer(
            this->targetImpl, this->layers[0]->getOutputTensor(0), 1, 1, 24, 1,
            1, 24, false, 1, 1,
            "./src/fallback_decision/src/cnn_aesnet0_0_imageinput_scale.bin",
            "./src/fallback_decision/src/"
            "cnn_aesnet0_0_imageinput_offset.bin",
            0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[2]))
        ->createFusedConvReLULayer(this->targetImpl, 1,
                                   this->layers[1]->getOutputTensor(0), 3, 3,
                                   24, 8, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                   "./src/fallback_decision/src/"
                                   "cnn_aesnet0_0_Convolution Layer 1_w.bin",
                                   "./src/fallback_decision/src/"
                                   "cnn_aesnet0_0_Convolution Layer 1_b.bin",
                                   1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[3]))
        ->createMaxPoolingLayer<float, float>(
            this->targetImpl, this->layers[2]->getOutputTensor(0), 2, 2, 2, 2,
            0, 0, 0, 0, 0, 0, "FLOAT", 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[4]))
        ->createFusedConvReLULayer(
            this->targetImpl, 1, this->layers[3]->getOutputTensor(0), 3, 3, 8,
            16, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            "./src/fallback_decision/src/cnn_aesnet0_0_conv_1_w.bin",
            "./src/fallback_decision/src/cnn_aesnet0_0_conv_1_b.bin", 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[5]))
        ->createMaxPoolingLayer<float, float>(
            this->targetImpl, this->layers[4]->getOutputTensor(0), 2, 2, 2, 2,
            0, 0, 0, 0, 0, 0, "FLOAT", 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[6]))
        ->createFusedConvReLULayer(
            this->targetImpl, 1, this->layers[5]->getOutputTensor(0), 3, 3, 16,
            32, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            "./src/fallback_decision/src/cnn_aesnet0_0_conv_2_w.bin",
            "./src/fallback_decision/src/cnn_aesnet0_0_conv_2_b.bin", 1);
    (static_cast<MWFCLayer *>(this->layers[7]))
        ->createFCLayer(this->targetImpl, this->layers[6]->getOutputTensor(0),
                        29760, 7,
                        "./src/fallback_decision/src/cnn_aesnet0_0_fully "
                        "Connected Layer_w.bin",
                        "./src/fallback_decision/src/cnn_aesnet0_0_fully "
                        "Connected Layer_b.bin",
                        0);
    (static_cast<MWSoftmaxLayer *>(this->layers[8]))
        ->createSoftmaxLayer(this->targetImpl,
                             this->layers[7]->getOutputTensor(0), 1);
    (static_cast<MWOutputLayer *>(this->layers[9]))
        ->createOutputLayer(this->targetImpl,
                            this->layers[8]->getOutputTensor(0), 1);
    this->outputTensors[0] = this->layers[9]->getOutputTensor(0);
    this->setSize();
  }
}

//
// Arguments    : cudaError_t errCode
//                const char *file
//                unsigned int line
// Return Type  : void
//
static void checkCleanupCudaError(cudaError_t errCode, const char *file,
                                  unsigned int line)
{
  if ((errCode != cudaSuccess) && (errCode != cudaErrorCudartUnloading)) {
    printf(errorString, cudaGetErrorString(errCode), file, line);
  }
}

//
// Arguments    : void
// Return Type  : ::aesnet0_0
//
aesnet0_0::aesnet0_0()
{
  this->numLayers = 10;
  this->isInitialized = false;
  this->targetImpl = 0;
  this->layers[0] = new MWInputLayer;
  this->layers[0]->setName("imageinput");
  this->layers[1] = new MWElementwiseAffineLayer;
  this->layers[1]->setName("imageinput_normalization");
  this->layers[1]->setInPlaceIndex(0, 0);
  this->layers[2] = new MWFusedConvReLULayer;
  this->layers[2]->setName("Convolution Layer 1_relu Layer 1");
  this->layers[3] = new MWMaxPoolingLayer;
  this->layers[3]->setName("max Pooling Layer 1");
  this->layers[4] = new MWFusedConvReLULayer;
  this->layers[4]->setName("conv_1_relu Layer 2");
  this->layers[5] = new MWMaxPoolingLayer;
  this->layers[5]->setName("max Pooling Layer 2");
  this->layers[6] = new MWFusedConvReLULayer;
  this->layers[6]->setName("conv_2_relu Layer 3");
  this->layers[7] = new MWFCLayer;
  this->layers[7]->setName("fully Connected Layer");
  this->layers[8] = new MWSoftmaxLayer;
  this->layers[8]->setName("softmax Layer");
  this->layers[9] = new MWOutputLayer;
  this->layers[9]->setName("classoutput");
  this->layers[9]->setInPlaceIndex(0, 0);
  this->targetImpl = new MWTargetNetworkImpl;
  this->inputTensors[0] = new MWTensor<float>;
  this->inputTensors[0]->setHeight(251);
  this->inputTensors[0]->setWidth(61);
  this->inputTensors[0]->setChannels(24);
  this->inputTensors[0]->setBatchSize(1);
  this->inputTensors[0]->setSequenceLength(1);
}

//
// Arguments    : void
// Return Type  : void
//
aesnet0_0::~aesnet0_0()
{
  this->cleanup();
  checkCleanupCudaError(cudaGetLastError(), __FILE__, __LINE__);
  for (int idx{0}; idx < 10; idx++) {
    delete this->layers[idx];
  }
  if (this->targetImpl) {
    delete this->targetImpl;
  }
  delete this->inputTensors[0];
}

//
// Arguments    : void
// Return Type  : int
//
int aesnet0_0::getBatchSize()
{
  return this->inputTensors[0]->getBatchSize();
}

//
// Arguments    : int b_index
// Return Type  : float *
//
float *aesnet0_0::getInputDataPointer(int b_index)
{
  return (static_cast<MWTensor<float> *>(this->inputTensors[b_index]))
      ->getData();
}

//
// Arguments    : void
// Return Type  : float *
//
float *aesnet0_0::getInputDataPointer()
{
  return (static_cast<MWTensor<float> *>(this->inputTensors[0]))->getData();
}

//
// Arguments    : int layerIndex
//                int portIndex
// Return Type  : float *
//
float *aesnet0_0::getLayerOutput(int layerIndex, int portIndex)
{
  return this->layers[layerIndex]->getLayerOutput(portIndex);
}

//
// Arguments    : int layerIndex
//                int portIndex
// Return Type  : int
//
int aesnet0_0::getLayerOutputSize(int layerIndex, int portIndex)
{
  return this->layers[layerIndex]
             ->getOutputTensor(portIndex)
             ->getNumElements() *
         sizeof(float);
}

//
// Arguments    : int b_index
// Return Type  : float *
//
float *aesnet0_0::getOutputDataPointer(int b_index)
{
  return (static_cast<MWTensor<float> *>(this->outputTensors[b_index]))
      ->getData();
}

//
// Arguments    : void
// Return Type  : float *
//
float *aesnet0_0::getOutputDataPointer()
{
  return (static_cast<MWTensor<float> *>(this->outputTensors[0]))->getData();
}

//
// Arguments    : void
// Return Type  : void
//
void aesnet0_0::predict()
{
  for (int idx{0}; idx < 10; idx++) {
    this->layers[idx]->predict();
  }
}

//
// Arguments    : aesnet0_0 *obj
// Return Type  : void
//
namespace coder {
void DeepLearningNetwork_setup(aesnet0_0 *obj)
{
  obj->setup();
}

} // namespace coder

//
// File trailer for DeepLearningNetwork.cu
//
// [EOF]
//
