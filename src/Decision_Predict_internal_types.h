//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Decision_Predict_internal_types.h
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 01-Sep-2021 16:48:58
//

#ifndef DECISION_PREDICT_INTERNAL_TYPES_H
#define DECISION_PREDICT_INTERNAL_TYPES_H

// Include Files
#include "Decision_Predict_types.h"
#include "rtwtypes.h"
#include "MWCNNLayer.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWTensorBase.hpp"

// Type Definitions
class aesnet0_0 {
public:
  aesnet0_0();
  void setSize();
  void resetState();
  void setup();
  void predict();
  void cleanup();
  float *getLayerOutput(int layerIndex, int portIndex);
  int getLayerOutputSize(int layerIndex, int portIndex);
  float *getInputDataPointer(int b_index);
  float *getInputDataPointer();
  float *getOutputDataPointer(int b_index);
  float *getOutputDataPointer();
  int getBatchSize();
  ~aesnet0_0();

private:
  void allocate();
  void postsetup();
  void deallocate();
  int numLayers;
  bool isInitialized;
  MWTensorBase *inputTensors[1];
  MWTensorBase *outputTensors[1];
  MWCNNLayer *layers[10];
  MWTargetNetworkImpl *targetImpl;
};

#endif
//
// File trailer for Decision_Predict_internal_types.h
//
// [EOF]
//
