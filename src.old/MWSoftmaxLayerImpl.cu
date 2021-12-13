#include "MWSoftmaxLayerImpl.hpp"
#include "MWCNNLayerImpl.hpp"
#include "MWCNNLayer.hpp"
#include "MWTensorBase.hpp"
#include "MWTensor.hpp"
#include "MWTargetNetworkImpl.hpp"
 MWSoftmaxLayerImpl::MWSoftmaxLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl) : MWCNNLayerImpl(layer, ntwk_impl)  { 
CUDNN_CALL(cudnnCreateTensorDescriptor(&shEncNmxJsMuJKwbrwok)); 
CUDNN_CALL(cudnnCreateTensorDescriptor(&sjLjZacPSDNBEjAccrGU)); } 
MWSoftmaxLayerImpl::~MWSoftmaxLayerImpl() { } void 
MWSoftmaxLayerImpl::propagateSize() { MWCNNLayer* sfmxLayer = getLayer(); 
MWTensorBase* ipTensor = sfmxLayer->getInputTensor(0); MWTensorBase* opTensor = 
sfmxLayer->getOutputTensor(0); 
CUDNN_CALL(cudnnSetTensor4dDescriptor(shEncNmxJsMuJKwbrwok, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, ipTensor->getSequenceLength()*ipTensor->getBatchSize(), 
ipTensor->getChannels(), ipTensor->getHeight(), ipTensor->getWidth())); 
CUDNN_CALL(cudnnSetTensor4dDescriptor(sjLjZacPSDNBEjAccrGU, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, opTensor->getSequenceLength()*opTensor->getBatchSize(), 
opTensor->getChannels(), opTensor->getHeight(), opTensor->getWidth())); } void 
MWSoftmaxLayerImpl::predict() { MWCNNLayer* sfmxLayer = getLayer(); 
MWTensorBase* ipTensorBase = sfmxLayer->getInputTensor(0); MWTensorBase* 
opTensorBase = sfmxLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); 
CUDNN_CALL(cudnnSoftmaxForward(*dMxIKDGTITyhdLqIHBLA->getCudnnHandle(), 
CUDNN_SOFTMAX_ACCURATE, CUDNN_SOFTMAX_MODE_CHANNEL, getOnePtr(), 
shEncNmxJsMuJKwbrwok, ipTensor->getData(), getZeroPtr(), 
sjLjZacPSDNBEjAccrGU, opTensor->getData())); } void 
MWSoftmaxLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyTensorDescriptor(shEncNmxJsMuJKwbrwok)); 
CUDNN_CALL(cudnnDestroyTensorDescriptor(sjLjZacPSDNBEjAccrGU)); }