/* Copyright 2020 The MathWorks, Inc. */

#ifndef __SOFTMAX_LAYER_IMPL_HPP
#define __SOFTMAX_LAYER_IMPL_HPP

#include "MWCNNLayerImpl.hpp"

class MWCNNLayer;
class MWTargetNetworkImpl;

//SoftmaxLayer
class MWSoftmaxLayerImpl: public MWCNNLayerImpl
{
public:
    MWSoftmaxLayerImpl(MWCNNLayer* , MWTargetNetworkImpl*);
    ~MWSoftmaxLayerImpl();

    void predict();
    void cleanup();
    void propagateSize();

private:
    cudnnLRNDescriptor_t          dJcdBfQQLhIAYHPxwQeg;
    cudnnTensorDescriptor_t       shEncNmxJsMuJKwbrwok;
    cudnnTensorDescriptor_t       sjLjZacPSDNBEjAccrGU;
};

#endif
