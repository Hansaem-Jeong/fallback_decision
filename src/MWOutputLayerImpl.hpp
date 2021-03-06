/* Copyright 2020 The MathWorks, Inc. */

#ifndef __OUTPUT_LAYER_IMPL_HPP
#define __OUTPUT_LAYER_IMPL_HPP

#include "MWCNNLayerImpl.hpp"

class MWCNNLayer;
class MWTargetNetworkImpl;

class MWOutputLayerImpl : public MWCNNLayerImpl {
public:
    MWOutputLayerImpl(MWCNNLayer*, MWTargetNetworkImpl*);
    ~MWOutputLayerImpl();

    void deallocateOutputData(int);
    void propagateSize();
    void predict();
    void cleanup();
};

#endif
