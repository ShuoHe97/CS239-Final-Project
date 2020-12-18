// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt
// =============================================================================
//
//
// =============================================================================

#ifndef CUDAMALLOCHELPER_H
#define CUDAMALLOCHELPER_H

namespace chrono {
namespace sensor {

template <class T>
inline T* cudaMallocHelper(unsigned int size) {
    void* ret;
    cudaError_t err = cudaMalloc(&ret, size * sizeof(T));
    if (err != cudaSuccess) {
        std::stringstream s;
        s << "cudaMalloc failed with error " << err;
        throw std::runtime_error(s.str());
    }
    return (T*)ret;
}

template <class T>
inline void cudaFreeHelper(T* ptr) {
    if (ptr)
        cudaFree((void*)ptr);
}

}  // namespace sensor
}  // namespace chrono

#endif
