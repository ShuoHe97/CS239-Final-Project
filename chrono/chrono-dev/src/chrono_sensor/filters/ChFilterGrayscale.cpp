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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/grayscale.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterGrayscale::ChFilterGrayscale(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterGrayscale::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The grayscale filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    std::shared_ptr<SensorOptixBuffer> pSen = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pSen) {
        throw std::runtime_error("The grayscale filter requires that the incoming buffer must be an optix buffer");
    }

    RTsize rwidth;
    RTsize rheight;
    pSen->Buffer->getSize(rwidth, rheight);
    unsigned int width = (unsigned int)rwidth;
    unsigned int height = (unsigned int)rheight;

    // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
    if (pSen->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
        throw std::runtime_error("The only format that can be converted to grayscale is RGBA8");
    }

    // std::unique_ptr<int> p1 = std::make_unique<int>(4);
    // std::unique_ptr<int> p2(std::move(p1));

    if (!m_buffer) {
        m_buffer = std::make_shared<SensorDeviceR8Buffer>();
        DeviceR8BufferPtr b(cudaMallocHelper<char>(width * height), cudaFreeHelper<char>);
        m_buffer->Buffer = std::move(b);
        m_buffer->Width = width;
        m_buffer->Height = height;
    }
    m_buffer->UpdateCount = pSensor->GetNumUpdates();

    // we need id of first device for this context (should only have 1 anyway)
    int device_id = pSen->Buffer->getContext()->getEnabledDevices()[0];
    void* ptr = pSen->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
    cuda_grayscale(ptr, m_buffer->Buffer.get(), (int)width, (int)height);
    // TODO: is this necessary now that we are making grayscale in a separate GPU buffer?
    // pSen->Buffer->markDirty();

    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
