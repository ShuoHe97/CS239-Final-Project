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

#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/cuda/pointcloud.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterPCfromDepth::ChFilterPCfromDepth(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterPCfromDepth::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The Pointcloud filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    std::shared_ptr<SensorOptixBuffer> pSen = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pSen) {
        throw std::runtime_error("The pointcloud filter requires that the incoming buffer must be an optix buffer");
    }

    std::shared_ptr<ChLidarSensor> pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor);
    if (!pLidar) {
        throw std::runtime_error("This sensor must be a lidar.");
    }

    RTsize rwidth;
    RTsize rheight;
    pSen->Buffer->getSize(rwidth, rheight);
    unsigned int width = (unsigned int)rwidth;
    unsigned int height = (unsigned int)rheight;

    // we only know how to convert Lidar Depth+Intensity to Pointcloud (not any other input format (yet))
    if (pSen->Buffer->getFormat() != RT_FORMAT_FLOAT2) {
        throw std::runtime_error("The only format that can be converted to pointcloud is RG32 (Depth, Intensity)");
    }

    // std::unique_ptr<int> p1 = std::make_unique<int>(4);
    // std::unique_ptr<int> p2(std::move(p1));

    if (!m_buffer) {
        m_buffer = std::make_shared<SensorDeviceXYZIBuffer>();
        DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(width * height), cudaFreeHelper<PixelXYZI>);
        m_buffer->Buffer = std::move(b);
        m_buffer->Width = width;
        m_buffer->Height = height;
    }
    m_buffer->UpdateCount = pSensor->GetNumUpdates();

    // we need id of first device for this context (should only have 1 anyway)
    int device_id = pSen->Buffer->getContext()->getEnabledDevices()[0];
    void* ptr = pSen->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
    LidarParams params;
    params.hFOV = pLidar->RenderHFOV();
    params.vFOV = pLidar->RenderVFOV();
    params.horizontal_samples = pLidar->RenderWidth();
    params.vertical_samples = pLidar->RenderHeight();

    cuda_pointcloud_from_depth(ptr, m_buffer->Buffer.get(), (int)width, (int)height, params);
    // TODO: is this necessary now that we are making grayscale in a separate GPU buffer?
    // pSen->Buffer->markDirty();

    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
