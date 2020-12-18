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

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostR8Buffer, LockedR8BufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorDeviceR8Buffer> pDev = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (!pDev) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host R8 buffer.");
    }

    std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

    m_buffer.Width = pDev->Width;
    m_buffer.Height = pDev->Height;
    m_buffer.UpdateCount = pDev->UpdateCount;
    // if we've never allocated the buffer, or if the user 'took' the buffer last time
    // s/he retrieved it, we need to allocate it.
    unsigned int sz = m_buffer.Width * m_buffer.Height;
    if (!m_buffer.Buffer) {
        m_buffer.Buffer = std::make_unique<char[]>(sz);
    }
    cudaMemcpy(m_buffer.Buffer.get(), pDev->Buffer.get(), sz, cudaMemcpyDeviceToHost);

    // bufferInOut = m_buffer;
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostRGBA8Buffer, LockedRGBA8BufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pOpx) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host RGBA8 buffer.");
    }

    RTsize width;
    RTsize height;
    pOpx->Buffer->getSize(width, height);

    void* img = pOpx->Buffer->map();

    {  // lock mutex inside this scope to update the buffer
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

        m_buffer.Width = (unsigned int)width;
        m_buffer.Height = (unsigned int)height;
        m_buffer.UpdateCount = pSensor->GetNumUpdates();
        // if we've never allocated the buffer, or if the user 'took' the buffer last time
        // s/he retrieved it, we need to allocate it.
        unsigned int sz = m_buffer.Width * m_buffer.Height;
        if (!m_buffer.Buffer) {
            m_buffer.Buffer = std::make_unique<PixelRGBA8[]>(sz);
        }
        memcpy(m_buffer.Buffer.get(), img, sz * sizeof(PixelRGBA8));
    }  // unlock mutex so user can get the buffer

    pOpx->Buffer->unmap();

    // bufferInOut = m_buffer;
}

// template <>
// CH_SENSOR_API void ChFilterAccess<SensorDeviceRGBA8Buffer, LockedSensorDeviceRGBA8BufferPtr>::Apply(
//     std::shared_ptr<ChSensor> pSensor,
//     std::shared_ptr<SensorBuffer>& bufferInOut) {
//     // to copy to a host buffer, we need to know what buffer to copy.
//     // for now, that means this filter can only work with sensor that use an Optix buffer.
//     std::shared_ptr<SensorOptixBuffer> pVis = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
//     if (!pVis) {
//         throw std::runtime_error("cannot copy supplied buffer type to a Device RGBA8 buffer.");
//     }
//
//     RTsize width;
//     RTsize height;
//     pVis->Buffer->getSize(width, height);
//
//     void* img = pVis->Buffer->map();
//
//     {  // lock mutex inside this scope to update the buffer
//         std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
//
//         m_buffer.Width = (unsigned int)width;
//         m_buffer.Height = (unsigned int)height;
//         m_buffer.UpdateCount = pSensor->GetNumUpdates();
//
//         // if we've never allocated the buffer, or if the user 'took' the buffer last time
//         // s/he retrieved it, we need to allocate it.
//         unsigned int sz = m_buffer.Width * m_buffer.Height;
//         if (!m_buffer.Buffer) {
//             m_buffer.Buffer = DeviceRGBA8BufferPtr(cudaMallocHelper<PixelRGBA8>(sz), cudaFreeHelper<PixelRGBA8>);
//         }
//         cudaMemcpy(m_buffer.Buffer.get(), img, sz * sizeof(PixelRGBA8), cudaMemcpyDeviceToDevice);
//     }  // unlock mutex so user can get the buffer
//
//     pVis->Buffer->unmap();
//
//     // bufferInOut = m_buffer;
// }

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostXYZRGBBuffer, LockedXYZRGBBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pOpx) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host RGBA8 buffer.");
    }

    RTsize width;
    RTsize height;
    pOpx->Buffer->getSize(width, height);

    void* img = pOpx->Buffer->map();

    {  // lock mutex inside this scope to update the buffer
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

        m_buffer.Width = (unsigned int)width;
        m_buffer.Height = (unsigned int)height;
        m_buffer.UpdateCount = pSensor->GetNumUpdates();

        // if we've never allocated the buffer, or if the user 'took' the buffer last time
        // s/he retrieved it, we need to allocate it.
        unsigned int sz = m_buffer.Width * m_buffer.Height;
        if (!m_buffer.Buffer) {
            m_buffer.Buffer = std::make_unique<PixelXYZRGB[]>(sz);
        }
        memcpy(m_buffer.Buffer.get(), img, sz * sizeof(PixelXYZRGB));
        // std::cout << "Copied Data Back to ZED on Host" << std::endl;
    }  // unlock mutex so user can get the buffer

    pOpx->Buffer->unmap();

    // bufferInOut = m_buffer;
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostXYZIBuffer, LockedXYZIBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorDeviceXYZIBuffer> pDev = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!pDev) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host XYZI buffer.");
    }

    std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

    m_buffer.Width = pDev->Width;
    m_buffer.Height = pDev->Height;
    m_buffer.UpdateCount = pDev->UpdateCount;
    // if we've never allocated the buffer, or if the user 'took' the buffer last time
    // s/he retrieved it, we need to allocate it.
    unsigned int sz = m_buffer.Width * m_buffer.Height * sizeof(PixelXYZI);
    if (!m_buffer.Buffer) {
        m_buffer.Buffer = std::make_unique<PixelXYZI[]>(sz);
    }
    cudaMemcpy(m_buffer.Buffer.get(), pDev->Buffer.get(), sz, cudaMemcpyDeviceToHost);
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostDIBuffer, LockedDIBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pOpx) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host RGBA8 buffer.");
    }

    RTsize width;
    RTsize height;
    pOpx->Buffer->getSize(width, height);

    void* img = pOpx->Buffer->map();

    {  // lock mutex inside this scope to update the buffer
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

        m_buffer.Width = (unsigned int)width;
        m_buffer.Height = (unsigned int)height;
        m_buffer.UpdateCount = pSensor->GetNumUpdates();

        // if we've never allocated the buffer, or if the user 'took' the buffer last time
        // s/he retrieved it, we need to allocate it.
        unsigned int sz = m_buffer.Width * m_buffer.Height;
        if (!m_buffer.Buffer) {
            m_buffer.Buffer = std::make_unique<PixelDI[]>(sz);
        }
        memcpy(m_buffer.Buffer.get(), img, sz * sizeof(PixelDI));
        // std::cout << "Copied Data Back to ZED on Host" << std::endl;
    }  // unlock mutex so user can get the buffer

    pOpx->Buffer->unmap();
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostIMUBuffer, LockedIMUBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorHostIMUBuffer> pIMU = std::dynamic_pointer_cast<SensorHostIMUBuffer>(bufferInOut);
    if (!pIMU) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host IMU buffer.");
    }

    // lock mutex inside this scope to update the buffer
    std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

    m_buffer.Width = 1;
    m_buffer.Height = 1;
    m_buffer.UpdateCount = pSensor->GetNumUpdates();
    // if we've never allocated the buffer, or if the user 'took' the buffer last time
    // s/he retrieved it, we need to allocate it.
    if (!m_buffer.Buffer) {
        m_buffer.Buffer = std::make_unique<IMUData[]>(1);
    }
    memcpy(m_buffer.Buffer.get(), pIMU->Buffer.get(), sizeof(IMUData));

    // bufferInOut = m_buffer;
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostGPSBuffer, LockedGPSBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorHostGPSBuffer> pGPS = std::dynamic_pointer_cast<SensorHostGPSBuffer>(bufferInOut);
    if (!pGPS) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host GPS buffer.");
    }

    // lock mutex inside this scope to update the buffer
    std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

    m_buffer.Width = 1;
    m_buffer.Height = 1;
    m_buffer.UpdateCount = pSensor->GetNumUpdates();
    // if we've never allocated the buffer, or if the user 'took' the buffer last time
    // s/he retrieved it, we need to allocate it.
    if (!m_buffer.Buffer) {
        m_buffer.Buffer = std::make_unique<GPSData[]>(1);
    }

    memcpy(m_buffer.Buffer.get(), pGPS->Buffer.get(), sizeof(GPSData));

    // bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
