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

#ifndef CHFILTERACCESS_H
#define CHFILTERACCESS_H

#include <functional>
#include <memory>
#include <mutex>
#include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

// a filter that, when applied to a sensor, creates a GUI window to visualize the sensor (using GLFW)
template <class BufferType, class LockedBufferType>
class CH_SENSOR_API ChFilterAccess : public ChFilter {
  public:
    ChFilterAccess(std::string name = {})
        : ChFilter(name.length() > 0 ? name : "CopyToFilter"),
          m_lockUserHasBuffer(m_mutexBufferAccess, std::defer_lock){};
    virtual ~ChFilterAccess() {}

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

    // user calls this to get (locked) access to the buffer.
    // user MUST NOT store the returned pointer.
    // user MUST release (or set to null) when finished, to unlock the buffer or rendering will stall.
    // user MAY move the SensorHostBuffer::Buffer to store (take ownership of) it before releasing the SensorHostBuffer,
    // in which case
    //   a new Buffer will be allocated on next rendering.
    LockedBufferType GetBuffer() {
        // start by locking the mutex. The user will have access to this buffer until
        // s/he releases the returned unique_ptr, which will unlock the mutex.
        m_lockUserHasBuffer.lock();

        // return a unique_ptr that is actually just a pointer to our internal m_buffer.
        // the trick is that the deleter of this unique ptr will be a (lambda) function that
        // just unlocks the mutex (doesn't really delete anything).
        auto ret = LockedBufferType(&m_buffer, [this](BufferType* p) -> void { m_lockUserHasBuffer.unlock(); });
        return ret;
    }

  private:
    std::mutex m_mutexBufferAccess;
    std::unique_lock<std::mutex> m_lockUserHasBuffer;
    BufferType m_buffer;
};

// typedefs for explicit Filters
using ChFilterR8Access = ChFilterAccess<SensorHostR8Buffer, LockedR8BufferPtr>;
using ChFilterRGBA8Access = ChFilterAccess<SensorHostRGBA8Buffer, LockedRGBA8BufferPtr>;
// using ChFilterRGBA8CopyToDevice = ChFilterAccess<SensorDeviceRGBA8Buffer, LockedSensorDeviceRGBA8BufferPtr>;
using ChFilterXYZRGBAccess = ChFilterAccess<SensorHostXYZRGBBuffer, LockedXYZRGBBufferPtr>;
using ChFilterXYZIAccess = ChFilterAccess<SensorHostXYZIBuffer, LockedXYZIBufferPtr>;
using ChFilterDIAccess = ChFilterAccess<SensorHostDIBuffer, LockedDIBufferPtr>;
using ChFilterIMUAccess = ChFilterAccess<SensorHostIMUBuffer, LockedIMUBufferPtr>;
using ChFilterGPSAccess = ChFilterAccess<SensorHostGPSBuffer, LockedGPSBufferPtr>;

}  // namespace sensor
}  // namespace chrono

#endif
