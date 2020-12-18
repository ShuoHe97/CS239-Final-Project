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
// Authors: Asher Elmquist
// =============================================================================
//
// Base class for all sensor
//
// =============================================================================

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/utils/ChSensorUtils.h"

#include <iostream>

namespace chrono {
namespace sensor {

// global constant for use in template parameters
const char ChFilterR8AccessName[] = "ChFilterR8Access";          // single channel 8 bit image
const char ChFilterRGBA8AccessName[] = "ChFilterRGBA8Access";    // 4 channel 8 bit image
const char ChFilterDIAccessName[] = "ChFilterDIAccess";          // 2 channel float, ->Depth+Intenisty
const char ChFilterXYZIAccessName[] = "ChFilterXYZIAccess";      // 4 channel float, XYZ positions+Intensity
const char ChFilterXYZRGBAccessName[] = "ChFilterXYZRGBAccess";  // 6 channel float, XYZ positions+color
const char ChFilterIMUAccessName[] = "ChFilterIMUAccess";        // IMU data format (6 floats total)
const char ChFilterGPSAccessName[] = "ChFilterGPSAccess";        // GPS data format (3 doubles total)

CH_SENSOR_API ChSensor::ChSensor(std::shared_ptr<chrono::ChBody> parent,
                                 double updateRate,
                                 chrono::ChFrame<double> offsetPose) {
    m_parent = parent;
    m_updateRate = (float)updateRate;
    m_offsetPose = offsetPose;
}

CH_SENSOR_API ChSensor::~ChSensor() {}

template <>
CH_SENSOR_API LockedR8BufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedR8BufferPtr, ChFilterR8Access, ChFilterR8AccessName>();
}

template <>
CH_SENSOR_API LockedRGBA8BufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedRGBA8BufferPtr, ChFilterRGBA8Access, ChFilterRGBA8AccessName>();
}

template <>
CH_SENSOR_API LockedDIBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedDIBufferPtr, ChFilterDIAccess, ChFilterDIAccessName>();
}

template <>
CH_SENSOR_API LockedXYZIBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedXYZIBufferPtr, ChFilterXYZIAccess, ChFilterXYZIAccessName>();
}

template <>
CH_SENSOR_API LockedXYZRGBBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedXYZRGBBufferPtr, ChFilterXYZRGBAccess, ChFilterXYZRGBAccessName>();
}

template <>
CH_SENSOR_API LockedIMUBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedIMUBufferPtr, ChFilterIMUAccess, ChFilterIMUAccessName>();
}

template <>
CH_SENSOR_API LockedGPSBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<LockedGPSBufferPtr, ChFilterGPSAccess, ChFilterGPSAccessName>();
}

template <class LockedBufferType, class FilterType, const char* FilterName>
LockedBufferType ChSensor::GetMostRecentBufferHelper() {
    // find the last filter in the filter list that is a 'FilterType'
    // (note using reverse iterator)
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<FilterType>(p) != nullptr;
    });

    if (it == m_filters.rend()) {
        std::stringstream s;
        s << "Cannot return device buffer: Filter List does not contain an entry of type " << FilterName;
        throw std::runtime_error(s.str());
    }

    return (std::dynamic_pointer_cast<FilterType>(*it))->GetBuffer();
}

}  // namespace sensor
}  // namespace chrono
