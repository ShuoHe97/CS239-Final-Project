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
// Container class for a camera sensor
//
// =============================================================================

#include "chrono_sensor/ChDepthCameraSensor.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChDepthCameraSensor::ChDepthCameraSensor(std::shared_ptr<chrono::ChBody> parent,
                                                       double updateRate,
                                                       chrono::ChFrame<double> offsetPose,
                                                       unsigned int w,  // image width
                                                       unsigned int h,  // image height
                                                       float hFOV,
                                                       float vFOV,
                                                       ProgramString program,
                                                       RTformat buffer_format)
    : ChSensor(parent, updateRate, offsetPose) {
    m_width = w;
    m_height = h;
    m_hFOV = hFOV;
    m_vFOV = vFOV;
    m_buffer_format = buffer_format;
    m_program_string = program;

    // Camera sensor get rendered by Optix, so they must has as their first filter an optix renderer.
    FilterList().push_front(std::make_shared<ChFilterOptixRender>());
}
CH_SENSOR_API ChDepthCameraSensor::~ChDepthCameraSensor() {}

CH_SENSOR_API void ChDepthCameraSensor::GetImageResolution(unsigned int& w, unsigned int& h) {
    w = m_width;
    h = m_height;
}

}  // namespace sensor
}  // namespace chrono