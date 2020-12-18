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
// Container class for a depth camera sensor
//
// =============================================================================

#ifndef CHDEPTHCAMERASENSOR_H
#define CHDEPTHCAMERASENSOR_H

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {
class CH_SENSOR_API ChDepthCameraSensor : public ChSensor, public IRendersWithOptix {
  public:
    ChDepthCameraSensor(std::shared_ptr<chrono::ChBody> parent,  // body to which depth camera is attached
                        double updateRate,                       // update rate in Hz
                        chrono::ChFrame<double> offsetPose,      // offset of depth camera from Body frame
                        unsigned int w,                          // image width
                        unsigned int h,                          // image height
                        float hFOV,
                        float vFOV,
                        ProgramString program = {"depth_camera", "pinhole_camera"},
                        RTformat buffer_format = RT_FORMAT_FLOAT4);
    ~ChDepthCameraSensor();

    void SetOffsetPose(chrono::ChFrame<double> pose) { m_offsetPose = pose; }
    chrono::ChFrame<double> GetOffsetPose() { return m_offsetPose; }

    void GetImageResolution(unsigned int& w, unsigned int& h);

    // IRendersWithOptix implementation
    virtual optix::Context& Context() { return m_context; }
    virtual optix::Buffer& Buffer() { return m_buffer; }
    virtual optix::Program& RayGenProgram() { return m_ray_gen; }
    virtual int& LaunchIndex() { return m_launch_index; }
    virtual unsigned int& RenderWidth() { return m_width; }
    virtual unsigned int& RenderHeight() { return m_height; }
    virtual float& RenderHFOV() { return m_hFOV; }
    virtual float& RenderVFOV() { return m_vFOV; }
    virtual ProgramString& RenderProgramString() { return m_program_string; }
    virtual RTformat& RenderBufferFormat() { return m_buffer_format; }

  private:
    unsigned int m_width;
    unsigned int m_height;
    float m_vFOV;
    float m_hFOV;

    optix::Context m_context;
    optix::Buffer m_buffer;
    optix::Program m_ray_gen;
    int m_launch_index;

    ProgramString m_program_string;
    RTformat m_buffer_format;
};
}  // namespace sensor
}  // namespace chrono

#endif
