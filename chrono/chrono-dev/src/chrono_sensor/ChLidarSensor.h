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
// Container class for a lidar sensor
//
// =============================================================================

#ifndef CHLIDARSENSOR_H
#define CHLIDARSENSOR_H

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {
class CH_SENSOR_API ChLidarSensor : public ChSensor, public IRendersWithOptix {
  public:
    ChLidarSensor(std::shared_ptr<chrono::ChBody> parent,  // body to which the lidar is attached
                  double updateRate,                       // update rate in Hz for full scan
                  chrono::ChFrame<double> offsetPose,      // offset of lidar from Body frame
                  unsigned int w,                          // image width
                  unsigned int h,                          // image height
                  float hfov,                              // horizontal field of view
                  float vfov,                              // vertical field of view
                  ProgramString program = {"lidar", "spherical"},
                  RTformat buffer_format = RT_FORMAT_FLOAT2);
    ~ChLidarSensor();

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
    float m_hFOV = 2.0f * (float)chrono::CH_C_PI;
    float m_vFOV = (float)chrono::CH_C_PI / 2.0f;

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
