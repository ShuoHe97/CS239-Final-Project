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

#ifndef CHSENSOR_H
#define CHSENSOR_H

#include "chrono_sensor/ChApiSensor.h"

#include <optix.h>
#include <optixu/optixpp.h>  //needed to make sure things are in the right namespace. Must be done before optixpp_namespace.h
#include <optixu/optixpp_namespace.h>  //is covered by optixpp.h but will be removed from optixpp.h in the future
#include <list>

#include "ChSensorBuffer.h"
#include "chrono/physics/ChBody.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"

namespace chrono {
namespace sensor {

// an interface that any classes derived from ChSensor should implement (inherit from)
// if they render themselves with Optix. (example: ChSensorCamera)
class IRendersWithOptix {
  public:
    virtual optix::Context& Context() = 0;
    virtual optix::Program& RayGenProgram() = 0;
    virtual optix::Buffer& Buffer() = 0;
    virtual int& LaunchIndex() = 0;

    // we have to know how to create the buffer (Todo: some figuring out to be done here)
    virtual unsigned int& RenderWidth() = 0;
    virtual unsigned int& RenderHeight() = 0;

    // all renders must have field of view (Todo: what about GPS ray cast tests?)
    virtual float& RenderHFOV() = 0;
    virtual float& RenderVFOV() = 0;

    virtual ProgramString& RenderProgramString() = 0;
    virtual RTformat& RenderBufferFormat() = 0;
};

// TODO: perhaps sensor specific data formats/functions should be in the corresponding sensor?

class CH_SENSOR_API ChSensor {
  public:
    ChSensor(std::shared_ptr<chrono::ChBody> parent, double updateRate, chrono::ChFrame<double> offsetPose);
    virtual ~ChSensor();

    // SensorType GetType() { return m_sensorType; }
    ChFrame<double> GetOffsetPose() { return m_offsetPose; }
    std::shared_ptr<ChBody> GetParent() { return m_parent; }

    void SetName(std::string name) { m_name = name; }
    std::string GetName() { return m_name; }

    void SetSaveEnabled(bool save) { m_save = save; }
    bool GetSaveEnabled() { return m_save; }

    void SetSavePath(std::string save_path) { m_save_path = save_path; }
    std::string GetSavePath() { return m_save_path; }

    float GetUpdateRate() { return m_updateRate; }
    void SetUpdateRate(float updateRate) { m_updateRate = updateRate; }

    unsigned int GetNumUpdates() { return m_num_updates; }
    void IncrementNumUpdates() { m_num_updates++; }

    std::list<std::shared_ptr<ChFilter>>& FilterList() { return m_filters; }

    template <class LockedBufferType>
    LockedBufferType GetMostRecentBuffer();  // explicit specializations exist for each buffer type avaiable

  protected:
    float m_updateRate = 0;
    float m_timeLastUpdated = 0;
    std::shared_ptr<chrono::ChBody> m_parent;
    chrono::ChFrame<double> m_offsetPose;
    std::string m_name = "";
    bool m_save = false;
    std::string m_save_path = "";
    unsigned int m_num_updates = 0;

  private:
    std::list<std::shared_ptr<ChFilter>> m_filters;

    template <class LockedBufferType, class FilterType, const char* FilterName>
    LockedBufferType GetMostRecentBufferHelper();  // explicit specializations exist for each buffer type avaiable

};  // class ChSensor

}  // namespace sensor
}  // namespace chrono

#endif
