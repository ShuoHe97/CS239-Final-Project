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
//
// =============================================================================

#ifndef CHFILTERSAVEXYZRGB_H
#define CHFILTERSAVEXYZRGB_H

#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

// a filter that, when applied to a sensor, creates a GUI window to visualize the sensor (using GLFW)
class CH_SENSOR_API ChFilterSaveXYZRGB : public ChFilter {
  public:
    ChFilterSaveXYZRGB(std::string data_path = "");
    virtual ~ChFilterSaveXYZRGB();

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::string m_path;
};

}  // namespace sensor
}  // namespace chrono

#endif
