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

#ifndef CHFILTER_H
#define CHFILTER_H

#include <memory>
#include <string>
#include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

// base class for all filters that can be applied to a sensor after initial rendering
class CH_SENSOR_API ChFilter {
  public:
    virtual ~ChFilter(){};

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) = 0;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) = 0;

    std::string& Name() { return m_name; }

  protected:
    ChFilter(std::string name) { Name() = name; }

  private:
    std::string m_name;
};

}  // namespace sensor
}  // namespace chrono

#endif
